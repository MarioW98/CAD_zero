#!/usr/bin/env bash
# scripts/start_codespaces.sh — Launch CAD_0 desktop app inside GitHub Codespaces.
#
# What this does, in order:
#   1. Verifies all required tools are installed (Xvfb, x11vnc, websockify,
#      fluxbox, the CAD_0 build, PySide6, PyOpenGL).
#   2. Starts a virtual X server on display :99 at 1280x800x24.
#   3. Starts fluxbox (lightweight window manager) on that display.
#   4. Starts the CAD_0 desktop app on that display.
#   5. Starts x11vnc attached to display :99, listening on port 5900.
#   6. Starts websockify on port 6080 — wraps the VNC stream in WebSocket
#      and serves the noVNC web client at http://localhost:6080/vnc.html
#
# In GitHub Codespaces, port 6080 is auto-forwarded and you get a public
# HTTPS URL like https://<codespace>-6080.app.github.dev/vnc.html —
# just click the "Ports" tab in VS Code/Codespace to find it.
#
# Usage:
#   cd CAD_0
#   ./scripts/start_codespaces.sh
#
# Stop everything:
#   ./scripts/start_codespaces.sh --stop

set -euo pipefail

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
PROJECT_DIR="/home/z/my-project/CAD_0"
LIBS_DIR="/home/z/my-project/.libs"
VNC_BIN_DIR="/home/z/my-project/vnc_extract/usr/bin"
NOVNC_DIR="/home/z/my-project/vnc_extract/usr/share/novnc"

DISPLAY_NUM=99
SCREEN_W=1280
SCREEN_H=800
SCREEN_D=24
VNC_PORT=5900            # internal VNC port (x11vnc listens here)
WEB_PORT=6080            # public-facing noVNC web port (forwarded by Codespaces)

DISPLAY=":${DISPLAY_NUM}"

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
log()  { printf '\033[1;36m[launch]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; }
die()  { err "$*"; exit 1; }

# On Codespaces we have sudo without a password. In this sandbox we don't —
# so we use the extracted .deb contents under $VNC_BIN_DIR as a fallback.
have() { command -v "$1" >/dev/null 2>&1; }

# -----------------------------------------------------------------------------
# Stop mode
# -----------------------------------------------------------------------------
if [[ "${1:-}" == "--stop" ]]; then
    log "stopping all CAD_0 / VNC processes..."
    pkill -f "python3 .*CAD_0.ui"     2>/dev/null || true
    pkill -f "x11vnc.*:99"            2>/dev/null || true
    pkill -f "websockify.*${WEB_PORT}" 2>/dev/null || true
    pkill -f "fluxbox"                2>/dev/null || true
    pkill -f "Xvfb ${DISPLAY}"        2>/dev/null || true
    rm -f /tmp/cad0_xvfb.log /tmp/cad0_vnc.log /tmp/cad0_websock.log \
          /tmp/cad0_app.log /tmp/cad0_xvfb.pid /tmp/cad0_vnc.pid \
          /tmp/cad0_websock.pid /tmp/cad0_app.pid /tmp/cad0_fluxbox.pid
    log "stopped."
    exit 0
fi

# -----------------------------------------------------------------------------
# Sanity checks
# -----------------------------------------------------------------------------
log "checking environment..."

# Xvfb (always available — system package)
have Xvfb || die "Xvfb not found. Install with: sudo apt-get install -y xvfb"

# x11vnc, fluxbox — try system first, then the extracted .deb fallback
if ! have x11vnc; then
    # Try to install system-wide (works on Codespaces where sudo is passwordless)
    if have sudo && sudo -n true 2>/dev/null; then
        log "installing x11vnc + fluxbox + libxcb-cursor0 via apt (sudo)..."
        sudo apt-get install -y x11vnc fluxbox libxcb-cursor0 libvncserver1 2>&1 | tail -3
    fi
fi
if ! have x11vnc; then
    if [[ -x "${VNC_BIN_DIR}/x11vnc" ]]; then
        export PATH="${VNC_BIN_DIR}:${PATH}"
        log "using extracted x11vnc from ${VNC_BIN_DIR}"
    else
        die "x11vnc not found. Try: sudo apt-get install -y x11vnc"
    fi
fi

if ! have fluxbox; then
    if [[ -x "${VNC_BIN_DIR}/fluxbox" ]]; then
        export PATH="${VNC_BIN_DIR}:${PATH}"
        log "using extracted fluxbox from ${VNC_BIN_DIR}"
    else
        die "fluxbox not found. Try: sudo apt-get install -y fluxbox"
    fi
fi

# websockify — Python package
if ! have websockify; then
    log "installing websockify via pip..."
    python3 -m pip install --quiet websockify 2>&1 | tail -3
fi
have websockify || die "websockify not found. Install with: pip install websockify"

# noVNC web client — try system location first, then the extracted fallback
if [[ ! -f "/usr/share/novnc/vnc.html" ]]; then
    if have sudo && sudo -n true 2>/dev/null; then
        log "installing novnc via apt (sudo)..."
        sudo apt-get install -y novnc 2>&1 | tail -3
    fi
fi
NOVNC_DIR="/usr/share/novnc"
if [[ ! -f "${NOVNC_DIR}/vnc.html" ]]; then
    NOVNC_DIR="/home/z/my-project/vnc_extract/usr/share/novnc"
fi
[[ -f "${NOVNC_DIR}/vnc.html" ]] || die "noVNC web client not found"

# EGL/GLES + libxcb-cursor + libvncserver stubs (Qt6 needs libEGL.so.1
# even for software rendering; libxcb-cursor is needed by the Qt xcb QPA
# plugin from Qt 6.5+; libvncserver is needed by x11vnc)
if [[ -d "${LIBS_DIR}" ]] && [[ -f "${LIBS_DIR}/libEGL.so.1" ]]; then
    export LD_LIBRARY_PATH="${LIBS_DIR}:${LD_LIBRARY_PATH:-}"
    log "staged libs at ${LIBS_DIR} (libEGL, libxcb-cursor, libvncserver, etc.)"
fi

# CAD_0 build — auto-build if missing
if [[ ! -d "${PROJECT_DIR}/build/python/CAD_0" ]]; then
    log "build not found — configuring and building with CMake..."
    
    if ! have cmake || ! have ninja; then
        if have sudo; then
            log "installing system cmake & ninja..."
            sudo apt-get install -y cmake ninja-build 2>&1 | tail -3
        fi
    fi

    cd "${PROJECT_DIR}"
    cmake -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCAD_0_BUILD_TESTS=ON \
        -DCAD_0_BUILD_EXAMPLES=ON \
        -DPython3_EXECUTABLE="$(which python3)"
    
    cmake --build build -j"$(nproc)" || die "cmake build failed"
fi

[[ -d "${PROJECT_DIR}/build/python/CAD_0" ]] || die "build directory invalid or bindings missing in build/python/CAD_0"

log "environment ready."

# Python deps
python3 -c "import PySide6, OpenGL, numpy" 2>/dev/null || {
    log "installing PySide6 + PyOpenGL + numpy via pip..."
    python3 -m pip install --quiet PySide6 PyOpenGL numpy 2>&1 | tail -3
}
python3 -c "import PySide6, OpenGL, numpy" 2>/dev/null || {
    err "PySide6 / PyOpenGL / numpy not importable in the current python"
    die "install with: pip install PySide6 PyOpenGL numpy"
}

log "environment OK."

# -----------------------------------------------------------------------------
# Start Xvfb
# -----------------------------------------------------------------------------
log "starting Xvfb on display ${DISPLAY} (${SCREEN_W}x${SCREEN_H}x${SCREEN_D})..."
# Clean up any stale Xvfb
pkill -f "Xvfb ${DISPLAY}" 2>/dev/null || true
sleep 0.2
Xvfb "${DISPLAY}" -screen 0 "${SCREEN_W}x${SCREEN_H}x${SCREEN_D}" \
                 -ac -nolisten tcp +extension RANDR \
                 >/tmp/cad0_xvfb.log 2>&1 &
echo $! > /tmp/cad0_xvfb.pid
sleep 1
if ! kill -0 "$(cat /tmp/cad0_xvfb.pid)" 2>/dev/null; then
    die "Xvfb failed to start. See /tmp/cad0_xvfb.log"
fi
log "Xvfb pid=$(cat /tmp/cad0_xvfb.pid)"

# -----------------------------------------------------------------------------
# Start fluxbox window manager
# -----------------------------------------------------------------------------
log "starting fluxbox..."
DISPLAY="${DISPLAY}" fluxbox -display "${DISPLAY}" \
    >/tmp/cad0_fluxbox.log 2>&1 &
echo $! > /tmp/cad0_fluxbox.pid
sleep 1
log "fluxbox pid=$(cat /tmp/cad0_fluxbox.pid)"

# -----------------------------------------------------------------------------
# Start the CAD_0 app
# -----------------------------------------------------------------------------
log "starting CAD_0 desktop app..."
cd "${PROJECT_DIR}"
DISPLAY="${DISPLAY}" \
PYTHONPATH="${PROJECT_DIR}/build/python" \
LIBGL_ALWAYS_SOFTWARE=1 \
MESA_GL_VERSION_OVERRIDE=2.1 \
QT_QPA_PLATFORM=xcb \
python3 -m CAD_0.ui \
    >/tmp/cad0_app.log 2>&1 &
echo $! > /tmp/cad0_app.pid
sleep 2
if ! kill -0 "$(cat /tmp/cad0_app.pid)" 2>/dev/null; then
    err "CAD_0 app failed to start. Last 20 lines of log:"
    tail -20 /tmp/cad0_app.log
    die "see /tmp/cad0_app.log for details"
fi
log "CAD_0 app pid=$(cat /tmp/cad0_app.pid)"

# -----------------------------------------------------------------------------
# Start x11vnc
# -----------------------------------------------------------------------------
log "starting x11vnc on port ${VNC_PORT}..."
pkill -f "x11vnc.*:99" 2>/dev/null || true
sleep 0.2
# -rfbauth would require a password file; we use -nopw for simplicity in
# Codespaces (the forwarded port is already auth-gated by GitHub).
x11vnc -display "${DISPLAY}" \
       -rfbport "${VNC_PORT}" \
       -nopw -forever -shared -noxdamage -noxfixes -noxrecord \
       -bg -o /tmp/cad0_vnc.log -quiet
sleep 1
# x11vnc forks to background by itself, so we find its pid another way
pgrep -f "x11vnc.*:${DISPLAY_NUM}" > /tmp/cad0_vnc.pid 2>/dev/null || true
log "x11vnc running on port ${VNC_PORT} (pid=$(cat /tmp/cad0_vnc.pid 2>/dev/null || echo '?'))"

# -----------------------------------------------------------------------------
# Start websockify (noVNC web client)
# -----------------------------------------------------------------------------
log "starting websockify on port ${WEB_PORT}..."
pkill -f "websockify.*${WEB_PORT}" 2>/dev/null || true
sleep 0.2
websockify --web="${NOVNC_DIR}" "${WEB_PORT}" "localhost:${VNC_PORT}" \
    >/tmp/cad0_websock.log 2>&1 &
echo $! > /tmp/cad0_websock.pid
sleep 1
if ! kill -0 "$(cat /tmp/cad0_websock.pid)" 2>/dev/null; then
    err "websockify failed to start. Last 20 lines of log:"
    tail -20 /tmp/cad0_websock.log
    die "see /tmp/cad0_websock.log for details"
fi
log "websockify pid=$(cat /tmp/cad0_websock.pid)"

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
cat <<EOF

============================================================
 CAD_0 desktop app is now running in your Codespace.

 Open this URL in a browser (Codespaces will auto-forward):

      http://localhost:${WEB_PORT}/vnc.html

 In the VS Code "Ports" tab, find port ${WEB_PORT} and click
 the globe icon to get a public HTTPS URL like:

      https://<your-codespace>-${WEB_PORT}.app.github.dev/vnc.html

 Controls:
   - Left-drag:  orbit camera
   - Middle-drag: pan
   - Wheel:       zoom
   - Menu:        Create → Sphere / Box / Cylinder / Torus
                  Edit → Undo / Redo / Delete
                  View → Show Grid / Axes / Frame All

 Stop everything:
      ./scripts/start_codespaces.sh --stop

 Logs:
   - app:       /tmp/cad0_app.log
   - Xvfb:      /tmp/cad0_xvfb.log
   - VNC:       /tmp/cad0_vnc.log
   - websock:   /tmp/cad0_websock.log
============================================================

EOF

log "ready. Press Ctrl+C in this terminal to stop the launcher (background processes keep running)."
log "Use './scripts/start_codespaces.sh --stop' to stop all processes."

# Wait for any of the background processes to die (or for Ctrl+C)
wait
