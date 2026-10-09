#!/usr/bin/env bash
# scripts/start_codespaces.sh — Launch CAD_0 desktop app inside GitHub Codespaces.
#
# What this does, in order:
#   1. Verifies all required tools are installed (Xvfb, x11vnc, websockify,
#      fluxbox, the CAD_0 build, PySide6, PyOpenGL, matplotlib, XCB libraries).
#   2. Starts a virtual X server on display :99 at 1280x800x24.
#   3. Starts fluxbox (lightweight window manager) on that display.
#   4. Starts the CAD_0 desktop app on that display.
#   5. Starts x11vnc attached to display :99, listening on port 5900.
#   6. Starts websockify on port 6080 — wraps the VNC stream in WebSocket
#      and serves the noVNC web client at http://localhost:6080/vnc.html
#
# Usage:
#   ./scripts/start_codespaces.sh
#
# Stop everything:
#   ./scripts/start_codespaces.sh --stop

set -euo pipefail

# -----------------------------------------------------------------------------
# Configuration (Dynamic paths)
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DISPLAY_NUM=99
SCREEN_W=1280
SCREEN_H=800
SCREEN_D=24
VNC_PORT=5900            # internal VNC port (x11vnc listens here)
WEB_PORT=6080            # public-facing noVNC web port (forwarded by Codespaces)

DISPLAY=":${DISPLAY_NUM}"
export DISPLAY

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
log()  { printf '\033[1;36m[launch]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; }
die()  { err "$*"; exit 1; }
have() { command -v "$1" >/dev/null 2>&1; }

# -----------------------------------------------------------------------------
# Stop mode
# -----------------------------------------------------------------------------
if [[ "${1:-}" == "--stop" ]]; then
    log "stopping all CAD_0 / VNC processes..."
    pkill -f "python3 .*CAD_0.ui"      2>/dev/null || true
    pkill -f "x11vnc.*:${DISPLAY_NUM}" 2>/dev/null || true
    pkill -f "websockify.*${WEB_PORT}" 2>/dev/null || true
    pkill -f "fluxbox.*${DISPLAY}"     2>/dev/null || true
    pkill -f "Xvfb ${DISPLAY}"         2>/dev/null || true
    rm -f /tmp/cad0_xvfb.log /tmp/cad0_vnc.log /tmp/cad0_websock.log \
          /tmp/cad0_app.log /tmp/cad0_xvfb.pid /tmp/cad0_vnc.pid \
          /tmp/cad0_websock.pid /tmp/cad0_app.pid /tmp/cad0_fluxbox.pid \
          "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
    log "stopped."
    exit 0
fi

# -----------------------------------------------------------------------------
# Sanity checks & System Dependencies
# -----------------------------------------------------------------------------
log "checking environment in ${PROJECT_DIR}..."

# System dependencies for X11, OpenGL, and Qt6 XCB
REQUIRED_PKGS=(
    xvfb fluxbox x11vnc novnc websockify
    libgl1-mesa-dri libegl1
    libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-image0
    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0
    libxcb-xfixes0 libxcb-shape0
)

MISSING_PKGS=()
for pkg in "${REQUIRED_PKGS[@]}"; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
        MISSING_PKGS+=("$pkg")
    fi
done

if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
    if have sudo; then
        log "installing missing system packages: ${MISSING_PKGS[*]}..."
        sudo apt-get update -qq
        sudo apt-get install -y --no-install-recommends "${MISSING_PKGS[@]}" 2>&1 | tail -3
        sudo ldconfig
    else
        die "Missing packages: ${MISSING_PKGS[*]}. Run apt-get install with root privileges."
    fi
fi

# Multi-arch linker path
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

# noVNC client check
NOVNC_DIR="/usr/share/novnc"
[[ -d "${NOVNC_DIR}" ]] || die "noVNC web directory not found in ${NOVNC_DIR}"

# Python runtime dependencies
log "checking Python dependencies..."
python3 -c "import PySide6, OpenGL, numpy, matplotlib" 2>/dev/null || {
    log "installing required Python wheels..."
    python3 -m pip install --quiet PySide6 PyOpenGL numpy matplotlib websockify 2>&1 | tail -3
}

# -----------------------------------------------------------------------------
# CAD_0 build check
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# Start Xvfb
# -----------------------------------------------------------------------------
log "starting Xvfb on display ${DISPLAY} (${SCREEN_W}x${SCREEN_H}x${SCREEN_D})..."
pkill -f "Xvfb ${DISPLAY}" 2>/dev/null || true
rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
sleep 0.2

Xvfb "${DISPLAY}" -screen 0 "${SCREEN_W}x${SCREEN_H}x${SCREEN_D}" \
                 -ac -nolisten tcp +extension RANDR +extension GLX \
                 >/tmp/cad0_xvfb.log 2>&1 &
echo $! > /tmp/cad0_xvfb.pid
sleep 1

if ! kill -0 "$(cat /tmp/cad0_xvfb.pid)" 2>/dev/null; then
    die "Xvfb failed to start. See /tmp/cad0_xvfb.log"
fi
log "Xvfb running (pid=$(cat /tmp/cad0_xvfb.pid))"

# -----------------------------------------------------------------------------
# Start fluxbox
# -----------------------------------------------------------------------------
log "starting fluxbox window manager..."
DISPLAY="${DISPLAY}" fluxbox -display "${DISPLAY}" \
    >/tmp/cad0_fluxbox.log 2>&1 &
echo $! > /tmp/cad0_fluxbox.pid
sleep 0.5
log "fluxbox running (pid=$(cat /tmp/cad0_fluxbox.pid))"

# -----------------------------------------------------------------------------
# Start the CAD_0 desktop app
# -----------------------------------------------------------------------------
log "starting CAD_0 desktop application..."
cd "${PROJECT_DIR}"

DISPLAY="${DISPLAY}" \
PYTHONPATH="${PROJECT_DIR}/build/python:${PROJECT_DIR}/app/src:${PROJECT_DIR}/bindings/python:${PYTHONPATH:-}" \
LIBGL_ALWAYS_SOFTWARE=1 \
QT_QPA_PLATFORM=xcb \
python3 -m CAD_0.ui \
    >/tmp/cad0_app.log 2>&1 &
echo $! > /tmp/cad0_app.pid
sleep 2

if ! kill -0 "$(cat /tmp/cad0_app.pid)" 2>/dev/null; then
    err "CAD_0 app failed to start. Last 20 lines of /tmp/cad0_app.log:"
    tail -n 20 /tmp/cad0_app.log
    die "see /tmp/cad0_app.log for full trace"
fi
log "CAD_0 app running (pid=$(cat /tmp/cad0_app.pid))"

# -----------------------------------------------------------------------------
# Start x11vnc
# -----------------------------------------------------------------------------
log "starting x11vnc on port ${VNC_PORT}..."
pkill -f "x11vnc.*:${DISPLAY_NUM}" 2>/dev/null || true
sleep 0.2

x11vnc -display "${DISPLAY}" \
       -rfbport "${VNC_PORT}" \
       -nopw -forever -shared -noxdamage -noxfixes -noxrecord \
       -bg -o /tmp/cad0_vnc.log -quiet
sleep 1

pgrep -f "x11vnc.*:${DISPLAY_NUM}" > /tmp/cad0_vnc.pid 2>/dev/null || true
log "x11vnc running on port ${VNC_PORT} (pid=$(cat /tmp/cad0_vnc.pid 2>/dev/null || echo '?'))"

# -----------------------------------------------------------------------------
# Start websockify (noVNC bridge)
# -----------------------------------------------------------------------------
log "starting websockify on port ${WEB_PORT}..."
pkill -f "websockify.*${WEB_PORT}" 2>/dev/null || true
sleep 0.2

websockify --web="${NOVNC_DIR}" "${WEB_PORT}" "localhost:${VNC_PORT}" \
    >/tmp/cad0_websock.log 2>&1 &
echo $! > /tmp/cad0_websock.pid
sleep 1

if ! kill -0 "$(cat /tmp/cad0_websock.pid)" 2>/dev/null; then
    err "websockify failed to start. Last 20 lines of /tmp/cad0_websock.log:"
    tail -n 20 /tmp/cad0_websock.log
    die "see /tmp/cad0_websock.log for details"
fi
log "websockify running on port ${WEB_PORT} (pid=$(cat /tmp/cad0_websock.pid))"

# -----------------------------------------------------------------------------
# Summary and URL Info
# -----------------------------------------------------------------------------
cat <<EOF

============================================================
 CAD_0 desktop app is now running in your Codespace.

 Open this URL in a browser (or via the VS Code "Ports" tab):

      http://localhost:${WEB_PORT}/vnc.html

 In the VS Code "Ports" tab, click the globe icon next to port ${WEB_PORT}:
      https://<your-codespace>-${WEB_PORT}.app.github.dev/vnc.html

 Stop everything:
      ./scripts/start_codespaces.sh --stop

 Logs:
   - App:      /tmp/cad0_app.log
   - Xvfb:     /tmp/cad0_xvfb.log
   - VNC:      /tmp/cad0_vnc.log
   - Websock:  /tmp/cad0_websock.log
============================================================

EOF

log "Ready. Press Ctrl+C to disconnect terminal monitor (processes continue in background)."
wait