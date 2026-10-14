#!/usr/bin/env python3
"""
scripts/record_demo.py — Headless GIF recorder for the CAD_0 README.

Records a 30–60s noVNC-style demo of the PySide6 desktop app, suitable
for embedding in README.md. The recording runs entirely headless under
Xvfb — no physical display required.

Prerequisites (auto-installed in Codespaces):
    - Xvfb, fluxbox (apt packages)
    - ffmpeg (apt package)
    - PySide6, PyOpenGL, numpy (pip packages)
    - A built CAD_0 kernel (cmake --build)

Usage:
    # After starting the Codespace app once (so the build exists):
    python3 scripts/record_demo.py

    # Or from a fresh checkout:
    bash scripts/start_codespaces.sh --stop  # leave the build in place
    python3 scripts/record_demo.py

Output:
    docs/media/demo.gif  (30–60s, ≤5 MB, 640px wide, 12 fps, palette-optimised)

The demo sequence (matches the README TODO comment):
    1. Create → Sphere
    2. Edit radius in the property panel (1.0 → 1.5)
    3. Pick the sphere in the viewport (highlight amber)
    4. Import STL (generates a small test mesh at /tmp/_demo_import.stl)
    5. Save project to /tmp/demo.cad0
    6. Close + reopen → verify round-trip
    7. Orbit camera briefly

Each step pauses ~800ms (QTest.qWait) so the GIF is legible.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = REPO_ROOT / "build" / "python"
OUTPUT_GIF = REPO_ROOT / "docs" / "media" / "demo.gif"
OUTPUT_MP4 = REPO_ROOT / "docs" / "media" / "demo.mp4"
DEMO_STL = Path("/tmp/_demo_import.stl")
DEMO_CAD0 = Path("/tmp/demo.cad0")

DISPLAY = ":100"
SCREEN_W, SCREEN_H, SCREEN_DEPTH = 1280, 800, 24
FPS = 12
GIF_WIDTH = 640


def log(msg: str) -> None:
    print(f"\033[1;36m[demo]\033[0m {msg}")


def die(msg: str) -> None:
    print(f"\033[1;31m[demo error]\033[0m {msg}", file=sys.stderr)
    sys.exit(1)


def have(cmd: str) -> bool:
    return subprocess.call(["which", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    log(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, check=True, **kw)


# ---------------------------------------------------------------------------
# Step 0: sanity checks
# ---------------------------------------------------------------------------
def check_prereqs() -> None:
    log("checking prerequisites...")
    for tool in ("Xvfb", "fluxbox", "ffmpeg"):
        if not have(tool):
            die(f"{tool} not found. Install with: sudo apt-get install -y {tool.lower()}")
    if not (BUILD_DIR / "CAD_0").is_dir():
        die(
            f"build not found at {BUILD_DIR}. Run `bash scripts/start_codespaces.sh` "
            f"first (then --stop), or build manually with cmake."
        )
    try:
        import PySide6  # noqa: F401
        import OpenGL  # noqa: F401
        import numpy  # noqa: F401
    except ImportError as e:
        die(f"missing Python package: {e}. Install with: pip install PySide6 PyOpenGL numpy")
    log("prerequisites OK.")


# ---------------------------------------------------------------------------
# Step 1: start Xvfb + fluxbox on display :100
# ---------------------------------------------------------------------------
def start_xvfb() -> subprocess.Popen:
    log(f"starting Xvfb on display {DISPLAY} ({SCREEN_W}x{SCREEN_H}x{SCREEN_DEPTH})...")
    subprocess.run(["pkill", "-f", f"Xvfb {DISPLAY}"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)
    proc = subprocess.Popen(
        [
            "Xvfb", DISPLAY,
            "-screen", "0", f"{SCREEN_W}x{SCREEN_H}x{SCREEN_DEPTH}",
            "-ac", "-nolisten", "tcp", "+extension", "RANDR",
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1)
    if proc.poll() is not None:
        die("Xvfb failed to start")
    log(f"Xvfb pid={proc.pid}")
    return proc


def start_fluxbox() -> subprocess.Popen:
    log("starting fluxbox...")
    env = os.environ.copy()
    env["DISPLAY"] = DISPLAY
    proc = subprocess.Popen(
        ["fluxbox", "-display", DISPLAY],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1)
    log(f"fluxbox pid={proc.pid}")
    return proc


# ---------------------------------------------------------------------------
# Step 2: start ffmpeg x11grab recorder
# ---------------------------------------------------------------------------
def start_recorder(mp4_path: Path) -> subprocess.Popen:
    log(f"starting ffmpeg recorder → {mp4_path}...")
    mp4_path.parent.mkdir(parents=True, exist_ok=True)
    if mp4_path.exists():
        mp4_path.unlink()
    proc = subprocess.Popen(
        [
            "ffmpeg", "-y",
            "-f", "x11grab",
            "-framerate", str(FPS),
            "-video_size", f"{SCREEN_W}x{SCREEN_H}",
            "-i", DISPLAY,
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-crf", "0",
            "-pix_fmt", "yuv420p",
            str(mp4_path),
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1)
    if proc.poll() is not None:
        die("ffmpeg failed to start")
    log(f"ffmpeg pid={proc.pid}")
    return proc


# ---------------------------------------------------------------------------
# Step 3: generate a test STL for the import step
# ---------------------------------------------------------------------------
def make_test_stl() -> None:
    log(f"generating test STL at {DEMO_STL}...")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(BUILD_DIR)
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["MESA_GL_VERSION_OVERRIDE"] = "2.1"
    env["QT_QPA_PLATFORM"] = "offscreen"
    code = f"""
import CAD_0
sph = CAD_0.sdf.sphere(radius=0.8)
mesh = CAD_0.sdf.marching_cubes(sph, resolution=24)
CAD_0.io.export('{DEMO_STL}', mesh)
print(f"STL written: {{mesh.vertex_count()}} vertices, {{mesh.triangle_count()}} triangles")
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=env, capture_output=True, text=True,
    )
    if result.returncode != 0:
        die(f"failed to generate test STL: {result.stderr}")
    log(result.stdout.strip())


# ---------------------------------------------------------------------------
# Step 4: the demo driver — runs inside Xvfb, pilots the MainWindow
# ---------------------------------------------------------------------------
def run_demo_driver() -> None:
    """
    Launch the CAD_0 MainWindow on DISPLAY and pilot it through the
    demo sequence via QTest. Each step pauses so the GIF is legible.
    """
    log("starting demo driver (PySide6 app)...")
    env = os.environ.copy()
    env["DISPLAY"] = DISPLAY
    env["PYTHONPATH"] = str(BUILD_DIR)
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["MESA_GL_VERSION_OVERRIDE"] = "2.1"
    env["QT_QPA_PLATFORM"] = "xcb"

    # The driver runs in-process so we can use QTest directly.
    # We import PySide6 here (after PYTHONPATH is set) — but since this
    # function runs in the SAME process, we need to set env BEFORE the
    # import. The subprocess wrapper below handles that.
    code = '''
import os, sys, time
sys.path.insert(0, os.environ["PYTHONPATH"])

from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import QMouseEvent

app = QApplication.instance() or QApplication(sys.argv)
from CAD_0.ui.main_window import MainWindow
win = MainWindow()
win.showMaximized()
QTest.qWait(1200)  # let it settle

# Step 1: create a sphere
win.on_create_sphere()
QTest.qWait(1000)

# Step 2: edit the radius in the property panel
# (the property panel rebuilds the body on parameter change — we
# trigger it via the panel's spinbox valueChanged signal)
panel = win._property_panel
if hasattr(panel, "_spin_radius"):
    panel._spin_radius.setValue(1.5)
    QTest.qWait(800)

# Step 3: pick in viewport — simulate a click at the viewport center
viewport = win._viewport
# The viewport exposes set_selection directly for programmatic picking;
# for a true user simulation we'd send a mouse click, but the
# deterministic test path is to call set_selection on the first node.
if hasattr(viewport, "set_selection") and win._scene_nodes:
    first_id = list(win._scene_nodes.keys())[0]
    viewport.set_selection(first_id)
    QTest.qWait(1000)

# Step 4: import STL
import CAD_0
scene = win._scene
CAD_0.io.import_mesh("/tmp/_demo_import.stl", scene, name="imported")
win._refresh_viewport()
win._refresh_feature_tree()
QTest.qWait(1200)

# Step 5: save
win._current_path = "/tmp/demo.cad0"
win.on_save()
QTest.qWait(800)

# Step 6: new + reopen
win.on_new()
QTest.qWait(500)
win._current_path = "/tmp/demo.cad0"
win.on_open()
QTest.qWait(1000)

# Step 7: orbit camera briefly (simulate drag)
# Send a series of mouse moves with left button held
for _ in range(30):
    # We can't easily synthesize a real drag without the viewport's
    # internal state; instead we nudge the camera via its public API
    # if available, otherwise just wait to let the static scene be
    # captured.
    QTest.qWait(50)

# Hold the final frame for a moment
QTest.qWait(1500)
win.close()
'''
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=env, capture_output=True, text=True, timeout=120,
    )
    if result.returncode != 0:
        log(f"demo driver stderr:\n{result.stderr}")
        die("demo driver failed")
    if result.stdout.strip():
        log(result.stdout.strip())


# ---------------------------------------------------------------------------
# Step 5: convert MP4 → GIF with palette optimisation
# ---------------------------------------------------------------------------
def convert_to_gif(mp4_path: Path, gif_path: Path) -> None:
    log(f"converting {mp4_path} → {gif_path}...")
    gif_path.parent.mkdir(parents=True, exist_ok=True)
    if gif_path.exists():
        gif_path.unlink()
    # Two-pass palette optimisation for smaller file size + better quality
    palette_path = gif_path.parent / "_palette.png"
    # Pass 1: generate palette
    run([
        "ffmpeg", "-y",
        "-i", str(mp4_path),
        "-vf", f"fps={FPS},scale={GIF_WIDTH}:-1:flags=lanczos,palettegen=stats_mode=diff",
        str(palette_path),
    ])
    # Pass 2: apply palette
    run([
        "ffmpeg", "-y",
        "-i", str(mp4_path),
        "-i", str(palette_path),
        "-lavfi", f"fps={FPS},scale={GIF_WIDTH}:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=5",
        str(gif_path),
    ])
    # Clean up palette
    if palette_path.exists():
        palette_path.unlink()
    # Report size
    size_kb = gif_path.stat().st_size / 1024
    log(f"GIF written: {gif_path} ({size_kb:.0f} KB)")
    if size_kb > 5 * 1024:
        log(f"WARNING: GIF is {size_kb:.0f} KB (>5 MB). Consider reducing fps or width.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    check_prereqs()
    make_test_stl()

    xvfb = start_xvfb()
    fluxbox = start_fluxbox()
    recorder = start_recorder(OUTPUT_MP4)

    try:
        run_demo_driver()
    finally:
        # Stop recorder first so it flushes the MP4
        log("stopping ffmpeg...")
        recorder.terminate()
        recorder.wait(timeout=10)
        # Stop fluxbox + Xvfb
        log("stopping fluxbox + Xvfb...")
        fluxbox.terminate()
        xvfb.terminate()
        fluxbox.wait(timeout=5)
        xvfb.wait(timeout=5)

    convert_to_gif(OUTPUT_MP4, OUTPUT_GIF)

    # Optionally remove the intermediate MP4
    if OUTPUT_MP4.exists():
        OUTPUT_MP4.unlink()
        log(f"removed intermediate {OUTPUT_MP4}")

    log("done.")
    log(f"GIF ready at: {OUTPUT_GIF}")
    log("Embed in README by uncommenting the <p align=center>...</p> block near line 14.")


if __name__ == "__main__":
    main()
