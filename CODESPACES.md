# Using CAD_0 in GitHub Codespaces

This guide explains how to run the CAD_0 desktop app inside a GitHub
Codespace, with no GUI on the host — the app is exposed through a
browser-based VNC client (noVNC).

## One-shot launch

After opening the project in a Codespace, run:

```bash
cd CAD_0
./scripts/start_codespaces.sh
```

The launcher will:
1. Install missing system packages via `apt-get` (x11vnc, fluxbox,
   libxcb-cursor0, libvncserver1) — uses passwordless sudo.
2. Install missing Python packages via `pip` (cmake, ninja, PySide6,
   PyOpenGL, websockify, numpy).
3. Build the C++ extension if `build/python/` doesn't exist yet.
4. Start a virtual X server (Xvfb) on display :99 at 1280x800.
5. Start fluxbox (a lightweight window manager).
6. Start the CAD_0 app on that display.
7. Start x11vnc on port 5900 (VNC server attached to display :99).
8. Start websockify on port 6080 (WebSocket → VNC bridge + noVNC web client).

Once the launcher prints "ready", open the URL:

```
http://localhost:6080/vnc.html
```

In the Codespace, port 6080 is auto-forwarded. To get a public HTTPS URL:
1. Open the **Ports** tab in the bottom panel of VS Code.
2. Find port `6080`.
3. Right-click → **Port Visibility** → **Public**.
4. Click the globe icon to open the URL in your browser.

## Controls

| Input | Action |
|---|---|
| Left-drag | Orbit camera |
| Middle-drag | Pan |
| Wheel | Zoom |
| Click on shape | Select it |

## Menu actions

- **File → New Scene**: clear the scene
- **File → Import STL/OBJ**: add an imported mesh node to the scene
- **File → Export STL/OBJ/3MF**: save the first mesh
- **Create → Sphere / Box / Cylinder / Torus**: add a primitive
- **Edit → Undo / Redo (Ctrl+Z / Ctrl+Y)**: undo/redo last action
- **Edit → Delete Selected (Del)**: remove the selected shape
- **View → Show Grid / Axes**: toggle ground grid and XYZ axes
- **View → Frame All (F)**: zoom to fit all shapes

## Stop the app

```bash
./scripts/start_codespaces.sh --stop
```

This kills Xvfb, fluxbox, x11vnc, websockify, and the CAD_0 app.

## UI smoke tests

`bindings/python/tests/test_ui_navigation.py` is the headless regression
suite for the two UI bugs fixed on 2026-10-13 (invisible menu bar on
non-native WMs; viewport navigation dead when GL is active). It checks
that the menu bar is forced in-window with all the key actions, that
synthetic mouse/wheel events delivered to the embedded GL window steer
the camera at the first movement, that short clicks still pick, and
that the no-GL fallback stays usable.

`bindings/python/tests/test_ui_startup.py` is the startup smoke test
for the maximized-window fix of the same day (the window used to open
at a fixed 1280x800 — exactly the 1280x800 Xvfb screen — so fluxbox's
title bar pushed the top of the window off-screen). It pins the policy
in source (no fixed screen-sized resize; a 1200x720 fallback only when
Qt reports no usable screen; `main()` calls `showMaximized()`, the
programmatic maximize button) and runs the REAL `main()` in a fresh
subprocess under the offscreen QPA: the window must construct, show
and paint without exceptions while the menu bar still carries
File/Create/Edit/View. The launcher is covered too: the pure-Python UI
is rsynced into `build/python/CAD_0/ui/` before every launch, so UI
edits are always the ones actually executed.

The camera-steering assertions need a real GL context: the offscreen
Qt platform creates it through EGL, and Mesa only opens EGL with a
display around. In a Codespace (Xvfb already running via the launcher):

```bash
export DISPLAY=:99
export QT_QPA_PLATFORM=offscreen
PYTHONPATH=build/linux-gcc-release/python \
  python3 -m pytest bindings/python/tests/test_ui_navigation.py \
    bindings/python/tests/test_ui_startup.py -v
```

On a desktop Linux with a GPU, the same command works as-is. On a
machine without any GL (no `libEGL`/Mesa), the GL-dependent tests skip
with an explicit reason — the menu-bar, startup and fallback checks
still run. CI runs the same modules under its own Xvfb (see
`.github/workflows/ci-linux.yml`, step "UI smoke tests (GL)").

Known unrelated residual: quitting a process whose GL window has
actually been shown and painted segfaults at interpreter exit (Qt/GL
tear-down, pre-existing, visible only at shutdown; the in-process
smoke tests deliberately never `show()` the window, and the startup
test shows one only inside a subprocess that exits through
`os._exit(0)` before Qt/GL teardown).

## Troubleshooting

### "Could not load the Qt platform plugin 'xcb'"

The Qt xcb plugin needs `libxcb-cursor.so.0`. The launcher installs it
automatically via `apt-get`. If that fails (no sudo), the launcher falls
back to the staged `.so` files under `/home/z/my-project/.libs/` — but
those are bundled with the zip distribution only when you run the
launcher from this sandbox. On a fresh Codespace clone, `sudo apt-get
install -y libxcb-cursor0` is the fix.

### "QOpenGLWindow: Failed to create context"

This is a warning, not an error. Codespaces doesn't have a real GPU, so
the viewport shows a "GL not available" label instead of rendering 3D.
All other functionality (scene graph, SDF, mesh extraction, undo/redo,
export) works without a GPU.

To get actual 3D rendering in Codespaces, install `mesa-utils` and run
with `LIBGL_ALWAYS_SOFTWARE=1` (software rendering, slow but works):

```bash
sudo apt-get install -y mesa-utils libgl1-mesa-dri
```

Then re-run the launcher.

### Port 6080 not auto-forwarding

In the Codespace's VS Code, open the **Ports** tab and click **Forward
Port** manually with `6080`. Set visibility to **Public** if you want
to share the URL.
