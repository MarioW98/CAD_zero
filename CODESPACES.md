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
