"""CAD_0 UI startup smoke test — maximized startup + launcher UI sync.

Regression guard for the 2026-10-13 startup bug: MainWindow.__init__ used
to resize the window to a FIXED 1280x800 — exactly the Xvfb/Codespace
screen — so once fluxbox added its title bar and borders the top of the
window (title bar AND menu bar) landed off-screen. The fix, two layers
like test_ui_import.py / test_ui_navigation.py:

* Source-level checks (no PySide6 required, run everywhere): no
  screen-sized fixed resize anymore; a small fallback size only under a
  no-usable-screen guard; main() shows the window via showMaximized()
  (the programmatic maximize button) instead of plain show(); the
  Codespaces launcher rsyncs the pure-Python UI into build/python right
  before launching the app (the build only installs the UI when
  build/python does not exist yet, so UI edits used to stay invisible
  until a re-build).

* Full end-to-end (PySide6 required; skipped when absent): the REAL
  main() runs in a fresh subprocess under the offscreen QPA with
  app.exec() patched to process events once and return — the window
  must construct, show, expose and paint without exceptions, and the
  menu bar must still carry File/Create/Edit/View. Per the spec the
  test asserts robustness, NOT geometry: on the offscreen platform the
  WindowMaximized state is meaningless, so it is deliberately not
  checked. The subprocess exits through os._exit(0) after printing its
  verdict: the pre-existing interpreter-teardown segfault of shown GL
  windows (see CODESPACES.md) never fires, and pytest stays isolated
  from it.
"""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

# The app source is at <root>/app/src/CAD_0/ui/ (the build copies it over
# the Python package; the canonical copy is the app one).
_ROOT = Path(__file__).resolve().parents[3]
UI_MAIN_WINDOW = _ROOT / "app" / "src" / "CAD_0" / "ui" / "main_window.py"
LAUNCHER = _ROOT / "scripts" / "start_codespaces.sh"

RSYNC_LINE = (
    'rsync -a "${PROJECT_DIR}/app/src/CAD_0/ui/" '
    '"${PROJECT_DIR}/build/python/CAD_0/ui/"'
)


# ---- Source-level checks (always run) ------------------------------------------


def test_no_fixed_screen_sized_resize_source():
    """The 1280x800 fixed resize is gone; only a guarded fallback stays."""
    src = UI_MAIN_WINDOW.read_text()
    # The original bug: a window exactly as big as the screen.
    assert "self.resize(1280, 800)" not in src
    # The edge-case fallback: a window strictly smaller than common
    # screens, only for platforms that report no usable screen at all.
    assert "self.resize(1200, 720)" in src
    assert "self.screen()" in src
    assert "screen is None" in src


def test_startup_shows_maximized_source():
    """main() calls showMaximized() — no plain show() left."""
    src = UI_MAIN_WINDOW.read_text()
    main_src = src.split("def main()", 1)[1]
    assert "window.showMaximized()" in main_src
    # Plain show() must be gone from the entry point (it used to leave
    # the window at the fixed size; note "window.show()" does not match
    # "window.showMaximized()").
    assert "window.show()" not in main_src


def test_launcher_syncs_pure_python_ui_source():
    """The launcher rsyncs app/src UI into build/python BEFORE launching."""
    launcher = LAUNCHER.read_text()
    assert RSYNC_LINE in launcher, "launcher must sync the pure-Python UI"
    # Order matters: the sync must happen before the app starts running
    # the (possibly stale) copy.
    assert launcher.index(RSYNC_LINE) < launcher.index("python3 -m CAD_0.ui")


# ---- Full end-to-end (PySide6) --------------------------------------------------


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


def test_real_main_runs_and_menus_present():
    """The real main() shows the window without exceptions, menus intact.

    Runs in a subprocess so the shown (and painted, where GL is up) GL
    window never reaches interpreter teardown — the documented
    pre-existing shutdown segfault stays out of the pytest process.
    Robustness only: no geometry/WindowMaximized assertions, the
    offscreen platform makes them meaningless.
    """
    pytest.importorskip("PySide6")
    pytest.importorskip("CAD_0")

    code = textwrap.dedent(
        """
        import os
        import sys

        from PySide6.QtWidgets import QApplication, QMessageBox

        # Never block headless runs on modal dialogs (kernel-missing
        # warning, close prompts): mirror the navigation suite's fixture.
        QMessageBox.warning = staticmethod(lambda *a, **k: None)

        # Run the startup sequence (show + expose + first paint) but
        # return from exec() right after one event-loop pass.
        def _exec_once(*args, **kwargs):
            QApplication.processEvents()
            return 0
        QApplication.exec = staticmethod(_exec_once)

        import CAD_0.ui.main_window as mw

        windows = []
        _orig_init = mw.MainWindow.__init__
        def _capture(self, *a, **k):
            _orig_init(self, *a, **k)
            windows.append(self)
        mw.MainWindow.__init__ = _capture

        try:
            mw.main()
        except SystemExit as exc:  # sys.exit(app.exec()) -> 0
            assert exc.code in (0, None), f"main() exited with {exc.code}"

        assert windows, "main() did not construct a MainWindow"
        titles = sorted(
            a.text().replace("&", "") for a in windows[0].menuBar().actions()
        )
        print("MENUS " + ",".join(titles), flush=True)
        missing = {"File", "Create", "Edit", "View"} - set(titles)
        assert not missing, f"missing top-level menus: {sorted(missing)}"
        print("STARTUP_OK", flush=True)

        # Bypass interpreter teardown on purpose: a shown+destroyed GL
        # window segfaults at exit (pre-existing, documented in
        # CODESPACES.md). os._exit skips Qt/GL destruction entirely.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
        """
    )

    env = dict(os.environ)
    env["QT_QPA_PLATFORM"] = "offscreen"  # works with or without a display
    proc = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True, text=True, timeout=180, env=env,
    )

    assert "STARTUP_OK" in proc.stdout, (
        f"startup subprocess failed (rc={proc.returncode})\n"
        f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}"
    )
    menus_line = next(
        (ln for ln in proc.stdout.splitlines() if ln.startswith("MENUS ")), ""
    )
    titles = set(menus_line[len("MENUS "):].split(","))
    assert {"File", "Create", "Edit", "View"} <= titles
