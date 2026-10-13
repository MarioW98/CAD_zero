"""CAD_0 UI import e2e — File menu Import STL/OBJ, feature tree, panel.

FASE 3 of the STL/OBJ import feature. Two layers:

* Source-level checks (no PySide6 required, run everywhere): the File
  menu carries the Import actions, the window dispatches on node kind,
  the property panel has the read-only field API.

* Full end-to-end (PySide6 required; skipped when absent — same policy
  as test_ui_smoke.py): headless MainWindow, import an STL via the
  dialog-free core, save, REOPEN through on_open, and verify the mesh
  survives with identical counts next to an editable parametric SDF
  node. Run headless with QT_QPA_PLATFORM=offscreen (or "minimal") and
  the usual Qt GL libraries present.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

import CAD_0
import CAD_0.io as cad_io
from CAD_0 import math as math_mod
from CAD_0 import sdf as sdf_mod

# The app source is at <root>/app/src/CAD_0/ui/ (the build copies it over
# the Python package; the canonical copy is the app one).
_ROOT = Path(__file__).resolve().parents[3]
_UI_DIR = _ROOT / "app" / "src" / "CAD_0" / "ui"
UI_MAIN_WINDOW = _UI_DIR / "main_window.py"
UI_PROPERTY_PANEL = _UI_DIR / "property_panel.py"


# ---- Source-level checks (always run) ------------------------------------------


def test_file_menu_has_import_actions():
    src = UI_MAIN_WINDOW.read_text()
    assert 'QAction("Import &STL...", self)' in src
    assert 'QAction("Import &OBJ...", self)' in src
    assert 'self._on_import("stl")' in src
    assert 'self._on_import("obj")' in src


def test_window_dispatches_on_node_kind():
    src = UI_MAIN_WINDOW.read_text()
    # Honest dispatch: mesh nodes never take the SDF paths.
    assert 'node.kind == "mesh"' in src
    assert 'set_readonly_fields' in src
    assert '"Mesh" if node.kind == "mesh" else "SDF"' in src
    # Status bar reports the post-welding counts.
    assert "(welded)" in src


def test_property_panel_has_readonly_fields():
    src = UI_PROPERTY_PANEL.read_text()
    assert "def set_readonly_fields" in src


# ---- Full end-to-end (PySide6) --------------------------------------------------


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.fixture()
def main_window(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "minimal")
    from PySide6.QtWidgets import QApplication, QMessageBox

    # Avoid modal dialogs blocking headless runs (close prompts).
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.Discard))
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    yield window
    window.close()
    window.deleteLater()
    app.processEvents()


def _write_stl(tmp: Path) -> Path:
    """A real STL to import: MC sphere exported by our own exporter."""
    mesh = sdf_mod.marching_cubes(sdf_mod.sphere(1.25), 20, True)
    path = tmp / "bracket.stl"
    assert cad_io.export_stl_binary(str(path), mesh, "bracket")
    return path


def test_import_save_reopen_end_to_end(main_window, monkeypatch, tmp_path):
    w = main_window
    assert w._has_kernel

    # A parametric SDF node (the registry path the Create menu uses).
    w._create_registered("sphere")
    assert w._scene.size() == 1
    assert w._scene.node_at(0).kind == "sdf"

    # Import the STL through the dialog-free core of _on_import.
    stl = _write_stl(tmp_path)
    assert w._import_mesh(str(stl), "stl") is True

    # Scene + feature tree.
    assert w._scene.size() == 2
    assert w._scene.node_at(1).kind == "mesh"
    assert w._scene.node_at(1).name == "bracket"
    assert w._feature_tree.topLevelItemCount() == 2
    assert w._feature_tree.topLevelItem(1).text(1) == "Mesh"
    assert w._feature_tree.topLevelItem(0).text(1) == "SDF"

    # The import fired NodeAdded -> dirty flag by construction.
    assert w._dirty is True

    # Counts post-welding match a direct kernel load.
    direct = cad_io.load_stl(str(stl))
    node = w._scene.node_at(1)
    assert node.mesh.vertex_count() == direct.vertex_count()
    assert node.mesh.triangle_count() == direct.triangle_count()

    # Status bar carries the post-welding counts.
    assert "welded" in w._status.currentMessage()
    assert str(node.mesh.vertex_count()) in w._status.currentMessage()

    # Selecting the mesh node shows read-only info (no editors).
    w._feature_tree.setCurrentItem(w._feature_tree.topLevelItem(1))
    info = w._mesh_info_fields(w._scene.node_at(1))
    assert info["Type"] == "Imported mesh"
    assert info["Vertices"] == str(node.mesh.vertex_count())
    assert info["Triangles"] == str(node.mesh.triangle_count())
    assert "Bounds min" in info

    # ---- save ----
    project = tmp_path / "imported.CAD_0"
    w._do_save(str(project))
    assert project.exists()
    assert w._dirty is False

    # ---- reopen through the real on_open flow (dialog patched) ----
    from PySide6.QtWidgets import QFileDialog
    monkeypatch.setattr(
        QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(project), "")))
    w.on_open()

    # The mesh survives with the same counts, next to the parametric SDF.
    assert w._scene.size() == 2
    mesh_node = next(
        w._scene.node_at(i) for i in range(2)
        if w._scene.node_at(i).kind == "mesh")
    assert mesh_node.name == "bracket"
    assert mesh_node.mesh.vertex_count() == node.mesh.vertex_count()
    assert mesh_node.mesh.triangle_count() == node.mesh.triangle_count()

    sdf_node = next(
        w._scene.node_at(i) for i in range(2)
        if w._scene.node_at(i).kind == "sdf")
    assert sdf_node.builder == "sphere"
    # Still editable: the UI re-armed the parametric metadata.
    assert w._node_params.get(sdf_node.id)
    probe = math_mod.Vec3f(0.5, 0.5, 0.5)
    # _create_registered uses the registry default (radius 1.0):
    # |probe| = sqrt(3)/2 -> value = sqrt(3)/2 - 1.
    assert sdf_node.body.value(probe) == pytest.approx(0.8660254 - 1.0, abs=1e-5)

    # The rebuilt tree labels the mesh node correctly.
    labels = [w._feature_tree.topLevelItem(i).text(1) for i in range(2)]
    assert sorted(labels) == ["Mesh", "SDF"]


def test_import_dialog_path_uses_qfiledialog(main_window, monkeypatch, tmp_path):
    """_on_import drives QFileDialog and routes into _import_mesh."""
    w = main_window
    from PySide6.QtWidgets import QFileDialog

    calls = {"n": 0}

    def fake_get_open(parent, title, d, filters):
        calls["n"] += 1
        assert "STL" in filters
        return (str(_write_stl(tmp_path)), "")

    monkeypatch.setattr(QFileDialog, "getOpenFileName", staticmethod(fake_get_open))
    assert w._on_import("stl") is None  # runs the flow
    assert calls["n"] == 1
    assert w._scene.size() == 1
    assert w._scene.node_at(0).kind == "mesh"

    # Cancelling the dialog imports nothing.
    monkeypatch.setattr(
        QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: ("", "")))
    w._on_import("obj")
    assert w._scene.size() == 1


def test_import_error_is_reported_not_fatal(main_window, tmp_path):
    w = main_window
    garbage = tmp_path / "garbage.stl"
    garbage.write_bytes(b"definitely not an STL file")
    # The error path shows a QMessageBox (patched) and returns False.
    from PySide6.QtWidgets import QMessageBox
    QMessageBox.critical = staticmethod(lambda *a, **k: None)
    assert w._import_mesh(str(garbage), "stl") is False
    assert w._scene.size() == 0
