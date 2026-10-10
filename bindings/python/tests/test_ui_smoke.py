"""Smoke test for the CAD_0.ui Python package.

Verifies that the UI modules import cleanly (when PySide6 is available)
and that the dataclasses/models behave as expected without a display.
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest


# Find the project root (where the CAD_0/ directory lives).
# This test file is at: <root>/bindings/python/tests/test_ui_smoke.py
# The app source is at: <root>/app/src/CAD_0/ui/
_ROOT = Path(__file__).resolve().parents[3]
_UI_DIR = _ROOT / "app" / "src" / "CAD_0" / "ui"


def test_feature_tree_dataclass_importable() -> None:
    """Import the FeatureNode dataclass without requiring PySide6."""
    import ast
    src_path = _UI_DIR / "feature_tree.py"
    if not src_path.exists():
        pytest.skip(f"UI source not found: {src_path}")
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "FeatureNode" in class_names
    assert "FeatureTreeModel" in class_names
    assert "FeatureTreePanel" in class_names


def test_property_panel_parses() -> None:
    """Verify the property_panel module parses without syntax errors."""
    import ast
    src_path = _UI_DIR / "property_panel.py"
    if not src_path.exists():
        pytest.skip(f"UI source not found: {src_path}")
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "PropertyPanel" in class_names


def test_main_window_parses() -> None:
    """Verify the main_window module parses without syntax errors."""
    import ast
    src_path = _UI_DIR / "main_window.py"
    if not src_path.exists():
        pytest.skip(f"UI source not found: {src_path}")
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "MainWindow" in class_names


def test_viewport_widget_parses() -> None:
    """Verify the viewport_widget module parses without syntax errors."""
    import ast
    src_path = _UI_DIR / "viewport_widget.py"
    if not src_path.exists():
        pytest.skip(f"UI source not found: {src_path}")
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "ViewportWidget" in class_names
    assert "_ViewportWindow" in class_names


def _pyside6_available() -> bool:
    """True if PySide6 is actually importable in this interpreter.

    Bug-fix note: the old skipif used `"PySide6" not in sys.modules`, which
    only checks whether something *already imported* PySide6 before test
    collection — so the test silently skipped on every machine where it
    could actually have run. Checking with find_spec() makes the test do
    what its docstring promises.
    """
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


@pytest.mark.skipif(
    not _pyside6_available(),
    reason="PySide6 not installed",
)
def test_ui_modules_import_with_pyside6() -> None:
    """If PySide6 is installed, the UI modules should import cleanly."""
    importlib.import_module("CAD_0.ui.viewport_widget")
    importlib.import_module("CAD_0.ui.feature_tree")
    importlib.import_module("CAD_0.ui.property_panel")
    importlib.import_module("CAD_0.ui.main_window")
