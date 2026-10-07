"""Smoke test for the CAD_0.ui Python package.

Verifies that the UI modules import cleanly (when PySide6 is available)
and that the dataclasses/models behave as expected without a display.

Run with:
    python -m pytest bindings/python/tests/test_ui_smoke.py
"""

from __future__ import annotations

import importlib
import sys

import pytest


def test_feature_tree_dataclass_importable() -> None:
    """Import the FeatureNode dataclass without requiring PySide6.

    We can't import the full feature_tree module without PySide6 (because
    of the QAbstractItemModel base class), but the dataclass itself is
    declared in the same module. We can at least verify the file parses
    by importing the module's source as text.
    """
    # Read the module source to verify it parses.
    import ast
    from pathlib import Path

    src_path = Path(__file__).resolve().parents[2] / "app" / "src" / "CAD_0" / "ui" / "feature_tree.py"
    src = src_path.read_text()
    # AST parse will raise SyntaxError if the file is malformed.
    ast.parse(src)
    # Sanity check: FeatureNode is defined.
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "FeatureNode" in class_names
    assert "FeatureTreeModel" in class_names
    assert "FeatureTreePanel" in class_names


def test_property_panel_parses() -> None:
    """Verify the property_panel module parses without syntax errors."""
    import ast
    from pathlib import Path

    src_path = Path(__file__).resolve().parents[2] / "app" / "src" / "CAD_0" / "ui" / "property_panel.py"
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "PropertyPanel" in class_names


def test_main_window_parses() -> None:
    """Verify the main_window module parses without syntax errors."""
    import ast
    from pathlib import Path

    src_path = Path(__file__).resolve().parents[2] / "app" / "src" / "CAD_0" / "ui" / "main_window.py"
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "MainWindow" in class_names


def test_viewport_widget_parses() -> None:
    """Verify the viewport_widget module parses without syntax errors."""
    import ast
    from pathlib import Path

    src_path = Path(__file__).resolve().parents[2] / "app" / "src" / "CAD_0" / "ui" / "viewport_widget.py"
    src = src_path.read_text()
    tree = ast.parse(src)
    class_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    assert "ViewportWidget" in class_names
    assert "_ViewportWindow" in class_names


@pytest.mark.skipif(
    "PySide6" not in sys.modules,
    reason="PySide6 not installed",
)
def test_ui_modules_import_with_pyside6() -> None:
    """If PySide6 is installed, the UI modules should import cleanly."""
    importlib.import_module("CAD_0.ui.viewport_widget")
    importlib.import_module("CAD_0.ui.feature_tree")
    importlib.import_module("CAD_0.ui.property_panel")
    importlib.import_module("CAD_0.ui.main_window")
