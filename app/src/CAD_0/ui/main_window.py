"""CAD_0.ui.main_window — Application main window.

Assembles the viewport + feature tree + property panel into a single
main window with menus and a toolbar.

Phase C.8: written but not exercised in CI. Requires PySide6.

Layout (left to right):
    +------------------+---------------------+
    | Feature tree     | Viewport            |
    | (250 px wide)    |                     |
    |                  |                     |
    +------------------+                     |
    | Property panel   |                     |
    | (auto height)    |                     |
    +------------------+---------------------+

Menu bar:
    File: New, Open, Save, Export STL/OBJ/3MF, Quit
    Edit: Undo, Redo
    View: Toggle grid, Toggle axes, Frame all
    Help: About

Toolbar: quick access to common actions.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QIcon, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QDockWidget,
    QFileDialog,
    QMainWindow,
    QMessageBox,
    QStatusBar,
    QToolBar,
    QWidget,
)

from .viewport_widget import ViewportWidget
from .feature_tree import FeatureTreePanel, FeatureTreeModel, FeatureNode
from .property_panel import PropertyPanel


class MainWindow(QMainWindow):
    """The CAD_0 application's main window."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("CAD_0 — Phase C MVP")
        self.resize(1280, 800)

        self._setup_central_widget()
        self._setup_dock_widgets()
        self._setup_menus()
        self._setup_toolbar()
        self._setup_statusbar()
        self._connect_signals()

    # ----- Setup -----

    def _setup_central_widget(self) -> None:
        self._viewport = ViewportWidget(self)
        self.setCentralWidget(self._viewport)

    def _setup_dock_widgets(self) -> None:
        # Left dock: feature tree on top, property panel on bottom.
        self._feature_tree = FeatureTreePanel(self)
        self._tree_dock = QDockWidget("Feature Tree", self)
        self._tree_dock.setWidget(self._feature_tree)
        self._tree_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self._tree_dock.setMinimumWidth(250)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._tree_dock)

        self._property_panel = PropertyPanel(self)
        self._props_dock = QDockWidget("Properties", self)
        self._props_dock.setWidget(self._property_panel)
        self._props_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._props_dock)

        # Stack the property panel below the feature tree.
        self.splitDockWidget(self._tree_dock, self._props_dock, Qt.Vertical)

    def _setup_menus(self) -> None:
        # ----- File menu -----
        file_menu = self.menuBar().addMenu("&File")

        new_action = QAction("&New", self)
        new_action.setShortcut(QKeySequence.New)
        new_action.triggered.connect(self.on_new)
        file_menu.addAction(new_action)

        open_action = QAction("&Open...", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.on_open)
        file_menu.addAction(open_action)

        file_menu.addSeparator()

        export_stl = QAction("Export &STL...", self)
        export_stl.triggered.connect(lambda: self._on_export("STL"))
        file_menu.addAction(export_stl)

        export_obj = QAction("Export &OBJ...", self)
        export_obj.triggered.connect(lambda: self._on_export("OBJ"))
        file_menu.addAction(export_obj)

        export_3mf = QAction("Export &3MF...", self)
        export_3mf.triggered.connect(lambda: self._on_export("3MF"))
        file_menu.addAction(export_3mf)

        file_menu.addSeparator()

        quit_action = QAction("&Quit", self)
        quit_action.setShortcut(QKeySequence.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # ----- Edit menu -----
        edit_menu = self.menuBar().addMenu("&Edit")

        self._undo_action = QAction("&Undo", self)
        self._undo_action.setShortcut(QKeySequence.Undo)
        self._undo_action.triggered.connect(self.on_undo)
        edit_menu.addAction(self._undo_action)

        self._redo_action = QAction("&Redo", self)
        self._redo_action.setShortcut(QKeySequence.Redo)
        self._redo_action.triggered.connect(self.on_redo)
        edit_menu.addAction(self._redo_action)

        # ----- View menu -----
        view_menu = self.menuBar().addMenu("&View")

        self._toggle_grid = QAction("Show &Grid", self, checkable=True)
        self._toggle_grid.setChecked(True)
        self._toggle_grid.triggered.connect(self.on_toggle_grid)
        view_menu.addAction(self._toggle_grid)

        self._toggle_axes = QAction("Show &Axes", self, checkable=True)
        self._toggle_axes.setChecked(True)
        self._toggle_axes.triggered.connect(self.on_toggle_axes)
        view_menu.addAction(self._toggle_axes)

        frame_all = QAction("&Frame All", self)
        frame_all.setShortcut(QKeySequence("F"))
        frame_all.triggered.connect(self.on_frame_all)
        view_menu.addAction(frame_all)

        # ----- Help menu -----
        help_menu = self.menuBar().addMenu("&Help")
        about = QAction("&About CAD_0", self)
        about.triggered.connect(self.on_about)
        help_menu.addAction(about)

    def _setup_toolbar(self) -> None:
        toolbar = QToolBar("Main", self)
        toolbar.setMovable(False)
        toolbar.addAction(self._undo_action)
        toolbar.addAction(self._redo_action)
        toolbar.addSeparator()
        toolbar.addAction(self._toggle_grid)
        toolbar.addAction(self._toggle_axes)
        self.addToolBar(toolbar)

    def _setup_statusbar(self) -> None:
        self._status = QStatusBar(self)
        self.setStatusBar(self._status)
        self._status.showMessage("Ready")

    def _connect_signals(self) -> None:
        # Selection in the feature tree → update the property panel.
        selection_model = self._feature_tree.selectionModel()
        if selection_model is not None:
            selection_model.currentChanged.connect(self._on_tree_selection_changed)

        # Property panel edits → propagate to the model (and the renderer).
        self._property_panel.parameter_changed.connect(self._on_parameter_changed)

    # ----- Slots -----

    def on_new(self) -> None:
        """Reset to a fresh scene."""
        self._feature_tree.setModel(FeatureTreeModel(FeatureNode(name="<root>")))
        self._property_panel.clear()
        self._viewport.frame_all()
        self._status.showMessage("New project", 3000)

    def on_open(self) -> None:
        """Open a CAD_0 project file (.CAD_0)."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open CAD_0 Project", "",
            "CAD_0 Projects (*.CAD_0);;All Files (*)"
        )
        if not path:
            return
        # Phase H will implement the actual deserialization.
        self._status.showMessage(f"Opened: {path}", 3000)

    def _on_export(self, format_name: str) -> None:
        """Export the current scene to a mesh file."""
        extensions = {
            "STL": "STL Mesh (*.stl)",
            "OBJ": "Wavefront OBJ (*.obj)",
            "3MF": "3D Manufacturing Format (*.3mf)",
        }
        path, _ = QFileDialog.getSaveFileName(
            self, f"Export {format_name}", "",
            f"{extensions[format_name]};;All Files (*)"
        )
        if not path:
            return
        # Phase B.5's export functions will be called here once the
        # binding is wired up. For now we just report the action.
        self._status.showMessage(f"Exported: {path}", 3000)

    def on_undo(self) -> None:
        """Undo the last command."""
        # The CommandStack (core/viewport/command_stack.hpp) is bound
        # to Python via nanobind. For Phase C we just report.
        self._status.showMessage("Undo", 1500)

    def on_redo(self) -> None:
        """Redo the last undone command."""
        self._status.showMessage("Redo", 1500)

    def on_toggle_grid(self, checked: bool) -> None:
        self._status.showMessage(f"Grid: {'on' if checked else 'off'}", 1500)

    def on_toggle_axes(self, checked: bool) -> None:
        self._status.showMessage(f"Axes: {'on' if checked else 'off'}", 1500)

    def on_frame_all(self) -> None:
        self._viewport.frame_all()
        self._status.showMessage("Framed all", 1500)

    def on_about(self) -> None:
        QMessageBox.about(
            self,
            "About CAD_0",
            "<h3>CAD_0</h3>"
            "<p>Dual-representation CAD kernel — native SDF + B-Rep.</p>"
            "<p>Phase C MVP — viewport, feature tree, property panel.</p>"
            "<p>License: Apache-2.0</p>"
        )

    def _on_tree_selection_changed(self, current, previous) -> None:
        """Update the property panel when the tree selection changes."""
        if not current.isValid():
            self._property_panel.clear()
            return
        node = current.data(Qt.UserRole)
        if node is None:
            self._property_panel.clear()
            return
        self._property_panel.set_parameters(node.name, node.parameters)

    def _on_parameter_changed(self, name: str, value) -> None:
        """A parameter was edited in the property panel."""
        self._status.showMessage(f"Set {name} = {value}", 1500)


def main() -> None:
    """Application entry point."""
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName("CAD_0")
    app.setOrganizationName("CAD_0")

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
