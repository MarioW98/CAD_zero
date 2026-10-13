"""CAD_0.ui.main_window — Application main window with working scene creation.

Now connects to the C++ Scene kernel and renders meshes in the viewport.
"""

from __future__ import annotations

import sys
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
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
from .property_panel import PropertyPanel

# Simple feature tree (no QAbstractItemModel for now — just QTreeWidget)
from PySide6.QtWidgets import QTreeWidget, QTreeWidgetItem, QAbstractItemView


class MainWindow(QMainWindow):
    """The CAD_0 application's main window."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("CAD_0 — Phase C+ Desktop")
        # Startup geometry policy: NO fixed size. The old resize(1280, 800)
        # matched the Xvfb/Codespace screen exactly, so once fluxbox added
        # its title bar + borders the top of the window (title bar AND menu
        # bar) landed off-screen. main() now shows the window maximized,
        # which fills the WM's available geometry on every screen. The
        # fallback below only covers the edge case of a platform that
        # reports no usable screen at all (e.g. offscreen QPA without a
        # screen): such a window is deliberately SMALLER than any common
        # screen — never exactly screen-sized, which is how the original
        # bug was born.
        screen = self.screen()
        if screen is None or not screen.availableGeometry().isValid():
            self.resize(1200, 720)

        # Try to import the C++ kernel
        try:
            import CAD_0
            # Check if this package has the compiled extension
            if not hasattr(CAD_0, '_CAD_0'):
                # Try to find _CAD_0 on sys.path
                import importlib.util
                spec = importlib.util.find_spec('_CAD_0')
                if spec is not None:
                    import importlib
                    _CAD_0_mod = importlib.import_module('_CAD_0')
                    # Inject submodules
                    CAD_0._CAD_0 = _CAD_0_mod
                    from CAD_0._CAD_0 import scene, camera, commands
                    CAD_0.scene = scene
                    CAD_0.camera = camera
                    CAD_0.commands = commands
                else:
                    raise ImportError("_CAD_0 compiled extension not found on sys.path")

            self._cpp = CAD_0
            self._scene = CAD_0.scene.Scene()
            self._command_stack = CAD_0.commands.CommandStack(1000)
            self._has_kernel = True
        except Exception as e:
            self._cpp = None
            self._scene = None
            self._command_stack = None
            self._has_kernel = False
            print(f"Warning: CAD_0 kernel not available: {e}")
            print("  Set PYTHONPATH to include the build/python directory.")

        # Track shape parameters and types for interactive editing.
        self._node_params = {}  # node_id → {param_name: value}
        self._node_types = {}   # node_id → "sphere"|"box"|"cylinder"|"torus"

        # Project persistence (native .CAD_0 format, ADR-0007).
        self._current_path: Optional[str] = None  # loaded/saved project path
        self._dirty = False                        # unsaved-changes flag
        self._loading = False                      # suppress dirty during load

        # Dirty tracking via SceneChange notifications: every scene mutation
        # (add/remove/param/visibility/clear) fires the C++ callback and
        # flags the project as modified. The callback is detached in
        # closeEvent() to break the Python↔C++ reference cycle.
        if self._has_kernel:
            self._scene.set_change_callback(self._on_scene_change)

        self._setup_central_widget()
        self._setup_dock_widgets()
        self._setup_menus()
        # BUG 1 fix (invisible menubar on non-native WMs): right after the
        # menus are built, force the bar to live INSIDE the window. On some
        # Linux desktops Qt delegates the menu bar to the platform's global
        # menu (a DBus appmenu registrar). Where no registrar exists —
        # Xvfb + fluxbox inside Codespaces is the reported case — the bar
        # is never drawn anywhere, yet its actions and shortcuts still
        # work (Ctrl+O opens files), which is exactly the reported
        # symptom. setNativeMenuBar(False) keeps the bar in-window on
        # every platform; desktops that do have a global menu simply opt
        # out of it (the standard cross-distribution fix). No layout
        # workaround involved: the toolbar and central widget are
        # untouched.
        self.menuBar().setNativeMenuBar(False)
        self.menuBar().setVisible(True)
        self._setup_toolbar()
        self._setup_statusbar()
        self._connect_signals()

        # Connect viewport click for picking
        self._viewport.clicked.connect(self._on_viewport_click)

        if not self._has_kernel:
            QMessageBox.warning(self, "Kernel not found",
                "Could not import the CAD_0 C++ kernel.\n"
                "Run with PYTHONPATH pointing to the build/python directory.")

    # ----- Setup -----

    def _setup_central_widget(self) -> None:
        self._viewport = ViewportWidget(self)
        self.setCentralWidget(self._viewport)
        if self._scene is not None:
            self._viewport.set_scene(self._scene)

    def _setup_dock_widgets(self) -> None:
        # Simple QTreeWidget for the feature tree
        self._feature_tree = QTreeWidget(self)
        self._feature_tree.setHeaderLabels(["Name", "Type"])
        self._feature_tree.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._feature_tree.setSelectionMode(QAbstractItemView.SingleSelection)

        self._tree_dock = QDockWidget("Feature Tree", self)
        self._tree_dock.setWidget(self._feature_tree)
        self._tree_dock.setMinimumWidth(250)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._tree_dock)

        self._property_panel = PropertyPanel(self)
        self._props_dock = QDockWidget("Properties", self)
        self._props_dock.setWidget(self._property_panel)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._props_dock)
        self.splitDockWidget(self._tree_dock, self._props_dock, Qt.Vertical)

    def _setup_menus(self) -> None:
        # ----- File menu -----
        file_menu = self.menuBar().addMenu("&File")

        new_action = QAction("&New Scene", self)
        new_action.setShortcut(QKeySequence.New)
        new_action.triggered.connect(self.on_new)
        file_menu.addAction(new_action)

        open_action = QAction("&Open...", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.on_open)
        file_menu.addAction(open_action)

        save_action = QAction("&Save", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self.on_save)
        file_menu.addAction(save_action)

        save_as_action = QAction("Save &As...", self)
        save_as_action.setShortcut(QKeySequence("Ctrl+Shift+S"))
        save_as_action.triggered.connect(self.on_save_as)
        file_menu.addAction(save_as_action)

        file_menu.addSeparator()

        import_stl = QAction("Import &STL...", self)
        import_stl.triggered.connect(lambda checked: self._on_import("stl"))
        file_menu.addAction(import_stl)

        import_obj = QAction("Import &OBJ...", self)
        import_obj.triggered.connect(lambda checked: self._on_import("obj"))
        file_menu.addAction(import_obj)

        file_menu.addSeparator()

        export_stl = QAction("Export &STL...", self)
        export_stl.triggered.connect(lambda: self._on_export("stl"))
        file_menu.addAction(export_stl)

        export_obj = QAction("Export &OBJ...", self)
        export_obj.triggered.connect(lambda: self._on_export("obj"))
        file_menu.addAction(export_obj)

        export_3mf = QAction("Export &3MF...", self)
        export_3mf.triggered.connect(lambda: self._on_export("3mf"))
        file_menu.addAction(export_3mf)

        file_menu.addSeparator()

        quit_action = QAction("&Quit", self)
        quit_action.setShortcut(QKeySequence.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # ----- Create menu -----
        # Registry-driven: the actions come from the kernel's builder
        # registry (CAD_0.scene.builder_specs — single source of truth),
        # so adding a builder in C++ automatically shows up here. Without
        # the kernel the menu stays empty (a warning is already shown).
        create_menu = self.menuBar().addMenu("&Create")
        if self._has_kernel:
            import CAD_0
            for builder_spec in CAD_0.scene.builder_specs():
                tag = builder_spec["tag"]
                action = QAction(f"Create &{builder_spec['display_name']}", self)
                action.triggered.connect(
                    lambda checked, t=tag: self._create_registered(t))
                create_menu.addAction(action)

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

        edit_menu.addSeparator()

        delete_action = QAction("&Delete Selected", self)
        delete_action.setShortcut(QKeySequence.Delete)
        delete_action.triggered.connect(self.on_delete_selected)
        edit_menu.addAction(delete_action)

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
        self._status.showMessage("Ready — try Create → Sphere")

    def _connect_signals(self) -> None:
        self._feature_tree.itemSelectionChanged.connect(self._on_tree_selection_changed)
        self._property_panel.parameter_changed.connect(self._on_parameter_changed)

    # ----- Slots -----

    def on_new(self) -> None:
        if not self._confirm_discard():
            return
        if self._scene is not None:
            self._scene.clear()
        self._node_params.clear()
        self._node_types.clear()
        self._feature_tree.clear()
        self._property_panel.clear()
        self._current_path = None
        self._dirty = False
        self._update_window_title()
        self._refresh_viewport()
        self._status.showMessage("New scene", 3000)

    # ----- Project persistence (native .CAD_0 format, ADR-0007) -------------

    def _on_scene_change(self, change, node_id) -> None:
        """SceneChange callback — flags the project as modified."""
        if self._loading:
            return
        self._mark_modified()

    def _mark_modified(self) -> None:
        if not self._dirty:
            self._dirty = True
            self._update_window_title()

    def _update_window_title(self) -> None:
        title = "CAD_0 — Phase C+ Desktop"
        if self._current_path:
            from pathlib import Path as _P
            name = _P(self._current_path).name
        else:
            name = "Untitled"
        self.setWindowTitle(f"{name}{' *' if self._dirty else ''} — {title}")

    def _confirm_discard(self) -> bool:
        """Ask the user what to do with unsaved changes. False = cancel."""
        if not self._dirty:
            return True
        ret = QMessageBox.question(
            self, "Unsaved changes",
            "The project has unsaved changes.\nSave them before continuing?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save)
        if ret == QMessageBox.Save:
            self.on_save()
            return not self._dirty  # save was cancelled/failed
        return ret == QMessageBox.Discard

    def on_open(self) -> None:
        """Open a .CAD_0 project (transactional: the current scene is only
        replaced after the file parses cleanly)."""
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Project", "",
            "CAD_0 Project (*.CAD_0);;All Files (*)")
        if not path:
            return
        import CAD_0
        import CAD_0.io as cad_io
        # Transactional load: parse into a fresh scene; only on success do
        # we swap it in. A corrupted/incompatible file raises RuntimeError
        # with an explicit message and leaves everything untouched.
        temp_scene = CAD_0.scene.Scene()
        try:
            cam = cad_io.load(path, temp_scene)
        except (RuntimeError, ValueError, OSError) as exc:
            QMessageBox.critical(
                self, "Cannot open project",
                f"Failed to open:\n{path}\n\n{exc}")
            return
        # Commit: swap scene + viewport, reset undo history (commands from
        # the previous document are meaningless now), rebuild the UI.
        if self._scene is not None:
            self._scene.set_change_callback(None)  # detach old callback
        self._scene = temp_scene
        self._scene.set_change_callback(self._on_scene_change)
        self._command_stack = CAD_0.commands.CommandStack(1000)
        self._viewport.set_scene(self._scene)
        self._rebuild_ui_from_scene()
        if cam:
            self._viewport.apply_camera_state(cam)
        self._current_path = path
        self._dirty = False
        self._update_window_title()
        self._status.showMessage(f"Opened {path} ({self._scene.size()} shapes)", 5000)

    def on_save(self) -> None:
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        if not self._current_path:
            self.on_save_as()
            return
        self._do_save(self._current_path)

    def on_save_as(self) -> None:
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Project As", self._current_path or "untitled.CAD_0",
            "CAD_0 Project (*.CAD_0);;All Files (*)")
        if not path:
            return
        if not path.lower().endswith(".cad_0"):
            path += ".CAD_0"
        self._current_path = path
        self._do_save(path)

    def _do_save(self, path: str) -> None:
        import CAD_0.io as cad_io
        try:
            # Atomic: C++ writes <path>.tmp then renames — a crash never
            # destroys the previous file. Camera state is persisted too.
            cad_io.save(path, self._scene, self._viewport.camera_state())
        except (RuntimeError, OSError) as exc:
            QMessageBox.critical(
                self, "Cannot save project",
                f"Failed to save:\n{path}\n\n{exc}")
            return
        self._dirty = False
        self._update_window_title()
        self._status.showMessage(f"Saved {path}", 4000)

    def _rebuild_ui_from_scene(self) -> None:
        """Rebuild feature tree + editing metadata from the (new) scene."""
        self._node_params.clear()
        self._node_types.clear()
        self._feature_tree.clear()
        self._property_panel.clear()
        self._selected_node_id = None
        if self._scene is None:
            return
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None:
                continue
            type_label = "Mesh" if node.kind == "mesh" else "SDF"
            item = QTreeWidgetItem([node.name, type_label])
            item.setData(0, Qt.UserRole, node.id)
            self._feature_tree.addTopLevelItem(item)
            if node.kind == "mesh":
                # Imported geometry: no editing metadata, no defaults.
                continue
            # Editing metadata survives the round-trip via the C++ node's
            # `builder` tag + `params` map (persisted by the native format).
            if node.builder:
                self._node_types[node.id] = node.builder
                params = dict(node.params)
                if params:
                    self._node_params[node.id] = params
                else:
                    self._make_default_params(node.builder, node.id)
            else:
                self._make_default_params(self._node_type_from_body(node), node.id)
        self._refresh_viewport()

    def _make_default_params(self, shape_type: Optional[str], node_id: int) -> None:
        """Populate default params for a node we know how to edit.

        The parameter table lives in the kernel's builder registry
        (CAD_0.scene.default_params — the single source of truth; the UI
        keeps NO hardcoded copy). An unknown builder tag degrades to a
        non-parametric node, mirroring the kernel's tolerant load
        contract (ADR-0007 PATCH compatibility).
        """
        import CAD_0
        if not shape_type:
            return
        self._node_types[node_id] = shape_type
        try:
            self._node_params[node_id] = dict(
                CAD_0.scene.default_params(shape_type))
        except RuntimeError:
            pass  # unknown builder tag: loadable, just not parametric

    def _node_type_from_body(self, node) -> Optional[str]:
        """Best-effort shape type for a node without a builder tag."""
        if node.builder:
            return node.builder
        return None

    def closeEvent(self, event) -> None:
        """Prompt for unsaved changes, then detach the scene callback."""
        if self._dirty:
            ret = QMessageBox.question(
                self, "Unsaved changes",
                "The project has unsaved changes.\nSave before closing?",
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
                QMessageBox.Save)
            if ret == QMessageBox.Save:
                self.on_save()
                if self._dirty:  # save was cancelled
                    event.ignore()
                    return
            elif ret == QMessageBox.Cancel:
                event.ignore()
                return
        # Break the Python↔C++ reference cycle (scene holds our bound
        # method; we hold the scene).
        if self._scene is not None:
            try:
                self._scene.set_change_callback(None)
            except RuntimeError:
                pass
        event.accept()

    def _add_shape_to_scene(self, name: str, body) -> None:
        """Add an SDF body to the scene and update the UI."""
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        node_id = self._scene.add_node(name, body)
        # Add to tree widget
        item = QTreeWidgetItem([name, "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        self._status.showMessage(f"Created {name} (id={node_id})", 3000)

    def _refresh_viewport(self) -> None:
        """Re-tessellate and redraw."""
        self._viewport._refresh_meshes()

    def _make_body(self, shape_type: str, params: dict):
        """Build an SDF body from a shape_type + params dict.

        Delegates to the kernel's builder registry (CAD_0.scene.make_body
        — the single source of truth). Unknown tags raise RuntimeError
        with an explicit message naming the tag.
        """
        import CAD_0
        return CAD_0.scene.make_body(
            shape_type, {k: float(v) for k, v in params.items()})

    def _shape_display_name(self, shape_type: str) -> str:
        """UI label for a builder tag, from the kernel registry."""
        import CAD_0
        for builder_spec in CAD_0.scene.builder_specs():
            if builder_spec["tag"] == shape_type:
                return builder_spec["display_name"]
        return shape_type

    def _create_registered(self, shape_type: str) -> None:
        """Create a shape from the kernel registry's default params."""
        import CAD_0
        if not self._has_kernel:
            return
        params = dict(CAD_0.scene.default_params(shape_type))
        self._push_create_command(shape_type, params)
        self._status.showMessage(
            f"Created {self._shape_display_name(shape_type)}", 3000)

    def _push_create_command(self, shape_type: str, params: dict) -> None:
        """Push a fully-functional undoable 'create shape' command.

        The command stack's push() calls execute() immediately, so we let
        execute() be the action that creates the node, and undo() removes it.
        A mutable holder tracks the node_id (which may differ between
        do/redo cycles because Scene assigns new IDs).
        """
        name = self._shape_display_name(shape_type)
        state = {"node_id": None}

        def execute():
            body = self._make_body(shape_type, params)
            nid = self._scene.add_node(name, body)
            # Persist the editing metadata on the C++ node too, so the
            # native save format round-trips it (ADR-0007 builder tag).
            node = self._scene.get_node(nid)
            if node is not None:
                node.builder = shape_type
                node.set_params({k: float(v) for k, v in params.items()})
            self._node_params[nid] = dict(params)
            self._node_types[nid] = shape_type
            item = QTreeWidgetItem([name, "SDF"])
            item.setData(0, Qt.UserRole, nid)
            self._feature_tree.addTopLevelItem(item)
            state["node_id"] = nid
            self._refresh_viewport()

        def undo():
            nid = state["node_id"]
            if nid is not None:
                # Remove from scene + side dicts + tree widget
                if self._scene and self._scene.remove_node(nid):
                    self._node_params.pop(nid, None)
                    self._node_types.pop(nid, None)
                    for i in range(self._feature_tree.topLevelItemCount()):
                        it = self._feature_tree.topLevelItem(i)
                        if it.data(0, Qt.UserRole) == nid:
                            self._feature_tree.takeTopLevelItem(i)
                            break
                    self._property_panel.clear()
                    state["node_id"] = None
                    self._refresh_viewport()

        if self._command_stack:
            self._command_stack.push(execute, undo, f"Create {name}")
        else:
            execute()

    def on_create_sphere(self) -> None:
        self._create_registered("sphere")

    def on_create_box(self) -> None:
        self._create_registered("box")

    def on_create_cylinder(self) -> None:
        self._create_registered("cylinder")

    def on_create_torus(self) -> None:
        self._create_registered("torus")

    def _rebuild_node(self, node_id: int) -> None:
        """Rebuild an SDF body from its stored params and type.

        The body construction is delegated to the kernel builder registry
        (single source of truth — see _make_body). Unknown tags leave the
        node untouched, mirroring the tolerant load contract.
        """
        if node_id not in self._node_params:
            return
        params = self._node_params[node_id]
        shape_type = self._node_types.get(node_id, "")
        try:
            body = self._make_body(shape_type, params)
        except RuntimeError:
            return  # unknown builder tag: not parametrically editable
        node = self._scene.get_node(node_id)
        if node:
            node.body = body
            node.mesh_dirty = True
            # Keep the C++ node's params in sync so the native save format
            # captures the edited values (round-trips through save/load).
            if shape_type:
                node.builder = shape_type
                node.set_params(
                    {k: float(v) for k, v in params.items()})
            self._refresh_viewport()

    def _on_import(self, format_name: str) -> None:
        """Import an STL/OBJ file as a first-class mesh scene node.

        The kernel loader (CAD_0.io.load_stl / load_obj) welds the
        vertices; the status bar reports the post-welding counts. The
        node is NOT undoable (importing copies potentially megabytes of
        geometry into an undo command; the command stack is reset on
        open anyway) — delete re-adds nothing parametric for meshes.
        """
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        filters = {
            "stl": "STL Mesh (*.stl);;All Files (*)",
            "obj": "Wavefront OBJ (*.obj);;All Files (*)",
        }
        path, _ = QFileDialog.getOpenFileName(
            self, f"Import {format_name.upper()}", "", filters[format_name])
        if not path:
            return
        self._import_mesh(path, format_name)

    def _import_mesh(self, path: str, format_name: str) -> bool:
        """Load `path` and add it as a mesh node (dialog-free core —
        also the test entry point). Returns True on success."""
        import CAD_0.io as cad_io
        from pathlib import Path as _Path
        try:
            loader = cad_io.load_stl if format_name == "stl" else cad_io.load_obj
            mesh = loader(path)
        except (RuntimeError, ValueError, OSError) as exc:
            QMessageBox.critical(
                self, "Cannot import file",
                f"Failed to import:\n{path}\n\n{exc}")
            return False
        # Node name from the filename (stem); ids stay numeric.
        name = _Path(path).stem or "imported"
        node_id = self._scene.add_node(name, mesh)
        node = self._scene.get_node(node_id)
        if node is None:  # defensive: add_node just returned it
            return False
        vcount = node.mesh.vertex_count()
        tcount = node.mesh.triangle_count()
        item = QTreeWidgetItem([name, "Mesh"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        # Post-welding counts (the loader welds: an STL's per-triangle
        # duplicates are merged back before the mesh lands here).
        self._status.showMessage(
            f"Imported {name}: {vcount} vertices, {tcount} triangles (welded)",
            5000)
        return True

    def _on_export(self, format_name: str) -> None:
        extensions = {
            "stl": "STL Mesh (*.stl)",
            "obj": "Wavefront OBJ (*.obj)",
            "3mf": "3D Manufacturing Format (*.3mf)",
        }
        path, _ = QFileDialog.getSaveFileName(
            self, f"Export {format_name.upper()}", "",
            f"{extensions[format_name]};;All Files (*)"
        )
        if not path or not self._has_kernel:
            return
        # Ensure the chosen format's extension is present — Qt does not
        # append it on every platform, and CAD_0.io.export() infers the
        # format from the extension.
        from pathlib import Path as _Path
        if _Path(path).suffix.lower() not in (".stl", ".obj", ".3mf"):
            path = f"{path}.{format_name}"
        # Import the io wrapper module explicitly (Bug fix: `CAD_0.io` is a
        # lazily-imported Python wrapper; without this, `self._cpp.io`
        # raised AttributeError).
        try:
            import CAD_0.io as cad_io
        except ImportError:
            self._status.showMessage("Export failed: CAD_0.io not importable", 3000)
            return
        # Export the first visible mesh with actual geometry.
        # (Bug fix: the original condition was inverted — it skipped valid
        # meshes and only tried to export null/hidden/empty nodes, so
        # every export ended in "No mesh to export".)
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None or not node.visible:
                continue
            if node.cached_mesh.vertex_count() == 0:
                continue
            cad_io.export(path, node.cached_mesh)
            self._status.showMessage(f"Exported: {path}", 3000)
            return
        self._status.showMessage("No mesh to export", 3000)

    def on_toggle_grid(self, checked: bool) -> None:
        self._viewport.set_show_grid(checked)

    def on_toggle_axes(self, checked: bool) -> None:
        self._viewport.set_show_axes(checked)

    def on_frame_all(self) -> None:
        self._viewport.frame_all()
        self._status.showMessage("Framed all", 1500)

    def on_undo(self) -> None:
        if self._command_stack and self._command_stack.can_undo():
            desc = self._command_stack.next_undo_description()
            self._command_stack.undo()
            self._refresh_viewport()
            self._status.showMessage(f"Undo: {desc}", 2000)
        else:
            self._status.showMessage("Nothing to undo", 1500)

    def on_redo(self) -> None:
        if self._command_stack and self._command_stack.can_redo():
            desc = self._command_stack.next_redo_description()
            self._command_stack.redo()
            self._refresh_viewport()
            self._status.showMessage(f"Redo: {desc}", 2000)
        else:
            self._status.showMessage("Nothing to redo", 1500)

    def on_delete_selected(self) -> None:
        items = self._feature_tree.selectedItems()
        if not items:
            return
        item = items[0]
        node_id = item.data(0, Qt.UserRole)
        if node_id is None:
            return
        # Snapshot the node so we can re-create it on redo-after-undo
        shape_type = self._node_types.get(node_id, "")
        params = dict(self._node_params.get(node_id, {}))
        name = self._shape_display_name(shape_type)
        state = {"node_id": node_id, "tree_item_idx": None}
        # Capture the tree-widget index so undo can reinsert at the same slot
        for i in range(self._feature_tree.topLevelItemCount()):
            if self._feature_tree.topLevelItem(i) is item:
                state["tree_item_idx"] = i
                break

        def execute():
            nid = state["node_id"]
            if nid is None:
                return
            if self._scene and self._scene.remove_node(nid):
                self._node_params.pop(nid, None)
                self._node_types.pop(nid, None)
                for i in range(self._feature_tree.topLevelItemCount()):
                    it = self._feature_tree.topLevelItem(i)
                    if it.data(0, Qt.UserRole) == nid:
                        self._feature_tree.takeTopLevelItem(i)
                        break
                self._property_panel.clear()
                state["node_id"] = None
                self._refresh_viewport()

        def undo():
            # Re-create the node with its original shape/params
            body = self._make_body(shape_type, params)
            new_id = self._scene.add_node(name, body)
            self._node_params[new_id] = dict(params)
            self._node_types[new_id] = shape_type
            new_item = QTreeWidgetItem([name, "SDF"])
            new_item.setData(0, Qt.UserRole, new_id)
            idx = state.get("tree_item_idx")
            if idx is not None and 0 <= idx <= self._feature_tree.topLevelItemCount():
                self._feature_tree.insertTopLevelItem(idx, new_item)
            else:
                self._feature_tree.addTopLevelItem(new_item)
            state["node_id"] = new_id
            self._refresh_viewport()

        if self._command_stack:
            self._command_stack.push(execute, undo, f"Delete {name}")
        else:
            execute()
        self._status.showMessage(f"Deleted {name}", 2000)

    def _do_remove_node(self, nnid: int) -> None:
        """Undo callback: remove a node (used by Create undo)."""
        if self._scene and self._scene.remove_node(nnid):
            self._node_params.pop(nnid, None)
            self._node_types.pop(nnid, None)
            # Remove from tree widget
            for i in range(self._feature_tree.topLevelItemCount()):
                item = self._feature_tree.topLevelItem(i)
                if item.data(0, Qt.UserRole) == nnid:
                    self._feature_tree.takeTopLevelItem(i)
                    break
            self._property_panel.clear()
            self._refresh_viewport()

    def _do_re_add_node(self, node_id: int, shape_type: str, params: dict) -> None:
        """Undo callback: re-add a deleted node.

        Body construction + display name come from the kernel builder
        registry (single source of truth — see _make_body). An unknown
        builder tag cannot be re-created: the undo is a no-op.
        """
        try:
            body = self._make_body(shape_type, params)
        except RuntimeError:
            return  # unknown builder tag: nothing to re-create
        name = self._shape_display_name(shape_type)
        # Re-add with the same ID (best effort — Scene assigns new IDs)
        new_id = self._scene.add_node(name, body)
        self._node_params[new_id] = params
        self._node_types[new_id] = shape_type
        item = QTreeWidgetItem([name, "SDF"])
        item.setData(0, Qt.UserRole, new_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()

    def on_about(self) -> None:
        QMessageBox.about(
            self,
            "About CAD_0",
            "<h3>CAD_0</h3>"
            "<p>Dual-representation CAD kernel — native SDF + B-Rep.</p>"
            "<p>Phase C+ — working viewport with real mesh rendering.</p>"
            "<p>License: Apache-2.0</p>"
        )

    def _on_tree_selection_changed(self) -> None:
        items = self._feature_tree.selectedItems()
        if not items:
            self._property_panel.clear()
            return
        item = items[0]
        node_id = item.data(0, Qt.UserRole)
        if node_id is None:
            return
        node = self._scene.get_node(node_id)
        if node is None:
            return
        self._selected_node_id = node_id
        if node.kind == "mesh":
            # Imported geometry: read-only info (nothing parametric to
            # edit — the mesh is data, not a recipe).
            self._property_panel.set_readonly_fields(
                node.name, self._mesh_info_fields(node))
            return
        # Show the node's parameters in the property panel.
        params = self._node_params.get(node_id, {})
        # Convert to dict[str, float|str|bool] for PropertyPanel
        display_params = {}
        for k, v in params.items():
            display_params[k] = float(v) if isinstance(v, (int, float)) else str(v)
        self._property_panel.set_parameters(node.name, display_params)

    @staticmethod
    def _mesh_info_fields(node) -> dict:
        """Read-only property-panel fields for a mesh-kind node."""
        mesh = node.mesh
        fields = {
            "Type": "Imported mesh",
            "Vertices": str(mesh.vertex_count()),
            "Triangles": str(mesh.triangle_count()),
        }
        try:
            import numpy as _np
            pos = _np.asarray(mesh.positions)
            if pos.size:
                lo, hi = pos.min(axis=0), pos.max(axis=0)
                fields["Bounds min"] = f"({lo[0]:.3f}, {lo[1]:.3f}, {lo[2]:.3f})"
                fields["Bounds max"] = f"({hi[0]:.3f}, {hi[1]:.3f}, {hi[2]:.3f})"
        except (ImportError, ValueError):
            pass  # numpy missing / empty positions: counts still shown
        return fields

    def _on_parameter_changed(self, name: str, value) -> None:
        """Called when the user edits a parameter in the property panel."""
        node_id = getattr(self, '_selected_node_id', None)
        if node_id is None or node_id not in self._node_params:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        old_value = self._node_params[node_id].get(name, 0.0)

        def execute():
            self._node_params[node_id][name] = value
            self._rebuild_node(node_id)
            # node.body assignment bypasses Scene-level notifications, so
            # the dirty flag is marked here explicitly.
            self._mark_modified()

        def undo():
            self._node_params[node_id][name] = old_value
            self._rebuild_node(node_id)

        if self._command_stack:
            self._command_stack.push(execute, undo, f"Set {name} = {value}")
        else:
            execute()
        self._status.showMessage(f"Set {name} = {value}", 2000)

    def _do_restore_param(self, node_id: int, name: str, old_value: float) -> None:
        """Legacy undo callback — kept for compatibility, unused after refactor."""
        if node_id in self._node_params:
            self._node_params[node_id][name] = old_value
            self._rebuild_node(node_id)

    def _on_viewport_click(self, ndc_x: float, ndc_y: float) -> None:
        """Picking: select the closest visible node in the viewport."""
        if not self._scene or self._scene.empty():
            return
        # Simple picking: find the node whose bounding box is closest
        # to the ray from the camera through the clicked pixel.
        import math
        # Camera position from spherical coords
        cp = math.cos(self._viewport._cam_pitch)
        sp = math.sin(self._viewport._cam_pitch)
        cy = math.cos(self._viewport._cam_yaw)
        sy = math.sin(self._viewport._cam_yaw)
        cam_dir = (cp * sy, sp, cp * cy)
        tx, ty, tz = self._viewport._cam_target
        cam_pos = (tx + cam_dir[0] * self._viewport._cam_distance,
                   ty + cam_dir[1] * self._viewport._cam_distance,
                   tz + cam_dir[2] * self._viewport._cam_distance)

        # Build ray direction from NDC (simplified — uses camera forward/right/up)
        fov = self._viewport._cam_fov
        aspect = self._viewport._cam_aspect
        tan_half = math.tan(math.radians(fov) * 0.5)
        # Forward = target - cam_pos (normalized)
        fx, fy, fz = tx - cam_pos[0], ty - cam_pos[1], tz - cam_pos[2]
        fl = math.sqrt(fx*fx + fy*fy + fz*fz)
        if fl < 1e-9:
            return
        fx, fy, fz = fx/fl, fy/fl, fz/fl
        # Right = forward x (0,1,0)
        rx, ry, rz = -fz, 0.0, fx
        rl = math.sqrt(rx*rx + rz*rz)
        if rl > 1e-9:
            rx, rz = rx/rl, rz/rl
        # Up = right x forward
        ux, uy, uz = ry*fz - rz*fy, rz*fx - rx*fz, rx*fy - ry*fx

        ray_dir = (
            fx + ndc_x * aspect * tan_half * rx + ndc_y * tan_half * ux,
            fy + ndc_x * aspect * tan_half * ry + ndc_y * tan_half * uy,
            fz + ndc_x * aspect * tan_half * rz + ndc_y * tan_half * uz,
        )
        rl = math.sqrt(ray_dir[0]**2 + ray_dir[1]**2 + ray_dir[2]**2)
        ray_dir = (ray_dir[0]/rl, ray_dir[1]/rl, ray_dir[2]/rl)

        # Ray-bbox intersection for each visible node
        best_node = None
        best_t = float('inf')
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None or not node.visible:
                continue
            # Mesh-kind nodes have no SDF body: their default bounds are
            # the empty box (+inf/-inf), whose slab test degenerates and
            # would swallow EVERY click. Picking stays an SDF-node
            # feature; mesh nodes are selected from the feature tree.
            if node.kind == "mesh":
                continue
            b = node.body.bounds()
            # Simple slab-based ray-AABB test
            tmin = -float('inf')
            tmax = float('inf')
            hit = True
            for i in range(3):
                o = [cam_pos[0], cam_pos[1], cam_pos[2]][i]
                d = ray_dir[i]
                bmin = [b.min.x, b.min.y, b.min.z][i]
                bmax = [b.max.x, b.max.y, b.max.z][i]
                if abs(d) < 1e-9:
                    if o < bmin or o > bmax:
                        hit = False
                        break
                else:
                    t1 = (bmin - o) / d
                    t2 = (bmax - o) / d
                    if t1 > t2:
                        t1, t2 = t2, t1
                    tmin = max(tmin, t1)
                    tmax = min(tmax, t2)
                    if tmin > tmax:
                        hit = False
                        break
            if hit and tmin < best_t and tmax > 0:
                best_t = tmin if tmin > 0 else 0
                best_node = node.id

        if best_node is not None:
            # Select this node in the tree
            for i in range(self._feature_tree.topLevelItemCount()):
                item = self._feature_tree.topLevelItem(i)
                if item.data(0, Qt.UserRole) == best_node:
                    self._feature_tree.setCurrentItem(item)
                    break
            self._status.showMessage(f"Selected node {best_node}", 2000)


def main() -> None:
    """Application entry point."""
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName("CAD_0")
    app.setOrganizationName("CAD_0")

    window = MainWindow()
    # Programmatic equivalent of clicking the maximize button: Qt asks the
    # window manager to maximize within the available geometry, so the
    # window fills the screen MINUS decorations/taskbar on every platform
    # (Xvfb 1280x800 + fluxbox, Codespaces, desktop Linux, Windows) with
    # no hardcoded dimensions. On a platform that ignores the request the
    # window simply keeps its layout size hint — never a screen-sized
    # fixed size — so the title/menu bars can never be pushed off-screen.
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
