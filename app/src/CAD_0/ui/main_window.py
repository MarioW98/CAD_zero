"""CAD_0.ui.main_window — Application main window with working scene creation.

Now connects to the C++ Scene kernel and renders meshes in the viewport.
"""

from __future__ import annotations

import sys
from typing import Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QDockWidget,
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .viewport_widget import ViewportWidget
from .property_panel import PropertyPanel
from .sketch_canvas import SketchCanvas, make_entity, params_from_entity

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

        # Currently selected node id (None = nothing selected).
        # Explicitly initialized here so the attribute exists from
        # construction on: readers use plain attribute access (no
        # defensive getattr), at any point of the widget's life. The
        # value is kept in sync by _on_tree_selection_changed() and
        # reset by _rebuild_ui_from_scene()/_clear_selection().
        self._selected_node_id: Optional[int] = None

        # Project persistence (native .CAD_0 format, ADR-0007).
        self._current_path: Optional[str] = None  # loaded/saved project path
        self._dirty = False                        # unsaved-changes flag
        self._loading = False                      # suppress dirty during load

        # Dirty tracking via SceneChange notifications: every scene mutation
        # (add/remove/param/visibility/clear) fires the C++ callback and
        # flags the project as modified. The callback is detached in
        # closeEvent() to break the Python↔C++ reference cycle.
        # KNOWN RESIDUAL (2026-10-16, verified by A/B experiment — see the
        # teardown note in bindings/python/tests/test_ui_navigation.py):
        # after a viewport click fires a change notification, the clicked
        # window's Scene (sometimes with its CommandStack) outlives
        # interpreter shutdown via a C++-side reference invisible to
        # Python's GC. Fixing it needs binding-level work (weak callback
        # storage or GC traversal) — deliberately out of the
        # consolidation sweep's scope.
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
        self._setup_sketch_toolbar()
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
        # The central area is a tab stack: the 3D viewport plus one
        # VIEW per open sketch canvas (ADR-0030 D5.6 — the canvas is a
        # view of the node, one per sketch at a time; closing a tab
        # commits NOTHING, every completed gesture already wrote).
        self._viewport = ViewportWidget(self)
        if self._scene is not None:
            self._viewport.set_scene(self._scene)
        self._tabs = QTabWidget(self)
        self._tabs.addTab(self._viewport, "3D View")
        self._tabs.setTabsClosable(True)
        # The 3D view is the document's face: its close request is
        # ignored (_on_tab_close_requested guards index 0 — PySide6
        # exposes no per-tab closable, and the tabButton() swap is
        # not bound); sketch tabs close their view.
        self._tabs.tabCloseRequested.connect(self._on_tab_close_requested)
        self.setCentralWidget(self._tabs)
        # node_id -> the open SketchCanvas view (one per sketch).
        self._sketch_views = {}
        # (sketch_id, chain, index) of the entity selected on a
        # canvas — the panel's entity context (None = node context).
        self._selected_entity = None

    def _setup_dock_widgets(self) -> None:
        # Simple QTreeWidget for the feature tree. ExtendedSelection
        # (ADR-0027 D5): the subtract entry point needs exactly two SDF
        # nodes selected; single-selection was the only selection-mode
        # change the slice required.
        self._feature_tree = QTreeWidget(self)
        self._feature_tree.setHeaderLabels(["Name", "Type"])
        self._feature_tree.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._feature_tree.setSelectionMode(QAbstractItemView.ExtendedSelection)

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
            create_menu.addSeparator()
            # A sketch is a node KIND, not a builder (ADR-0030 D2): its
            # own Create entry, opening the canvas view (D5.6).
            sketch_action = QAction("Create Ske&tch", self)
            sketch_action.triggered.connect(self.on_create_sketch)
            create_menu.addAction(sketch_action)

        # ----- Feature menu -----
        # The feature-creation commands: Extrude consumes a sketch and
        # produces the prism operation node (ADR-0030 D5.5/D5.6).
        feature_menu = self.menuBar().addMenu("&Feature")
        self._extrude_action = QAction("&Extrude...", self)
        self._extrude_action.triggered.connect(self.on_extrude)
        feature_menu.addAction(self._extrude_action)

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

        edit_menu.addSeparator()

        # ADR-0027 D5 — the boolean feature's entry point: exactly two
        # SDF-kind nodes selected → a minimal dialog (target/tool + the
        # tool placement). Operands stay in the tree, hidden from the
        # viewport; the operation row names them.
        self._subtract_action = QAction("Su&btract (boolean difference)", self)
        self._subtract_action.triggered.connect(self.on_subtract)
        edit_menu.addAction(self._subtract_action)

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

        view_menu.addSeparator()

        # Tessellation resolution (graphics sprint, optional item): a
        # VIEW setting like grid/axes — not a scene mutation, no undo
        # command, not persisted in .CAD_0. Default 48 (the resolution
        # every pinned count and golden in the repo assumes). Changing
        # it re-tessellates every Sdf node: coarser is faster to orbit,
        # finer smooths the display — and the exported mesh follows
        # (the tessellation cache IS the export surface in this
        # architecture; at the default 48 exports are byte-identical).
        self._tess_resolution = 48
        self._tess_actions = {}
        from PySide6.QtGui import QActionGroup
        tess_group = QActionGroup(self)
        tess_menu = view_menu.addMenu("Tessellation &Resolution")
        for res in (32, 48, 64, 96):
            act = QAction(f"&{res}", self, checkable=True)
            act.setActionGroup(tess_group)
            act.setChecked(res == self._tess_resolution)
            act.triggered.connect(
                lambda checked, r=res: self._set_tessellation_resolution(r))
            tess_menu.addAction(act)
            self._tess_actions[res] = act

        # ----- Debug menu -----
        # Read-only inspection tools (ADR-0026 consumer step): they
        # report on the current scene without touching it — no authoring,
        # no scene/document mutation, nothing on the undo stack.
        debug_menu = self.menuBar().addMenu("&Debug")
        sdf_to_brep_action = QAction("Extract SDF → B-Rep (topology report)", self)
        sdf_to_brep_action.triggered.connect(self.on_debug_sdf_to_brep_report)
        debug_menu.addAction(sdf_to_brep_action)

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

    def _setup_sketch_toolbar(self) -> None:
        """The sketch tool selector — the wiring that REACHES what the
        canvas already implements (ADR-0030 D5.1's gestures; the
        diagnosis this closes: set_tool existed with zero call sites,
        the status bar promised tools the UI never carried).

        Four checkable actions — Select, Segment, Circle, Arc — with
        the pinned shortcuts 1/2/3/4, an exclusive group (the active
        tool is VISIBLE as the checked action), acting on the ACTIVE
        sketch view only: each canvas view carries its own tool
        (per-view state), the selector reflects the current view's
        tool on every tab change and modifies only that view's. The
        buttons call canvas.set_tool() and nothing else — every
        gesture semantic below that call is the canvas's own.

        With no sketch view as the current tab the selector is
        HIDDEN (the declared choice: nothing to act on, no dead
        chrome — and a hidden toolbar's shortcuts are inert too, so
        1–4 mean nothing until a canvas is in front of the user).
        """
        from PySide6.QtGui import QActionGroup
        self._tool_actions = {}
        self._sketch_toolbar = QToolBar("Sketch Tools", self)
        self._sketch_toolbar.setMovable(False)
        group = QActionGroup(self)
        group.setExclusive(True)
        tips = {
            "select": "Select (1): pick an entity, drag its handles",
            "segment": "Segment (2): press at A, drag, release at B",
            "circle": "Circle (3): press at the centre, release on "
                      "the rim",
            "arc": "Arc (4): press-drag-release fixes r and a0, the "
                  "click fixes a1",
        }
        for tool, key in (("select", "1"), ("segment", "2"),
                          ("circle", "3"), ("arc", "4")):
            act = QAction(tool.capitalize(), self, checkable=True)
            act.setShortcut(QKeySequence(key))
            act.setToolTip(tips[tool])
            act.setStatusTip(tips[tool])
            act.setActionGroup(group)
            act.triggered.connect(
                lambda checked, t=tool: self._on_tool_action(t))
            self._sketch_toolbar.addAction(act)
            self._tool_actions[tool] = act
        self._tool_actions["select"].setChecked(True)
        self.addToolBar(self._sketch_toolbar)
        # No sketch view is open at startup: hidden (the choice above).
        self._sketch_toolbar.setVisible(False)
        # The selector follows the ACTIVE view: every tab change
        # re-reflects visibility, the checked action and the hint.
        self._tabs.currentChanged.connect(self._on_tab_changed)

    def _setup_statusbar(self) -> None:
        self._status = QStatusBar(self)
        self.setStatusBar(self._status)
        self._status.showMessage("Ready — try Create → Sphere")

    def _connect_signals(self) -> None:
        self._feature_tree.itemSelectionChanged.connect(self._on_tree_selection_changed)
        self._feature_tree.itemDoubleClicked.connect(
            self._on_tree_item_double_clicked)
        self._property_panel.parameter_changed.connect(self._on_parameter_changed)

    # ----- Slots -----

    def on_new(self) -> None:
        if not self._confirm_discard():
            return
        if self._scene is not None:
            self._scene.clear()
        for nid in list(self._sketch_views):
            self._close_sketch_view(nid)
        self._node_params.clear()
        self._node_types.clear()
        self._feature_tree.clear()
        self._property_panel.clear()
        self._selected_entity = None
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
            # Operation nodes get their own Type column: "CSG" for a
            # subtract, "FEAT" for an extrude (ADR-0030 D5.5), with
            # the visible dangling "!" suffix (ADR-0027 D2).
            op = getattr(node, "operation", None)
            if op is not None:
                if op.get("type") == "extrude":
                    base_label = "FEAT"
                    type_key = "feat_extrude"
                else:
                    base_label = "CSG"
                    type_key = "csg_subtract"
                type_label = base_label + "!" if node.operand_missing \
                    else base_label
            elif node.kind == "sketch":
                type_label = "Sketch"
                type_key = None
            else:
                type_label = "Mesh" if node.kind == "mesh" else "SDF"
                type_key = None
            item = QTreeWidgetItem([node.name, type_label])
            item.setData(0, Qt.UserRole, node.id)
            self._feature_tree.addTopLevelItem(item)
            if node.kind == "sketch":
                # 2D data: no builder, no params — the entity payload
                # is edited on the canvas / in the panel per entity.
                continue
            if node.kind == "mesh":
                # Imported geometry: no editing metadata, no defaults.
                continue
            if op is not None:
                self._node_types[node.id] = type_key
                if type_key == "feat_extrude":
                    self._node_params[node.id] = {
                        "distance": float(op["distance"])}
                else:
                    self._node_params[node.id] = {
                        "offset_x": float(op["offset_x"]),
                        "offset_y": float(op["offset_y"]),
                        "offset_z": float(op["offset_z"])}
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

    # ----- Sketch views (ADR-0030 D5.6: the canvas is a node view) -----

    def on_create_sketch(self) -> None:
        """Create → Sketch: the third node kind, undoable and
        id-stable, then open its canvas view (the natural next act
        after creating one)."""
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        state = {"node_id": None}

        def execute():
            if state["node_id"] is None:
                nid = self._scene.add_sketch_node("")
            else:
                nid = self._scene.restore_sketch_node(
                    state["node_id"], "", [])
            state["node_id"] = nid
            node = self._scene.get_node(nid)
            item = QTreeWidgetItem([node.name, "Sketch"])
            item.setData(0, Qt.UserRole, nid)
            self._feature_tree.addTopLevelItem(item)
            self._refresh_viewport()

        def undo():
            nid = state["node_id"]
            if nid is not None and self._scene.remove_node(nid):
                self._close_sketch_view(nid)
                self._remove_tree_row(nid)
                self._property_panel.clear()
                self._refresh_viewport()

        if self._command_stack:
            self._command_stack.push(execute, undo, "Create Sketch")
        else:
            execute()
        self.open_sketch_view(state["node_id"])
        self._status.showMessage(
            f"Created {self._scene.get_node(state['node_id']).name} "
            "— pick a tool on the sketch toolbar (1–4) and drag to draw",
            4000)

    def open_sketch_view(self, node_id: int) -> Optional[SketchCanvas]:
        """Open (or focus) the canvas view of a sketch node — ONE view
        per sketch at a time (D5.6). Double-click on the tree row and
        Create → Sketch both land here."""
        if not self._has_kernel or node_id is None:
            return None
        node = self._scene.get_node(node_id)
        if node is None or node.kind != "sketch":
            return None
        view = self._sketch_views.get(node_id)
        if view is not None:
            self._tabs.setCurrentWidget(view)
            return view
        view = SketchCanvas(self._scene, self._command_stack, node_id,
                            parent=self)
        view.entity_selected.connect(self._on_canvas_entity_selected)
        view.sketch_edited.connect(self._on_canvas_sketch_edited)
        self._sketch_views[node_id] = view
        self._tabs.addTab(view, f"Sketch: {node.name}")
        self._tabs.setCurrentWidget(view)
        # Reflect the now-active view in the selector (currentChanged
        # fires only when the tab actually changed — this also covers
        # the already-current case; idempotent either way).
        self._sync_sketch_selector()
        return view

    def _close_sketch_view(self, node_id: int) -> None:
        """Close the view of a sketch (D5.6: closing commits NOTHING —
        every completed gesture already wrote; it just stops looking)."""
        view = self._sketch_views.pop(node_id, None)
        if view is None:
            return
        if self._selected_entity is not None and \
                self._selected_entity[0] == node_id:
            self._selected_entity = None
        index = self._tabs.indexOf(view)
        if index >= 0:
            self._tabs.removeTab(index)
        # Re-reflect the active view (the closed tab may have been
        # current: the selector follows whatever is in front now).
        self._sync_sketch_selector()
        view.deleteLater()

    def _on_tab_close_requested(self, index: int) -> None:
        if index <= 0:
            return
        widget = self._tabs.widget(index)
        for nid, view in list(self._sketch_views.items()):
            if view is widget:
                self._close_sketch_view(nid)
                return

    def _on_tree_item_double_clicked(self, item, _column) -> None:
        node_id = item.data(0, Qt.UserRole)
        if node_id is None or not self._scene:
            return
        node = self._scene.get_node(node_id)
        if node is not None and node.kind == "sketch":
            self.open_sketch_view(node_id)

    def _on_canvas_entity_selected(self, target) -> None:
        """A canvas selected (or deselected) an entity: route its
        named parameters to the panel (the registry consumer — zero
        new panel code, D5.1)."""
        if target is None:
            self._selected_entity = None
            self._on_tree_selection_changed()
            return
        chain, index = target
        # Find which sketch's canvas emitted (the sender).
        view = self.sender()
        node_id = view.node_id() if view is not None else None
        if node_id is None:
            return
        self._selected_entity = (node_id, chain, index)
        chains = self._scene.sketch_chains(node_id)
        e = chains[chain][index]
        label = f"{e['kind']}[chain {chain} #{index}]"
        self._property_panel.set_parameters(label, params_from_entity(e))

    def _on_canvas_sketch_edited(self) -> None:
        """A canvas committed a write: the recipe changed — refresh
        the operation rows (dangling/pending state), the viewport and
        the entity routing (the panel is a view of the recipe).

        The panel refresh never destroys editors while one may still
        be inside its own event handler (the typed-parameter
        use-after-free: the recipe echo used to rebuild the rows from
        INSIDE the spinbox's keyPressEvent, deleting the very editor
        the stack was about to return into — QLineEdit::text() on a
        freed object). Same entity showing → the editors' VALUES
        update IN PLACE (no row destroyed, the committing editor
        stays live); any other label → the rebuild is DEFERRED to the
        next loop pass, when every event handler has returned.
        """
        self._refresh_operation_rows()
        self._refresh_viewport()
        if self._selected_entity is not None:
            node_id, chain, index = self._selected_entity
            chains = self._scene.sketch_chains(node_id) \
                if self._scene.get_node(node_id) is not None else []
            if chain < len(chains) and index < len(chains[chain]):
                e = chains[chain][index]
                label = f"{e['kind']}[chain {chain} #{index}]"
                if self._property_panel.current_label() == label:
                    # The same entity's surface: values only, in place.
                    self._property_panel.update_values(
                        params_from_entity(e))
                else:
                    # A different surface is showing: rebuild on the
                    # NEXT loop pass — no widget dies while it may be
                    # processing an event.
                    QTimer.singleShot(
                        0, self._deferred_entity_panel_refresh)
        # The write may have emptied the view again (an undo of the
        # first entity): the empty-sketch hint follows the recipe.
        view = self.sender()
        if not isinstance(view, SketchCanvas):
            view = None
        if view is None and self._selected_entity is not None:
            view = self._sketch_views.get(self._selected_entity[0])
        self._update_empty_hint(view)

    def _deferred_entity_panel_refresh(self) -> None:
        """The next-pass panel rebuild for the selected entity.

        Scheduled by _on_canvas_sketch_edited when the panel is not
        showing the edited entity: by the time this runs, the event
        loop has re-entered and every editor's event handler has
        returned — a row rebuild can no longer delete a live
        editor under its own stack. The refresh reads the CURRENT
        selection at fire time, so it can never clobber a newer
        selection with a stale snapshot.
        """
        if self._selected_entity is None or self._scene is None:
            return
        node_id, chain, index = self._selected_entity
        if self._scene.get_node(node_id) is None:
            return
        chains = self._scene.sketch_chains(node_id)
        if chain >= len(chains) or index >= len(chains[chain]):
            return
        e = chains[chain][index]
        label = f"{e['kind']}[chain {chain} #{index}]"
        if self._property_panel.current_label() != label:
            self._property_panel.set_parameters(
                label, params_from_entity(e))

    # ----- Sketch tool selector (the wiring that reaches D5.1) -----

    EMPTY_SKETCH_HINT = (
        "empty sketch — pick a tool (1 Select, 2 Segment, 3 Circle, "
        "4 Arc) and drag to draw")

    def _active_sketch_view(self) -> Optional[SketchCanvas]:
        """The SketchCanvas of the CURRENT tab, or None (the 3D view
        or no view at all): the one view the selector acts on."""
        widget = self._tabs.currentWidget()
        for view in self._sketch_views.values():
            if view is widget:
                return view
        return None

    def _on_tool_action(self, tool: str) -> None:
        """A selector action fired (button click or pinned shortcut):
        set the ACTIVE view's tool — set_tool() and nothing else; the
        canvas owns every gesture semantic that follows it."""
        view = self._active_sketch_view()
        if view is not None:
            view.set_tool(tool)

    def _on_tab_changed(self, _index: int) -> None:
        self._sync_sketch_selector()

    def _sync_sketch_selector(self) -> None:
        """Reflect the active view in the selector: toolbar visibility,
        the checked action (the view's OWN tool — per-view state, the
        canvas keeps it, the selector only mirrors it) and the
        empty-sketch hint. Called on every tab change and on every
        view open/close; idempotent."""
        view = self._active_sketch_view()
        self._sketch_toolbar.setVisible(view is not None)
        if view is not None:
            tool = view.tool()
            for name, act in self._tool_actions.items():
                act.setChecked(name == tool)
            self._update_empty_hint(view)

    def _update_empty_hint(self, view: Optional[SketchCanvas]) -> None:
        """The empty-canvas hint — through the PINNED status channel
        only (the canvas status bar, D5.3's aspect table / D5.5), no
        new graphic element: while the view carries no entity, the
        channel carries "pick a tool and drag to draw"; the first
        gesture clears the channel (the canvas's own rule, nothing
        added) and the hint returns if an undo empties the view
        again. A refusal standing in the channel is never clobbered.

        The canvas pins this channel but exposes no public setter —
        and sketch_canvas.py is complete and untouched by this wiring:
        the message is written to the channel's backing attribute
        directly, the one bridge, exactly as narrow as the pinned
        channel it serves.
        """
        if view is None or self._scene is None:
            return
        if self._scene.get_node(view.node_id()) is None:
            return
        chains = self._scene.sketch_chains(view.node_id())
        if any(chain for chain in chains):
            return
        if view.status_message:
            return  # a refusal (or the hint itself) holds the channel
        view._status_message = self.EMPTY_SKETCH_HINT
        view.update()

    # ----- Extrude command (ADR-0030 D5.5/D5.6) -----

    def on_extrude(self) -> None:
        """Feature → Extrude: with a sketch node selected, the minimal
        dialog (distance, direction ±Z; defaults 1.0/+Z). Anything
        else is a visible warning, never a guess."""
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        node = None
        if self._selected_node_id is not None:
            node = self._scene.get_node(self._selected_node_id)
        if node is None or node.kind != "sketch":
            QMessageBox.information(
                self, "Extrude",
                "Select a sketch node in the feature tree,\n"
                "then Feature → Extrude.")
            return
        result = self._extrude_dialog(node)
        if result is None:
            return
        distance, direction = result
        self._do_extrude(node.id, distance, direction)

    def _extrude_dialog(self, node):
        """The minimal dialog: distance + direction. Returns
        (distance, ±1) or None when cancelled. OK with distance <= 0
        or non-finite is refused with the message (D5.5)."""
        import math as _math
        dlg = QDialog(self)
        dlg.setWindowTitle(f"Extrude — {node.name}")
        layout = QVBoxLayout(dlg)

        row = QHBoxLayout()
        spin = QDoubleSpinBox(dlg)
        spin.setRange(-1.0e9, 1.0e9)
        spin.setDecimals(6)
        spin.setSingleStep(0.1)
        spin.setValue(1.0)
        combo = QComboBox(dlg)
        combo.addItem("+Z")
        combo.addItem("-Z")
        row.addWidget(spin)
        row.addWidget(combo)
        layout.addLayout(row)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dlg)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        layout.addWidget(buttons)

        while True:
            if dlg.exec() != QDialog.Accepted:
                return None
            d = spin.value()
            if not _math.isfinite(d) or d <= 0.0:
                QMessageBox.warning(
                    dlg, "Extrude",
                    f"Distance must be finite and > 0, got {d} — "
                    "the slab would be empty or inverted.")
                continue
            return (d, 1 if combo.currentIndex() == 0 else -1)

    def _do_extrude(self, sketch_id: int, distance: float,
                    direction: int) -> Optional[int]:
        """Dialog-free core: create the extrude operation from the
        sketch (undoable, id-stable). The KERNEL's certificate is the
        one opinion: an open chain, a mid-span crossing or a bad
        distance refuses with the kernel's own message (naming the
        offenders) and NOTHING is written — visible, never silent."""
        sketch = self._scene.get_node(sketch_id)
        sketch_name = sketch.name if sketch is not None else "?"
        label = f"Extrude (Sketch#{sketch_id})"
        state = {"op_id": None}

        def execute():
            if state["op_id"] is None:
                try:
                    nid = self._scene.add_extrude_node(
                        label, sketch_id, distance, direction)
                except (ValueError, RuntimeError) as exc:
                    # The refusal is visible, with the kernel's own
                    # words (the open entities are named inside).
                    QMessageBox.critical(self, "Extrude", str(exc))
                    return
            else:
                nid = self._scene.restore_extrude_node(
                    state["op_id"], label, sketch_id, distance, direction)
            state["op_id"] = nid
            node = self._scene.get_node(nid)
            if node is not None:
                node.name = label
            self._node_params[nid] = {"distance": float(distance)}
            self._node_types[nid] = "feat_extrude"
            item = QTreeWidgetItem([label, "FEAT"])
            item.setData(0, Qt.UserRole, nid)
            self._feature_tree.addTopLevelItem(item)
            self._refresh_viewport()
            self._status.showMessage(f"Created {label}", 3000)

        def undo():
            nid = state["op_id"]
            if nid is not None and self._scene.remove_node(nid):
                self._node_params.pop(nid, None)
                self._node_types.pop(nid, None)
                self._remove_tree_row(nid)
                self._property_panel.clear()
                self._refresh_viewport()

        if self._command_stack:
            self._command_stack.push(execute, undo, "Extrude")
        else:
            execute()
        return state["op_id"]

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

        Id-stable across undo/redo cycles (ADR-0027 D2, the verified
        finding-2 fix): the FIRST execution allocates the id, undo
        removes the node, and redo re-creates it WITH THE SAME ID via
        restore-style insertion — references held by operations must
        never dangle spuriously. (The old behavior re-allocated ids on
        redo, documented as acceptable before any feature referenced
        ids; Phase F made it a defect.)
        """
        name = self._shape_display_name(shape_type)
        state = {"node_id": None}

        def execute():
            if state["node_id"] is None:
                body = self._make_body(shape_type, params)
                nid = self._scene.add_node(name, body)
            else:
                # Redo: the same node returns with its ORIGINAL id
                # (restore-style insertion; a live id would raise).
                nid = self._scene.restore_sdf_node(
                    state["node_id"], name, self._make_body(shape_type, params))
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
                # Remove from scene + side dicts + tree widget. The id
                # stays pinned in `state` for the redo.
                if self._scene and self._scene.remove_node(nid):
                    self._node_params.pop(nid, None)
                    self._node_types.pop(nid, None)
                    self._remove_tree_row(nid)
                    self._property_panel.clear()
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
        """Rebuild a PRIMITIVE node's body from its stored params and
        type, then run the DAG pass so every dependent operation
        follows the edit (ADR-0027 D2).

        Operation nodes do NOT go through here: their editable surface
        is the placement, handled by _push_operation_offset_command
        (which reads the other two axes from the node's CURRENT recipe
        — the recipe is the source of truth; the UI keeps no second
        copy of any parameter table).
        """
        if node_id not in self._node_params or not self._scene:
            return
        params = self._node_params[node_id]
        shape_type = self._node_types.get(node_id, "")
        if shape_type == "csg_subtract":
            return  # operations re-materialize through their command

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
            # The edit may feed dependent operations (ADR-0027 D2:
            # the DAG pass propagates operand edits to dependents).
            self._scene.rebuild_operations()
            self._refresh_operation_rows()
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

    def _set_tessellation_resolution(self, resolution: int) -> None:
        """Core of View -> Tessellation Resolution (dialog-free, the
        _do_subtract pattern for offscreen tests).

        A pure view setting: no undo command, no scene mutation beyond
        re-dirtying the tessellation caches. Marks every Sdf node
        dirty and re-tessellates through the normal refresh path, so
        the viewport (and everything that reads the cache afterwards)
        sees the new resolution. Mesh-kind nodes are skipped by kind —
        imported geometry is never re-tessellated.
        """
        if resolution == self._tess_resolution:
            return
        self._tess_resolution = resolution
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None or node.kind != "sdf":
                continue
            node.mesh_dirty = True
        self._viewport.set_tessellation_resolution(resolution)
        self._refresh_viewport()
        self._status.showMessage(f"Tessellation resolution: {resolution}", 3000)

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
        # Snapshot the node so we can re-create it on undo
        shape_type = self._node_types.get(node_id, "")
        params = dict(self._node_params.get(node_id, {}))
        is_mesh = False
        is_sketch = False
        node = self._scene.get_node(node_id)
        if node is not None and node.kind == "mesh":
            is_mesh = True
        if node is not None and node.kind == "sketch":
            # A sketch's recipe is its entity payload: snapshot the
            # chains (dict form) for the id-stable undo — deleting a
            # sketch referenced by an extrude SUCCEEDS, the extrude
            # enters the visible dangling state, undo recovers it
            # (D5.6).
            is_sketch = True
            sketch_payload = self._scene.sketch_chains(node_id)
            sketch_name = node.name
        name = self._shape_display_name(shape_type) if shape_type else (
            node.name if node is not None else "")
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
                self._remove_tree_row(nid)
                self._property_panel.clear()
                if is_sketch:
                    # The open canvas view of the deleted sketch CLOSES
                    # (D5.6: the node is gone; undo recovers it).
                    self._close_sketch_view(nid)
                # An operand deletion may dangle dependent operations:
                # run the DAG pass and refresh their rows (visible
                # state, never a silent rebind — ADR-0027 D2).
                if self._has_kernel:
                    self._scene.rebuild_operations()
                    self._refresh_operation_rows()
                self._refresh_viewport()

        def undo():
            nid = state["node_id"]
            if nid is None or is_mesh:
                # Imported meshes are not re-creatable parametrically
                # (pre-existing gap, unchanged by Phase F: delete of a
                # mesh node is not undoable today).
                return
            if is_sketch:
                # The sketch returns WITH ITS ORIGINAL ID and its full
                # entity payload; the DAG pass recovers the dependents.
                chains = self._chains_from_payload(sketch_payload)
                self._scene.restore_sketch_node(nid, sketch_name, chains)
                new_item = QTreeWidgetItem([sketch_name, "Sketch"])
                new_item.setData(0, Qt.UserRole, nid)
                idx = state.get("tree_item_idx")
                if idx is not None and 0 <= idx <= self._feature_tree.topLevelItemCount():
                    self._feature_tree.insertTopLevelItem(idx, new_item)
                else:
                    self._feature_tree.addTopLevelItem(new_item)
                if self._has_kernel:
                    self._scene.rebuild_operations()
                    self._refresh_operation_rows()
                self._refresh_viewport()
                return
            # Re-create the node with its ORIGINAL id (id-stable undo,
            # ADR-0027 D2): restore-style insertion, then re-attach the
            # editing metadata so save round-trips and the panel works.
            try:
                body = self._make_body(shape_type, params)
            except RuntimeError:
                return  # unknown builder tag: nothing to re-create
            new_id = self._scene.restore_sdf_node(nid, name, body)
            node = self._scene.get_node(new_id)
            if node is not None and shape_type:
                node.builder = shape_type
                node.set_params({k: float(v) for k, v in params.items()})
            self._node_params[new_id] = dict(params)
            self._node_types[new_id] = shape_type
            new_item = QTreeWidgetItem([name, "SDF"])
            new_item.setData(0, Qt.UserRole, new_id)
            idx = state.get("tree_item_idx")
            if idx is not None and 0 <= idx <= self._feature_tree.topLevelItemCount():
                self._feature_tree.insertTopLevelItem(idx, new_item)
            else:
                self._feature_tree.addTopLevelItem(new_item)
            # The restored operand may recover dangling operations.
            if self._has_kernel:
                self._scene.rebuild_operations()
                self._refresh_operation_rows()
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

    def _chains_from_payload(self, payload):
        """The readable chains payload (dicts) back into kernel
        SketchChain objects — the delete-undo's re-creation material."""
        import CAD_0
        scene = CAD_0.scene
        out = []
        for chain in payload:
            ents = [make_entity(e["kind"], params_from_entity(e))
                    for e in chain]
            out.append(scene.SketchChain(ents))
        return out

    # ----- Boolean feature: CSG subtract (ADR-0027 D5) -----

    def _selected_sdf_ids(self) -> list:
        """The ids of the selected SDF-kind nodes (tree order)."""
        out = []
        for item in self._feature_tree.selectedItems():
            nid = item.data(0, Qt.UserRole)
            if nid is None:
                continue
            node = self._scene.get_node(nid) if self._scene else None
            if node is not None and node.kind == "sdf":
                out.append(nid)
        return out

    def on_subtract(self) -> None:
        """Edit → Subtract: the boolean difference of two selected bodies.

        Entry point (ADR-0027 D5): exactly two SDF-kind nodes selected
        → a minimal dialog (target/tool pre-filled from the selection,
        default target = the lower FeatureId — deterministic, no
        geometric heuristics — plus the tool placement offset, world
        coordinates). Anything else is a visible warning, never a guess.
        """
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        ids = self._selected_sdf_ids()
        if len(ids) != 2:
            QMessageBox.information(
                self, "Subtract",
                "Select exactly two SDF bodies in the feature tree:\n"
                "the TARGET (material to keep) and the TOOL\n"
                "(material to remove), then Edit → Subtract.")
            return
        target_id, tool_id, ox, oy, oz = self._subtract_dialog(ids[0], ids[1])
        if target_id is None:
            return  # cancelled
        self._do_subtract(target_id, tool_id, ox, oy, oz)

    def _subtract_dialog(self, first_id: int, second_id: int):
        """Minimal subtract dialog. Returns (target, tool, ox, oy, oz),
        or (None, None, 0, 0, 0) when cancelled."""
        # The selectable SDF nodes (label + id), in tree order.
        entries = []
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None or node.kind != "sdf":
                continue
            entries.append((f"{node.name}  (id {node.id})", node.id))

        dlg = QDialog(self)
        dlg.setWindowTitle("Subtract (boolean difference)")
        layout = QVBoxLayout(dlg)

        combo_target = QComboBox(dlg)
        combo_tool = QComboBox(dlg)
        for label, _ in entries:
            combo_target.addItem(label)
            combo_tool.addItem(label)
        # Deterministic default (ADR-0027 D5): target = the lower
        # FeatureId of the two selected nodes — never a geometric
        # heuristic.
        lo, hi = min(first_id, second_id), max(first_id, second_id)
        combo_target.setCurrentIndex([nid for _, nid in entries].index(lo))
        combo_tool.setCurrentIndex([nid for _, nid in entries].index(hi))

        row1 = QHBoxLayout()
        row1.addWidget(combo_target)
        row1.addWidget(combo_tool)
        layout.addLayout(row1)

        spins = []
        row2 = QHBoxLayout()
        for axis in ("x", "y", "z"):
            spin = QDoubleSpinBox(dlg)
            spin.setRange(-10.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            spin.setValue(0.0)
            spins.append(spin)
            row2.addWidget(spin)
        layout.addLayout(row2)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dlg)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        layout.addWidget(buttons)

        if dlg.exec() != QDialog.Accepted:
            return None, None, 0.0, 0.0, 0.0
        target_id = [nid for _, nid in entries][combo_target.currentIndex()]
        tool_id = [nid for _, nid in entries][combo_tool.currentIndex()]
        if target_id == tool_id:
            QMessageBox.warning(
                self, "Subtract",
                "Target and tool must be two different bodies.")
            return None, None, 0.0, 0.0, 0.0
        return (target_id, tool_id,
                spins[0].value(), spins[1].value(), spins[2].value())

    def _operation_label(self, target_id: int, tool_id: int) -> str:
        """The operation row's label: it NAMES its operands (ADR-0027
        D5) — 'Subtract (Box#1 - Cylinder#2)'. Names are read from the
        live nodes; a deleted operand keeps its last known name here
        (the row still identifies what was subtracted)."""
        parts = []
        for nid in (target_id, tool_id):
            node = self._scene.get_node(nid) if self._scene else None
            name = node.name if node is not None else "?"
            parts.append(f"{name}#{nid}")
        return f"Subtract ({parts[0]} - {parts[1]})"

    def _do_subtract(self, target_id: int, tool_id: int,
                     ox: float, oy: float, oz: float) -> None:
        """Dialog-free core: create the subtract operation (undoable,
        id-stable). The operands are hidden from the viewport (the
        operation is the display); they stay in the tree, selectable
        and editable, exactly as before."""
        state = {"op_id": None}
        label = self._operation_label(target_id, tool_id)

        def execute():
            if state["op_id"] is None:
                nid = self._scene.add_operation_node(
                    "Subtract", target_id, tool_id, ox, oy, oz)
            else:
                # Redo: the operation returns with its ORIGINAL id
                # (id-stable, ADR-0027 D2 — references never rebind).
                nid = self._scene.restore_operation_node(
                    state["op_id"], "Subtract", target_id, tool_id,
                    ox, oy, oz)
            state["op_id"] = nid
            node = self._scene.get_node(nid)
            if node is not None:
                node.name = label
            # The placement offsets are the operation's editable
            # params (the property panel is generic over param dicts).
            self._node_params[nid] = {
                "offset_x": float(ox), "offset_y": float(oy),
                "offset_z": float(oz)}
            self._node_types[nid] = "csg_subtract"
            # Hide the operands (serialized state — round-trips).
            self._scene.set_visible(target_id, False)
            self._scene.set_visible(tool_id, False)
            item = QTreeWidgetItem([label, "CSG"])
            item.setData(0, Qt.UserRole, nid)
            self._feature_tree.addTopLevelItem(item)
            self._refresh_viewport()
            self._status.showMessage(f"Created {label}", 3000)

        def undo():
            nid = state["op_id"]
            if nid is not None and self._scene.remove_node(nid):
                self._node_params.pop(nid, None)
                self._node_types.pop(nid, None)
                self._remove_tree_row(nid)
                # The operands become visible again.
                self._scene.set_visible(target_id, True)
                self._scene.set_visible(tool_id, True)
                self._property_panel.clear()
                self._refresh_viewport()

        if self._command_stack:
            self._command_stack.push(execute, undo, f"Subtract")
        else:
            execute()
        return state["op_id"]

    def _remove_tree_row(self, node_id: int) -> None:
        """Remove the feature-tree row of a node id (helper shared by
        every id-stable command)."""
        for i in range(self._feature_tree.topLevelItemCount()):
            it = self._feature_tree.topLevelItem(i)
            if it.data(0, Qt.UserRole) == node_id:
                self._feature_tree.takeTopLevelItem(i)
                return

    def _refresh_operation_rows(self) -> None:
        """Refresh the operation rows' Type column from the live DAG
        state: 'CSG'/'FEAT' when healthy, 'CSG!'/'FEAT!' when dangling
        (the visible error state — ADR-0027 D2: reported, never
        silently re-picked)."""
        if not self._scene:
            return
        for i in range(self._feature_tree.topLevelItemCount()):
            it = self._feature_tree.topLevelItem(i)
            nid = it.data(0, Qt.UserRole)
            if nid is None:
                continue
            node = self._scene.get_node(nid)
            if node is None or not getattr(node, "operation", None):
                continue
            base = "FEAT" if node.operation.get("type") == "extrude" \
                else "CSG"
            it.setText(1, base + "!" if node.operand_missing else base)

    def on_about(self) -> None:
        # License line: backed by the Apache-2.0 LICENSE at the repo
        # root (see docs/audits/2026-10-16-third-party-license-audit.md
        # for the dependency inventory). State, not history — no phase
        # narration here.
        QMessageBox.about(
            self,
            "About CAD_0",
            "<h3>CAD_0</h3>"
            "<p>Dual-representation CAD kernel — native SDF + B-Rep, "
            "with an SDF → B-Rep extraction bridge and boolean "
            "operations on the B-Rep side.</p>"
            "<p>PySide6 desktop app: parametric creation, undo/redo, "
            "viewport picking, STL/OBJ import/export.</p>"
            "<p>License: Apache-2.0 — see LICENSE in the repository "
            "root.</p>"
        )

    # ----- Debug menu (read-only inspection) -----

    def _sdf_to_brep_report_text(self, node) -> Optional[str]:
        """The Debug → Extract SDF → B-Rep report TEXT for a node (or
        None, with the warning already shown, when the node is not
        extractable). Pure string building — no dialogs — so the UI
        smoke tests can assert the report content offscreen without
        touching QMessageBox.

        Amended content (ADR-0027 D6): χ, genus and the STRUCTURAL
        contract (is_closed_orientable) — not just closed yes/no. The
        genus follows the acceptance-correction-1 rule: derived only
        when χ ≤ 2 and even; a multi-shell body (χ > 2) reports
        "n/d" (the components counter is deferred).
        """
        if node is None or node.kind != "sdf":
            return None

        # The bridge submodule: re-exported by the package __init__; the
        # direct fallback covers a bare _CAD_0 found on sys.path. Any
        # failure here (e.g. a stale mixed old-package/new-extension
        # install) is reported, never a crash.
        import CAD_0
        try:
            bridge_mod = getattr(CAD_0, "bridge", None) or CAD_0._CAD_0.bridge
        except Exception as exc:
            QMessageBox.critical(self, "SDF → B-Rep", str(exc))
            return None

        # Timed extraction (default options: resolution 48, primitive
        # fit first). Bridge errors (ValueError / RuntimeError) cross
        # the binding with the FULL message — diagnosis and remedy are
        # already inside (ADR-0026 D2): shown verbatim, never
        # translated, never reduced to a log line.
        import time
        opts = bridge_mod.SdfToBrepOptions()
        t0 = time.perf_counter()
        try:
            rep = bridge_mod.sdf_to_brep_report(node.body, opts)
        except Exception as exc:
            QMessageBox.critical(self, "SDF → B-Rep", str(exc))
            return None
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        fit_label = {
            "sphere": "exact primitive (sphere)",
            "box": "exact primitive (box)",
            "cylinder": "exact primitive (cylinder)",
            "faceted": "faceted extraction (MC33)",
        }.get(rep["fit"], rep["fit"])

        # Labels are precomputed OUTSIDE the f-string: a backslash
        # inside an f-string expression (e.g. the '\u00ec' escape of
        # the old inline conditional) is a syntax error on Python
        # 3.10, the supported floor (CI). The file is UTF-8, so the
        # plain literal is fine.
        orientable_label = "yes" if rep["orientable"] else "NO"
        loops_label = "yes" if rep["loops"] else "NO"

        # Genus under the correction-1 rule (ADR-0027 D6): an int when
        # derived (χ ≤ 2 and even, structurally sound); "n/d" for a
        # multi-shell body (χ > 2 — the components counter is
        # deferred), never a fabricated half-integer.
        if rep["genus"] is None:
            genus_label = ("n/d (multi-shell? chi > 2 — components "
                           "counter deferred)")
        else:
            genus_label = str(rep["genus"])

        # min |f| over the sampled grid (ADR-0026 Consequences
        # left-over, resolved as a report field): the P2 margin — how
        # far the closest grid sample sits from the surface. 0 would
        # mean an on-grid zero (the faceted path rejects it). A
        # diagnostic, not a contract: shown for both paths.
        min_abs_f = rep["min_abs_field"]

        return (
            f"Body: \"{node.name}\" (id {node.id}, {node.kind})\n"
            f"Path: {fit_label}\n"
            f"V = {rep['V']}, E = {rep['E']}, F = {rep['F']}\n"
            f"\u03c7 = V - E + F = {rep['chi']}\n"
            f"Genus = {genus_label}\n"
            f"Closed orientable (is_closed_orientable): {orientable_label}\n"
            f"Loops (loops_are_closed): {loops_label}\n"
            f"min |f| on grid (P2 margin): {min_abs_f:.6g}\n"
            f"Extraction time: {elapsed_ms:.1f} ms"
        )

    def on_debug_sdf_to_brep_report(self) -> None:
        """Debug → Extract SDF → B-Rep: topology report (ADR-0026
        consumer, amended by ADR-0027 D6).

        Runs the SDF → B-Rep bridge on the selected SDF body (or the
        scene's only body when nothing is selected) and reports V/E/F,
        the Euler characteristic, the genus (correction-1 rule), the
        structural manifold/loop invariants, the path the bridge took
        (exact primitive vs faceted extraction) and the extraction
        time. Strictly read-only: the scene, the document and the
        undo stack are untouched — an inspection tool, not an
        authoring action.
        """
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return

        # Which body? The selected node; with an empty selection, the
        # scene's only node (the "active" body). Anything ambiguous is
        # a visible warning — never a guess.
        node = None
        if self._selected_node_id is not None:
            node = self._scene.get_node(self._selected_node_id)
        if node is None and self._scene.node_count() == 1:
            node = self._scene.node_at(0)
        if node is None:
            QMessageBox.warning(
                self, "SDF → B-Rep",
                "No SDF body selected.\n"
                "Select a body in the feature tree (or in the viewport); "
                "if the scene contains exactly one body, it is used "
                "automatically.")
            return
        if node.kind != "sdf":
            QMessageBox.warning(
                self, "SDF → B-Rep",
                f"The node \"{node.name}\" is a mesh node, not an SDF node: "
                "there is no field to convert (ADR-0002).")
            return

        text = self._sdf_to_brep_report_text(node)
        if text is None:
            return
        box = QMessageBox(
            QMessageBox.Icon.Information,
            "SDF → B-Rep — topology report",
            text,
            QMessageBox.StandardButton.Ok,
            self,
        )
        # Inspection output: let the user select/copy the numbers.
        box.setTextInteractionFlags(Qt.TextSelectableByMouse)
        box.exec()

    def _clear_selection(self) -> None:
        """Clear the current selection, whatever its origin.

        Used when the user clicks empty space in the viewport: if a
        node is selected, the feature-tree row, the 3D highlight and
        the property panel are all reset — the same visible effect as
        Ctrl-clicking the selected tree row. No-op when nothing is
        selected (no spurious signals, no panel flicker).
        """
        if self._selected_node_id is None:
            return
        self._selected_node_id = None
        # clearSelection() fires itemSelectionChanged, which routes
        # through _on_tree_selection_changed() (empty selection →
        # viewport highlight + property panel cleared). The explicit
        # calls below are a safety net for the impossible-but-defensive
        # case where the tree selection and _selected_node_id disagree.
        self._feature_tree.clearSelection()
        self._viewport.set_selection(None)
        self._property_panel.clear()

    def _on_tree_selection_changed(self) -> None:
        items = self._feature_tree.selectedItems()
        if not items:
            # Single source of truth: every deselection (tree click,
            # Ctrl-click, viewport empty click, programmatic clear)
            # lands here — the MainWindow-side id must be reset too,
            # not only the viewport highlight and the panel.
            self._selected_node_id = None
            self._selected_entity = None
            self._viewport.set_selection(None)
            self._property_panel.clear()
            return
        item = items[0]
        node_id = item.data(0, Qt.UserRole)
        if node_id is None:
            self._selected_node_id = None
            self._viewport.set_selection(None)
            return
        node = self._scene.get_node(node_id)
        if node is None:
            self._selected_node_id = None
            self._viewport.set_selection(None)
            return
        # A node selection takes the panel back from any entity
        # selection (the canvas's entity context is view state).
        self._selected_entity = None
        # 3D highlight follows every selection change, whatever its
        # origin (viewport click, tree click, programmatic set) — and
        # covers EVERY selected row of the extended selection: both
        # bodies of a subtract pair light up, not just the anchor the
        # panel routes on (the selection always worked; only the
        # highlight ignored the rows past the first).
        selected_ids = []
        for it in items:
            nid = it.data(0, Qt.UserRole)
            if nid is not None:
                selected_ids.append(nid)
        self._viewport.set_selection(selected_ids)
        self._selected_node_id = node_id
        if node.kind == "mesh":
            # Imported geometry: read-only info (nothing parametric to
            # edit — the mesh is data, not a recipe).
            self._property_panel.set_readonly_fields(
                node.name, self._mesh_info_fields(node))
            return
        if node.kind == "sketch":
            # The sketch node itself: read-only summary — the entity
            # payload is addressed per entity (canvas selection).
            chains = self._scene.sketch_chains(node_id)
            entities = sum(len(c) for c in chains)
            closed = "yes" if self._scene.sketch_closed(node_id) else \
                "NO — open chains (the extrude would refuse)"
            self._property_panel.set_readonly_fields(node.name, {
                "Kind": "Sketch (world XY)",
                "Chains": str(len(chains)),
                "Entities": str(entities),
                "Fully defined": closed,
            })
            return
        # Operation node (ADR-0027 D5): the placement offsets are the
        # editable params; the operand summary (names + ids) and any
        # dangling error are read-only context below them. The
        # extrude's editable surface is the recipe's distance
        # (ADR-0030 D2).
        op = getattr(node, "operation", None)
        if op:
            params = self._node_params.get(node_id, {})
            if not params:
                if op.get("type") == "extrude":
                    params = {"distance": float(op["distance"])}
                else:
                    params = {"offset_x": float(op["offset_x"]),
                              "offset_y": float(op["offset_y"]),
                              "offset_z": float(op["offset_z"])}
                self._node_params[node_id] = params
            display_params = {k: float(v) for k, v in params.items()
                              if isinstance(v, (int, float))}
            readonly = self._operation_info_fields(node_id, op)
            self._property_panel.set_parameters_and_readonly(
                node.name, display_params, readonly)
            return
        # Show the node's parameters in the property panel.
        params = self._node_params.get(node_id, {})
        # Convert to dict[str, float|str|bool] for PropertyPanel
        display_params = {}
        for k, v in params.items():
            display_params[k] = float(v) if isinstance(v, (int, float)) else str(v)
        self._property_panel.set_parameters(node.name, display_params)

    def _operation_info_fields(self, node_id: int, op: dict) -> dict:
        """Read-only property-panel fields for an operation node: the
        operand summary (names + ids) and, when dangling, the
        actionable error (ADR-0027 D2 — reported, never re-picked)."""
        fields = {}
        if op.get("type") == "extrude":
            nid = op.get("sketch")
            operand = self._scene.get_node(nid) if nid is not None else None
            name = operand.name if operand is not None else "?"
            fields["Sketch operand"] = f"{name}  (id {nid})"
            fields["Direction"] = "+Z" if op.get("direction", 1) == 1 \
                else "-Z"
        else:
            for role in ("target", "tool"):
                nid = op.get(role)
                operand = self._scene.get_node(nid) if nid is not None \
                    else None
                name = operand.name if operand is not None else "?"
                fields[f"{role.capitalize()} operand"] = f"{name}  (id {nid})"
        err = self._scene.operation_error(node_id)
        if err:
            fields["ERROR"] = err
        return fields

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
        # A canvas entity's parameter (the sketch's recipe): the SAME
        # route a released drag uses — one truth, two affordances
        # (D5.2). The write validates through the kernel and lands as
        # ONE undo entry; the panel re-routes from the recipe after.
        if self._selected_entity is not None:
            try:
                value = float(value)
            except (TypeError, ValueError):
                return
            node_id, chain, index = self._selected_entity
            view = self._sketch_views.get(node_id)
            if view is not None:
                view.edit_entity_params(
                    chain, index, {name: value}, f"Set {name} = {value}")
                self._on_canvas_sketch_edited()
            return

        node_id = self._selected_node_id
        if node_id is None or node_id not in self._node_params:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return

        # Operation nodes: the editable surface is the placement — a
        # dedicated undoable command that reads the other two axes from
        # the node's CURRENT recipe (source of truth, no UI copy). The
        # extrude's surface is the recipe's distance (ADR-0030 D2).
        node_type = self._node_types.get(node_id)
        if node_type == "csg_subtract":
            self._push_operation_offset_command(node_id, name, value)
            return
        if node_type == "feat_extrude" and name == "distance":
            self._push_extrude_distance_command(node_id, value)
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

    def _push_operation_offset_command(self, node_id: int,
                                        axis: str, value: float) -> None:
        node = self._scene.get_node(node_id)
        op = node.operation if node is not None else None
        if op is None or axis not in ("offset_x", "offset_y", "offset_z"):
            return

        def apply(v: float) -> None:
            axes = {"offset_x": op["offset_x"],
                    "offset_y": op["offset_y"],
                    "offset_z": op["offset_z"]}
            axes[axis] = float(v)
            self._scene.set_operation_offset(
                node_id, axes["offset_x"], axes["offset_y"], axes["offset_z"])
            self._node_params[node_id][axis] = float(v)
            self._refresh_operation_rows()
            self._refresh_viewport()
            self._mark_modified()

        old_value = float(op[axis])
        if self._command_stack:
            self._command_stack.push(
                lambda: apply(value), lambda: apply(old_value),
                f"Set {axis} = {value}")
        else:
            apply(value)
        self._status.showMessage(f"Set {axis} = {value}", 2000)

    def _push_extrude_distance_command(self, node_id: int,
                                        distance: float) -> None:
        """Undoable distance edit for an extrude operation node: the
        recipe is re-created with the SAME id (restore-style — the
        references never rebind) and the DAG pass re-materializes the
        prism and its dependents. The kernel's d > 0 check refuses a
        non-positive distance visibly, nothing written."""
        node = self._scene.get_node(node_id)
        op = node.operation if node is not None else None
        if op is None or op.get("type") != "extrude":
            return
        if not (distance > 0.0) or distance != distance:  # <= 0 or NaN
            self._status.showMessage(
                f"distance must be finite and > 0, got {distance} — "
                "the previous value stands", 4000)
            return
        sketch_id = op["sketch"]
        direction = op["direction"]
        name = node.name
        old_distance = float(op["distance"])

        def apply(d: float) -> None:
            self._scene.remove_node(node_id)
            self._scene.restore_extrude_node(
                node_id, name, sketch_id, d, direction)
            self._scene.rebuild_operations()
            self._node_params[node_id] = {"distance": float(d)}
            self._refresh_operation_rows()
            self._refresh_viewport()
            self._mark_modified()

        if self._command_stack:
            self._command_stack.push(
                lambda: apply(distance), lambda: apply(old_distance),
                f"Set distance = {distance}")
        else:
            apply(distance)
        self._status.showMessage(f"Set distance = {distance}", 2000)

    def _do_restore_param(self, node_id: int, name: str, old_value: float) -> None:
        """Legacy undo callback — kept for compatibility, unused after refactor."""
        if node_id in self._node_params:
            self._node_params[node_id][name] = old_value
            self._rebuild_node(node_id)

    def _node_pick_bounds(self, node):
        """World-space AABB used by viewport picking, as ((x,y,z) min,
        (x,y,z) max) — or None when the node has no pickable geometry.

        SDF nodes read the kernel's body bounds; mesh-kind nodes
        (imported STL/OBJ) have no SDF body, so their bounds come from
        the mesh's own vertex positions (cached tessellation first, the
        raw payload as fallback).
        """
        if node.kind != "mesh":
            b = node.body.bounds()
            return ((b.min.x, b.min.y, b.min.z),
                    (b.max.x, b.max.y, b.max.z))
        try:
            import numpy as _np
            pos = None
            for cand in (getattr(node, "cached_mesh", None),
                         getattr(node, "mesh", None)):
                if cand is None:
                    continue
                cand_pos = getattr(cand, "positions", None)
                if cand_pos is not None and len(cand_pos) > 0:
                    pos = cand_pos
                    break
            if pos is None:
                return None
            arr = _np.asarray(pos, dtype=float)
            if arr.ndim != 2 or arr.shape[0] == 0:
                return None
            lo = arr.min(axis=0)
            hi = arr.max(axis=0)
            return ((float(lo[0]), float(lo[1]), float(lo[2])),
                    (float(hi[0]), float(hi[1]), float(hi[2])))
        except (ImportError, ValueError, TypeError):
            return None

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

        # Ray-bbox intersection for each visible node. Mesh-kind nodes
        # (imported STL/OBJ) have no SDF body: their pick bounds come from
        # the mesh's own vertex positions (see _node_pick_bounds), so a
        # click on imported geometry selects it like any SDF shape.
        best_node = None
        best_t = float('inf')
        for i in range(self._scene.node_count()):
            node = self._scene.node_at(i)
            if node is None or not node.visible:
                continue
            pick = self._node_pick_bounds(node)
            if pick is None:
                continue
            bmin, bmax = pick
            # Simple slab-based ray-AABB test
            tmin = -float('inf')
            tmax = float('inf')
            hit = True
            for axis in range(3):
                o = [cam_pos[0], cam_pos[1], cam_pos[2]][axis]
                d = ray_dir[axis]
                lo, hi = bmin[axis], bmax[axis]
                if abs(d) < 1e-9:
                    if o < lo or o > hi:
                        hit = False
                        break
                else:
                    t1 = (lo - o) / d
                    t2 = (hi - o) / d
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
        else:
            # The ray hit nothing: a click on empty space deselects
            # whatever is selected (mirrors every major CAD tool —
            # clicking the void is how you drop a selection).
            self._clear_selection()


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
