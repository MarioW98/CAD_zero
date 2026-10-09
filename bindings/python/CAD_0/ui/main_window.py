"""CAD_0.ui.main_window — Application main window with working scene creation.

Now connects to the C++ Scene kernel and renders meshes in the viewport.
"""

from __future__ import annotations

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
        self.resize(1280, 800)

        # Try to import the C++ kernel
        try:
            # The kernel might be in a different CAD_0 package (bindings/python/)
            # Try importing from the current package first, then from a
            # separate CAD_0 package on sys.path.
            try:
                import CAD_0 as _kernel_pkg
                # Check if this package has the compiled extension
                if not hasattr(_kernel_pkg, '_CAD_0'):
                    raise ImportError("No _CAD_0 in this package")
            except ImportError:
                pass

            # Force reimport with the right sys.path
            import importlib
            if '_CAD_0' not in sys.modules:
                # Try to find _CAD_0 on sys.path
                import importlib.util
                spec = importlib.util.find_spec('_CAD_0')
                if spec is None:
                    raise ImportError("_CAD_0 compiled extension not found")

            import CAD_0
            self._cpp = CAD_0
            self._scene = CAD_0.scene.Scene()
            self._command_stack = CAD_0.commands.CommandStack(1000)
            self._has_kernel = True
        except (ImportError, AttributeError) as e:
            self._cpp = None
            self._scene = None
            self._command_stack = None
            self._has_kernel = False
            print(f"Warning: CAD_0 kernel not available: {e}")
            print("  Set PYTHONPATH to include the build/python directory.")

        # Track shape parameters and types for interactive editing.
        self._node_params = {}  # node_id → {param_name: value}
        self._node_types = {}   # node_id → "sphere"|"box"|"cylinder"|"torus"

        self._setup_central_widget()
        self._setup_dock_widgets()
        self._setup_menus()
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
        create_menu = self.menuBar().addMenu("&Create")

        create_sphere = QAction("Create &Sphere", self)
        create_sphere.triggered.connect(self.on_create_sphere)
        create_menu.addAction(create_sphere)

        create_box = QAction("Create &Box", self)
        create_box.triggered.connect(self.on_create_box)
        create_menu.addAction(create_box)

        create_cylinder = QAction("Create &Cylinder", self)
        create_cylinder.triggered.connect(self.on_create_cylinder)
        create_menu.addAction(create_cylinder)

        create_torus = QAction("Create &Torus", self)
        create_torus.triggered.connect(self.on_create_torus)
        create_menu.addAction(create_torus)

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

    # ----- Slots -----

    def on_new(self) -> None:
        if self._scene is not None:
            self._scene.clear()
        self._node_params.clear()
        self._node_types.clear()
        self._feature_tree.clear()
        self._property_panel.clear()
        self._refresh_viewport()
        self._status.showMessage("New scene", 3000)

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

    def on_create_sphere(self) -> None:
        if not self._has_kernel:
            return
        import CAD_0
        params = {"radius": 1.0}
        body = CAD_0.sdf.sphere(radius=1.0)
        node_id = self._scene.add_node("Sphere", body)
        self._node_params[node_id] = params
        self._node_types[node_id] = "sphere"
        item = QTreeWidgetItem(["Sphere", "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)

        # Register undo: remove the node we just added
        if self._command_stack:
            self._command_stack.push(
                lambda: None,  # already executed (node added above)
                lambda nid=node_id: self._do_remove_node(nnid=nid),
                "Create Sphere"
            )
        self._refresh_viewport()
        self._status.showMessage(f"Created Sphere (id={node_id})", 3000)

    def on_create_box(self) -> None:
        if not self._has_kernel:
            return
        import CAD_0
        params = {"extent_x": 0.5, "extent_y": 0.5, "extent_z": 0.5}
        body = CAD_0.sdf.box(CAD_0.math.Vec3f(0.5, 0.5, 0.5))
        node_id = self._scene.add_node("Box", body)
        self._node_params[node_id] = params
        self._node_types[node_id] = "box"
        item = QTreeWidgetItem(["Box", "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        self._status.showMessage(f"Created Box (id={node_id})", 3000)

    def on_create_cylinder(self) -> None:
        if not self._has_kernel:
            return
        import CAD_0
        params = {"radius": 0.5, "height": 2.0}
        body = CAD_0.sdf.cylinder(radius=0.5, height=2.0)
        node_id = self._scene.add_node("Cylinder", body)
        self._node_params[node_id] = params
        self._node_types[node_id] = "cylinder"
        item = QTreeWidgetItem(["Cylinder", "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        self._status.showMessage(f"Created Cylinder (id={node_id})", 3000)

    def on_create_torus(self) -> None:
        if not self._has_kernel:
            return
        import CAD_0
        params = {"major_radius": 1.0, "minor_radius": 0.3}
        body = CAD_0.sdf.torus(R=1.0, r=0.3)
        node_id = self._scene.add_node("Torus", body)
        self._node_params[node_id] = params
        self._node_types[node_id] = "torus"
        item = QTreeWidgetItem(["Torus", "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        self._status.showMessage(f"Created Torus (id={node_id})", 3000)

    def _rebuild_node(self, node_id: int) -> None:
        """Rebuild an SDF body from its stored params and type."""
        import CAD_0
        if node_id not in self._node_params:
            return
        params = self._node_params[node_id]
        shape_type = self._node_types.get(node_id, "")
        if shape_type == "sphere":
            r = params.get("radius", 1.0)
            body = CAD_0.sdf.sphere(radius=r)
        elif shape_type == "box":
            ex = params.get("extent_x", 0.5)
            ey = params.get("extent_y", 0.5)
            ez = params.get("extent_z", 0.5)
            body = CAD_0.sdf.box(CAD_0.math.Vec3f(ex, ey, ez))
        elif shape_type == "cylinder":
            r = params.get("radius", 0.5)
            h = params.get("height", 2.0)
            body = CAD_0.sdf.cylinder(radius=r, height=h)
        elif shape_type == "torus":
            R = params.get("major_radius", 1.0)
            r = params.get("minor_radius", 0.3)
            body = CAD_0.sdf.torus(R=R, r=r)
        else:
            return
        node = self._scene.get_node(node_id)
        if node:
            node.body = body
            node.mesh_dirty = True
            self._refresh_viewport()

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
        # Export first visible mesh
        for node in self._scene.nodes():
            if node.visible and node.cached_mesh.vertex_count() > 0:
                self._cpp.io.export(path, node.cached_mesh)
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
        # Store info for undo (re-add)
        shape_type = self._node_types.get(node_id, "")
        params = dict(self._node_params.get(node_id, {}))

        if self._scene and self._scene.remove_node(node_id):
            self._node_params.pop(node_id, None)
            self._node_types.pop(node_id, None)
            self._feature_tree.takeTopLevelItem(self._feature_tree.indexOfTopLevelItem(item))
            self._property_panel.clear()

            # Register undo: re-add the node
            if self._command_stack:
                self._command_stack.push(
                    lambda: None,  # already executed (node removed)
                    lambda nid=node_id, st=shape_type, p=params: self._do_re_add_node(nid, st, p),
                    f"Delete {shape_type}"
                )
            self._refresh_viewport()
            self._status.showMessage(f"Deleted node {node_id}", 2000)

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
        """Undo callback: re-add a deleted node."""
        import CAD_0
        if shape_type == "sphere":
            r = params.get("radius", 1.0)
            body = CAD_0.sdf.sphere(radius=r)
            name = "Sphere"
        elif shape_type == "box":
            body = CAD_0.sdf.box(CAD_0.math.Vec3f(
                params.get("extent_x", 0.5),
                params.get("extent_y", 0.5),
                params.get("extent_z", 0.5)))
            name = "Box"
        elif shape_type == "cylinder":
            body = CAD_0.sdf.cylinder(
                radius=params.get("radius", 0.5),
                height=params.get("height", 2.0))
            name = "Cylinder"
        elif shape_type == "torus":
            body = CAD_0.sdf.torus(
                R=params.get("major_radius", 1.0),
                r=params.get("minor_radius", 0.3))
            name = "Torus"
        else:
            return
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
        # Show the node's parameters in the property panel.
        params = self._node_params.get(node_id, {})
        # Convert to dict[str, float|str|bool] for PropertyPanel
        display_params = {}
        for k, v in params.items():
            display_params[k] = float(v) if isinstance(v, (int, float)) else str(v)
        self._property_panel.set_parameters(node.name, display_params)
        # Store the selected node_id for the parameter_changed callback
        self._selected_node_id = node_id

    def _on_parameter_changed(self, name: str, value) -> None:
        """Called when the user edits a parameter in the property panel."""
        node_id = getattr(self, '_selected_node_id', None)
        if node_id is None or node_id not in self._node_params:
            return
        old_value = self._node_params[node_id].get(name, 0.0)
        self._node_params[node_id][name] = float(value)
        # Register undo: restore old value
        if self._command_stack:
            self._command_stack.push(
                lambda: None,  # already applied
                lambda nid=node_id, n=name, ov=old_value: self._do_restore_param(nid, n, ov),
                f"Set {name} = {value}"
            )
        self._rebuild_node(node_id)
        self._status.showMessage(f"Set {name} = {value}", 2000)

    def _do_restore_param(self, node_id: int, name: str, old_value: float) -> None:
        """Undo callback: restore a parameter to its old value."""
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
        for node in self._scene.nodes():
            if not node.visible:
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
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
