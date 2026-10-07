"""CAD_0.ui.feature_tree — Feature tree panel.

A QTreeView that displays the CAD_0 scene's feature tree. Each node
is a `Shape` (see core/geometry/shape.hpp) with a representation kind
(SDF / BRep / Hybrid).

Phase C.7: written but not exercised in CI. Requires PySide6.

The model wraps a Python-side list of `FeatureNode` dataclass entries.
In a full build, these would be backed by the C++ `Shape` class via
nanobind; for now we use a Python dataclass so the panel can be tested
in isolation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from PySide6.QtCore import Qt, QAbstractItemModel, QModelIndex, QPointF
from PySide6.QtGui import QColor, QPainter, QPalette
from PySide6.QtWidgets import (
    QAbstractItemView,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTreeView,
    QWidget,
)


@dataclass
class FeatureNode:
    """A node in the feature tree."""

    name: str
    representation: str = "SDF"  # "SDF" | "BRep" | "Hybrid"
    visible: bool = True
    selected: bool = False
    children: list[FeatureNode] = field(default_factory=list)
    # Parent pointer, set when the node is added to a parent's children.
    parent: Optional[FeatureNode] = None
    # Parameters for the property panel (Phase C.7).
    parameters: dict[str, float | str | bool] = field(default_factory=dict)


class FeatureTreeModel(QAbstractItemModel):
    """Qt model that exposes the feature tree to a QTreeView.

    Columns:
        0: Name (with visibility checkbox + representation icon)
        1: Representation (SDF / BRep / Hybrid)
    """

    def __init__(self, root: FeatureNode, parent: QWidget = None) -> None:
        super().__init__(parent)
        self._root = root

    def index(self, row: int, column: int, parent: QModelIndex = QModelIndex()) -> QModelIndex:
        if not self.hasIndex(row, column, parent):
            return QModelIndex()
        parent_node = self._node(parent) or self._root
        if 0 <= row < len(parent_node.children):
            child = parent_node.children[row]
            return self.createIndex(row, column, child)
        return QModelIndex()

    def parent(self, index: QModelIndex) -> QModelIndex:
        if not index.isValid():
            return QModelIndex()
        node = self._node(index)
        if node is None or node.parent is None or node.parent is self._root:
            return QModelIndex()
        # Find the parent's row in its own parent's children list.
        grandparent = node.parent.parent or self._root
        for i, sibling in enumerate(grandparent.children):
            if sibling is node.parent:
                return self.createIndex(i, 0, node.parent)
        return QModelIndex()

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        node = self._node(parent) or self._root
        return len(node.children)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 2

    def data(self, index: QModelIndex, role: int = Qt.DisplayRole):
        if not index.isValid():
            return None
        node = self._node(index)
        if node is None:
            return None
        if role == Qt.DisplayRole:
            if index.column() == 0:
                return node.name
            elif index.column() == 1:
                return node.representation
        elif role == Qt.CheckStateRole and index.column() == 0:
            return Qt.Checked if node.visible else Qt.Unchecked
        elif role == Qt.UserRole:
            return node  # expose the node for the property panel
        elif role == Qt.ToolTipRole:
            return f"{node.name} ({node.representation})"
        return None

    def setData(self, index: QModelIndex, value, role: int = Qt.EditRole) -> bool:
        if not index.isValid():
            return False
        node = self._node(index)
        if node is None:
            return False
        if role == Qt.CheckStateRole and index.column() == 0:
            node.visible = bool(value)
            self.dataChanged.emit(index, index, [role])
            return True
        if role == Qt.EditRole and index.column() == 0:
            node.name = str(value)
            self.dataChanged.emit(index, index, [role])
            return True
        return False

    def flags(self, index: QModelIndex) -> Qt.ItemFlags:
        if not index.isValid():
            return Qt.NoItemFlags
        flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable
        if index.column() == 0:
            flags |= Qt.ItemIsUserCheckable | Qt.ItemIsEditable
        return flags

    def headerData(self, section: int, orientation: Qt.Orientation, role: int = Qt.DisplayRole):
        if role != Qt.DisplayRole or orientation != Qt.Horizontal:
            return None
        return ["Name", "Type"][section] if section in (0, 1) else None

    # ----- Helpers -----

    def _node(self, index: QModelIndex) -> Optional[FeatureNode]:
        if not index.isValid():
            return None
        return index.internalPointer()

    def add_node(self, parent_index: QModelIndex, node: FeatureNode) -> None:
        parent = self._node(parent_index) or self._root
        node.parent = parent
        row = len(parent.children)
        self.beginInsertRows(parent_index, row, row)
        parent.children.append(node)
        self.endInsertRows()

    def remove_node(self, index: QModelIndex) -> bool:
        node = self._node(index)
        if node is None or node.parent is None:
            return False
        parent = node.parent
        row = parent.children.index(node)
        self.beginRemoveRows(self.parent(index), row, row)
        parent.children.pop(row)
        self.endRemoveRows()
        return True


class FeatureTreePanel(QTreeView):
    """The actual tree view widget."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setHeaderHidden(False)
        self.setUniformRowHeights(True)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setEditTriggers(QAbstractItemView.DoubleClicked)
        self.setAnimated(True)
        # Default model with an empty root.
        self.setModel(FeatureTreeModel(FeatureNode(name="<root>")))
