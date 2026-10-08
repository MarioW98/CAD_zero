"""CAD_0.ui.property_panel — Property panel for the selected feature.

Displays the parameters of the currently-selected `FeatureNode` from
the feature tree. Allows inline editing of numeric, string, and bool
parameters.

Phase C.7: written but not exercised in CI. Requires PySide6.

The panel uses QFormLayout — label on the left, editor on the right.
When the selection changes, the panel rebuilds itself from the new
node's `parameters` dict.
"""

from __future__ import annotations

from typing import Optional, Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSpinBox,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


class PropertyPanel(QWidget):
    """Displays editable parameters for the currently-selected feature."""

    # Emitted when a parameter value is changed by the user.
    # Arguments: (parameter_name, new_value)
    parameter_changed = Signal(str, object)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._current_node_name: Optional[str] = None
        self._setup_ui()

    def _setup_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)

        # Header: name of the currently selected feature.
        self._header = QLabel("No selection")
        self._header.setStyleSheet("font-weight: bold; padding: 4px;")
        outer.addWidget(self._header)

        # Form layout with one row per parameter.
        self._form = QFormLayout()
        self._form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._form.setHorizontalSpacing(12)
        self._form.setVerticalSpacing(8)
        outer.addLayout(self._form)

        outer.addStretch(1)

    def set_parameters(self, node_name: str, parameters: dict[str, Any]) -> None:
        """Replace the displayed parameters with the given ones."""
        self._current_node_name = node_name
        self._header.setText(node_name)

        # Clear the existing form.
        while self._form.rowCount() > 0:
            self._form.removeRow(0)

        for name, value in parameters.items():
            label = QLabel(name)
            editor = self._make_editor(name, value)
            self._form.addRow(label, editor)

    def clear(self) -> None:
        self._current_node_name = None
        self._header.setText("No selection")
        while self._form.rowCount() > 0:
            self._form.removeRow(0)

    # ----- Internal helpers -----

    def _make_editor(self, name: str, value: Any) -> QWidget:
        """Create the appropriate editor widget for the given value type."""
        if isinstance(value, bool):
            w = QCheckBox()
            w.setChecked(value)
            w.toggled.connect(lambda checked, n=name: self._on_value_changed(n, checked))
            return w
        if isinstance(value, int) and not isinstance(value, bool):
            w = QSpinBox()
            w.setRange(-2**31, 2**31 - 1)
            w.setValue(value)
            w.valueChanged.connect(lambda v, n=name: self._on_value_changed(n, v))
            return w
        if isinstance(value, float):
            w = QDoubleSpinBox()
            w.setRange(-1e9, 1e9)
            w.setDecimals(6)
            w.setSingleStep(0.1)
            w.setValue(value)
            w.valueChanged.connect(lambda v, n=name: self._on_value_changed(n, v))
            return w
        if isinstance(value, str):
            w = QLineEdit(str(value))
            w.textChanged.connect(lambda t, n=name: self._on_value_changed(n, t))
            return w

        # Fallback: read-only text representation.
        w = QLabel(str(value))
        w.setStyleSheet("color: gray;")
        return w

    def _on_value_changed(self, name: str, value: Any) -> None:
        self.parameter_changed.emit(name, value)
