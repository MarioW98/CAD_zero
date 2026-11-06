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

from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QFontMetrics, QPainter
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

# The bound the panel's read-only VALUE column lives in (G2-u3
# m6-correttiva, the click-test's defect 2): the longest golden name
# — fil#5:isect[ext#3:lat(0) × fil#5:flank[box#1:top ×
# ext#3:lat(0)]], 66 characters — renders inside it, elided with the
# full string in the tooltip; a value NEVER widens the window.
READONLY_VALUE_BOUND_PX = 280


class _BoundValueLabel(QLabel):
    """A read-only value label that LIVES INSIDE THE PANEL (G2-u3
    m6-correttiva, the click-test's defect 2: the long born-name rows
    did not wrap and the window widened, eating the work area).

    The DATA is the full string — ``text()`` returns it whole (the m5
    pins, "character for character", keep reading it); the RENDER
    never demands more than the bound:

      * a value that fits the bound renders as before (the counts, the
        volume — the m5 rows' own shape);
      * a value whose LONGEST WORD fits the bound WRAPS (multi-line,
        everything readable — the refusal rows, the error rows, the
        constraint renders: the m5 surfaces keep their readability);
      * a value with a word longer than the bound (the born names, the
        reference string — single tokens the wrapper cannot break)
        renders ELIDED at the available width, the full string in the
        tooltip.

    ``sizeHint``/``minimumSizeHint`` are bounded: the QFormLayout's
    minimum width never grows with the content, so the dock — and the
    window — never widens for a value (the work area is never eaten).
    """

    def __init__(self, text, parent=None):
        super().__init__(str(text), parent)
        self._full_text = str(text)
        self.setToolTip(self._full_text)
        self.setTextInteractionFlags(Qt.TextSelectableByMouse)
        fm = self.fontMetrics()
        full_px = fm.horizontalAdvance(self._full_text)
        longest_word_px = max(
            (fm.horizontalAdvance(w)
             for w in self._full_text.split(" ")),
            default=0)
        if full_px <= READONLY_VALUE_BOUND_PX:
            self._mode = "plain"
        elif longest_word_px <= READONLY_VALUE_BOUND_PX:
            self._mode = "wrap"
            self.setWordWrap(True)
        else:
            # The token the wrapper cannot break: elide at the width
            # the panel actually gives the row, tooltip whole.
            self._mode = "elide"

    # ---- The bounded geometry (the size policy that forbids the
    # widening): the layout's demands are capped by the bound, the
    # minimum lets the row shrink instead of pushing the window.
    def sizeHint(self):
        fm = self.fontMetrics()
        if self._mode == "wrap":
            lines = self._wrapped_lines(READONLY_VALUE_BOUND_PX)
            return QSize(READONLY_VALUE_BOUND_PX,
                         fm.lineSpacing() * len(lines) + 4)
        if self._mode == "elide":
            shown = fm.elidedText(self._full_text, Qt.ElideRight,
                                  READONLY_VALUE_BOUND_PX)
            return QSize(fm.horizontalAdvance(shown),
                         fm.height() + 4)
        return QLabel.sizeHint(self)

    def minimumSizeHint(self):
        fm = self.fontMetrics()
        if self._mode == "wrap":
            longest = max(
                (fm.horizontalAdvance(w)
                 for w in self._full_text.split(" ")),
                default=0)
            return QSize(min(longest, READONLY_VALUE_BOUND_PX),
                         fm.lineSpacing() + 4)
        # plain and elide: the row can shrink to nothing — the elide
        # adapts on paint, the plain text clips like any label.
        return QSize(0, fm.height() + 4)

    def _wrapped_lines(self, bound):
        fm = self.fontMetrics()
        lines = []
        current = ""
        for word in self._full_text.split(" "):
            trial = word if not current else current + " " + word
            if fm.horizontalAdvance(trial) <= bound or not current:
                current = trial
            else:
                lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines or [""]

    def paintEvent(self, event):
        if self._mode != "elide":
            super().paintEvent(event)
            return
        painter = QPainter(self)
        fm = self.fontMetrics()
        shown = fm.elidedText(self._full_text, Qt.ElideRight,
                              max(10, self.width()))
        painter.drawText(self.rect(),
                         int(self.alignment()) | int(Qt.AlignVCenter),
                         shown)


class PropertyPanel(QWidget):
    """Displays editable parameters for the currently-selected feature.

    Commit semantics (the typed-parameter use-after-free, D5.2): every
    editor commits ONCE per completed modification. The numeric fields
    (QSpinBox/QDoubleSpinBox) run with keyboardTracking DISABLED — the
    default emits valueChanged on EVERY typed digit, which meant a
    half-typed "15" wrote the recipe twice (two undo entries) and
    rebuilt the panel while the spinbox's own keyPressEvent was still
    on the stack (the crash: QLineEdit::text() on a freed object,
    QAbstractSpinBox::keyPressEvent in the user's faulthandler stack).
    The string field (QLineEdit) commits on editingFinished (Return or
    focus-out), not per keystroke: one completed edit = one write =
    one undo entry.

    Refresh semantics: ``set_parameters`` rebuilds the rows (it
    destroys editors — safe when the event owner is another widget,
    fatal when the recipe echo lands while an editor is still inside
    its event handler); ``update_values`` refreshes the editors'
    VALUES in place, destroying nothing — the echo path after a
    parameter write, where the committing editor must stay alive.
    """

    # Emitted when a parameter value is changed by the user.
    # Arguments: (parameter_name, new_value)
    parameter_changed = Signal(str, object)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._current_node_name: Optional[str] = None
        # name -> editor widget, for the in-place value refresh
        # (update_values); emptied by every row rebuild.
        self._editors: dict[str, QWidget] = {}
        # name -> {"last": committed text}, the string editors'
        # dedup state (editingFinished fires on Return AND focus-out:
        # the second fire of the same edit must not re-commit).
        self._editor_state: dict[str, dict] = {}
        # True while update_values writes programmatic values: a
        # refresh is not a user edit — no parameter_changed echo.
        self._updating = False
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
        """Replace the displayed parameters with the given ones.

        This REBUILDS the rows (QFormLayout::removeRow deletes the old
        editors synchronously) — never call it from a path that may
        run while a panel editor is still processing its own event
        (the recipe echo after a parameter write goes through
        update_values for exactly that reason).
        """
        self._current_node_name = node_name
        self._header.setText(node_name)
        self._editors = {}
        self._editor_state = {}

        # Clear the existing form.
        while self._form.rowCount() > 0:
            self._form.removeRow(0)

        for name, value in parameters.items():
            label = QLabel(name)
            editor = self._make_editor(name, value)
            self._form.addRow(label, editor)
            self._editors[name] = editor

    def set_readonly_fields(self, node_name: str, fields: dict[str, str]) -> None:
        """Display read-only informational fields (no editors).

        Used for "mesh"-kind nodes (imported STL/OBJ geometry): the
        property panel shows counts and bounds — there is nothing
        parametric to edit (the geometry is data, not a recipe).
        """
        self._current_node_name = node_name
        self._header.setText(node_name)
        self._editors = {}
        self._editor_state = {}

        while self._form.rowCount() > 0:
            self._form.removeRow(0)

        for name, value in fields.items():
            label = QLabel(name)
            text = _BoundValueLabel(str(value))
            self._form.addRow(label, text)

    def set_parameters_and_readonly(
            self, node_name: str, parameters: dict[str, Any],
            readonly_fields: dict[str, str]) -> None:
        """Editable parameters PLUS trailing read-only fields.

        Used for operation nodes (ADR-0027 D5): the placement offsets
        are editable parameters; the operand summary (names + ids) and
        any dangling-reference error are read-only context below them.
        """
        self.set_parameters(node_name, parameters)
        for name, value in readonly_fields.items():
            label = QLabel(name)
            text = _BoundValueLabel(str(value))
            self._form.addRow(label, text)

    def clear(self) -> None:
        self._current_node_name = None
        self._header.setText("No selection")
        self._editors = {}
        self._editor_state = {}
        while self._form.rowCount() > 0:
            self._form.removeRow(0)

    # ----- Internal helpers -----

    def current_label(self) -> Optional[str]:
        """The label the panel is showing right now (None when
        cleared) — the caller's test for "same surface, refresh the
        values in place" versus "different surface, rebuild"."""
        return self._current_node_name

    def update_values(self, parameters: dict[str, Any]) -> None:
        """Refresh the editors' VALUES in place — no row is created or
        destroyed, no editor is deleted.

        The recipe echo after a parameter write (the canvas re-routes
        the selected entity's parameters to the panel) lands here: the
        editor that is COMMITTING that very change may still be inside
        its own keyPressEvent/focus-out handler, and a row rebuild
        here would delete it under its own feet (the use-after-free).
        The programmatic writes are guarded: they emit no
        parameter_changed, so the echo cannot cascade into a second
        write or a second undo entry.
        """
        self._updating = True
        try:
            for name, value in parameters.items():
                w = self._editors.get(name)
                if w is None:
                    continue  # a field this surface does not carry
                if isinstance(w, QCheckBox):
                    w.setChecked(bool(value))
                elif isinstance(w, QDoubleSpinBox):
                    if isinstance(value, (int, float)) and \
                            not isinstance(value, bool):
                        w.setValue(float(value))
                elif isinstance(w, QSpinBox):
                    if isinstance(value, (int, float)) and \
                            not isinstance(value, bool):
                        w.setValue(int(value))
                elif isinstance(w, QLineEdit):
                    text = str(value)
                    w.setText(text)
                    state = self._editor_state.get(name)
                    if state is not None:
                        # The dedup anchor follows the recipe: a later
                        # focus-out without further typing is not an
                        # edit and must not re-commit this value.
                        state["last"] = text
        finally:
            self._updating = False

    def _make_editor(self, name: str, value: Any) -> QWidget:
        """Create the appropriate editor widget for the given value type.

        Every editor is COMMIT-ONLY (see the class docstring): signals
        fire on a completed modification (Return, arrows, focus-out),
        never per keystroke.
        """
        if isinstance(value, bool):
            w = QCheckBox()
            w.setChecked(value)
            w.toggled.connect(lambda checked, n=name: self._on_value_changed(n, checked))
            return w
        if isinstance(value, int) and not isinstance(value, bool):
            w = QSpinBox()
            w.setRange(-2**31, 2**31 - 1)
            # Commit-only: with the default keyboardTracking every
            # typed digit would emit valueChanged — a half-typed
            # number is not a modification (D5.2: one completed edit
            # = one write = one undo entry).
            w.setKeyboardTracking(False)
            w.setValue(value)
            w.valueChanged.connect(lambda v, n=name: self._on_value_changed(n, v))
            return w
        if isinstance(value, float):
            w = QDoubleSpinBox()
            w.setRange(-1e9, 1e9)
            w.setDecimals(6)
            w.setSingleStep(0.1)
            # Commit-only, same rule as the int field — this is the
            # radius field of the crash: valueChanged now fires when
            # the edit is DONE, not while it is being typed.
            w.setKeyboardTracking(False)
            w.setValue(value)
            w.valueChanged.connect(lambda v, n=name: self._on_value_changed(n, v))
            return w
        if isinstance(value, str):
            w = QLineEdit(str(value))
            # Commit-only: editingFinished (Return or focus-out)
            # instead of textChanged (every keystroke — the string
            # twin of the spinbox mine, verified per-keystroke before
            # this fix). The "last committed" anchor dedupes the
            # Return-then-focus-out double fire of ONE edit.
            state = {"last": str(value)}

            def _commit_string(n=name, edit=w, st=state) -> None:
                text = edit.text()
                if text != st["last"]:
                    st["last"] = text
                    self._on_value_changed(n, text)

            w.editingFinished.connect(_commit_string)
            self._editor_state[name] = state
            return w

        # Fallback: read-only text representation.
        w = QLabel(str(value))
        w.setStyleSheet("color: gray;")
        return w

    def _on_value_changed(self, name: str, value: Any) -> None:
        if self._updating:
            # A programmatic in-place refresh echoing the recipe back
            # into the editors — not a user edit, no write, no undo.
            return
        self.parameter_changed.emit(name, value)
