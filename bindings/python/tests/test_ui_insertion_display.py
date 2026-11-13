"""The insertion gesture's display path — the 2026-11-13 correttiva's
own suite.

The maintainer's click-test (2026-11-13, the PERF 1/PERF 2 landing
day): after Create -> primitive from the interface, the solid does not
appear in the viewport. The data half is intact — the feature tree
gains its row, the save round-trips, the node's cached mesh is derived
(the C++ dirty-flag discipline covers insertions: add_node,
restore_sdf_node and the loader all hand the node over dirty, and
update_meshes tessellates it). The defect is one link lower, on the
display road: the frame's mesh pass issues its glDrawElements with a
byte-offset argument PERF 1 wrote as a bare Python int — and
PyOpenGL's glDrawElements wrapper does NOT read a bare int as a
buffer offset. Its `indices` parameter goes through AsArrayOfType
(OpenGL/GL/pointers.py), which converts a Python int to a ctypes
c_uint scalar and passes ITS heap address as the pointer: with an
ELEMENT_ARRAY_BUFFER bound, that address becomes the offset into the
buffer — a wild offset far beyond its size. The fetch is undefined;
on software GL (and on the maintainer's card) it yields nothing. The
grids and the axes draw (they pass None — the offset-0 form), the
mirror is populated, the digests agree, and the viewport stays empty:
a silence the recorder-based PERF 1 suite could not see, because the
recorder COUNTS the draw call and never asks what GL would do with
its arguments — and the timing rite measures the calls, never the
pixels. The insertion gesture was the one act no suite ever drove
end to end with a real context and eyes on the framebuffer; this
file closes that gap four ways:

* THE DATA HALF (headless, every column): the Create handler's own
  sequence populates the draw mirror — the guard that keeps the
  intact half intact while the display road is repaired under it.

* THE DRAW-CALL CONTRACT (headless, every column — the reproduction
  at the unit boundary, red on the defective tip): the mesh pass's
  glDrawElements must receive its indices argument in the byte-offset
  form GL interprets against the bound IBO — None (offset 0) or a
  ctypes.c_void_p holding the offset — never a bare Python int, the
  form PyOpenGL silently turns into a client heap pointer. This is
  the pin that catches the class of the defect on every column of
  the matrix, sanitizer runs included: it needs no GL at all, only
  the recorded argument.

* THE CLICK-TEST'S OWN VERDICT (real GL — the Xvfb step, the same
  posture as the navigation/startup GL tests): the Create gesture,
  a real frame, the framebuffer read back — the inserted solid's
  pixels must be there, distinct from the background, the grid and
  the axes. The pixels, not the calls: the difference between a
  frame that ISSUES a draw and a frame that SHOWS one is precisely
  the distance between PERF 1's green suite and the maintainer's
  red screen.

* THE DECLARED DIAGNOSTIC (headless, every column): a visible node
  whose display mesh is empty after the tessellation pass is never
  skipped in silence again — the refresh names it on the status log
  (the [viewport] tag, the module's own convention). The case is
  unreachable for valid nodes (the C++ discipline hands every
  insertion over dirty); when a defect manufactures it anyway
  (a discipline bug, a degenerate body, a corrupted payload), the
  disappearance is accompanied by its reason. Sketch nodes are
  exempt by design: 2D data, no 3D mesh, never a defect (ADR-0030
  D2).
"""

from __future__ import annotations

import ctypes
from pathlib import Path

import pytest


_ROOT = Path(__file__).resolve().parents[3]


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# The fixtures — the workbench suite's own offscreen pattern (the modal
# prompts answered Discard, a headless box never blocks).
# ---------------------------------------------------------------------------

@pytest.fixture()
def main_window(monkeypatch):
    if not _pyside6_available():
        pytest.skip("PySide6 not installed")
    pytest.importorskip("CAD_0")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QMessageBox
    QMessageBox.question = staticmethod(
        lambda *a, **k: QMessageBox.Discard)
    app = QApplication.instance() or QApplication([])
    from CAD_0.ui.main_window import MainWindow

    window = MainWindow()
    window._current_path = None
    yield window
    # The navigation fixture's own teardown discipline: flush Qt's
    # deferred deletes so the C++ widget dies before the interpreter.
    from PySide6.QtCore import QCoreApplication, QEvent
    window.close()
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    del window
    import gc
    gc.collect()


def _create_box(window) -> int:
    """The Create -> Box gesture, the menu handler itself."""
    window.on_create_box()
    nodes = window._scene.node_count()
    assert nodes >= 1, "the Create gesture must add its node to the scene"
    return window._scene.node_at(nodes - 1).id


# ---------------------------------------------------------------------------
# THE DATA HALF — the mirror populated (the guard for the intact half).
# ---------------------------------------------------------------------------

def test_the_insertion_gesture_populates_the_draw_mirror(main_window):
    """Create -> Box: the node's mesh is derived and mirrored.

    The same sequence the menu handler runs — Scene.add_node with the
    registry's default body, then the viewport refresh — must leave
    the node with a non-empty cached mesh AND a draw-mirror entry
    carrying it. This was green on the defective tip (the data half
    was never broken) and stays green: the correttiva repairs the
    display road under it, never by weakening what the mirror holds.
    """
    vp = main_window._viewport
    _create_box(main_window)

    node = main_window._scene.node_at(0)
    assert len(node.cached_mesh.positions) > 0, \
        "the inserted node's tessellation cache must be derived"
    assert len(node.cached_mesh.indices) > 0

    entries = [e for e in vp._draw_nodes if e.node_id == node.id]
    assert len(entries) == 1, (
        "the draw mirror must carry exactly one entry for the "
        f"inserted node (has {len(entries)} for id {node.id})")
    assert entries[0].triangle_count > 0, \
        "the mirror entry must carry triangles"


# ---------------------------------------------------------------------------
# THE DRAW-CALL CONTRACT — the offset form, pinned at the unit boundary.
# ---------------------------------------------------------------------------

def test_the_frame_passes_a_buffer_offset_never_a_client_pointer(
        main_window, monkeypatch):
    """The mesh pass's glDrawElements carries a byte OFFSET.

    With an ELEMENT_ARRAY_BUFFER bound, GL reads the `indices`
    argument as an offset into that buffer. The offset forms GL and
    PyOpenGL agree on are None (offset 0) and a ctypes.c_void_p
    holding the byte offset — PyOpenGL's array converter passes both
    through as the pointer VALUE. A bare Python int is NOT one of
    them: AsArrayOfType turns it into a c_uint scalar and hands GL
    the scalar's heap ADDRESS (a client pointer wearing an offset's
    clothes — the correttiva's own broken link, red on the defective
    tip). The recorder below captures the argument exactly as the
    module passes it; no GL context is needed to judge the form.
    """
    import numpy as np  # noqa: F401  (the viewport module imports it)
    import CAD_0.ui.viewport_widget as vw

    _create_box(main_window)
    entries = list(main_window._viewport._draw_nodes)
    assert entries, "the gesture must populate the mirror first"

    # The recorder: the mesh pass's whole GL surface stubbed (the
    # perf1 cache suite's own _GlRecorder pattern — without a context
    # every real call would die), with glDrawElements' arguments
    # captured verbatim. The argument's FORM is the verdict; no GL
    # context is needed to judge it.
    recorded = []
    stub_names = ("glGenBuffers", "glDeleteBuffers", "glBindBuffer",
                  "glBufferData", "glEnableClientState",
                  "glDisableClientState", "glVertexPointer",
                  "glNormalPointer", "glColor3f")
    saved = {n: getattr(vw, n) for n in stub_names}
    saved["glDrawElements"] = vw.glDrawElements
    next_name = [1000]

    def stub_gen(n):
        ids = list(range(next_name[0], next_name[0] + n))
        next_name[0] += n
        return ids

    def recording_draw(mode, count, type_, indices):
        recorded.append((mode, count, type_, indices))
        # Never executed: no context exists here — the recorded form
        # is the whole verdict.

    def _noop(*_a, **_k):
        return None

    for n in stub_names:
        setattr(vw, n, _noop)
    vw.glGenBuffers = stub_gen
    vw.glDrawElements = recording_draw
    try:
        # A light parent carrying the REAL gesture's mirror entries
        # (the perf1 cache suite's own pattern), and the mesh pass.
        class _Parent:
            _draw_nodes = entries
            _selected_node_ids = set()

        window = vw._ViewportWindow(_Parent())
        window._node_buffers = {}
        window._draw_meshes()
    finally:
        for n, fn in saved.items():
            setattr(vw, n, fn)

    draws = [r for r in recorded if r[0] == vw.GL_TRIANGLES]
    assert draws, "the mesh pass must issue its triangle draws"
    for mode, count, type_, indices in draws:
        assert isinstance(indices, (type(None), ctypes.c_void_p)), (
            "glDrawElements' indices argument must be the byte-offset "
            "form (None or ctypes.c_void_p) — a bare Python int is "
            "converted by PyOpenGL to a CLIENT heap pointer, and with "
            "the IBO bound that address is read as a wild offset: the "
            f"draw fetches nothing (got {type(indices).__name__}: "
            f"{indices!r})")


# ---------------------------------------------------------------------------
# THE CLICK-TEST'S OWN VERDICT — the pixels, under a real GL context.
# ---------------------------------------------------------------------------

def test_the_inserted_solid_renders_in_the_viewport(main_window):
    """Create -> Box, one real frame: the solid's pixels are there.

    The click-test's assertion, reproduced headless (offscreen QPA
    with EGL under Xvfb — the CI's own GL posture, see
    ci-linux.yml). The framebuffer is read back and censused: the
    solid's shaded body is chromatic with every channel lifted (the
    lit base color), unlike the background (dark, near-gray), the
    grid (mid gray) and the axes (pure channels with zeros). When no
    GL context exists the test skips with the reason named — the
    Xvfb step is the column that judges it, exactly like the
    navigation GL tests.
    """
    pytest.importorskip("numpy")
    window = main_window
    vp = window._viewport
    if vp._gl_window is None:
        pytest.skip(
            "GL context unavailable in this environment — the "
            "insertion display test needs software GL (offscreen QPA "
            "+ EGL under a display; the CI's Xvfb step is its judging "
            "column)")

    import time

    _create_box(window)
    # Frame the new body so the verdict does not depend on the
    # default camera (the user's own first gesture after an insert),
    # and SHOW the window: an unexposed QOpenGLWindow has no frame to
    # grab (the grab comes back null — measured in this suite's own
    # bring-up).
    window.resize(800, 600)
    window.show()
    vp.frame_all()

    # THE FRAME'S OWN DELIVERY: requestUpdate's UpdateRequest arrives
    # on the running loop's cadence — a bare processEvents() flushes
    # it only once time passes (measured in the correttiva's own
    # probes). Spin the loop: events processed, wall time advancing,
    # the post-gesture frame delivered and painted. (An exec()-driven
    # grab inside a timer callback was tried first and DEADLOCKS
    # grabFramebuffer under the pytest host — the spin is the honest
    # replacement, same delivery, no nested-loop reentrancy.)
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance()
    for _ in range(120):
        app.processEvents()
        time.sleep(0.02)

    img = vp._gl_window.grabFramebuffer()
    assert not img.isNull(), "the framebuffer grab must come back alive"

    import numpy as np
    w, h, bpl = img.width(), img.height(), img.bytesPerLine()
    raw = np.frombuffer(img.bits(), dtype=np.uint8)[: h * bpl]
    arr = raw.reshape(h, bpl)[:, : w * 4].reshape(-1, 4)[:, :3].astype(int)

    # The solid's family: chromatic (the base color's channels differ)
    # with every channel lifted by the lighting — the bg (dark,
    # near-gray), the grid (flat gray) and the axes (two channels at
    # zero) all fail this mask regardless of byte-order conventions.
    spread = arr.max(axis=1) - arr.min(axis=1)
    lifted = arr.min(axis=1)
    solid = (spread > 12) & (lifted > 60)
    count = int(solid.sum())
    assert count > 1000, (
        f"the inserted solid must render — {count} qualifying pixels "
        "in the grabbed frame; the background, the grid and the axes "
        "alone mean the mesh pass drew nothing visible (the "
        "click-test's defect)")


# ---------------------------------------------------------------------------
# THE DECLARED DIAGNOSTIC — never the silent skip again.
# ---------------------------------------------------------------------------

def test_an_underivable_visible_node_is_diagnosed_never_silent(
        main_window, capsys):
    """A visible node with no derivable mesh is NAMED, not dropped.

    The old `_refresh_meshes` skipped such nodes with a bare
    `continue` — a visible body vanishing without a word. The C++
    discipline makes the case unreachable for valid nodes (every
    insertion path hands the node over dirty), so the diagnostic can
    only fire on a defect: this test manufactures the defective state
    exactly as the discipline's own failure would leave it — a new
    node whose dirty flag was cleared before its mesh was ever
    derived (mesh_dirty is the bindings' own writable surface) — and
    asserts the refresh names it on the status log instead of
    dropping it in silence.
    """
    window = main_window
    vp = window._viewport

    # A healthy insertion first: the gesture, the mesh, the mirror.
    _create_box(window)
    assert vp._draw_nodes, "the healthy insertion must be mirrored"

    # The defective state: a second body arrives, and whatever was
    # supposed to mark it for tessellation did not (the hypothesized
    # discipline failure, manufactured through the exposed flag).
    import CAD_0
    body = CAD_0.scene.make_body(
        "sphere", dict(CAD_0.scene.default_params("sphere")))
    nid = window._scene.add_node("Sphere", body)
    node = window._scene.get_node(nid)
    assert node is not None
    node.mesh_dirty = False  # the defect: never marked, never derived

    capsys.readouterr()  # isolate the refresh's own output
    vp._refresh_meshes()

    out = capsys.readouterr().out
    assert f"[viewport]" in out, (
        "a visible node whose display mesh is empty after the refresh "
        "must produce the declared diagnostic (the [viewport] status "
        "log), never a silent disappearance")
    assert str(nid) in out, (
        "the diagnostic must name the underivered node's id so the "
        "defect is actionable")
    # The rest of the mirror is untouched: the healthy node stays.
    assert any(e.node_id != nid for e in vp._draw_nodes), (
        "the diagnostic is per-node — the healthy features keep "
        "drawing")
