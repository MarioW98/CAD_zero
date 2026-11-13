# bindings/python/tests/test_perf1_vbo_cache.py
#
# PERF 1 — IL CACHE VBO/IBO, LA SUA CORRETTEZZA (the register's G row,
# Phase C.6's registered seat): the CI member that verifies the cache
# where the timing cannot run. The frame-time itself is measured by
# the rite on the maintainer's machine (tests/benchmarks/
# perf1_frametime.py, the m8 methodology — the benchmark suite's own
# posture, never a ctest member); the CI runner has no GPU, and a
# timing gate on a machine whose noise nobody pinned is a decorative
# number. What CI CAN verify — and this suite does, with GL recorded
# (the ninth workbench test's own model: the functions monkeypatched,
# no context needed) — is the cache's CONTRACT:
#
#   1. THE DIGEST IS THE IDENTITY — blake2b-64 over the display
#      payload's declared-LE bytes: same mesh -> same digest (every
#      call, every machine), one float32 or one index moved -> the
#      digest moves, a big-endian-declared copy of the same VALUES
#      digests identically (the bytes are declared, never assumed),
#      and a fixed payload answers a PINNED hex (the known-answer
#      pin: the digest is an instrument, instruments get calibrated);
#   2. THE FIRST SIGHT uploads every visible feature ONCE — three
#      buffers per feature, the labels set at the upload, the draw
#      one glDrawElements per range (the m6-correttiva grammar);
#   3. THE STABLE FRAME uploads NOTHING — the same mirror drawn
#      again: zero glBufferData, the buffers served from the cache,
#      the served bytes still the mirror's own (the contract below);
#   4. THE EDIT re-uploads ONE feature — the touched feature's three
#      buffers alone; the untouched features' buffers are not
#      re-specified, and there is NO whole-scene flush (the count is
#      the proof: a courtesy flush is what this cache exists to
#      refuse);
#   5. THE REMOVAL RULE — a feature that left the mirror (hidden,
#      deleted, the scene replaced) hands its buffers back, and ONLY
#      its (glDeleteBuffers), the survivors untouched;
#   6. THE CONTEXT-DEATH RULE — a fresh context (initializeGL) empties
#      the cache: GL ids do not survive their context, the labels die
#      with the ids, and the next mesh pass re-uploads everything
#      (the first-sight act again);
#   7. THE RANCID COUNTERPROOF — a cache that serves a mesh VECCHIA
#      after an edit is a decorative buffer: the served-contract
#      oracle (what the buffers HOLD, hashed independently, vs what
#      the mirror DECLARES) must catch the stale serving and NAME the
#      stale mesh with its digest. The injection forges the cache
#      label — exactly "the invalidation skipped", the age-based
#      serving — and the assertion is the catch: the test is red iff
#      the cache would serve the stale bytes silently;
#   8. THE FRAME-TIME ROW'S CI POSTURE — the declared skip: the
#      timing is the maintainer's measure (the Windows commands in
#      the register's PERF 1 row), the bench's methodology contract
#      verified at the source level here, the timing itself skipped
#      WITH the declaration, never in silence (the GL/PySide6 skip
#      model already in the house).
#
# The scenes here are HAND-BUILT mirror entries (numpy payloads, one
# per feature): the cache's contract is entries-in/buffers-out, and
# the mirror's own construction from real scenes is the workbench
# suite's pinned territory (test_the_fillet_is_visible_after_apply,
# test_viewport_uploads_field_sampled_buffer) — the same door, never
# a second opinion.
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


_ROOT = Path(__file__).resolve().parents[3]
_BENCH = _ROOT / "tests" / "benchmarks" / "perf1_frametime.py"


# ---------------------------------------------------------------------------
# THE RECORDER — GL recorded, never run (the ninth workbench test's own
# model): every buffer name, bind, upload and draw captured, the
# payloads COPIED so the oracle can hash what the buffers hold.
# ---------------------------------------------------------------------------

class _GlRecorder:
    """A recording stand-in for the module's GL surface.

    The cache path touches: glGenBuffers/glDeleteBuffers (the names),
    glBindBuffer (the target's current name), glBufferData (the
    upload — payload copied, attributed to the bound name), the
    client-state/pointer/draw calls (counted, the grammar's pin).
    """

    def __init__(self, module):
        self.module = module
        self.next_id = 1
        self.allocated: list[int] = []
        self.deleted: list[int] = []
        self.binds: dict[int, int] = {}          # target -> current name
        self.payloads: dict[int, object] = {}    # buffer name -> bytes held
        self.upload_counts: dict[int, int] = {}  # buffer name -> glBufferData
        self.calls: dict[str, int] = {}
        self._saved = {}
        for name in ("glGenBuffers", "glDeleteBuffers", "glBindBuffer",
                     "glBufferData", "glEnableClientState",
                     "glDisableClientState", "glVertexPointer",
                     "glNormalPointer", "glDrawElements", "glEnable",
                     "glDepthFunc", "glLineWidth"):
            self._saved[name] = getattr(module, name, None)

    # -- lifecycle ------------------------------------------------------
    def install(self):
        def gen(n):
            ids = list(range(self.next_id, self.next_id + n))
            self.next_id += n
            self.allocated.extend(ids)
            return ids

        def delete(n, ids):
            for i in ids:
                i = int(i)
                self.deleted.append(i)
                self.payloads.pop(i, None)

        def bind(target, buf):
            self.binds[int(target)] = int(buf)
            self.calls["glBindBuffer"] = self.calls.get("glBindBuffer",
                                                        0) + 1

        def bufdata(target, data, usage):
            name = self.binds.get(int(target))
            assert name is not None, "glBufferData with no bound name"
            import numpy as np
            self.payloads[name] = np.array(data, copy=True)
            self.upload_counts[name] = self.upload_counts.get(name, 0) + 1
            self.calls["glBufferData"] = self.calls.get("glBufferData",
                                                        0) + 1

        def counting(name):
            def f(*a, **k):
                self.calls[name] = self.calls.get(name, 0) + 1
            return f

        setattr(self.module, "glGenBuffers", gen)
        setattr(self.module, "glDeleteBuffers", delete)
        setattr(self.module, "glBindBuffer", bind)
        setattr(self.module, "glBufferData", bufdata)
        for name in ("glEnableClientState", "glDisableClientState",
                     "glVertexPointer", "glNormalPointer",
                     "glDrawElements", "glEnable", "glDepthFunc",
                     "glLineWidth"):
            setattr(self.module, name, counting(name))

    def restore(self):
        for name, fn in self._saved.items():
            if fn is not None:
                setattr(self.module, name, fn)

    # -- queries ---------------------------------------------------------
    def uploads(self, *names):
        """Total glBufferData calls on the given buffer names (all if
        none given)."""
        if not names:
            return sum(self.upload_counts.values())
        return sum(self.upload_counts.get(int(n), 0) for n in names)

    def draws(self):
        return self.calls.get("glDrawElements", 0)

    def uploads_on(self, name):
        return self.upload_counts.get(int(name), 0)


def _buf_digest(payload, dtype) -> str:
    """blake2b-64 over ONE buffer's declared-LE bytes — the oracle's
    own instrument, the same convention _display_mesh_digest holds
    for the whole payload (positions/normals '<f4', indices '<u4')."""
    import numpy as np
    flat = np.ascontiguousarray(payload, dtype=dtype).ravel()
    h = hashlib.blake2b(digest_size=8)
    h.update(flat.view(np.uint8))
    return h.hexdigest()


# ---------------------------------------------------------------------------
# THE FIXTURES — a window with a light parent (the cache's own door),
# GL recorded, the mirror entries hand-built.
# ---------------------------------------------------------------------------

@pytest.fixture()
def gl_window(monkeypatch):
    if not _pyside6_available():
        pytest.skip("PySide6 not installed")
    pytest.importorskip("numpy")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    import CAD_0.ui.viewport_widget as vw

    class _Parent:
        _draw_nodes = []
        _selected_node_ids = set()

    window = vw._ViewportWindow(_Parent())
    recorder = _GlRecorder(vw)
    recorder.install()
    try:
        yield vw, window, recorder
    finally:
        recorder.restore()


def _entry(vw, node_id, seed, n_tris=8):
    """A hand-built mirror entry: a distinct mesh per (node_id, seed).

    The payload is deterministic in the seed — the same seed answers
    the same bytes, a different seed a different mesh on ALL THREE
    payloads (a real re-derivation moves positions, normals AND
    indices: a re-tessellated feature is a new mesh everywhere, and
    the rancid counterproof needs a mesh whose every buffer went
    stale)."""
    import numpy as np
    n_verts = n_tris + 2
    pos = np.zeros((n_verts, 3), dtype="<f4")
    pos[:, 0] = np.linspace(0.0, 1.0, n_verts) + seed
    pos[:, 1] = seed * 0.5
    nor = np.zeros_like(pos)
    nor[:, 2] = 1.0
    nor[:, 0] = 0.25 * seed
    idx = np.zeros(n_tris * 3, dtype="<u4")
    shift = int(seed * 3) % n_verts
    idx[0::3] = (np.arange(n_tris) + shift) % n_verts
    idx[1::3] = (np.arange(1, n_tris + 1) + shift) % n_verts
    idx[2::3] = (np.arange(2, n_tris + 2) + shift) % n_verts
    digest = vw._display_mesh_digest(pos, nor, idx)
    return vw._NodeDrawEntry(node_id, pos, nor, idx, digest)


def _served_contract(vw, window, recorder, entries):
    """THE SERVED-CONTRACT ORACLE — an independent road (the m7
    spirit): what the buffers HOLD, hashed by THIS function's own
    blake2b, against what the mirror DECLARES. The cache's own labels
    are not consulted: a forged label is precisely the rancid state
    this oracle exists to catch. A node whose held bytes differ from
    its mirror payload is reported ONCE, as a MESH — the stale mesh
    named with ITS OWN digest (the whole-payload digest of the held
    bytes, the same instrument the mirror's identity uses), the
    mirror's current digest beside it, and how many of the three
    buffers are stale."""
    violations = []
    for e in entries:
        bufs = window._node_buffers.get(e.node_id)
        if bufs is None:
            violations.append((e.node_id, "no buffers"))
            continue
        held = {}
        stale_buffers = 0
        for buf_id, payload, dtype in (
                (bufs.pos_buf, e.positions, "<f4"),
                (bufs.nor_buf, e.normals, "<f4"),
                (bufs.idx_buf, e.indices, "<u4")):
            bytes_held = recorder.payloads.get(int(buf_id))
            if bytes_held is None:
                held = None
                break
            held[buf_id] = (bytes_held, payload, dtype)
            if _buf_digest(bytes_held, dtype) != _buf_digest(payload,
                                                             dtype):
                stale_buffers += 1
        if held is None:
            violations.append((e.node_id, "buffer never uploaded"))
            continue
        if stale_buffers == 0:
            continue   # the served bytes ARE the mirror's
        # The stale MESH's own digest: the whole payload of the held
        # bytes, hashed with the mirror's own instrument — the name
        # the rancid mesh answers to.
        held_pos = held[bufs.pos_buf][0]
        held_nor = held[bufs.nor_buf][0]
        held_idx = held[bufs.idx_buf][0]
        stale_mesh_digest = vw._display_mesh_digest(held_pos, held_nor,
                                                    held_idx)
        violations.append(
            (e.node_id, f"stale mesh (digest {stale_mesh_digest}, the "
                        f"mirror declares {e.digest}; {stale_buffers} "
                        f"of 3 buffers stale)"))
    return violations


# ---------------------------------------------------------------------------
# 1. THE DIGEST IS THE IDENTITY
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_digest_is_the_identity():
    pytest.importorskip("numpy")
    import numpy as np
    monkeypatch_env = None  # no GL needed: pure function
    import CAD_0.ui.viewport_widget as vw

    pos = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]], dtype=np.float32)
    nor = np.array([[0.0, 0.0, 1.0]] * 4, dtype=np.float32)
    idx = np.array([0, 1, 2, 1, 3, 2], dtype=np.uint32)

    # THE KNOWN-ANSWER PIN: the instrument calibrated on a fixed
    # payload — any drift in the digest's construction (the order, the
    # dtypes, the size) moves this hex and the suite says so.
    assert vw._display_mesh_digest(pos, nor, idx) == \
        "853e02f9b5c993ec"

    # Same mesh, every call: the same digest.
    d1 = vw._display_mesh_digest(pos, nor, idx)
    d2 = vw._display_mesh_digest(pos, nor, idx)
    assert d1 == d2

    # One float32 in ONE vertex moved -> the digest moves.
    pos_moved = pos.copy()
    pos_moved[2, 1] = 0.5
    assert vw._display_mesh_digest(pos_moved, nor, idx) != d1

    # One index moved -> the digest moves (the indices are payload
    # too: a cache that ignored them would serve the old topology).
    idx_moved = idx.copy()
    idx_moved[0] = 1
    assert vw._display_mesh_digest(pos, nor, idx_moved) != d1

    # The LE declaration: a big-endian-declared copy of the SAME
    # VALUES digests identically — the bytes are declared, never
    # assumed (the digest is well-defined on every machine).
    assert vw._display_mesh_digest(
        pos.astype(">f4"), nor.astype(">f4"), idx.astype(">u4")) == d1


# ---------------------------------------------------------------------------
# 2. THE FIRST SIGHT — every visible feature uploaded ONCE
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_first_sight_uploads_every_feature_once(gl_window):
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0), _entry(vw, 3, 2.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()

    # Three buffers per feature, uploaded ONCE: the ids allocated, the
    # labels set at the upload.
    assert rec.uploads() == 9
    assert len(rec.allocated) == 9
    for e in entries:
        bufs = window._node_buffers[e.node_id]
        assert bufs.digest == e.digest, (
            "the label must be the mesh's own digest, set at the upload")
        assert rec.uploads_on(bufs.pos_buf) == 1
        assert rec.uploads_on(bufs.nor_buf) == 1
        assert rec.uploads_on(bufs.idx_buf) == 1

    # The grammar (m6-correttiva): one draw call per range.
    assert rec.draws() == 3

    # And the served contract holds from the first sight.
    assert _served_contract(vw, window, rec, entries) == []


# ---------------------------------------------------------------------------
# 3. THE STABLE FRAME — zero uploads, the cache serves
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_stable_frame_uploads_nothing(gl_window):
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0), _entry(vw, 3, 2.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()
    uploads_first = rec.uploads()
    assert uploads_first == 9

    # THE STABLE PATH (an orbit, a selection, any frame that changes
    # no mesh bytes): the same mirror drawn again — NOTHING uploaded.
    window._draw_meshes()
    assert rec.uploads() == 9, (
        "the stable frame must upload ZERO buffers: the cache serves "
        "the digest-certified bytes, the upload is the exception")
    assert rec.draws() == 6          # one draw call per range, per frame
    assert len(rec.allocated) == 9   # no new names either

    # The served contract still holds: what the buffers hold is what
    # the mirror declares.
    assert _served_contract(vw, window, rec, entries) == []


# ---------------------------------------------------------------------------
# 4. THE EDIT — ONE feature re-uploads, never the scene
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_edit_reuploads_one_feature_only(gl_window):
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0), _entry(vw, 3, 2.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()

    # THE EDIT: feature 2's mesh re-derived (a new radius, a new
    # resolution — here a new seed), its digest moved.
    edited = _entry(vw, 2, 1.5)
    assert edited.digest != entries[1].digest, (
        "the edit's mesh must MOVE the digest — else this test "
        "measures nothing")
    entries[1] = edited
    window._parent._draw_nodes = entries
    window._draw_meshes()

    # ONE feature re-uploaded: its three buffers, and ONLY theirs.
    assert rec.uploads() == 12, (
        "the edit must re-upload exactly the touched feature's three "
        "buffers (9 + 3) — a whole-scene flush is the courtesy this "
        "cache exists to refuse")
    b1 = window._node_buffers[1]
    b2 = window._node_buffers[2]
    b3 = window._node_buffers[3]
    assert rec.uploads_on(b1.pos_buf) == 1
    assert rec.uploads_on(b1.nor_buf) == 1
    assert rec.uploads_on(b1.idx_buf) == 1
    assert rec.uploads_on(b2.pos_buf) == 2   # first sight + the edit
    assert rec.uploads_on(b2.nor_buf) == 2
    assert rec.uploads_on(b2.idx_buf) == 2
    assert rec.uploads_on(b3.pos_buf) == 1
    assert rec.uploads_on(b3.nor_buf) == 1
    assert rec.uploads_on(b3.idx_buf) == 1

    # The label followed the edit; the served contract holds.
    assert window._node_buffers[2].digest == edited.digest
    assert _served_contract(vw, window, rec, entries) == []

    # The stable frame after the edit: zero uploads again.
    window._draw_meshes()
    assert rec.uploads() == 12


# ---------------------------------------------------------------------------
# 5. THE REMOVAL RULE — absent features hand their buffers back
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_removal_rule_hands_absent_buffers_back(gl_window):
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0), _entry(vw, 3, 2.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()

    gone = window._node_buffers[3]
    survivors = (window._node_buffers[1], window._node_buffers[2])

    # Feature 3 hidden (it left the mirror): its buffers — and ONLY
    # its — are handed back.
    entries = [entries[0], entries[1]]
    window._parent._draw_nodes = entries
    window._draw_meshes()
    assert sorted(rec.deleted) == sorted(
        [gone.pos_buf, gone.nor_buf, gone.idx_buf])
    assert 1 in window._node_buffers and 2 in window._node_buffers
    assert 3 not in window._node_buffers

    # The survivors served with zero uploads, still one draw per range.
    assert rec.uploads() == 9
    assert rec.draws() == 5   # 3 (first frame) + 2 (this frame)
    for bufs in survivors:
        assert rec.uploads_on(bufs.pos_buf) == 1


# ---------------------------------------------------------------------------
# 6. THE CONTEXT-DEATH RULE — a fresh context empties the cache
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_context_death_empties_the_cache(gl_window, monkeypatch):
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()
    assert rec.uploads() == 6

    # A FRESH CONTEXT: initializeGL's door, the context check answered
    # (the recorded GL stands in for the real one — the rule under
    # test is the cache's, not the context's).
    from PySide6.QtGui import QOpenGLContext
    monkeypatch.setattr(
        QOpenGLContext, "currentContext",
        staticmethod(lambda: object()), raising=True)
    window.initializeGL()

    # The cache is EMPTY: the ids died with their context, the labels
    # with the ids (nothing to glDeleteBuffers on the new context —
    # the dict is simply reborn).
    assert window._node_buffers == {}
    # The overlays were re-initialized on the fresh context (their
    # five names, uploaded once here — initialization, not frame work).
    assert len(rec.allocated) == 6 + 5
    assert window._overlay_buffers is not None

    # The next mesh pass is the first-sight act again: every feature
    # re-uploaded, on FRESH names.
    before = rec.uploads()
    window._draw_meshes()
    assert rec.uploads() == before + 6
    assert _served_contract(vw, window, rec, entries) == []


# ---------------------------------------------------------------------------
# 7. THE RANCID COUNTERPROOF — the stale serving caught, named
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_rancid_cache_is_caught_red(gl_window, monkeypatch):
    """A cache that serves a mesh VECCHIA after an edit is a
    decorative buffer. The injection forges the cache's label —
    exactly "the invalidation skipped" (the age-based serving: the
    flag said fresh, the bytes are stale) — and the served-contract
    oracle must catch it, naming the stale mesh with its digest. The
    failure IS the assertion: without the catch, this test is red."""
    vw, window, rec = gl_window
    entries = [_entry(vw, 1, 0.0), _entry(vw, 2, 1.0)]
    window._parent._draw_nodes = entries
    window._draw_meshes()
    old_digest = entries[1].digest

    # THE EDIT: feature 2 re-derived, its digest moved.
    edited = _entry(vw, 2, 1.5)
    assert edited.digest != old_digest

    # THE RANCID TWIN: the label forged to the new digest BEFORE the
    # honest comparison — the comparison then passes, the stale bytes
    # served with ZERO upload. This is the age-based cache's own
    # serving, injected through the real code path.
    honest_ensure = vw._ViewportWindow._ensure_node_buffers

    def rancid_ensure(self, entry):
        cached = self._node_buffers.get(entry.node_id)
        if cached is not None and cached.digest != entry.digest:
            cached.digest = entry.digest  # the LIE
        return honest_ensure(self, entry)

    monkeypatch.setattr(vw._ViewportWindow, "_ensure_node_buffers",
                        rancid_ensure)
    entries[1] = edited
    window._parent._draw_nodes = entries
    window._draw_meshes()

    # The twin really served stale: NO upload happened for feature 2.
    b2 = window._node_buffers[2]
    assert rec.uploads_on(b2.pos_buf) == 1, (
        "the rancid twin must have served the stale bytes (zero "
        "re-upload) — else this counterproof measured nothing")

    # THE CATCH: the served-contract oracle goes RED, naming feature 2's
    # STALE MESH with its own digest (the held bytes' whole-payload
    # identity — the same instrument the mirror uses). A cache without
    # this catch would serve the stale mesh silently — and this
    # assertion would fail, which is the point.
    violations = _served_contract(vw, window, rec, entries)
    assert violations, (
        "the rancid serving was NOT caught — the cache is a "
        "decorative buffer")
    assert len(violations) == 1, (
        "exactly feature 2 is stale, nothing else")
    node_id, what = violations[0]
    assert node_id == 2
    assert old_digest in what, (
        "the stale mesh must be NAMED with its digest")
    assert edited.digest in what, (
        "and the mirror's current digest beside it")
    assert "3 of 3 buffers stale" in what, (
        "the whole mesh went stale — a re-derived feature is a new "
        "mesh everywhere")

    monkeypatch.undo()
    # THE HONEST TWIN beside it (the control): the same edit through
    # the honest comparison re-uploads feature 2 and the contract is
    # green again.
    entries[1] = _entry(vw, 2, 2.5)
    window._parent._draw_nodes = entries
    window._draw_meshes()
    assert rec.uploads_on(window._node_buffers[2].pos_buf) == 2
    assert _served_contract(vw, window, rec, entries) == []


# ---------------------------------------------------------------------------
# 8. THE FRAME-TIME ROW'S CI POSTURE — the declared skip
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _pyside6_available(), reason="PySide6 not installed")
def test_the_frame_time_row_measured_by_the_rite():
    """The benchmark suite's frame-time row (PERF 1, beside the m8
    row zero — never in its place): the methodology contract verified
    at the source level, the timing itself measured by the RITE on
    the maintainer's machine. The CI runner has no GPU and is not the
    measuring machine — the skip is DECLARED here, never silent (the
    GL/PySide6 skip model already in the house)."""
    src = _BENCH.read_text()

    # The bench exists and declares the m8 methodology, pinned where
    # it runs: the warmup, the iterations, the median (never the min,
    # never the raw mean), the within-run A/B gate with its declared
    # ratio and floor, the digest-first order, the machine on the
    # first printed line — a bench that silently changed its
    # methodology fails here.
    assert "WARMUP = 3" in src
    assert "ITERATIONS = 21" in src
    assert "GATE_RATIO" in src
    assert "GATE_FLOOR_S" in src
    assert "median" in src
    for token in ("the digests verified", "digest"):
        assert token in src
    assert "the machine" in src and "declared" in src

    # THE TIMING ITSELF: skipped WITH the declaration — the CI runner
    # is not the measuring machine; the true frame-times are the
    # maintainer's Windows measure (the verbatim commands in the
    # register's PERF 1 row), never a pinned second.
    pytest.skip(
        "the frame-time row is measured by the rite on the "
        "maintainer's machine (tests/benchmarks/perf1_frametime.py, "
        "the m8 methodology: warmup 3, iterations 21, medians, "
        "digests before times, within-run A/B ratios) — the CI runner "
        "has no GPU and is not the measuring machine (the register's "
        "m8 row's own law: the absolute numbers are indicative, the "
        "ratios are the substance)")
