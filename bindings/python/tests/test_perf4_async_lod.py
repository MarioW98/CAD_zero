# bindings/python/tests/test_perf4_async_lod.py
#
# PERF 4 — VALUTAZIONE ASINCRONA E LOD A TRE LIVELLI, LA SUA
# CORRETTEZZA (the register's G row, the PERF sequence's fourth and
# closing morso): the CI member that verifies the async display road
# and its three LOD levels. The perf1 suite's own model (the VBO
# cache's correctness member): the TIMING is the rite's measure on the
# measuring machine (tests/benchmarks/perf4_async_lod.py, the m8
# methodology); what CI verifies — here, with the worker driven
# through EXPLICIT SYNC POINTS (never a time sleep, the brief's own
# law) — is the CONTRACT:
#
#   1. THE LADDER, the three commutation rules each named and each
#      testable alone: BOZZA during the gesture (draft, resolution 12),
#      MEDIO at the gesture's release (medium, 24), FINE at the
#      declared idle (the display resolution, 48 today — the static
#      path's own, the contractual mesh); the resolutions FIXED NAMED
#      CONSTANTS, never runtime-measured;
#   2. THE BIT-IDENTITY LAW (ADR-0014 extended to the display road):
#      the async FINE landing is byte-identical to the sync road's
#      own answer — the same derive the static path rides, the same
#      digest (PERF 1's instrument, unmoved);
#   3. THE LEVEL IS IN THE CACHE KEY: the VBO/IBO cache serves on the
#      (level, digest) PAIR — a bozza mesh is never served where the
#      fine one is declared, NOT EVEN when the bytes coincide (the
#      pathological empty/degenerate mesh): the pair refuses it by
#      construction — CONTROPROVA (1), red on the digest-only naive
#      cache;
#   4. CANCELLATION IS THE GENERATION GATE: a result that completes
#      after its generation was superseded never lands — not in the
#      cache, not in the viewport, not in the scene — CONTROPROVA (2),
#      red on the no-gate naive pipeline;
#   5. THE WORKER NEVER MUTATES THE SCENE: the derivation runs on the
#      worker thread over its OWN snapshot; the scene's cached_mesh is
#      written ONLY by the fine landing, on the owner (main) thread —
#      CONTROPROVA (3), red on the update_meshes-from-the-worker
#      naive road;
#   6. THE SYNC BARRIER: flush_fine settles the dirty visible set at
#      the CURRENT recipe's fine mesh on the caller's thread — the
#      exports' and the picks' door, the commutation never degrading
#      the data;
#   7. NO MIXED STATE: a job's nodes land as ONE delivery — the
#      previous mirror or the new one, never a mix.
#
# The scenes: hand-driven pipelines on real kernel scenes (a sphere
# for the protocol tests, the chain-5 — the perf1 bench's own
# construction, quoted — for the identity and the barrier). The GL
# half (the cache-key controprova) runs with GL RECORDED (the perf1
# suite's own model: the functions monkeypatched, no context needed).
from __future__ import annotations

import threading
from pathlib import Path

import pytest


def _pyside6_available() -> bool:
    try:
        import importlib.util
        return importlib.util.find_spec("PySide6") is not None
    except ImportError:
        return False


def _kernel_available() -> bool:
    try:
        import CAD_0  # noqa: F401
        return True
    except ImportError:
        return False


# ---------------------------------------------------------------------------
# The corpus — the perf1 bench's own construction, quoted (the m8
# LoadedChain: box#1 -> sk#2 -> ext#3 -> sub#4 -> fil#5, the BLIND
# hole, the rim fillet; the operands step aside, the fillet is the
# frame's one visible body).
# ---------------------------------------------------------------------------

RADIUS_A = 0.25
RADIUS_B = 0.125


def _build_chain5(scene):
    from CAD_0 import sdf as sdf_mod
    from CAD_0 import scene as scene_mod
    from CAD_0 import brep as brep_mod
    from CAD_0 import math as math_mod

    bx = scene.add_node("Box", sdf_mod.box(math_mod.Vec3f(1.0, 1.0, 1.0)))
    node = scene.get_node(bx)
    node.builder = "box"
    node.set_params({"extent_x": 1.0, "extent_y": 1.0, "extent_z": 1.0})
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    sk = scene.add_sketch_node("Sketch", chains, [])
    ex = scene.add_extrude_node("Extrude", sketch=sk, distance=2.0,
                                direction=1)
    sub = scene.add_operation_node("Subtract", target=bx, tool=ex,
                                   offset_z=-0.5)
    rim = None
    for i in range(brep_mod.edge_count(scene.get_node(sub).exact_brep)):
        e = brep_mod.edge_query(scene.get_node(sub).exact_brep, i)
        if "@hi" in e["name"] and "isect" in e["name"]:
            rim = e["name"]
            break
    assert rim is not None, "the chain-5's rim edge was not found"
    fid = scene.add_fillet_node("Fillet", rim, RADIUS_A)
    for nid in (bx, sk, ex, sub):
        scene.set_visible(nid, False)
    return fid


# ---------------------------------------------------------------------------
# The pipeline fixtures
# ---------------------------------------------------------------------------

class _Lands:
    """The delivery witness: every landing recorded with the thread it
    landed on (the ownership rule's own instrument — the fine landing
    must be the MAIN thread's act, never the worker's). The fine level
    INSTALLS into the scene exactly the widget's handler does
    (land_fine_mesh): the tests read the node's contractual mesh, and
    the pipeline's own contract is that the land_fn owns the install."""

    def __init__(self, scene):
        self.scene = scene
        self.entries = []      # (generation, level, node_ids, thread)

    def __call__(self, generation, level, results):
        from CAD_0.ui.display_pipeline import LOD_FINE
        self.entries.append((generation, level,
                             [nid for (nid, _) in results],
                             threading.current_thread()))
        if level == LOD_FINE:
            for (nid, derivation) in results:
                self.scene.land_fine_mesh(nid, derivation.mesh)

    def levels(self):
        return [level for (_, level, _, _) in self.entries]

    def threads(self):
        return [t for (_, _, _, t) in self.entries]


@pytest.fixture()
def pipeline_scene():
    """A one-sphere scene + a REAL pipeline with a recording land_fn."""
    if not _kernel_available():
        pytest.skip("the native kernel is not importable")
    from CAD_0 import sdf as sdf_mod
    from CAD_0 import scene as scene_mod
    from CAD_0.ui.display_pipeline import AsyncDisplayPipeline

    scene = scene_mod.Scene()
    nid = scene.add_node("Sphere", sdf_mod.sphere(1.0))
    lands = _Lands(scene)
    pipeline = AsyncDisplayPipeline(
        scene=scene,
        fine_resolution_fn=lambda: 48,
        land_fn=lands,
    )
    try:
        yield scene, nid, pipeline, lands
    finally:
        pipeline.stop()


# ---------------------------------------------------------------------------
# 1 — THE LADDER: the three commutation rules, each named
# ---------------------------------------------------------------------------

def test_the_ladder_three_rules_three_resolutions(pipeline_scene):
    """BOZZA at the edit, MEDIO at the release, FINE at the declared
    idle — each switch schedules its own level, the resolutions the
    FIXED NAMED CONSTANTS (12/24/48), and each level lands in its own
    time (the draft lands before the medium is even scheduled)."""
    scene, nid, pipeline, lands = pipeline_scene
    from CAD_0.ui import display_pipeline as dp

    # The named constants, recalled (a constant that moved is a
    # contract change, and the register wants to hear of it).
    assert dp.DRAFT_RESOLUTION == 12
    assert dp.MEDIUM_RESOLUTION == 24
    assert dp.IDLE_FINE_DELAY_MS == 200
    assert dp.level_resolution(dp.LOD_DRAFT, 48) == 12
    assert dp.level_resolution(dp.LOD_MEDIUM, 48) == 24
    assert dp.level_resolution(dp.LOD_FINE, 48) == 48

    # THE BOZZA RULE: an edit event schedules the draft alone.
    pipeline.on_edit()
    assert pipeline.wait_until_settled()
    assert lands.levels() == [dp.LOD_DRAFT]

    # THE MEDIO RULE: the release schedules the medium of the SAME
    # generation (no bump: the generation is the gesture's own).
    gen_after_edit = pipeline.generation
    pipeline.on_release()
    assert pipeline.generation == gen_after_edit
    assert pipeline.wait_until_settled()
    assert lands.levels() == [dp.LOD_DRAFT, dp.LOD_MEDIUM]

    # THE FINE RULE: the declared idle schedules the fine — and the
    # fine's landing installs the contractual mesh: the node's dirty
    # flag clears, cached_mesh carries the derivation.
    pipeline.on_idle()
    assert pipeline.generation == gen_after_edit
    assert pipeline.wait_until_settled()
    assert lands.levels() == [dp.LOD_DRAFT, dp.LOD_MEDIUM, dp.LOD_FINE]
    node = scene.get_node(nid)
    assert not node.mesh_dirty
    assert node.cached_mesh.triangle_count() > 0


def test_the_new_edit_supersedes_the_ladder_in_flight(pipeline_scene):
    """A second edit while the first ladder is mid-flight opens a NEW
    generation: the first's pending rungs die at the gate (never
    land), the second's ladder answers. The cancelled levels are
    ABSENT from the lands — not delivered late, not delivered at all."""
    scene, nid, pipeline, lands = pipeline_scene
    from CAD_0.ui import display_pipeline as dp

    pipeline.on_edit()          # generation 1: draft queued
    pipeline.on_edit()          # generation 2: gen 1's draft superseded
    assert pipeline.generation == 2
    assert pipeline.wait_until_settled()
    # Only generation 2's draft landed; generation 1's landed nothing.
    assert lands.levels() == [dp.LOD_DRAFT]
    assert all(g == 2 for (g, _, _, _) in lands.entries)

    # The medium of the CURRENT generation still answers its release.
    pipeline.on_release()
    assert pipeline.wait_until_settled()
    assert lands.levels() == [dp.LOD_DRAFT, dp.LOD_MEDIUM]


# ---------------------------------------------------------------------------
# 2 — THE BIT-IDENTITY LAW (the Python side; the C++ side is the
#     kernel suite's own member, tests/unit/scene/
#     test_perf4_display_derivation.cpp)
# ---------------------------------------------------------------------------

def test_the_fine_landing_is_byte_identical_to_the_sync_road():
    """The chain-5 corpus (the perf1 bench's own): the async road's
    FINE landing answers the same mirror digest the sync road answers
    for the same recipe — the ladder's top rung IS the static path,
    bit for bit (ADR-0014's own law riding the display road)."""
    if not (_kernel_available() and _pyside6_available()):
        pytest.skip("kernel + PySide6 required")
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from CAD_0 import scene as scene_mod
    from CAD_0.ui.viewport_widget import ViewportWidget

    scene = scene_mod.Scene()
    fid = _build_chain5(scene)
    vp = ViewportWidget()
    try:
        vp.set_scene(scene)
        vp._refresh_meshes()          # the sync road: the "before"
        assert len(vp._draw_nodes) == 1
        sync_digest = vp._draw_nodes[0].digest
        assert vp._draw_nodes[0].level == "fine"
        assert vp._draw_nodes[0].triangle_count == 38432, (
            "the corpus's fine mesh moved — the perf1 pins own it")

        # The gesture: the radius edit (the m8 corpus's own act).
        scene.set_fillet_radius(fid, RADIUS_B)
        vp.refresh_edit_gesture()
        assert vp._display_pipeline.wait_until_settled()
        # Mid-ladder: the mirror carries a coarser mesh than the fine.
        assert vp._draw_nodes[0].level in ("draft", "medium")
        assert vp._draw_nodes[0].triangle_count < 38432

        # The declared idle: the fine lands, the mesh settles.
        vp._display_pipeline.on_idle()
        assert vp._display_pipeline.wait_until_settled()
        assert vp._draw_nodes[0].level == "fine"
        assert not scene.get_node(fid).mesh_dirty
        async_digest = vp._draw_nodes[0].digest

        # THE IDENTITY: a sync refresh on the same recipe answers the
        # SAME digest — the two roads, one answer.
        vp._refresh_meshes()
        assert vp._draw_nodes[0].digest == async_digest
        assert vp._draw_nodes[0].digest != sync_digest  # the edit moved it
    finally:
        vp.shutdown_display_pipeline()


# ---------------------------------------------------------------------------
# 3 — CONTROPROVA (1): a bozza mesh served in place of the fine is
#     REFUSED (the level is in the cache key)
# ---------------------------------------------------------------------------

class _GlRecorder:
    """The perf1 suite's own model: the GL functions the cache calls,
    patched on the VIEWPORT MODULE'S OWN IMPORTED NAMES (the
    `from OpenGL.GL import ...` bindings, not the library's — the
    perf1 recorder's own lesson), every upload counted with the bytes
    it carried. No context needed: the cache's contract is
    entries-in/buffers-out, recorded — the CI runner has no GPU and
    still judges this door."""

    def __init__(self, module):
        self.module = module
        self.uploads = 0
        self.served_levels = []
        self._saved = {}
        self._next_id = 100
        self._bound = {}

    def install(self):
        mod = self.module

        def gen(n):
            ids = list(range(self._next_id, self._next_id + n))
            self._next_id += n
            return ids

        def bind(target, buf):
            self._bound[int(target)] = int(buf)

        def bufdata(target, data, usage):
            self.uploads += 1

        for name, fn in (("glGenBuffers", gen), ("glBindBuffer", bind),
                         ("glBufferData", bufdata)):
            self._saved[name] = getattr(mod, name, None)
            setattr(mod, name, fn)

    def restore(self):
        for name, fn in self._saved.items():
            if fn is not None:
                setattr(self.module, name, fn)


def test_controprova_1_a_draft_served_as_the_fine_is_refused(
        monkeypatch):
    """THE CACHE KEY IS THE (LEVEL, DIGEST) PAIR. The pathological
    witness: a draft payload and a fine payload whose BYTES COINCIDE
    (a body whose mesh is identical at both resolutions — constructed
    by hand, the perf1 suite's own hand-built-entry model). The naive
    digest-only cache serves the draft's buffers for the fine entry
    with ZERO re-upload — the bytes match, after all; the honest cache
    RE-UPLOADS, because the level is a DECLARATION the bytes do not
    carry. The assertion is the upload count: red on the naive, green
    on the cure. THE FAILURE IS THE ASSERTION."""
    if not (_kernel_available() and _pyside6_available()):
        pytest.skip("kernel + PySide6 required")
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import numpy as np
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from CAD_0 import scene as scene_mod
    from CAD_0.ui import viewport_widget as vw_mod
    from CAD_0.ui.viewport_widget import (
        ViewportWidget, _NodeDrawEntry, _ViewportWindow)

    scene = scene_mod.Scene()
    scene.add_node("Sphere", __import__("CAD_0").sdf.sphere(1.0))
    vp = ViewportWidget()
    try:
        # The GL road is RECORDED, not needed: the cache's contract is
        # exercised on a bare window with the module's GL names
        # patched (the perf1 recorder's own model — CI has no GPU and
        # still judges this door).
        window = _ViewportWindow(vp)
        recorder = _GlRecorder(vw_mod)
        recorder.install()
        try:
            # THE PATHOLOGICAL PAIR: same bytes, different declarations.
            pos = np.zeros((5, 3), dtype="<f4")
            pos[:, 0] = [0.0, 1.0, 0.0, 1.0, 0.5]
            nor = np.zeros_like(pos)
            nor[:, 2] = 1.0
            idx = np.array([0, 1, 2, 1, 3, 2, 0, 2, 4, 2, 3, 4],
                           dtype="<u4")
            digest = vw_mod._display_mesh_digest(pos, nor, idx)
            draft_entry = _NodeDrawEntry(7, pos, nor, idx, digest,
                                         "draft")
            fine_entry = _NodeDrawEntry(7, pos, nor, idx, digest,
                                        "fine")

            # First sight of the draft: three uploads.
            assert recorder.uploads == 0
            bufs = window._ensure_node_buffers(draft_entry)
            assert recorder.uploads == 3
            assert bufs.level == "draft" and bufs.digest == digest

            # The same bytes again, STILL declared draft: ZERO uploads
            # — the pair matches, the cache serves (PERF 1's own
            # contract, unmoved).
            bufs2 = window._ensure_node_buffers(draft_entry)
            assert recorder.uploads == 3
            assert bufs2 is bufs

            # THE REFUSAL: the same bytes declared FINE. The naive
            # digest-only cache would serve the draft's buffers here
            # (the bytes match); the honest cache re-uploads — the
            # level is in the key. RED on the naive: uploads stay 3
            # and the served entry keeps the draft's declaration.
            bufs3 = window._ensure_node_buffers(fine_entry)
            assert recorder.uploads == 6, (
                "the cache served a DRAFT's buffers for a FINE "
                "declaration — the level is not in the key")
            assert bufs3.level == "fine"

            # And the road back: the fine served again, zero uploads.
            window._ensure_node_buffers(fine_entry)
            assert recorder.uploads == 6
        finally:
            recorder.restore()
    finally:
        vp.shutdown_display_pipeline()


# ---------------------------------------------------------------------------
# 4 — CONTROPROVA (2): an obsolete result after cancellation never
#     lands
# ---------------------------------------------------------------------------

def test_controprova_2_the_obsolete_result_never_lands(pipeline_scene):
    """THE GENERATION GATE. The deterministic instrument: an injectable
    derive that BLOCKS on an Event the test holds — the worker's
    completion is OBSERVED, never timed. Edit A (generation 1) blocks
    in the derive; edit B (generation 2) supersedes it; the test
    releases A's derive; A's result completes on the worker — and dies
    at the drain gate: no landing, no cache write, no viewport state.
    The naive no-gate pipeline would deliver A's result late, over the
    fresher generation's state — the assertion (the lands carry
    generation 2 alone) is red on that road."""
    scene, nid, pipeline, lands = pipeline_scene
    from CAD_0.ui import display_pipeline as dp
    import CAD_0
    from CAD_0 import sdf as sdf_mod

    real_derive = CAD_0.scene.derive_display_mesh
    release_a = threading.Event()
    entered_a = threading.Event()
    main_thread = threading.current_thread()
    derive_threads = []

    def controlled_derive(snapshot, resolution):
        if resolution == dp.DRAFT_RESOLUTION and not release_a.is_set():
            # Generation 1's draft: park inside the worker until the
            # test has superseded it — the in-flight mesh the gate
            # must refuse.
            entered_a.set()
            release_a.wait(timeout=30.0)
        derive_threads.append(threading.current_thread())
        return real_derive(snapshot, resolution)

    pipeline._derive = controlled_derive

    # Edit A: generation 1, its draft parks in the worker.
    node = scene.get_node(nid)
    node.body = sdf_mod.sphere(0.9)
    node.mesh_dirty = True
    pipeline.on_edit()
    assert entered_a.wait(timeout=10.0), "the worker never entered A's derive"
    assert pipeline.generation == 1

    # Edit B: generation 2 supersedes — B's own body, its own ladder.
    node.body = sdf_mod.sphere(0.7)
    node.mesh_dirty = True
    pipeline.on_edit()
    assert pipeline.generation == 2

    # Release A's derive: its result completes and is posted — the
    # gate must drop it at the drain (generation 1 is dead).
    release_a.set()
    assert pipeline.wait_until_settled()

    # THE ASSERTION: generation 2's draft landed; generation 1's
    # result NEVER did — not as a landing, not into the scene.
    assert lands.entries, "generation 2's draft should have landed"
    assert all(g == 2 for (g, _, _, _) in lands.entries), (
        "an obsolete result of a superseded generation landed — "
        "the gate did not hold")
    # The scene was never written by the cancelled pass: still dirty
    # (only a FINE landing writes it, and no fine was scheduled).
    assert scene.get_node(nid).mesh_dirty

    # The ladder completes for the live generation: the freshest
    # body's fine lands and clears the dirty flag.
    pipeline.on_release()
    pipeline.on_idle()
    assert pipeline.wait_until_settled()
    assert not scene.get_node(nid).mesh_dirty
    assert scene.get_node(nid).cached_mesh.triangle_count() > 0
    # And every derive ran OFF the main thread (the worker's own
    # witness — the third controprova's instrument, recalled here).
    assert all(t is not main_thread for t in derive_threads)


# ---------------------------------------------------------------------------
# 5 — CONTROPROVA (3): an async evaluation never mutates the scene
#     from the worker
# ---------------------------------------------------------------------------

def test_controprova_3_the_worker_never_mutates_the_scene(pipeline_scene):
    """THE OWNERSHIP RULE. The worker derives over its OWN snapshot;
    the scene's cached_mesh is written ONLY by the fine landing, on
    the OWNER (main) thread. Witnessed three ways:

      * the lands all happen on the MAIN thread (the delivery's own
        thread identity — the naive worker-that-writes would land, or
        worse write, from the worker);
      * after the draft and the medium, the node's cached_mesh is
        STILL the pre-edit mesh and the dirty flag is STILL set — the
        levels are display-pipeline state alone (the naive road that
        calls update_meshes from the worker clears the flag and
        installs the COARSE mesh — this assertion is red there);
      * the fine landing alone writes the scene — and it happens on
        the main thread, through land_fine_mesh.
    """
    scene, nid, pipeline, lands = pipeline_scene
    from CAD_0.ui import display_pipeline as dp
    from CAD_0 import sdf as sdf_mod
    import hashlib
    import numpy as np

    # The settle-first baseline: the node's fine mesh, in place.
    pipeline.on_edit()
    pipeline.on_release()
    pipeline.on_idle()
    assert pipeline.wait_until_settled()
    assert not scene.get_node(nid).mesh_dirty

    def cached_digest():
        mesh = scene.get_node(nid).cached_mesh
        pos = np.ascontiguousarray(mesh.positions, dtype="<f4")
        idx = np.ascontiguousarray(mesh.indices, dtype="<u4")
        h = hashlib.blake2b(digest_size=8)
        h.update(pos.view(np.uint8))
        h.update(idx.view(np.uint8))
        return h.hexdigest()

    before = cached_digest()

    # The edit: a new body, the gesture's ladder. The worker derives
    # it all — the drafts and the medium NEVER touch cached_mesh.
    node = scene.get_node(nid)
    node.body = sdf_mod.sphere(0.5)
    node.mesh_dirty = True
    pipeline.on_edit()
    pipeline.on_release()
    assert pipeline.wait_until_settled()
    assert dp.LOD_MEDIUM in lands.levels()

    # THE ASSERTIONS: the scene unmoved by the async pass —
    assert scene.get_node(nid).mesh_dirty, (
        "the worker cleared the dirty flag — the scene was mutated "
        "off the owner thread (or the medium wrote the cache)")
    assert cached_digest() == before, (
        "cached_mesh moved during the draft/medium pass — the "
        "commutation degraded the data: the contractual mesh is the "
        "FINE one alone")

    # Every landing so far happened on the MAIN thread.
    main_thread = threading.current_thread()
    assert all(t is main_thread for t in lands.threads()), (
        "a landing was delivered off the owner thread")

    # The fine alone lands into the scene — on the main thread.
    pipeline.on_idle()
    assert pipeline.wait_until_settled()
    assert not scene.get_node(nid).mesh_dirty
    assert cached_digest() != before   # the fine's own mesh, installed


# ---------------------------------------------------------------------------
# 6 — THE SYNC BARRIER
# ---------------------------------------------------------------------------

def test_the_barrier_settles_the_current_recipe_at_fine(pipeline_scene):
    """flush_fine: the exports' and picks' door. Mid-ladder (a draft
    landed, the fine not scheduled), the barrier derives the dirty
    visible set at FINE on the CALLER's thread — the consumer reads
    the CURRENT recipe's full-resolution mesh, exactly the static
    path's own answer."""
    scene, nid, pipeline, lands = pipeline_scene
    from CAD_0.ui import display_pipeline as dp
    from CAD_0 import sdf as sdf_mod

    node = scene.get_node(nid)
    node.body = sdf_mod.sphere(0.8)
    node.mesh_dirty = True
    pipeline.on_edit()
    assert pipeline.wait_until_settled()
    assert lands.levels() == [dp.LOD_DRAFT]
    assert scene.get_node(nid).mesh_dirty

    # The barrier: synchronous, on this (main) thread.
    assert pipeline.flush_fine()
    assert not scene.get_node(nid).mesh_dirty
    assert scene.get_node(nid).cached_mesh.triangle_count() > 0

    # The barrier's fine is the sync road's own answer: a scheduled
    # idle afterwards finds nothing dirty (the vacuous pass — the
    # ladder's dedup witness) and lands nothing.
    pipeline.on_idle()
    assert pipeline.wait_until_settled()
    assert dp.LOD_FINE not in lands.levels()


# ---------------------------------------------------------------------------
# 7 — NO MIXED STATE: one job, one delivery
# ---------------------------------------------------------------------------

def test_no_mixed_state_one_job_lands_atomically():
    """A two-node scene, both dirty: ONE job covers both nodes and ONE
    delivery lands both — the mirror never holds the previous mesh of
    one node beside the new mesh of the other (the previous-or-new
    rule, witnessed at the landing's own boundary)."""
    if not _kernel_available():
        pytest.skip("the native kernel is not importable")
    from CAD_0 import sdf as sdf_mod
    from CAD_0 import scene as scene_mod
    from CAD_0.ui.display_pipeline import AsyncDisplayPipeline

    scene = scene_mod.Scene()
    a = scene.add_node("A", sdf_mod.sphere(1.0))
    b = scene.add_node("B", sdf_mod.box(
        __import__("CAD_0").math.Vec3f(0.5, 0.5, 0.5)))
    lands = _Lands(scene)
    pipeline = AsyncDisplayPipeline(
        scene=scene, fine_resolution_fn=lambda: 48, land_fn=lands)
    try:
        pipeline.on_edit()
        assert pipeline.wait_until_settled()
        # THE ATOMIC DELIVERY: one landing, BOTH nodes in it.
        assert len(lands.entries) == 1
        _, level, node_ids, _ = lands.entries[0]
        assert level == "draft"
        assert set(node_ids) == {a, b}
    finally:
        pipeline.stop()
