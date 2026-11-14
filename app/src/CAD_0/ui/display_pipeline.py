"""CAD_0.ui.display_pipeline — PERF 4: the async display derivation
and its three LOD levels (the gesture's road).

THE PROBLEM THIS MODULE CLOSES (the register's PERF 1 row, its own
attribution): the edit act's cost is ~86% re-tessellation — "the frame
is innocent, the mesh rebuild is the edit's cost". Until PERF 4 the
whole derivation (the field evaluation, the tessellation, the display
normals) ran ON THE UI THREAD inside the gesture handler: the user
dragged a parameter and the interface blocked on the derivation of
every step.

THE ARCHITECTURE, every rule named (the brief's own three levels and
three FASE 3 clauses, made code):

THE THREE LEVELS (FASE 2 — the commutation rules NAMED, each one
testable alone):

  * BOZZA (LOD_DRAFT, resolution 12) — DURING the gesture: every edit
    event schedules a draft. Lowest resolution, lowest latency: the
    first thing the user sees after a gesture step.
  * MEDIO (LOD_MEDIUM, resolution 24) — AT THE GESTURE'S RELEASE: the
    declared end of the gesture (a canvas drag's release; a discrete
    commit IS its own release — the property panel's spinbox has no
    "while held" signal to read, and the honest model of a discrete
    edit is a gesture that begins and ends in the same commit).
  * FINE (LOD_FINE, the display resolution — 48, TODAY's own, the
    static path's) — AT THE DECLARED IDLE: a FIXED threshold
    (IDLE_FINE_DELAY_MS, a named constant — never a runtime-measured,
    machine-dependent value) after the last edit/release event.

The draft and medium resolutions are FIXED NAMED CONSTANTS (the mesh
at a level is a pure function of (body, resolution) — bit-identical
across runs and machines, ADR-0014's own law); the FINE level's
resolution is the widget's own display resolution, the one the static
path has always installed — the contract (PERF 1's own words):
"exports and the bridge read those from the node, byte-identically".
The levels are DISPLAY-ONLY: the node's cached_mesh is always the FINE
mesh (drafts and mediums live in this pipeline's caller-side display
state and never touch the scene — the commutation never degrades the
data).

THE ASYNC (FASE 3):
  1. THE GESTURE RETURNS IMMEDIATELY. The edit handler schedules a
     draft and returns; the derivation runs on ONE worker thread (a
     plain Python thread — deliberately NOT the TBB pool: the display
     worker is a UI service, and the kernel's parallel road (PERF 3)
     already rides each mesh's own arithmetic inside the derive).
  2. THE WORKER READS ITS OWN DATA. The scene's thread-ownership rule
     (scene.hpp, the DisplaySourceSnapshot seat): the UI thread owns
     every mutation; the worker derives from SNAPSHOTS taken on the UI
     thread at schedule time (scene.snapshot_display_source) and never
     touches the mutable scene. The heavy call
     (scene.derive_display_mesh) releases the GIL for its whole body
     (the binding's own call_guard) — the UI thread stays live while
     the worker tessellates. The job carries its RESOLUTION captured
     at schedule time: the worker reads nothing on the UI side.
  3. CANCELLATION IS THE GENERATION GATE. Every edit bumps the
     generation; a job of an older generation is dropped before its
     first node (checked per job) and between nodes (checked per node
     — a multi-node stale pass aborts early), and a result that
     completes after its generation was superseded is dropped at the
     drain boundary — it never lands, not in the cache, not in the
     viewport. The in-flight single mesh is not preempted mid-mesh
     (marching_cubes has no abort hook, and the PERF 2 pins own that
     file's arithmetic): its result dies at the gate instead — the
     observable contract ("an obsolete result never lands") is exactly
     what the permanent test pins.
  4. NO MIXED STATE. A job's results land as ONE delivery: the mirror
     is rebuilt atomically after the whole job's nodes are in — the
     viewport draws the previous mirror or the new one, never a mix.
     Within the mirror, each node's entry carries its own level and
     digest (PERF 1's witness): the untouched nodes keep their FINE
     meshes (byte-identical, digest-matched, ZERO re-upload — PERF 1's
     granularity carries over), the edited node walks
     draft -> medium -> fine.
  5. TESTS OBSERVE EXPLICIT SYNC POINTS. wait_until_settled() joins the
     worker with threading primitives (never a time sleep); the
     derive function is injectable so a test can BLOCK the worker on
     an Event and release it when the test chooses — the completion is
     observed, never timed.

THE SYNC BARRIER (flush_fine): every consumer of the node's
cached_mesh OUTSIDE the display pipeline (the exports, the pick
bounds) must see the CURRENT recipe's FINE mesh — the barrier drops
pending async work (a new epoch makes it redundant) and derives the
dirty visible set at FINE on the CALLER thread (the owner), exactly
the static path's own behavior (the same derive, the same bytes).
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any, Callable, Optional

# ---------------------------------------------------------------------------
# THE LEVELS — named constants, the commutation rules' own vocabulary
# ---------------------------------------------------------------------------

LOD_DRAFT = "draft"      # la bozza: during the gesture
LOD_MEDIUM = "medium"    # il medio: at the gesture's release
LOD_FINE = "fine"        # il fine: at the declared idle — the static
                         # path's own resolution, the contractual mesh

# The draft and medium resolutions: FIXED, NAMED, never measured at
# runtime (the mesh at a level is a pure function of (body,
# resolution) — the determinism the suite pins). The FINE level's
# resolution is the widget's own display resolution (48 today): the
# levels are a display question; the contractual mesh is the fine one.
DRAFT_RESOLUTION = 12
MEDIUM_RESOLUTION = 24

# THE DECLARED IDLE: the fixed time threshold after the last edit or
# release event at which the FINE level is scheduled. A named constant
# (the brief's own law: "soglia temporale fissata, non misurata a
# runtime in modo dipendente dalla macchina") — the widget arms a
# single-shot QTimer of exactly this many milliseconds; the tests call
# on_idle() directly (the switch is the rule, the timer is the mezzo).
IDLE_FINE_DELAY_MS = 200

# The kinds that carry a derivation (the honest dispatch, ADR-0002 —
# an imported mesh IS the geometry, a sketch is 2D data: the pipeline
# never schedules them, and the C++ derive refuses them loudly).
_DERIVABLE_KINDS = ("sdf", "fillet", "ellipsoid")


def level_resolution(level: str, fine_resolution: int) -> int:
    """The resolution of a level — the one mapping the ladder rides."""
    if level == LOD_DRAFT:
        return DRAFT_RESOLUTION
    if level == LOD_MEDIUM:
        return MEDIUM_RESOLUTION
    if level == LOD_FINE:
        return int(fine_resolution)
    raise ValueError(f"unknown LOD level {level!r}")


class _Job:
    """One scheduled derivation pass: a generation, a level, the
    resolution CAPTURED AT SCHEDULE TIME (the worker reads nothing on
    the UI side), and the node snapshots (taken at schedule time, on
    the owner thread — the ownership rule's own act)."""

    __slots__ = ("generation", "level", "resolution", "nodes")

    def __init__(self, generation: int, level: str, resolution: int,
                 nodes: list) -> None:
        self.generation = generation
        self.level = level
        self.resolution = resolution
        # list of (node_id, DisplaySourceSnapshot, node_name)
        self.nodes = nodes


class AsyncDisplayPipeline:
    """The gesture's display road: the LOD ladder, one worker thread,
    the generation gate.

    THREAD ROLES (the ownership rule, binding):
      * The UI thread (the owner) calls every mutating method:
        on_edit, on_release, on_idle, flush_fine, reset, stop, drain.
      * The worker thread runs _worker_loop alone: it reads the job
        queue and the generation counter under the lock and calls
        derive_fn (the GIL-free C++ derive) — it never touches the
        scene, the mirror, or any widget.
    """

    def __init__(
        self,
        scene: Any,
        fine_resolution_fn: Callable[[], int],
        land_fn: Callable[[int, str, list], None],
        derive_fn: Optional[Callable[[Any, int], Any]] = None,
        notify_fn: Optional[Callable[[], None]] = None,
    ) -> None:
        """`scene` — the CAD_0.scene.Scene (read for snapshots and the
        dirty walk; written ONLY through land_fn/flush_fine on the UI
        thread — never by the worker).
        `fine_resolution_fn` — answers the CURRENT fine resolution
        (the widget's own; read at schedule time, on the UI thread).
        `land_fn(generation, level, results)` — the delivery, ALWAYS
        called on the UI thread from drain(): results is a list of
        (node_id, DisplayDerivation).
        `derive_fn(snapshot, resolution)` — the heavy call; the real
        CAD_0.scene.derive_display_mesh by default, injectable for the
        deterministic tests (a stub that blocks on an Event is the
        explicit sync point the suite demands).
        `notify_fn()` — called FROM THE WORKER THREAD when a result is
        ready (the widget marshals to the UI thread via a queued Qt
        signal — the hop is the caller's business; tests leave it None
        and call drain()/wait_until_settled() themselves).
        """
        self._scene = scene
        self._fine_resolution_fn = fine_resolution_fn
        self._land_fn = land_fn
        self._notify_fn = notify_fn

        import CAD_0 as _cad0
        self._derive = _cad0.scene.derive_display_mesh
        self._snapshot = _cad0.scene.snapshot_display_source
        if derive_fn is not None:
            self._derive = derive_fn

        # THE SYNC CORE — one condition, every state change under it:
        # the worker's BUSY flag flips to True under the lock BEFORE
        # the job leaves the queue, so a waiter that reads
        # (idle, jobs-empty, completed-empty) ATOMICALLY under the
        # same lock can never observe "settled" while a job is in the
        # worker's hands (the event-based witness had exactly that
        # window: set-then-get-then-clear). wait_until_settled is the
        # explicit sync point the suite demands — a condition wait,
        # never a time sleep.
        self._cv = threading.Condition()
        self._generation = 0            # the mirror epoch
        self._jobs: "queue.SimpleQueue[Optional[_Job]]" = queue.SimpleQueue()
        self._completed: list = []      # (generation, level, results)
        self._worker = threading.Thread(
            target=self._worker_loop, name="CAD_0-display-worker",
            daemon=True)
        self._worker_started = False    # lazy: the first schedule starts it
        self._worker_idle = True        # under _cv: no job in hand
        self._stop_requested = False    # under _cv: the sentinel's flag
        # (generation, level) already queued or delivered — the
        # ladder's own dedup witness (a level runs once per
        # generation, however often the switch is thrown).
        self._scheduled: set = set()

    # ----- the ladder (UI thread) -----------------------------------------

    def on_edit(self) -> None:
        """THE BOZZA RULE: an edit event opens a new generation and
        schedules the draft. The gesture handler returns immediately —
        the derivation is the worker's business now."""
        with self._cv:
            self._generation += 1
            self._scheduled.clear()
            self._schedule_unlocked(self._generation, LOD_DRAFT)

    def on_release(self) -> None:
        """IL MEDIO RULE: the gesture's declared end schedules the
        medium — the SAME generation (a release never opens a new one;
        the release's medium belongs to the gesture that just ended)."""
        with self._cv:
            self._schedule_unlocked(self._generation, LOD_MEDIUM)

    def on_idle(self) -> None:
        """IL FINE RULE: the declared idle threshold elapsed — the
        fine level, the contractual mesh, the same generation."""
        with self._cv:
            self._schedule_unlocked(self._generation, LOD_FINE)

    def _schedule_unlocked(self, generation: int, level: str) -> None:
        """Schedule a level of a generation — caller holds the lock
        (every schedule site is a switch with its rule named above).
        Snapshots the dirty visible set HERE, on the owner thread (the
        ownership rule's own act), captures the resolution HERE (the
        worker reads nothing on the UI side), and enqueues."""
        if generation <= 0:
            return                      # no gesture yet — nothing to schedule
        if (generation, level) in self._scheduled:
            return                      # the ladder's dedup
        if generation != self._generation:
            return                      # superseded between bump and here
        nodes = []
        count = self._scene.node_count()
        for i in range(count):
            node = self._scene.node_at(i)
            if node is None or not node.visible or not node.mesh_dirty:
                continue
            if getattr(node, "kind", "sdf") not in _DERIVABLE_KINDS:
                continue
            nodes.append((node.id, self._snapshot(node), node.name))
        self._scheduled.add((generation, level))
        if not nodes:
            self._cv.notify_all()       # a vacuous pass is still a state change
            return                      # nothing dirty: a vacuous pass
        if not self._worker_started:
            self._worker.start()
            self._worker_started = True
        resolution = level_resolution(level, self._fine_resolution_fn())
        self._jobs.put(_Job(generation, level, resolution, nodes))
        self._cv.notify_all()           # wake the parked worker

    # ----- the worker ------------------------------------------------------

    def _worker_loop(self) -> None:
        while True:
            # Park: mark idle under the condition, then wait for a
            # job. The flip to busy happens UNDER THE LOCK as the job
            # is taken — a waiter can never see idle+empty while a
            # job is in hand.
            with self._cv:
                while True:
                    try:
                        job = self._jobs.get_nowait()
                        break
                    except queue.Empty:
                        self._worker_idle = True
                        self._cv.notify_all()
                        self._cv.wait(timeout=0.5)
                        if self._stop_requested:
                            return      # stop() — seen at the parking lot
                self._worker_idle = False
                current = self._generation
            if job is None:
                with self._cv:
                    self._stop_requested = True
                    self._cv.notify_all()
                return                  # stop() — the sentinel
            if job.generation != current:
                continue                # CANCELLED before the first node
            results = []
            for (node_id, snapshot, name) in job.nodes:
                with self._cv:
                    current = self._generation
                if job.generation != current:
                    # CANCELLED between nodes: the stale pass aborts
                    # early, its partial work never posted.
                    results = None
                    break
                try:
                    derivation = self._derive(snapshot, job.resolution)
                except Exception as e:  # noqa: BLE001
                    # A derivation that throws is named, never silent,
                    # and never kills the worker: the display keeps
                    # the previous mesh (the previous-or-new rule) and
                    # the words go to the status log (the [viewport]
                    # convention).
                    print(f"[viewport] display derivation failed for "
                          f"node {node_id} ({name!r}) at level "
                          f"{job.level}: {e} — the previous mesh stands")
                    continue
                results.append((node_id, derivation))
            if not results:
                continue                # nothing survived (cancelled or all failed)
            with self._cv:
                current = self._generation
                if job.generation == current:
                    self._completed.append(
                        (job.generation, job.level, results))
                    posted = True
                else:
                    posted = False      # CANCELLED before the post
            if posted and self._notify_fn is not None:
                try:
                    self._notify_fn()   # worker thread — the hop to the
                                        # UI thread is the caller's own
                except Exception:       # noqa: BLE001
                    pass

    # ----- the delivery (UI thread) ----------------------------------------

    def drain(self) -> int:
        """Deliver every completed result to land_fn — UI THREAD ONLY.
        THE GENERATION GATE LIVES HERE: a result whose generation is
        not the current one is dropped WITHOUT delivery (an obsolete
        result never lands — not in the cache, not in the viewport:
        the permanent test's own assertion). Returns the number of
        deliveries made."""
        delivered = 0
        while True:
            with self._cv:
                if not self._completed:
                    return delivered
                generation, level, results = self._completed.pop(0)
                current = self._generation
            if generation != current:
                # Superseded between the worker's post and this drain:
                # the result dies here — named, never landed.
                print(f"[viewport] display result of generation "
                      f"{generation} (level {level}) dropped — "
                      f"superseded by generation {current}")
                continue
            self._land_fn(generation, level, results)
            delivered += 1

    def wait_until_settled(self, timeout: Optional[float] = 60.0) -> bool:
        """THE EXPLICIT SYNC POINT (deterministic tests, the brief's
        own law): block until the worker sits idle (no derive in
        flight) and both queues are empty, then drain. Uses threading
        primitives — NEVER a time sleep; the timeout is a safety valve
        against a wedged worker, not part of the contract. Returns
        False on timeout (the caller decides what that means)."""
        deadline = time.monotonic() + timeout
        while True:
            # Drain FIRST (it only delivers, never schedules): the
            # completed results are emptied BY the drain, never by
            # waiting on them.
            self.drain()
            with self._cv:
                if (self._worker_idle and self._jobs.empty()
                        and not self._completed):
                    return True
                if time.monotonic() >= deadline:
                    return False
                self._cv.wait(timeout=max(
                    0.01, deadline - time.monotonic()))

    # ----- the barriers (UI thread) ----------------------------------------

    def flush_fine(self) -> bool:
        """THE SYNC BARRIER (the exports' and the picks' door): the
        visible dirty set derived at FINE on the CALLER thread (the
        owner — the static path's own behavior, byte-identical), the
        pending async work dropped (a new epoch makes it redundant).
        Returns True if any node landed (the caller rebuilds)."""
        with self._cv:
            self._generation += 1
            self._scheduled.clear()
            self._cv.notify_all()
        changed = False
        count = self._scene.node_count()
        for i in range(count):
            node = self._scene.node_at(i)
            if node is None or not node.visible or not node.mesh_dirty:
                continue
            if getattr(node, "kind", "sdf") not in _DERIVABLE_KINDS:
                continue
            snapshot = self._snapshot(node)
            try:
                derivation = self._derive(
                    snapshot, self._fine_resolution_fn())
            except Exception as e:  # noqa: BLE001
                print(f"[viewport] flush_fine: derivation failed for node "
                      f"{node.id} ({node.name!r}): {e} — the previous "
                      "mesh stands")
                continue
            if self._scene.land_fine_mesh(node.id, derivation.mesh):
                changed = True
        return changed

    def supersede(self) -> None:
        """The SYNC road's epoch kill: a synchronous refresh (a load,
        an undo, a visibility flip — the static path's own callers)
        has just rebuilt the mirror from the live scene; every pending
        async result belongs to a world that no longer exists. The
        epoch moves: in-flight results die at the drain gate, queued
        jobs die at the worker's. The display state (the caller's
        side) is the CALLER's to clear — this method only owns the
        pipeline's own arithmetic."""
        with self._cv:
            self._generation += 1
            self._scheduled.clear()
            self._cv.notify_all()

    def reset(self) -> None:
        """The scene was replaced wholesale (new/open/clear): every
        pending job belongs to a scene that no longer exists — the
        epoch moves, the queues are emptied, nothing of the old scene
        can land in the new one (node ids could even collide)."""
        with self._cv:
            self._generation += 1
            self._scheduled.clear()
            while True:
                try:
                    self._jobs.get_nowait()
                except queue.Empty:
                    break
            self._completed.clear()
            self._cv.notify_all()

    def stop(self) -> None:
        """The widget is going away: the worker exits at the next job
        boundary (the sentinel). The thread is a daemon — the process
        never waits on it."""
        with self._cv:
            self._stop_requested = True
            self._cv.notify_all()
        self._jobs.put(None)
        with self._cv:
            self._stop_requested = True
            self._cv.notify_all()
        if self._worker_started:
            self._worker.join(timeout=2.0)

    # ----- introspection (the tests' witnesses) -----------------------------

    @property
    def generation(self) -> int:
        """The current epoch — the tests' own witness (the gate's
        arithmetic is observable, not assumed)."""
        with self._cv:
            return self._generation

    @property
    def worker_thread(self) -> Optional[threading.Thread]:
        """The worker's own identity — the never-the-main-thread
        witness (the third controprova's own instrument)."""
        return self._worker if self._worker_started else None
