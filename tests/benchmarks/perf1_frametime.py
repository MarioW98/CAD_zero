#!/usr/bin/env python3
# tests/benchmarks/perf1_frametime.py
#
# PERF 1 — THE FRAME-TIME ROW (the benchmark suite's second row,
# beside the m8 row zero, never in its place): the frame-time of the
# STABLE path — the VBO/IBO cache's own seat, Phase C.6's registered
# one (the register's G row). The row zero (the lag profile, the m8
# suite) measures the EDIT end to end — from the gesture to the
# ledger updated; this row measures the FRAME — what the user sees
# per paint — over the register's own corpus, the chain-5 (box#1 ->
# sk#2 -> ext#3 -> sub#4 -> fil#5, the m8 bench's own construction,
# quoted): the same bodies the ledgers and the gates hold, never a
# corpus the register does not know.
#
# THE THREE ACTS (the frame's own three faces, each timed alone; the
# timed regions are declared at each site — the kernel acts and the
# camera moves are OUTSIDE them, the row zero's territory and the
# gesture, never the frame):
#   * THE FIRST SIGHT — the scene-replaced state (the removal rule's
#     own door: the old buffers handed back, the cache cold): every
#     visible feature uploads once, three buffers each;
#   * THE STABLE FRAME — the orbit: the camera moves outside the
#     timed region, the mesh bytes do not move at all, the cache
#     serves with ZERO uploads — the m6-correttiva grammar riding
#     PERF 1's buffers, one draw call per range;
#   * THE EDIT — the m6 corpus's own radius edit (r 0.25 <-> 0.125,
#     the D-chain's own values): the kernel act (set_fillet_radius,
#     the row zero's own territory) UNTIMED, then the frame's own
#     path TIMED (the mirror rebuild — the re-tessellation of the
#     ONE touched node, the display normals, the digest's own price —
#     and the paint with ONE feature re-uploading).
#
# THE CORPUS'S FRAME, declared: the chain-5's visible body is the
# fillet alone (the Apply's own hiding pattern — the operands step
# aside), 38,432 triangles at the display resolution 48. The
# multi-featured granularity — the edit touching ONE feature among
# many, the others served with zero uploads — is the correctness
# suite's own seat (bindings/python/tests/test_perf1_vbo_cache.py,
# the CI member): the same door, never a second opinion.
#
# THE METHODOLOGY, m8's own (declared, never implied):
#   * the inputs are FIXED IN THE CODE — the corpus's parameters and
#     the two radii, no time source, no unseeded random;
#   * warmup 3, iterations 21 (odd: the median is a sample), the
#     central number the MEDIAN (never the minimum, never the raw
#     mean), p95 beside it (nearest-rank);
#   * the measuring machine is declared on the first printed lines —
#     platform and GL renderer — never in a threshold: the CI runner
#     is not the maintainer's machine, the absolute numbers are
#     indicative, THE RATIOS ARE THE SUBSTANCE;
#   * THE DIGESTS BEFORE THE TIMES: every iteration verifies the
#     mirror's digest against the pinned corpus digests (the edit's
#     alternation included — both radii's digests are pinned), the
#     cache's served label against the mirror's, and the act's upload
#     budget against its declared count — a frame that uploaded what
#     it should not, or served a digest it does not declare, is a
#     frame whose time is not believed. A performance number bought
#     with a determinism drift is not a number (ADR-0014, the m8
#     row's own clause).
#
# THE GATE, RELATIVE ONLY (the m8 law): a WITHIN-RUN A/B regression —
# pass B's median over pass A's, PER ACT, red beyond 3.0x (declared),
# the ratio's denominator floored at 1 us (declared, so a
# nanosecond act cannot counterfeit a regression) — never a second
# pinned on a machine that is not the one reading. The between-act
# ratios are the PROFILE the row carries (which act owns the frame),
# never a threshold.
#
# THE COUNTERPROOFS (a gate that cannot say no is a decorative
# number, not a gate):
#   * THE SLOWED ACT — the stable act measured with a DECLARED dummy
#     spin injected into its timed region (pure arithmetic load, the
#     accumulator sunk in a module global, touching NOTHING the
#     digests read), calibrated at runtime to burn >= 4x the honest
#     median: the gate goes RED with the NAME of the slowed act, the
#     other two green beside it (the controls);
#   * THE DIGEST DRIFT — a pass whose mirror digest differs by one
#     character: the gate REFUSES to judge the times (the digest
#     identity is the FIRST predicate — the times are not believed).
#
# THE VERDICT: the DETERMINISTIC half of everything measured — the
# corpus pins (the rim string, the two mirror digests, the chain
# digests' FNV), the acts' upload budgets and draw counts, the gate's
# booleans, both counterproofs' outcomes — rendered as one text,
# FNV-1a 64 PINNED. The times are printed in the report beside it
# and never pinned: they are the machine's own, the verdict is the
# suite's own. The bench exits 0 only when every check holds.
#
# THE POSTURE (tests/benchmarks/CMakeLists.txt's own, the m8
# declaration): NOT a ctest member — the shared CI runner is not the
# measuring machine and a timing gate on machines whose noise nobody
# pinned is a decorative number. The suite runs BY THE RITE, on the
# measuring machine, the same presets the CI builds. When the bench
# finds NO GL context (the CI runner's own case) it runs the
# deterministic half — the corpus witnesses, the mirror digests — and
# SKIPS the timing DECLARING it, exit 0: never a silence, never a red
# that means nothing (the GL/PySide6 skip model already in the house).
# The cache's full correctness — the invalidation contract, the
# rancid counterproof — is the pytest member's own seat.
#
# NO NEW DEPENDENCIES: the standard library, numpy, and the project's
# own package (PySide6/PyOpenGL where the frame runs — the same
# requirements the app itself declares).
#
# Usage (the rite, the measuring machine; the Windows commands are in
# the register's PERF 1 row, verbatim):
#
#   PYTHONPATH=<build>/<preset>/python \
#       python tests/benchmarks/perf1_frametime.py
#
# The first clean run on the measuring machine pins the three FNV
# constants (printed, then written into the source — the m7/m8
# calibration pattern); every run after compares them.
#
# Exit code: 0 only when every deterministic check holds (and the
# timing, when it ran, passed its gate).

from __future__ import annotations

import os
import platform
import sys
import time

# ---------------------------------------------------------------------------
# THE DECLARED CONSTANTS — the methodology, pinned where it runs.
# ---------------------------------------------------------------------------

WARMUP = 3                 # declared warmup, discarded
ITERATIONS = 21            # measured, odd -> the median is a sample
SABOTAGE_ITERATIONS = 7    # the counterproof's own count (odd too)
GATE_RATIO = 3.0           # the within-run A/B red line
GATE_FLOOR_S = 1e-6        # 1 us: the ratio's denominator floor
SABOTAGE_BUDGET_X = 4.0    # the spin burns >= 4x the honest median

# The corpus — the m8 bench's own chain-5, its rim string pinned (the
# m8 LoadedChain's own pin, quoted) and the two radii the D-chain and
# the m6 corpus already ride (0.25 <-> 0.125).
RIM_STRING = "sub#4:isect[box#1:top \u00d7 ext#3:lat(0)]@hi"
RADIUS_A = 0.25
RADIUS_B = 0.125

# The corpus's DISPLAY identity — the mirror digest at each radius
# (blake2b-64 over the display payload, the viewport's own
# instrument). CALIBRATION NOTE (the m7/m8 pattern): read from this
# suite's own first clean run and pinned; any drift in the
# tessellator, the display normals or the mirror's construction moves
# them and the bench goes red before one time is believed.
PIN_MIRROR_DIGEST_A = "e475fc64517a9097"
PIN_MIRROR_DIGEST_B = "345301fe7be5aa71"

# The BEFORE's own deterministic count — the m6 road's per-frame
# vertex payload (the client arrays the driver re-uploads at every
# glDrawElements): the mirror entry's three buffers, in bytes. Read
# from the first clean run and pinned; it is the register's own
# before/after form (the cause named, the bound declared, before and
# after measured).
PIN_M6_ROAD_PAYLOAD_BYTES = 945024

# The witnesses — the m8 corpus recall: the chain digests of the
# blind body (sub#4) and the fillet (fil#5), FNV-1a 64 over the
# brep::chain_digest texts. The bench measures the SAME bodies the
# m8 suite measured; a corpus that drifted is a corpus whose times
# say nothing about the register's world. CALIBRATION NOTE: pinned at
# the first clean run (printed by this bench, then written here).
PIN_BLIND_CHAIN_FNV = 0xAAB7839C175A890D
PIN_FIL_CHAIN_FNV = 0xE30D3C50988400E2

# The verdict's own pin — the deterministic half's FNV, compared on
# every run after the calibration. CALIBRATION NOTE: pinned at the
# first clean run.
PIN_VERDICT_FNV = 0xAB539AB669D555DC

ACT_NAMES = ("first sight", "stable frame", "edit")

# ---------------------------------------------------------------------------
# The bench's own discipline — every failure collected, each named
# (the m5 gate's own philosophy). The hard stops are for the states
# the next line cannot survive; everything else is collected and
# reported at the exit.
# ---------------------------------------------------------------------------

g_failures = 0


def require(cond, what):
    global g_failures
    if not cond:
        print(f"  [perf1-frametime] FAIL: {what}", flush=True)
        g_failures += 1


def fail_hard(what):
    print(f"[perf1-frametime] FATAL: {what}", flush=True)
    raise SystemExit(1)


def fnv1a64(text: str) -> int:
    """FNV-1a 64 — the m5/m6/m7/m8 suites' own instrument, quoted
    (the cross-run identity's single authority in this house)."""
    h = 0xCBF29CE484222325
    for ch in text.encode("utf-8"):
        h ^= ch
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def hex16(v: int) -> str:
    return f"{v:016x}"


# ---------------------------------------------------------------------------
# THE TIMING HARNESS — time.perf_counter alone (monotonic): the
# median and the p95 (nearest-rank), never the minimum, never the
# raw mean.
# ---------------------------------------------------------------------------

def stats_of(samples):
    s = sorted(samples)
    count = len(s)
    median = s[count // 2]        # count odd: a real sample
    rank = -(-95 * count // 100)  # ceil(0.95 * count), nearest-rank
    p95 = s[min(rank, count) - 1]
    return median, p95


def ms_of(seconds: float) -> float:
    return seconds * 1e3


# THE COUNTERPROOF'S OWN INJECTION — a declared dummy spin: pure
# arithmetic load, the accumulator sunk in a module global (CPython
# has no volatile; the global sink and the deadline loop are the
# honest analog — the loop's arithmetic is the load, it touches
# NOTHING the digests read). Calibrated at runtime: it burns until it
# has spent >= 4x the act's honest median, so the red lands past the
# 3.0x gate on any machine, never by noise.
g_spin_sink = 0.0


def spin_for(budget_s: float) -> None:
    deadline = time.perf_counter() + budget_s
    acc = 0.0
    while time.perf_counter() < deadline:
        for i in range(1, 10001):
            acc += 1.0 / i
    global g_spin_sink
    g_spin_sink = acc


# ---------------------------------------------------------------------------
# THE M6 ROAD — the before, quoted from the committed m6-correttiva
# (839b32e's own _paint_impl/_draw_grid/_draw_axes/_draw_tri_range):
# the grid and the axes as immediate-mode walks, the mesh through
# CLIENT-SIDE vertex arrays — the driver re-uploading the whole
# payload at every glDrawElements. Measured on the same corpus, the
# same machine, the same run (the register's performance-row form:
# before and after measured, never the m6 register row's numbers
# borrowed from a different scene). Its times are PRINTED, never
# gated: the gate stays within-run A/B on the honest road (the m8
# law — a "VBO must be Nx faster" threshold would pin a machine's
# DMA speed, not the code's own).
# ---------------------------------------------------------------------------

def m6_road_frame(viewport, entry):
    """One m6-road frame: the m6-correttiva's committed draw, quoted.

    The setup (clear, projection, view, light) is the current
    _paint_impl's own — the m6 code's setup was identical; the
    drawing is the m6's: 44 glVertex3f for the grid, 6 more for the
    axes (the immediate walks), then the mesh's whole range through
    client-side arrays and ONE glDrawElements — the payload riding
    the call, per frame, forever.
    """
    import math
    from OpenGL.GL import (
        glBegin, glEnd, glVertex3f, glNormal3f,
        GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_PROJECTION,
        GL_MODELVIEW, GL_LIGHTING, GL_LIGHT0, GL_POSITION, GL_DIFFUSE,
        GL_AMBIENT, GL_LINES, GL_TRIANGLES, GL_VERTEX_ARRAY,
        GL_NORMAL_ARRAY, GL_FLOAT, GL_UNSIGNED_INT,
    )
    import CAD_0.ui.viewport_widget as vw
    from ctypes import c_float as GLfloat

    glClearColor = vw.glClearColor
    glClear = vw.glClear
    glMatrixMode = vw.glMatrixMode
    glLoadIdentity = vw.glLoadIdentity
    glFrustum = vw.glFrustum
    glMultMatrixf = vw.glMultMatrixf
    glTranslatef = vw.glTranslatef
    glLightfv = vw.glLightfv
    glColor3f = vw.glColor3f
    glLineWidth = vw.glLineWidth
    glEnableClientState = vw.glEnableClientState
    glDisableClientState = vw.glDisableClientState
    glVertexPointer = vw.glVertexPointer
    glNormalPointer = vw.glNormalPointer
    glDrawElements = vw.glDrawElements

    glClearColor(0.15, 0.15, 0.18, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    fov_y = viewport._cam_fov
    aspect = viewport._cam_aspect
    near_p = 0.01
    far_p = 1000.0
    top = near_p * math.tan(math.radians(fov_y) * 0.5)
    right = top * aspect
    glFrustum(-right, right, -top, top, near_p, far_p)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    cp = math.cos(viewport._cam_pitch)
    sp = math.sin(viewport._cam_pitch)
    cy = math.cos(viewport._cam_yaw)
    sy = math.sin(viewport._cam_yaw)
    cam_dir = (cp * sy, sp, cp * cy)
    tx, ty, tz = viewport._cam_target
    cam_pos = (tx + cam_dir[0] * viewport._cam_distance,
               ty + cam_dir[1] * viewport._cam_distance,
               tz + cam_dir[2] * viewport._cam_distance)
    fx = tx - cam_pos[0]
    fy = ty - cam_pos[1]
    fz = tz - cam_pos[2]
    fl = math.sqrt(fx * fx + fy * fy + fz * fz)
    if fl < 1e-9:
        fx, fy, fz = 0, 0, -1
    else:
        fx, fy, fz = fx / fl, fy / fl, fz / fl
    rx, ry, rz = -fz, 0.0, fx
    rl = math.sqrt(rx * rx + rz * rz)
    if rl > 1e-9:
        rx, rz = rx / rl, rz / rl
    else:
        rx, rz = 1, 0
    ux = ry * fz - rz * fy
    uy = rz * fx - rx * fz
    uz = rx * fy - ry * fx
    m = (GLfloat * 16)(
        rx, ux, -fx, 0, ry, uy, -fy, 0, rz, uz, -fz, 0, 0, 0, 0, 1)
    glMultMatrixf(m)
    glTranslatef(-cam_pos[0], -cam_pos[1], -cam_pos[2])
    glLightfv(GL_LIGHT0, GL_POSITION, [0.5, 0.8, 0.3, 0.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])

    # The grid — the m6 immediate walk (44 vertices).
    vw.glDisable(GL_LIGHTING)
    glColor3f(0.3, 0.3, 0.3)
    glBegin(GL_LINES)
    extent = 5.0
    step = 1.0
    i = -extent
    while i <= extent:
        glVertex3f(i, 0, -extent)
        glVertex3f(i, 0, extent)
        glVertex3f(-extent, 0, i)
        glVertex3f(extent, 0, i)
        i += step
    glEnd()
    vw.glEnable(GL_LIGHTING)

    # The axes — the m6 immediate walk (6 vertices, three colors).
    vw.glDisable(GL_LIGHTING)
    glLineWidth(2.0)
    glColor3f(1, 0, 0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(2, 0, 0)
    glEnd()
    glColor3f(0, 1, 0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 2, 0)
    glEnd()
    glColor3f(0, 0, 1)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 2)
    glEnd()
    glLineWidth(1.0)
    vw.glEnable(GL_LIGHTING)

    # The mesh — the m6 client-array draw: the payload rides the call.
    glColor3f(0.7, 0.75, 0.8)
    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)
    glNormalPointer(GL_FLOAT, 0, entry.normals)
    glVertexPointer(3, GL_FLOAT, 0, entry.positions)
    glDrawElements(GL_TRIANGLES, int(entry.triangle_count * 3),
                   GL_UNSIGNED_INT, entry.indices)
    glDisableClientState(GL_NORMAL_ARRAY)
    glDisableClientState(GL_VERTEX_ARRAY)


# ---------------------------------------------------------------------------
# THE CORPUS — the m8 bench's own chain-5, QUOTED through the Python
# scene doors (the same doors the workbench rides; ids 1..5, the rim
# found by the find, never typed by hand — the grammar checked).
# ---------------------------------------------------------------------------

def build_corpus(scene):
    """box#1 -> sk#2 -> ext#3 -> sub#4 -> fil#5 on the GIVEN scene.

    The construction is the m8 LoadedChain's own (quoted): the box
    [-1,1]^3, the circle r=0.5, the prism [0,2] offset (0,0,-0.5) —
    the BLIND hole, the rim fillet at r=0.25 discovered by the find.
    Returns (fillet_id, rim) with both checked against the pins.
    """
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
    sub_node = scene.get_node(sub)
    rim = None
    for i in range(brep_mod.edge_count(sub_node.exact_brep)):
        e = brep_mod.edge_query(sub_node.exact_brep, i)
        if "@hi" in e["name"] and "isect" in e["name"]:
            rim = e["name"]
            break
    if rim != RIM_STRING:
        fail_hard(f"the corpus: the rim string is not the m8 pin "
                  f"({rim!r} != {RIM_STRING!r})")
    fid = scene.add_fillet_node("Fillet", rim, RADIUS_A)
    # The Apply's own hiding pattern: the operands step aside, the
    # fillet is the frame's one visible body (declared in the header).
    for nid in (bx, sk, ex, sub):
        scene.set_visible(nid, False)
    if fid != 5:
        fail_hard(f"the corpus: the fillet is not node 5 ({fid})")
    return fid, rim


def mirror_digest_of(viewport):
    """The mirror's one digest (the chain-5's single visible body) —
    None when the mirror does not carry exactly one body (a frame
    that carries not-the-corpus is a frame the bench refuses to
    time)."""
    entries = viewport._draw_nodes
    if len(entries) != 1:
        return None
    return entries[0].digest


def chain_digest_fnv(scene, node_id) -> int:
    """FNV-1a 64 over a body's brep::chain_digest text — the m8
    corpus recall's own instrument."""
    from CAD_0 import brep as brep_mod
    node = scene.get_node(node_id)
    body = node.fillet_brep if node.fillet_brep is not None \
        else node.exact_brep
    return fnv1a64(str(brep_mod.chain_digest(body)))


# ---------------------------------------------------------------------------
# THE PASS-THROUGH COUNTERS — the real GL functions wrapped, never
# mocked: every upload and draw counted while it really runs (the
# frames are CERTIFIED, not simulated — a measured frame that
# uploaded what it should not is a frame whose time is not believed).
# ---------------------------------------------------------------------------

class _GlCounters:
    def __init__(self, module):
        self.module = module
        self.uploads = 0
        self.upload_bytes = 0
        self.draws = 0
        self._saved = {}

    def install(self):
        mod = self.module
        real_bufdata = mod.glBufferData
        real_draw = mod.glDrawElements
        self._saved = {"glBufferData": real_bufdata,
                       "glDrawElements": real_draw}

        def bufdata(target, data, usage):
            self.uploads += 1
            try:
                import numpy as np
                self.upload_bytes += int(np.asarray(data).nbytes)
            except Exception:
                pass
            return real_bufdata(target, data, usage)

        def draw(mode, count, type_, indices):
            self.draws += 1
            return real_draw(mode, count, type_, indices)

        mod.glBufferData = bufdata
        mod.glDrawElements = draw

    def restore(self):
        for name, fn in self._saved.items():
            setattr(self.module, name, fn)

    def reset_frame(self):
        self.uploads = 0
        self.upload_bytes = 0
        self.draws = 0


# ---------------------------------------------------------------------------
# THE ACTS — each iteration certified inside (the digests BEFORE the
# time is returned), the timed regions declared at each site.
# ---------------------------------------------------------------------------

def run_act(viewport, gl_window, scene, fid, act, counters, iteration,
            spin_budget_s=0.0):
    """One iteration of one act. Returns (seconds, digest_record):
    the record is the canonical digest identity the gate compares
    (a single digest for the mesh-stable acts, the alternation pair
    for the edit — parity-independent by construction)."""
    if act == "first sight":
        # THE SCENE-REPLACED STATE, untimed: the removal rule's own
        # door — every cached buffer handed back (glDeleteBuffers,
        # the ids die properly, no leak), the cache cold.
        gl_window._drop_absent_buffers(set())
        counters.reset_frame()
        t0 = time.perf_counter()
        gl_window._paint_impl()
        dt = time.perf_counter() - t0
        expected = 3 * len(viewport._draw_nodes)
        require(counters.uploads == expected,
                f"first sight: uploads {counters.uploads} != "
                f"{expected} (3 per visible feature)")
        digest = mirror_digest_of(viewport)
        require(digest == PIN_MIRROR_DIGEST_A,
                f"first sight: the mirror digest is not the pinned "
                f"corpus digest ({digest})")
        record = digest
    elif act == "stable frame":
        # THE ORBIT: the camera moves OUTSIDE the timed region, the
        # mesh bytes do not move at all.
        viewport.set_camera_yaw_pitch(0.05 * (iteration + 1), 0.3)
        counters.reset_frame()
        t0 = time.perf_counter()
        gl_window._paint_impl()
        if spin_budget_s > 0.0:
            spin_for(spin_budget_s)   # the counterproof's injection
        dt = time.perf_counter() - t0
        require(counters.uploads == 0,
                f"stable frame: uploaded {counters.uploads} buffers — "
                "the stable path must upload NOTHING")
        digest = mirror_digest_of(viewport)
        require(digest == PIN_MIRROR_DIGEST_A,
                f"stable frame: the mirror digest drifted ({digest})")
        record = digest
    elif act == "edit":
        # THE RADIUS EDIT: the kernel act UNTIMED (the row zero's own
        # territory), the absolute alternation B, A, B, ... (the pass
        # prelude normalizes to A, so every iteration MOVES the mesh).
        radius = RADIUS_B if iteration % 2 == 0 else RADIUS_A
        answer = scene.set_fillet_radius(fid, radius)
        require(answer == "reference_resolved",
                f"edit: the radius edit refused ({answer})")
        counters.reset_frame()
        t0 = time.perf_counter()
        viewport._refresh_meshes()
        gl_window._paint_impl()
        dt = time.perf_counter() - t0
        require(counters.uploads == 3,
                f"edit: uploaded {counters.uploads} buffers — exactly "
                "the touched feature's three, never the scene")
        want = PIN_MIRROR_DIGEST_B if iteration % 2 == 0 \
            else PIN_MIRROR_DIGEST_A
        digest = mirror_digest_of(viewport)
        require(digest == want,
                f"edit: the mirror digest is not the pinned "
                f"alternation ({digest} != {want})")
        record = "+".join(sorted((PIN_MIRROR_DIGEST_A,
                                  PIN_MIRROR_DIGEST_B)))
    else:
        fail_hard(f"unknown act {act!r}")

    # The served label — the cache's own contract, every act, every
    # iteration: what the cache says it holds is what the mirror
    # declares (the bytes-level oracle is the correctness suite's
    # own seat; here the label and the upload budgets certify).
    for e in viewport._draw_nodes:
        bufs = gl_window._node_buffers.get(e.node_id)
        if bufs is None or bufs.digest != e.digest:
            require(False,
                    f"act {act!r}: the cache's label for node "
                    f"{e.node_id} is not the mirror's digest")
            break
    return dt, record


def run_pass(pass_name, viewport, gl_window, scene, fid, counters,
             acts, iterations, warmup, spin_budget_s=0.0):
    """One measurement pass: the prelude normalizes the corpus to
    r=A (untimed), then per act the warmup is discarded and the
    iterations timed, the digests verified every iteration before one
    time is believed."""
    # THE PASS PRELUDE — the canonical state, untimed.
    node = scene.get_node(fid)
    if abs(float(node.fillet_radius) - RADIUS_A) > 1e-12:
        scene.set_fillet_radius(fid, RADIUS_A)
        viewport._refresh_meshes()

    result = {}
    for act in acts:
        samples = []
        records = set()
        for it in range(warmup + iterations):
            dt, record = run_act(viewport, gl_window, scene, fid, act,
                                 counters, it,
                                 spin_budget_s=spin_budget_s)
            if it >= warmup:
                samples.append(dt)
            records.add(record)
        require(len(records) == 1,
                f"{pass_name} / {act}: the digest record is not "
                f"canonical ({records})")
        median, p95 = stats_of(samples)
        result[act] = {"median": median, "p95": p95,
                       "record": records.pop(),
                       "uploads": counters.uploads,
                       "draws": counters.draws}
        print(f"  [perf1-frametime] {pass_name} / {act}: median "
              f"{ms_of(median):.3f} ms, p95 {ms_of(p95):.3f} ms "
              f"(uploads this frame: {counters.uploads}, draws: "
              f"{counters.draws})", flush=True)
    return result


# ---------------------------------------------------------------------------
# THE GATE — digest identity FIRST (the times believed only after
# it), then the per-act within-run A/B regression. RELATIVE ONLY:
# the ratio 3.0 over medians, the denominator floored at 1 us.
# ---------------------------------------------------------------------------

def frame_gate(pass_a, pass_b, acts):
    verdict = {"pass": True, "digest_identity": True,
               "digest_failure": "", "acts": {}}

    for act in acts:
        if pass_a[act]["record"] != pass_b[act]["record"]:
            verdict["digest_identity"] = False
            verdict["digest_failure"] += (", " if
                                          verdict["digest_failure"]
                                          else "") + act
    if not verdict["digest_identity"]:
        verdict["pass"] = False
        return verdict

    for act in acts:
        denom = max(pass_a[act]["median"], GATE_FLOOR_S)
        ratio = pass_b[act]["median"] / denom
        regressed = ratio > GATE_RATIO
        verdict["acts"][act] = {"ratio": ratio, "regressed": regressed}
        if regressed:
            verdict["pass"] = False
    return verdict


def render_gate(verdict):
    if not verdict["digest_identity"]:
        return (f"DIGEST DRIFT at {verdict['digest_failure']} — the "
                "times are NOT believed (a performance number bought "
                "with a determinism drift is not a number)")
    text = []
    for act, v in verdict["acts"].items():
        if v["regressed"]:
            text.append(f"REGRESSED {act}: pass B's median is "
                        f"{v['ratio']:.2f}x pass A's (beyond the "
                        f"declared {GATE_RATIO}x)")
    if not text:
        return (f"green — every act judged, none regressed beyond "
                f"{GATE_RATIO}x within the run")
    return "; ".join(text)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main() -> int:
    print("[perf1-frametime] PERF 1 — the frame-time row (the "
          "benchmark suite's second row; the methodology declared in "
          "this source's own header)", flush=True)
    print(f"[perf1-frametime] machine: {platform.platform()} "
          f"| python {platform.python_version()}", flush=True)

    pinning = "--pin" in sys.argv
    if pinning:
        print("[perf1-frametime] --pin: the calibration pass (the "
              "m7/m8 pattern) — the FNV constants are PRINTED for "
              "pinning in the source, the verdict's pin is not "
              "compared", flush=True)

    # The application shell — the real widget, the real doors (the
    # workbench suite's own offscreen pattern: the modal prompts
    # answered Discard, a headless box never blocks).
    from PySide6.QtWidgets import QApplication, QMessageBox
    QMessageBox.question = staticmethod(
        lambda *a, **k: QMessageBox.Discard)
    app = QApplication.instance() or QApplication([])
    import CAD_0.ui.viewport_widget as vw
    from CAD_0.ui.main_window import MainWindow

    win = MainWindow()
    win._current_path = None
    fid, rim = build_corpus(win._scene)
    win._rebuild_ui_from_scene()
    viewport = win._viewport
    viewport._refresh_meshes()

    # ------------------------------------------------------------------
    # THE DETERMINISTIC HALF — the corpus witnesses (the measurement's
    # precondition: a bench that has not verified WHAT it measures
    # measures nothing). These run with or without GL.
    # ------------------------------------------------------------------
    entries = viewport._draw_nodes
    require(len(entries) == 1,
            f"the corpus's mirror must carry the one visible body "
            f"({len(entries)} entries)")
    digest = mirror_digest_of(viewport)
    require(digest == PIN_MIRROR_DIGEST_A,
            f"the mirror digest is not the pinned corpus digest "
            f"({digest})")
    tris = entries[0].triangle_count
    require(tris > 0, "the mirror's body has no triangles")
    blind_fnv = chain_digest_fnv(win._scene, 4)
    fil_fnv = chain_digest_fnv(win._scene, fid)
    if PIN_BLIND_CHAIN_FNV is not None:
        require(blind_fnv == PIN_BLIND_CHAIN_FNV,
                "the blind body's chain digest FNV is not the pin")
    if PIN_FIL_CHAIN_FNV is not None:
        require(fil_fnv == PIN_FIL_CHAIN_FNV,
                "the fillet's chain digest FNV is not the pin")
    corpus_line = (f"corpus: rim {rim!r}; mirror digest {digest}; "
                   f"triangles {tris}; blind chain fnv "
                   f"{hex16(blind_fnv)}; fillet chain fnv "
                   f"{hex16(fil_fnv)}")
    print(f"[perf1-frametime] {corpus_line}", flush=True)

    # ------------------------------------------------------------------
    # THE GL DOOR — without a context (the CI runner's own case) the
    # timing is SKIPPED, DECLARED (never a silence): the deterministic
    # half above stands, the frame-time is the measuring machine's.
    # ------------------------------------------------------------------
    gl_window = viewport._gl_window
    if gl_window is None:
        print("[perf1-frametime] TIMING SKIPPED — no GL path (the "
              "viewport's own probe said no context: the CI runner "
              "is not the measuring machine; the register's m8 row's "
              "own law — the absolute numbers are indicative, the "
              "ratios are the substance). The frame-time is measured "
              "by the rite on the maintainer's machine — the Windows "
              "commands verbatim in the register's PERF 1 row.",
              flush=True)
        ok = g_failures == 0
        print(f"[perf1-frametime] {'PASS' if ok else 'FAIL'} "
              f"({g_failures} failures, deterministic half)",
              flush=True)
        return 0 if ok else 1

    win.show()
    for _ in range(50):
        app.processEvents()
    if not gl_window._initialized:
        print("[perf1-frametime] TIMING SKIPPED — no GL context (the "
              "window never initialized one; the deterministic half "
              "above stands; the frame-time is the rite's, on the "
              "measuring machine)", flush=True)
        ok = g_failures == 0
        print(f"[perf1-frametime] {'PASS' if ok else 'FAIL'} "
              f"({g_failures} failures, deterministic half)",
              flush=True)
        return 0 if ok else 1

    # The machine's GL half, declared beside the platform.
    try:
        from OpenGL.GL import glGetString, GL_RENDERER, GL_VERSION
        print(f"[perf1-frametime] GL renderer: "
              f"{glGetString(GL_RENDERER).decode()} | "
              f"{glGetString(GL_VERSION).decode()}", flush=True)
    except Exception as e:
        fail_hard(f"the GL strings could not be read ({e})")

    # The context current, once: the timed paints are direct calls
    # into the frame's own body (_paint_impl — the clear, the
    # projection, the view, the light, the overlays, the meshes; the
    # swap and the event-loop plumbing are outside, declared).
    ctx = gl_window.context()
    if ctx is None or not ctx.makeCurrent(gl_window):
        fail_hard("the GL context could not be made current")

    counters = _GlCounters(vw)
    counters.install()
    try:
        print(f"[perf1-frametime] methodology: warmup {WARMUP}, "
              f"iterations {ITERATIONS}, median (never the min, "
              "never the raw mean), p95 nearest-rank; gate: "
              f"within-run A/B per act, red beyond {GATE_RATIO}x, "
              "denominator floor 1us; the machine is declared above "
              "— the absolute numbers are indicative, the ratios are "
              "the substance", flush=True)

        # PASS A — the honest row.
        pass_a = run_pass("pass A", viewport, gl_window, win._scene,
                          fid, counters, ACT_NAMES, ITERATIONS, WARMUP)

        # PASS B — the same work again (the A/B's other half).
        pass_b = run_pass("pass B", viewport, gl_window, win._scene,
                          fid, counters, ACT_NAMES, ITERATIONS, WARMUP)

        # THE GATE.
        gate = frame_gate(pass_a, pass_b, ACT_NAMES)
        print(f"[perf1-frametime] the A/B gate: {render_gate(gate)}",
              flush=True)
        require(gate["pass"], "the A/B gate is red")

        # PASS C — THE M6 ROAD (the before): the m6-correttiva's
        # committed draw on the same corpus, same machine, same run.
        # Its times are PRINTED for the register's before/after form,
        # never gated (the gate stays within-run A/B on the honest
        # road — the m8 law).
        entry = viewport._draw_nodes[0]
        m6_payload_bytes = (int(entry.positions.nbytes)
                            + int(entry.normals.nbytes)
                            + int(entry.indices.nbytes))
        require(mirror_digest_of(viewport) == PIN_MIRROR_DIGEST_A,
                "the m6 road: the mirror digest is not the pin")
        m6_samples = []
        for it in range(WARMUP + ITERATIONS):
            viewport.set_camera_yaw_pitch(0.05 * (it + 1), 0.3)
            counters.reset_frame()
            t0 = time.perf_counter()
            m6_road_frame(viewport, entry)
            dt = time.perf_counter() - t0
            if it >= WARMUP:
                m6_samples.append(dt)
        m6_median, m6_p95 = stats_of(m6_samples)
        require(counters.uploads == 0,
                "the m6 road uploads via glBufferData never — the "
                "payload rides the client arrays, the driver's own "
                "implicit copy (the declared count is the bytes)")
        print(f"  [perf1-frametime] m6 road / the before: median "
              f"{ms_of(m6_median):.3f} ms, p95 {ms_of(m6_p95):.3f} ms "
              f"(client-array payload per frame: "
              f"{m6_payload_bytes} bytes, re-uploaded by the driver "
              f"at every draw)", flush=True)
        if PIN_M6_ROAD_PAYLOAD_BYTES is not None:
            require(m6_payload_bytes == PIN_M6_ROAD_PAYLOAD_BYTES,
                    "the m6 road's per-frame payload is not the pin")

        # THE PROFILE (the between-act ratios — the attribution,
        # never a threshold).
        total = sum(pass_a[a]["median"] for a in ACT_NAMES)
        for a in ACT_NAMES:
            share = pass_a[a]["median"] / total if total > 0 else 0.0
            print(f"[perf1-frametime] profile: {a} owns "
                  f"{share * 100.0:.1f}% of the three acts' median "
                  f"sum", flush=True)

        # COUNTERPROOF 1 — THE SLOWED ACT: the spin injected into the
        # stable act's timed region, calibrated >= 4x its honest
        # median; the other two acts green beside it (the controls).
        spin_budget = SABOTAGE_BUDGET_X * pass_a["stable frame"]["median"]
        sabot = run_pass("sabotage", viewport, gl_window, win._scene,
                         fid, counters, ACT_NAMES,
                         SABOTAGE_ITERATIONS, 0,
                         spin_budget_s=spin_budget)
        gate_sabot = frame_gate(pass_a, sabot, ACT_NAMES)
        print(f"[perf1-frametime] the counterproof's gate: "
              f"{render_gate(gate_sabot)}", flush=True)
        require(not gate_sabot["pass"],
                "the slowed-act counterproof did NOT go red — the "
                "gate cannot say no (a decorative number, not a gate)")
        require(gate_sabot["acts"]["stable frame"]["regressed"],
                "the counterproof must name the STABLE act (the "
                "slowed one)")
        require(not gate_sabot["acts"]["first sight"]["regressed"]
                and not gate_sabot["acts"]["edit"]["regressed"],
                "the counterproof's controls (first sight, edit) "
                "must stay green")

        # COUNTERPROOF 2 — THE DIGEST DRIFT: a pass whose mirror
        # digest differs by ONE character; the gate refuses to judge.
        drifted = {a: dict(pass_b[a]) for a in ACT_NAMES}
        d = drifted["stable frame"]["record"]
        drifted["stable frame"]["record"] = (
            "0" if d[0] != "0" else "1") + d[1:]
        gate_drift = frame_gate(pass_a, drifted, ACT_NAMES)
        print(f"[perf1-frametime] the digest-drift gate: "
              f"{render_gate(gate_drift)}", flush=True)
        require(not gate_drift["pass"]
                and not gate_drift["digest_identity"],
                "the digest drift must be REFUSED (the times are not "
                "believed — the identity is the first predicate)")

        # THE VERDICT — the deterministic half only: the pins, the
        # budgets, the booleans, the counterproofs. The times stay in
        # the report, never in the pin.
        verdict_lines = [
            corpus_line,
            f"warmup {WARMUP} iterations {ITERATIONS}",
            f"gate ratio {GATE_RATIO} floor {GATE_FLOOR_S}",
            f"acts {len(ACT_NAMES)}: " + ", ".join(ACT_NAMES),
            f"stable uploads {pass_a['stable frame']['uploads']} "
            f"draws {pass_a['stable frame']['draws']}",
            f"first-sight uploads {pass_a['first sight']['uploads']}",
            f"edit uploads {pass_a['edit']['uploads']}",
            f"m6-road payload bytes {m6_payload_bytes}",
            f"gate green {gate['pass']}",
            f"sabotage red {not gate_sabot['pass']} "
            f"(stable named "
            f"{gate_sabot['acts']['stable frame']['regressed']})",
            f"drift refused {not gate_drift['pass']}",
        ]
        verdict_text = "\n".join(verdict_lines)
        verdict_fnv = fnv1a64(verdict_text)
        print("[perf1-frametime] the verdict (deterministic half "
              "only — the times above are the machine's own, never "
              f"pinned):\n{verdict_text}", flush=True)
        print(f"[perf1-frametime] verdict fnv: {hex16(verdict_fnv)}",
              flush=True)

        if pinning:
            print("[perf1-frametime] --pin: write these into the "
                  "source's constants (the first clean run's own "
                  f"values): PIN_BLIND_CHAIN_FNV = {hex16(blind_fnv)}; "
                  f"PIN_FIL_CHAIN_FNV = {hex16(fil_fnv)}; "
                  f"PIN_M6_ROAD_PAYLOAD_BYTES = {m6_payload_bytes}; "
                  f"PIN_VERDICT_FNV = {hex16(verdict_fnv)}", flush=True)
        elif PIN_VERDICT_FNV is not None:
            require(verdict_fnv == PIN_VERDICT_FNV,
                    f"the verdict's FNV drifted ({hex16(verdict_fnv)} "
                    f"!= {hex16(PIN_VERDICT_FNV)})")
    finally:
        counters.restore()

    ok = g_failures == 0
    print(f"[perf1-frametime] {'PASS' if ok else 'FAIL'} "
          f"({g_failures} failures)", flush=True)
    # The house's own model for a real exposed GL window (the startup
    # test's precedent): exit BEFORE the Qt/GL teardown — the
    # shutdown segfault is the declared pre-existing noise, never a
    # bench verdict.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    raise SystemExit(main())
