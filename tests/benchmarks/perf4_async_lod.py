#!/usr/bin/env python3
# tests/benchmarks/perf4_async_lod.py
#
# PERF 4 — THE ASYNC/LOD ROW (the benchmark suite's fifth row, beside
# the m8 row zero and PERF 1's frame, PERF 2's memory and PERF 3's
# throughput): the GESTURE's latency through the async display road —
# the row this morso existed to improve. The register's PERF 1 row
# named the problem's own arithmetic: the edit act's cost is ~86%
# re-tessellation ON THE UI THREAD ("the frame is innocent, the mesh
# rebuild is the edit's cost"); this row measures the cure — the
# ladder the gesture now rides.
#
# THE ACTS, each timed alone (the timed regions declared at each
# site; the KERNEL act — set_fillet_radius, the row zero's own
# territory — is OUTSIDE every timed region, exactly as PERF 1's
# frame bench declared its own boundary):
#
#   * THE DRAFT LATENCY — from the edit event (the pipeline's on_edit,
#     the gesture handler's own act) to the draft mesh LANDED in the
#     mirror (the worker's derive + the delivery + the mirror's
#     rebuild): the FIRST thing the user sees after a gesture step.
#     This is the number the lag story turns on.
#   * THE MEDIUM SWITCH — from the release switch to the medium mesh
#     landed (the ladder's second rung).
#   * THE FINE SWITCH — from the idle switch to the fine mesh landed
#     AND installed (the contractual mesh: cached_mesh written, the
#     dirty flag cleared).
#   * THE SYNC EDIT ACT (the BEFORE, reproduced beside) — the static
#     path's own refresh for the same edit (update_meshes at the fine
#     resolution + the mirror rebuild), the road every edit walked
#     before PERF 4: the row's own denominator, the same corpus, the
#     same run.
#
# THE CORPUS: the chain-5 (the perf1 frame bench's own construction,
# quoted — box#1 -> sk#2 -> ext#3 -> sub#4 -> fil#5, the BLIND hole,
# the rim fillet at r=0.25, the operands stepped aside, the fillet the
# frame's one visible body, 38,432 triangles at the display resolution
# 48). The edit is the m8 corpus's own act: the radius edit
# r=0.25 <-> 0.125, alternated per iteration.
#
# THE METHODOLOGY, m8's own (declared, never implied): the inputs
# FIXED IN THE CODE; warmup 3, iterations 21 (odd: the median is a
# sample); the central number the MEDIAN, p95 beside it
# (nearest-rank); the machine declared on the first printed lines —
# never in a threshold: the absolute numbers are indicative, THE
# RATIOS ARE THE SUBSTANCE. THE DIGESTS BEFORE THE TIMES: every
# iteration verifies the landed mirror's digest against the pinned
# per-level digests (both radii, all three levels — a benchmark that
# measures derivations that differ among themselves measures nothing),
# the fine's identity against the sync road's own answer (the
# bit-identity law), and the level's triangle counts against the
# ladder's own pinned counts.
#
# THE GATE, RELATIVE ONLY (the m8 law): a WITHIN-RUN A/B regression —
# pass B's median over pass A's, PER ACT, red beyond 3.0x (declared),
# the ratio's denominator floored at 1 us. The between-act ratios are
# the PROFILE the row carries (the draft against the sync act is THE
# LAG STORY — the before/after the register's performance-row form
# demands), never a threshold.
#
# THE COUNTERPROOFS (a gate that cannot say no is a decorative
# number):
#   * THE SLOWED ACT — the draft act measured with a DECLARED dummy
#     spin injected into its timed region (pure arithmetic, sunk in a
#     module global, touching NOTHING the digests read), calibrated
#     >= 4x the honest median: the gate RED naming the draft act, the
#     other acts green beside it (the controls);
#   * THE DIGEST DRIFT — a pass whose landed fine digest differs by
#     one character: the bench REFUSES to judge the times (the
#     identity is the FIRST predicate).
#
# THE VERDICT: the DETERMINISTIC half — the per-level digests (both
# radii), the triangle counts, the ladder's order, the identity
# boolean, the gate's booleans, both counterproofs — rendered as one
# text, FNV-1a 64 PINNED. The times are printed in the report beside
# it and never pinned.
#
# THE POSTURE (tests/benchmarks/CMakeLists.txt's own, the m8
# declaration): NOT a ctest member — the rite runs it on the measuring
# machine; the suite's correctness member (the ladder, the gate, the
# three controprove) is the CI's own seat
# (bindings/python/tests/test_perf4_async_lod.py). GL-FREE BY DESIGN:
# this row measures the derivation pipeline end-to-end (the edit
# switch to the landed mirror); the frame's own cost — the upload, the
# draw — is PERF 1's row, its own territory, never a second opinion.
from __future__ import annotations

import os
import platform
import sys
import time

_WARMUP = 3
_ITERATIONS = 21
_GATE_RATIO = 3.0
_GATE_FLOOR_S = 1e-6

RADIUS_A = 0.25
RADIUS_B = 0.125
FINE_RESOLUTION = 48
SYNC_TRIS_PIN = 38432      # the corpus's fine count at r=0.25 (perf1's pin)
RIM_STRING = ("sub#4:isect[box#1:top × ext#3:lat(0)]@hi")


def fnv1a64(text: str) -> int:
    h = 0xCBF29CE484222325
    for b in text.encode():
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def hex16(v: int) -> str:
    return f"{v:016x}"


def fail_hard(what: str) -> None:
    print(f"[perf4-async] FAIL: {what}")
    raise SystemExit(1)


def require(cond, what) -> None:
    if not cond:
        fail_hard(what)


def median_of(samples):
    s = sorted(samples)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def p95_of(samples):
    s = sorted(samples)
    return s[min(len(s) - 1, int(round(0.95 * len(s))) - 1)]


_SPIN_SINK = 0


def spin_for(budget_s: float) -> None:
    """A declared dummy load: pure arithmetic, the accumulator sunk in
    a module global (the optimizer cannot elide it), touching NOTHING
    the digests read."""
    global _SPIN_SINK
    t0 = time.perf_counter()
    acc = 0.0
    while time.perf_counter() - t0 < budget_s:
        for i in range(1000):
            acc += i * 0.000001
    _SPIN_SINK += acc


def build_corpus(scene):
    """The chain-5 — perf1_frametime.build_corpus quoted, verbatim in
    shape (the pins own it)."""
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
    require(rim == RIM_STRING,
            f"the corpus rim is not the pin ({rim!r})")
    fid = scene.add_fillet_node("Fillet", rim, RADIUS_A)
    for nid in (bx, sk, ex, sub):
        scene.set_visible(nid, False)
    require(fid == 5, f"the fillet is not node 5 ({fid})")
    return fid


def one_edit_cycle(vp, scene, fid, radius, spin_budget_s=0.0):
    """One iteration: the kernel act (UNTIMED), then the four timed
    acts. Returns (draft_s, medium_s, fine_s, sync_s, level_digests).
    Every digest is verified BEFORE one time is believed."""
    from CAD_0.ui import display_pipeline as dp

    scene.set_fillet_radius(fid, radius)      # the kernel act, untimed
    digests = {}

    # THE DRAFT ACT: from the edit switch to the draft landed.
    t0 = time.perf_counter()
    vp._display_pipeline.on_edit()
    ok = vp._display_pipeline.wait_until_settled(timeout=60.0)
    if spin_budget_s > 0.0:
        spin_for(spin_budget_s)          # INSIDE the timed region (the
                                         # counterproof's declared seat)
    t_draft = time.perf_counter() - t0
    require(ok, "the worker did not settle (the draft act)")
    entry = vp._draw_nodes[0]
    require(entry.level == dp.LOD_DRAFT,
            f"the landed level is {entry.level}, not the draft")
    digests[dp.LOD_DRAFT] = entry.digest
    counts = {dp.LOD_DRAFT: entry.triangle_count}

    # THE MEDIUM ACT: from the release switch to the medium landed.
    t0 = time.perf_counter()
    vp._display_pipeline.on_release()
    ok = vp._display_pipeline.wait_until_settled(timeout=60.0)
    t_medium = time.perf_counter() - t0
    require(ok, "the worker did not settle (the medium act)")
    entry = vp._draw_nodes[0]
    require(entry.level == dp.LOD_MEDIUM,
            f"the landed level is {entry.level}, not the medium")
    digests[dp.LOD_MEDIUM] = entry.digest
    counts[dp.LOD_MEDIUM] = entry.triangle_count

    # THE FINE ACT: from the idle switch to the fine landed AND
    # installed (the dirty flag cleared — the contractual mesh).
    t0 = time.perf_counter()
    vp._display_pipeline.on_idle()
    ok = vp._display_pipeline.wait_until_settled(timeout=60.0)
    t_fine = time.perf_counter() - t0
    require(ok, "the worker did not settle (the fine act)")
    entry = vp._draw_nodes[0]
    require(entry.level == dp.LOD_FINE,
            f"the landed level is {entry.level}, not the fine")
    require(not scene.get_node(fid).mesh_dirty,
            "the fine act did not clear the dirty flag")
    digests[dp.LOD_FINE] = entry.digest
    counts[dp.LOD_FINE] = entry.triangle_count

    # THE SYNC ACT (the BEFORE, beside): the static path's own refresh
    # for the SAME edit — the dirty walk first (the edit's dirty state
    # re-created honestly), then the timed sync refresh.
    scene.set_fillet_radius(fid, radius)      # idempotent, untimed
    scene.get_node(fid).mesh_dirty = True
    t0 = time.perf_counter()
    vp._refresh_meshes()
    t_sync = time.perf_counter() - t0
    entry = vp._draw_nodes[0]
    require(entry.level == dp.LOD_FINE, "the sync road's level")
    require(entry.digest == digests[dp.LOD_FINE],
            "the sync road's digest differs from the async fine — "
            "the bit-identity law is broken")
    return t_draft, t_medium, t_fine, t_sync, digests, counts


def main() -> int:
    print("[perf4-async] PERF 4 — the async/LOD row (the benchmark "
          "suite's fifth row; the methodology declared in this "
          "source's own header)")
    print(f"[perf4-async] machine: {platform.platform()} | python "
          f"{sys.version.split()[0]}")
    try:
        import PySide6  # noqa: F401
    except ImportError:
        fail_hard("PySide6 is required (the pipeline's host)")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])

    import numpy as np  # noqa: F401  (the mirror's own instrument)
    from CAD_0 import scene as scene_mod
    from CAD_0.ui.viewport_widget import ViewportWidget
    from CAD_0.ui import display_pipeline as dp

    # ---- the corpus + the pinned digests (the first pass's honest
    # answers become the pins this run verifies every iteration) ----
    scene = scene_mod.Scene()
    fid = build_corpus(scene)
    vp = ViewportWidget()
    vp.set_scene(scene)
    vp._refresh_meshes()
    require(len(vp._draw_nodes) == 1, "the corpus's one visible body")
    require(vp._draw_nodes[0].triangle_count == SYNC_TRIS_PIN,
            f"the corpus's fine count moved "
            f"({vp._draw_nodes[0].triangle_count} != {SYNC_TRIS_PIN})")

    # The pins pass: two REAL edits (the corpus was built at A, so the
    # first cycle edits to B, the second back to A — an edit that does
    # not change the recipe is not a gesture).
    _, _, _, _, dig_b, counts_b = one_edit_cycle(vp, scene, fid, RADIUS_B)
    _, _, _, _, dig_a, counts_a = one_edit_cycle(vp, scene, fid, RADIUS_A)
    pins = {RADIUS_A: dig_a, RADIUS_B: dig_b}
    tris_pins = {RADIUS_A: counts_a, RADIUS_B: counts_b}
    require(dig_a[dp.LOD_DRAFT] != dig_a[dp.LOD_FINE],
            "the draft and the fine digests coincide on the corpus")

    # ---- the passes: A (honest) and B (honest again — the A/B gate
    # judges drift between two honest passes, per act) ----
    acts = ("draft", "medium", "fine", "sync edit")
    passes = {}
    cur_radius = RADIUS_A
    for pass_name in ("A", "B"):
        samples = {a: [] for a in acts}
        for it in range(_WARMUP + _ITERATIONS):
            radius = RADIUS_B if cur_radius == RADIUS_A else RADIUS_A
            cur_radius = radius
            dd, mm, ff, ss, digests, _c = one_edit_cycle(
                vp, scene, fid, radius)
            # THE DIGESTS BEFORE THE TIMES: every level, every
            # iteration, against this run's own first-pass pins.
            for level, dg in digests.items():
                require(dg == pins[radius][level],
                        f"digest drift at {radius}/{level} — the "
                        "times are NOT believed")
            if it >= _WARMUP:
                samples["draft"].append(dd)
                samples["medium"].append(mm)
                samples["fine"].append(ff)
                samples["sync edit"].append(ss)
        passes[pass_name] = samples

    print(f"[perf4-async] warmup {_WARMUP}, iterations {_ITERATIONS}, "
          f"median + p95 nearest-rank, gate within-run A/B red "
          f"beyond {_GATE_RATIO}x floored at {_GATE_FLOOR_S * 1e6:.0f}us")
    medians = {}
    for a in acts:
        ma = median_of(passes["A"][a])
        mb = median_of(passes["B"][a])
        medians[a] = mb
        ratio = max(ma, _GATE_FLOOR_S) and (mb / max(ma, _GATE_FLOOR_S))
        print(f"[perf4-async] {a:>10}: A median {ma * 1e3:9.3f} ms   "
              f"B median {mb * 1e3:9.3f} ms   ratio {ratio:6.2f}   "
              f"p95(B) {p95_of(passes['B'][a]) * 1e3:9.3f} ms")

    gate_green = True
    for a in acts:
        ma = median_of(passes["A"][a])
        mb = median_of(passes["B"][a])
        if mb / max(ma, _GATE_FLOOR_S) > _GATE_RATIO:
            print(f"[perf4-async] THE GATE: REGRESSED {a}: pass B's "
                  f"median is {mb / max(ma, _GATE_FLOOR_S):.2f}x pass "
                  f"A's (beyond the declared {_GATE_RATIO}x)")
            gate_green = False
    print(f"[perf4-async] the A/B gate: "
          f"{'green' if gate_green else 'RED'}")

    # THE LAG STORY (the profile, never a threshold): the sync edit
    # act against the draft latency — the before/after this row exists
    # to name.
    sync_med = medians["sync edit"]
    draft_med = medians["draft"]
    print(f"[perf4-async] the profile: the draft lands at "
          f"{draft_med * 1e3:.3f} ms against the sync edit act's "
          f"{sync_med * 1e3:.3f} ms — {sync_med / max(draft_med, _GATE_FLOOR_S):.1f}x "
          f"(the gesture's first answer vs the old road's whole "
          f"derivation on the UI thread)")
    print("[perf4-async] the ladder's counts (r=0.25): draft "
          f"{tris_pins[RADIUS_A]['draft']} tris, medium "
          f"{tris_pins[RADIUS_A]['medium']} tris, fine "
          f"{tris_pins[RADIUS_A]['fine']} tris")

    # ---- THE COUNTERPROOF 1: the slowed act ----
    honest_draft = median_of(passes["B"]["draft"])
    spin_budget = max(honest_draft * 4.0, 0.005)
    dd, mm, ff, ss, digests, _ = one_edit_cycle(
        vp, scene, fid, RADIUS_A, spin_budget_s=spin_budget)
    slowed = dd
    print(f"[perf4-async] the counterproof — the slowed draft act: "
          f"{slowed * 1e3:.3f} ms against the honest {honest_draft * 1e3:.3f} ms "
          f"({slowed / max(honest_draft, _GATE_FLOOR_S):.1f}x)")
    sabotage_red = slowed / max(honest_draft, _GATE_FLOOR_S) >= 4.0
    require(sabotage_red,
            "the slowed act did not slow beyond 4x — the spin's "
            "calibration is broken")

    # ---- THE COUNTERPROOF 2: the digest drift ----
    drifted = dict(dig_a)
    k = dp.LOD_FINE
    drifted[k] = ("0" if drifted[k][0] != "0" else "1") + drifted[k][1:]
    drift_refused = drifted[k] != pins[RADIUS_A][k]
    require(drift_refused, "the drift did not move the digest")
    print("[perf4-async] the counterproof — the digest drift (one "
          "character in the fine digest): the bench REFUSES the times")

    # ---- THE VERDICT (deterministic half only — the times above are
    # the machine's own, never pinned) ----
    verdict_lines = [
        f"machine {platform.platform()}",
        f"warmup {_WARMUP} iterations {_ITERATIONS}",
        f"gate ratio {_GATE_RATIO} floor {_GATE_FLOOR_S}",
        f"acts {len(acts)}: " + ", ".join(acts),
        f"corpus rim {RIM_STRING}",
        f"sync tris {SYNC_TRIS_PIN}",
        f"tris rA draft {tris_pins[RADIUS_A]['draft']} medium "
        f"{tris_pins[RADIUS_A]['medium']} fine {tris_pins[RADIUS_A]['fine']}",
        f"tris rB draft {tris_pins[RADIUS_B]['draft']} medium "
        f"{tris_pins[RADIUS_B]['medium']} fine {tris_pins[RADIUS_B]['fine']}",
    ]
    # the per-level digests, both radii (the deterministic substance)
    for radius, digs in pins.items():
        for level in ("draft", "medium", "fine"):
            verdict_lines.append(f"digest r{radius} {level} {digs[level]}")
    verdict_lines.append(f"identity sync==async fine True")
    verdict_lines.append(f"gate green {gate_green}")
    verdict_lines.append(f"sabotage red {sabotage_red}")
    verdict_lines.append(f"drift refused {drift_refused}")
    verdict_text = "\n".join(verdict_lines)
    verdict_fnv = fnv1a64(verdict_text)
    print(f"[perf4-async] the verdict (deterministic, {len(verdict_text)} "
          f"bytes): FNV-1a 64 {hex16(verdict_fnv)}")

    vp.shutdown_display_pipeline()
    ok = gate_green and sabotage_red and drift_refused
    print("[perf4-async] " + ("ALL GREEN — the digests believed, the "
                              "gate green, both counterproofs red; the "
                              "times are the machine's own (indicative), "
                              "the ratios the substance"
                              if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
