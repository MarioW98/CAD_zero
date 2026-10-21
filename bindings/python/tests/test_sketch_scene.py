# bindings/python/tests/test_sketch_scene.py
#
# ADR-0030 D2 unit 2 — the sketch/extrude bindings surface, tested
# WITHOUT PySide (the F-1 pattern, from code): the third node kind,
# the entity editing primitives, the visible definition state, the
# extrude's field (sampled against the pinned analytic values — the
# bit-identity oracle lives in the C++ suite, where both construction
# paths run the same code), the refusal/pending/dangling states and
# the id-stable undo/redo.
import pytest

import CAD_0

scene = CAD_0.scene
sketch_segment = scene.sketch_segment
sketch_circle = scene.sketch_circle
SketchChain = scene.SketchChain
V3 = CAD_0.math.Vec3f


def val(body, x, y, z):
    """body.value at a point (Vec3f is the bound point type)."""
    return body.value(V3(x, y, z))


def ring_chains():
    """The fixture's profile: outer rectangle + one hole circle."""
    rect = SketchChain([
        sketch_segment((-1.0, -0.5), (1.0, -0.5), True),
        sketch_segment((1.0, -0.5), (1.0, 0.5), True),
        sketch_segment((1.0, 0.5), (-1.0, 0.5), True),
        sketch_segment((-1.0, 0.5), (-1.0, -0.5), True),
    ])
    hole = SketchChain([sketch_circle((0.0, 0.0), 0.2, True)])
    return [rect, hole]


def three_side_chains():
    """An OPEN chain: a legal, visibly-flagged editing state."""
    return [SketchChain([
        sketch_segment((-1.0, -0.5), (1.0, -0.5), True),
        sketch_segment((1.0, -0.5), (1.0, 0.5), True),
        sketch_segment((1.0, 0.5), (-1.0, 0.5), True),
    ])]


def test_sketch_node_creation_and_definition_state():
    s = scene.Scene()
    sid = s.add_sketch_node("S")
    n = s.get_node(sid)
    assert n.kind == "sketch"
    assert n.operation is None
    assert not n.operand_missing
    assert not n.operation_pending
    assert not s.sketch_closed(sid)  # empty: nothing closed

    # Entity-by-entity authoring: chain 0 OPENS on the first append
    # (chain == count); three sides leave it open (legal, flagged).
    s.sketch_add_entity(sid, 0, sketch_segment((-1.0, -0.5), (1.0, -0.5), True))
    s.sketch_add_entity(sid, 0, sketch_segment((1.0, -0.5), (1.0, 0.5), True))
    assert len(s.sketch_chains(sid)) == 1
    assert not s.sketch_closed(sid)
    s.sketch_add_entity(sid, 0, sketch_segment((1.0, 0.5), (-1.0, 0.5), True))
    s.sketch_add_entity(sid, 0, sketch_segment((-1.0, 0.5), (-1.0, -0.5), True))
    assert s.sketch_closed(sid)

    # The readable payload: the panel/canvas read surface.
    chains = s.sketch_chains(sid)
    assert len(chains) == 1 and len(chains[0]) == 4
    assert chains[0][0]["kind"] == "segment"
    assert chains[0][0]["a"] == (-1.0, -0.5)
    assert chains[0][0]["interior_left"] is True

    # EDIT: a vertex moved past the neighbour OPENS the chain.
    s.sketch_set_entity(sid, 0, 0, sketch_segment((-1.0, -0.5), (1.0, -0.6), True))
    assert not s.sketch_closed(sid)
    s.sketch_set_entity(sid, 0, 0, sketch_segment((-1.0, -0.5), (1.0, -0.5), True))
    assert s.sketch_closed(sid)

    # DELETE opens it again; a hole chain opens as chain 1 (a single
    # circle closes itself).
    assert s.sketch_remove_entity(sid, 0, 3)
    assert not s.sketch_closed(sid)
    s.sketch_add_entity(sid, 1, sketch_circle((0.0, 0.0), 0.2, True))
    chains = s.sketch_chains(sid)
    assert len(chains) == 2
    assert chains[1][0]["kind"] == "circle"
    assert chains[1][0]["hole"] is True
    # ...and closing chain 0 closes the sketch.
    s.sketch_add_entity(sid, 0, sketch_segment((-1.0, 0.5), (-1.0, -0.5), True))
    assert s.sketch_closed(sid)

    # Honest dispatch: entity editing on a non-sketch node is loud.
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    with pytest.raises(RuntimeError, match="is not a sketch"):
        s.sketch_add_entity(box, 0, sketch_circle((0.0, 0.0), 1.0, True))
    with pytest.raises(RuntimeError, match="no entity at chain"):
        s.sketch_set_entity(sid, 7, 0, sketch_circle((0.0, 0.0), 1.0, True))


def test_extrude_prism_field_and_bounds():
    s = scene.Scene()
    sid = s.add_sketch_node("S", ring_chains())
    assert s.sketch_closed(sid)

    ex = s.add_extrude_node("Extrude", sid, distance=1.5, direction=1)
    n = s.get_node(ex)
    assert n.kind == "sdf"  # the prism IS a body
    op = n.operation
    assert op["type"] == "extrude"
    assert op["sketch"] == sid
    assert op["distance"] == 1.5
    assert op["direction"] == 1
    assert not n.operand_missing
    assert not n.operation_pending
    assert s.operation_error(ex) == ""

    # The prism's bounds: the profile's box x the slab's z-range.
    b = n.body.bounds()
    assert (b.min.x, b.min.y, b.min.z) == (-1.0, -0.5, 0.0)
    assert (b.max.x, b.max.y, b.max.z) == (1.0, 0.5, 1.5)

    # Pinned analytic samples (the max-composition, by hand):
    #  - inside the ring's material, mid-slab: the right wall wins.
    assert val(n.body, 0.7, 0.0, 0.75) == pytest.approx(-0.3, abs=1e-6)
    #  - the hole's centre: no material (the hole's field wins).
    assert val(n.body, 0.0, 0.0, 0.75) == pytest.approx(0.2, abs=1e-6)
    #  - below the bottom cap: the slab wins.
    assert val(n.body, 0.7, 0.0, -0.1) == pytest.approx(0.1, abs=1e-6)
    #  - past the prism laterally: the profile wins.
    assert val(n.body, 1.4, 0.0, 0.75) == pytest.approx(0.4, abs=1e-6)

    # The other direction: the slab spans z in [-d, 0].
    ex2 = s.add_extrude_node("Extrude-", sid, 1.5, -1)
    b2 = s.get_node(ex2).body.bounds()
    assert (b2.min.z, b2.max.z) == (-1.5, 0.0)
    assert val(s.get_node(ex2).body, 0.7, 0.0, -0.75) == pytest.approx(-0.3, abs=1e-6)

    # The rebuild pass is idempotent through the bindings too.
    res = s.rebuild_operations()
    assert res["rebuilt"] == 2
    assert res["dangling"] == []
    assert res["pending"] == []
    assert val(s.get_node(ex).body, 0.7, 0.0, 0.75) == pytest.approx(-0.3, abs=1e-6)


def test_open_sketch_refuses_nothing_written():
    s = scene.Scene()
    sid = s.add_sketch_node("S", three_side_chains())
    assert not s.sketch_closed(sid)
    count, nxt = s.node_count(), s.next_id()

    # Closure is the extrude's precondition: the certificate refuses
    # the open chain (the gap named), nothing is written.
    with pytest.raises(ValueError, match="must share their joint"):
        s.add_extrude_node("Extrude", sid)
    assert s.node_count() == count
    assert s.next_id() == nxt

    # A missing operand is a caller bug, loudly.
    with pytest.raises(RuntimeError, match="does not exist"):
        s.add_extrude_node("E", 4242)
    # A degenerate distance refuses via the kernel's own check.
    sid2 = s.add_sketch_node("Closed", ring_chains())
    with pytest.raises(ValueError, match="distance"):
        s.add_extrude_node("E2", sid2, 0.0)


def test_edit_opens_chain_pending_state_body_kept():
    s = scene.Scene()
    sid = s.add_sketch_node("S", ring_chains())
    ex = s.add_extrude_node("Extrude", sid, 1.5, 1)
    kept = val(s.get_node(ex).body, 0.7, 0.0, 0.75)

    # The edit that OPENS the chain.
    s.sketch_set_entity(sid, 0, 1, sketch_segment((1.0, -0.5), (1.0, 0.9), True))
    assert not s.sketch_closed(sid)
    res = s.rebuild_operations()
    assert res["pending"] == [ex]
    assert res["rebuilt"] == 0
    assert res["dangling"] == []

    n = s.get_node(ex)
    assert n.operation_pending
    assert not n.operand_missing  # the sketch EXISTS
    # The last good body is KEPT — display continues.
    assert val(n.body, 0.7, 0.0, 0.75) == kept
    err = s.operation_error(ex)
    assert "does not certify" in err
    assert "must share their joint" in err

    # Recovery is an edit away: close the chain, the pass re-materializes.
    s.sketch_set_entity(sid, 0, 1, sketch_segment((1.0, -0.5), (1.0, 0.5), True))
    res = s.rebuild_operations()
    assert res["rebuilt"] == 1
    assert res["pending"] == []
    assert not s.get_node(ex).operation_pending
    assert s.operation_error(ex) == ""
    assert val(s.get_node(ex).body, 0.7, 0.0, 0.75) == pytest.approx(-0.3, abs=1e-6)


def test_delete_sketch_dangling_then_recovered_same_id():
    s = scene.Scene()
    sid = s.add_sketch_node("S", ring_chains())
    ex = s.add_extrude_node("Extrude", sid, 1.5, 1)

    assert s.remove_node(sid)
    res = s.rebuild_operations()
    assert res["dangling"] == [ex]
    n = s.get_node(ex)
    assert n.operand_missing
    assert not n.operation_pending  # the sketch is GONE, not broken
    assert val(n.body, 0.7, 0.0, 0.75) == pytest.approx(-0.3, abs=1e-6)

    err = s.operation_error(ex)
    assert f"#{sid} (sketch)" in err
    assert "undo" in err

    # The undo of the delete: the sketch returns WITH ITS ORIGINAL ID.
    s.restore_sketch_node(sid, "S", ring_chains())
    res = s.rebuild_operations()
    assert res["rebuilt"] == 1
    assert res["dangling"] == []
    assert not s.get_node(ex).operand_missing
    assert s.operation_error(ex) == ""


def test_entity_edit_propagates_to_dependent_subtract():
    # The ADR's end-to-end genealogy through the bindings: sketch ->
    # circle -> extrude -> subtract from a box (the parametric hole).
    s = scene.Scene()
    sid = s.add_sketch_node("S", [SketchChain([sketch_circle((0.0, 0.0), 0.35, False)])])
    ex = s.add_extrude_node("Extrude", sid, 3.0, 1)
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.2, 1.2, 1.2)))
    hole = s.add_operation_node("Hole", box, ex, 0.3, 0.0, -1.5)
    res = s.rebuild_operations()
    assert res["rebuilt"] == 2

    # On the hole's axis, at the box's centre: inside the removed
    # prism -> the subtract field is the (negated) tool depth.
    assert val(s.get_node(hole).body, 0.3, 0.0, 0.0) == pytest.approx(0.35, abs=1e-6)

    # THE EDIT: the sketch circle's radius (an entity parameter). The
    # extrude regenerates and the dependent subtract follows.
    s.sketch_set_entity(sid, 0, 0, sketch_circle((0.0, 0.0), 0.5, False))
    res = s.rebuild_operations()
    assert res["rebuilt"] == 2
    assert res["pending"] == [] and res["dangling"] == []
    assert val(s.get_node(hole).body, 0.3, 0.0, 0.0) == pytest.approx(0.5, abs=1e-6)


def test_entity_class_checks_and_per_chain_closure():
    """The D5.5/D5.3 surfaces unit 4's canvas calls.

    sketch_entity_error: the kernel's own per-entity construction
    checks (one opinion — the certificate runs the same predicates;
    the UI keeps no second copy of the rules). sketch_chain_closed:
    the per-chain closure the canvas's amber aspect derives from,
    the same chain_is_closed behind sketch_closed and the extrude's
    closure precondition.
    """
    # In class: empty string, no exception.
    assert scene.sketch_entity_error(
        sketch_segment((-1.0, -0.5), (1.0, 0.5), True)) == ""
    assert scene.sketch_entity_error(
        scene.sketch_circle((0.0, 0.0), 0.25, False)) == ""
    assert scene.sketch_entity_error(
        scene.sketch_arc((0.0, 0.0), 1.0, False, 0.0, 1.5)) == ""

    # Out of class: the kernel's own message, the refused value named.
    assert "p0 != p1" in scene.sketch_entity_error(
        sketch_segment((0.5, 0.5), (0.5, 0.5), True))
    assert "radius" in scene.sketch_entity_error(
        scene.sketch_circle((0.0, 0.0), 0.0, False))
    sweep_err = scene.sketch_entity_error(
        scene.sketch_arc((0.0, 0.0), 1.0, False, 0.0, 0.0))
    assert "sweep" in sweep_err and "(0, 2" in sweep_err

    # Per-chain closure: the open chain flags, the closed one does
    # not; the sketch-wide state is the conjunction (sketch_closed).
    s = scene.Scene()
    sid = s.add_sketch_node("S", three_side_chains())
    s.sketch_add_entity(sid, 1, sketch_circle((0.0, 0.0), 0.2, True))
    assert not s.sketch_chain_closed(sid, 0)   # the three-side chain
    assert s.sketch_chain_closed(sid, 1)       # the hole circle
    assert not s.sketch_closed(sid)
    s.sketch_add_entity(sid, 0,
                        sketch_segment((-1.0, 0.5), (-1.0, -0.5), True))
    assert s.sketch_chain_closed(sid, 0)
    assert s.sketch_closed(sid)

    # Honest dispatch: a non-sketch node and a past-the-end chain
    # refuse loudly (never a silent guess).
    box = s.add_node("Box", CAD_0.sdf.box(V3(1.0, 1.0, 1.0)))
    with pytest.raises(RuntimeError, match="is not a sketch"):
        s.sketch_chain_closed(box, 0)
    with pytest.raises(RuntimeError, match="past the end"):
        s.sketch_chain_closed(sid, 7)


def test_extrude_command_undo_redo_same_ids():
    stack = CAD_0.commands.CommandStack(64)
    s = scene.Scene()
    sid = s.add_sketch_node("S", ring_chains())

    state = {"id": 0}

    def do_extrude():
        if state["id"] == 0:
            state["id"] = s.add_extrude_node("Extrude", sid, 1.5, 1)
        else:
            s.restore_extrude_node(state["id"], "Extrude", sid, 1.5, 1)
        s.rebuild_operations()

    def undo_extrude():
        s.remove_node(state["id"])

    stack.push(do_extrude, undo_extrude, "Extrude")
    assert stack.undo_count() == 1  # ONE entry per execution
    ex = state["id"]
    value = val(s.get_node(ex).body, 0.7, 0.0, 0.75)
    assert s.get_node(sid).visible  # a sketch has no viewport body to hide

    stack.undo()
    assert s.get_node(ex) is None
    stack.redo()
    assert s.get_node(ex).id == ex  # SAME id — no rebind
    assert s.get_node(ex).operation["sketch"] == sid
    assert val(s.get_node(ex).body, 0.7, 0.0, 0.75) == value
    assert s.next_id() == ex + 1
