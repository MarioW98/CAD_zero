// tests/unit/scene/test_sketch_scene.cpp
//
// The ADR-0030 D2 document-layer contract (MVP unit 2), tested WITHOUT
// PySide — the Phase F scene/operations pattern on the REAL C++ core:
// the third NodeKind, the extrude recipe, the role-list
// generalization of the rebuild pass, the visible definition/pending/
// dangling states, and the id-stable command re-creation with the
// undo-interleaving case (the F-1 fix carrying over to the extrude).
//
// Determinism oracle: the subtract's tests compare serialize_node JSON,
// but the v0.3 serializer REFUSES an ExtrudeSDF body by design (the
// sketch/extrude format is v0.4, unit 3) — so this file's oracle is
// the FIELD ITSELF: two bodies are bit-identical iff they sample
// bit-identically (value and gradient, float ==) on a deterministic
// stratified grid AND their bounds agree component-wise. Both paths
// (scene-materialized and direct kernel construction) run the SAME
// constructors on the SAME data, so ADR-0014 guarantees the same bits
// — any drift in what the scene feeds the kernel (a wrong chain, a
// mutated parameter, a stale copy) moves the field somewhere on the
// grid.
#include <doctest/doctest.h>

#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/scene/builders.hpp"
#include "CAD_0/viewport/command_stack.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace CAD_0;
using scene::NodeKind;
using scene::OperationRecipe;
using scene::OperationType;

namespace {

// A CCW axis-aligned rectangle chain: interior on the left of every
// traversal (the natural authoring orientation — the profile test's
// authoring helper, verbatim).
sdf::SketchChain rect_chain(float x0, float y0, float x1, float y1) {
    // Built as an initializer list (the GCC 14 stringop-overflow false
    // positive on the emplace_back path of this aggregate is documented
    // in the toolchain contract's caveat 1 family).
    return sdf::SketchChain{{
        sdf::sketch_segment({x0, y0}, {x1, y0}, true), // bottom, E
        sdf::sketch_segment({x1, y0}, {x1, y1}, true), // right, N
        sdf::sketch_segment({x1, y1}, {x0, y1}, true), // top, W
        sdf::sketch_segment({x0, y1}, {x0, y0}, true), // left, S
    }};
}

// The fixture's profile: the outer rectangle + one hole circle (the
// ring of D6 in miniature).
std::vector<sdf::SketchChain> ring_chains() {
    return {rect_chain(-1.0f, -0.5f, 1.0f, 0.5f),
            sdf::SketchChain{{sdf::sketch_circle({0.0f, 0.0f}, 0.2f, /*hole=*/true)}}};
}

// The determinism oracle: bit-identical sampling (value + gradient)
// and bounds. Grid: a stratified lattice over the bounds inflated by
// 15% on every side, PLUS the exact cap planes z = 0 / z = d and a
// ring on the hole's circle (the seam regimes the table's rows name).
std::vector<math::Vec3f> oracle_grid(const math::Bboxf& b, float d, int direction) {
    std::vector<math::Vec3f> pts;
    const math::Vec3f span = b.max - b.min;
    const math::Vec3f lo = b.min - span * 0.15f;
    const math::Vec3f hi = b.max + span * 0.15f;
    constexpr int nx = 11, ny = 9, nz = 7;
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k) {
                const float u = static_cast<float>(i) / (nx - 1);
                const float v = static_cast<float>(j) / (ny - 1);
                const float w = static_cast<float>(k) / (nz - 1);
                pts.push_back(lo + math::Vec3f{(hi.x - lo.x) * u, (hi.y - lo.y) * v,
                                               (hi.z - lo.z) * w});
            }
    // The cap planes (the slab's zero-set) at the profile's extents.
    const float zc = static_cast<float>(direction) * d * 0.5f;
    for (const float z : {zc - d * 0.5f, zc, zc + d * 0.5f})
        for (int i = 0; i < 24; ++i) {
            const float a = static_cast<float>(i) * (3.14159265f / 12.0f);
            pts.push_back(math::Vec3f{std::cos(a) * 1.1f, std::sin(a) * 0.6f, z});
            pts.push_back(math::Vec3f{std::cos(a) * 0.2f, std::sin(a) * 0.2f, z});
        }
    return pts;
}

bool bodies_bit_identical(const sdf::SDFBody& a, const sdf::SDFBody& b,
                          const std::vector<math::Vec3f>& pts) {
    const math::Bboxf ba = a.bounds(), bb = b.bounds();
    if (ba.min.x != bb.min.x || ba.min.y != bb.min.y || ba.min.z != bb.min.z ||
        ba.max.x != bb.max.x || ba.max.y != bb.max.y || ba.max.z != bb.max.z)
        return false;
    for (const math::Vec3f& p : pts) {
        const sdf::SDFSample sa = a.sample(p);
        const sdf::SDFSample sb = b.sample(p);
        if (sa.value != sb.value || sa.gradient.x != sb.gradient.x ||
            sa.gradient.y != sb.gradient.y || sa.gradient.z != sb.gradient.z)
            return false;
    }
    return true;
}

// The direct 2D-kernel construction of the fixture's extrude (what
// the scene path must reproduce bit-for-bit).
sdf::SDFBody direct_extrude(const std::vector<sdf::SketchChain>& chains,
                            float distance, int direction) {
    return sdf::SDFBody(std::make_unique<sdf::ExtrudeSDF>(chains, distance, direction));
}

} // namespace

// --- The sketch node: creation, entity add/edit/delete, definition state --

TEST_CASE("Sketch node: creation, entity add/edit/delete, open/closed "
          "state visible (revision 1: open is legal, flagged)") {
    scene::Scene s;
    const std::uint32_t id = s.add_sketch_node("S");
    REQUIRE(s.get_node(id) != nullptr);

    const scene::SceneNode* n = s.get_node(id);
    CHECK(n->kind == NodeKind::Sketch);
    CHECK(n->body.root() == nullptr);   // no body: nothing 3D to evaluate
    CHECK(n->factory == nullptr);       // no factory: no parametric recipe
    CHECK_FALSE(n->mesh_dirty);         // nothing to tessellate, ever
    CHECK(n->sketch.empty());

    // The empty sketch is NOT closed — there is no contour to extrude.
    CHECK_FALSE(s.sketch_closed(id));
    CHECK_FALSE(sdf::all_chains_closed(n->sketch));

    // update_meshes skips the node by kind (no crash, no cache).
    s.update_meshes(8);
    CHECK(s.get_node(id)->cached_mesh.positions.empty());
    // A sketch contributes no 3D extent to the scene bounds.
    CHECK(s.bounds().empty());

    // Build the rectangle entity by entity: chain 0 opens on the first
    // append (chain == count), the chain is OPEN (legal) after three
    // sides, closed after the fourth.
    const float x0 = -1.0f, y0 = -0.5f, x1 = 1.0f, y1 = 0.5f;
    s.sketch_add_entity(id, 0, sdf::sketch_segment({x0, y0}, {x1, y0}, true));
    s.sketch_add_entity(id, 0, sdf::sketch_segment({x1, y0}, {x1, y1}, true));
    CHECK(s.get_node(id)->sketch.size() == 1);
    CHECK_FALSE(s.sketch_closed(id)); // three sides: the chain is open
    s.sketch_add_entity(id, 0, sdf::sketch_segment({x1, y1}, {x0, y1}, true));
    s.sketch_add_entity(id, 0, sdf::sketch_segment({x0, y1}, {x0, y0}, true));
    CHECK(s.sketch_closed(id));
    CHECK(sdf::chain_is_closed(s.get_node(id)->sketch[0]));

    // Entity DELETE opens the chain again — visibly.
    CHECK(s.sketch_remove_entity(id, 0, 3)); // the left side
    CHECK_FALSE(s.sketch_closed(id));
    // ...and the edit that closes it back.
    s.sketch_add_entity(id, 0, sdf::sketch_segment({x0, y1}, {x0, y0}, true));
    CHECK(s.sketch_closed(id));

    // Entity EDIT (a parameter change): a vertex moved within closure
    // keeps the chain closed; a move past the neighbour opens it.
    s.sketch_set_entity(id, 0, 1, sdf::sketch_segment({x1, y0}, {x1, y1 + 0.25f}, true));
    CHECK_FALSE(s.sketch_closed(id)); // the top no longer meets the right
    s.sketch_set_entity(id, 0, 1, sdf::sketch_segment({x1, y0}, {x1, y1}, true));
    CHECK(s.sketch_closed(id));

    // A hole chain opens as chain 1 (a single circle closes itself).
    s.sketch_add_entity(id, 1, sdf::sketch_circle({0.0f, 0.0f}, 0.2f, true));
    CHECK(s.get_node(id)->sketch.size() == 2);
    CHECK(s.sketch_closed(id));
    CHECK(sdf::chain_is_closed(s.get_node(id)->sketch[1]));

    // Honest dispatch, loudly: entity editing on a non-sketch node is
    // a category error, and a chain index past the end is a caller bug.
    const scene::BuilderSpec* box_spec = scene::find_builder("box");
    REQUIRE(box_spec != nullptr);
    const std::uint32_t box_id =
        s.add_node("Box", box_spec->make({}), box_spec->factory(), box_spec->default_params());
    CHECK_THROWS_WITH(s.sketch_add_entity(box_id, 0, sdf::sketch_circle({0, 0}, 1, true)),
                      doctest::Contains("is not a sketch"));
    CHECK_THROWS_WITH(s.sketch_set_entity(id, 7, 0, sdf::sketch_circle({0, 0}, 1, true)),
                      doctest::Contains("no entity at chain 7"));
    // Param editing on a sketch is inert (its editing surface is the
    // payload, not a parametric recipe).
    CHECK_NOTHROW(s.set_node_param(id, "radius", 2.0f));
    CHECK(s.get_node(id)->params.empty());
}

// --- The extrude: the determinism oracle ------------------------------------

TEST_CASE("Extrude: closed sketch -> the prism, bit-identical to the "
          "direct 2D-kernel construction (both directions)") {
    for (const int direction : {1, -1}) {
        const float distance = 1.5f;
        scene::Scene s;
        const std::vector<sdf::SketchChain> chains = ring_chains();
        const std::uint32_t sk = s.add_sketch_node("S", chains);
        REQUIRE(s.sketch_closed(sk));

        OperationRecipe r;
        r.type = OperationType::Extrude;
        r.sketch = sk;
        r.distance = distance;
        r.direction = direction;
        const std::uint32_t ex = s.add_operation_node("Extrude", r);
        const scene::SceneNode* n = s.get_node(ex);
        REQUIRE(n != nullptr);
        CHECK(n->kind == NodeKind::Sdf);            // the prism IS a body
        REQUIRE(n->operation.has_value());
        CHECK(n->operation->type == OperationType::Extrude);
        CHECK(n->operation->sketch == sk);          // the genealogy
        CHECK_FALSE(n->operand_missing);
        CHECK_FALSE(n->operation_pending);
        CHECK(scene::operation_error(s, ex).empty());

        // The oracle: the scene's materialization vs the direct kernel
        // construction — bit-identical field and bounds.
        const sdf::SDFBody fresh = direct_extrude(chains, distance, direction);
        const auto pts = oracle_grid(n->body.bounds(), distance, direction);
        INFO("direction=", direction);
        CHECK(bodies_bit_identical(n->body, fresh, pts));
        // The z-range is the direction's half-line from the sketch plane.
        CHECK(n->body.bounds().min.z == (direction > 0 ? 0.0f : -distance));
        CHECK(n->body.bounds().max.z == (direction > 0 ? distance : 0.0f));

        // The rebuild pass is idempotent: an unchanged sketch
        // re-materializes to the SAME bits.
        const auto res = scene::rebuild_operations(s);
        CHECK(res.rebuilt == 1);
        CHECK(res.dangling.empty());
        CHECK(res.pending.empty());
        CHECK(bodies_bit_identical(s.get_node(ex)->body, fresh, pts));
    }
}

// --- Closure: the extrude's precondition (D2 revision 1) --------------------

TEST_CASE("Extrude: open sketch -> refused at creation, NO node, NO body") {
    scene::Scene s;
    // Three sides of the rectangle: a legal, OPEN editing state.
    const std::uint32_t sk = s.add_sketch_node("S", {sdf::SketchChain{{
        sdf::sketch_segment({-1.0f, -0.5f}, {1.0f, -0.5f}, true),
        sdf::sketch_segment({1.0f, -0.5f}, {1.0f, 0.5f}, true),
        sdf::sketch_segment({1.0f, 0.5f}, {-1.0f, 0.5f}, true),
    }}});
    REQUIRE_FALSE(s.sketch_closed(sk));
    const std::size_t count_before = s.node_count();
    const std::uint32_t next_before = s.next_id();

    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 1.0f;
    r.direction = 1;

    // Closure is the extrude's PRECONDITION: the certificate refuses
    // the open chain (the gap named), nothing is allocated, nothing
    // written.
    CHECK_THROWS_AS(s.add_operation_node("Extrude", r), std::invalid_argument);
    CHECK(s.node_count() == count_before);  // no node
    CHECK(s.next_id() == next_before);      // no id consumed, no body written

    // The same refusal, via the message the certificate owns: the gap
    // is named (the open chain's entities).
    try {
        s.add_operation_node("Extrude", r);
        FAIL("expected std::invalid_argument for an open sketch");
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("must share their joint") != std::string::npos);
    }

    // The recipe's own parameters refuse loudly too (the ADR-0029 D5
    // family, via the kernel): a non-positive distance never builds a
    // slab — checked against the CLOSED sketch, so the refusal is the
    // slab's own, not the certificate's.
    const std::uint32_t closed = s.add_sketch_node("Closed", ring_chains());
    OperationRecipe bad = r;
    bad.sketch = closed;
    bad.distance = 0.0f;
    CHECK_THROWS_WITH(s.add_operation_node("E2", bad),
                      doctest::Contains("distance"));

    // Honest dispatch: the sketch operand must BE a sketch.
    OperationRecipe wrong = r;
    const scene::BuilderSpec* box_spec = scene::find_builder("box");
    REQUIRE(box_spec != nullptr);
    const std::uint32_t box_id =
        s.add_node("Box", box_spec->make({}), box_spec->factory(), box_spec->default_params());
    wrong.sketch = box_id;
    CHECK_THROWS_WITH(s.add_operation_node("E3", wrong),
                      doctest::Contains("must be a sketch-kind node"));
    OperationRecipe absent = r;
    absent.sketch = 4242; // no such node
    CHECK_THROWS_WITH(s.add_operation_node("E4", absent),
                      doctest::Contains("does not exist"));
}

TEST_CASE("Extrude: an edit opens the chain -> the visible pending state "
          "(last good body kept, the certificate's message carried)") {
    scene::Scene s;
    const std::vector<sdf::SketchChain> chains = ring_chains();
    const std::uint32_t sk = s.add_sketch_node("S", chains);
    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 1.5f;
    r.direction = 1;
    const std::uint32_t ex = s.add_operation_node("Extrude", r);

    const sdf::SDFBody last_good = s.get_node(ex)->body; // deep copy
    const auto pts = oracle_grid(last_good.bounds(), 1.5f, 1);

    // The edit that OPENS the chain: move the right side's far vertex
    // past the top's reach.
    s.sketch_set_entity(sk, 0, 1,
                        sdf::sketch_segment({1.0f, -0.5f}, {1.0f, 0.9f}, true));
    CHECK_FALSE(s.sketch_closed(sk));

    const auto res = scene::rebuild_operations(s);
    REQUIRE(res.pending.size() == 1);
    CHECK(res.pending[0] == ex);
    CHECK(res.rebuilt == 0);
    CHECK(res.dangling.empty());

    const scene::SceneNode* n = s.get_node(ex);
    CHECK(n->operation_pending);           // the visible pending state
    CHECK_FALSE(n->operand_missing);      // the sketch EXISTS
    CHECK(bodies_bit_identical(n->body, last_good, pts)); // display continues

    // The actionable message: the certificate's own (the gap named),
    // under the state's name.
    const std::string err = scene::operation_error(s, ex);
    CHECK_FALSE(err.empty());
    CHECK(err.find("does not certify") != std::string::npos);
    CHECK(err.find("must share their joint") != std::string::npos);

    // Recovery is an edit away: close the chain, the pass
    // re-materializes — bit-identical to a fresh construction.
    s.sketch_set_entity(sk, 0, 1,
                        sdf::sketch_segment({1.0f, -0.5f}, {1.0f, 0.5f}, true));
    const auto res2 = scene::rebuild_operations(s);
    CHECK(res2.rebuilt == 1);
    CHECK(res2.pending.empty());
    CHECK_FALSE(s.get_node(ex)->operation_pending);
    CHECK(scene::operation_error(s, ex).empty());
    CHECK(bodies_bit_identical(s.get_node(ex)->body,
                               direct_extrude(chains, 1.5f, 1), pts));
}

// --- Propagation: the edit flows end-to-end (F-2 -> F-1) -------------------

TEST_CASE("Extrude: entity param edit -> the prism regenerates and the "
          "dependent subtract follows (bit-identical to fresh constructions)") {
    scene::Scene s;
    // The ADR's end-to-end genealogy: sketch -> circle -> extrude ->
    // subtract from a box (the parametric hole).
    const std::uint32_t sk = s.add_sketch_node("S", {sdf::SketchChain{{
        sdf::sketch_circle({0.0f, 0.0f}, 0.35f, /*hole=*/false)}}});
    REQUIRE(s.sketch_closed(sk));

    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 3.0f;
    r.direction = 1;
    const std::uint32_t ex = s.add_operation_node("Extrude", r);

    const scene::BuilderSpec* box_spec = scene::find_builder("box");
    REQUIRE(box_spec != nullptr);
    auto box_params = box_spec->default_params();
    box_params["extent_x"] = 1.2f;
    box_params["extent_y"] = 1.2f;
    box_params["extent_z"] = 1.2f;
    const std::uint32_t box_id = s.add_node("Box", box_spec->make(box_params),
                                            box_spec->factory(), box_params);

    OperationRecipe sub;
    sub.type = OperationType::Subtract;
    sub.target = box_id;
    sub.tool = ex; // the extrude downstream: an operation consuming an operation
    sub.offset_x = 0.3f;
    sub.offset_z = -1.5f; // the prism (z in [0,3]) pierces the box
    const std::uint32_t hole = s.add_operation_node("Hole", sub);
    REQUIRE(scene::rebuild_operations(s).rebuilt == 2); // extrude + subtract

    const std::string before_err = scene::operation_error(s, hole);
    CHECK(before_err.empty());

    // Grids for both levels of the chain.
    const auto ex_pts = oracle_grid(s.get_node(ex)->body.bounds(), 3.0f, 1);
    math::Bboxf sub_bounds = s.get_node(hole)->body.bounds();
    const math::Vec3f sub_span = sub_bounds.max - sub_bounds.min;
    sub_bounds.min = sub_bounds.min - sub_span * 0.15f;
    sub_bounds.max = sub_bounds.max + sub_span * 0.15f;
    const auto sub_pts = oracle_grid(sub_bounds, 1.2f, 1);

    // ---- THE EDIT: the sketch's circle radius (an entity parameter).
    s.sketch_set_entity(sk, 0, 0,
                        sdf::sketch_circle({0.0f, 0.0f}, 0.5f, /*hole=*/false));
    const auto res = scene::rebuild_operations(s);
    CHECK(res.rebuilt == 2); // the extrude AND its dependent, one pass
    CHECK(res.dangling.empty());
    CHECK(res.pending.empty());

    // The extrude: bit-identical to a fresh construction with the
    // EDITED parameter (the determinism oracle at level 1).
    const std::vector<sdf::SketchChain> edited{ sdf::SketchChain{{
        sdf::sketch_circle({0.0f, 0.0f}, 0.5f, false)}} };
    const sdf::SDFBody fresh_ex = direct_extrude(edited, 3.0f, 1);
    CHECK(bodies_bit_identical(s.get_node(ex)->body, fresh_ex, ex_pts));

    // The subtract followed: bit-identical to a fresh subtraction of
    // the fresh extrude (level 2 — the propagation is end-to-end).
    const sdf::SDFBody fresh_sub = sdf::sdf_subtract(
        s.get_node(box_id)->body,
        sdf::sdf_translate(fresh_ex, math::Vec3f{0.3f, 0.0f, -1.5f}));
    CHECK(bodies_bit_identical(s.get_node(hole)->body, fresh_sub, sub_pts));
}

// --- Dangling: delete sketch -> visible; undo (same id) -> recovered -------

TEST_CASE("Extrude: delete sketch -> visible dangling (body kept); undo "
          "(same id) -> recovered") {
    scene::Scene s;
    const std::vector<sdf::SketchChain> chains = ring_chains();
    const std::uint32_t sk = s.add_sketch_node("S", chains);
    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 1.5f;
    r.direction = 1;
    const std::uint32_t ex = s.add_operation_node("Extrude", r);

    const sdf::SDFBody last_good = s.get_node(ex)->body;
    const auto pts = oracle_grid(last_good.bounds(), 1.5f, 1);

    CHECK(s.remove_node(sk));
    const auto res = scene::rebuild_operations(s);
    REQUIRE(res.dangling.size() == 1);
    CHECK(res.dangling[0] == ex);
    CHECK(res.pending.empty());

    const scene::SceneNode* n = s.get_node(ex);
    CHECK(n->operand_missing);
    CHECK_FALSE(n->operation_pending); // the sketch is GONE, not broken
    CHECK(bodies_bit_identical(n->body, last_good, pts)); // display continues

    const std::string err = scene::operation_error(s, ex);
    CHECK_FALSE(err.empty());
    CHECK(err.find("#" + std::to_string(sk)) != std::string::npos);
    CHECK(err.find("(sketch)") != std::string::npos);
    CHECK(err.find("undo") != std::string::npos);

    // The undo of the delete: the sketch returns WITH ITS ORIGINAL ID
    // (restore-style insertion), the pass recovers the extrude to the
    // same body.
    s.restore_sketch_node(sk, "S", chains);
    const auto res2 = scene::rebuild_operations(s);
    CHECK(res2.rebuilt == 1);
    CHECK(res2.dangling.empty());
    CHECK_FALSE(s.get_node(ex)->operand_missing);
    CHECK(scene::operation_error(s, ex).empty());
    CHECK(bodies_bit_identical(s.get_node(ex)->body, last_good, pts));
}

// --- Cycles / self-reference ------------------------------------------------

TEST_CASE("Extrude: cyclic and self references are an explicit error") {
    scene::Scene s;
    // Hand-built structurally corrupt documents (only a broken file
    // can produce these in-app — creation is operands-first).
    auto make_extrude_node = [&](std::uint32_t id, std::uint32_t sketch_ref) {
        scene::SceneNode n;
        n.id = id;
        n.name = "E" + std::to_string(id);
        n.kind = NodeKind::Sdf;
        n.body = sdf::make_box(math::Vec3f{1, 1, 1});
        OperationRecipe r;
        r.type = OperationType::Extrude;
        r.sketch = sketch_ref;
        n.operation = r;
        s.restore_node(std::move(n));
    };
    make_extrude_node(1, 1); // self-reference (sketch == own id)
    make_extrude_node(2, 3); // 2 <-> 3 cycle
    make_extrude_node(3, 2);

    CHECK_THROWS_MESSAGE(scene::rebuild_operations(s), "cyclic");
}

// --- The extrude command: id-stable, ONE undo entry -------------------------

TEST_CASE("Extrude command: id-stable create/delete across undo/redo, "
          "ONE undo entry, the sketch stays visible") {
    viewport::CommandStack stack(64);
    scene::Scene s;
    const std::vector<sdf::SketchChain> chains = ring_chains();
    const std::uint32_t sk = s.add_sketch_node("S", chains);

    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 1.5f;
    r.direction = 1;

    // The extrude command (execute = create / re-create with the
    // pinned id; undo = remove): ONE undo entry per execution. The
    // subtract's operand auto-hide does NOT apply — a sketch has no
    // viewport body to hide (ADR-0030 D2).
    auto op_state = std::make_shared<std::uint32_t>(0u);
    stack.push(
        [&s, op_state, r] {
            if (*op_state == 0)
                *op_state = s.add_operation_node("Extrude", r);
            else
                s.restore_operation_node(*op_state, "Extrude", r);
            scene::rebuild_operations(s);
        },
        [&s, op_state] { s.remove_node(*op_state); },
        "Extrude");

    CHECK(stack.undo_count() == 1); // ONE entry per execution
    const std::uint32_t ex = *op_state;
    REQUIRE(s.get_node(ex) != nullptr);
    const sdf::SDFBody first = s.get_node(ex)->body;
    const auto pts = oracle_grid(first.bounds(), 1.5f, 1);

    stack.undo();
    CHECK(s.get_node(ex) == nullptr);
    CHECK(s.get_node(sk) != nullptr);
    CHECK(s.get_node(sk)->visible); // the sketch was never hidden

    stack.redo();
    REQUIRE(s.get_node(ex) != nullptr);
    CHECK(s.get_node(ex)->id == ex); // SAME id after redo — no rebind
    CHECK(s.get_node(ex)->operation->sketch == sk); // unchanged reference
    CHECK(bodies_bit_identical(s.get_node(ex)->body, first, pts)); // same body
    CHECK(s.next_id() == ex + 1);
}

// --- Undo interleaving (the F-1 case, extrude edition) ----------------------

TEST_CASE("Undo interleaving: delete sketch -> dangling -> undo the extrude "
          "-> redo: coherent ids, no rebind") {
    // The stack is [.., Extrude, Delete-sketch]; the LIFO order forces
    // the interleave. Ids are compared at every step — any drift is
    // exactly the failure this test exists to catch (acceptance
    // correction 3, applied to the extrude).
    viewport::CommandStack stack(64);
    scene::Scene s;
    const std::vector<sdf::SketchChain> chains = ring_chains();
    const std::uint32_t sk = s.add_sketch_node("S", chains);
    OperationRecipe r;
    r.type = OperationType::Extrude;
    r.sketch = sk;
    r.distance = 1.5f;
    r.direction = 1;
    const std::uint32_t ex = s.add_operation_node("Extrude", r);
    const sdf::SDFBody body = s.get_node(ex)->body;
    const auto pts = oracle_grid(body.bounds(), 1.5f, 1);

    // The Extrude command (redo restores the pinned id).
    auto op_state = std::make_shared<std::uint32_t>(ex);
    stack.push(
        [&s, op_state, r] {
            if (s.get_node(*op_state) == nullptr)
                s.restore_operation_node(*op_state, "Extrude", r);
            scene::rebuild_operations(s);
        },
        [&s, op_state] { s.remove_node(*op_state); },
        "Extrude");

    // The Delete-sketch command (execute = delete + rebuild to dangle;
    // undo = restore the same id + rebuild to recover).
    stack.push(
        [&s, sk] {
            s.remove_node(sk);
            scene::rebuild_operations(s);
        },
        [&s, sk, chains] {
            s.restore_sketch_node(sk, "S", chains);
            scene::rebuild_operations(s);
        },
        "Delete Sketch");

    // After the Delete: the extrude dangles, keeping its body.
    REQUIRE(s.get_node(ex)->operand_missing);
    CHECK(s.get_node(sk) == nullptr);

    // ---- Undo the Delete (LIFO pops it first): recovered, same ids.
    stack.undo();
    CHECK(s.get_node(sk) != nullptr);
    CHECK(s.get_node(sk)->id == sk);                    // sketch: SAME id
    CHECK_FALSE(s.get_node(ex)->operand_missing);      // recovered
    CHECK(s.get_node(ex)->id == ex);                    // extrude: SAME id
    CHECK(bodies_bit_identical(s.get_node(ex)->body, body, pts));

    // ---- Undo the Extrude: the operation node is removed; the sketch
    // stays (it was never hidden — nothing to unhide).
    stack.undo();
    CHECK(s.get_node(ex) == nullptr);
    CHECK(s.get_node(sk) != nullptr);
    CHECK(s.get_node(sk)->visible);

    // ---- Redo the Extrude: it returns WITH THE SAME ID, referencing
    // THE SAME sketch — no rebind, ever.
    stack.redo();
    REQUIRE(s.get_node(ex) != nullptr);
    CHECK(s.get_node(ex)->id == ex);
    CHECK(s.get_node(ex)->operation->sketch == sk); // unchanged reference
    CHECK_FALSE(s.get_node(ex)->operand_missing);
    CHECK(bodies_bit_identical(s.get_node(ex)->body, body, pts));

    // ---- Redo the Delete: the extrude re-dangles, same ids, same body.
    stack.redo();
    REQUIRE(s.get_node(ex) != nullptr);
    CHECK(s.get_node(ex)->operand_missing);
    CHECK(s.get_node(sk) == nullptr);
    CHECK(bodies_bit_identical(s.get_node(ex)->body, body, pts));
}
