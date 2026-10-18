// tests/unit/scene/test_operations.cpp
//
// The Phase F document-layer contract (ADR-0027 D2), tested WITHOUT
// PySide: the recipe, the pure materialization, the DAG rebuild pass,
// the visible dangling state, id-stable command re-creation, and the
// undo-interleaving case (acceptance correction 3) on the REAL C++
// CommandStack — the same primitives the desktop app drives through
// the bindings.
//
// Bit-identity oracle: io::native::serialize_node() renders an SDF
// tree as deterministic JSON — two bodies are bit-identical iff their
// serializations are byte-equal (the same oracle the format's
// round-trip tests use).
#include <doctest/doctest.h>

#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/scene/builders.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/viewport/command_stack.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <memory>
#include <string>

using namespace CAD_0;
using scene::NodeKind;
using scene::OperationRecipe;
using scene::OperationType;

namespace {

// Deterministic JSON rendering of an SDF tree — the bit-identity
// oracle (two bodies are bit-identical iff these are byte-equal).
std::string body_json_s(const sdf::SDFBody& b) {
    return io::native::serialize_node(*b.root()).dump(true);
}

std::string body_json(const scene::SceneNode& n) {
    return body_json_s(n.body);
}

// The pierced-box scene of the ADR-0027 MVP: a parametric box (id B)
// and a parametric cylinder (id C), both wired to the builder
// registry so param edits rebuild their bodies.
struct PiercedScene {
    scene::Scene s;
    std::uint32_t box_id{0};
    std::uint32_t cyl_id{0};
    std::uint32_t op_id{0};

    explicit PiercedScene(float cyl_radius = 0.25f) {
        const scene::BuilderSpec* box_spec = scene::find_builder("box");
        const scene::BuilderSpec* cyl_spec = scene::find_builder("cylinder");
        REQUIRE(box_spec != nullptr);
        REQUIRE(cyl_spec != nullptr);

        box_id = s.add_node("Box", box_spec->make({}),
                            box_spec->factory(), box_spec->default_params());
        auto cyl_params = cyl_spec->default_params();
        cyl_params["radius"] = cyl_radius;
        cyl_id = s.add_node("Cylinder", cyl_spec->make(cyl_params),
                            cyl_spec->factory(), cyl_params);

        OperationRecipe r;
        r.type = OperationType::Subtract;
        r.target = box_id;
        r.tool = cyl_id;
        op_id = s.add_operation_node("Subtract", r);
    }
};

} // namespace

TEST_CASE("Operations: materialize_operation is subtract+translate of the operands") {
    auto box = sdf::make_box(math::Vec3f{1, 1, 1});
    auto cyl = sdf::make_cylinder(0.25f, 3.0f);

    OperationRecipe r;
    r.target = 1;
    r.tool = 2;
    r.offset_x = 0.5f;
    r.offset_y = 0.0f;
    r.offset_z = -0.25f;

    const auto body = scene::materialize_operation(r, box, cyl);
    REQUIRE(body.root() != nullptr);
    // The pure function is exactly sdf_subtract(a, translate(b, off)):
    // same construction, same tree.
    const auto reference = sdf::sdf_subtract(
        box, sdf::sdf_translate(cyl, math::Vec3f{0.5f, 0.0f, -0.25f}));
    CHECK(body_json_s(body) == body_json_s(reference));
}

TEST_CASE("Operations: add_operation_node materializes and rebuild is idempotent") {
    PiercedScene ps;
    const scene::SceneNode* op = ps.s.get_node(ps.op_id);
    REQUIRE(op != nullptr);
    REQUIRE(op->operation.has_value());
    CHECK(op->operation->target == ps.box_id);
    CHECK(op->operation->tool == ps.cyl_id);
    CHECK_FALSE(op->operand_missing);

    // The materialized body equals a fresh pure construction.
    const std::string fresh = body_json_s(scene::materialize_operation(
        *op->operation, ps.s.get_node(ps.box_id)->body,
        ps.s.get_node(ps.cyl_id)->body));
    CHECK(body_json(*op) == fresh);

    // An unchanged-input rebuild pass is idempotent: same bytes.
    const auto r1 = scene::rebuild_operations(ps.s);
    CHECK(r1.rebuilt == 1);
    CHECK(r1.dangling.empty());
    CHECK(body_json(*ps.s.get_node(ps.op_id)) == fresh);
}

TEST_CASE("Operations: operand param edit propagates bit-identically to a fresh construction") {
    PiercedScene ps;
    const scene::SceneNode* op = ps.s.get_node(ps.op_id);
    const std::string before = body_json(*op);

    // Edit the tool's radius: the operand body rebuilds, the DAG pass
    // re-materializes the dependent.
    ps.s.set_node_param(ps.cyl_id, "radius", 0.4f);
    const auto r = scene::rebuild_operations(ps.s);
    CHECK(r.rebuilt == 1);
    CHECK(r.dangling.empty());

    const std::string after = body_json(*ps.s.get_node(ps.op_id));
    CHECK(after != before);  // the edit did change the result

    // Bit-identical to a fresh construction from the EDITED operands
    // (ADR-0027 D3: determinism of the rebuild).
    const scene::SceneNode* cyl = ps.s.get_node(ps.cyl_id);
    const OperationRecipe rec = *ps.s.get_node(ps.op_id)->operation;
    const std::string fresh = body_json_s(scene::materialize_operation(
        rec, ps.s.get_node(ps.box_id)->body, cyl->body));
    CHECK(after == fresh);
}

TEST_CASE("Operations: chained subtracts propagate in one pass") {
    PiercedScene ps;

    // A second hole: subtract the SAME cylinder from the first
    // operation's result (an operation whose target is an operation).
    OperationRecipe r2;
    r2.target = ps.op_id;
    r2.tool = ps.cyl_id;
    r2.offset_x = 0.5f;
    const std::uint32_t op2 = ps.s.add_operation_node("Subtract2", r2);

    ps.s.set_node_param(ps.cyl_id, "radius", 0.3f);
    const auto r = scene::rebuild_operations(ps.s);
    CHECK(r.rebuilt == 2);  // both re-materialized, in dependency order
    CHECK(r.dangling.empty());

    // The chained body is the fresh construction from the CURRENT
    // first-operation body.
    const OperationRecipe rec2 = *ps.s.get_node(op2)->operation;
    const std::string fresh = body_json_s(scene::materialize_operation(
        rec2, ps.s.get_node(ps.op_id)->body,
        ps.s.get_node(ps.cyl_id)->body));
    CHECK(body_json(*ps.s.get_node(op2)) == fresh);
}

TEST_CASE("Operations: operand delete -> visible dangling, last-good body kept") {
    PiercedScene ps;
    const std::string last_good = body_json(*ps.s.get_node(ps.op_id));

    CHECK(ps.s.remove_node(ps.cyl_id));
    const auto r = scene::rebuild_operations(ps.s);
    REQUIRE(r.dangling.size() == 1);
    CHECK(r.dangling[0] == ps.op_id);

    const scene::SceneNode* op = ps.s.get_node(ps.op_id);
    REQUIRE(op != nullptr);
    CHECK(op->operand_missing);
    // The last materialized body is KEPT — display continues.
    CHECK(body_json(*op) == last_good);

    // The actionable message names the missing operand and the role.
    const std::string err = scene::operation_error(ps.s, ps.op_id);
    CHECK_FALSE(err.empty());
    CHECK(err.find("#" + std::to_string(ps.cyl_id)) != std::string::npos);
    CHECK(err.find("(tool)") != std::string::npos);
    CHECK(err.find("undo") != std::string::npos);
    // A healthy node reports no error.
    CHECK(scene::operation_error(ps.s, ps.box_id).empty());
}

TEST_CASE("Operations: undo of the delete (same id) -> recovered") {
    PiercedScene ps;
    const std::string before = body_json(*ps.s.get_node(ps.op_id));

    ps.s.remove_node(ps.cyl_id);
    scene::rebuild_operations(ps.s);
    REQUIRE(ps.s.get_node(ps.op_id)->operand_missing);

    // The undo of the delete: the operand returns WITH ITS ORIGINAL ID
    // (restore-style insertion), params intact.
    const scene::BuilderSpec* cyl_spec = scene::find_builder("cylinder");
    REQUIRE(cyl_spec != nullptr);
    auto cyl_params = cyl_spec->default_params();
    cyl_params["radius"] = 0.25f;
    ps.s.restore_sdf_node(ps.cyl_id, "Cylinder",
                          cyl_spec->make(cyl_params));
    {
        // Re-attach the editing metadata (restore_sdf_node takes the
        // body; builder/params are editing metadata).
        scene::SceneNode* n = ps.s.get_node_mut(ps.cyl_id);
        n->builder = "cylinder";
        n->params = cyl_params;
        n->factory = cyl_spec->factory();
    }

    const auto r = scene::rebuild_operations(ps.s);
    CHECK(r.rebuilt == 1);
    CHECK(r.dangling.empty());
    CHECK_FALSE(ps.s.get_node(ps.op_id)->operand_missing);
    // Recovered to the same body as before the round-trip.
    CHECK(body_json(*ps.s.get_node(ps.op_id)) == before);
}

TEST_CASE("Operations: cyclic references are an explicit error") {
    scene::Scene s;
    // Hand-built structurally corrupt document (only a broken file can
    // produce this in practice — in-app creation is operands-first).
    auto make_op_node = [&](std::uint32_t id, std::uint32_t target,
                            std::uint32_t tool) {
        scene::SceneNode n;
        n.id = id;
        n.name = "Op" + std::to_string(id);
        n.kind = NodeKind::Sdf;
        n.body = sdf::make_box(math::Vec3f{1, 1, 1});
        OperationRecipe r;
        r.target = target;
        r.tool = tool;
        n.operation = r;
        s.restore_node(std::move(n));
    };
    make_op_node(1, 2, 3);
    make_op_node(2, 1, 3);  // 1 <-> 2 cycle
    make_op_node(3, 3, 3);  // self-reference

    CHECK_THROWS_MESSAGE(scene::rebuild_operations(s), "cyclic");
}

TEST_CASE("Operations: restore refuses an id already in use (never a rebind)") {
    PiercedScene ps;
    auto box = sdf::make_box(math::Vec3f{1, 1, 1});
    CHECK_THROWS(ps.s.restore_sdf_node(ps.box_id, "Clone", box));
    CHECK_THROWS(ps.s.restore_operation_node(
        ps.op_id, "Clone", OperationRecipe{OperationType::Subtract,
                                           ps.box_id, ps.cyl_id}));
}

TEST_CASE("Operations: create/delete commands keep ids stable across undo/redo (ADR-0027 D2 prerequisite)") {
    // The REAL C++ CommandStack, the same class the app drives.
    viewport::CommandStack stack(64);
    scene::Scene s;
    const scene::BuilderSpec* cyl_spec = scene::find_builder("cylinder");
    REQUIRE(cyl_spec != nullptr);
    auto cyl_params = cyl_spec->default_params();
    cyl_params["radius"] = 0.25f;

    // A create command with id-stable redo, exactly the shape the app
    // uses after this unit: execute creates (or re-creates with the
    // pinned id), undo removes.
    auto state = std::make_shared<std::uint32_t>(0);
    const std::uint32_t expected_id = s.next_id();
    stack.push(
        [&s, cyl_spec, cyl_params, state] {
            if (*state == 0) {
                *state = s.add_node("Cylinder", cyl_spec->make(cyl_params),
                                    cyl_spec->factory(), cyl_params);
            } else {
                s.restore_sdf_node(*state, "Cylinder",
                                   cyl_spec->make(cyl_params));
            }
        },
        [&s, state] { s.remove_node(*state); },
        "Create Cylinder");

    REQUIRE(*state == expected_id);
    const std::uint32_t cyl_id = *state;

    stack.undo();
    CHECK(s.get_node(cyl_id) == nullptr);
    stack.redo();
    REQUIRE(s.get_node(cyl_id) != nullptr);  // SAME id after redo
    CHECK(s.get_node(cyl_id)->id == cyl_id);
    CHECK(s.next_id() == cyl_id + 1);
}

TEST_CASE("Operations: undo interleaving - delete operand, undo subtract, redo (same ids, no rebind)") {
    // Acceptance correction 3, on the real CommandStack: the stack is
    // [.., Subtract, Delete-cylinder]; the LIFO order forces the
    // interleave. Ids are compared at every step — any drift is
    // exactly the failure this test exists to catch.
    viewport::CommandStack stack(64);
    PiercedScene ps;
    const std::uint32_t cyl_id = ps.cyl_id;
    const std::uint32_t op_id = ps.op_id;

    const scene::BuilderSpec* cyl_spec = scene::find_builder("cylinder");
    REQUIRE(cyl_spec != nullptr);
    auto cyl_params = cyl_spec->default_params();
    cyl_params["radius"] = 0.25f;

    // The Subtract command (execute = create + hide operands; undo =
    // remove + unhide; redo = restore with the pinned id + re-hide).
    auto op_state = std::make_shared<std::uint32_t>(op_id);
    OperationRecipe rec = *ps.s.get_node(op_id)->operation;
    stack.push(
        [&s = ps.s, op_state, rec, cyl_id, box_id = ps.box_id] {
            if (s.get_node(*op_state) == nullptr) {
                s.restore_operation_node(*op_state, "Subtract", rec);
            }
            // (Re-)hide the operands: the operation is the display.
            s.set_visible(box_id, false);
            s.set_visible(cyl_id, false);
            scene::rebuild_operations(s);
        },
        [&s = ps.s, op_state, cyl_id, box_id = ps.box_id] {
            s.remove_node(*op_state);
            s.set_visible(box_id, true);
            s.set_visible(cyl_id, true);
        },
        "Subtract");

    // The Delete-operand command (execute = delete + rebuild to
    // dangle; undo = restore same id + rebuild to recover).
    stack.push(
        [&s = ps.s, cyl_id] {
            s.remove_node(cyl_id);
            scene::rebuild_operations(s);
        },
        [&s = ps.s, cyl_id, cyl_spec, cyl_params] {
            s.restore_sdf_node(cyl_id, "Cylinder",
                               cyl_spec->make(cyl_params));
            scene::SceneNode* n = s.get_node_mut(cyl_id);
            n->builder = "cylinder";
            n->params = cyl_params;
            n->factory = cyl_spec->factory();
            scene::rebuild_operations(s);
        },
        "Delete Cylinder");

    // After the Delete: the operation dangles, keeping its body.
    REQUIRE(ps.s.get_node(op_id)->operand_missing);
    const std::string op_body_dangling =
        body_json(*ps.s.get_node(op_id));

    // ---- Undo the Delete (LIFO pops it first): recovered, same ids.
    stack.undo();
    CHECK(ps.s.get_node(cyl_id) != nullptr);
    CHECK(ps.s.get_node(cyl_id)->id == cyl_id);          // operand: SAME id
    CHECK_FALSE(ps.s.get_node(op_id)->operand_missing);  // recovered
    CHECK(ps.s.get_node(op_id)->id == op_id);            // operation: SAME id

    // ---- Undo the Subtract: the operation node is removed, the
    // operands visible again.
    stack.undo();
    CHECK(ps.s.get_node(op_id) == nullptr);
    CHECK(ps.s.get_node(cyl_id) != nullptr);
    CHECK(ps.s.get_node(ps.box_id)->visible);
    CHECK(ps.s.get_node(cyl_id)->visible);

    // ---- Redo the Subtract: the operation returns WITH THE SAME ID,
    // referencing THE SAME operand ids — no rebind, ever.
    stack.redo();
    REQUIRE(ps.s.get_node(op_id) != nullptr);
    CHECK(ps.s.get_node(op_id)->id == op_id);
    const scene::SceneNode* op = ps.s.get_node(op_id);
    REQUIRE(op->operation.has_value());
    CHECK(op->operation->target == ps.box_id);  // unchanged reference
    CHECK(op->operation->tool == cyl_id);       // unchanged reference
    CHECK_FALSE(op->operand_missing);
    CHECK(body_json(*op) == op_body_dangling);  // same materialized body
    CHECK_FALSE(ps.s.get_node(ps.box_id)->visible);  // operands re-hidden
    CHECK_FALSE(ps.s.get_node(cyl_id)->visible);

    // ---- Redo the Delete: the operation re-dangles, same ids.
    stack.redo();
    REQUIRE(ps.s.get_node(op_id) != nullptr);
    CHECK(ps.s.get_node(op_id)->operand_missing);
    CHECK(ps.s.get_node(cyl_id) == nullptr);
    CHECK(body_json(*ps.s.get_node(op_id)) == op_body_dangling);
}
