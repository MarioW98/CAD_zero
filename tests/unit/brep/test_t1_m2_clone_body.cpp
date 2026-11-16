// tests/unit/brep/test_t1_m2_clone_body.cpp
//
// T1 m2 — THE SANCTIONED CLONE, CHARACTERIZED (ADR-0044 D3.4/D7-m2,
// accepted 2026-11-16). clone_body is the ONE producer of an
// independent topology generation; the raw copy and the raw move are
// deleted at the class. This suite pins the bite's twelve mandates,
// each against a real body (the box, the k=1 fillet, the ellipsoid,
// the bridge's faceted world — the one m1's evidence never measured):
//
//   1. the copy/move members are compile-time rejected (static
//      asserts) — the audited silent std::move-to-copy fallback is
//      gone;
//   2. a cloned analytic B-Rep is independent of its source;
//   3. the source can die while the clone stays valid;
//   4. every cloned radial pointer resolves to storage owned by the
//      clone;
//   5. no cloned radial pointer aliases source CoEdge storage;
//   6. repeated relinking is idempotent;
//   7. clone topology passes the structural invariants (loop
//      closure, radial incidence, edge manifoldness, orientation,
//      Euler characteristic);
//   8. values survive whole: ids, ShapeIds, names, provenance,
//      p-curves, vertex positions; the surfaces stay shared;
//   9. an invalid source is refused LOUDLY (the strong guarantee) —
//      the silent omission of link_radial_edges is not inherited;
//  10. SceneNode cloning (copy ctor, assignment, the async display
//      snapshot) yields independent generations;
//  11. fillet reconstruction stays deterministic through the clone
//      (clone twice, compare whole);
//  12. the migrated subtract paths stay correct (the separated tool
//      returns the exact clone of the target); the pierced path's
//      correctness is the existing boolean suite's own green, now
//      running through the migrated tool clone.
//
// Lifetime discipline (the bite's own law): no test dereferences or
// compares a pointer whose target has died — addresses are compared
// as VALUES inside sets captured while every allocation is alive.
#include <doctest/doctest.h>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace b = CAD_0::brep;
namespace bridge = CAD_0::bridge;
namespace math = CAD_0::math;
namespace sdf = CAD_0::sdf;
namespace scene = CAD_0::scene;
using CAD_0::geometry::FeatureId;

namespace {

constexpr double kR = 0.5;

// Every coedge address the body owns (its loops' own storage) — the
// membership witness for the independence pins (the m1 helper, one
// suite over).
std::set<const b::CoEdge*> all_coedge_addresses(
    const b::BRepBody& body) {
    std::set<const b::CoEdge*> out;
    for (const auto& f : body.solid.shell().faces())
        for (const auto& lp : f.loops())
            for (const auto& ce : lp.coedges())
                out.insert(&ce);
    return out;
}

// Every radial pointer the body's edges hold.
std::set<const b::CoEdge*> all_radial_pointers(
    const b::BRepBody& body) {
    std::set<const b::CoEdge*> out;
    for (const auto& e : body.edges)
        for (const b::CoEdge* ce : e.coedges())
            out.insert(ce);
    return out;
}

// The face-name sequence — the genealogy witness (names are the
// cross-generation identity, ADR-0036 D1/D3).
std::vector<std::string> face_names(const b::BRepBody& body) {
    std::vector<std::string> out;
    for (const auto& f : body.solid.shell().faces())
        out.push_back(f.name());
    return out;
}

// The full structural invariants, named: loop closure, radial
// incidence (2 per edge — manifoldness), orientation (opposite
// pairs), and the Euler characteristic the caller expects.
void check_structural(const b::BRepBody& body,
                      std::int64_t want_chi) {
    CHECK(b::loops_are_closed(body));
    for (const auto& e : body.edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() !=
              e.coedges()[1]->reversed());
    }
    CHECK(b::is_closed_orientable(body));
    CHECK(b::euler_characteristic(body) == want_chi);
}

// The k=1 fillet body on a unit box (the m1 fixture's own recipe).
std::unique_ptr<b::BRepBody> make_fillet_body() {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid =
        s.add_fillet_node("", b::FilletRecipe{"box#1:vertical2", kR});
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);
    // The scene owns the node; the test takes the sanctioned clone
    // (the body itself is the scene's generation — never moved out).
    return b::clone_body(*f->fillet_body);
}

}  // namespace

// ---------------------------------------------------------------------------
// MANDATE 1 — the raw copy and move are compile-time rejected; the
// sanctioned producer is the only road. The audited hazard: the
// user-declared dtor suppressed the implicit moves, so std::move
// silently selected the COPY (radials aliasing the source). Deleted
// members turn every such site into a compile error.
// ---------------------------------------------------------------------------
static_assert(!std::is_copy_constructible_v<b::BRepBody>,
              "T1 m2: BRepBody's raw copy is deleted — the shallow "
              "radial hazard cannot be reached (ADR-0044 D3.4)");
static_assert(!std::is_copy_assignable_v<b::BRepBody>,
              "T1 m2: BRepBody's raw copy assignment is deleted — an "
              "unlinked body cannot be minted by assignment");
static_assert(!std::is_move_constructible_v<b::BRepBody>,
              "T1 m2: BRepBody's raw move is deleted — no second "
              "producer of the same generation (moves travel by "
              "unique_ptr)");
static_assert(!std::is_move_assignable_v<b::BRepBody>,
              "T1 m2: BRepBody's raw move assignment is deleted");

TEST_SUITE("brep clone body (T1 m2)") {

TEST_CASE("the clone is a fully independent generation: every radial "
          "pointer resolves in the clone, none aliases the source, "
          "the values and the identity survive whole") {
    auto src = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(src != nullptr);
    REQUIRE(b::is_closed_orientable(*src));

    const std::set<const b::CoEdge*> src_storage =
        all_coedge_addresses(*src);
    const std::set<const b::CoEdge*> src_radials =
        all_radial_pointers(*src);

    auto dst = b::clone_body(*src);
    REQUIRE(dst != nullptr);

    // MANDATE 4 — every radial pointer resolves to storage owned by
    // the clone (radial incidence preserved: 2 per edge).
    const std::set<const b::CoEdge*> own = all_coedge_addresses(*dst);
    REQUIRE(own.size() == dst->edge_count() * 2);
    const std::set<const b::CoEdge*> dst_radials =
        all_radial_pointers(*dst);
    REQUIRE(dst_radials.size() == dst->edge_count() * 2);
    for (const b::CoEdge* ce : dst_radials) CHECK(own.count(ce) == 1);

    // MANDATE 5 — no cloned radial pointer aliases source storage
    // (compared as address VALUES, both bodies alive: no dead
    // dereference anywhere in this suite).
    for (const b::CoEdge* ce : dst_radials)
        CHECK(src_storage.count(ce) == 0);
    CHECK(dst_radials != src_radials);

    // MANDATE 8 — the values survive whole: counts, ids, positions,
    // names, ShapeIds, and the SHARED surfaces (immutable analytic
    // values — the one sharing that is by design).
    REQUIRE(dst->vertex_count() == src->vertex_count());
    REQUIRE(dst->edge_count() == src->edge_count());
    REQUIRE(dst->face_count() == src->face_count());
    for (std::size_t i = 0; i < src->vertex_count(); ++i)
        CHECK(dst->vertices[i].position().x ==
              src->vertices[i].position().x);
    for (std::size_t i = 0; i < src->edge_count(); ++i) {
        CHECK(dst->edges[i].id() == src->edges[i].id());
        CHECK(dst->edges[i].vertex(0) == src->edges[i].vertex(0));
        CHECK(dst->edges[i].vertex(1) == src->edges[i].vertex(1));
    }
    for (std::size_t i = 0; i < src->face_count(); ++i) {
        CHECK(dst->solid.shell().faces()[i].id() ==
              src->solid.shell().faces()[i].id());
        CHECK(dst->solid.shell().faces()[i].name() ==
              src->solid.shell().faces()[i].name());
        CHECK(dst->solid.shell().faces()[i].shape_id() ==
              src->solid.shell().faces()[i].shape_id());
        CHECK(dst->solid.shell().faces()[i].surface() ==
              src->solid.shell().faces()[i].surface());
    }

    // MANDATE 7 — the structural invariants on the clone's own.
    check_structural(*dst, 2);
}

TEST_CASE("the source can die while the clone stays valid — the "
          "generation is independent by construction") {
    // The fillet body: the clone's own object graph (named faces,
    // p-curves, a non-trivial generation) held across the source's
    // death.
    std::unique_ptr<b::BRepBody> dst;
    std::vector<std::string> names_before;
    {
        auto src = make_fillet_body();
        REQUIRE(src != nullptr);
        REQUIRE(src->vertex_count() == 10);
        names_before = face_names(*src);
        dst = b::clone_body(*src);
        REQUIRE(dst != nullptr);
    }
    // The source's whole graph is dead here; the clone answers from
    // its own storage alone (no pointer of the clone ever pointed
    // into the source — pinned above, relied on here).
    REQUIRE(dst->vertex_count() == 10);
    REQUIRE(dst->edge_count() == 15);
    REQUIRE(dst->face_count() == 7);
    CHECK(face_names(*dst) == names_before);
    check_structural(*dst, 2);
}

TEST_CASE("repeated relinking is idempotent — the clone re-links to "
          "the same targets") {
    auto src = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    auto dst = b::clone_body(*src);
    REQUIRE(dst != nullptr);
    const std::set<const b::CoEdge*> before =
        all_radial_pointers(*dst);
    b::link_radial_edges(*dst);
    b::link_radial_edges(*dst);
    CHECK(all_radial_pointers(*dst) == before);
    check_structural(*dst, 2);
}

TEST_CASE("the measured bodies all clone whole: the fillet's p-curves, "
          "the ellipsoid, the bridge's faceted world (the one m1 "
          "never measured)") {
    // (a) THE FILLET (k=1): the named generation with p-curves.
    {
        auto src = make_fillet_body();
        REQUIRE(src != nullptr);
        auto dst = b::clone_body(*src);
        REQUIRE(dst != nullptr);
        REQUIRE(dst->vertex_count() == 10);
        REQUIRE(dst->edge_count() == 15);
        REQUIRE(dst->face_count() == 7);
        CHECK(face_names(*dst) == face_names(*src));
        // The p-curves ride the loops' co-edges (values, ADR-0032
        // D2.2): the flank's loop coedges stay p-curved in the clone.
        std::size_t src_p = 0, dst_p = 0;
        for (const auto& f : src->solid.shell().faces())
            for (const auto& lp : f.loops())
                for (const auto& ce : lp.coedges())
                    if (ce.p_curve() != nullptr) ++src_p;
        for (const auto& f : dst->solid.shell().faces())
            for (const auto& lp : f.loops())
                for (const auto& ce : lp.coedges())
                    if (ce.p_curve() != nullptr) ++dst_p;
        REQUIRE(src_p > 0);
        CHECK(dst_p == src_p);
        // Provenance rides every entity (exact/0 for the analytic
        // path — the worst-of rule's own inputs).
        CHECK(dst->solid.shell().faces()[0].provenance().is_exact() ==
              src->solid.shell().faces()[0].provenance().is_exact());
        const std::set<const b::CoEdge*> src_storage =
            all_coedge_addresses(*src);
        for (const b::CoEdge* ce : all_radial_pointers(*dst))
            CHECK(src_storage.count(ce) == 0);
        check_structural(*dst, 2);
    }
    // (b) THE ELLIPSOID (K1 m3's spheroid — seam-wired, disk-faced).
    {
        auto src = b::make_ellipsoid(1.0, 0.6, math::Vec3d{},
                                     FeatureId{7});
        REQUIRE(src != nullptr);
        REQUIRE(b::is_closed_orientable(*src));
        auto dst = b::clone_body(*src);
        REQUIRE(dst != nullptr);
        REQUIRE(dst->vertex_count() == src->vertex_count());
        REQUIRE(dst->edge_count() == src->edge_count());
        REQUIRE(dst->face_count() == src->face_count());
        const std::set<const b::CoEdge*> src_storage =
            all_coedge_addresses(*src);
        for (const b::CoEdge* ce : all_radial_pointers(*dst))
            CHECK(src_storage.count(ce) == 0);
        check_structural(*dst, b::euler_characteristic(*src));
    }
    // (c) THE FACETED BRIDGE BODY: each welded MC triangle is one
    // plane face, every entity Approximated with the declared grid
    // tolerance — the world ADR-0044 D6 covered by contract and m1
    // never measured. The clone carries it whole: this IS the
    // dedicated test the bite allowed to defer.
    {
        auto src = bridge::sdf_to_brep_faceted(
            sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
        REQUIRE(src != nullptr);
        REQUIRE(src->face_count() > 20);  // triangles, not the 6
                                          // analytic planes
        REQUIRE(b::loops_are_closed(*src));
        auto dst = b::clone_body(*src);
        REQUIRE(dst != nullptr);
        REQUIRE(dst->vertex_count() == src->vertex_count());
        REQUIRE(dst->edge_count() == src->edge_count());
        REQUIRE(dst->face_count() == src->face_count());
        // The sampling declaration rides the clone undiluted
        // (Layer-2 honesty: a faceted clone that read exact would be
        // the lie the provenance field exists to prevent).
        CHECK(dst->solid.shell().faces()[0].provenance().is_exact() ==
              src->solid.shell().faces()[0].provenance().is_exact());
        CHECK_FALSE(
            src->solid.shell().faces()[0].provenance().is_exact());
        const std::set<const b::CoEdge*> src_storage =
            all_coedge_addresses(*src);
        const std::set<const b::CoEdge*> own =
            all_coedge_addresses(*dst);
        // One triangle loop per face, three coedges per loop: the
        // membership set is exactly 3 x faces (26504 faces -> 79512
        // coedges on this column, measured).
        REQUIRE(own.size() == src->face_count() * 3);
        for (const b::CoEdge* ce : all_radial_pointers(*dst)) {
            CHECK(own.count(ce) == 1);
            CHECK(src_storage.count(ce) == 0);
        }
        CHECK(b::loops_are_closed(*dst));
    }
}

TEST_CASE("an invalid source is refused loudly — the strong "
          "guarantee, no partially linked clone ever escapes") {
    // A body whose loop references an edge that does not exist:
    // link_radial_edges would SILENTLY omit it (the audited
    // omission); the sanctioned copy refuses.
    b::BRepBody bad;
    bad.vertices.emplace_back(math::Vec3d{0.0, 0.0, 0.0});
    bad.edges.emplace_back(0, 0);
    bad.solid.shell_mut().set_closed(true);
    {
        b::Face face;
        b::Loop loop;
        loop.add_coedge(b::CoEdge{0, false});
        loop.add_coedge(b::CoEdge{999, true});  // out of range
        face.add_loop(std::move(loop));
        bad.solid.shell_mut().add_face(std::move(face));
    }
    CHECK_THROWS_AS(b::clone_body(bad), std::invalid_argument);

    // The refusal is a judgment of the SOURCE, not a poisoned well:
    // a valid body clones fine right after.
    auto good = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    auto dst = b::clone_body(*good);
    REQUIRE(dst != nullptr);
    check_structural(*dst, 2);
}

TEST_CASE("SceneNode cloning works: the copy's bodies are "
          "independent generations, and the assignment path cannot "
          "mint an unlinked body either") {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid =
        s.add_fillet_node("", b::FilletRecipe{"box#1:vertical2", kR});
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);

    // COPY CONSTRUCTION (the undo/redo, ParsedDocument and
    // command-stack road).
    scene::SceneNode copy = *f;
    REQUIRE(copy.fillet_body != nullptr);
    REQUIRE(copy.fillet_body != f->fillet_body);
    const std::set<const b::CoEdge*> src_storage =
        all_coedge_addresses(*f->fillet_body);
    const std::set<const b::CoEdge*> own =
        all_coedge_addresses(*copy.fillet_body);
    for (const b::CoEdge* ce : all_radial_pointers(*copy.fillet_body)) {
        CHECK(own.count(ce) == 1);
        CHECK(src_storage.count(ce) == 0);
    }
    check_structural(*copy.fillet_body, 2);

    // The edit that rebuilds the ORIGINAL whole (a new allocation,
    // m1 scenario 2's own law) leaves the copy's generation alone:
    // the copy keeps the old geometry while the node re-derives.
    const math::Vec3d old_pos =
        copy.fillet_body->vertices[0].position();
    (void)s.set_fillet_radius(fid, 0.3);
    CHECK(copy.fillet_body->vertices[0].position().x == old_pos.x);
    CHECK(copy.fillet_body->vertices[0].position().y == old_pos.y);
    CHECK(copy.fillet_body->vertices[0].position().z == old_pos.z);
    CHECK(copy.fillet_body->vertex_count() == 10);

    // ASSIGNMENT (the second sanctioned copy path).
    scene::SceneNode assigned;
    assigned = *f;
    REQUIRE(assigned.fillet_body != nullptr);
    const std::set<const b::CoEdge*> own_a =
        all_coedge_addresses(*assigned.fillet_body);
    const std::set<const b::CoEdge*> src2 =
        all_coedge_addresses(*f->fillet_body);
    for (const b::CoEdge* ce :
         all_radial_pointers(*assigned.fillet_body)) {
        CHECK(own_a.count(ce) == 1);
        CHECK(src2.count(ce) == 0);
    }
    check_structural(*assigned.fillet_body, 2);

    // THE ASYNC DISPLAY SNAPSHOT (the worker thread's own
    // independent generation — the migration's most safety-critical
    // consumer).
    const auto snap = scene::snapshot_display_source(*f);
    REQUIRE(snap.exact != nullptr);
    const std::set<const b::CoEdge*> snap_src =
        all_coedge_addresses(*f->fillet_body);
    for (const b::CoEdge* ce : all_radial_pointers(*snap.exact)) {
        CHECK(snap_src.count(ce) == 0);
    }
    check_structural(*snap.exact, 2);
}

TEST_CASE("cloning is deterministic: the same source clones to the "
          "same whole (counts, chi, names, radial incidence)") {
    auto src = make_fillet_body();
    REQUIRE(src != nullptr);
    auto a = b::clone_body(*src);
    auto c = b::clone_body(*src);
    REQUIRE(a != nullptr);
    REQUIRE(c != nullptr);
    CHECK(a->vertex_count() == c->vertex_count());
    CHECK(a->edge_count() == c->edge_count());
    CHECK(a->face_count() == c->face_count());
    CHECK(b::euler_characteristic(*a) == b::euler_characteristic(*c));
    CHECK(face_names(*a) == face_names(*c));
    for (std::size_t i = 0; i < a->edge_count(); ++i)
        CHECK(a->edges[i].coedges().size() ==
              c->edges[i].coedges().size());
    check_structural(*a, 2);
    check_structural(*c, 2);
}

TEST_CASE("the migrated subtract path stays correct: the separated "
          "tool returns the EXACT CLONE of the target — a closed "
          "generation whose radials are its own") {
    auto A = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    auto B = b::make_box({0.5, 0.5, 0.5}, FeatureId{2});
    REQUIRE(A != nullptr);
    REQUIRE(B != nullptr);
    // The tool translated far below the target: nothing is removed,
    // the migrated site answers clone_body(A).
    auto sep = b::exact_subtract(*A, *B, math::Vec3d{0.0, 0.0, -10.0},
                                 b::kSubtractTolerance,
                                 b::SubtractNaming{});
    REQUIRE(sep != nullptr);
    CHECK(sep->vertex_count() == A->vertex_count());
    CHECK(sep->edge_count() == A->edge_count());
    CHECK(sep->face_count() == A->face_count());
    // The exact copy is a CLONE, not an aliased copy: its radial
    // links live in its own storage (the pre-T1-m2 site relinked by
    // hand; the sanction now carries the guarantee).
    const std::set<const b::CoEdge*> a_storage =
        all_coedge_addresses(*A);
    for (const b::CoEdge* ce : all_radial_pointers(*sep))
        CHECK(a_storage.count(ce) == 0);
    check_structural(*sep, 2);
    // The pierce/removal paths (the tool copy bp included) are the
    // existing boolean suite's own green — those tests run through
    // the migrated clone in this very build.
}

}  // TEST_SUITE
