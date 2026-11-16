// tests/unit/brep/test_t1_m1_topology_lifecycle.cpp
//
// T1 m1 — THE TOPOLOGY LIFECYCLE CHARACTERIZATION (ADR-0044's own
// evidence base, the proposal this bite carries beside these pins).
//
// THE CAMPAIGN'S LAW: these tests pin the behavior that IS, never
// the behavior we wish existed. Every assertion carries beside it
// the OBSERVED behavior it pins — the declaration, not the hope.
// No test crosses the UB boundary: a handle into a dead body is
// never dereferenced here (that act is what the debug column's
// ASan/UBSan catches LOUDLY — the net for any future code path
// that holds raw handles across a body's replacement); the pins
// are the address-level facts that are deterministic on every
// column: aliasing, disjoint allocations, identity survival, and
// the sanctioned contract's answers.
//
// THE FOUR SCENARIOS (the brief's own list):
//   1. COPY        — BRepBody copied; handles of the SOURCE used
//                    on/against the copy.
//   2. MUTATION    — set_fillet_radius + rebuild; handles taken
//                    before the edit, read after.
//   3. DESTRUCTION — the body destroyed; the surviving handles'
//                    fate, and what the VALUES survive.
//   4. LOOP
//      REGENERATION — the fillet ring act absorbing a flank
//                    (ADR-0043 D5's Absorbed, never
//                    Born-sotto-falso): loop handles before, the
//                    re-derived loop after.
//
// THE MEASURED HEADLINES (the probe's own output, 2026-11-15):
//   * the copy's Edge::coedges_ ALIAS the source's loops — the one
//     shallow member of an otherwise deep copy — and
//     link_radial_edges(copy) closes it (the scene's own pattern);
//     [AMENDED in T1 m2, 2026-11-16 — ADR-0044 D3.4/D7-m2, accepted:
//     the raw copy is DELETED at the class and clone_body() is the
//     one sanctioned producer; scenario 1 re-pins the law below,
//     and the measured aliasing stays recorded here as the history
//     it now is — the pin updated in the bite that changed it, the
//     new measurement beside it, D5's own rule];
//   * the mutation replaces the body WHOLE (a fresh allocation,
//     built while the old is still alive); the NAME survives and
//     the feature re-runs ("reference_resolved"), raw handles do
//     not survive;
//   * the destruction kills the body with its node; the recipe —
//     a VALUE — re-derives the identical topology;
//   * the absorbed flank keeps its name and stamp while its loop
//     is re-derived whole: same coedge count, a COMPLETELY
//     different edge set (the measured {10,8,11,9} → {45,0,42,1}).
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/scene/scene.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace scene = CAD_0::scene;
namespace math = CAD_0::math;
namespace sdf = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;

namespace {

constexpr double kR = 0.5;

const b::Face* face_by_name(const b::BRepBody& body,
                            const std::string& name) {
    for (const auto& f : body.solid.shell().faces())
        if (f.name() == name) return &f;
    return nullptr;
}

// Every coedge address the body owns (its loops' own storage) —
// the membership witness for the aliasing and regeneration pins.
std::set<const b::CoEdge*> all_coedge_addresses(
    const b::BRepBody& body) {
    std::set<const b::CoEdge*> out;
    for (const auto& f : body.solid.shell().faces())
        for (const auto& lp : f.loops())
            for (const auto& ce : lp.coedges())
                out.insert(&ce);
    return out;
}

// The Cylinder-family face of a k=1 vertical fillet body (exactly
// one exists — the fillet's own flank), read through the family
// enum: no dynamic_cast, no string match (H1's typed dispatch).
const b::Face* cylinder_face(const b::BRepBody& body) {
    for (const auto& f : body.solid.shell().faces())
        if (f.surface() != nullptr &&
            f.surface()->family() == b::SurfaceFamily::Cylinder)
            return &f;
    return nullptr;
}

}  // namespace

TEST_SUITE("brep topology lifecycle (T1 m1)") {

// ---------------------------------------------------------------------------
// SCENARIO 1 — THE COPY. [AMENDED in T1 m2, 2026-11-16 — ADR-0044
// D3.4/D7-m2, accepted: the raw copy that measured as "one shallow
// member, aliasing the source" is DELETED at the class; the ONE
// sanctioned producer of an independent body is clone_body(). The
// pin updates in the bite that changed it, the new measurement
// beside the old (D5): what was "the copy aliases, the relink
// closes" is now "the copy does not exist; the clone is born
// closed".]
// ---------------------------------------------------------------------------
TEST_CASE("T1 m1 scenario 1 — the copy (as amended by T1 m2): the "
          "raw copy is GONE; clone_body is the one sanctioned "
          "producer, born closed") {
    auto src = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(src != nullptr);
    // OBSERVED: the factory links (every edge radial == 2, closed,
    // chi == 2) — the radial links are valid from birth.
    REQUIRE(b::is_closed_orientable(*src));
    REQUIRE(b::euler_characteristic(*src) == 2);
    for (const auto& e : src->edges) REQUIRE(e.coedges().size() == 2);

    // A raw handle into the SOURCE, held across the clone — the
    // m1-era witness, still meaningful: the clone must never
    // contain it.
    const b::CoEdge* const src_handle = src->edges[0].coedges()[0];
    const std::set<const b::CoEdge*> src_storage =
        all_coedge_addresses(*src);

    // THE LAW (T1 m2, ADR-0044 D3.4): BRepBody's copy and move are
    // DELETED at the class — compile-time rejected (the m2 suite
    // pins the four static_asserts) — and the ONE sanctioned
    // producer of an independent body is clone_body: the copy and
    // the relink in a single call.
    std::unique_ptr<b::BRepBody> dst = b::clone_body(*src);

    // OBSERVED AND PINNED: the clone's radial links resolve INSIDE
    // the clone — every one of them, from birth (the m1-era raw
    // copy aliased the source; the sanctioned clone never does).
    {
        const std::set<const b::CoEdge*> own =
            all_coedge_addresses(*dst);
        REQUIRE(own.size() == dst->edge_count() * 2);
        for (const auto& e : dst->edges)
            for (const b::CoEdge* ce : e.coedges())
                CHECK(own.count(ce) == 1);
    }
    // And NONE of them aliases the source's storage — the m1-era
    // measured aliasing (dst.edges[0].coedges()[0] == src_handle,
    // pinned here before T1 m2) is now impossible by construction.
    for (const auto& e : dst->edges)
        for (const b::CoEdge* ce : e.coedges())
            CHECK(src_storage.count(ce) == 0);
    CHECK(dst->edges[0].coedges()[0] != src_handle);

    // OBSERVED: the surface is SHARED between source and clone —
    // the same immutable analytic object (shared_ptr, by design:
    // analytic surfaces are values, never mutated in place).
    CHECK(dst->solid.shell().faces()[0].surface() ==
          src->solid.shell().faces()[0].surface());

    // OBSERVED: the address plan is untouched by the clone — ids
    // ride the copied vectors verbatim (ADR-0036 D1's split: the
    // address is the body's own bookkeeping).
    CHECK(dst->edges[0].id() == src->edges[0].id());
    CHECK(dst->vertex_count() == src->vertex_count());
    CHECK(dst->edge_count() == src->edge_count());
    CHECK(dst->face_count() == src->face_count());
    CHECK(b::is_closed_orientable(*dst));
    CHECK(b::euler_characteristic(*dst) == 2);

    // THE SOURCE DIES, THE CLONE STANDS: the m1-era hazard — the
    // copy's radial links dangling into the freed source — cannot
    // exist any more (no pointer of the clone ever pointed there).
    // The death of the source's storage changes nothing in the
    // clone; the check reads only the clone's own storage.
    src.reset();
    CHECK(b::is_closed_orientable(*dst));
    CHECK(b::euler_characteristic(*dst) == 2);
    CHECK(dst->edges[0].coedges().size() == 2);
}

// ---------------------------------------------------------------------------
// SCENARIO 2 — THE MUTATION. set_fillet_radius + the rebuild, the
// scene's own edit path (the m6 "silent radius" ledger's road).
// ---------------------------------------------------------------------------
TEST_CASE("T1 m1 scenario 2 — the mutation: set_fillet_radius "
          "rebuilds the body WHOLE; the name survives, the raw "
          "handles do not") {
    scene::Scene s;
    const auto box_id =
        s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f,
                                                   1.0f}));
    REQUIRE(box_id != 0);
    const auto fid = s.add_fillet_node(
        "", b::FilletRecipe{"box#1:vertical2", kR});
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);

    // OBSERVED: the k=1 walk's counts (the formula 8+2k / 12+3k /
    // 6+k at k = 1 — the same numbers the G5 chain's first golden
    // pins).
    REQUIRE(f->fillet_body->vertex_count() == 10);
    REQUIRE(f->fillet_body->edge_count() == 15);
    REQUIRE(f->fillet_body->face_count() == 7);

    // MY OWN WITNESS of the pre-edit state: a deep copy, closed
    // the sanctioned way (the scenario-1 pattern applied where the
    // scene itself applies it). It keeps the old body's world
    // alive for the after-the-edit comparisons — every address
    // below is backed by a live allocation.
    auto before = b::clone_body(*f->fillet_body);

    // HANDLES TAKEN BEFORE THE EDIT:
    const b::BRepBody* const old_body = f->fillet_body.get();
    const b::CoEdge* const old_ce =
        &f->fillet_body->solid.shell().faces()[0]
             .loops()[0]
             .coedges()[0];
    const b::Face* const old_flank =
        cylinder_face(*f->fillet_body);
    REQUIRE(old_flank != nullptr);
    // The flank's radius, read through the surface's own bounds
    // (the analytic cylinder's bbox is the 2R square of its
    // axis-plane cross-section).
    const double old_extent =
        old_flank->surface()->bounds().max.x -
        old_flank->surface()->bounds().min.x;
    // The SURFACE OBJECT's address, taken while the old body is
    // ALIVE — the raw surface pointer stays valid across the old
    // body's death (the witness copy below holds the shared_ptr
    // that owns it), while the FACE POINTER does not (the Face
    // lives in the body's own vector: it dies with the body). The
    // first draft of this very test learned the difference from
    // the debug column's own net: comparing through old_flank
    // AFTER the edit was a heap-use-after-free ASan caught on the
    // spot — the characterization campaign's first catch, and the
    // net working exactly as designed.
    const b::SurfaceGeometry* const old_surf =
        old_flank->surface();

    // THE EDIT — the mutation under test.
    const std::string answer = s.set_fillet_radius(fid, 0.25);

    // OBSERVED AND PINNED: THE NAME SURVIVED — the sanctioned
    // handle is the recipe's reference string; the edit path
    // re-resolved it and the feature re-ran, the m6 ledger's own
    // "reference_resolved" (never a guess, never a silent skip).
    CHECK(answer == "reference_resolved");

    // Re-fetch the node (the discipline this very test pins: the
    // scene owns the node; the test never holds scene-internal
    // pointers across the edit longer than it must).
    f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);

    // OBSERVED AND PINNED: the body is a NEW ALLOCATION — the
    // rebuild constructs the new body while the old one is still
    // owned by the node (the old is released only at the
    // assignment), so the two addresses are distinct BY
    // CONSTRUCTION, deterministically, on every column.
    CHECK(f->fillet_body.get() != old_body);
    // And the new body's own coedges live in the new allocation:
    // the pre-edit handle's address is not the new body's storage
    // (two disjoint allocations; the handle is never dereferenced
    // — see the declaration below).
    CHECK(&f->fillet_body->solid.shell().faces()[0]
               .loops()[0]
               .coedges()[0] != old_ce);

    // OBSERVED: the counts are RADIUS-INVARIANT (the formula's
    // own — the walk's arithmetic does not move with r in band).
    CHECK(f->fillet_body->vertex_count() == 10);
    CHECK(f->fillet_body->edge_count() == 15);
    CHECK(f->fillet_body->face_count() == 7);

    // OBSERVED AND PINNED: the geometry DID change — the new
    // flank's radius is the edited 0.25 (the bounds' extent 2R),
    // while the witness snapshot keeps the pre-edit 0.5.
    const b::Face* const new_flank =
        cylinder_face(*f->fillet_body);
    REQUIRE(new_flank != nullptr);
    const double new_extent =
        new_flank->surface()->bounds().max.x -
        new_flank->surface()->bounds().min.x;
    CHECK(new_extent == doctest::Approx(0.5));
    CHECK(old_extent == doctest::Approx(1.0));

    // OBSERVED AND PINNED: the rebuild re-derives the geometry
    // WHOLE — a fresh surface object, not a mutation of the old
    // one (the witness keeps the old surface object alive through
    // its shared_ptr; the addresses differ — read through the
    // pre-edit surface pointer, never through the dead Face).
    CHECK(new_flank->surface() != old_surf);
    // ...and the old world is intact in the witness (the deep
    // copy's fidelity: the snapshot's flank still reads the
    // pre-edit radius).
    CHECK(cylinder_face(*before)->surface()->bounds().max.x -
              cylinder_face(*before)->surface()->bounds().min.x ==
          doctest::Approx(1.0));

    // THE DECLARED FATE OF THE RAW HANDLES, documented not
    // dereferenced: old_body and old_ce point into the freed
    // pre-edit body — using them is the UB the debug column's
    // ASan catches loudly; the release column would read freed
    // memory silently. The scene's own discipline (this edit's
    // own implementation) holds handles as NAMES and tavole —
    // values — never as addresses across the rebuild. ADR-0044's
    // contract writes this discipline down for every future path.
}

// ---------------------------------------------------------------------------
// SCENARIO 3 — THE DESTRUCTION. The body dies with its node; the
// recipe — a VALUE — re-derives the identical topology.
// ---------------------------------------------------------------------------
TEST_CASE("T1 m1 scenario 3 — the destruction: the body dies with "
          "its node; the recipe re-derives the identical topology, "
          "the raw handles dangle") {
    scene::Scene s;
    const auto box_id =
        s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f,
                                                   1.0f}));
    REQUIRE(box_id != 0);
    // The SANCTIONED handle — the recipe's own reference string, a
    // VALUE the test holds (the recipe is the source of truth;
    // the body is cache, ADR-0027 D3's law).
    const b::FilletRecipe recipe{"box#1:vertical2", kR};
    const auto fid = s.add_fillet_node("", recipe);
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);
    // A raw handle COULD be taken here (a coedge of the body's
    // first loop, the scenario-2 form); the pin below is the
    // VALUE side's survival, so the handle's fate is declared in
    // the comment rather than pinned — see the declaration after
    // the destruction.

    // THE DESTRUCTION — the node removed, the body (a unique_ptr
    // member) freed with it.
    REQUIRE(s.remove_node(fid));
    // OBSERVED AND PINNED: the node is gone; there is no body to
    // reach — the ownership direction was total (the body owned
    // the shell owned the faces owned the loops owned the
    // coedges: with the body, all of it died at once).
    CHECK(s.get_node(fid) == nullptr);

    // THE DECLARED FATE OF THE RAW HANDLES, documented not
    // dereferenced: any coedge pointer taken from the removed
    // body's loops now points into freed storage. Any
    // dereference is the UB the debug column's ASan/UBSan kills
    // LOUDLY (use-after-free: the whole suite aborts, the failure
    // deterministic); on release it would read silently freed
    // memory. This test never takes one across the death — the
    // pin below is the VALUE side's survival.

    // OBSERVED AND PINNED: the VALUES survive — the recipe's
    // string needed no body to exist, and re-creating the node
    // re-derives the SAME topology, bit-deterministic (the counts
    // of the k=1 walk, the same numbers the fresh node read
    // before the destruction; the derivation is a pure function
    // of the recipe, ADR-0036 D1's own law).
    const auto fid2 = s.add_fillet_node("", recipe);
    CHECK(fid2 != fid);  // a new node id — the id counter moved
    const scene::SceneNode* f2 = s.get_node(fid2);
    REQUIRE(f2 != nullptr);
    REQUIRE(f2->fillet_body != nullptr);
    CHECK(f2->fillet_body->vertex_count() == 10);
    CHECK(f2->fillet_body->edge_count() == 15);
    CHECK(f2->fillet_body->face_count() == 7);
    CHECK(b::euler_characteristic(*f2->fillet_body) == 2);
    CHECK(b::is_closed_orientable(*f2->fillet_body));
    // The re-derived flank reads the recipe's radius (the value's
    // own round trip).
    const b::Face* const flank2 =
        cylinder_face(*f2->fillet_body);
    REQUIRE(flank2 != nullptr);
    CHECK(flank2->surface()->bounds().max.x -
              flank2->surface()->bounds().min.x ==
          doctest::Approx(1.0));

    // OBSERVED: the re-derived body is a fresh allocation whose
    // storage need not relate to the dead one's addresses (the
    // allocator may or may not reuse the freed block — the test
    // asserts nothing about that; the identity that survives is
    // the DERIVED STATE, never the allocation).
}

// ---------------------------------------------------------------------------
// SCENARIO 4 — THE LOOP REGENERATION. ADR-0043 D5's Absorbed: the
// ring act re-windows the surviving vertical flank; the loop is
// re-derived WHOLE (the measured edge-set replacement), never a
// Born-sotto-falso.
// ---------------------------------------------------------------------------
TEST_CASE("T1 m1 scenario 4 — the loop regeneration: the ring act "
          "re-windows the ABSORBED flank; the loop is re-derived "
          "whole, the identity survives") {
    // The chain: the box, the four verticals (the formula's
    // k = 1..4 states), then the ring act (the m4 fixture's own
    // road — one act, indivisible).
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(box != nullptr);
    b::BRepBody* cur = box.get();
    std::vector<std::unique_ptr<b::BRepBody>> steps;
    for (int k = 0; k < 4; ++k) {
        const ShapeId v{FeatureId{1},
                        SubshapeRef{}.push(
                            static_cast<std::uint32_t>(8 + k)),
                        {}};
        const b::FilletNaming ctx{
            FeatureId{static_cast<std::uint64_t>(2 + k)},
            &n::kBoxNaming};
        steps.push_back(b::fillet_edge(*cur, v, kR, ctx));
        REQUIRE(steps.back() != nullptr);
        cur = steps.back().get();
    }
    // OBSERVED: the k=4 state (the verticals' final pass).
    REQUIRE(cur->vertex_count() == 16);
    REQUIRE(cur->edge_count() == 24);
    REQUIRE(cur->face_count() == 10);

    // THE FLANK — born at the first vertical (fil#2), the face
    // the ring act will ABSORB (re-window, keep the identity).
    const std::string flank_name =
        "fil#2:flank[box#1:front × box#1:left]";
    const b::Face* const before_flank =
        face_by_name(*steps[3], flank_name);
    REQUIRE(before_flank != nullptr);
    // HANDLES BEFORE THE ACT: the flank's loop storage addresses
    // and its boundary edge ids.
    REQUIRE(before_flank->loops().size() == 1);
    const b::Loop& before_loop = before_flank->loops()[0];
    std::vector<b::EdgeId> before_edges;
    for (const auto& ce : before_loop.coedges())
        before_edges.push_back(ce.edge());
    // OBSERVED: the flank's loop pre-act carries 4 coedges (2
    // verticals + 2 capitel arcs), every one p-curved.
    REQUIRE(before_edges.size() == 4);
    for (const auto& ce : before_loop.coedges())
        CHECK(ce.p_curve() != nullptr);

    // THE ACT — the ring + the corners, indivisible.
    const b::FilletNaming actx{FeatureId{6}, &n::kBoxNaming};
    auto after = b::fillet_ring_act(*cur, kR, actx);
    REQUIRE(after != nullptr);
    // OBSERVED AND PINNED: the act's counts (D7.1's own) — the
    // formula closed, the body a NEW whole.
    REQUIRE(after->vertex_count() == 24);
    REQUIRE(after->edge_count() == 48);
    REQUIRE(after->face_count() == 26);
    REQUIRE(b::inner_loop_count(*after) == 0);
    REQUIRE(b::euler_characteristic(*after) == 2);
    for (const auto& e : after->edges)
        REQUIRE(e.coedges().size() == 2);

    // OBSERVED AND PINNED — THE IDENTITY SURVIVED (Absorbed,
    // never Born-sotto-falso): the flank is in the result, under
    // the SAME NAME and the SAME STAMP (the m4 pin's own
    // clause, ADR-0043 D6.3).
    const b::Face* const after_flank =
        face_by_name(*after, flank_name);
    REQUIRE(after_flank != nullptr);
    CHECK(after_flank->shape_id() == before_flank->shape_id());
    // The act's born signature: the 8 corner spheres (the
    // fil#6:corner faces), the ring flanks beside them.
    {
        std::size_t corners = 0, ring_flanks = 0;
        for (const auto& fc : after->solid.shell().faces()) {
            if (fc.name().find("fil#6:corner[") == 0) ++corners;
            if (fc.name().find("fil#6:flank[") == 0) ++ring_flanks;
        }
        CHECK(corners == 8);
        CHECK(ring_flanks == 8);
    }

    // OBSERVED AND PINNED — THE LOOP WAS RE-DERIVED WHOLE: the
    // absorbed flank's loop keeps its SHAPE (one loop, 4
    // coedges, every one p-curved — the re-window is a
    // replacement of the boundary, not a reshaping of the face's
    // structure)...
    REQUIRE(after_flank->loops().size() == 1);
    const b::Loop& after_loop = after_flank->loops()[0];
    REQUIRE(after_loop.coedges().size() == 4);
    for (const auto& ce : after_loop.coedges())
        CHECK(ce.p_curve() != nullptr);
    // ...but its EDGE SET IS ENTIRELY REPLACED: no boundary edge
    // of the pre-act flank survives into the re-derived loop
    // (the measured {10, 8, 11, 9} → {45, 0, 42, 1}: the
    // capitel arcs died Vanished, the coaxial arcs were Born,
    // and even the re-spanned verticals carry new ids — the
    // addresses are the walk's own, renumbered from the new
    // whole).
    std::size_t shared = 0;
    for (const auto& ce : after_loop.coedges())
        for (const auto& eid : before_edges)
            if (ce.edge() == eid) ++shared;
    CHECK(shared == 0);
    // And the PRE-ACT loop's storage addresses are nowhere in
    // the new body (both bodies are alive here — the membership
    // witness is exact, not an allocator artifact).
    {
        const std::set<const b::CoEdge*> after_own =
            all_coedge_addresses(*after);
        for (const auto& ce : before_loop.coedges())
            CHECK(after_own.count(&ce) == 0);
    }

    // THE DECLARED FATE OF THE PRE-ACT HANDLES, documented not
    // dereferenced once the before-body is gone: this test keeps
    // the before-body alive (the steps vector owns it), so the
    // pins above are exact; in the SCENE's own road the chain
    // releases each step as the next is built (cur = fillet_edge
    // — scene.hpp's own pattern), and any handle into a released
    // step dangles exactly as scenarios 2 and 3 declared. The
    // names and stamps — the values — are what the chain's
    // ledger reads, and they survived.
}

}  // TEST_SUITE("brep topology lifecycle (T1 m1)")
