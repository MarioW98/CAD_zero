// tests/unit/brep/test_t1_followup_ownership_lookup.cpp
//
// T1 follow-up — THE PORTABLE OWNERSHIP LOOKUP, CHARACTERIZED
// (2026-11-16, the register's T row): faces_around_edge's membership
// test is OBJECT-ADDRESS EQUALITY, never a relational range. The old
// `ce >= base && ce < end` compared pointers from possibly different
// allocations — unspecified ordering, and a foreign pointer whose
// numeric address fell inside a loop's storage would have been
// CLAIMED as owned. These tests pin the cured contract against an
// INDEPENDENT ORACLE: a face is incident to an edge iff one of its
// co-edges REFERENCES that edge by id (the topological definition —
// no pointers involved). Where the oracle and the query agree, the
// pointer walk is proven to resolve exactly the topology's own
// incidence.
//
// THE EIGHT CASES (the bite's own list): the box edge's two faces;
// faceted edges; a body with inner loops; deterministic multi-query;
// a foreign LIVE co-edge never claimed; out-of-range ids; after
// clone_body; after link_radial_edges again.
//
// Lifetime discipline (the suite's own law, T1 m1's): no test
// dereferences a pointer whose target has died — foreign addresses
// are compared as VALUES, and every body that lends an address to a
// test stays alive until that test's last assertion.
#include <doctest/doctest.h>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace b = CAD_0::brep;
namespace bridge = CAD_0::bridge;
namespace math = CAD_0::math;
namespace sdf = CAD_0::sdf;
namespace scene = CAD_0::scene;
using CAD_0::geometry::FeatureId;

namespace {

// THE ORACLE: the faces incident to an edge by pure id reference —
// one pass over the face tree, no radial pointers, no ownership
// questions. Face order, first referencing loop per face.
std::vector<b::FaceId> incident_by_reference(
    const b::BRepBody& body, b::EdgeId eid) {
    std::vector<b::FaceId> out;
    const auto& faces = body.solid.shell().faces();
    for (b::FaceId fid = 0; fid < faces.size(); ++fid) {
        for (const auto& lp : faces[fid].loops()) {
            bool refs = false;
            for (const auto& co : lp.coedges())
                if (co.edge() == eid) { refs = true; break; }
            if (refs) { out.push_back(fid); break; }
        }
    }
    return out;
}

std::vector<b::FaceId> sorted(std::vector<b::FaceId> v) {
    std::sort(v.begin(), v.end());
    return v;
}

// The k=1 fillet body on a unit box (the T1 suite's own fixture).
std::unique_ptr<b::BRepBody> make_fillet_body() {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid =
        s.add_fillet_node("", b::FilletRecipe{"box#1:vertical2", 0.5});
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->fillet_body != nullptr);
    return b::clone_body(*f->fillet_body);
}

}  // namespace

TEST_SUITE("brep ownership lookup (T1 follow-up)") {

TEST_CASE("a box edge answers its two incident faces, proven by the "
          "reference oracle") {
    auto body = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(body != nullptr);
    REQUIRE(body->edge_count() == 12);
    for (b::EdgeId eid = 0; eid < body->edge_count(); ++eid) {
        const auto faces = b::faces_around_edge(*body, eid);
        REQUIRE(faces.size() == 2);
        CHECK(faces[0] != faces[1]);
        // The oracle agreement: the pointer walk resolves exactly the
        // topology's own id-reference incidence.
        CHECK(sorted(faces) == sorted(incident_by_reference(*body, eid)));
    }
}

TEST_CASE("faceted edges answer through the same equality walk — "
          "the bridge's 26,504-triangle world") {
    auto body = bridge::sdf_to_brep_faceted(
        sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    REQUIRE(body != nullptr);
    REQUIRE(body->face_count() == 26504);
    REQUIRE(body->edge_count() == 39756);
    // Deterministic sample: the first, the last, every 500th.
    for (b::EdgeId eid = 0; eid < body->edge_count(); eid += 500) {
        const auto faces = b::faces_around_edge(*body, eid);
        REQUIRE(faces.size() == 2);
        CHECK(faces[0] != faces[1]);
        CHECK(sorted(faces) == sorted(incident_by_reference(*body, eid)));
    }
    const auto last =
        b::faces_around_edge(*body, b::EdgeId{body->edge_count() - 1});
    REQUIRE(last.size() == 2);
    CHECK(sorted(last) ==
          sorted(incident_by_reference(
              *body, b::EdgeId{body->edge_count() - 1})));
}

TEST_CASE("edges of a body with inner loops answer — the cieco, "
          "one face carrying the hole's inner loop") {
    // The blind door's own recipe (test_g2_unit2_m4's BlindGuide): a
    // box target, the extruded CIRCLE profile (the seam-wired family
    // the landed door consumes — a chain-profile tool terminating
    // strictly inside is REFUSED by the class's own contract), prism
    // height 2 at offset -0.5 -> the tool crosses the top face alone
    // and its bottom cap sits inside: a blind hole, the pierced top
    // face carrying an INNER loop (H = 1).
    auto target = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    std::vector<sdf::SketchChain> chains;
    chains.push_back(sdf::SketchChain{
        {sdf::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
    auto tool = b::make_prism(chains, 2.0f, +1, FeatureId{2});
    auto body = b::exact_subtract(*target, *tool,
                                  math::Vec3d{0.0, 0.0, -0.5},
                                  b::kSubtractTolerance,
                                  b::SubtractNaming{});
    REQUIRE(body != nullptr);
    REQUIRE(b::is_closed_orientable(*body));

    // The face with two loops — the pierced top.
    const auto& faces = body->solid.shell().faces();
    std::vector<b::FaceId> two_loop_faces;
    for (b::FaceId fid = 0; fid < faces.size(); ++fid)
        if (faces[fid].loops().size() == 2)
            two_loop_faces.push_back(fid);
    REQUIRE(two_loop_faces.size() == 1);
    const b::FaceId pierced = two_loop_faces[0];

    // Every edge the pierced face references — outer rim, inner rim
    // (the hole's boundary), the shared seams — answers two faces,
    // oracle-proven.
    std::vector<b::EdgeId> face_edges;
    for (const auto& lp : faces[pierced].loops())
        for (const auto& co : lp.coedges())
            face_edges.push_back(co.edge());
    // The outer rim's 4 co-edges + the hole's boundary walked as the
    // seam-wired pair of π-sweep arcs (the factory's own circle
    // convention) = 6 on this body, measured.
    REQUIRE(face_edges.size() >= 6);
    for (const b::EdgeId eid : face_edges) {
        const auto ans = b::faces_around_edge(*body, eid);
        REQUIRE(ans.size() == 2);
        CHECK(sorted(ans) == sorted(incident_by_reference(*body, eid)));
    }
    // The inner rim specifically: every edge of the pierced face's
    // SECOND loop (the hole's boundary, born after the outer) must
    // answer with the pierced face among its two.
    for (const auto& co : faces[pierced].loops()[1].coedges()) {
        const auto ans = b::faces_around_edge(*body, co.edge());
        REQUIRE(ans.size() == 2);
        bool sees_owner = false;
        for (const b::FaceId f : ans)
            sees_owner = sees_owner || f == pierced;
        CHECK(sees_owner);
    }
}

TEST_CASE("multiple edges queried deterministically — the whole "
          "fillet body, twice, identical answers") {
    auto body = make_fillet_body();
    REQUIRE(body != nullptr);
    REQUIRE(body->edge_count() == 15);
    std::vector<std::vector<b::FaceId>> first, second;
    for (b::EdgeId eid = 0; eid < body->edge_count(); ++eid)
        first.push_back(b::faces_around_edge(*body, eid));
    for (b::EdgeId eid = 0; eid < body->edge_count(); ++eid)
        second.push_back(b::faces_around_edge(*body, eid));
    REQUIRE(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        REQUIRE(first[i].size() == 2);
        CHECK(first[i] == second[i]);
        CHECK(sorted(first[i]) ==
              sorted(incident_by_reference(*body,
                                           static_cast<b::EdgeId>(i))));
    }
}

TEST_CASE("a valid CoEdge address from ANOTHER live body is never "
          "claimed as local ownership — the portability regression") {
    auto a = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    auto other = b::make_box({1.0, 1.0, 1.0}, FeatureId{2});
    REQUIRE(a != nullptr);
    REQUIRE(other != nullptr);

    // The pre-registration truth: edge 0 answers its two own faces.
    const auto before = b::faces_around_edge(*a, 0);
    REQUIRE(before.size() == 2);

    // A LIVE co-edge of the OTHER body — a perfectly valid object,
    // registered by address into THIS body's edge-0 radial list.
    // The old range test could have claimed it whenever its numeric
    // address happened to fall inside one of a's loop storages (a
    // heap-layout lottery); equality never claims it.
    const b::CoEdge* const foreign =
        &other->solid.shell().faces()[0].loops()[0].coedges()[0];
    a->edges[0].add_coedge(foreign);
    REQUIRE(a->edges[0].coedges().size() == 3);

    const auto after = b::faces_around_edge(*a, 0);
    // The foreign pointer resolves to NOTHING: the answer is exactly
    // the two own faces it was before the registration.
    REQUIRE(after.size() == 2);
    CHECK(after == before);
    CHECK(sorted(after) == sorted(incident_by_reference(*a, 0)));

    // The other body stays alive to the last assertion: its co-edge
    // was never dereferenced through a's topology — only compared.
}

TEST_CASE("out-of-range EdgeIds answer empty — the boundary, the "
          "far, and the maximal") {
    auto body = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(body != nullptr);
    CHECK(b::faces_around_edge(*body, body->edge_count()).empty());
    CHECK(b::faces_around_edge(*body, 9999).empty());
    CHECK(b::faces_around_edge(
              *body, std::numeric_limits<b::EdgeId>::max())
              .empty());
}

TEST_CASE("after clone_body() the clone answers its own faces — "
          "and the original's death changes nothing") {
    auto original = make_fillet_body();
    REQUIRE(original != nullptr);
    auto clone = b::clone_body(*original);
    REQUIRE(clone != nullptr);

    std::vector<std::vector<b::FaceId>> answers;
    for (b::EdgeId eid = 0; eid < clone->edge_count(); ++eid) {
        const auto ans = b::faces_around_edge(*clone, eid);
        REQUIRE(ans.size() == 2);
        CHECK(sorted(ans) == sorted(incident_by_reference(*clone, eid)));
        answers.push_back(ans);
    }

    // The original dies; the clone's answers are unchanged (its
    // radial pointers never pointed there — the pointer VALUES are
    // compared, never dereferenced, so the death is unobservable
    // here by construction).
    original.reset();
    for (b::EdgeId eid = 0; eid < clone->edge_count(); ++eid)
        CHECK(b::faces_around_edge(*clone, eid) ==
              answers[eid]);
}

TEST_CASE("after link_radial_edges() runs again the answers are "
          "identical — the relink is idempotent, the query with it") {
    auto body = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(body != nullptr);
    std::vector<std::vector<b::FaceId>> before;
    for (b::EdgeId eid = 0; eid < body->edge_count(); ++eid)
        before.push_back(b::faces_around_edge(*body, eid));

    b::link_radial_edges(*body);
    b::link_radial_edges(*body);

    for (b::EdgeId eid = 0; eid < body->edge_count(); ++eid)
        CHECK(b::faces_around_edge(*body, eid) == before[eid]);
}

}  // TEST_SUITE
