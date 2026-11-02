// tests/unit/brep/test_g2_unit2_m3_crossed.cpp
//
// G2 UNIT 2, m3 — LO SBORDO: the crossed corner, the outer loop
// re-authored, joint(i) born, the golden 12/18/8 (ADR-0038 D1.2/D2/
// D3/D5, the numbers derived there and pinned HERE before the code —
// the naive counterproof transformed: the same call that the CROSSED
// pin (:952's family) refused yesterday answers today, and the wall
// becoming a door is the discrimination).
//
// THE GUIDE CASE (D5): the box [-1,1]^3, the tool the square prism
// footprint [0.5,1.5]x[0.5,1.5] at the (+X,+Y) corner of the top
// face, z in [-1.5, 1.5] through. The removed corner column:
// [0.5,1]x[0.5,1]x[-1,1], volume 0.5.
//
// THE DERIVED COUNTS, reproduced here as the test's own body (the
// derivation VISIBLE, never the dry number):
//
//   * VERTICES: the box's 8 -> 6 (the (1,1) column's two corners
//     die — no face contains them: the top/bottom faces lose the
//     corner, the right/back faces shrink past them) + the born six
//     (the exits (1,0.5,+-1) and (0.5,1,+-1), the inner joint
//     (0.5,0.5,+-1)): V = 6 + 6 = 12.
//   * EDGES: the box's 12 -> 11 (the (1,1) vertical dies; the +X and
//     +Y horizontals at both quotas are absorbed — re-spanned; the
//     rest kept) + the born six (the four notch rims — the pierced
//     faces x the tool's walls, the isect grammar — and the two exit
//     verticals — the side faces x the tool's walls, the plane x
//     plane entry, the level ABSENT) + the absorbed joint vertical
//     (the tool's joint(3), re-timed to [-1,1]): E = 11 + 6 + 1 = 18.
//     The co-edge incidence: 36 co-edges over the eight faces' loops
//     / 2 = 18 — every edge exactly two opposite co-edges.
//   * FACES: the box's 6 (top/bottom/right/back absorbed — the outer
//     loops re-authored; left/front kept) + the two surviving walls
//     (lat(0) the y=0.5 wall re-windowed to x in [0.5,1]; lat(3) the
//     x=0.5 wall re-windowed to y in [0.5,1]): F = 8. The tool's
//     lat(1)/lat(2) and both caps vanish (cut away outside).
//   * H: 0 — the pierced faces are L-shaped, simply connected; the
//     re-authored outer loop of the top face, walked in full:
//     (-1,-1) -> (1,-1) [kept] -> (1,0.5) [absorbed, re-spanned] ->
//     (0.5,0.5) [born: isect[box#1:top x ext#3:lat(0)]@hi] ->
//     (0.5,1) [born: isect[box#1:top x ext#3:lat(3)]@hi] -> (-1,1)
//     [absorbed, re-spanned] -> (-1,-1) [kept] — six edges, closed.
//   * THE CHI IDENTITY: chi = 12 - 18 + 8 - 0 = 2; the box's own
//     chi = 2 — the dent family (Delta-chi = 0), genus 0.
//   * THE VOLUME: abc - (the corner column) = 8 - 0.5 = 7.5.
//
// THE NUMBERING (the walk table this morso pins — the box's
// survivors keep their relative order, the dead column dropped, the
// born appended in the notch's own order):
//
//   vertices : 0..5 = the box's v0,v1,v3,v4,v5,v7 (compacted);
//              6/7 = the exit (1,0.5) at z_bot/z_top; 8/9 = the
//              joint (0.5,0.5); 10/11 = the exit (0.5,1).
//   edges    : 0..10 = the box's e0..e9, e11 (compacted, e10 dead);
//              11 = the joint vertical (8 -> 9, the absorbed
//              ext#3:joint(3)); 12 = the rim on lat(0) at z_bot
//              (8 -> 6); 13 = the rim on lat(3) at z_bot (10 -> 8);
//              14 = the rim on lat(0) at z_top (7 -> 9); 15 = the
//              rim on lat(3) at z_top (9 -> 11); 16 = the exit
//              vertical at (1,0.5) (6 -> 7); 17 = the exit vertical
//              at (0.5,1) (10 -> 11).
//   faces    : 0..5 = the box's own; 6 = lat(0); 7 = lat(3) (the
//              surviving walls in the tool's canonical walk order).
//
// THE RE-AUTHORED LOOPS (the sequences the output gate reads — every
// edge exactly two OPPOSITE co-edges, the table verified by hand and
// pinned by the XOR check below):
//
//   top    (1): [4 F, 5 F, 14 F, 15 F, 6 F, 7 F]
//   bottom (0): [3 R, 2 R, 13 F, 12 F, 1 R, 0 R]
//   right  (5): [1 F, 16 F, 5 R, 9 R]
//   back   (3): [2 F, 10 F, 6 R, 17 R]     (10 = the box's e11)
//   left   (4): [3 F, 8 F, 7 R, 10 R]
//   front  (2): [0 F, 9 F, 4 R, 8 R]
//   lat(0) (6): [11 F, 14 R, 16 R, 12 R]
//   lat(3) (7): [17 F, 15 R, 11 R, 13 R]
//
// THE NAMES (D3): the notch rims sub#4:isect[box#1:top x
// ext#3:lat(0)]@hi and [box#1:top x ext#3:lat(3)]@hi (and the @lo
// mirrors on box#1:bottom); the exit edges sub#4:isect[box#1:right x
// ext#3:lat(0)] and [box#1:back x ext#3:lat(3)] — the UNLEVELLED
// isect row, the level ABSENT (the pair's planes ARE the line); the
// joint vertical ext#3:joint(3) — THE VOCABULARY ROW THIS UNIT PAYS
// (ADR-0036's morse registers' "yet", redeemed); the walls inherit
// the tool's own lat(0)/lat(3); the absorbed faces keep the
// factory's names.
//
// THE TAVOLA (D3): 64 rows — 38 result (6 kept + 6 born vertices; 7
// kept + 5 absorbed + 6 born edges; 2 kept + 6 absorbed faces) + 26
// vanished (A's corner column 3; the tool's 23), the two-sided
// closure green.
//
// THE ADDITIVITY (D1.4): the G1 suite, the coplanar door's goldens
// and THE BLIND WALL'S PINS (test_g2_unit2_blind_wall.cpp — the wall
// stays red-demonstrated, its refusal honest) all green WITHOUT a
// re-pin: they run beside this file in the same binary, and the
// still-refused families keep their refusals (the crossing circle's
// "CROSSES", the tangential byte-identical, the blind's wall).
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The guide case's chain (the D1 chain the m4 tests practice):
// box#1 -> ext#3 -> sub#4.
struct CrossedGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;   // z in [0,3], offset -1.5
    Vec3d offset{0.0, 0.0, -1.5};
    // The naming context (the named mode) — the unnamed test builds
    // its own call.
    b::SubtractNaming named{};
};

std::vector<s::SketchChain> square_chains_corner() {
    // The square [0.5,1.5]^2, the CCW walk: entity 0 = the bottom
    // (y=0.5) edge east, 1 = the right (x=1.5) north, 2 = the top
    // (y=1.5) west, 3 = the left (x=0.5) south — the tool's joints
    // J_0..J_3 = (1.5,0.5), (1.5,1.5), (0.5,1.5), (0.5,0.5).
    return {s::SketchChain{{
        s::sketch_segment({0.5f, 0.5f}, {1.5f, 0.5f}, true),
        s::sketch_segment({1.5f, 0.5f}, {1.5f, 1.5f}, true),
        s::sketch_segment({1.5f, 1.5f}, {0.5f, 1.5f}, true),
        s::sketch_segment({0.5f, 1.5f}, {0.5f, 0.5f}, true),
    }}};
}

CrossedGuide crossed_guide() {
    CrossedGuide g;
    g.box = b::make_box({1.0, 1.0, 1.0},
                        CAD_0::geometry::FeatureId{1});
    g.tool = b::make_prism(square_chains_corner(), 3.0f, +1,
                           CAD_0::geometry::FeatureId{3});
    g.named = b::SubtractNaming{
        CAD_0::geometry::FeatureId{4},
        &b::naming::kBoxNaming,
        &b::naming::kExtNaming};
    return g;
}

// The digests (the m4 oracle's shape: "<address>|<name>\n").
std::string name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_face(body, i).name;
        out += '\n';
    }
    return out;
}

std::string edge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

// Every p-curve of a face's loop, chained endpoint-to-endpoint in the
// domain (the DIRECTION convention's own closure discipline — the
// unit-4 helper, replicated).
bool pcurves_chain(const b::BRepBody& body, const b::Face& face,
                   std::size_t loop_index) {
    const auto& loop = face.loops()[loop_index];
    const std::size_t n = loop.coedges().size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto* pc = loop.coedges()[i].p_curve();
        if (pc == nullptr) return false;
        const auto* next = loop.coedges()[(i + 1) % n].p_curve();
        if (next == nullptr) return false;
        Vec2d end{}, next_start{};
        if (pc->kind == b::PCurve::Kind::Segment) {
            end = pc->segment.b;
        } else if (pc->kind == b::PCurve::Kind::Arc) {
            end = b::pcurve_arc_end_point(pc->arc);
        } else {
            return false;
        }
        if (next->kind == b::PCurve::Kind::Segment) {
            next_start = next->segment.a;
        } else if (next->kind == b::PCurve::Kind::Arc) {
            next_start = b::pcurve_arc_start_point(next->arc);
        } else {
            return false;
        }
        if (std::abs(end.x - next_start.x) > 1e-9 ||
            std::abs(end.y - next_start.y) > 1e-9)
            return false;
    }
    return true;
}

// One expected tavola row (the m4 golden's own shape).
struct ExpectedRow {
    b::Descent relation{};
    b::EntityKind entity{};
    std::uint64_t address{};
    bool has_operand{};
    std::uint8_t operand{};
    b::EntityKind operand_kind{};
    std::uint64_t operand_address{};
    const char* name{};
    const char* cause{};
};

bool row_matches(const b::DescentRow& row, const ExpectedRow& want) {
    if (row.relation != want.relation) return false;
    if (row.entity != want.entity) return false;
    if (row.address != want.address) return false;
    if (row.name != want.name) return false;
    if (want.cause == nullptr) {
        if (!row.cause.empty()) return false;
    } else if (row.cause != want.cause) {
        return false;
    }
    if (want.has_operand) {
        if (row.operands.size() != 1) return false;
        if (row.operands[0].operand != want.operand) return false;
        if (row.operands[0].kind != want.operand_kind) return false;
        if (row.operands[0].address != want.operand_address) return false;
    } else if (!row.operands.empty()) {
        return false;
    }
    return row.from.empty();
}

constexpr const char* kColumnCause =
    "the corner column cut away by the notch";
constexpr const char* kCutAwayCause =
    "cut away outside the target";
constexpr const char* kReAuthoredCause =
    "re-authored with the window";

} // namespace

// ---------------------------------------------------------------------------
// THE GOLDEN — the counts, the chi identity, the volume, the gate.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the golden: the crossed corner - 12/18/8/H=0/"
          "chi=2, the volume 7.5, all exact, the output gate green") {
    const CrossedGuide g = crossed_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                                b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);

    // THE COUNTS — the derivation reproduced (the header's own walk):
    // the box's 8 - 2 (the dead corner column's corners) + 6 born;
    // the box's 12 - 1 (the dead vertical) + 6 born + 1 absorbed
    // joint; the box's 6 + the 2 surviving walls; H = 0 (the L-shaped
    // pierced faces are simply connected).
    REQUIRE(r->vertex_count() == 12);
    REQUIRE(r->edge_count() == 18);
    REQUIRE(r->face_count() == 8);
    REQUIRE(b::inner_loop_count(*r) == 0);
    // THE CHI IDENTITY, visible: 12 - 18 + 8 - 0 = 2 = the box's own
    // chi — the dent family (the through hole's chi = 0, the handle
    // family; the blind's chi = 2 as well, the m2 verdict's proof).
    REQUIRE(b::euler_characteristic(*r) == 2);
    CHECK(b::is_closed_orientable(*r));
    CHECK(b::loops_are_closed(*r));
    // The output gate's XOR: every edge exactly two opposite
    // co-edges — the co-edge incidence 36/2 = 18 verified by the
    // loop table below.
    for (const auto& e : r->edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() != e.coedges()[1]->reversed());
    }
    // The id discipline (the factories' own): id == the vector index.
    for (std::size_t i = 0; i < r->vertices.size(); ++i)
        CHECK(r->vertices[i].id() == static_cast<b::VertexId>(i));
    for (std::size_t i = 0; i < r->edges.size(); ++i)
        CHECK(r->edges[i].id() == static_cast<b::EdgeId>(i));

    // THE VERTICES — the numbering table pinned: the box's survivors
    // (v0,v1,v3,v4,v5,v7 compacted to 0..5), then the born in the
    // notch's own order (exit/joint/exit, lo then hi per position).
    const Vec3d pv[12] = {
        {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1},    // v0, v1, v3
        {-1, -1, 1},  {1, -1, 1},  {-1, 1, 1},     // v4, v5, v7
        {1, 0.5, -1}, {1, 0.5, 1},                 // the exit (1,0.5)
        {0.5, 0.5, -1}, {0.5, 0.5, 1},             // the joint (0.5,0.5)
        {0.5, 1, -1}, {0.5, 1, 1},                 // the exit (0.5,1)
    };
    for (std::size_t i = 0; i < 12; ++i) {
        CAPTURE(i);
        const Vec3d p = r->vertices[i].position();
        CHECK(std::abs(p.x - pv[i].x) < 1e-12);
        CHECK(std::abs(p.y - pv[i].y) < 1e-12);
        CHECK(std::abs(p.z - pv[i].z) < 1e-12);
    }

    // THE EDGES — the numbering table pinned (the box's survivors
    // compacted, the joint vertical, the born rims and exits).
    const std::pair<std::size_t, std::size_t> pe[18] = {
        {0, 1}, {1, 6}, {10, 2}, {2, 0},           // e0; e1'; e2'; e3
        {3, 4}, {4, 7}, {11, 5}, {5, 3},           // e4; e5'; e6'; e7
        {0, 3}, {1, 4}, {2, 5},                    // e8, e9, e11->10
        {8, 9},                                    // 11: the joint
        {8, 6}, {10, 8}, {7, 9}, {9, 11},          // the rims
        {6, 7}, {10, 11},                          // the exits
    };
    for (std::size_t i = 0; i < 18; ++i) {
        CAPTURE(i);
        CHECK(r->edges[i].vertex(0) == pe[i].first);
        CHECK(r->edges[i].vertex(1) == pe[i].second);
    }

    // THE VOLUME — the classical form is the test: abc - (the corner
    // column) = 8 - 0.5 = 7.5. The centroid: the column's own (0.75,
    // 0.75, 0) removed: (0 - 0.5*(0.75, 0.75, 0)) / 7.5 = (-0.05,
    // -0.05, 0).
    const auto mp =
        b::mass_properties(*r, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 7.5));
    CHECK(mp.all_exact);
    CHECK(close(mp.centroid.x, -0.05));
    CHECK(close(mp.centroid.y, -0.05));
    CHECK(close(mp.centroid.z, 0.0));

    // The provenance: the survivors keep the operand's own records
    // (the copy carried them); the born carry the combined one —
    // exact throughout (the operands are exact).
    for (const auto& v : r->vertices) CHECK(v.provenance().is_exact());
    for (const auto& e : r->edges) CHECK(e.provenance().is_exact());
    for (const auto& f : r->solid.shell().faces())
        CHECK(f.provenance().is_exact());
}

// ---------------------------------------------------------------------------
// THE RE-AUTHORED LOOPS — the sequences the output gate reads, every
// face's co-edge table pinned (the flags and the chaining).
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the re-authoring: the eight faces' loops - the "
          "sequences, the flags, the p-curves chained") {
    const CrossedGuide g = crossed_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                                b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);
    const auto& faces = r->solid.shell().faces();

    // The expected co-edge tables (the header's own derivation).
    const std::pair<b::EdgeId, bool>* want[8];
    const std::size_t want_len[8] = {6, 6, 4, 4, 4, 4, 4, 4};
    {
        static const std::pair<b::EdgeId, bool> bottom[] = {
            {3, true}, {2, true}, {13, false}, {12, false},
            {1, true}, {0, true}};
        static const std::pair<b::EdgeId, bool> top[] = {
            {4, false}, {5, false}, {14, false}, {15, false},
            {6, false}, {7, false}};
        static const std::pair<b::EdgeId, bool> front[] = {
            {0, false}, {9, false}, {4, true}, {8, true}};
        static const std::pair<b::EdgeId, bool> back[] = {
            {2, false}, {10, false}, {6, true}, {17, true}};
        static const std::pair<b::EdgeId, bool> left[] = {
            {3, false}, {8, false}, {7, true}, {10, true}};
        static const std::pair<b::EdgeId, bool> right[] = {
            {1, false}, {16, false}, {5, true}, {9, true}};
        static const std::pair<b::EdgeId, bool> lat0[] = {
            {11, false}, {14, true}, {16, true}, {12, true}};
        static const std::pair<b::EdgeId, bool> lat3[] = {
            {17, false}, {15, true}, {11, true}, {13, true}};
        want[0] = bottom; want[1] = top; want[2] = front;
        want[3] = back; want[4] = left; want[5] = right;
        want[6] = lat0; want[7] = lat3;
    }
    for (std::size_t f = 0; f < 8; ++f) {
        CAPTURE(f);
        REQUIRE(faces[f].loops().size() == 1);   // H = 0: no inner loops
        REQUIRE(faces[f].loops()[0].is_outer());
        const auto& ces = faces[f].loops()[0].coedges();
        REQUIRE(ces.size() == want_len[f]);
        for (std::size_t i = 0; i < ces.size(); ++i) {
            CAPTURE(i);
            CHECK(ces[i].edge() == want[f][i].first);
            CHECK(ces[i].reversed() == want[f][i].second);
        }
        CHECK(pcurves_chain(*r, faces[f], 0));
    }

    // The re-authored top loop's rim payloads, pinned: the top face's
    // domain (u,v) = (-x,-y); the rim at y=0.5 runs from dom(1,0.5) =
    // (-1,-0.5) to dom(0.5,0.5) = (-0.5,-0.5); the rim at x=0.5 from
    // (-0.5,-0.5) to (-0.5,-1) — both material-on-left (the L's own
    // interior on the left of the walk, the factory's discipline).
    {
        const auto& ces = faces[1].loops()[0].coedges();
        const auto* rim_a = ces[2].p_curve();
        const auto* rim_b = ces[3].p_curve();
        REQUIRE(rim_a != nullptr);
        REQUIRE(rim_b != nullptr);
        CHECK(rim_a->kind == b::PCurve::Kind::Segment);
        CHECK(rim_b->kind == b::PCurve::Kind::Segment);
        CHECK(std::abs(rim_a->segment.a.x - (-1.0)) < 1e-12);
        CHECK(std::abs(rim_a->segment.a.y - (-0.5)) < 1e-12);
        CHECK(std::abs(rim_a->segment.b.x - (-0.5)) < 1e-12);
        CHECK(std::abs(rim_a->segment.b.y - (-0.5)) < 1e-12);
        CHECK(rim_a->segment.interior_left == true);
        CHECK(std::abs(rim_b->segment.a.x - (-0.5)) < 1e-12);
        CHECK(std::abs(rim_b->segment.a.y - (-0.5)) < 1e-12);
        CHECK(std::abs(rim_b->segment.b.x - (-0.5)) < 1e-12);
        CHECK(std::abs(rim_b->segment.b.y - (-1.0)) < 1e-12);
        CHECK(rim_b->segment.interior_left == true);
    }

    // The wall lat(0)'s window walk, pinned: the plane y=0.5 (the
    // outward +y, the entity's left normal), the origin at the joint
    // (0.5,0.5,-1); the domain (u,v) = (-(z+1), -(x-0.5)) — the
    // window rectangle [-2,0] x [-0.5,0] walked CCW: up the joint,
    // east the top rim, down the exit, west the bottom rim — every
    // Line2 material-on-left (the rectangle IS the face).
    {
        const auto* cyl = dynamic_cast<const b::PlaneSurface*>(
            faces[6].surface());
        REQUIRE(cyl != nullptr);
        CHECK(std::abs(cyl->normal_vec().y - 1.0) < 1e-12);
        const auto& ces = faces[6].loops()[0].coedges();
        // [joint F: (0,0)->(-2,0); rim@hi R: (-2,0)->(-2,-0.5);
        //  exit R: (-2,-0.5)->(0,-0.5); rim@lo R: (0,-0.5)->(0,0)]
        const std::pair<Vec2d, Vec2d> seg[4] = {
            {{0, 0}, {-2, 0}}, {{-2, 0}, {-2, -0.5}},
            {{-2, -0.5}, {0, -0.5}}, {{0, -0.5}, {0, 0}}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            const auto* pc = ces[i].p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Segment);
            CHECK(std::abs(pc->segment.a.x - seg[i].first.x) < 1e-12);
            CHECK(std::abs(pc->segment.a.y - seg[i].first.y) < 1e-12);
            CHECK(std::abs(pc->segment.b.x - seg[i].second.x) < 1e-12);
            CHECK(std::abs(pc->segment.b.y - seg[i].second.y) < 1e-12);
            CHECK(pc->segment.interior_left == true);
        }
    }

    // The wall lat(3)'s window, pinned the same way: the plane x=0.5
    // (the outward +x, the entity's left normal), the domain (u,v) =
    // ((z+1), -(y-0.5)) — the window [0,2] x [-0.5,0].
    {
        const auto* pl = dynamic_cast<const b::PlaneSurface*>(
            faces[7].surface());
        REQUIRE(pl != nullptr);
        CHECK(std::abs(pl->normal_vec().x - 1.0) < 1e-12);
        const auto& ces = faces[7].loops()[0].coedges();
        // [exit F: (0,-0.5)->(2,-0.5); rim@hi R: (2,-0.5)->(2,0);
        //  joint R: (2,0)->(0,0); rim@lo R: (0,0)->(0,-0.5)]
        const std::pair<Vec2d, Vec2d> seg[4] = {
            {{0, -0.5}, {2, -0.5}}, {{2, -0.5}, {2, 0}},
            {{2, 0}, {0, 0}}, {{0, 0}, {0, -0.5}}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            const auto* pc = ces[i].p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Segment);
            CHECK(std::abs(pc->segment.a.x - seg[i].first.x) < 1e-12);
            CHECK(std::abs(pc->segment.a.y - seg[i].first.y) < 1e-12);
            CHECK(std::abs(pc->segment.b.x - seg[i].second.x) < 1e-12);
            CHECK(std::abs(pc->segment.b.y - seg[i].second.y) < 1e-12);
            CHECK(pc->segment.interior_left == true);
        }
    }
}

// ---------------------------------------------------------------------------
// THE NAMES (D3) — the rims, the exits (the unlevelled row), the
// joint (the vocabulary row this unit pays), the walls' inheritance,
// the absorbed faces' own.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the names: the notch rims, the exit edges, the "
          "joint(3), the walls' inheritance - the D3 table on the "
          "crossed case") {
    const CrossedGuide g = crossed_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                                b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);

    // The born edges' relational names (the digest pins them all;
    // here the table, entry for entry).
    CHECK(b::query_edge(*r, 12).name ==
          "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo");
    CHECK(b::query_edge(*r, 13).name ==
          "sub#4:isect[box#1:bottom × ext#3:lat(3)]@lo");
    CHECK(b::query_edge(*r, 14).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*r, 15).name ==
          "sub#4:isect[box#1:top × ext#3:lat(3)]@hi");
    // THE EXIT EDGES — the unlevelled row: the pair's planes ARE the
    // line (the level ABSENT), the side face first (the SSI table's
    // declared operand order).
    CHECK(b::query_edge(*r, 16).name ==
          "sub#4:isect[box#1:right × ext#3:lat(0)]");
    CHECK(b::query_edge(*r, 17).name ==
          "sub#4:isect[box#1:back × ext#3:lat(3)]");
    // THE JOINT VERTICAL — the vocabulary row this unit pays, the
    // first instance: the absorbed tool's joint(3), the extrude's
    // own grammar (no sub context involved — the birth name, like
    // the wall's inheritance).
    CHECK(b::query_edge(*r, 11).name == "ext#3:joint(3)");
    // The walls inherit the tool's own laterals.
    CHECK(b::query_face(*r, 6).name == "ext#3:lat(0)");
    CHECK(b::query_face(*r, 7).name == "ext#3:lat(3)");
    // The absorbed faces keep the factory's names (identity is where
    // a thing was born).
    CHECK(b::query_face(*r, 0).name == "box#1:bottom");
    CHECK(b::query_face(*r, 1).name == "box#1:top");
    CHECK(b::query_face(*r, 3).name == "box#1:back");
    CHECK(b::query_face(*r, 5).name == "box#1:right");
    CHECK(b::query_face(*r, 2).name == "box#1:front");
    CHECK(b::query_face(*r, 4).name == "box#1:left");
    // The absorbed box edges keep their names (re-spanned, not
    // renamed); the kept ones verbatim.
    CHECK(b::query_edge(*r, 1).name == "box#1:bottom1");
    CHECK(b::query_edge(*r, 5).name == "box#1:top1");
    CHECK(b::query_edge(*r, 10).name == "box#1:vertical3");

    // THE STRUCTURAL BY-NAME QUERY — the joint(3) interrogated
    // structurally (the resolver's own read: ShapeId equality, never
    // the render): the creating feature ext#3 and the path
    // [kExtJointRole, 3] find the edge.
    {
        CAD_0::geometry::SubshapeRef ref{};
        ref = ref.push(b::naming::kExtJointRole).push(3);
        const CAD_0::geometry::ShapeId want{
            CAD_0::geometry::FeatureId{3}, ref, {}};
        const std::vector<b::EdgeId> found =
            b::query_edge_by_name(*r, want);
        REQUIRE(found.size() == 1);
        CHECK(found[0] == 11);
    }

    // The honest boundary: the unnamed mode — the born rims/exits
    // read honestly EMPTY, the joint's birth name and the walls'
    // inheritance flow regardless (stamp copies and the extrude's
    // own table, no sub context involved).
    {
        auto r2 = b::exact_subtract(*g.box, *g.tool, g.offset,
                                     b::kSubtractTolerance);
        REQUIRE(r2 != nullptr);
        for (const b::EdgeId i : {b::EdgeId{12}, b::EdgeId{13},
                                  b::EdgeId{14}, b::EdgeId{15},
                                  b::EdgeId{16}, b::EdgeId{17}}) {
            CAPTURE(static_cast<unsigned>(i));
            CHECK(b::query_edge(*r2, i).name.empty());
        }
        CHECK(b::query_edge(*r2, 11).name == "ext#3:joint(3)");
        CHECK(b::query_face(*r2, 6).name == "ext#3:lat(0)");
        CHECK(b::query_face(*r2, 7).name == "ext#3:lat(3)");
        // The counts identical: the naming never moves the address
        // plane (the m4 pin, replicated on the crossed).
        CHECK(r2->vertex_count() == 12);
        CHECK(r2->edge_count() == 18);
        CHECK(r2->face_count() == 8);
    }
}

// ---------------------------------------------------------------------------
// THE TAVOLA PIENA — the 64 rows, row by row (D3's counts: 38 result
// + 26 vanished; the two-sided closure green).
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the tavola piena: the crossed case's 64 rows, "
          "row by row, the closure green") {
    const CrossedGuide g = crossed_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                                b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);
    const b::DescentTable t = b::subtract_descent(*r, *g.box, *g.tool);

    // D3's own count, verified: 38 out + 26 vanished = 64.
    REQUIRE(t.rows.size() == 64);
    CHECK(t.count(b::Descent::Kept) == 15);      // 6 V + 7 E + 2 F
    CHECK(t.count(b::Descent::Absorbed) == 11);  // 5 E + 6 F
    CHECK(t.count(b::Descent::Born) == 12);      // 6 V + 6 E
    CHECK(t.count(b::Descent::Vanished) == 26);
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    // The expected rows (the m4 golden's own shape) — the result's
    // walk first (vertices, edges, faces), then the vanished tail in
    // the dying operands' own order (A's corner column, then the
    // tool's dead).
    const std::vector<ExpectedRow> expected = {
        // ---- the result's vertices: 6 kept (A's own addresses) + 6 born
        {b::Descent::Kept, b::EntityKind::Vertex, 0,
         true, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 1,
         true, 0, b::EntityKind::Vertex, 1, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 2,
         true, 0, b::EntityKind::Vertex, 3, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 3,
         true, 0, b::EntityKind::Vertex, 4, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 4,
         true, 0, b::EntityKind::Vertex, 5, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 5,
         true, 0, b::EntityKind::Vertex, 7, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 6,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 7,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 8,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 9,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 10,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 11,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        // ---- the result's edges: 7 kept + 5 absorbed + 6 born
        {b::Descent::Kept, b::EntityKind::Edge, 0,
         true, 0, b::EntityKind::Edge, 0, "box#1:bottom0", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Edge, 1,
         true, 0, b::EntityKind::Edge, 1, "box#1:bottom1", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Edge, 2,
         true, 0, b::EntityKind::Edge, 2, "box#1:bottom2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 3,
         true, 0, b::EntityKind::Edge, 3, "box#1:bottom3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 4,
         true, 0, b::EntityKind::Edge, 4, "box#1:top0", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Edge, 5,
         true, 0, b::EntityKind::Edge, 5, "box#1:top1", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Edge, 6,
         true, 0, b::EntityKind::Edge, 6, "box#1:top2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 7,
         true, 0, b::EntityKind::Edge, 7, "box#1:top3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 8,
         true, 0, b::EntityKind::Edge, 8, "box#1:vertical0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 9,
         true, 0, b::EntityKind::Edge, 9, "box#1:vertical1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 10,
         true, 0, b::EntityKind::Edge, 11, "box#1:vertical3", nullptr},
        // THE JOINT — the absorbed tool's vertical at J_3 (B's edge
        // 2n+3 = 11), the vocabulary row's first instance.
        {b::Descent::Absorbed, b::EntityKind::Edge, 11,
         true, 1, b::EntityKind::Edge, 11, "ext#3:joint(3)", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 12, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 13, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:bottom × ext#3:lat(3)]@lo", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 14, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 15, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(3)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 16, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:right × ext#3:lat(0)]", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 17, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:back × ext#3:lat(3)]", nullptr},
        // ---- the result's faces: 2 kept + 6 absorbed
        {b::Descent::Absorbed, b::EntityKind::Face, 0,
         true, 0, b::EntityKind::Face, 0, "box#1:bottom", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 1,
         true, 0, b::EntityKind::Face, 1, "box#1:top", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 2,
         true, 0, b::EntityKind::Face, 2, "box#1:front", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 3,
         true, 0, b::EntityKind::Face, 3, "box#1:back", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 4,
         true, 0, b::EntityKind::Face, 4, "box#1:left", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 5,
         true, 0, b::EntityKind::Face, 5, "box#1:right", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 6,
         true, 1, b::EntityKind::Face, 0, "ext#3:lat(0)", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 7,
         true, 1, b::EntityKind::Face, 3, "ext#3:lat(3)", nullptr},
        // ---- the vanished tail: A's corner column (A's own order)
        {b::Descent::Vanished, b::EntityKind::Vertex, 2,
         true, 0, b::EntityKind::Vertex, 2, "", kColumnCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 6,
         true, 0, b::EntityKind::Vertex, 6, "", kColumnCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 10,
         true, 0, b::EntityKind::Edge, 10, "box#1:vertical2",
         kColumnCause},
        // ---- the vanished tail: the tool's dead (B's own order —
        // vertices: the J_3 pair re-timed, the rest outside)
        {b::Descent::Vanished, b::EntityKind::Vertex, 0,
         true, 1, b::EntityKind::Vertex, 0, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 1,
         true, 1, b::EntityKind::Vertex, 1, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 2,
         true, 1, b::EntityKind::Vertex, 2, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 3,
         true, 1, b::EntityKind::Vertex, 3, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 4,
         true, 1, b::EntityKind::Vertex, 4, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 5,
         true, 1, b::EntityKind::Vertex, 5, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 6,
         true, 1, b::EntityKind::Vertex, 6, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 7,
         true, 1, b::EntityKind::Vertex, 7, "", kReAuthoredCause},
        // edges: the notch-chain entities' curves re-authored (the
        // entities 0 and 3: B's edges 0, 4, 3, 7), the outside
        // entities' curves (1, 5, 2, 6) and the J_0..J_2 verticals
        // (8, 9, 10) cut away
        {b::Descent::Vanished, b::EntityKind::Edge, 0,
         true, 1, b::EntityKind::Edge, 0, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 1,
         true, 1, b::EntityKind::Edge, 1, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 2,
         true, 1, b::EntityKind::Edge, 2, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 3,
         true, 1, b::EntityKind::Edge, 3, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 4,
         true, 1, b::EntityKind::Edge, 4, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 5,
         true, 1, b::EntityKind::Edge, 5, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 6,
         true, 1, b::EntityKind::Edge, 6, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 7,
         true, 1, b::EntityKind::Edge, 7, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 8,
         true, 1, b::EntityKind::Edge, 8, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 9,
         true, 1, b::EntityKind::Edge, 9, "", kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 10,
         true, 1, b::EntityKind::Edge, 10, "", kCutAwayCause},
        // faces: the outside walls and the caps cut away
        {b::Descent::Vanished, b::EntityKind::Face, 1,
         true, 1, b::EntityKind::Face, 1, "ext#3:lat(1)",
         kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Face, 2,
         true, 1, b::EntityKind::Face, 2, "ext#3:lat(2)",
         kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Face, 4,
         true, 1, b::EntityKind::Face, 4, "ext#3:cap(lo)",
         kCutAwayCause},
        {b::Descent::Vanished, b::EntityKind::Face, 5,
         true, 1, b::EntityKind::Face, 5, "ext#3:cap(hi)",
         kCutAwayCause},
    };
    REQUIRE(expected.size() == 64);
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CAPTURE(i);
        CHECK(row_matches(t.rows[i], expected[i]));
    }

    // THE CLOSURE, both sides, hard (the m4 activation runs it in
    // debug builds inside; here the explicit pins).
    std::string why;
    CHECK(b::descent_closed(*r, t, &why));
    CHECK(b::old_generation_closed(*g.box, *g.tool, t, &why));
}

// ---------------------------------------------------------------------------
// THE DIGEST + THE DETERMINISM — the names beside the G1 ones, N=10
// rebuilds bit-identical.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the digest and the determinism: N=10 rebuilds, "
          "one digest, bit for bit") {
    const CrossedGuide g = crossed_guide();
    auto r0 = b::exact_subtract(*g.box, *g.tool, g.offset,
                                 b::kSubtractTolerance, g.named);
    REQUIRE(r0 != nullptr);
    const std::string fd = name_digest(*r0);
    const std::string ed = edge_name_digest(*r0);

    // The digest's shape: the crossed case's own rows — the fillet's
    // future strings live here (the notch rims and the exits), the
    // joint's birth name beside them, the box's own names carried.
    REQUIRE(ed.size() > 0);
    CHECK(ed.find("12|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n") !=
          std::string::npos);
    CHECK(ed.find("14|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n") !=
          std::string::npos);
    CHECK(ed.find("16|sub#4:isect[box#1:right × ext#3:lat(0)]\n") !=
          std::string::npos);
    CHECK(ed.find("11|ext#3:joint(3)\n") != std::string::npos);
    CHECK(ed.find("0|box#1:bottom0\n") != std::string::npos);
    // The dead entity's name is NOWHERE in the digest (the (1,1)
    // vertical died with the column).
    CHECK(ed.find("vertical2") == std::string::npos);

    // N=10 rebuilds: the same recipe, the same digests, bit for bit.
    for (int i = 0; i < 10; ++i) {
        CrossedGuide g2 = crossed_guide();
        auto r = b::exact_subtract(*g2.box, *g2.tool, g2.offset,
                                    b::kSubtractTolerance, g2.named);
        REQUIRE(r != nullptr);
        CHECK(name_digest(*r) == fd);
        CHECK(edge_name_digest(*r) == ed);
    }
}

// ---------------------------------------------------------------------------
// LA CONTROPROVA + THE STILL-REFUSED FAMILIES — the wall that became
// a door, and the walls that stay walls (the additivity's own pins).
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m3 the counterproof and the borders: the corner "
          "answers, the mid-edge family and the tangential border and "
          "the blind wall keep their refusals") {
    // THE COUNTERPROOF: the guide case's own call — the placement the
    // CROSSED pin (:952's family) refused yesterday — answers today
    // with the result. The discrimination is the wall becoming a
    // door: pinned above (the golden, the loops, the names); here the
    // same call's SHAPE, one assertion, the door's own answer.
    {
        const CrossedGuide g = crossed_guide();
        auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                                    b::kSubtractTolerance, g.named);
        REQUIRE(r != nullptr);
        REQUIRE(r->face_count() == 8);
    }

    // THE MID-EDGE FAMILY still refuses (the crossing not at a
    // covered corner: the pin's own circle, crossing one edge) — the
    // G1 pin's phrases still answer.
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        std::vector<s::SketchChain> chains{
            s::SketchChain{{s::sketch_circle({0.9f, 0.0f}, 0.35f,
                                             false)}}};
        auto tool = b::make_prism(chains, 3.0f, +1);
        bool threw = false;
        std::string msg;
        try {
            auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.5},
                                        b::kSubtractTolerance);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("CROSSES") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE TANGENTIAL BORDER — LA PARETE's own pin (:914): the
    // tangential family keeps today's refusal BYTE-IDENTICAL (the
    // message's own words, checked verbatim).
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        std::vector<s::SketchChain> chains{
            s::SketchChain{{s::sketch_circle({0.6f, 0.0f}, 0.4f,
                                             false)}}};
        auto tool = b::make_prism(chains, 3.0f, +1);
        bool threw = false;
        std::string msg;
        try {
            auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.5},
                                        b::kSubtractTolerance);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        // The pinned disposition, verbatim: the TANGENT band, the
        // deterministic tie-break, the refusal.
        CHECK(msg.find("TANGENT") != std::string::npos);
        CHECK(msg.find("DETERMINISTIC") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE BLIND WALL — the m2 pin, alive: the blind family keeps its
    // refusal with the orientation wall's own name (the decoupling
    // morse cited). The crossed door does not pass through it.
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        std::vector<s::SketchChain> chains{
            s::SketchChain{{s::sketch_circle({0.0f, 0.0f}, 0.25f,
                                             false)}}};
        auto tool = b::make_prism(chains, 1.6f, +1);
        bool threw = false;
        std::string msg;
        try {
            auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.2},
                                        b::kSubtractTolerance);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("BLIND") != std::string::npos);
        CHECK(msg.find("ORIENTATION WALL") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE BLIND-CROSSED COMPOSITE — the notch footprint with the
    // blind span: refused with its own honest name (the queued
    // family, the composite's cap blocked by the wall).
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        auto tool = b::make_prism(square_chains_corner(), 2.0f, +1);
        // offset (0,0,-0.5): z in [-0.5, 1.5] — the bottom cap
        // inside: the blind span; the footprint crosses the corner.
        bool threw = false;
        std::string msg;
        try {
            auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -0.5},
                                        b::kSubtractTolerance);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("COMPOSITE") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE CIRCULAR CROSSED — an arc exit at the corner: refused with
    // the family's own name (the arc class shares the blind's wall;
    // queued with the reason sharpened). The tool is the landed
    // SECTOR vocabulary (segment-arc-segment, the G1-u3 mixed
    // profile): the pie at (0.2, 0.6), r = 1.0, the arms at +-45
    // degrees — the arc crosses the x=1 edge at (1, 0), the arm
    // crosses the y=1 edge at (0.6, 1), and the corner (1,1) is
    // covered (the arc's slack 0.106, the arms' 0.849 and 0.283).
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        const float a0 = -0.25f * static_cast<float>(kPi);
        const float sweep = 0.5f * static_cast<float>(kPi);
        const float r = 1.0f;
        const float c_x = 0.2f, c_y = 0.6f;
        const Vec2d c{c_x, c_y};
        const Vec2d ps{c.x + r * std::cos(a0), c.y + r * std::sin(a0)};
        const Vec2d pe{c.x + r * std::cos(a0 + sweep),
                       c.y + r * std::sin(a0 + sweep)};
        std::vector<s::SketchChain> chains{s::SketchChain{{
            s::sketch_segment({c_x, c_y},
                              {static_cast<float>(ps.x),
                               static_cast<float>(ps.y)}, true),
            s::sketch_arc({c_x, c_y}, r, false, a0, sweep),
            s::sketch_segment({static_cast<float>(pe.x),
                               static_cast<float>(pe.y)},
                              {c_x, c_y}, true),
        }}};
        auto tool = b::make_prism(chains, 3.0f, +1);
        bool threw = false;
        std::string msg;
        try {
            auto r2 = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.5},
                                         b::kSubtractTolerance);
            (void)r2;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("CROSSES") != std::string::npos);
        CHECK(msg.find("arc") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE ADDITIVITY, pinned on the same box: the THROUGH family
    // answers the landed golden beside the crossed door (the counts,
    // the gate, the volume — one assertion each, the suite carries
    // the rest).
    {
        auto a = b::make_box({1.0, 1.0, 1.0});
        std::vector<s::SketchChain> chains{
            s::SketchChain{{s::sketch_circle({0.0f, 0.0f}, 0.5f,
                                             false)}}};
        auto tool = b::make_prism(chains, 3.0f, +1);
        auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.5},
                                    b::kSubtractTolerance);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 12);
        REQUIRE(r->edge_count() == 17);
        REQUIRE(r->face_count() == 7);
        REQUIRE(b::euler_characteristic(*r) == 0);
        const auto mp =
            b::mass_properties(*r,
                               CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.5 * kPi));
    }
}
