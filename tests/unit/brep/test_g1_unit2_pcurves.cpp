// tests/unit/brep/test_g1_unit2_pcurves.cpp
//
// G1 unit 2 — THE PRIMITIVES WITH TRUE TRIMMING (ADR-0032 D1.1, on
// top of D1.2/D2/D8 of the accepted ADR). Unit 1 planted the fields
// and nobody wrote them; this unit gives the three factories their
// exact content — the faces stop being "closed by construction" and
// become REGIONS OF DOMAINS (D2.3): the face IS (surface, region),
// and the trimming loop IS the region's boundary in the (u,v) plane.
//
// Written FIRST (D9): on the tree of unit 1 this file fails for the
// right reason — the query API is absent (the include names it), and
// every p-curve pin below fails on nullptr (the factories do not
// write yet). Four families, per the order:
//
//   * THE NEW GOLDENS — for every primitive, for every face: family,
//     loop counts, the p-curves' types and parameters, hand-derived
//     from the surfaces' (u,v) frames (the "how an exact face
//     sounds", pinned);
//   * THE COMPATIBILITY — the counts do not move (box 8/12/6, sphere
//     2/1/1, cylinder 4/5/3, tetra 4/6/4; all H = 0, χ = 2), and the
//     faceted world (bridge extracts, golden bodies) still carries
//     NO p-curves — the per-world invariant of D2.2;
//   * THE PROVENANCE — the factories write exact/0, enumerated per
//     primitive AND per face through the query API (the D8 contract
//     at its consumption point);
//   * THE CONVENTIONS — for every p-curve written, direct tests (not
//     interpretations): DIRECTION (the p-curve's endpoints evaluate
//     to the co-edge's start/end vertex positions), CHAINING (a
//     loop's p-curves chain endpoint-to-endpoint, modulo the seam
//     period where the surface is periodic), REGION SIDE (ε-left of
//     the traversal is inside the face's region, ε-right is outside
//     — except on seam copies, where the right side is the
//     neighbouring window's material).
//
// The convention correction this unit lands (the order's "il primo
// uso vero: se una convenzione è ambigua, il codice ce lo dice"):
// CylinderSurface was the analytic set's LEFT-handed parametrization
// (its natural frame ∂u×∂v points INWARD), while PlaneSurface and
// SphereSurface are right-handed around the outward normal. Writing
// the wall's and the caps' loops all material-on-left (the pinned
// pcurve.hpp convention) then walks the shared rim edges in the SAME
// direction twice — breaking the radial-edge orientation-XOR. No
// loop rearrangement fixes that with both faces reading their
// regions honestly: the domains disagree about handedness. The
// correction (documented in analytic.hpp): the cylinder's (u,v)
// frame is flipped to the outward orientation — ONE rule for every
// analytic surface, and the existing loop structure of all three
// factories becomes exactly the material-on-left walk. The two
// z-sign pins that encoded the old handedness are amended in
// test_analytic_surfaces.cpp, and the orientation itself is pinned
// there as a regression guard.
//
// Frame derivations, hand-worked here so the pins are not magic:
//
//   * PlaneSurface frame (the constructor's rule): u_axis = n×x̂ when
//     |n·ŷ| > 0.9 else n×ŷ, v_axis = n×u_axis — always right-handed
//     around the given normal (u_axis×v_axis = n), so CCW in the
//     domain is CCW around the outward normal in 3D.
//   * Box faces (half extents (x,y,z) = (1,2,3)):
//       face 0 bottom (−Z): (u,v) = (X, −Y)
//       face 1 top    (+Z): (u,v) = (−X, −Y)
//       face 2 front  (−Y): (u,v) = (Z, −X)
//       face 3 back   (+Y): (u,v) = (−Z, −X)
//       face 4 left   (−X): (u,v) = (−Z, −Y)
//       face 5 right  (+X): (u,v) = (Z, −Y)
//   * Sphere (u azimuth [0,2π], v polar [0,π], v=0 north): the slit
//     rectangle's boundary walk (CCW, material on left) is bottom
//     east → right seam UP → top west → left seam DOWN; the
//     degenerate pole edges carry no co-edges, so the 3D loop is
//     [seam fwd, seam rev] with the fwd copy on the RIGHT edge
//     (u=2π) and the rev copy on the LEFT edge (u=0).
//   * Cylinder (corrected frame: u = azimuth measured so that
//     evaluate(u,v) = (R cos u, v, −R sin u); u=0 is the seam
//     meridian through v_top1/v_bot1 = (+R, ·, 0)): the wall's
//     region is the window [0,2π]×[−h,h]; its CCW boundary walk is
//     bottom EAST (v=−h), right seam UP (u=2π), top WEST (v=+h),
//     left seam DOWN (u=0) — which is exactly the factory's existing
//     loop [seam f, e_top2 rev, e_top1 rev, seam r, e_bot2 rev,
//     e_bot1 rev] once e_top1 is the −Z half-arc and e_top2 the +Z
//     half-arc (the corrected side assignment; e_bot1/e_bot2 keep
//     +Z/−Z). The caps' rims are full circles walked CCW in the
//     caps' own plane frames — two π-sweep arcs each.
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
namespace br = CAD_0::bridge;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kTwoPi = 6.28318530717958647692;
// The sketch chain's own joint discipline (sketch_profile.hpp
// kJointCoincidenceTol), mirrored — the p-curve chaining uses the
// same coincidence rule the sketcher enforces on its chains.
constexpr double kCoincidenceTol = 1e-5;
// 3D cross-check tolerance: float trig on the way, O(1) coordinates —
// a wrong endpoint is off by O(1), noise is off by ~1e-6.
constexpr double kEvalTol = 1e-4;

bool close(double a, double b, double tol = kCoincidenceTol) {
    return std::abs(a - b) <= tol;
}

bool close2(const Vec2d& a, const Vec2d& b, double tol = kCoincidenceTol) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol);
}

bool close3(const Vec3d& a, const Vec3d& b, double tol = kEvalTol) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol) &&
           close(a.z, b.z, tol);
}

// The domain endpoints of a p-curve. Line2: stored. Arc2: derived by
// the sketch kernel's own arc_start_point/arc_end_point — one
// opinion, one code (the D2.1 discipline; a second endpoint formula
// here would be exactly the divergence the reuse exists to prevent).
Vec2d pcurve_start(const b::PCurve& p) {
    switch (p.kind) {
    case b::PCurve::Kind::Segment: return p.segment.a;
    case b::PCurve::Kind::Arc:     return b::pcurve_arc_start_point(p.arc);
    case b::PCurve::Kind::Circle:  return p.circle.center;
    }
    return {};
}

Vec2d pcurve_end(const b::PCurve& p) {
    switch (p.kind) {
    case b::PCurve::Kind::Segment: return p.segment.b;
    case b::PCurve::Kind::Arc:     return b::pcurve_arc_end_point(p.arc);
    case b::PCurve::Kind::Circle:  return p.circle.center;
    }
    return {};
}

// Coincidence with optional u-periodicity — the seam discipline: the
// slit rectangle's two vertical edges are one period apart in the
// domain, and the walk crosses between them at the poles (sphere) or
// not at all (cylinder wall). Consecutive p-curves chain if their
// endpoints coincide, OR coincide after a ±period shift in u.
bool coincident(Vec2d a, Vec2d b, bool periodic_u, double period) {
    if (close2(a, b)) return true;
    if (periodic_u) {
        if (close(a.x, b.x + period) && close(a.y, b.y)) return true;
        if (close(a.x, b.x - period) && close(a.y, b.y)) return true;
    }
    return false;
}

// The face's region in its domain, as the test's per-family model —
// a convex CCW polygon (the box faces' projected rectangles, the
// tetrahedron's projected triangles), a window with optional
// u-periodicity (the sphere's slit rectangle, the wall's strip), or
// a disc (the caps). u-periodic windows identify u modulo their
// width (the seam).
struct RegionSpec {
    std::vector<Vec2d> polygon{};  // CCW corners (walk order)
    bool periodic_u{false};
    double umin{0}, umax{0}, vmin{0}, vmax{0};
    bool disc{false};
    double radius{0};

    bool contains(const Vec2d& p) const {
        if (disc) return p.length() <= radius + kCoincidenceTol;
        if (!polygon.empty()) {
            // Point in convex CCW polygon: on the LEFT of every edge
            // (within tolerance). For a CW ordering the left
            // half-planes' intersection is empty — so a wrongly
            // oriented loop fails the LEFT check below, exactly the
            // discrimination the convention test wants.
            const std::size_t n = polygon.size();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2d& a = polygon[i];
                const Vec2d& b = polygon[(i + 1) % n];
                const double cross =
                    (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
                if (cross < -1e-6) return false;
            }
            return true;
        }
        double u = p.x;
        if (periodic_u) {
            const double w = umax - umin;
            u = std::fmod(u - umin, w);
            if (u < 0) u += w;
            u += umin;
        }
        return u >= umin - kCoincidenceTol && u <= umax + kCoincidenceTol &&
               p.y >= vmin - kCoincidenceTol && p.y <= vmax + kCoincidenceTol;
    }
};

// The midpoint and the left/right offset points of a p-curve, in its
// traversal direction: the Arc2's tangent is the CCW sweep direction
// (the sketch vocabulary stores CCW sweeps only), whose left is
// toward the circle's center — the disc side, as the region-side
// convention reads.
Vec2d pcurve_mid(const b::PCurve& p) {
    switch (p.kind) {
    case b::PCurve::Kind::Segment:
        return Vec2d{(p.segment.a.x + p.segment.b.x) * 0.5,
                     (p.segment.a.y + p.segment.b.y) * 0.5};
    case b::PCurve::Kind::Arc: {
        const double t = p.arc.start_angle + p.arc.sweep * 0.5;
        return p.arc.center + Vec2d{p.arc.radius * std::cos(t),
                                    p.arc.radius * std::sin(t)};
    }
    case b::PCurve::Kind::Circle:
        return p.circle.center + Vec2d{p.circle.radius, 0};
    }
    return {};
}

Vec2d left_offset(const b::PCurve& p, double eps) {
    Vec2d t{1, 0};
    switch (p.kind) {
    case b::PCurve::Kind::Segment: {
        const Vec2d d = p.segment.b - p.segment.a;
        const double len = d.length();
        if (len > 0) t = Vec2d{d.x / len, d.y / len};
        break;
    }
    case b::PCurve::Kind::Arc: {
        const double ang = p.arc.start_angle + p.arc.sweep * 0.5;
        t = Vec2d{-std::sin(ang), std::cos(ang)};
        break;
    }
    case b::PCurve::Kind::Circle:
        t = Vec2d{0, 1};
        break;
    }
    // Left of the heading = the heading rotated +90° (CCW): perp(t).
    const Vec2d left{-t.y, t.x};
    const Vec2d m = pcurve_mid(p);
    return m + Vec2d{left.x * eps, left.y * eps};
}

Vec2d right_offset(const b::PCurve& p, double eps) {
    Vec2d t{1, 0};
    switch (p.kind) {
    case b::PCurve::Kind::Segment: {
        const Vec2d d = p.segment.b - p.segment.a;
        const double len = d.length();
        if (len > 0) t = Vec2d{d.x / len, d.y / len};
        break;
    }
    case b::PCurve::Kind::Arc: {
        const double ang = p.arc.start_angle + p.arc.sweep * 0.5;
        t = Vec2d{-std::sin(ang), std::cos(ang)};
        break;
    }
    case b::PCurve::Kind::Circle:
        t = Vec2d{0, 1};
        break;
    }
    const Vec2d left{-t.y, t.x};
    const Vec2d m = pcurve_mid(p);
    return m - Vec2d{left.x * eps, left.y * eps};
}

// ---- the pinned tables ------------------------------------------------

struct PinnedLine {
    b::EdgeId edge;
    bool reversed;
    Vec2d a;
    Vec2d b;
};

struct PinnedArc {
    b::EdgeId edge;
    bool reversed;
    Vec2d center;
    double radius;
    bool hole;
    double start_angle;
    double sweep;
};

// Compare one face's single outer loop against a pinned Line2 table
// (kind, 3D edge reference, orientation, domain endpoints, region
// side). The pinned values are the golden: bit-level drift in the
// factories' authoring lands here first.
void check_loop_lines(const b::Face& face, const PinnedLine* pinned,
                      std::size_t n) {
    REQUIRE(face.loops().size() == 1);
    const auto& coedges = face.loops()[0].coedges();
    REQUIRE(coedges.size() == n);
    for (std::size_t i = 0; i < n; ++i) {
        const b::CoEdge& ce = coedges[i];
        REQUIRE(ce.p_curve() != nullptr);
        CHECK(ce.p_curve()->kind == b::PCurve::Kind::Segment);
        CHECK(ce.edge() == pinned[i].edge);
        CHECK(ce.reversed() == pinned[i].reversed);
        CHECK(ce.p_curve()->segment.interior_left == true);
        CHECK(close2(ce.p_curve()->segment.a, pinned[i].a));
        CHECK(close2(ce.p_curve()->segment.b, pinned[i].b));
    }
}

void check_loop_arcs(const b::Face& face, const PinnedArc* pinned,
                     std::size_t n) {
    REQUIRE(face.loops().size() == 1);
    const auto& coedges = face.loops()[0].coedges();
    REQUIRE(coedges.size() == n);
    for (std::size_t i = 0; i < n; ++i) {
        const b::CoEdge& ce = coedges[i];
        REQUIRE(ce.p_curve() != nullptr);
        CHECK(ce.p_curve()->kind == b::PCurve::Kind::Arc);
        CHECK(ce.edge() == pinned[i].edge);
        CHECK(ce.reversed() == pinned[i].reversed);
        const b::Arc2& arc = ce.p_curve()->arc;
        CHECK(close2(arc.center, pinned[i].center));
        CHECK(close(arc.radius, pinned[i].radius));
        CHECK(arc.hole == pinned[i].hole);
        CHECK(close(arc.start_angle, pinned[i].start_angle));
        CHECK(close(arc.sweep, pinned[i].sweep));
    }
}

// ---- the conventions checker (family D) --------------------------------

// For one face (single outer loop — every factory face): every
// co-edge's p-curve satisfies the pinned conventions, DIRECTLY:
//
//   DIRECTION — evaluate(p_curve start) == the co-edge's start vertex
//   position, ditto end (the u candidate may shift by ±period on a
//   periodic surface: the seam's two copies are one meridian);
//
//   CHAINING — p-curve i's end coincides with p-curve (i+1 mod n)'s
//   start, modulo the same period;
//
//   REGION SIDE — ε-left of each p-curve's midpoint is INSIDE the
//   face's region; ε-right is OUTSIDE, except on seam copies (the
//   is_seam flags) where the right side is the neighbouring window's
//   material — inside after the periodic identification. A seam is
//   not a boundary of the region; it is where the window was slit.
void check_conventions(const b::BRepBody& body, const b::Face& face,
                       const RegionSpec& region,
                       const std::vector<bool>& is_seam) {
    const bool periodic = region.periodic_u;
    const double period = region.umax - region.umin;
    const double eps = 1e-3;  // ≪ region extents, ≫ float noise

    REQUIRE(face.loops().size() == 1);
    const auto& coedges = face.loops()[0].coedges();
    REQUIRE(coedges.size() == is_seam.size());
    const std::size_t n = coedges.size();

    const auto surface = face.surface();
    REQUIRE(surface != nullptr);

    for (std::size_t i = 0; i < n; ++i) {
        const b::CoEdge& ce = coedges[i];
        CAPTURE(i);
        CAPTURE(ce.edge());
        REQUIRE(ce.p_curve() != nullptr);
        const Vec2d ps = pcurve_start(*ce.p_curve());
        const Vec2d pe = pcurve_end(*ce.p_curve());

        // DIRECTION (start vertex): try u as stored, then ± period.
        const Vec3d want_start =
            body.vertices[ce.start_vertex(body.edges)].position();
        const Vec3d want_end =
            body.vertices[ce.end_vertex(body.edges)].position();
        bool start_ok = close3(surface->evaluate(ps.x, ps.y), want_start);
        bool end_ok = close3(surface->evaluate(pe.x, pe.y), want_end);
        if (periodic) {
            if (!start_ok)
                start_ok = close3(surface->evaluate(ps.x + period, ps.y),
                                  want_start) ||
                           close3(surface->evaluate(ps.x - period, ps.y),
                                  want_start);
            if (!end_ok)
                end_ok = close3(surface->evaluate(pe.x + period, pe.y),
                                want_end) ||
                         close3(surface->evaluate(pe.x - period, pe.y),
                                want_end);
        }
        CHECK(start_ok);
        CHECK(end_ok);

        // CHAINING: my end meets the next co-edge's start (mod period).
        const b::CoEdge& next = coedges[(i + 1) % n];
        REQUIRE(next.p_curve() != nullptr);
        const Vec2d next_start = pcurve_start(*next.p_curve());
        CHECK(coincident(pe, next_start, periodic, period));

        // REGION SIDE: left is material, right is not — unless seam.
        CHECK(region.contains(left_offset(*ce.p_curve(), eps)));
        if (is_seam[i]) {
            // The seam's right side is the neighbouring window's
            // material: inside after the periodic identification.
            CHECK(region.contains(right_offset(*ce.p_curve(), eps)));
        } else {
            CHECK_FALSE(region.contains(right_offset(*ce.p_curve(), eps)));
        }
    }
}

// The convex polygon of a planar face's region, derived from the
// loop's own Line2 endpoints in walk order (the corners — a
// rectangle for the box's faces, a triangle for the tetrahedron's;
// NOT the bounding box, which is the wrong region for a triangle:
// an edge's outside can still lie under the box's roof).
RegionSpec polygon_from_corners(const b::Face& face) {
    RegionSpec r;
    for (const auto& loop : face.loops())
        for (const auto& ce : loop.coedges()) {
            REQUIRE(ce.p_curve() != nullptr);
            REQUIRE(ce.p_curve()->kind == b::PCurve::Kind::Segment);
            r.polygon.push_back(ce.p_curve()->segment.a);
        }
    return r;
}

// Read a golden file as text (the v0.1.0 fixture is pure JSON — it
// goes through parse_document, not load_file; same helper shape as
// test_native.cpp's and unit 1's).
std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ifstream::binary);
    const std::string missing = "cannot open: " + p.string();
    REQUIRE_MESSAGE(static_cast<bool>(in), missing);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::filesystem::path golden_file(const char* name) {
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" / name;
}

} // namespace

// ---------------------------------------------------------------------------
// Family A — the new goldens: the p-curve tables, hand-derived
// ---------------------------------------------------------------------------

TEST_CASE("G1-U2 golden: make_box(1,2,3) — 6 planar faces, 4 Line2 each, pinned domains") {
    auto body = b::make_box(Vec3d{1, 2, 3});
    REQUIRE(body != nullptr);

    struct FaceTable {
        b::FaceId id;
        const char* frame;  // the (u,v) meaning, for the report
        PinnedLine lines[4];
    };
    // The derivations are in the file header; each table is the
    // loop's walk in the face's own domain, CCW (material on left).
    const FaceTable tables[] = {
        // face 0 bottom (−Z), (u,v) = (X,−Y), walk 0→3→2→1.
        {0, "(X,-Y)",
         {{3, true, {-1, 2}, {-1, -2}},
          {2, true, {-1, -2}, {1, -2}},
          {1, true, {1, -2}, {1, 2}},
          {0, true, {1, 2}, {-1, 2}}}},
        // face 1 top (+Z), (u,v) = (−X,−Y), walk 4→5→6→7.
        {1, "(-X,-Y)",
         {{4, false, {1, 2}, {-1, 2}},
          {5, false, {-1, 2}, {-1, -2}},
          {6, false, {-1, -2}, {1, -2}},
          {7, false, {1, -2}, {1, 2}}}},
        // face 2 front (−Y), (u,v) = (Z,−X), walk 0→1→5→4.
        {2, "(Z,-X)",
         {{0, false, {-3, 1}, {-3, -1}},
          {9, false, {-3, -1}, {3, -1}},
          {4, true, {3, -1}, {3, 1}},
          {8, true, {3, 1}, {-3, 1}}}},
        // face 3 back (+Y), (u,v) = (−Z,−X), walk 2→3→7→6.
        {3, "(-Z,-X)",
         {{2, false, {3, -1}, {3, 1}},
          {11, false, {3, 1}, {-3, 1}},
          {6, true, {-3, 1}, {-3, -1}},
          {10, true, {-3, -1}, {3, -1}}}},
        // face 4 left (−X), (u,v) = (−Z,−Y), walk 3→0→4→7.
        {4, "(-Z,-Y)",
         {{3, false, {3, -2}, {3, 2}},
          {8, false, {3, 2}, {-3, 2}},
          {7, true, {-3, 2}, {-3, -2}},
          {11, true, {-3, -2}, {3, -2}}}},
        // face 5 right (+X), (u,v) = (Z,−Y), walk 1→2→6→5.
        {5, "(Z,-Y)",
         {{1, false, {-3, 2}, {-3, -2}},
          {10, false, {-3, -2}, {3, -2}},
          {5, true, {3, -2}, {3, 2}},
          {9, true, {3, 2}, {-3, 2}}}},
    };

    for (const auto& ft : tables) {
        CAPTURE(ft.id);
        CAPTURE(ft.frame);
        REQUIRE(ft.id < body->face_count());
        const auto& faces = body->solid.shell().faces();
        // The factory numbers faces by shell order == face id.
        REQUIRE(faces[ft.id].id() == ft.id);
        check_loop_lines(faces[ft.id], ft.lines, 4);
    }
}

TEST_CASE("G1-U2 golden: make_sphere(2) — the seam-wired slit rectangle, 2 Line2 pinned") {
    auto body = b::make_sphere(2.0f);
    REQUIRE(body != nullptr);
    REQUIRE(body->face_count() == 1);
    const auto& face = body->solid.shell().faces()[0];

    // The slit rectangle [0,2π]×[0,π]: the seam's forward copy is the
    // RIGHT edge (u=2π, walked north→south = v:0→π), the reversed
    // copy is the LEFT edge (u=0, walked south→north = v:π→0). Both
    // carry material on the left (the rectangle's interior).
    const PinnedLine pinned[] = {
        {0, false, {kTwoPi, 0.0f}, {kTwoPi, kPi}},
        {0, true, {0.0f, kPi}, {0.0f, 0.0f}},
    };
    check_loop_lines(face, pinned, 2);
}

TEST_CASE("G1-U2 golden: make_cylinder(1,2) — the wall's rectangle walk and the caps' half-arc pairs") {
    auto body = b::make_cylinder(1.0f, 2.0f);
    REQUIRE(body != nullptr);
    REQUIRE(body->face_count() == 3);
    const auto& faces = body->solid.shell().faces();

    // Face 0 — the wall, domain (u,v) = (θ, axis), window
    // [0,2π]×[−1,1]. The CCW walk: right seam UP, top WEST, left seam
    // DOWN, bottom EAST — the factory's existing loop order, with
    // e_top1 = the −Z half-arc and e_top2 = the +Z half-arc (the
    // corrected side assignment; see the file header).
    const PinnedLine wall[] = {
        {4, false, {kTwoPi, -1.0f}, {kTwoPi, 1.0f}},  // seam fwd (right edge, up)
        {1, true, {kTwoPi, 1.0f}, {kPi, 1.0f}},       // e_top2 rev (+Z half, west)
        {0, true, {kPi, 1.0f}, {0.0f, 1.0f}},         // e_top1 rev (−Z half, west)
        {4, true, {0.0f, 1.0f}, {0.0f, -1.0f}},       // seam rev (left edge, down)
        {3, true, {0.0f, -1.0f}, {kPi, -1.0f}},       // e_bot2 rev (−Z half, east)
        {2, true, {kPi, -1.0f}, {kTwoPi, -1.0f}},     // e_bot1 rev (+Z half, east)
    };
    check_loop_lines(faces[0], wall, 6);

    // Faces 1 and 2 — the caps, plane domains with the rim as a full
    // circle walked CCW: two π-sweep arcs, from v_top1/v_bot1's
    // domain position (0,−R) [angle −π/2] and v_top2/v_bot2's (0,R)
    // [angle +π/2]. Both caps' frames ((−Z,−X) and (Z,−X)) put the
    // rim at the same in-domain ANGLES — but the edge ids differ
    // (the top cap walks e_top1/e_top2, the bottom cap e_bot1/e_bot2).
    const PinnedArc top_cap[] = {
        {0, false, {0.0f, 0.0f}, 1.0f, false, -0.5f * kPi, kPi},
        {1, false, {0.0f, 0.0f}, 1.0f, false, 0.5f * kPi, kPi},
    };
    const PinnedArc bottom_cap[] = {
        {2, false, {0.0f, 0.0f}, 1.0f, false, -0.5f * kPi, kPi},
        {3, false, {0.0f, 0.0f}, 1.0f, false, 0.5f * kPi, kPi},
    };
    REQUIRE(faces[1].id() == 1);
    check_loop_arcs(faces[1], top_cap, 2);
    REQUIRE(faces[2].id() == 2);
    check_loop_arcs(faces[2], bottom_cap, 2);
}

TEST_CASE("G1-U2 golden: make_tetrahedron — 4 planar faces, 3 Line2 each (the fourth factory joins the discipline)") {
    auto body = b::make_tetrahedron();
    REQUIRE(body != nullptr);
    REQUIRE(body->face_count() == 4);
    for (const auto& face : body->solid.shell().faces()) {
        CAPTURE(face.id());
        REQUIRE(face.loops().size() == 1);
        const auto& coedges = face.loops()[0].coedges();
        REQUIRE(coedges.size() == 3);
        for (const auto& ce : coedges) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == b::PCurve::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == true);
        }
    }
}

// ---------------------------------------------------------------------------
// Family B — the compatibility proof: counts do not move, the
// faceted world stays p-curve-empty, the historical goldens load
// unchanged (the byte-identity of the files themselves is the
// delivery's git-status proof; here the materialized bodies keep
// their pinned topology class and gain nothing).
// ---------------------------------------------------------------------------

TEST_CASE("G1-U2 compat: the counts do not move — χ confirmed WITH the H term") {
    struct Pin {
        const char* name;
        std::size_t v, e, f;
    };
    const std::pair<Pin, std::unique_ptr<b::BRepBody>> bodies[] = {
        {Pin{"box", 8, 12, 6}, b::make_box(Vec3d{1, 2, 3})},
        {Pin{"sphere", 2, 1, 1}, b::make_sphere(2.0f)},
        {Pin{"cylinder", 4, 5, 3}, b::make_cylinder(1.0f, 2.0f)},
        {Pin{"tetrahedron", 4, 6, 4}, b::make_tetrahedron()},
    };
    for (const auto& entry : bodies) {
        // Binding members, not structured bindings — the clang-14
        // CAPTURE lesson (P1091 lands in clang 16).
        CAPTURE(entry.first.name);
        REQUIRE(entry.second != nullptr);
        CHECK(entry.second->vertex_count() == entry.first.v);
        CHECK(entry.second->edge_count() == entry.first.e);
        CHECK(entry.second->face_count() == entry.first.f);
        // Every face is disk-faced: H = 0, and χ reads the classical
        // 2 — now CONFIRMED through the extended term, not assumed.
        CHECK(b::inner_loop_count(*entry.second) == 0);
        CHECK(b::euler_characteristic(*entry.second) == 2);
        CHECK(b::is_closed_orientable(*entry.second));
        CHECK(b::loops_are_closed(*entry.second));
    }
}

TEST_CASE("G1-U2 compat: the faceted world still writes no p-curves — bridge extracts and golden bodies") {
    // The bridge's faceted path (plane-per-triangle): the per-world
    // invariant of D2.2 — the field stays empty, the provenance says
    // approximated. Unchanged by this unit, pinned again because the
    // boundary moved on the factory side.
    {
        s::SDFBody blend =
            s::sdf_smooth_union(s::make_sphere(0.8f), s::make_sphere(0.6f), 0.2f);
        br::SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto faceted = br::sdf_to_brep(blend, opts);
        REQUIRE(faceted != nullptr);
        for (const auto& face : faceted->solid.shell().faces())
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges())
                    CHECK(ce.p_curve() == nullptr);
    }

    // The four historical goldens: they load, they materialize through
    // the document's own path (the bridge's faceted extraction), their
    // topology class is unchanged (the io/bridge suites pin the full
    // tables), and the p-curve field stays empty on every body — the
    // exact primitives' new content must not leak into the faceted
    // world's serialization or its bridge path.
    const char* goldens[] = {"native_v0_1_0.CAD_0", "native_v0_2_0.CAD_0",
                             "native_v0_3_0.CAD_0", "native_v0_4_0.CAD_0"};
    for (const char* name : goldens) {
        CAPTURE(name);
        const auto path = golden_file(name);
        const std::string missing = "golden file missing: " + path.string();
        REQUIRE_MESSAGE(std::filesystem::exists(path), missing);
        // v0.1.0 is pure JSON — it goes through parse_document (the
        // same dispatch unit 1's golden test uses); the rest load.
        const std::string fname = name;
        const auto doc = fname == "native_v0_1_0.CAD_0"
            ? CAD_0::io::native::parse_document(read_file(path))
            : CAD_0::io::native::load_file(path);
        for (const auto& node : doc.nodes) {
            if (node.body.root() == nullptr)
                continue;  // mesh-only / kernel-body-less node
            br::SdfToBrepOptions opts;
            opts.march_resolution = 16;
            opts.fit_primitives = false;  // the document path: faceted
            auto body = br::sdf_to_brep(node.body, opts);
            REQUIRE(body != nullptr);
            CHECK(b::inner_loop_count(*body) == 0);
            CHECK(b::euler_characteristic(*body) ==
                  static_cast<std::int64_t>(body->vertex_count()) -
                      static_cast<std::int64_t>(body->edge_count()) +
                      static_cast<std::int64_t>(body->face_count()));
            for (const auto& face : body->solid.shell().faces())
                for (const auto& loop : face.loops())
                    for (const auto& ce : loop.coedges())
                        CHECK(ce.p_curve() == nullptr);
        }
    }
}

// ---------------------------------------------------------------------------
// Family C — the provenance, enumerated per primitive AND per face
// through the query API (the D8 contract at its consumption point).
// ---------------------------------------------------------------------------

TEST_CASE("G1-U2 provenance: every face of every factory reads exact/0 through the query") {
    const std::pair<const char*, std::unique_ptr<b::BRepBody>> bodies[] = {
        {"make_box", b::make_box(Vec3d{1, 2, 3})},
        {"make_sphere", b::make_sphere(2.0f)},
        {"make_cylinder", b::make_cylinder(1.0f, 2.0f)},
        {"make_tetrahedron", b::make_tetrahedron()},
    };
    for (auto& entry : bodies) {
        CAPTURE(entry.first);
        const auto infos = b::query_all_faces(*entry.second);
        REQUIRE(infos.size() == entry.second->face_count());
        for (const auto& info : infos) {
            CHECK(info.exact);
            CHECK(info.linear_tolerance == 0.0);
        }
    }
}

// ---------------------------------------------------------------------------
// Family D — the conventions, tested directly (see check_conventions)
// ---------------------------------------------------------------------------

TEST_CASE("G1-U2 conventions: the box — direction, chaining, region side on all 6 faces") {
    auto body = b::make_box(Vec3d{1, 2, 3});
    REQUIRE(body != nullptr);
    for (const auto& face : body->solid.shell().faces()) {
        CAPTURE(face.id());
        // Every box edge is a real boundary — no seams on planes.
        const std::vector<bool> no_seams(face.loops()[0].coedges().size(),
                                         false);
        check_conventions(*body, face, polygon_from_corners(face), no_seams);
    }
}

TEST_CASE("G1-U2 conventions: the sphere — the periodic seam chains and the region side holds") {
    auto body = b::make_sphere(2.0f);
    REQUIRE(body != nullptr);
    RegionSpec region;
    region.umin = 0;
    region.umax = kTwoPi;
    region.vmin = 0;
    region.vmax = kPi;
    region.periodic_u = true;
    // Both co-edges are seam copies: each one's right side is the
    // neighbouring window's material (inside, modulo the period).
    check_conventions(*body, body->solid.shell().faces()[0], region,
                      {true, true});
}

TEST_CASE("G1-U2 conventions: the cylinder — wall and caps, with the seam pair identified") {
    auto body = b::make_cylinder(1.0f, 2.0f);
    REQUIRE(body != nullptr);
    const auto& faces = body->solid.shell().faces();

    // The wall: window [0,2π]×[−1,1], u periodic. The seam pair is
    // the loop's co-edges 0 and 3 (seam fwd on the right edge, seam
    // rev on the left); the four rim co-edges are real boundaries.
    {
        RegionSpec region;
        region.umin = 0;
        region.umax = kTwoPi;
        region.vmin = -1;
        region.vmax = 1;
        region.periodic_u = true;
        check_conventions(*body, faces[0], region,
                          {true, false, false, true, false, false});
    }

    // The caps: the disc region, all real boundaries.
    for (std::size_t fi = 1; fi <= 2; ++fi) {
        CAPTURE(fi);
        RegionSpec region;
        region.disc = true;
        region.radius = 1.0f;
        check_conventions(*body, faces[fi], region, {false, false});
    }
}

TEST_CASE("G1-U2 conventions: the tetrahedron — the same discipline on the fourth factory") {
    auto body = b::make_tetrahedron();
    REQUIRE(body != nullptr);
    for (const auto& face : body->solid.shell().faces()) {
        CAPTURE(face.id());
        const std::vector<bool> no_seams(face.loops()[0].coedges().size(),
                                         false);
        check_conventions(*body, face, polygon_from_corners(face), no_seams);
    }
}

// ---------------------------------------------------------------------------
// Family E — the query surface itself (the H1 contract taking shape):
// plain values per face, honest on both worlds, loud on bad input.
// ---------------------------------------------------------------------------

TEST_CASE("G1-U2 query: the box through query_face — family, loops, per-co-edge p-curves") {
    auto body = b::make_box(Vec3d{1, 2, 3});
    REQUIRE(body != nullptr);

    const auto q0 = b::query_face(*body, 0);
    CHECK(q0.family == "plane");
    CHECK(q0.has_surface);
    CHECK(q0.loop_count == 1);
    CHECK(q0.outer_loop_count == 1);
    CHECK(q0.inner_loop_count == 0);
    CHECK(q0.exact);
    CHECK(q0.linear_tolerance == 0.0);
    REQUIRE(q0.loops.size() == 1);
    REQUIRE(q0.loops[0].coedges.size() == 4);
    // The first co-edge of the bottom face, read as plain values:
    // edge 3 reversed, Line2 from (−1,2) to (−1,−2), material left.
    const auto& c0 = q0.loops[0].coedges[0];
    CHECK(c0.edge == 3);
    CHECK(c0.reversed);
    CHECK(c0.kind == b::PCurveInfo::Kind::Line);
    CHECK(c0.interior_left);
    CHECK(close2(c0.a, Vec2d{-1, 2}));
    CHECK(close2(c0.b, Vec2d{-1, -2}));

    // Every box face reports the same shape of answer.
    const auto all = b::query_all_faces(*body);
    REQUIRE(all.size() == 6);
    for (const auto& q : all) {
        CHECK(q.family == "plane");
        CHECK(q.loop_count == 1);
        CHECK(q.outer_loop_count == 1);
        CHECK(q.inner_loop_count == 0);
        for (const auto& loop : q.loops)
            for (const auto& ce : loop.coedges)
                CHECK(ce.kind == b::PCurveInfo::Kind::Line);
    }
}

TEST_CASE("G1-U2 query: the cylinder and sphere — families and kinds per face") {
    {
        auto body = b::make_cylinder(1.0f, 2.0f);
        REQUIRE(body != nullptr);
        const auto wall = b::query_face(*body, 0);
        CHECK(wall.family == "cylinder");
        CHECK(wall.loop_count == 1);
        CHECK(wall.outer_loop_count == 1);
        CHECK(wall.inner_loop_count == 0);
        for (const auto& loop : wall.loops)
            for (const auto& ce : loop.coedges)
                CHECK(ce.kind == b::PCurveInfo::Kind::Line);

        for (const b::FaceId cap_id : {b::FaceId{1}, b::FaceId{2}}) {
            CAPTURE(cap_id);
            const auto cap = b::query_face(*body, cap_id);
            CHECK(cap.family == "plane");
            CHECK(cap.loop_count == 1);
            REQUIRE(cap.loops.size() == 1);
            REQUIRE(cap.loops[0].coedges.size() == 2);
            for (const auto& ce : cap.loops[0].coedges) {
                CHECK(ce.kind == b::PCurveInfo::Kind::Arc);
                CHECK(close(ce.radius, 1.0f));
                CHECK_FALSE(ce.hole);
                CHECK(close(ce.sweep, kPi));
            }
        }
    }
    {
        auto body = b::make_sphere(2.0f);
        REQUIRE(body != nullptr);
        const auto q = b::query_face(*body, 0);
        CHECK(q.family == "sphere");
        CHECK(q.loop_count == 1);
        REQUIRE(q.loops.size() == 1);
        REQUIRE(q.loops[0].coedges.size() == 2);
        for (const auto& ce : q.loops[0].coedges)
            CHECK(ce.kind == b::PCurveInfo::Kind::Line);
        // The seam pair's domain positions, as plain values: the
        // forward copy on the window's right edge, the reversed copy
        // on the left.
        const auto& fwd = q.loops[0].coedges[0];
        const auto& rev = q.loops[0].coedges[1];
        CHECK(close2(fwd.a, Vec2d{kTwoPi, 0.0f}));
        CHECK(close2(fwd.b, Vec2d{kTwoPi, kPi}));
        CHECK(close2(rev.a, Vec2d{0.0f, kPi}));
        CHECK(close2(rev.b, Vec2d{0.0f, 0.0f}));
    }
}

TEST_CASE("G1-U2 query: bad face id is loud — invalid_argument naming the remedy") {
    auto body = b::make_box(Vec3d{1, 1, 1});
    REQUIRE(body != nullptr);
    CHECK_THROWS_AS(b::query_face(*body, 99), std::invalid_argument);
    CHECK_THROWS_AS(b::query_face(*body, 6), std::invalid_argument);
    CHECK_NOTHROW(b::query_face(*body, 5));
}

TEST_CASE("G1-U2 query: the faceted world reads honestly — family plane, no p-curves, approximated") {
    s::SDFBody blend =
        s::sdf_smooth_union(s::make_sphere(0.8f), s::make_sphere(0.6f), 0.2f);
    br::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto faceted = br::sdf_to_brep(blend, opts);
    REQUIRE(faceted != nullptr);

    const auto infos = b::query_all_faces(*faceted);
    REQUIRE(infos.size() == faceted->face_count());
    for (const auto& q : infos) {
        CHECK(q.family == "plane");
        CHECK(q.loop_count == 1);
        CHECK(q.outer_loop_count == 1);
        CHECK_FALSE(q.exact);
        CHECK(q.linear_tolerance > 0.0);
        for (const auto& loop : q.loops)
            for (const auto& ce : loop.coedges)
                CHECK(ce.kind == b::PCurveInfo::Kind::None);
    }
}
