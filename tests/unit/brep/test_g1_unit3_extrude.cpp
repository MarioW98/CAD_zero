// tests/unit/brep/test_g1_unit3_extrude.cpp
//
// G1 UNIT 3 — THE EXACT EXTRUDE (ADR-0032 D6, the discriminating
// tests, written FIRST: every family here failed on the pre-unit tree
// for the right reason — the ABSENCE of brep::make_prism,
// CylinderZSurface and the SceneNode hybrid landing; the probe build
// named them).
//
// The unit's claim under test, per the order's four families:
//
//   * GEOMETRY — extruding a circle produces a body whose lateral
//     face is ONE cylindrical face (the exact world); the faceted
//     bridge of the SAME profile's SDF yields N planar strips with no
//     p-curves and a declared tolerance (the today world, for
//     contrast — both honest, only one exact);
//   * COUNTS — the two pinned formulas: a closed chain of n entities
//     → V = 2n, E = 3n, F = n+2, χ = 2 for EVERY n (the box is the
//     n = 4 case); the circle profile → the ADR-0023 seam counts
//     4V/5E/3F. The multi-chain extension (the ring — a TESTED flow
//     of the SDF extrude) is pinned too: each through-hole adds its
//     wall and one inner loop per cap, χ = 2 − 2·(holes);
//   * THE CAPS — the sketch chain IS the cap's outer trimming loop,
//     typed entity for entity (segment→Line2, arc→Arc2, circle→
//     Circle2 on the hole rims — the first Circle2 writer), every
//     written subshape reading exact/0 through the query;
//   * THE CONVENTIONS, DIRECTLY (no interpretation) — DIRECTION: the
//     p-curve's evaluated start/end are the co-edge's start/end
//     vertices' images (± the u period on seam copies); CHAINING:
//     consecutive p-curves chain endpoint-to-endpoint; REGION SIDE:
//     the material is on the side the ENTITY'S OWN BIT declares
//     (interior_left / !hole) — the unit-2 checker's ε-left model
//     extended to authored bits, exactly the sketch kernel's own
//     region semantics ("the same mathematics"); FRAMES: every
//     surface's (∂u×∂v)·n_outward > 0 at sampled points, the
//     CylinderZSurface u-sign included;
//   * THE HYBRIDBODY LANDING (ADR-0031 D6) — the extrude node owns
//     BOTH materializations (the SDF twin + the exact B-Rep) with
//     bridge tolerance 0; the SDF keeps feeding display unchanged;
//     the subtract downstream consumes the SDF side and its bridge
//     extract stays faceted/approximated (the honesty chain — the
//     prism says exact, the subtracted body says approximated, both
//     the truth); the out-of-class profile (a notch-side arc) keeps
//     its SDF materialization with the exact side's refusal RECORDED,
//     never guessed;
//   * DETERMINISM (ADR-0014) — a radius edit cycle regenerates BOTH
//     materializations bit-identically; the v0.4 golden re-loads and
//     re-materializes the exact prism identically.
//
// The pinned tables document the builder's numbering discipline
// (ADR-0005: stable ids by construction order, the factories' rule) —
// extrude.hpp's header carries the same tables as the authoring
// contract; a drift between the two fails HERE first.

#include <doctest/doctest.h>

#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/io/native.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3f;

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717958647692f;

bool close(float a, float b, float tol = 1e-5f) {
    return std::abs(a - b) <= tol;
}
bool close2(const Vec2f& a, const Vec2f& b, float tol = 1e-5f) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol);
}
bool close3(const Vec3f& a, const Vec3f& b, float tol = 1e-5f) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol) &&
           close(a.z, b.z, tol);
}

// ---- the profiles (the sketch kernel's own vocabulary, authored as
// the canvas does: interior on the left of the walk, hole = false) ----

// The unit rectangle: CCW, 4 segments, interior_left = true (n = 4 —
// the box-equivalent). Joint J_i = entity i's END:
//   J0=(2,-1)  J1=(2,1)  J2=(-2,1)  J3=(-2,-1)
std::vector<s::SketchChain> rect_chains(Vec2f lo = {-2.0f, -1.0f},
                                        Vec2f hi = {2.0f, 1.0f}) {
    return {s::SketchChain{{
        s::sketch_segment(lo, {hi.x, lo.y}, true),          // bottom, east
        s::sketch_segment({hi.x, lo.y}, hi, true),          // right, north
        s::sketch_segment(hi, {lo.x, hi.y}, true),          // top, west
        s::sketch_segment({lo.x, hi.y}, lo, true),          // left, south
    }}};
}

// The circle profile: a single circle chain, hole = false (the solid
// disc — the guide case's tool).
std::vector<s::SketchChain> circle_chains(Vec2f center = {0.5f, 0.25f},
                                          float radius = 1.25f) {
    return {s::SketchChain{{s::sketch_circle(center, radius, false)}}};
}

// The ring profile (the TESTED multi-chain flow — test_sketch_scene's
// ring_chains): the rectangle + a hole circle inside.
std::vector<s::SketchChain> ring_chains() {
    auto chains = rect_chains();
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.2f, true)}});
    return chains;
}

// The sector: 2 segments from the centre + the closing arc
// (n = 3, segments + arc — the mixed polygon). CCW, interior on the
// left, the arc a convex fillet (hole = false, the disc side).
std::vector<s::SketchChain> sector_chains() {
    // Centre C=(0,0), r=1.5, from angle a0=-0.2π to a1=+0.35π
    // (nothing special — the pins must hold for ANY span).
    const float a0 = -0.2f * kPi, a1 = 0.35f * kPi;
    const float r = 1.5f;
    const Vec2f c{0.0f, 0.0f};
    const Vec2f ps = c + Vec2f{r * std::cos(a0), r * std::sin(a0)};
    const Vec2f pe = c + Vec2f{r * std::cos(a1), r * std::sin(a1)};
    return {s::SketchChain{{
        s::sketch_segment(c, ps, true),                     // radius out
        s::sketch_arc(c, r, false, a0, a1 - a0),            // the rim
        s::sketch_segment(pe, c, true),                     // radius home
    }}};
}

// The lens: two arcs sharing both endpoints (n = 2, arcs only — the
// formula's smallest case). The region is disc1 ∩ disc2.
std::vector<s::SketchChain> lens_chains() {
    // Two circles of radius 2 crossing at A=(0,1.6) and B=(0,-1.6)
    // (centres (±1.2,0): the crossings sit at y = ±sqrt(4−1.44)):
    // arc1 on c1 from B CCW to A (the right side of c1), arc2 on c2
    // from A CCW to B (the left side of c2). The spans are
    // normalized CCW by the sketch vocabulary's own rule (sweep in
    // (0, 2π)).
    const float r = 2.0f;
    const Vec2f c1{-1.2f, 0.0f}, c2{1.2f, 0.0f};
    const float y = std::sqrt(r * r - 1.2f * 1.2f);
    const Vec2f a{0.0f, y}, bb{0.0f, -y};
    const float ang_b1 = std::atan2(bb.y - c1.y, bb.x - c1.x);
    const float ang_a1 = std::atan2(a.y - c1.y, a.x - c1.x);
    float span1 = ang_a1 - ang_b1;
    if (span1 < 0.0f) span1 += kTwoPi;
    const float ang_a2 = std::atan2(a.y - c2.y, a.x - c2.x);
    const float ang_b2 = std::atan2(bb.y - c2.y, bb.x - c2.x);
    float span2 = ang_b2 - ang_a2;
    if (span2 < 0.0f) span2 += kTwoPi;
    return {s::SketchChain{{
        s::sketch_arc(c1, r, false, ang_b1, span1),
        s::sketch_arc(c2, r, false, ang_a2, span2),
    }}};
}

// The notch profile (OUT of the exact class, IN the SDF class): the
// square [0,3]x[0,2] with a bite at the top-right corner — walked
// CLOCKWISE around the material (all segments interior_left = false,
// the material on the right), the bite arc walked CCW (the chain's
// only direction through an arc) with the material on its right
// (hole = true, the complement). The SDF materializes it today; the
// exact B-Rep refuses it (the typed 2D vocabulary stores CCW sweeps
// only — a notch arc's cap trimming runs CW in BOTH cap domains).
std::vector<s::SketchChain> notch_chains() {
    // The bite: circle centre (3,2) r=1.4. The chain enters the arc
    // at (1.6,2) (angle π) and leaves at (3,0.6) (angle 3π/2),
    // sweeping CCW through the square's interior.
    return {s::SketchChain{{
        s::sketch_segment({0.0f, 0.0f}, {0.0f, 2.0f}, false),   // left, up
        s::sketch_segment({0.0f, 2.0f}, {1.6f, 2.0f}, false),   // top, east
        s::sketch_arc({3.0f, 2.0f}, 1.4f, true, kPi, 0.5f * kPi),
        s::sketch_segment({3.0f, 0.6f}, {3.0f, 0.0f}, false),   // right, down
        s::sketch_segment({3.0f, 0.0f}, {0.0f, 0.0f}, false),   // bottom, west
    }}};
}

// The extrude recipe helper (the C++ surface the Python binding
// wraps: add_extrude_node builds this recipe and calls
// add_operation_node).
std::uint32_t add_extrude(CAD_0::scene::Scene& sc, const std::string& name,
                          std::uint32_t sketch, float distance,
                          int direction) {
    CAD_0::scene::OperationRecipe r;
    r.type = CAD_0::scene::OperationType::Extrude;
    r.sketch = sketch;
    r.distance = distance;
    r.direction = direction;
    return sc.add_operation_node(name, r);
}

// ---- the conventions checker (unit 2's, extended to authored bits) --

Vec2f pcurve_start(const b::PCurve& p) {
    switch (p.kind) {
    case s::SketchEntity::Kind::Segment: return p.segment.a;
    case s::SketchEntity::Kind::Arc:     return s::arc_start_point(p.arc);
    case s::SketchEntity::Kind::Circle:  return p.circle.center;
    }
    return {};
}
Vec2f pcurve_end(const b::PCurve& p) {
    switch (p.kind) {
    case s::SketchEntity::Kind::Segment: return p.segment.b;
    case s::SketchEntity::Kind::Arc:     return s::arc_end_point(p.arc);
    case s::SketchEntity::Kind::Circle:  return p.circle.center;
    }
    return {};
}
Vec2f pcurve_mid(const b::PCurve& p) {
    switch (p.kind) {
    case s::SketchEntity::Kind::Segment:
        return Vec2f{(p.segment.a.x + p.segment.b.x) * 0.5f,
                     (p.segment.a.y + p.segment.b.y) * 0.5f};
    case s::SketchEntity::Kind::Arc: {
        const float t = p.arc.start_angle + p.arc.sweep * 0.5f;
        return p.arc.center + Vec2f{p.arc.radius * std::cos(t),
                                    p.arc.radius * std::sin(t)};
    }
    case s::SketchEntity::Kind::Circle:
        return p.circle.center + Vec2f{p.circle.radius, 0.0f};
    }
    return {};
}

// The entity's declared material side: LEFT iff (interior_left on a
// segment) or (!hole on a circle/arc) — the sketch kernel's own
// region semantics, read from the payload verbatim.
bool material_left(const b::PCurve& p) {
    switch (p.kind) {
    case s::SketchEntity::Kind::Segment: return p.segment.interior_left;
    case s::SketchEntity::Kind::Circle:  return !p.circle.hole;
    case s::SketchEntity::Kind::Arc:     return !p.arc.hole;
    }
    return false;
}

// One p-curve's traversal tangent at the midpoint. The Arc2's tangent
// is the CCW sweep direction (the vocabulary stores CCW sweeps only);
// the Circle2's is CCW at the checkpoint (0° — the same point
// pcurve_mid uses).
Vec2f tangent(const b::PCurve& p) {
    switch (p.kind) {
    case s::SketchEntity::Kind::Segment: {
        Vec2f d = p.segment.b - p.segment.a;
        const float len = d.length();
        if (len > 0) d = Vec2f{d.x / len, d.y / len};
        return d;
    }
    case s::SketchEntity::Kind::Arc: {
        const float t = p.arc.start_angle + p.arc.sweep * 0.5f;
        return Vec2f{-std::sin(t), std::cos(t)};
    }
    case s::SketchEntity::Kind::Circle:
        return Vec2f{0.0f, 1.0f};
    }
    return {1.0f, 0.0f};
}
Vec2f offset_to_material(const b::PCurve& p, float eps) {
    // The side the entity's region lies on: left of the tangent iff
    // material_left(p), else right — the extended convention.
    const Vec2f t = tangent(p);
    const Vec2f left{-t.y, t.x};
    const float side = material_left(p) ? 1.0f : -1.0f;
    const Vec2f m = pcurve_mid(p);
    return Vec2f{m.x + side * left.x * eps, m.y + side * left.y * eps};
}
Vec2f offset_away_from_material(const b::PCurve& p, float eps) {
    const Vec2f t = tangent(p);
    const Vec2f left{-t.y, t.x};
    const float side = material_left(p) ? -1.0f : 1.0f;
    const Vec2f m = pcurve_mid(p);
    return Vec2f{m.x + side * left.x * eps, m.y + side * left.y * eps};
}

// The face's region, as the checker's per-family model. The prism's
// faces: projected convex polygons (the segment lateral faces), (u,v)
// windows with optional periodicity (the arc walls, the circle wall,
// the hole walls), and cap regions = the sketch profile's own region
// (composed by the 2D kernel — ONE opinion, no second copy of the
// region mathematics). A cap's domain is the sketch's XY under the
// cap plane's frame (the -Z cap mirrors y, the +Z cap rotates by
// π): profile samples map the domain point BACK to sketch
// coordinates first — the region is the same mathematics, read in
// its own frame.
struct RegionSpec {
    // Convex CCW polygon in the domain (walk order).
    std::vector<Vec2f> polygon{};
    // Window (possibly u-periodic): [umin,umax] x [vmin,vmax].
    bool window{false};
    bool periodic_u{false};
    float umin{0}, umax{0}, vmin{0}, vmax{0};
    // Cap: the region is the 2D profile's, in the sketch's own
    // coordinates — reached from the domain through `to_sketch`.
    const s::ProfileSDF* profile{nullptr};
    enum class SketchMap { Identity, MirrorY, Rot180 };
    SketchMap to_sketch{SketchMap::Identity};

    bool contains(const Vec2f& p) const {
        if (profile != nullptr) {
            Vec2f sp = p;
            switch (to_sketch) {
            case SketchMap::MirrorY: sp = Vec2f{p.x, -p.y}; break;
            case SketchMap::Rot180:  sp = Vec2f{-p.x, -p.y}; break;
            case SketchMap::Identity: break;
            }
            return profile->sample(sp).value < 0.0f;
        }
        if (window) {
            float u = p.x;
            if (periodic_u) {
                const float w = umax - umin;
                u = std::fmod(u - umin, w);
                if (u < 0) u += w;
                u += umin;
            }
            return u >= umin - 1e-4f && u <= umax + 1e-4f &&
                   p.y >= vmin - 1e-4f && p.y <= vmax + 1e-4f;
        }
        const std::size_t n = polygon.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2f& a = polygon[i];
            const Vec2f& b = polygon[(i + 1) % n];
            const float cross =
                (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            if (cross < -1e-5f) return false;
        }
        return true;
    }
};

// DIRECTION + CHAINING + REGION SIDE for ONE loop of a face, given
// the face's surface (for the DIRECTION check) and its region.
// `is_seam` marks the periodic seam copies (empty = none): a seam is
// NOT a boundary of the region — it is where the window was slit,
// and its "away" side is the neighbouring period's material, inside
// the region after the identification (unit 2's checker discipline,
// verbatim).
void check_loop_conventions(const b::BRepBody& body, const b::Face& face,
                            const b::Loop& loop, const RegionSpec& region,
                            const b::SurfaceGeometry* surf,
                            bool periodic_u, float period,
                            const std::vector<bool>& is_seam) {
    const float eps = 1e-3f;
    const auto& coedges = loop.coedges();
    const std::size_t n = coedges.size();
    REQUIRE(n > 0);

    for (std::size_t i = 0; i < n; ++i) {
        const b::CoEdge& ce = coedges[i];
        CAPTURE(i);
        CAPTURE(ce.edge());
        REQUIRE(ce.p_curve() != nullptr);
        const b::PCurve& pc = *ce.p_curve();

        // A Circle2 is a CLOSED trim curve: it has no start/end to
        // chain and no traversal direction to pin (the co-edge's
        // orientation flag still obeys the radial XOR, checked by
        // is_closed_orientable). Its REGION SIDE is checked.
        if (pc.kind == s::SketchEntity::Kind::Circle) {
            CHECK(region.contains(offset_to_material(pc, eps)));
            CHECK_FALSE(region.contains(offset_away_from_material(pc, eps)));
            continue;
        }

        // DIRECTION — the p-curve's evaluated start/end are the
        // co-edge's start/end vertex images (± the u period).
        const Vec3f vstart =
            body.vertices[static_cast<std::size_t>(
                ce.start_vertex(body.edges))].position();
        const Vec3f vend =
            body.vertices[static_cast<std::size_t>(
                ce.end_vertex(body.edges))].position();
        REQUIRE(surf != nullptr);
        if (pc.kind == s::SketchEntity::Kind::Segment) {
            const Vec3f es = surf->evaluate(pc.segment.a.x, pc.segment.a.y);
            const Vec3f ee = surf->evaluate(pc.segment.b.x, pc.segment.b.y);
            CHECK(close3(es, vstart));
            CHECK(close3(ee, vend));
        } else {  // Arc
            const Vec2f as = s::arc_start_point(pc.arc);
            const Vec2f ae = s::arc_end_point(pc.arc);
            CHECK(close3(surf->evaluate(as.x, as.y), vstart));
            CHECK(close3(surf->evaluate(ae.x, ae.y), vend));
        }

        // CHAINING — p-curve i's end meets p-curve (i+1 mod n)'s
        // start (Circle2 neighbours are skipped: a closed trim curve
        // has no endpoints; the loop's closure for a Circle2-only
        // loop is structural — loops_are_closed).
        const b::CoEdge& next = coedges[(i + 1) % n];
        if (next.p_curve() != nullptr &&
            next.p_curve()->kind != s::SketchEntity::Kind::Circle &&
            pc.kind != s::SketchEntity::Kind::Circle) {
            const Vec2f e = pcurve_end(pc);
            const Vec2f ns = pcurve_start(*next.p_curve());
            if (periodic_u) {
                const bool chains =
                    close2(e, ns) ||
                    (close(e.x, ns.x + period) && close(e.y, ns.y)) ||
                    (close(e.x, ns.x - period) && close(e.y, ns.y));
                CHECK(chains);
            } else {
                CHECK(close2(e, ns));
            }
        }

        // REGION SIDE — the entity's declared side is inside the
        // face's region; the other side is outside (for a cap's inner
        // loop, the away-side falls in the hole — outside the
        // profile's material — the same check), EXCEPT on seam
        // copies, where the away side is the neighbouring period's
        // material (inside after the identification).
        CHECK(region.contains(offset_to_material(pc, eps)));
        const bool seam = i < is_seam.size() && is_seam[i];
        if (!seam)
            CHECK_FALSE(region.contains(offset_away_from_material(pc, eps)));
    }
}

// The full-face conventions: every loop, plus the frame rule on the
// surface ((du x dv) . n > 0 at sampled points, verified numerically
// through evaluate(); the outward_hint cross-checks the analytic
// normal's outwardness at those same points).
void check_face_conventions(const b::BRepBody& body, const b::Face& face,
                            const std::vector<RegionSpec>& loop_regions,
                            const std::vector<Vec2f>& domain_samples,
                            const Vec3f& outward_hint,
                            const std::vector<bool>& seam_flags = {}) {
    const auto* surf = face.surface();
    REQUIRE(surf != nullptr);
    const std::size_t nl = face.loops().size();
    REQUIRE(loop_regions.size() == nl);

    for (const Vec2f& p : domain_samples) {
        const Vec3f pu = surf->evaluate(p.x + 1e-3f, p.y);
        const Vec3f pm = surf->evaluate(p.x, p.y);
        const Vec3f pv = surf->evaluate(p.x, p.y + 1e-3f);
        const Vec3f du{pu.x - pm.x, pu.y - pm.y, pu.z - pm.z};
        const Vec3f dv{pv.x - pm.x, pv.y - pm.y, pv.z - pm.z};
        const Vec3f cross = du.cross(dv);
        const Vec3f nrm = surf->normal(p.x, p.y);
        CHECK(cross.dot(nrm) > 0.0f);
        CHECK(nrm.dot(outward_hint) > 0.0f);
    }

    bool periodic = false;
    float period = 0.0f;
    const std::string fam = surf->type_name();
    if (fam == "cylinder" || fam == "sphere")
        periodic = true, period = kTwoPi;

    for (std::size_t li = 0; li < nl; ++li)
        check_loop_conventions(body, face, face.loops()[li],
                               loop_regions[li], surf, periodic, period,
                               seam_flags);
}

// The structural invariants, one call per body.
void check_body_invariants(const b::BRepBody& body, std::size_t V,
                           std::size_t E, std::size_t F, std::int64_t H,
                           std::int64_t chi) {
    REQUIRE(body.vertex_count() == V);
    REQUIRE(body.edge_count() == E);
    REQUIRE(body.face_count() == F);
    CHECK(b::inner_loop_count(body) == H);
    CHECK(b::euler_characteristic(body) == chi);
    CHECK(b::loops_are_closed(body));
    CHECK(b::is_closed_orientable(body));
    // Every edge manifold: exactly 2 co-edges (the seam edges carry
    // their pair WITHIN one loop — the ADR-0023 pattern).
    for (const auto& e : body.edges)
        CHECK(e.coedges().size() == 2);
}

// Every subshape reads exact/0 — the Layer-2 enumeration (the unit's
// writer is the extrude: the factories' discipline).
void check_exact_provenance(const b::BRepBody& body) {
    for (const auto& v : body.vertices) {
        CHECK(v.provenance().is_exact());
        CHECK(v.provenance().linear_tolerance == 0.0);
    }
    for (const auto& e : body.edges) {
        CHECK(e.provenance().is_exact());
        CHECK(e.provenance().linear_tolerance == 0.0);
    }
    for (const auto& f : body.solid.shell().faces()) {
        CHECK(f.provenance().is_exact());
        CHECK(f.provenance().linear_tolerance == 0.0);
        // Every co-edge of every loop carries a typed p-curve — the
        // trimming contract (the faceted world carries none).
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges())
                CHECK(ce.p_curve() != nullptr);
    }
}

// Append one float with full precision (hex — bit-exact round-trip;
// ADR-0014's quantization: two materializations are identical iff
// every field re-serializes to the same bytes).
void append_float(std::string& out, float v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", static_cast<double>(v));
    out += buf;
}
void append_vec2(std::string& out, const Vec2f& v) {
    out += '(';
    append_float(out, v.x);
    out += ',';
    append_float(out, v.y);
    out += ')';
}

// Flatten a body's full query answer into a comparable string — the
// determinism checks compare TWO materializations through it (every
// family, loop, co-edge, p-curve parameter, provenance field).
std::string query_digest(const b::BRepBody& body) {
    std::string out;
    for (const auto& fi : b::query_all_faces(body)) {
        out += "[F ";
        out += fi.family;
        append_float(out, static_cast<float>(fi.loop_count));
        append_float(out, static_cast<float>(fi.inner_loop_count));
        append_float(out, fi.exact ? 1.0f : 0.0f);
        append_float(out, static_cast<float>(fi.linear_tolerance));
        for (const auto& loop : fi.loops) {
            out += "(L ";
            append_float(out, loop.is_outer ? 1.0f : 0.0f);
            for (const auto& ce : loop.coedges) {
                out += "<";
                append_float(out, static_cast<float>(ce.edge));
                append_float(out, ce.reversed ? 1.0f : 0.0f);
                append_float(out, static_cast<float>(
                                      static_cast<int>(ce.kind)));
                append_vec2(out, ce.a);
                append_vec2(out, ce.b);
                append_float(out, ce.interior_left ? 1.0f : 0.0f);
                append_vec2(out, ce.center);
                append_float(out, ce.radius);
                append_float(out, ce.hole ? 1.0f : 0.0f);
                append_float(out, ce.start_angle);
                append_float(out, ce.sweep);
                out += ">";
            }
            out += ")";
        }
        out += "]";
    }
    return out;
}

// The SDF side's digest: pinned sample values over the body's bounds
// (the display materialization's determinism).
std::string sdf_digest(const s::SDFBody& body) {
    std::string out;
    const auto bb = body.bounds();
    for (int i = 0; i <= 4; ++i)
        for (int j = 0; j <= 4; ++j)
            for (int k = 0; k <= 4; ++k) {
                const Vec3f p{
                    bb.min.x + (bb.max.x - bb.min.x) * static_cast<float>(i) / 4.0f,
                    bb.min.y + (bb.max.y - bb.min.y) * static_cast<float>(j) / 4.0f,
                    bb.min.z + (bb.max.z - bb.min.z) * static_cast<float>(k) / 4.0f};
                append_float(out, body.sample(p).value);
            }
    return out;
}

// The extrude node of a scene, by recipe type (the golden's loader
// test walks the nodes; the id is not assumed).
const CAD_0::scene::SceneNode* find_extrude_node(
    const CAD_0::scene::Scene& sc) {
    for (const auto& n : sc.nodes())
        if (n.operation &&
            n.operation->type == CAD_0::scene::OperationType::Extrude)
            return &n;
    return nullptr;
}

} // namespace

// ===========================================================================
// The counts — the two pinned formulas + the multi-chain extension
// ===========================================================================

TEST_CASE("G1-U3 counts: a closed chain of n entities → V=2n, E=3n, F=n+2, χ=2 for every n") {
    // n = 4 — the box-equivalent (the order's own case: the box IS
    // the n=4 prism; make_box's counts, from the sketch chain).
    {
        auto body = b::make_prism(rect_chains(), 1.5f, +1);
        check_body_invariants(*body, 8, 12, 6, 0, 2);
        check_exact_provenance(*body);
    }
    // n = 5 — a pentagon (the formula at one more n).
    {
        const Vec2f p0{0.0f, 0.0f}, p1{2.0f, 0.0f}, p2{3.2f, 0.9f},
            p3{1.6f, 2.1f}, p4{-0.4f, 1.3f};
        std::vector<s::SketchChain> chains{s::SketchChain{{
            s::sketch_segment(p0, p1, true),
            s::sketch_segment(p1, p2, true),
            s::sketch_segment(p2, p3, true),
            s::sketch_segment(p3, p4, true),
            s::sketch_segment(p4, p0, true),
        }}};
        auto body = b::make_prism(chains, 0.75f, -1);
        check_body_invariants(*body, 10, 15, 7, 0, 2);
    }
    // n = 2 — the lens (two arcs): the formula's smallest case.
    {
        auto body = b::make_prism(lens_chains(), 1.0f, +1);
        check_body_invariants(*body, 4, 6, 4, 0, 2);
    }
    // n = 3 — the sector (segments + arc: the mixed polygon).
    {
        auto body = b::make_prism(sector_chains(), 2.0f, +1);
        check_body_invariants(*body, 6, 9, 5, 0, 2);
    }
}

TEST_CASE("G1-U3 counts: the circle profile → the ADR-0023 seam counts 4V/5E/3F") {
    auto body = b::make_prism(circle_chains(), 3.0f, +1);
    check_body_invariants(*body, 4, 5, 3, 0, 2);
    check_exact_provenance(*body);

    // The counts ARE the factory cylinder's (the pattern reused
    // VERBATIM — the order's pin): same V/E/F, same χ.
    auto cyl = b::make_cylinder(1.25f, 3.0f);
    CHECK(cyl->vertex_count() == body->vertex_count());
    CHECK(cyl->edge_count() == body->edge_count());
    CHECK(cyl->face_count() == body->face_count());
}

TEST_CASE("G1-U3 counts: the ring — each hole adds its wall and one inner loop per cap (χ = 2 − 2·holes)") {
    // The rectangle (n=4) + one circle hole: V=2·4+2, E=3·4+3,
    // F=4+1+2, H=2 → χ=0, genus 1. (Holes are CIRCLES by the
    // kernel's own mathematics — the max-composition intersects
    // regions, and an intersection of half-planes is convex: a
    // multi-entity hole's complement is inexpressible. The canvas
    // authors exactly this class.)
    {
        auto body = b::make_prism(ring_chains(), 1.5f, +1);
        check_body_invariants(*body, 10, 15, 7, 2, 0);
        CHECK(b::genus_from_chi(b::euler_characteristic(*body)) == 1);
        check_exact_provenance(*body);
    }
    // Two holes (the circle + a second, smaller one): χ = 2 − 2·2 =
    // −2, genus 2 — each through-hole is one handle.
    {
        auto chains = rect_chains();
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.2f, true)}});
        chains.push_back(s::SketchChain{
            {s::sketch_circle({1.2f, 0.4f}, 0.15f, true)}});
        auto body = b::make_prism(chains, 1.5f, +1);
        check_body_invariants(*body, 12, 18, 8, 4, -2);
        CHECK(b::genus_from_chi(b::euler_characteristic(*body)) == 2);
    }
}

// ===========================================================================
// The geometry — the exact claim vs the faceted today
// ===========================================================================

TEST_CASE("G1-U3 geometry: the circle's lateral face is ONE cylinder — exact vs the faceted today") {
    auto body = b::make_prism(circle_chains(), 3.0f, +1);

    // The wall: family "cylinder", ONE lateral face.
    const auto& faces = body->solid.shell().faces();
    REQUIRE(faces.size() == 3);
    CHECK(std::string(faces[0].surface()->type_name()) == "cylinder");
    const auto wall = b::query_face(*body, 0);
    CHECK(wall.family == "cylinder");
    CHECK(wall.loop_count == 1);

    // The caps: family "plane", the disc region.
    for (const b::FaceId cap : {b::FaceId{1}, b::FaceId{2}}) {
        const auto q = b::query_face(*body, cap);
        CHECK(q.family == "plane");
        CHECK(q.loop_count == 1);
        CHECK(q.inner_loop_count == 0);
    }

    // The TODAY world, for contrast: the same profile's SDF through
    // the faceted bridge — N planar strips, no p-curves, a declared
    // tolerance. Both honest; only one exact.
    s::SDFBody sdf_body(std::make_unique<s::ExtrudeSDF>(
        circle_chains(), 3.0f, +1));
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;   // the extrude is not a fitted primitive
    opts.march_resolution = 24;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(sdf_body, opts);
    REQUIRE(faceted != nullptr);
    CHECK(faceted->face_count() > 3);  // N strips, not 3 true faces
    std::size_t planar = 0, with_pcurve = 0, approx = 0;
    for (const auto& fi : b::query_all_faces(*faceted)) {
        if (fi.family == "plane") ++planar;
        if (!fi.exact) ++approx;
        for (const auto& loop : fi.loops)
            for (const auto& ce : loop.coedges)
                if (ce.kind != b::PCurveInfo::Kind::None) ++with_pcurve;
    }
    CHECK(planar == faceted->face_count());
    CHECK(with_pcurve == 0);
    CHECK(approx == faceted->face_count());
}

// ===========================================================================
// The pinned tables — the builder's numbering discipline (ADR-0005)
// ===========================================================================

TEST_CASE("G1-U3 golden: the segment prism (n=4) — the full pinned tables") {
    auto body = b::make_prism(rect_chains(), 1.5f, +1);
    REQUIRE(body->vertex_count() == 8);

    // Vertices: id 2i = the bottom copy of joint J_i (entity i's
    // END), id 2i+1 = its top copy; z spans [0, 1.5] (direction +1).
    //   J0=(2,-1)  J1=(2,1)  J2=(-2,1)  J3=(-2,-1)
    const Vec3f pv[8] = {
        {2.0f, -1.0f, 0.0f}, {2.0f, -1.0f, 1.5f},    // J0 bottom, top
        {2.0f, 1.0f, 0.0f}, {2.0f, 1.0f, 1.5f},      // J1
        {-2.0f, 1.0f, 0.0f}, {-2.0f, 1.0f, 1.5f},    // J2
        {-2.0f, -1.0f, 0.0f}, {-2.0f, -1.0f, 1.5f},  // J3
    };
    for (std::size_t i = 0; i < 8; ++i) {
        CAPTURE(i);
        CHECK(close3(body->vertices[i].position(), pv[i], 0.0f));
    }

    // Edges: i = the bottom curve of entity i (its own direction);
    // 4+i = its top curve; 8+i = the vertical at joint J_i.
    const std::pair<std::size_t, std::size_t> pe[12] = {
        {6, 0}, {0, 2}, {2, 4}, {4, 6},      // bottom curves (entity dir)
        {7, 1}, {1, 3}, {3, 5}, {5, 7},      // top curves
        {0, 1}, {2, 3}, {4, 5}, {6, 7},      // verticals
    };
    for (std::size_t i = 0; i < 12; ++i) {
        CAPTURE(i);
        CHECK(body->edges[i].vertex(0) == pe[i].first);
        CHECK(body->edges[i].vertex(1) == pe[i].second);
    }

    // Faces: 0..3 = the lateral faces of entities 0..3; 4 = the -Z
    // cap; 5 = the +Z cap.
    const auto& faces = body->solid.shell().faces();
    REQUIRE(faces.size() == 6);

    // Lateral face 0 (entity 0: the bottom segment, east, material on
    // the left = north): the plane through the segment with the
    // OUTWARD normal (south); the rectangle walk [bottom fwd, up at
    // J0, top rev, down at J3].
    {
        const auto& f = faces[0];
        CHECK(std::string(f.surface()->type_name()) == "plane");
        const auto* ps = dynamic_cast<const b::PlaneSurface*>(f.surface());
        REQUIRE(ps != nullptr);
        CHECK(close3(ps->origin(), {-2.0f, -1.0f, 0.0f}, 0.0f));
        CHECK(close3(ps->normal_vec(), {0.0f, -1.0f, 0.0f}, 0.0f));
        REQUIRE(f.loops().size() == 1);
        const auto& ces = f.loops()[0].coedges();
        REQUIRE(ces.size() == 4);
        // [bottom_0 fwd, vertical_0 fwd, top_0 rev, vertical_3 rev]
        CHECK(ces[0].edge() == 0);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[1].edge() == 8);
        CHECK(ces[1].reversed() == false);
        CHECK(ces[2].edge() == 4);
        CHECK(ces[2].reversed() == true);
        CHECK(ces[3].edge() == 11);
        CHECK(ces[3].reversed() == true);
        // The p-curves: Line2 by projection, material on the left.
        for (const auto& ce : ces) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == true);
        }
        // The plane's frame for normal (0,-1,0) is (u=+Z, v=-X): the
        // bottom co-edge (J3 -> J0 at z=0) reads (0,0) -> (0,-4) —
        // the segment at u=0, the verticals along v; the up vertical
        // at J0 reads (0,-4) -> (1.5,-4).
        CHECK(close2(ces[0].p_curve()->segment.a, {0.0f, 0.0f}, 0.0f));
        CHECK(close2(ces[0].p_curve()->segment.b, {0.0f, -4.0f}, 0.0f));
        CHECK(close2(ces[1].p_curve()->segment.a, {0.0f, -4.0f}, 0.0f));
        CHECK(close2(ces[1].p_curve()->segment.b, {1.5f, -4.0f}, 0.0f));
    }

    // The -Z cap (face 4): the chain walked REVERSED (the mirrored
    // domain (u,v) = (x,-y) keeps the authored bits), entities 3..0
    // each reversed.
    {
        const auto& f = faces[4];
        CHECK(std::string(f.surface()->type_name()) == "plane");
        const auto* ps = dynamic_cast<const b::PlaneSurface*>(f.surface());
        REQUIRE(ps != nullptr);
        CHECK(close3(ps->normal_vec(), {0.0f, 0.0f, -1.0f}, 0.0f));
        REQUIRE(f.loops().size() == 1);
        const auto& ces = f.loops()[0].coedges();
        REQUIRE(ces.size() == 4);
        // [bottom_3 rev, bottom_2 rev, bottom_1 rev, bottom_0 rev]
        CHECK(ces[0].edge() == 3);
        CHECK(ces[0].reversed() == true);
        CHECK(ces[1].edge() == 2);
        CHECK(ces[1].reversed() == true);
        CHECK(ces[2].edge() == 1);
        CHECK(ces[2].reversed() == true);
        CHECK(ces[3].edge() == 0);
        CHECK(ces[3].reversed() == true);
        for (const auto& ce : ces) {
            REQUIRE(ce.p_curve() != nullptr);
            // The bits are the AUTHORED ones (interior_left = true):
            // the cap's region IS the profile region.
            CHECK(ce.p_curve()->segment.interior_left == true);
        }
        // The mirrored domain: the first co-edge (entity 3 reversed:
        // J3 -> J2, i.e. (-2,-1) -> (-2,1) in 3D) reads
        // (-2,1) -> (-2,-1).
        CHECK(close2(ces[0].p_curve()->segment.a, {-2.0f, 1.0f}, 0.0f));
        CHECK(close2(ces[0].p_curve()->segment.b, {-2.0f, -1.0f}, 0.0f));
    }

    // The +Z cap (face 5): the chain walked AUTHORED (the rotated
    // domain (u,v) = (-x,-y)), entities 0..3 forward on the TOP
    // curves.
    {
        const auto& f = faces[5];
        const auto* ps = dynamic_cast<const b::PlaneSurface*>(f.surface());
        REQUIRE(ps != nullptr);
        CHECK(close3(ps->normal_vec(), {0.0f, 0.0f, 1.0f}, 0.0f));
        const auto& ces = f.loops()[0].coedges();
        REQUIRE(ces.size() == 4);
        // [top_0 fwd, top_1 fwd, top_2 fwd, top_3 fwd]
        CHECK(ces[0].edge() == 4);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[1].edge() == 5);
        CHECK(ces[2].edge() == 6);
        CHECK(ces[3].edge() == 7);
        // The rotated domain: the first co-edge (entity 0: J3 -> J0,
        // i.e. (-2,-1) -> (2,-1) in 3D) reads (2,1) -> (-2,1).
        CHECK(close2(ces[0].p_curve()->segment.a, {2.0f, 1.0f}, 0.0f));
        CHECK(close2(ces[0].p_curve()->segment.b, {-2.0f, 1.0f}, 0.0f));
        for (const auto& ce : ces)
            CHECK(ce.p_curve()->segment.interior_left == true);
    }

    // The conventions on ALL 6 faces (DIRECTION/CHAINING/REGION SIDE
    // + the frame rule), each against its region model.
    {
        s::ProfileSDF profile(rect_chains());
        const Vec3f hints[4] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
                                {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
        for (std::size_t fi = 0; fi < 4; ++fi) {
            // The lateral rectangle in the entity's plane domain: the
            // four p-curve starts, CCW (the walk order).
            std::vector<Vec2f> corners;
            for (const auto& ce : faces[fi].loops()[0].coedges())
                corners.push_back(ce.p_curve()->segment.a);
            check_face_conventions(*body, faces[fi], {{RegionSpec{corners}}},
                                   {corners[0], corners[2]}, hints[fi]);
        }
        // The caps: the region IS the profile's (the kernel's own
        // sample — one opinion, read back through the cap's frame).
        const RegionSpec cap_region{.profile = &profile};
        const RegionSpec cap_lo_region{.profile = &profile,
                                       .to_sketch =
                                           RegionSpec::SketchMap::MirrorY};
        const RegionSpec cap_hi_region{
            .profile = &profile,
            .to_sketch = RegionSpec::SketchMap::Rot180};
        check_face_conventions(*body, faces[4], {cap_lo_region},
                               {{0.0f, 0.0f}}, {0.0f, 0.0f, -1.0f});
        check_face_conventions(*body, faces[5], {cap_hi_region},
                               {{0.0f, 0.0f}}, {0.0f, 0.0f, 1.0f});
    }
}

TEST_CASE("G1-U3 golden: the sector (segments + arc) — the arc's cylindrical patch") {
    auto body = b::make_prism(sector_chains(), 2.0f, +1);
    REQUIRE(body->face_count() == 5);

    const auto& faces = body->solid.shell().faces();
    const float a0 = -0.2f * kPi, a1 = 0.35f * kPi;
    const float am = 0.5f * (a0 + a1);

    // Face 1 = the arc's lateral face (entity 1): a cylindrical patch.
    const auto& wall = faces[1];
    const auto* cyl = dynamic_cast<const b::CylinderZSurface*>(wall.surface());
    REQUIRE(cyl != nullptr);
    CHECK(std::string(cyl->type_name()) == "cylinder");
    CHECK(close(cyl->radius(), 1.5f, 0.0f));
    // The wall's loop: the rectangle walk in the (u,v) window
    // [a0, a1] x [0, h], four Line2, material on the left.
    REQUIRE(wall.loops().size() == 1);
    const auto& ces = wall.loops()[0].coedges();
    REQUIRE(ces.size() == 4);
    for (const auto& ce : ces) {
        REQUIRE(ce.p_curve() != nullptr);
        CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
        CHECK(ce.p_curve()->segment.interior_left == true);
    }
    // The bottom co-edge runs the walk's u direction: from a0 to a1
    // at v=0 (entity 1's authored span).
    CHECK(close2(ces[0].p_curve()->segment.a, {a0, 0.0f}));
    CHECK(close2(ces[0].p_curve()->segment.b, {a1, 0.0f}));
    // The second co-edge is the vertical up at the arc's end.
    CHECK(close2(ces[1].p_curve()->segment.a, {a1, 0.0f}));
    CHECK(close2(ces[1].p_curve()->segment.b, {a1, 2.0f}));

    // The caps: entity 1's co-edge is an Arc2 with the sketch's own
    // radius and sweep (typed entity for entity).
    for (const b::FaceId cap : {b::FaceId{3}, b::FaceId{4}}) {
        const auto q = b::query_face(*body, cap);
        CHECK(q.family == "plane");
        REQUIRE(q.loops.size() == 1);
        REQUIRE(q.loops[0].coedges.size() == 3);
        std::size_t arcs = 0, lines = 0;
        for (const auto& ce : q.loops[0].coedges) {
            if (ce.kind == b::PCurveInfo::Kind::Arc) {
                ++arcs;
                CHECK(close(ce.radius, 1.5f));
                CHECK(close(ce.sweep, a1 - a0));
                CHECK(ce.hole == false);
            } else if (ce.kind == b::PCurveInfo::Kind::Line) {
                ++lines;
            }
        }
        CHECK(arcs == 1);
        CHECK(lines == 2);
    }

    // The conventions on every face. The lateral hints: the segments'
    // outward normals — perp of the walk direction, negated for the
    // material-on-the-left authoring (outward = (d.y, −d.x)) — and
    // the arc's +radial at mid.
    s::ProfileSDF profile(sector_chains());
    const RegionSpec cap_lo_region{
        .profile = &profile, .to_sketch = RegionSpec::SketchMap::MirrorY};
    const RegionSpec cap_hi_region{
        .profile = &profile, .to_sketch = RegionSpec::SketchMap::Rot180};
    const Vec2f c0{0.0f, 0.0f};
    const Vec2f ps = c0 + Vec2f{1.5f * std::cos(a0), 1.5f * std::sin(a0)};
    const Vec2f pe = c0 + Vec2f{1.5f * std::cos(a1), 1.5f * std::sin(a1)};
    const Vec3f n0 = Vec3f{ps.y - c0.y, -(ps.x - c0.x), 0.0f}.normalized();
    const Vec3f n2 = Vec3f{-(pe.y - c0.y), pe.x - c0.x, 0.0f}.normalized();
    for (std::size_t fi : {std::size_t{0}, std::size_t{2}}) {
        std::vector<Vec2f> corners;
        for (const auto& ce : faces[fi].loops()[0].coedges())
            corners.push_back(ce.p_curve()->segment.a);
        check_face_conventions(*body, faces[fi], {{RegionSpec{corners}}},
                               {corners[0], corners[2]},
                               fi == 0 ? n0 : n2);
    }
    // The arc wall: the window region, the outward +radial at mid.
    check_face_conventions(*body, faces[1],
                           {{RegionSpec{.window = true,
                                        .umin = a0, .umax = a1,
                                        .vmin = 0.0f, .vmax = 2.0f}}},
                           {{am, 1.0f}},
                           {std::cos(am), std::sin(am), 0.0f});
    // The caps (the sector's mid-radius point, mapped through each
    // cap's frame into its domain: the -Z cap's mirror and the +Z
    // cap's rotation).
    const Vec2f in_sector{0.7f * std::cos(am), 0.7f * std::sin(am)};
    check_face_conventions(*body, faces[3], {cap_lo_region},
                           {{in_sector.x, -in_sector.y}}, {0, 0, -1.0f});
    check_face_conventions(*body, faces[4], {cap_hi_region},
                           {{-in_sector.x, -in_sector.y}}, {0, 0, 1.0f});
}

TEST_CASE("G1-U3 golden: the circle — the seam-wired pattern VERBATIM on Z") {
    auto body = b::make_prism(circle_chains({0.5f, 0.25f}, 1.25f), 3.0f, +1);
    REQUIRE(body->vertex_count() == 4);
    REQUIRE(body->edge_count() == 5);

    // Vertices: 0/1 = the bottom seam/antipode, 2/3 = the top pair.
    // The seam meridian is the +X direction from the centre (u = 0).
    CHECK(close3(body->vertices[0].position(), {1.75f, 0.25f, 0.0f}, 0.0f));
    CHECK(close3(body->vertices[1].position(), {-0.75f, 0.25f, 0.0f}, 0.0f));
    CHECK(close3(body->vertices[2].position(), {1.75f, 0.25f, 3.0f}, 0.0f));
    CHECK(close3(body->vertices[3].position(), {-0.75f, 0.25f, 3.0f}, 0.0f));

    // Edges: 0/1 = the bottom rim halves (seam->antipode via +Y,
    // antipode->seam via -Y); 2/3 = the top rim halves; 4 = the seam.
    const std::pair<std::size_t, std::size_t> pe[5] = {
        {0, 1}, {1, 0}, {2, 3}, {3, 2}, {0, 2}};
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(body->edges[i].vertex(0) == pe[i].first);
        CHECK(body->edges[i].vertex(1) == pe[i].second);
    }

    const auto& faces = body->solid.shell().faces();
    REQUIRE(faces.size() == 3);

    // The wall (face 0): the slit window [0,2pi] x [0,3] walked CCW
    // with SIX Line2: [seam fwd (right copy, u=2pi), rim halves west,
    // seam rev (left copy, u=0), rim halves east].
    {
        const auto* cyl =
            dynamic_cast<const b::CylinderZSurface*>(faces[0].surface());
        REQUIRE(cyl != nullptr);
        CHECK(close(cyl->radius(), 1.25f, 0.0f));
        const auto& ces = faces[0].loops()[0].coedges();
        REQUIRE(ces.size() == 6);
        // The seam pair: edge 4 twice — forward then reversed.
        CHECK(ces[0].edge() == 4);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[3].edge() == 4);
        CHECK(ces[3].reversed() == true);
        // The right copy at u=2pi, the left at u=0.
        CHECK(close2(ces[0].p_curve()->segment.a, {kTwoPi, 0.0f}));
        CHECK(close2(ces[0].p_curve()->segment.b, {kTwoPi, 3.0f}));
        CHECK(close2(ces[3].p_curve()->segment.a, {0.0f, 3.0f}));
        CHECK(close2(ces[3].p_curve()->segment.b, {0.0f, 0.0f}));
        // The rim halves west then east.
        CHECK(ces[1].edge() == 3);
        CHECK(ces[1].reversed() == true);
        CHECK(ces[2].edge() == 2);
        CHECK(ces[2].reversed() == true);
        CHECK(ces[4].edge() == 0);
        CHECK(ces[4].reversed() == false);
        CHECK(ces[5].edge() == 1);
        CHECK(ces[5].reversed() == false);
        for (const auto& ce : ces)
            CHECK(ce.p_curve()->segment.interior_left == true);
        // The window conventions (samples on the +Y half of the rim,
        // where the outward radial is +Y-ish; the seam copies at
        // positions 0 and 3 skip the away-side check — the periodic
        // identification makes it the neighbour's material).
        check_face_conventions(*body, faces[0],
                               {{RegionSpec{.window = true, .periodic_u = true,
                                            .umin = 0.0f, .umax = kTwoPi,
                                            .vmin = 0.0f, .vmax = 3.0f}}},
                               {{0.5f * kPi, 1.5f}},
                               {0.0f, 1.0f, 0.0f},
                               /*seam_flags=*/{true, false, false, true,
                                               false, false});
    }

    // The caps: the disc, two pi-sweep Arc2 each, chained CCW in each
    // cap's own domain (the rotation-vs-mirror tables — pinned).
    {
        // The -Z cap (face 1): domain (x,-y); the walk (3D CW) reads
        // CCW in the mirror.
        const auto& f = faces[1];
        const auto* ps = dynamic_cast<const b::PlaneSurface*>(f.surface());
        REQUIRE(ps != nullptr);
        CHECK(close3(ps->normal_vec(), {0.0f, 0.0f, -1.0f}, 0.0f));
        const auto& ces = f.loops()[0].coedges();
        REQUIRE(ces.size() == 2);
        for (const auto& ce : ces) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Arc);
            const auto& arc = ce.p_curve()->arc;
            CHECK(close2(arc.center, {0.5f, -0.25f}));
            CHECK(close(arc.radius, 1.25f, 0.0f));
            CHECK(arc.hole == false);
            CHECK(close(arc.sweep, kPi));
        }
        // The +Z cap (face 2): domain (-x,-y); the Arc2 centres at
        // (-0.5,-0.25), the same radius/sweep.
        const auto& f2 = faces[2];
        const auto& ces2 = f2.loops()[0].coedges();
        REQUIRE(ces2.size() == 2);
        for (const auto& ce : ces2) {
            const auto& arc = ce.p_curve()->arc;
            CHECK(close2(arc.center, {-0.5f, -0.25f}));
            CHECK(close(arc.radius, 1.25f, 0.0f));
            CHECK(arc.hole == false);
            CHECK(close(arc.sweep, kPi));
        }
        // Both caps' conventions against the disc region (the
        // profile's own sample, read through each cap's frame).
        s::ProfileSDF profile(circle_chains({0.5f, 0.25f}, 1.25f));
        const RegionSpec disc_lo{
            .profile = &profile,
            .to_sketch = RegionSpec::SketchMap::MirrorY};
        const RegionSpec disc_hi{
            .profile = &profile,
            .to_sketch = RegionSpec::SketchMap::Rot180};
        check_face_conventions(*body, faces[1], {disc_lo},
                               {{0.5f, -0.25f}}, {0.0f, 0.0f, -1.0f});
        check_face_conventions(*body, faces[2], {disc_hi},
                               {{-0.5f, -0.25f}}, {0.0f, 0.0f, 1.0f});
    }
}

TEST_CASE("G1-U3 golden: the ring — the caps carry the hole as an inner Circle2 loop") {
    auto body = b::make_prism(ring_chains(), 1.5f, +1);
    REQUIRE(body->face_count() == 7);

    const auto& faces = body->solid.shell().faces();

    // Faces: 0..3 = the outer laterals; 4 = the hole's wall; 5 = -Z
    // cap; 6 = +Z cap.
    const auto* hole_wall =
        dynamic_cast<const b::CylinderZSurface*>(faces[4].surface());
    REQUIRE(hole_wall != nullptr);
    CHECK(close(hole_wall->radius(), 0.2f, 0.0f));

    // The hole's wall: the seam-wired window on the REVERSED u (the
    // hole's outward normal is -radial — the frame rule); four Line2.
    {
        const auto& ces = faces[4].loops()[0].coedges();
        REQUIRE(ces.size() == 4);
        // The seam pair within the single loop (edges 11 and 13 by
        // the layout: the hole's closed rims are edges 12/13... see
        // the layout table in extrude.hpp; here: two co-edges share
        // one edge id).
        std::size_t pairs = 0;
        for (std::size_t i = 0; i < ces.size(); ++i)
            for (std::size_t j = i + 1; j < ces.size(); ++j)
                if (ces[i].edge() == ces[j].edge()) ++pairs;
        CHECK(pairs == 1);  // the seam, used twice in this one loop
        for (const auto& ce : ces)
            CHECK(ce.p_curve()->segment.interior_left == true);
        check_face_conventions(*body, faces[4],
                               {{RegionSpec{.window = true, .periodic_u = true,
                                            .umin = -kTwoPi, .umax = 0.0f,
                                            .vmin = 0.0f, .vmax = 1.5f}}},
                               {{-0.5f * kPi, 0.75f}},
                               {0.0f, -1.0f, 0.0f},
                               /*seam_flags=*/{true, false, true, false});
    }

    // The caps: 2 loops each — the outer (the rectangle chain) and
    // the inner (the hole, ONE co-edge carrying a Circle2 with
    // hole=true: the annulus is the entity's region).
    for (const std::size_t cap : {std::size_t{5}, std::size_t{6}}) {
        const auto& f = faces[cap];
        REQUIRE(f.loops().size() == 2);
        CHECK(f.loops()[0].is_outer() == true);
        CHECK(f.loops()[1].is_outer() == false);
        // The outer loop: 4 Line2 (the chain).
        REQUIRE(f.loops()[0].coedges().size() == 4);
        for (const auto& ce : f.loops()[0].coedges()) {
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == true);
        }
        // The inner loop: 1 co-edge, the Circle2, hole=true.
        const auto& inner = f.loops()[1].coedges();
        REQUIRE(inner.size() == 1);
        REQUIRE(inner[0].p_curve() != nullptr);
        CHECK(inner[0].p_curve()->kind == s::SketchEntity::Kind::Circle);
        const auto& circ = inner[0].p_curve()->circle;
        CHECK(close(circ.radius, 0.2f, 0.0f));
        CHECK(circ.hole == true);
    }
    // The hole is centred: both caps' Circle2 centres read (0,0)
    // (the mirror and the rotation agree on the origin).
    CHECK(close2(faces[5].loops()[1].coedges()[0].p_curve()->circle.center,
                 {0.0f, 0.0f}));
    CHECK(close2(faces[6].loops()[1].coedges()[0].p_curve()->circle.center,
                 {0.0f, 0.0f}));

    // The caps' conventions: both loops against the profile region
    // (the annulus — the kernel's own sample, read through each
    // cap's frame; the inner loop's away-side falls in the hole,
    // outside the material).
    s::ProfileSDF profile(ring_chains());
    const RegionSpec annulus_lo{
        .profile = &profile, .to_sketch = RegionSpec::SketchMap::MirrorY};
    const RegionSpec annulus_hi{
        .profile = &profile, .to_sketch = RegionSpec::SketchMap::Rot180};
    check_face_conventions(*body, faces[5], {annulus_lo, annulus_lo},
                           {{0.0f, 0.0f}}, {0.0f, 0.0f, -1.0f});
    check_face_conventions(*body, faces[6], {annulus_hi, annulus_hi},
                           {{0.0f, 0.0f}}, {0.0f, 0.0f, 1.0f});
}

// ===========================================================================
// The class boundary — the refusal is loud and the SDF path stands
// ===========================================================================

TEST_CASE("G1-U3 class: the notch-side arc refuses the exact B-Rep loudly — the vocabulary's honest boundary") {
    // The notch profile certifies in 2D (the SDF world accepts it).
    REQUIRE_NOTHROW(s::ProfileSDF{notch_chains()});

    // The exact B-Rep refuses: the typed vocabulary stores CCW sweeps
    // only; a notch arc's cap trimming runs CW in BOTH cap domains.
    bool refused = false;
    std::string why;
    try {
        auto discarded = b::make_prism(notch_chains(), 1.0f, +1);
        (void)discarded;
    } catch (const std::invalid_argument& e) {
        refused = true;
        why = e.what();
    }
    REQUIRE(refused);
    // The message names the reason and the remedy (the actionable
    // culture) — no silent fallback, no approximated-as-exact.
    CHECK(why.find("CCW") != std::string::npos);
    CHECK(why.find("Remedy") != std::string::npos);

    // The all-segment CW polygon (material on the right) IS in class:
    // Line2 has no direction problem.
    {
        std::vector<s::SketchChain> cw{s::SketchChain{{
            s::sketch_segment({0.0f, 0.0f}, {0.0f, 1.0f}, false),
            s::sketch_segment({0.0f, 1.0f}, {1.0f, 1.0f}, false),
            s::sketch_segment({1.0f, 1.0f}, {1.0f, 0.0f}, false),
            s::sketch_segment({1.0f, 0.0f}, {0.0f, 0.0f}, false),
        }}};
        auto body = b::make_prism(cw, 1.0f, +1);
        check_body_invariants(*body, 8, 12, 6, 0, 2);
        check_exact_provenance(*body);
        // The caps' bits are the authored ones (interior_left=false):
        // the region is the same square, the walk adapted.
        const auto& caps = body->solid.shell().faces();
        for (std::size_t ci : {std::size_t{4}, std::size_t{5}}) {
            for (const auto& ce : caps[ci].loops()[0].coedges())
                CHECK(ce.p_curve()->segment.interior_left == false);
        }
    }
}

// ===========================================================================
// The HybridBody landing (ADR-0031 D6 / ADR-0032 D6)
// ===========================================================================

TEST_CASE("G1-U3 landing: the extrude node owns BOTH materializations, bridge tolerance 0") {
    CAD_0::scene::Scene sc;
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);

    const auto* n = sc.get_node(eid);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);
    CHECK(n->hybrid->brep != nullptr);
    CHECK(n->hybrid->sdf != nullptr);
    CHECK(n->hybrid->bridge_tolerance.linear == 0.0);
    CHECK(n->hybrid->bridge_tolerance.angular == 0.0);
    CHECK(n->hybrid->role == CAD_0::geometry::HybridRole::BRepWithSDFShell);
    CHECK(n->hybrid_refusal.empty());

    // The exact B-Rep: the seam-wired prism, exact/0.
    REQUIRE(n->hybrid->brep->face_count() == 3);
    check_exact_provenance(*n->hybrid->brep);
    check_body_invariants(*n->hybrid->brep, 4, 5, 3, 0, 2);

    // The SDF twin: the same recipe's materialization — bounds and
    // samples identical to the node's display body.
    CHECK(n->hybrid->sdf->bounds().min.x == n->body.bounds().min.x);
    CHECK(n->hybrid->sdf->bounds().max.z == n->body.bounds().max.z);
    for (const Vec3f p : {Vec3f{0.5f, 0.25f, 1.5f}, Vec3f{2.0f, 0.25f, 1.5f},
                          Vec3f{0.5f, 0.25f, -0.1f}}) {
        CHECK(n->hybrid->sdf->sample(p).value == n->body.sample(p).value);
    }

    // The display path is UNCHANGED: the node's body is the ExtrudeSDF
    // the world already knew (bounds pinned against a fresh build).
    s::SDFBody fresh(
        std::make_unique<s::ExtrudeSDF>(circle_chains(), 3.0f, +1));
    CHECK(n->body.bounds().min.x == fresh.bounds().min.x);
    CHECK(n->body.bounds().max.x == fresh.bounds().max.x);
    CHECK(n->body.bounds().min.z == fresh.bounds().min.z);
    CHECK(n->body.bounds().max.z == fresh.bounds().max.z);
    CHECK(n->body.sample({0.5f, 0.25f, 1.5f}).value ==
          fresh.sample({0.5f, 0.25f, 1.5f}).value);
}

TEST_CASE("G1-U3 landing: the honesty chain — the extruded prism says exact, the subtracted body says approximated") {
    CAD_0::scene::Scene sc;
    // The guide case's shape: a box, a circle sketch, the extrude,
    // the subtract consuming the extrude's SDF side.
    const auto bid = sc.add_node(
        "Box", s::SDFBody(std::make_unique<s::ExtrudeSDF>(
                                   rect_chains({1.0f, 1.0f}, {2.0f, 1.6f}),
                                   2.0f, +1)));
    const auto sid =
        sc.add_sketch_node("S", circle_chains({1.5f, 0.5f}, 0.4f));
    const auto eid = add_extrude(sc, "Extrude", sid, 2.0f, +1);

    CAD_0::scene::OperationRecipe sub;
    sub.type = CAD_0::scene::OperationType::Subtract;
    sub.target = bid;
    sub.tool = eid;
    const auto did = sc.add_operation_node("Sub", sub);

    const auto* ex = sc.get_node(eid);
    const auto* dn = sc.get_node(did);
    REQUIRE(ex != nullptr);
    REQUIRE(dn != nullptr);

    // The prism: exact (the hybrid's B-Rep).
    REQUIRE(ex->hybrid != nullptr);
    check_exact_provenance(*ex->hybrid->brep);

    // The subtract: consumes the SDF side; its exact chain ENDS here
    // today (unit 4's subtract is not built). The node carries NO
    // exact B-Rep and no refusal (it is not an extrude) — no new lie
    // is permitted.
    CHECK(dn->hybrid == nullptr);
    CHECK(dn->hybrid_refusal.empty());

    // The bridge extract of the subtracted body: faceted,
    // approximated with the declared grid tolerance (the unit-1 rules
    // propagate the worst — the honesty chain, unchanged).
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 20;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(dn->body, opts);
    REQUIRE(faceted != nullptr);
    std::size_t approx = 0, with_pcurve = 0;
    for (const auto& fi : b::query_all_faces(*faceted)) {
        if (!fi.exact) ++approx;
        for (const auto& loop : fi.loops)
            for (const auto& ce : loop.coedges)
                if (ce.kind != b::PCurveInfo::Kind::None) ++with_pcurve;
    }
    CHECK(approx == faceted->face_count());
    CHECK(with_pcurve == 0);
}

TEST_CASE("G1-U3 landing: the out-of-class profile keeps its SDF, the refusal recorded") {
    CAD_0::scene::Scene sc;
    const auto sid = sc.add_sketch_node("S", notch_chains());
    // The creation SUCCEEDS (the SDF certifies; no visible change to
    // the flow) — the exact side's refusal is recorded, not thrown.
    std::uint32_t eid = 0;
    REQUIRE_NOTHROW(eid = add_extrude(sc, "Extrude", sid, 1.0f, +1));
    const auto* n = sc.get_node(eid);
    REQUIRE(n != nullptr);

    CHECK(n->hybrid == nullptr);
    CHECK_FALSE(n->hybrid_refusal.empty());
    CHECK(n->hybrid_refusal.find("CCW") != std::string::npos);

    // The SDF stands: the node's body is the ExtrudeSDF the world
    // already knew (the display continues).
    s::SDFBody fresh(
        std::make_unique<s::ExtrudeSDF>(notch_chains(), 1.0f, +1));
    CHECK(n->body.sample({1.5f, 1.0f, 0.5f}).value ==
          fresh.sample({1.5f, 1.0f, 0.5f}).value);
    CHECK(n->operand_missing == false);
    CHECK(n->operation_pending == false);

    // And an in-class extrude in the SAME scene lands its hybrid (the
    // refusal is per-node, never global).
    const auto rid = sc.add_sketch_node("R", rect_chains());
    const auto reid = add_extrude(sc, "RE", rid, 1.0f, +1);
    REQUIRE(sc.get_node(reid)->hybrid != nullptr);
}

// ===========================================================================
// Determinism (ADR-0014) — both materializations, bit-identical
// ===========================================================================

TEST_CASE("G1-U3 determinism: the radius edit cycle regenerates both materializations bit-identically") {
    CAD_0::scene::Scene sc;
    const auto sid =
        sc.add_sketch_node("S", circle_chains({0.5f, 0.25f}, 1.25f));
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto* n = sc.get_node(eid);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);

    const std::string brep_before = query_digest(*n->hybrid->brep);
    const std::string sdf_before = sdf_digest(n->body);

    // The edit: the radius 1.25 -> 1.1 (entity 0 of chain 0).
    sc.sketch_set_entity(
        sid, 0, 0,
        s::sketch_circle({0.5f, 0.25f}, 1.1f, false));
    auto r1 = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(r1.rebuilt == 1);
    REQUIRE(n->hybrid != nullptr);
    CHECK(query_digest(*n->hybrid->brep) != brep_before);  // it moved
    CHECK(sdf_digest(n->body) != sdf_before);

    // The edit back: 1.1 -> 1.25. Both materializations regenerate
    // BIT-IDENTICALLY to the originals.
    sc.sketch_set_entity(
        sid, 0, 0,
        s::sketch_circle({0.5f, 0.25f}, 1.25f, false));
    auto r2 = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(r2.rebuilt == 1);
    REQUIRE(n->hybrid != nullptr);
    CHECK(query_digest(*n->hybrid->brep) == brep_before);
    CHECK(sdf_digest(n->body) == sdf_before);

    // And the SDF twin inside the hybrid matches the display body.
    CHECK(sdf_digest(*n->hybrid->sdf) == sdf_before);
}

TEST_CASE("G1-U3 determinism: the v0.4 golden re-loads and re-materializes the exact prism identically") {
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_4_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));

    // The golden's extrude node (the parametric hole's tool): its
    // hybrid re-materialized on load — deterministic, exact.
    const CAD_0::scene::SceneNode* extrude_node = find_extrude_node(sc);
    REQUIRE(extrude_node != nullptr);
    REQUIRE(extrude_node->hybrid != nullptr);
    CHECK(extrude_node->hybrid_refusal.empty());

    // The exact prism: a circle profile → the seam counts, all exact.
    check_exact_provenance(*extrude_node->hybrid->brep);
    CHECK(b::inner_loop_count(*extrude_node->hybrid->brep) == 0);

    // The re-materialization is deterministic: save the scene back,
    // reload, compare the digests.
    CAD_0::io::native::NativeCamera cam{};
    const std::string text1 =
        CAD_0::io::native::serialize_document(sc, cam);
    const std::string digest1 = query_digest(*extrude_node->hybrid->brep);

    auto doc2 = CAD_0::io::native::parse_document(text1);
    CAD_0::scene::Scene sc2;
    CAD_0::io::native::install_document(sc2, std::move(doc2));
    const CAD_0::scene::SceneNode* extrude2 = find_extrude_node(sc2);
    REQUIRE(extrude2 != nullptr);
    REQUIRE(extrude2->hybrid != nullptr);
    CHECK(query_digest(*extrude2->hybrid->brep) == digest1);

    // The file bytes round-trip identically (the format did not move:
    // same recipe, same serialization — D11).
    const std::string text2 =
        CAD_0::io::native::serialize_document(sc2, cam);
    CHECK(text2 == text1);
}
