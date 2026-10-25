// core/brep/include/CAD_0/brep/subtract.hpp
//
// THE EXACT SUBTRACT — ADR-0032 D5+D7 (G1 unit 4): the exact chain
// extends past the extrude to the SUBTRACT on the G1 class — the
// project's first intersection between curved surfaces, the most
// supervised step of the phase.
//
// THE SSI ENTRY (D5) — plane ∩ cylinder, the three orientations:
//
//   | Orientation          | 3D curve        | Cylinder (u,v) p-curve |
//   |----------------------|-----------------|------------------------|
//   | plane ⊥ axis         | CIRCLE (r)      | the Line2 v = h        |
//   | plane ∥ axis, |d|<r  | LINE PAIR       | the Line2s u = -φ₀ ±   |
//   | plane ∥ axis, |d|=r  | tangent double  | one Line2 (doubled)    |
//   | plane ∥ axis, |d|>r  | empty           | —                       |
//   | oblique              | ELLIPSE         | the SINUSOID z(θ) —    |
//   |                      |                 | NOT typifiable in G1:  |
//   |                      |                 | REFUSED (G3's splines) |
//
// The 3D typed curves: the entry returns the circle as (center,
// radius) data and the lines as (point, direction) — the analytic
// CURVE classes (curve.hpp) are the Y-placement's (CircleCurve lives
// in the XZ plane, axis Y — the same placement split the surfaces
// carry: CylinderSurface vs CylinderZSurface; G1 does not unify
// them, and the Z-placement consumer is this unit). The p-curve
// payloads the PIPELINE authors are typed Line2s on the cylinder and
// Circle2/Line2s on the plane — exact/0, never a smuggled sinusoid.
//
// THE CLASS (D7.1/D7.3, pinned BEFORE the algorithm — the gate is on
// the EXACT MATERIALIZATION, never on the operation):
//
//   * A — convex, planar-faced (a box, a planar prism): every face a
//     plane, every face's normal ⊥ or ∥ to the tool's axis (the
//     oblique refusal names the angle), single-loop faces, and the
//     half-space convexity (every vertex inside every face plane);
//     exactly one +Z face (at z_max) and one −Z face (at z_min) —
//     the pierced pair;
//   * B — a Z-axis prism of the D6 class (the extrude's exact
//     product, or the bridge's fitted box): exactly two ±Z caps and
//     vertical laterals (planes or CylinderZSurface), single-loop,
//     convex, no inner loops (a ring profile's tool is not convex);
//   * both operands EXACT (the unit-1 finding: the booleans consume
//     exactness — an approximated operand refuses loudly, the faceted
//     path stands);
//   * the placement: B translated by the recipe's world offset.
//
// THE PIPELINE (D7.2) for the in-class THROUGH-hole — the only
// productive G1 topology (every other placement is a pinned refusal
// or a miss): the tool's profile strictly inside the pierced faces'
// footprint, the tool's z-span covering the target's. Then:
//
//   1. INTERSECT — the pierced planes × the tool's wall via the D5
//      entry (the circles; on the wall's (u,v) domain the horizontal
//      Line2s v = h — the window's own top/bottom edges);
//   2. CLASSIFY — the footprint relation by the support function
//      (the centroid-then-predicate discipline's coarse cut: the
//      profile's support over each footprint edge), with the
//      declared Layer-3 tolerance band and DETERMINISTIC tie-breaks
//      (canonical-key ordering over the edges, never input order);
//   3. ASSEMBLE — the proven pattern: A's entities copied VERBATIM
//      (ids preserved, the factory's own p-curves intact), the new
//      entities appended by the walk tables below, hard
//      postconditions with the H term.
//
// THE NEW TOPOLOGY the exact path creates (D7.4): the pierced faces
// GAIN INNER LOOPS (the intersection circles, split at the seam pair
// per the ADR-0023 pattern); the surviving wall is the cylindrical
// patch BETWEEN the circles; the tool's caps outside A die (a flush
// cap ABSORBS — the coplanar dedup rule, the counts unchanged); the
// seam carries the wall's periodic identification (its two co-edges
// in the wall's single loop, orientation XOR satisfied).
//
// THE PINNED TABLE of the guide case (box a×b×c, through hole r,
// axis ⊥ the two pierced faces, circles strictly interior):
//
//   | Quantity | Value | Source                                        |
//   |----------|-------|-----------------------------------------------|
//   | V        | 12    | 8 box corners + 4 seam/split (2 per circle)  |
//   | E        | 17    | 12 box + 2+2 arc pairs + 1 seam               |
//   | F        | 7     | 4 sides + 2 pierced + 1 wall                  |
//   | H        | 2     | one inner loop per pierced face               |
//   | χ        | 0     | genus 1 — agrees with the faceted report       |
//
// and the general formula: a chain tool of n entities through a
// target with (V_A, E_A, F_A) gives V = V_A+2n, E = E_A+3n,
// F = F_A+n, H = 2 → χ = 0 for every n — every prism the extrude
// can author is pinned by it (the circle is the seam-wired special
// case).
//
// THE CONVENTION THIS UNIT DISCOVERS (pinned by the tests as
// regression guards — the second real use of the pinned conventions,
// and the code's own answer):
//
//   * the surviving wall keeps the prism's PHYSICAL window walk,
//     re-framed by the flipped orientation (the cavity's outward
//     normal is −radial / the mirrored plane frame): the walk reads
//     REGION-ON-RIGHT in the wall's own domain (interior_left =
//     false on its Line2s). WHY IT IS FORCED: the pierced faces'
//     inner loops must read CCW in their domains for the Arc2
//     vocabulary (CCW sweeps only — the same boundary shape as the
//     unit-3 hole-arc refusal), and the radial-edge XOR ties the
//     two walks together — one requirement, seen from both sides.
//     The unit-3 material-right precedent ("the walk adapts and the
//     bits carry the region") applied to a lateral face;
//   * the inner loops' region bits are FLIPPED from the caps' (the
//     material is now OUTSIDE the profile: the arcs carry
//     hole = true, the segments interior_left = false) — the loop
//     direction follows the pierced face's own frame parity (the +Z
//     face walks the tool's canonical cycle, the −Z face the
//     reversed one — exactly the caps' own walks), and the bits
//     carry the region on every co-edge.
//
// NUMBERING (ADR-0005's discipline — the tables the tests pin):
// A's entities keep their ids (vertices 0..V_A−1, edges 0..E_A−1,
// faces 0..F_A−1, the pierced faces among them); the new entities
// append in the tool's CANONICAL WALK order (the +Z cap's own loop —
// CCW around the profile in the world, the frame rule's guarantee):
//
//   Chain of n entities (J_i = entity i's end joint in the walk):
//     vertices : V_A + 2i = J_i at z_bot, V_A + 2i+1 = J_i at z_top
//     edges    : E_A + i = entity i's bottom curve (J_(i-1) -> J_i),
//                E_A + n+i = its top curve, E_A + 2n+i = the
//                vertical at J_i (bottom -> top)
//     faces    : F_A + i = entity i's wall
//   Circle (the seam-wired special case, n = 2 on one cylinder):
//     vertices : V_A+0..3 = S_b, A_b, S_t, A_t (S = the seam +X,
//                A = the antipode; J_0 = S, J_1 = A)
//     edges    : E_A+0 = the -Y half bottom (A_b -> S_b),
//                E_A+1 = the +Y half bottom (S_b -> A_b),
//                E_A+2/3 = their top copies, E_A+4 = the seam
//                (the vertical at J_0 — NO antipodal vertical: the
//                antipode is the split vertex only)
//     faces    : F_A + 0 = the wall (the ONE face)
//
// Layer-2 (ADR-0032 D2.4): the new entities carry the worst of the
// operands' records (combined_provenance — exact when both are, and
// the class check demands exactly that); A's surviving entities keep
// A's own records untouched.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::brep {

// The operation's declared Layer-3 tolerance (ADR-0004 / ADR-0031 D5:
// decision tolerances are EXPLICIT per operation — no defaults at the
// signature, no global epsilon). The value is consumed at two sites,
// each documented at its use: the ANGULAR classification bands (the
// cosine band |n·â| vs {0, 1} — the kParallelDotMin discipline of
// ADR-0025, now a parameter) and the LINEAR bands (the z-covering,
// the tangency, the SAT separation, the containment margins — mm).
inline constexpr float kSubtractTolerance = 1e-6f;

// ---------------------------------------------------------------------------
// The SSI entry — plane ∩ cylinder (D5)
// ---------------------------------------------------------------------------

// The typed classification + the geometric payload of the
// intersection. The consumer (the pipeline) authors the p-curves in
// ITS faces' domains from this data; the entry itself owns the three
// orientations' mathematics and nothing else.
struct PlaneCylinderSSI {
    enum class Kind : std::uint8_t {
        Circle,      // plane ⊥ axis: the circle (the hole case)
        LinePair,    // plane ∥ axis, |d| < r: two parallel lines
        TangentLine, // plane ∥ axis, |d| = r (within tol): the double line
        Empty,       // plane ∥ axis, |d| > r: no intersection
        Oblique,     // 0 < angle < π/2: the sinusoid p-curve — G3's
    };

    Kind kind{Kind::Oblique};

    // Circle: the centre (where the axis meets the plane) and the
    // radius (the cylinder's own); cylinder_v = the circle's height
    // above the cylinder's z_base (the horizontal Line2's v on the
    // (u,v) domain).
    math::Vec3f circle_center{};
    float circle_radius{0.0f};
    float cylinder_v{0.0f};

    // LinePair / TangentLine: a point on the first line (on the
    // cylinder, at the cylinder's z_base), the shared direction (the
    // axis), the two world azimuths (TangentLine: phi_a == phi_b),
    // and the signed axis-to-plane distance.
    math::Vec3f line_point{};
    math::Vec3f line_dir{};
    float phi_a{0.0f};
    float phi_b{0.0f};
    float axis_distance{0.0f};

    // Oblique: the angle between the plane's normal and the axis
    // (radians) — the refusal's own number.
    float angle{0.0f};
};

// ssi_plane_cylinder — the entry point. `angle_tol` is the cosine
// band that separates ⊥ / ∥ / oblique (the classification
// tolerance); `linear_tol` separates the line pair / tangent / empty
// cases (|d| vs r). Both come from the operation's declared Layer-3
// value. Pure function, deterministic (ADR-0014).
inline PlaneCylinderSSI ssi_plane_cylinder(const PlaneSurface& plane,
                                           const CylinderZSurface& cyl,
                                           float angle_tol, float linear_tol);

// ---------------------------------------------------------------------------
// translate_body — the tool offset, on the exact side
// ---------------------------------------------------------------------------

// Translate a B-Rep body by a world delta: every vertex moves, every
// face's surface is re-anchored (the plane's origin, the cylinder's
// centre / z_base). The loops, the co-edges, the P-CURVES and the
// provenance are UNTOUCHED — the p-curve coordinates are
// frame-relative and the frame moves with its surface (the same
// mathematics, one code path; a second p-curve table would be exactly
// the divergence ADR-0032 exists to prevent). Throws for a surface
// family outside the analytic translation table (loud, never a
// silent skip).
void translate_body(BRepBody& body, math::Vec3f delta);

// ---------------------------------------------------------------------------
// exact_subtract — the G1 class (D7)
// ---------------------------------------------------------------------------

// Compute A \ (B + tool_offset) on the exact G1 class. THE GATE IS ON
// THE MATERIALIZATION, NOT ON THE OPERATION: in-class inputs
// materialize the exact result; every out-of-class input refuses with
// an actionable std::invalid_argument (the diagnosis names what, the
// remedy names how) — the caller records the refusal and the SDF path
// stands (no silent downgrade, no regression).
//
// The in-class outcomes:
//   * the THROUGH-hole (the guide case + the coplanar flush): the
//     pinned topology — the pierced faces gain inner loops, the wall
//     survives between the circles, the tool's caps die or absorb;
//   * the MISS (the tool disjoint from the target, or touching it
//     within the tolerance from outside — the deterministic
//     tie-break: a touch removes nothing): the EXACT COPY of A.
//
// Postconditions (hard, the bridge's culture): the result passes
// is_closed_orientable and loops_are_closed, every edge carries
// exactly two oppositely-oriented co-edges, and χ equals the expected
// value (0 for the through-hole; A's own χ for the miss) — a
// violation throws std::runtime_error (an assembly defect, never a
// returned broken body).
std::unique_ptr<BRepBody> exact_subtract(const BRepBody& A, const BRepBody& B,
                                         math::Vec3f tool_offset,
                                         float tol);

// ===========================================================================
// Implementation
// ===========================================================================

namespace subtract_detail {

inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kTwoPi = 6.28318530717958647692f;

// The domain coordinates of a world point in a plane's (u,v) frame
// (the factories' own projection — one inverse, no second frame).
inline math::Vec2f plane_domain(const PlaneSurface& s, math::Vec3f p) {
    const math::Vec3f d = p - s.origin();
    return {d.dot(s.u_axis()), d.dot(s.v_axis())};
}

// Normalize an angle to [0, 2π).
inline float norm_two_pi(float a) {
    float r = std::fmod(a, kTwoPi);
    if (r < 0.0f) r += kTwoPi;
    return r;
}

} // namespace subtract_detail

// ---------------------------------------------------------------------------
// ssi_plane_cylinder
// ---------------------------------------------------------------------------
inline PlaneCylinderSSI ssi_plane_cylinder(const PlaneSurface& plane,
                                           const CylinderZSurface& cyl,
                                           float angle_tol, float linear_tol) {
    PlaneCylinderSSI out;
    const math::Vec3f n = plane.normal_vec().normalized();
    const math::Vec3f axis{0.0f, 0.0f, 1.0f};
    const float c = std::abs(n.dot(axis));  // |cos| of the normal-axis angle

    if (c > angle_tol && c < 1.0f - angle_tol) {
        // OBLIQUE: the intersection is an ellipse; on the cylinder's
        // (θ, z) domain the p-curve is the SINUSOID z(θ) — not
        // typifiable in the G1 vocabulary (D5's honest boundary; the
        // G3 spline table owns it). Refused with the angle.
        out.kind = PlaneCylinderSSI::Kind::Oblique;
        out.angle = std::acos(std::min(1.0f, c));
        return out;
    }

    const math::Vec3f p0 = plane.origin();
    if (c <= angle_tol) {
        // Plane ∥ axis: the two vertical lines where the cylinder
        // meets the plane. The plane's horizontal normal direction:
        // mu = the azimuth of (n.x, n.y) (n is unit; |n.z| ≈ 0 here).
        const float mu = std::atan2(n.y, n.x);
        // The signed offset of the plane from the axis, along the
        // plane's horizontal normal: D = (p0 - c)·n_xy. The cylinder
        // meets the plane where cos(phi - mu) = D/r (the classical
        // pair / tangent / empty trichotomy).
        const float D = (p0.x - cyl.axis_center().x) * n.x +
                        (p0.y - cyl.axis_center().y) * n.y;
        const float d = D;
        const float r = cyl.radius();
        out.axis_distance = d;
        if (std::abs(d) > r + linear_tol) {
            out.kind = PlaneCylinderSSI::Kind::Empty;
            return out;
        }
        if (std::abs(std::abs(d) - r) <= linear_tol) {
            // Tangent: the double line at azimuth mu (for d ≈ +r the
            // touch is on the +mu side; for d ≈ -r on the -mu side).
            out.kind = PlaneCylinderSSI::Kind::TangentLine;
            const float phi = d >= 0.0f ? mu : mu + subtract_detail::kPi;
            out.phi_a = out.phi_b = phi;
            out.line_dir = axis;
            out.line_point = {cyl.axis_center().x + r * std::cos(phi),
                              cyl.axis_center().y + r * std::sin(phi),
                              cyl.z_base()};
            return out;
        }
        // The pair: phi = mu ± arccos(d / r).
        out.kind = PlaneCylinderSSI::Kind::LinePair;
        const float theta0 = std::acos(std::max(-1.0f, std::min(1.0f, d / r)));
        out.phi_a = subtract_detail::norm_two_pi(mu + theta0);
        out.phi_b = subtract_detail::norm_two_pi(mu - theta0);
        out.line_dir = axis;
        out.line_point = {cyl.axis_center().x + r * std::cos(out.phi_a),
                          cyl.axis_center().y + r * std::sin(out.phi_a),
                          cyl.z_base()};
        return out;
    }

    // Plane ⊥ axis: the CIRCLE, centred where the axis meets the
    // plane, radius the cylinder's own. On the cylinder's (u,v)
    // domain: the horizontal Line2 v = z_plane - z_base. (The plane's
    // origin is ON the plane and a ⊥ plane is horizontal, so its z
    // IS the plane's z.)
    out.kind = PlaneCylinderSSI::Kind::Circle;
    out.circle_center = {cyl.axis_center().x, cyl.axis_center().y,
                         plane.origin().z};
    out.circle_radius = cyl.radius();
    out.cylinder_v = plane.origin().z - cyl.z_base();
    return out;
}

// ---------------------------------------------------------------------------
// translate_body
// ---------------------------------------------------------------------------
inline void translate_body(BRepBody& body, math::Vec3f delta) {
    for (auto& v : body.vertices)
        v.set_position(v.position() + delta);
    for (auto& f : body.solid.shell_mut().faces_mut()) {
        const auto* surf = f.surface();
        if (surf == nullptr) {
            continue;  // (a faceless face cannot move; the class
                       // checks refuse such bodies long before here)
        }
        const std::string family = surf->type_name();
        if (family == "plane") {
            const auto* pl = dynamic_cast<const PlaneSurface*>(surf);
            f.set_surface(std::make_shared<PlaneSurface>(pl->origin() + delta,
                                                         pl->normal_vec()));
        } else if (family == "cylinder") {
            // The extrude's own Z-axis cylinder OR the primitive
            // factory's Y-axis one — both carry their placement in
            // the surface; both re-anchor the same way through their
            // own accessors.
            if (const auto* cz =
                    dynamic_cast<const CylinderZSurface*>(surf)) {
                f.set_surface(std::make_shared<CylinderZSurface>(
                    cz->axis_center() +
                        math::Vec2f{delta.x, delta.y},
                    cz->radius(), cz->z_base() + delta.z,
                    cz->u_sign() > 0.0f ? +1 : -1));
            } else if (const auto* cy =
                           dynamic_cast<const CylinderSurface*>(surf)) {
                f.set_surface(std::make_shared<CylinderSurface>(
                    cy->radius(), cy->height(), cy->origin() + delta));
            } else {
                throw std::invalid_argument(
                    "translate_body: unknown 'cylinder' surface subtype "
                    "- the translation table covers CylinderSurface and "
                    "CylinderZSurface (a third subtype would be a kernel "
                    "extension, not a silent skip). Remedy: extend the "
                    "table with its own placement accessor.");
            }
        } else if (family == "sphere") {
            const auto* sp = dynamic_cast<const SphereSurface*>(surf);
            f.set_surface(std::make_shared<SphereSurface>(sp->radius(),
                                                          sp->origin() +
                                                              delta));
        } else if (family == "cone") {
            const auto* co = dynamic_cast<const ConeSurface*>(surf);
            f.set_surface(std::make_shared<ConeSurface>(co->base_radius(),
                                                        co->height(),
                                                        co->origin() + delta));
        } else if (family == "torus") {
            const auto* to = dynamic_cast<const TorusSurface*>(surf);
            f.set_surface(std::make_shared<TorusSurface>(to->major_radius(),
                                                         to->minor_radius(),
                                                         to->origin() + delta));
        } else {
            throw std::invalid_argument(
                "translate_body: the surface family '" + family +
                "' is outside the analytic translation table (plane, "
                "cylinder, sphere, cone, torus — the G1 surfaces with "
                "exact consumers). Remedy: the operand's class does not "
                "reach the exact subtract; report the body.");
        }
    }
}

// ---------------------------------------------------------------------------
// The implementation core
// ---------------------------------------------------------------------------
namespace subtract_detail {

// One entity of the tool's CANONICAL WALK (the +Z cap's own loop, in
// order — CCW around the profile in the world, the frame rule's
// guarantee for every exact prism's +Z cap).
struct WalkEntity {
    enum class Kind : std::uint8_t { Segment, Arc };
    Kind kind{Kind::Segment};
    math::Vec2f p{}, q{};  // the traversal's world endpoints
    // Arc only: the circle + the CCW world span the walk traverses
    // (the canonical walk runs psi-up on every fillet arc — the +Z
    // cap's loop is CCW around the profile, and a fillet arc's
    // material lies toward its centre).
    math::Vec2f center{};
    float radius{0.0f};
    float psi_start{0.0f};
    float sweep{0.0f};
};

// The tool as the pipeline consumes it: the canonical walk + the
// caps' z (already translated).
struct ToolWalk {
    std::vector<WalkEntity> entities;
    bool is_circle{false};  // the seam-wired special case (n = 2 arcs
                            // of one circle totalling 2π)
    math::Vec2f circle_center{};
    float circle_radius{0.0f};
    float z_lo{0.0f}, z_hi{0.0f};
};

// The +Z cap's surface, verified planar with a +Z normal.
inline const PlaneSurface* cap_surface_of(const Face& f) {
    const auto* surf = f.surface();
    if (surf == nullptr) return nullptr;
    if (std::string(surf->type_name()) != "plane") return nullptr;
    const auto* pl = dynamic_cast<const PlaneSurface*>(surf);
    if (pl == nullptr) return nullptr;
    if (pl->normal_vec().z < 0.9f) return nullptr;  // not +Z-ish
    return pl;
}

// Read the canonical walk from the +Z cap's outer loop. Guards (each
// loud, each naming the remedy — a malformed tool is a kernel bug or
// a non-prism body that slipped the class check):
//   * every co-edge carries a Segment or Arc p-curve (the exact
//     prisms' caps author exactly these; a Circle2 on the OUTER loop
//     is not a shape the exact world produces);
//   * the walk chains endpoint-to-endpoint in the world;
//   * every arc's world sweep matches its payload (the frame rule's
//     rotation preserves it) and runs psi-up.
inline ToolWalk read_tool_walk(const BRepBody& B) {
    ToolWalk w;
    // The caps: the +Z planar cap (the walk's source) and the -Z one
    // (the tool's lower bound).
    const PlaneSurface* cap = nullptr;
    const Face* cap_face = nullptr;
    for (const auto& f : B.solid.shell().faces()) {
        if (const PlaneSurface* pl = cap_surface_of(f)) {
            cap = pl;
            cap_face = &f;
            break;
        }
    }
    if (cap == nullptr || cap_face == nullptr)
        throw std::invalid_argument(
            "exact_subtract: the tool has no +Z planar cap to read the "
            "profile from - not a prism of the D6 class. Remedy: use an "
            "extruded profile (or the fitted box) as the tool.");
    w.z_hi = cap->origin().z;
    for (const auto& f : B.solid.shell().faces()) {
        const auto* surf = f.surface();
        if (surf == nullptr) continue;
        if (std::string(surf->type_name()) != "plane") continue;
        const auto* pl = dynamic_cast<const PlaneSurface*>(surf);
        if (pl == nullptr || pl->normal_vec().z > -0.9f) continue;
        w.z_lo = pl->origin().z;
        break;
    }

    const auto& loop = cap_face->loops();
    if (loop.empty())
        throw std::invalid_argument(
            "exact_subtract: the tool's cap carries no loop - a "
            "malformed prism. Remedy: report the tool's construction.");

    const auto dom = [cap](const math::Vec2f& uv) {
        return cap->evaluate(uv.x, uv.y);
    };
    for (const auto& ce : loop[0].coedges()) {
        const auto* pc = ce.p_curve();
        if (pc == nullptr ||
            (pc->kind != sdf::SketchEntity::Kind::Segment &&
             pc->kind != sdf::SketchEntity::Kind::Arc)) {
            throw std::invalid_argument(
                "exact_subtract: the tool's cap carries a p-curve outside "
                "the prism vocabulary (a Segment or an Arc per co-edge, "
                "the exact extrude's own authoring). Remedy: report the "
                "tool's construction.");
        }
        WalkEntity e;
        if (pc->kind == sdf::SketchEntity::Kind::Segment) {
            e.kind = WalkEntity::Kind::Segment;
            const math::Vec3f a = dom(pc->segment.a);
            const math::Vec3f b = dom(pc->segment.b);
            e.p = {a.x, a.y};
            e.q = {b.x, b.y};
        } else {
            e.kind = WalkEntity::Kind::Arc;
            const math::Vec3f c = dom(math::Vec2f{pc->arc.center.x,
                                                  pc->arc.center.y});
            e.center = {c.x, c.y};
            e.radius = pc->arc.radius;
            e.sweep = pc->arc.sweep;
            const math::Vec3f a = dom(sdf::arc_start_point(pc->arc));
            const math::Vec3f b = dom(sdf::arc_end_point(pc->arc));
            e.p = {a.x, a.y};
            e.q = {b.x, b.y};
            // The world span: the cap's domain is right-handed around
            // +Z (the frame rule), so the domain's CCW sweep IS the
            // world's — verify the endpoints agree (the guard).
            e.psi_start = std::atan2(e.p.y - e.center.y,
                                     e.p.x - e.center.x);
            const float psi_q = std::atan2(e.q.y - e.center.y,
                                           e.q.x - e.center.x);
            float span = norm_two_pi(psi_q - e.psi_start);
            if (std::abs(span - e.sweep) > 1e-4f &&
                std::abs(span - e.sweep + kTwoPi) > 1e-4f) {
                throw std::invalid_argument(
                    "exact_subtract: the tool's arc p-curve disagrees "
                    "with its world image (domain sweep " +
                    std::to_string(e.sweep) + " vs world span " +
                    std::to_string(span) + ") - the cap's frame is not "
                    "the right-handed +Z plane the exact world authors. "
                    "Remedy: report the tool's construction.");
            }
        }
        w.entities.push_back(e);
    }
    // The chaining guard: the walk closes in the world.
    if (!w.entities.empty()) {
        const math::Vec2f first = w.entities.front().p;
        math::Vec2f cursor = w.entities.front().p;
        for (const auto& e : w.entities) {
            if (std::abs(e.p.x - cursor.x) > 1e-4f ||
                std::abs(e.p.y - cursor.y) > 1e-4f)
                throw std::invalid_argument(
                    "exact_subtract: the tool's canonical walk does not "
                    "chain endpoint-to-endpoint - a malformed prism cap. "
                    "Remedy: report the tool's construction.");
            cursor = e.q;
        }
        if (std::abs(cursor.x - first.x) > 1e-4f ||
            std::abs(cursor.y - first.y) > 1e-4f)
            throw std::invalid_argument(
                "exact_subtract: the tool's canonical walk does not "
                "close - a malformed prism cap. Remedy: report the "
                "tool's construction.");
    } else {
        throw std::invalid_argument(
            "exact_subtract: the tool's cap loop is empty - a "
            "malformed prism. Remedy: report the tool's construction.");
    }

    // The circle detection: two arcs of one circle totalling 2π.
    w.is_circle = false;
    if (w.entities.size() == 2 &&
        w.entities[0].kind == WalkEntity::Kind::Arc &&
        w.entities[1].kind == WalkEntity::Kind::Arc) {
        const auto& e0 = w.entities[0];
        const auto& e1 = w.entities[1];
        if (std::abs(e0.center.x - e1.center.x) <= 1e-5f &&
            std::abs(e0.center.y - e1.center.y) <= 1e-5f &&
            std::abs(e0.radius - e1.radius) <= 1e-5f &&
            std::abs(e0.sweep + e1.sweep - kTwoPi) <= 1e-4f) {
            w.is_circle = true;
            w.circle_center = e0.center;
            w.circle_radius = e0.radius;
        }
    }
    return w;
}

// The target as the pipeline consumes it (the class check's product).
struct TargetInfo {
    std::size_t top_face{0}, bottom_face{0};  // the pierced pair
    float z_top{0.0f}, z_bot{0.0f};
    // The footprint (the pierced faces' own polygon — the same region
    // for a convex all-vertical-sides body, read from the TOP face's
    // outer loop, verified CCW-in-world):
    std::vector<math::Vec2f> verts;   // in loop order
    std::vector<math::Vec2f> normals; // unit outward normal per edge
};

// The tool profile's support along a unit direction d (the coarse
// cut of the centroid-then-predicate discipline: the vertices carry
// the segments, the arcs add their extreme point when the direction's
// azimuth falls inside the span).
inline float profile_support(const ToolWalk& w, math::Vec2f d) {
    float m = -std::numeric_limits<float>::infinity();
    for (const auto& e : w.entities) {
        m = std::max(m, d.x * e.p.x + d.y * e.p.y);
        m = std::max(m, d.x * e.q.x + d.y * e.q.y);
        if (e.kind == WalkEntity::Kind::Arc) {
            const float phi = norm_two_pi(std::atan2(d.y, d.x));
            const float rel = norm_two_pi(phi - e.psi_start);
            if (rel <= e.sweep)
                m = std::max(m, d.x * e.center.x + d.y * e.center.y +
                                      e.radius);
        }
    }
    return m;
}

// Is a point inside the tool's profile (the convex region: every
// entity's material side, with the tolerance band)?
inline bool point_in_profile(const ToolWalk& w, math::Vec2f pt, float tol) {
    for (const auto& e : w.entities) {
        if (e.kind == WalkEntity::Kind::Segment) {
            // The profile's material is on the LEFT of p->q: the
            // outward normal is the right normal.
            const math::Vec2f d{e.q.x - e.p.x, e.q.y - e.p.y};
            const float len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len <= 1e-9f) continue;
            const math::Vec2f n{d.y / len, -d.x / len};
            if ((pt.x - e.p.x) * n.x + (pt.y - e.p.y) * n.y > tol)
                return false;
        } else {
            if (std::hypot(pt.x - e.center.x, pt.y - e.center.y) >
                e.radius + tol)
                return false;
        }
    }
    return true;
}

} // namespace subtract_detail

// ---------------------------------------------------------------------------
// exact_subtract
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> exact_subtract(const BRepBody& A,
                                                const BRepBody& B,
                                                math::Vec3f tool_offset,
                                                float tol) {
    // ==== THE CLASS CHECK (D7.1/D7.3) — pinned BEFORE the algorithm =
    //
    // A: convex, planar-faced, every face ⊥ or ∥ to the axis.
    {
        std::size_t pos_z = 0, neg_z = 0;
        for (const auto& f : A.solid.shell().faces()) {
            const auto* surf = f.surface();
            if (surf == nullptr || std::string(surf->type_name()) != "plane")
                throw std::invalid_argument(
                    "exact_subtract: the target carries a non-planar face "
                    "(family '" +
                    (surf ? std::string(surf->type_name()) : std::string("none")) +
                    "') - the G1 class takes convex PLANAR-faced targets (a "
                    "box, a planar prism); a curved target face meets the "
                    "tool in the general SSI table (G2's). Remedy: use a "
                    "planar-faced target, or accept the faceted path (the "
                    "SDF materialization stands).");
            const auto* pl = dynamic_cast<const PlaneSurface*>(surf);
            const float nz = pl->normal_vec().z;
            if (std::abs(nz) > tol && std::abs(nz) < 1.0f - tol)
                throw std::invalid_argument(
                    "exact_subtract: the target has an OBLIQUE face (its "
                    "normal makes angle " +
                    std::to_string(std::acos(std::min(1.0f, std::abs(nz)))) +
                    " rad with the tool's axis) - the oblique plane∩cylinder "
                    "SSI is an ellipse whose cylinder-domain p-curve is a "
                    "SINUSOID, not typifiable in the G1 vocabulary "
                    "(ADR-0032 D5: the G3 spline table). Remedy: reorient "
                    "the target's face (axis-aligned), or accept the "
                    "faceted path.");
            if (nz >= 1.0f - tol) ++pos_z;
            if (nz <= -(1.0f - tol)) ++neg_z;
            if (f.loops().size() != 1)
                throw std::invalid_argument(
                    "exact_subtract: the target carries a face with inner "
                    "loops - not a convex planar body (a pierced or holed "
                    "target is outside the G1 class: the exact subtract "
                    "takes the FIRST cut on a convex target). Remedy: "
                    "reorder the operations so this subtract's target is "
                    "uncut, or accept the faceted path.");
        }
        if (pos_z != 1 || neg_z != 1)
            throw std::invalid_argument(
                "exact_subtract: the target lacks the pierced pair "
                "(exactly one +Z face and one -Z face; found " +
                std::to_string(pos_z) + " and " + std::to_string(neg_z) +
                ") - not the box/prism class. Remedy: use a box or planar "
                "prism target, or accept the faceted path.");
    }

    // A's half-space convexity: every vertex inside every face plane.
    for (const auto& f : A.solid.shell().faces()) {
        const auto* pl =
            dynamic_cast<const PlaneSurface*>(f.surface());
        const math::Vec3f o = pl->origin();
        const math::Vec3f n = pl->normal_vec();
        for (const auto& v : A.vertices) {
            if ((v.position() - o).dot(n) > tol)
                throw std::invalid_argument(
                    "exact_subtract: the target is not CONVEX (a vertex "
                    "lies outside a face's half-space) - the G1 class "
                    "takes convex planar targets. Remedy: use a convex "
                    "target, or accept the faceted path.");
        }
    }

    // The operand exactness (the unit-1 finding: the booleans consume
    // exactness — the honesty chain).
    {
        const auto check = [](const BRepBody& b, const char* what) {
            const auto consider = [&](const ProvenanceInfo& p) {
                if (!p.is_exact())
                    throw std::invalid_argument(
                        std::string("exact_subtract: the ") + what +
                        "'s materialization is not exact (provenance "
                        "approximated, tolerance " +
                        std::to_string(p.linear_tolerance) +
                        ") - the exact subtract consumes EXACT operands "
                        "(the booleans consume exactness, ADR-0032 D8's "
                        "honesty chain); the faceted path stands. Remedy: "
                        "re-materialize the operand exactly (a factory "
                        "primitive or an in-class operation), or accept "
                        "the faceted path.");
            };
            for (const auto& v : b.vertices) consider(v.provenance());
            for (const auto& e : b.edges) consider(e.provenance());
            for (const auto& f : b.solid.shell().faces())
                consider(f.provenance());
        };
        check(A, "target");
        check(B, "tool");
    }

    // B: the Z-axis prism signature (D6's class).
    {
        std::size_t pos_z = 0, neg_z = 0;
        for (const auto& f : B.solid.shell().faces()) {
            const auto* surf = f.surface();
            if (surf == nullptr ||
                (std::string(surf->type_name()) != "plane" &&
                 std::string(surf->type_name()) != "cylinder"))
                throw std::invalid_argument(
                    "exact_subtract: the tool is not a Z-axis prism of "
                    "the D6 class (a face of family '" +
                    (surf ? std::string(surf->type_name())
                          : std::string("none")) +
                    "') - the G1 exact subtract consumes the exact "
                    "extrude's prisms (a convex profile or a circle), "
                    "placed by the recipe's offset. Remedy: use an "
                    "extruded profile (or the fitted box) as the tool, "
                    "or accept the faceted path.");
            if (std::string(surf->type_name()) == "cylinder" &&
                dynamic_cast<const CylinderZSurface*>(surf) == nullptr)
                throw std::invalid_argument(
                    "exact_subtract: the tool is not a Z-axis prism of "
                    "the D6 class (its cylinder's axis is Y - the "
                    "primitive factory's placement) - the G1 exact "
                    "subtract consumes the exact extrude's Z-axis "
                    "prisms. Remedy: model the tool as an extruded "
                    "circle profile, or accept the faceted path.");
            const float nz =
                std::string(surf->type_name()) == "plane"
                    ? dynamic_cast<const PlaneSurface*>(surf)
                          ->normal_vec().z
                    : 0.0f;  // cylinder walls are ∥ to the axis
            if (std::string(surf->type_name()) == "plane") {
                if (nz > 1.0f - tol) ++pos_z;
                else if (nz < -(1.0f - tol)) ++neg_z;
                else if (std::abs(nz) > tol)
                    throw std::invalid_argument(
                        "exact_subtract: the tool has an OBLIQUE face - "
                        "not a prism of the D6 class. Remedy: use an "
                        "extruded profile as the tool, or accept the "
                        "faceted path.");
            }
            if (f.loops().size() != 1)
                throw std::invalid_argument(
                    "exact_subtract: the tool is not CONVEX (a face "
                    "carries inner loops - a ring profile's tool: the "
                    "caps' holes make it non-convex) - the G1 class "
                    "takes a convex profile or a circle. Remedy: use a "
                    "convex tool profile, or accept the faceted path.");
        }
        if (pos_z != 1 || neg_z != 1)
            throw std::invalid_argument(
                "exact_subtract: the tool is not a Z-axis prism of the "
                "D6 class (it has " + std::to_string(pos_z) + " +Z faces "
                "and " + std::to_string(neg_z) + " -Z faces; a prism has "
                "exactly one of each - its caps) - the G1 exact subtract "
                "consumes the exact extrude's prisms. Remedy: use an "
                "extruded profile (or the fitted box) as the tool, or "
                "accept the faceted path.");
    }

    // B's half-space convexity over its planar faces (the L-profile
    // and every non-convex segment chain land here).
    for (const auto& f : B.solid.shell().faces()) {
        if (std::string(f.surface()->type_name()) != "plane") continue;
        const auto* pl =
            dynamic_cast<const PlaneSurface*>(f.surface());
        const math::Vec3f o = pl->origin();
        const math::Vec3f n = pl->normal_vec();
        for (const auto& v : B.vertices) {
            if ((v.position() - o).dot(n) > tol)
                throw std::invalid_argument(
                    "exact_subtract: the tool is not CONVEX (a vertex "
                    "lies outside a face's half-space - a reflex profile "
                    "corner) - the G1 class takes a convex profile or a "
                    "circle. Remedy: use a convex tool profile, or "
                    "accept the faceted path.");
        }
    }

    // ==== THE PIPELINE (D7.2) ==========================================

    // B' = the translated tool (the recipe's world offset).
    auto bp = std::make_unique<BRepBody>(B);
    translate_body(*bp, tool_offset);

    // The walk (the canonical cycle) + the caps' z.
    const subtract_detail::ToolWalk w = subtract_detail::read_tool_walk(*bp);

    // The target's pierced pair + the footprint.
    subtract_detail::TargetInfo t;
    {
        bool have_top = false, have_bot = false;
        for (std::size_t i = 0; i < A.face_count(); ++i) {
            const auto& f = A.solid.shell().faces()[i];
            const auto* pl =
                dynamic_cast<const PlaneSurface*>(f.surface());
            const float nz = pl->normal_vec().z;
            if (nz >= 1.0f - tol) {
                t.top_face = i;
                t.z_top = pl->origin().z;
                have_top = true;
            } else if (nz <= -(1.0f - tol)) {
                t.bottom_face = i;
                t.z_bot = pl->origin().z;
                have_bot = true;
            }
        }
        if (!have_top || !have_bot || !(t.z_top > t.z_bot))
            throw std::invalid_argument(
                "exact_subtract: the target's pierced pair is degenerate "
                "(the +Z and -Z faces do not bound a positive z span) - "
                "not the box/prism class. Remedy: use a box or planar "
                "prism target, or accept the faceted path.");

        // The footprint from the TOP face's outer loop (CCW in the
        // world — the material-on-left authoring's guarantee; the
        // signed-area guard makes it loud if a foreign authoring
        // appears).
        const auto& top = A.solid.shell().faces()[t.top_face];
        const auto* top_plane =
            dynamic_cast<const PlaneSurface*>(top.surface());
        double area2 = 0.0;
        for (const auto& ce : top.loops()[0].coedges()) {
            const VertexId vid = ce.start_vertex(A.edges);
            t.verts.push_back(
                {A.vertices[vid].position().x, A.vertices[vid].position().y});
        }
        for (std::size_t i = 0; i < t.verts.size(); ++i) {
            const math::Vec2f& a = t.verts[i];
            const math::Vec2f& b = t.verts[(i + 1) % t.verts.size()];
            area2 += static_cast<double>(a.x) * b.y -
                     static_cast<double>(b.x) * a.y;
            const math::Vec2f d{b.x - a.x, b.y - a.y};
            const float len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len <= 1e-9f)
                throw std::invalid_argument(
                    "exact_subtract: the target's pierced face has a "
                    "degenerate edge - a malformed target. Remedy: "
                    "report the target's construction.");
            t.normals.push_back({d.y / len, -d.x / len});
        }
        if (area2 <= 0.0)
            throw std::invalid_argument(
                "exact_subtract: the target's pierced face loop is not "
                "CCW in the world (signed area " +
                std::to_string(area2 * 0.5) +
                ") - the exact world's material-on-left authoring is "
                "violated. Remedy: report the target's construction.");
        (void)top_plane;
    }

    // ---- The placement decision tree --------------------------------
    // (1) The z-side miss: the tool's span does not reach the
    //     target's, or touches it from outside within the tolerance
    //     (the deterministic tie-break: a touch removes nothing).
    const float z_lo = w.z_lo;  // the translated caps
    const float z_hi = w.z_hi;
    if (z_hi <= t.z_bot + tol || z_lo >= t.z_top - tol) {
        // The EXACT COPY of A (see below — the same code path).
        auto r = std::make_unique<BRepBody>(A);
        link_radial_edges(*r);
        return r;
    }

    // (2) The footprint relation: the SAT over the canonical axes,
    //     then the swallow, then the containment bands.
    {
        // The SAT axes: the footprint's edge normals + the profile's
        // segment normals + the arc-vertex radial directions.
        std::vector<math::Vec2f> axes = t.normals;
        for (const auto& e : w.entities) {
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                const math::Vec2f d{e.q.x - e.p.x, e.q.y - e.p.y};
                const float len = std::sqrt(d.x * d.x + d.y * d.y);
                if (len > 1e-9f)
                    axes.push_back({d.y / len, -d.x / len});
            } else {
                for (const auto& v : t.verts) {
                    const math::Vec2f d{v.x - e.center.x, v.y - e.center.y};
                    const float len = std::sqrt(d.x * d.x + d.y * d.y);
                    if (len > 1e-9f)
                        axes.push_back({d.x / len, d.y / len});
                }
            }
        }
        float best_gap = -std::numeric_limits<float>::infinity();
        for (const auto& a : axes) {
            float fp_min = std::numeric_limits<float>::infinity();
            for (const auto& v : t.verts)
                fp_min = std::min(fp_min, a.x * v.x + a.y * v.y);
            const float gap =
                fp_min - subtract_detail::profile_support(w, a);
            best_gap = std::max(best_gap, gap);
        }
        if (best_gap > -tol) {
            // Separated, or touching within the band: nothing (above
            // the tolerance) is removed — the exact copy.
            auto r = std::make_unique<BRepBody>(A);
            link_radial_edges(*r);
            return r;
        }

        // The swallow: every footprint vertex inside the profile.
        bool swallow = true;
        for (const auto& v : t.verts)
            if (!subtract_detail::point_in_profile(w, v, tol)) {
                swallow = false;
                break;
            }
        if (swallow)
            throw std::invalid_argument(
                "exact_subtract: the tool's profile SWALLOWS the "
                "target's cross-section (every pierced-face vertex "
                "lies inside the profile) - the exact result would be "
                "the empty solid. Remedy: use a smaller tool, or "
                "accept the faceted path.");

        // The containment bands per footprint edge — the CANONICAL KEY
        // tie-break: the triggering edges are ordered by their
        // geometric key (origin, then normal — lexicographic), never
        // by input order, so the same geometry always reports the
        // same edge.
        struct Trigger {
            math::Vec2f origin, normal;
            float margin;
        };
        std::vector<Trigger> crossing, tangential;
        for (std::size_t i = 0; i < t.verts.size(); ++i) {
            const math::Vec2f& n = t.normals[i];
            const float line = n.x * t.verts[i].x + n.y * t.verts[i].y;
            const float margin =
                subtract_detail::profile_support(w, n) - line;
            if (margin > tol)
                crossing.push_back({t.verts[i], n, margin});
            else if (margin > -tol)
                tangential.push_back({t.verts[i], n, margin});
        }
        const auto canonical_less = [](const Trigger& a, const Trigger& b) {
            if (a.origin.x != b.origin.x) return a.origin.x < b.origin.x;
            if (a.origin.y != b.origin.y) return a.origin.y < b.origin.y;
            if (a.normal.x != b.normal.x) return a.normal.x < b.normal.x;
            return a.normal.y < b.normal.y;
        };
        if (!crossing.empty()) {
            std::sort(crossing.begin(), crossing.end(), canonical_less);
            throw std::invalid_argument(
                "exact_subtract: the tool's profile CROSSES the boundary "
                "of the pierced face (it pokes beyond the edge from (" +
                std::to_string(crossing[0].origin.x) + ", " +
                std::to_string(crossing[0].origin.y) + ") outward along (" +
                std::to_string(crossing[0].normal.x) + ", " +
                std::to_string(crossing[0].normal.y) + ") by " +
                std::to_string(crossing[0].margin) +
                ") - the non-annular pierced face is honest G2 "
                "topology, not G1. Remedy: move or resize the hole so "
                "its profile stays strictly inside the pierced face, "
                "or accept the faceted path.");
        }
        if (!tangential.empty()) {
            std::sort(tangential.begin(), tangential.end(), canonical_less);
            throw std::invalid_argument(
                "exact_subtract: the tool's wall is TANGENT to the "
                "pierced face's boundary from inside (the clearance to "
                "the edge from (" +
                std::to_string(tangential[0].origin.x) + ", " +
                std::to_string(tangential[0].origin.y) + ") outward is " +
                std::to_string(-tangential[0].margin) +
                ", within the declared tolerance " + std::to_string(tol) +
                ") - the pierced face would pinch at the tangency (a "
                "non-manifold vertex on its boundary). The tie-break is "
                "DETERMINISTIC (canonical-key ordering over the face's "
                "edges, never input order) and the pinned disposition "
                "is: refuse. Remedy: move or resize the hole by more "
                "than the tolerance, or accept the faceted path.");
        }
    }

    // (3) The z-bands: the blind hole vs the through-hole.
    const bool blind_lo = z_lo > t.z_bot + tol && z_lo < t.z_top - tol;
    const bool blind_hi = z_hi > t.z_bot + tol && z_hi < t.z_top - tol;
    if (blind_lo || blind_hi)
        throw std::invalid_argument(
            "exact_subtract: the BLIND hole - the tool terminates inside "
            "the target (its " +
            std::string(blind_lo ? "bottom" : "top") + " cap at z = " +
            std::to_string(blind_lo ? z_lo : z_hi) +
            " lies strictly between the pierced faces z = [" +
            std::to_string(t.z_bot) + ", " + std::to_string(t.z_top) +
            "]) - the internal cap face class is G2's (it needs the "
            "SSI-driven cap insertion, not the pass-through circle pair "
            "G1 builds). Remedy: extend the extrusion distance so the "
            "tool passes through, or accept the faceted path.");
    if (!(z_lo <= t.z_bot + tol && z_hi >= t.z_top - tol))
        throw std::logic_error(
            "exact_subtract: the placement decision tree fell through - "
            "an internal inconsistency (the miss, blind and through "
            "cases should be exhaustive); report this as a kernel bug.");

    // ==== THE ASSEMBLY — the through-hole builder (D7.2 step 3) ====
    //
    // A's entities copied VERBATIM (the ids preserved, the factory's
    // own p-curves intact); the new entities appended by the walk
    // tables; the pierced faces GAIN their inner loops; the walls
    // re-time the prism's windows to [z_bot, z_top] on the FLIPPED
    // orientation (region-on-right); the tool's caps die (a flush cap
    // absorbed — nothing is emitted for them, which IS the dedup).
    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    R->vertices = A.vertices;
    R->edges = A.edges;
    for (const auto& f : A.solid.shell().faces()) shell.add_face(f);

    const std::size_t n = w.entities.size();
    const std::size_t v_base = R->vertices.size();
    const std::size_t e_base = R->edges.size();
    // (the new faces number themselves by the shell's running size,
    // which starts at A's own face count — the factories' discipline)
    const float h = t.z_top - t.z_bot;
    const ProvenanceInfo prov = combined_provenance(A, B);

    const auto vid = [&](std::size_t i) { return static_cast<VertexId>(i); };
    const auto eid = [&](std::size_t i) { return static_cast<EdgeId>(i); };

    // The joints of the canonical walk: J_i = entity i's q.
    std::vector<math::Vec2f> J(n), P(n);
    for (std::size_t i = 0; i < n; ++i) {
        J[i] = w.entities[i].q;
        P[i] = w.entities[i].p;  // == J[(i + n - 1) % n] (the guard proved it)
    }

    // ---- the new vertices: 2i = J_i at z_bot, 2i+1 = J_i at z_top
    for (std::size_t i = 0; i < n; ++i) {
        Vertex vb{math::Vec3f{J[i].x, J[i].y, t.z_bot}};
        vb.set_id(vid(v_base + 2 * i));
        vb.set_provenance(prov);
        R->vertices.push_back(vb);
        Vertex vt{math::Vec3f{J[i].x, J[i].y, t.z_top}};
        vt.set_id(vid(v_base + 2 * i + 1));
        vt.set_provenance(prov);
        R->vertices.push_back(vt);
    }

    // ---- the new edges
    // Chain edges: i = entity i's bottom (J_(i-1) -> J_i, the
    // canonical direction), n+i = its top, 2n+i = the vertical at J_i.
    // (The circle case creates the vertical at J_0 ONLY — the seam.)
    const bool is_circle = w.is_circle;
    const std::size_t verticals = is_circle ? 1 : n;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        Edge eb{vid(v_base + 2 * prev), vid(v_base + 2 * i)};
        eb.set_id(eid(e_base + i));
        eb.set_provenance(prov);
        R->edges.push_back(eb);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n;
        Edge et{vid(v_base + 2 * prev + 1), vid(v_base + 2 * i + 1)};
        et.set_id(eid(e_base + n + i));
        et.set_provenance(prov);
        R->edges.push_back(et);
    }
    for (std::size_t i = 0; i < verticals; ++i) {
        Edge ev{vid(v_base + 2 * i), vid(v_base + 2 * i + 1)};
        ev.set_id(eid(e_base + 2 * n + i));
        ev.set_provenance(prov);
        R->edges.push_back(ev);
    }

    // A local Line2 authoring helper (region-on-right throughout —
    // the convention this unit pins for the surviving walls).
    const auto line_rr = [](math::Vec2f a, math::Vec2f b) {
        return sdf::sketch_segment(a, b, /*interior_left=*/false);
    };

    // ---- the walls ---------------------------------------------------
    const auto add_wall_face = [&](std::shared_ptr<SurfaceGeometry> surf,
                                   Loop loop) {
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        shell.add_face(std::move(face));
    };

    if (!is_circle) {
        // One wall per entity: the plane (segments) or the cylindrical
        // patch (arcs), the window re-timed to [z_bot, z_top], the
        // walk [bottom FWD, up at J_i FWD, top REV, down at J_(i-1)
        // REV] — the prism's physical walk, region-on-right.
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t prev = (i + n - 1) % n;
            const auto& e = w.entities[i];
            Loop loop;
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                // The cavity's outward normal: the LEFT normal of the
                // canonical direction (the profile's interior side).
                const math::Vec2f d{e.q.x - e.p.x, e.q.y - e.p.y};
                const float len = std::sqrt(d.x * d.x + d.y * d.y);
                const math::Vec2f ln{-d.y / len, d.x / len};
                auto surf = std::make_shared<PlaneSurface>(
                    math::Vec3f{e.p.x, e.p.y, t.z_bot},
                    math::Vec3f{ln.x, ln.y, 0.0f});
                const auto dom = [&](const math::Vec2f& xy, float z) {
                    return subtract_detail::plane_domain(
                        *surf, math::Vec3f{xy.x, xy.y, z});
                };
                CoEdge bot{eid(e_base + i), false};
                bot.set_p_curve(line_rr(dom(e.p, t.z_bot), dom(e.q, t.z_bot)));
                CoEdge up{eid(e_base + 2 * n + i), false};
                up.set_p_curve(line_rr(dom(e.q, t.z_bot), dom(e.q, t.z_top)));
                CoEdge top{eid(e_base + n + i), true};
                top.set_p_curve(line_rr(dom(e.q, t.z_top), dom(e.p, t.z_top)));
                CoEdge down{eid(e_base + 2 * n + prev), true};
                down.set_p_curve(
                    line_rr(dom(e.p, t.z_top), dom(e.p, t.z_bot)));
                loop.add_coedge(bot);
                loop.add_coedge(up);
                loop.add_coedge(top);
                loop.add_coedge(down);
                add_wall_face(std::move(surf), std::move(loop));
            } else {
                // The arc patch: CylinderZSurface on the CAVITY side
                // (u_sign = -1 — the frame rule), the window
                // [-psi_end, -psi_start] x [0, h].
                auto surf = std::make_shared<CylinderZSurface>(
                    e.center, e.radius, t.z_bot, -1);
                const float u_p = -e.psi_start;
                const float u_q = -(e.psi_start + e.sweep);
                CoEdge bot{eid(e_base + i), false};
                bot.set_p_curve(line_rr({u_p, 0.0f}, {u_q, 0.0f}));
                CoEdge up{eid(e_base + 2 * n + i), false};
                up.set_p_curve(line_rr({u_q, 0.0f}, {u_q, h}));
                CoEdge top{eid(e_base + n + i), true};
                top.set_p_curve(line_rr({u_q, h}, {u_p, h}));
                CoEdge down{eid(e_base + 2 * n + prev), true};
                down.set_p_curve(line_rr({u_p, h}, {u_p, 0.0f}));
                loop.add_coedge(bot);
                loop.add_coedge(up);
                loop.add_coedge(top);
                loop.add_coedge(down);
                add_wall_face(std::move(surf), std::move(loop));
            }
        }
    } else {
        // THE SEAM-WIRED WALL (the ADR-0023 pattern, the patch between
        // the circles): the window [u0 - 2pi, u0] x [0, h] on the
        // REVERSED u, walked [seam down at u0 (REV), bottom west
        // (entity 1 then entity 0, both FWD), seam up at u0 - 2pi
        // (FWD), top east (entity 0 then entity 1, both REV)].
        const float psi0 = std::atan2(J[0].y - w.circle_center.y,
                                      J[0].x - w.circle_center.x);
        const float u0 = -psi0;
        auto surf = std::make_shared<CylinderZSurface>(
            w.circle_center, w.circle_radius, t.z_bot, -1);
        Loop loop;
        CoEdge s_down{eid(e_base + 2 * n), true};
        s_down.set_p_curve(line_rr({u0, h}, {u0, 0.0f}));
        CoEdge bot1{eid(e_base + 1), false};
        bot1.set_p_curve(line_rr({u0, 0.0f}, {u0 - subtract_detail::kPi, 0.0f}));
        CoEdge bot0{eid(e_base + 0), false};
        bot0.set_p_curve(
            line_rr({u0 - subtract_detail::kPi, 0.0f},
                    {u0 - subtract_detail::kTwoPi, 0.0f}));
        CoEdge s_up{eid(e_base + 2 * n), false};
        s_up.set_p_curve(
            line_rr({u0 - subtract_detail::kTwoPi, 0.0f},
                    {u0 - subtract_detail::kTwoPi, h}));
        CoEdge top0{eid(e_base + n + 0), true};
        top0.set_p_curve(
            line_rr({u0 - subtract_detail::kTwoPi, h},
                    {u0 - subtract_detail::kPi, h}));
        CoEdge top1{eid(e_base + n + 1), true};
        top1.set_p_curve(
            line_rr({u0 - subtract_detail::kPi, h}, {u0, h}));
        loop.add_coedge(s_down);
        loop.add_coedge(bot1);
        loop.add_coedge(bot0);
        loop.add_coedge(s_up);
        loop.add_coedge(top0);
        loop.add_coedge(top1);
        add_wall_face(std::move(surf), std::move(loop));
    }

    // ---- the pierced faces' INNER LOOPS ------------------------------
    // The chain itself, entity for entity, the region bits FLIPPED
    // (the material is OUTSIDE the profile now: interior_left ->
    // false, the fillet arcs' hole -> true). The +Z face walks the
    // canonical cycle; the -Z face the reversed one (their own
    // frames' parity — the caps' own walks, unit 3's precedent).
    {
        const auto& top_f = A.solid.shell().faces()[t.top_face];
        const auto& bot_f = A.solid.shell().faces()[t.bottom_face];
        const auto* top_plane =
            dynamic_cast<const PlaneSurface*>(top_f.surface());
        const auto* bot_plane =
            dynamic_cast<const PlaneSurface*>(bot_f.surface());
        const auto dom_top = [&](const math::Vec2f& xy) {
            return subtract_detail::plane_domain(
                *top_plane, math::Vec3f{xy.x, xy.y, t.z_top});
        };
        const auto dom_bot = [&](const math::Vec2f& xy) {
            return subtract_detail::plane_domain(
                *bot_plane, math::Vec3f{xy.x, xy.y, t.z_bot});
        };

        // The TOP inner loop: [entity 0 FWD, ..., entity n-1 FWD].
        Loop top_inner;
        top_inner.set_outer(false);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& e = w.entities[i];
            CoEdge ce{eid(e_base + n + i), false};
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                ce.set_p_curve(sdf::sketch_segment(
                    dom_top(e.p), dom_top(e.q), /*interior_left=*/false));
            } else {
                const math::Vec2f c = dom_top(e.center);
                const math::Vec2f s = dom_top(e.p);
                ce.set_p_curve(sdf::sketch_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x), e.sweep));
            }
            top_inner.add_coedge(ce);
        }

        // The BOTTOM inner loop: [entity 0 REV, entity n-1 REV, ...,
        // entity 1 REV] — the clockwise cycle starting at J_0.
        Loop bot_inner;
        bot_inner.set_outer(false);
        const auto add_bottom = [&](std::size_t i) {
            const auto& e = w.entities[i];
            CoEdge ce{eid(e_base + i), true};
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                ce.set_p_curve(sdf::sketch_segment(
                    dom_bot(e.q), dom_bot(e.p), /*interior_left=*/false));
            } else {
                const math::Vec2f c = dom_bot(e.center);
                const math::Vec2f s = dom_bot(e.q);  // the REVERSED start
                ce.set_p_curve(sdf::sketch_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x), e.sweep));
            }
            bot_inner.add_coedge(ce);
        };
        add_bottom(0);
        for (std::size_t k = n - 1; k >= 1; --k) add_bottom(k);

        shell.faces_mut()[t.top_face].add_loop(std::move(top_inner));
        shell.faces_mut()[t.bottom_face].add_loop(std::move(bot_inner));
    }

    // ---- the postconditions (hard, with the H term) ------------------
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - a loop does not "
            "chain (an assembly defect in the through-hole builder; "
            "report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - the result is not a "
            "closed orientable manifold (the radial-edge XOR or the "
            "shell closure broke; report this as a kernel bug)");
    for (const auto& e : R->edges) {
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            throw std::runtime_error(
                "exact_subtract: postcondition failed - edge " +
                std::to_string(e.id()) +
                " is not manifold-with-XOR (an assembly defect; report "
                "this as a kernel bug)");
    }
    if (euler_characteristic(*R) != 0)
        throw std::runtime_error(
            "exact_subtract: postcondition failed - chi = " +
            std::to_string(euler_characteristic(*R)) +
            " (the through-hole's pinned value is 0, genus 1; report "
            "this as a kernel bug)");
    return R;
}

} // namespace CAD_0::brep
