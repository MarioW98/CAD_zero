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
#include "CAD_0/brep/naming.hpp"
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
inline constexpr double kSubtractTolerance = 1e-6;

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
    math::Vec3d circle_center{};
    double circle_radius{0.0};
    double cylinder_v{0.0};

    // LinePair / TangentLine: a point on the first line (on the
    // cylinder, at the cylinder's z_base), the shared direction (the
    // axis), the two world azimuths (TangentLine: phi_a == phi_b),
    // and the signed axis-to-plane distance.
    math::Vec3d line_point{};
    math::Vec3d line_dir{};
    double phi_a{0.0};
    double phi_b{0.0};
    double axis_distance{0.0};

    // Oblique: the angle between the plane's normal and the axis
    // (radians) — the refusal's own number.
    double angle{0.0};
};

// ssi_plane_cylinder — the entry point. `angle_tol` is the cosine
// band that separates ⊥ / ∥ / oblique (the classification
// tolerance); `linear_tol` separates the line pair / tangent / empty
// cases (|d| vs r). Both come from the operation's declared Layer-3
// value. Pure function, deterministic (ADR-0014).
inline PlaneCylinderSSI ssi_plane_cylinder(const PlaneSurface& plane,
                                           const CylinderZSurface& cyl,
                                           double angle_tol, double linear_tol);

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
void translate_body(BRepBody& body, math::Vec3d delta);

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
//
// THE NAMING CONTEXT (ADR-0036 D1, H2-u6 morso 4): the names are
// stamped in the pass that builds, beside the walk indices the
// construction already holds — the address plane untouched, every
// existing golden and query keeps working unchanged (D1's split is
// additive by construction, D5's declaration). The context carries
// the two facts the bodies alone do not:
//
//   * `feature` — the subtract's OWN DAG id: the born names' creator
//     part (`sub#4`). Defaulted like the factories' own parameter
//     (make_box's, make_prism's): the standalone callers and the
//     scene's hybrid materialization read sub#0 — the token is the
//     pin there, the id is the caller's to supply (the DAG identity
//     flows with the resolver's morso, D6 rung 4).
//   * `target` / `tool` — the OPERANDS' naming declarations: the
//     creating operations' role tables, the one thing the bodies'
//     own stamps do not record. The operands' FEATURE IDS and ROLE
//     PATHS flow from their OWN stamped ShapeIds — the wall's, the
//     pierced faces' — read at the stamping sites, so the relational
//     names embed the operands' own structure verbatim and nothing
//     can drift between the two (the same discipline the morsi 2-3
//     stamps practice).
//
// The DEFAULT is the UNNAMED MODE: a null table (or an operand whose
// own stamps are unpopulated) leaves the born edges honestly EMPTY
// (the D4 queue's discipline, never a guess) — the wall's
// inheritance and the kept/pierced names flow regardless: they are
// stamp COPIES of the operands' own, no table involved.
struct SubtractNaming {
    geometry::FeatureId feature{};
    const naming::OperationNaming* target{};
    const naming::OperationNaming* tool{};
};

std::unique_ptr<BRepBody> exact_subtract(const BRepBody& A, const BRepBody& B,
                                         math::Vec3d tool_offset,
                                         double tol,
                                         const SubtractNaming& naming =
                                             SubtractNaming{});

// ===========================================================================
// Implementation
// ===========================================================================

namespace subtract_detail {

inline constexpr double kPi = 3.14159265358979;
inline constexpr double kTwoPi = 6.28318530717958647692;

// The domain coordinates of a world point in a plane's (u,v) frame
// (the factories' own projection — one inverse, no second frame).
// DOUBLE END TO END (morso 3, ADR-0035 D4): the projection computes
// in the exact world and the payload IS the exact world's — the
// demotion that quantized it to the sketch domain's float is
// retired with the float payload itself.
inline math::Vec2d plane_domain(const PlaneSurface& s, math::Vec3d p) {
    const math::Vec3d d = p - s.origin();
    return {d.dot(s.u_axis()), d.dot(s.v_axis())};
}

// Normalize an angle to [0, 2π).
inline double norm_two_pi(double a) {
    double r = std::fmod(a, kTwoPi);
    if (r < 0.0) r += kTwoPi;
    return r;
}

// THE PAYLOAD DOMAIN (morso 3, ADR-0035 D4): the p-curve payloads
// are the BREP parametric domain's own DOUBLE vocabulary
// (pcurve.hpp — one opinion per world); the exact world and the
// payload world are the SAME world now, so the v2f quantizer that
// lived here is gone — there is nothing to convert. The sketch
// world is never entered from this file.

} // namespace subtract_detail

// ---------------------------------------------------------------------------
// ssi_plane_cylinder
// ---------------------------------------------------------------------------
inline PlaneCylinderSSI ssi_plane_cylinder(const PlaneSurface& plane,
                                           const CylinderZSurface& cyl,
                                           double angle_tol, double linear_tol) {
    PlaneCylinderSSI out;
    const math::Vec3d n = plane.normal_vec().normalized();
    const math::Vec3d axis{0.0, 0.0, 1.0};
    const double c = std::abs(n.dot(axis));  // |cos| of the normal-axis angle

    if (c > angle_tol && c < 1.0 - angle_tol) {
        // OBLIQUE: the intersection is an ellipse; on the cylinder's
        // (θ, z) domain the p-curve is the SINUSOID z(θ) — not
        // typifiable in the G1 vocabulary (D5's honest boundary; the
        // G3 spline table owns it). Refused with the angle.
        out.kind = PlaneCylinderSSI::Kind::Oblique;
        out.angle = std::acos(std::min(1.0, c));
        return out;
    }

    const math::Vec3d p0 = plane.origin();
    if (c <= angle_tol) {
        // Plane ∥ axis: the two vertical lines where the cylinder
        // meets the plane. The plane's horizontal normal direction:
        // mu = the azimuth of (n.x, n.y) (n is unit; |n.z| ≈ 0 here).
        const double mu = std::atan2(n.y, n.x);
        // The signed offset of the plane from the axis, along the
        // plane's horizontal normal: D = (p0 - c)·n_xy. The cylinder
        // meets the plane where cos(phi - mu) = D/r (the classical
        // pair / tangent / empty trichotomy).
        const double D = (p0.x - cyl.axis_center().x) * n.x +
                        (p0.y - cyl.axis_center().y) * n.y;
        const double d = D;
        const double r = cyl.radius();
        out.axis_distance = d;
        if (std::abs(d) > r + linear_tol) {
            out.kind = PlaneCylinderSSI::Kind::Empty;
            return out;
        }
        if (std::abs(std::abs(d) - r) <= linear_tol) {
            // Tangent: the double line at azimuth mu (for d ≈ +r the
            // touch is on the +mu side; for d ≈ -r on the -mu side).
            out.kind = PlaneCylinderSSI::Kind::TangentLine;
            const double phi = d >= 0.0 ? mu : mu + subtract_detail::kPi;
            out.phi_a = out.phi_b = phi;
            out.line_dir = axis;
            out.line_point = {cyl.axis_center().x + r * std::cos(phi),
                              cyl.axis_center().y + r * std::sin(phi),
                              cyl.z_base()};
            return out;
        }
        // The pair: phi = mu ± arccos(d / r).
        out.kind = PlaneCylinderSSI::Kind::LinePair;
        const double theta0 = std::acos(std::max(-1.0, std::min(1.0, d / r)));
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
// The translation table's payload reader (H1, ADR-0034 D3): the
// visitor — no string compare, no dynamic_cast chain.
namespace subtract_detail {
struct TranslateReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    const CylinderSurface* cyl_y{nullptr};
    const CylinderZSurface* cyl_z{nullptr};
    const SphereSurface* sphere{nullptr};
    const ConeSurface* cone{nullptr};
    const TorusSurface* torus{nullptr};
    const EllipsoidSurface* ellipsoid{nullptr};
    void visit(const PlaneSurface& s) override { plane = &s; }
    void visit(const CylinderSurface& s) override { cyl_y = &s; }
    void visit(const CylinderZSurface& s) override { cyl_z = &s; }
    void visit(const SphereSurface& s) override { sphere = &s; }
    void visit(const ConeSurface& s) override { cone = &s; }
    void visit(const TorusSurface& s) override { torus = &s; }
    void visit(const EllipsoidSurface& s) override { ellipsoid = &s; }
};
} // namespace subtract_detail

inline void translate_body(BRepBody& body, math::Vec3d delta) {
    for (auto& v : body.vertices)
        v.set_position(v.position() + delta);
    for (auto& f : body.solid.shell_mut().faces_mut()) {
        const auto* surf = f.surface();
        if (surf == nullptr) {
            continue;  // (a faceless face cannot move; the class
                       // checks refuse such bodies long before here)
        }
        subtract_detail::TranslateReader r;
        surf->accept(r);
        switch (surf->family()) {
        case SurfaceFamily::Plane:
            f.set_surface(std::make_shared<PlaneSurface>(
                r.plane->origin() + delta, r.plane->normal_vec()));
            break;
        case SurfaceFamily::Cylinder:
            // The extrude's own Z-axis cylinder OR the primitive
            // factory's Y-axis one — both carry their placement in
            // the surface; both re-anchor the same way through their
            // own accessors.
            if (r.cyl_z != nullptr) {
                f.set_surface(std::make_shared<CylinderZSurface>(
                    r.cyl_z->axis_center() +
                        math::Vec2d{delta.x, delta.y},
                    r.cyl_z->radius(), r.cyl_z->z_base() + delta.z,
                    r.cyl_z->u_sign() > 0.0 ? +1 : -1));
            } else if (r.cyl_y != nullptr) {
                f.set_surface(std::make_shared<CylinderSurface>(
                    r.cyl_y->radius(), r.cyl_y->height(),
                    r.cyl_y->origin() + delta));
            } else {
                throw std::invalid_argument(
                    "translate_body: unknown 'cylinder' surface subtype "
                    "- the translation table covers CylinderSurface and "
                    "CylinderZSurface (a third subtype would be a kernel "
                    "extension, not a silent skip). Remedy: extend the "
                    "table with its own placement accessor.");
            }
            break;
        case SurfaceFamily::Sphere:
        {
            // G2 unit 1: the sphere's frame is DERIVED state carried by
            // the surface (the re-frame rule) — the translation moves
            // the mathematical object (C, R) and keeps the frame, so a
            // re-framed face's p-curves stay valid verbatim (the same
            // mathematics, one code path — the header's own doctrine).
            f.set_surface(std::make_shared<SphereSurface>(
                r.sphere->radius(), r.sphere->origin() + delta,
                r.sphere->pole(), r.sphere->azimuth()));
            break;
        }
        case SurfaceFamily::Cone:
            f.set_surface(std::make_shared<ConeSurface>(
                r.cone->base_radius(), r.cone->height(),
                r.cone->origin() + delta));
            break;
        case SurfaceFamily::Torus:
            f.set_surface(std::make_shared<TorusSurface>(
                r.torus->major_radius(), r.torus->minor_radius(),
                r.torus->origin() + delta));
            break;
        case SurfaceFamily::Ellipsoid:
            // The probe's family (K1 m3, ADR-0042): the sphere's own
            // discipline — the translation moves the mathematical
            // object (C, a, c) and keeps the INTRINSIC frame (the pole
            // is the revolution axis, derived state like the sphere's
            // re-framed pole — the authored p-curves stay valid
            // verbatim).
            f.set_surface(std::make_shared<EllipsoidSurface>(
                r.ellipsoid->equatorial_semiaxis(),
                r.ellipsoid->polar_semiaxis(),
                r.ellipsoid->origin() + delta,
                r.ellipsoid->pole(), r.ellipsoid->azimuth()));
            break;
        default:
            throw std::invalid_argument(
                "translate_body: the surface family '" +
                std::string(family_name(surf->family())) +
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
    math::Vec2d p{}, q{};  // the traversal's world endpoints
    // Arc only: the circle + the CCW world span the walk traverses
    // (the canonical walk runs psi-up on every fillet arc — the +Z
    // cap's loop is CCW around the profile, and a fillet arc's
    // material lies toward its centre).
    math::Vec2d center{};
    double radius{0.0};
    double psi_start{0.0};
    double sweep{0.0};
};

// The tool as the pipeline consumes it: the canonical walk + the
// caps' z (already translated).
struct ToolWalk {
    std::vector<WalkEntity> entities;
    bool is_circle{false};  // the seam-wired special case (n = 2 arcs
                            // of one circle totalling 2π)
    math::Vec2d circle_center{};
    double circle_radius{0.0};
    double z_lo{0.0}, z_hi{0.0};

    // (morso 4) the +Z cap the walk was read from — the walk's own
    // source face, kept so the stamping sites can read the cap's
    // coedges in the WALK'S OWN ORDER without a second read (one
    // opinion, one pass: the same face read_tool_walk found).
    const Face* cap{nullptr};
};

// The +Z cap's surface, verified planar with a +Z normal.
inline const PlaneSurface* cap_surface_of(const Face& f) {
    const auto* surf = f.surface();
    if (surf == nullptr) return nullptr;
    if (surf->family() != SurfaceFamily::Plane) return nullptr;
    TranslateReader r;
    surf->accept(r);
    if (r.plane == nullptr) return nullptr;
    if (r.plane->normal_vec().z < 0.9) return nullptr;  // not +Z-ish
    return r.plane;
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
        if (surf->family() != SurfaceFamily::Plane) continue;
        TranslateReader r;
        surf->accept(r);
        if (r.plane == nullptr || r.plane->normal_vec().z > -0.9) continue;
        w.z_lo = r.plane->origin().z;
        break;
    }

    const auto& loop = cap_face->loops();
    if (loop.empty())
        throw std::invalid_argument(
            "exact_subtract: the tool's cap carries no loop - a "
            "malformed prism. Remedy: report the tool's construction.");

    const auto dom = [cap](const math::Vec2d& uv) {
        return cap->evaluate(uv.x, uv.y);
    };
    for (const auto& ce : loop[0].coedges()) {
        const auto* pc = ce.p_curve();
        if (pc == nullptr ||
            (pc->kind != PCurve::Kind::Segment &&
             pc->kind != PCurve::Kind::Arc)) {
            throw std::invalid_argument(
                "exact_subtract: the tool's cap carries a p-curve outside "
                "the prism vocabulary (a Segment or an Arc per co-edge, "
                "the exact extrude's own authoring). Remedy: report the "
                "tool's construction.");
        }
        WalkEntity e;
        if (pc->kind == PCurve::Kind::Segment) {
            e.kind = WalkEntity::Kind::Segment;
            const math::Vec3d a = dom(pc->segment.a);
            const math::Vec3d b = dom(pc->segment.b);
            e.p = {a.x, a.y};
            e.q = {b.x, b.y};
        } else {
            e.kind = WalkEntity::Kind::Arc;
            const math::Vec3d c = dom(pc->arc.center);
            e.center = {c.x, c.y};
            e.radius = pc->arc.radius;
            e.sweep = pc->arc.sweep;
            const math::Vec3d a = dom(pcurve_arc_start_point(pc->arc));
            const math::Vec3d b = dom(pcurve_arc_end_point(pc->arc));
            e.p = {a.x, a.y};
            e.q = {b.x, b.y};
            // The world span: the cap's domain is right-handed around
            // +Z (the frame rule), so the domain's CCW sweep IS the
            // world's — verify the endpoints agree (the guard).
            e.psi_start = std::atan2(e.p.y - e.center.y,
                                     e.p.x - e.center.x);
            const double psi_q = std::atan2(e.q.y - e.center.y,
                                           e.q.x - e.center.x);
            double span = norm_two_pi(psi_q - e.psi_start);
            if (std::abs(span - e.sweep) > 1e-4 &&
                std::abs(span - e.sweep + kTwoPi) > 1e-4) {
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
        const math::Vec2d first = w.entities.front().p;
        math::Vec2d cursor = w.entities.front().p;
        for (const auto& e : w.entities) {
            if (std::abs(e.p.x - cursor.x) > 1e-4 ||
                std::abs(e.p.y - cursor.y) > 1e-4)
                throw std::invalid_argument(
                    "exact_subtract: the tool's canonical walk does not "
                    "chain endpoint-to-endpoint - a malformed prism cap. "
                    "Remedy: report the tool's construction.");
            cursor = e.q;
        }
        if (std::abs(cursor.x - first.x) > 1e-4 ||
            std::abs(cursor.y - first.y) > 1e-4)
            throw std::invalid_argument(
                "exact_subtract: the tool's canonical walk does not "
                "close - a malformed prism cap. Remedy: report the "
                "tool's construction.");
    } else {
        throw std::invalid_argument(
            "exact_subtract: the tool's cap loop is empty - a "
            "malformed prism. Remedy: report the tool's construction.");
    }

    w.cap = cap_face;  // (morso 4) the walk's own source face

    // The circle detection: two arcs of one circle totalling 2π.
    w.is_circle = false;
    if (w.entities.size() == 2 &&
        w.entities[0].kind == WalkEntity::Kind::Arc &&
        w.entities[1].kind == WalkEntity::Kind::Arc) {
        const auto& e0 = w.entities[0];
        const auto& e1 = w.entities[1];
        if (std::abs(e0.center.x - e1.center.x) <= 1e-5 &&
            std::abs(e0.center.y - e1.center.y) <= 1e-5 &&
            std::abs(e0.radius - e1.radius) <= 1e-5 &&
            std::abs(e0.sweep + e1.sweep - kTwoPi) <= 1e-4) {
            w.is_circle = true;
            w.circle_center = e0.center;
            w.circle_radius = e0.radius;
        }
    }
    return w;
}

// tool_lateral_of (morso 4, ADR-0036 D1): the tool's LATERAL face for
// a walk entity — the non-cap face whose loops reference the cap's
// coedge's edge. THE GENEALOGICAL CORRESPONDENCE, an edge reference,
// never a geometric signature (ADR-0005's own correction, upheld):
// the absorbed wall of the result descends from the operand's own
// lateral face, and the tavola and the stamping both find it here.
// The caps are skipped by their own placement signature (|n·ẑ| ≈ 1,
// cap_surface_of's own rule); a cylinder is always a lateral. The
// first match in the shell's own order wins — the deterministic walk
// every derivation practices. Returns null for a walk edge no
// non-cap face references (a malformed tool the class checks missed:
// the callers treat it as unstamped, honestly).
inline const Face* tool_lateral_of(const BRepBody& body, EdgeId edge) {
    for (const auto& f : body.solid.shell().faces()) {
        const auto* surf = f.surface();
        if (surf == nullptr) continue;
        if (surf->family() == SurfaceFamily::Plane) {
            TranslateReader r;
            surf->accept(r);
            if (r.plane != nullptr &&
                std::abs(r.plane->normal_vec().z) > 0.9)
                continue;  // a ±Z cap, never a lateral
        }
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges())
                if (ce.edge() == edge) return &f;
    }
    return nullptr;
}

// The target as the pipeline consumes it (the class check's product).
struct TargetInfo {
    std::size_t top_face{0}, bottom_face{0};  // the pierced pair
    double z_top{0.0}, z_bot{0.0};
    // The footprint (the pierced faces' own polygon — the same region
    // for a convex all-vertical-sides body, read from the TOP face's
    // outer loop, verified CCW-in-world):
    std::vector<math::Vec2d> verts;   // in loop order
    std::vector<math::Vec2d> normals; // unit outward normal per edge
};

// The tool profile's support along a unit direction d (the coarse
// cut of the centroid-then-predicate discipline: the vertices carry
// the segments, the arcs add their extreme point when the direction's
// azimuth falls inside the span).
inline double profile_support(const ToolWalk& w, math::Vec2d d) {
    double m = -std::numeric_limits<double>::infinity();
    for (const auto& e : w.entities) {
        m = std::max(m, d.x * e.p.x + d.y * e.p.y);
        m = std::max(m, d.x * e.q.x + d.y * e.q.y);
        if (e.kind == WalkEntity::Kind::Arc) {
            const double phi = norm_two_pi(std::atan2(d.y, d.x));
            const double rel = norm_two_pi(phi - e.psi_start);
            if (rel <= e.sweep)
                m = std::max(m, d.x * e.center.x + d.y * e.center.y +
                                      e.radius);
        }
    }
    return m;
}

// Is a point inside the tool's profile (the convex region: every
// entity's material side, with the tolerance band)?
inline bool point_in_profile(const ToolWalk& w, math::Vec2d pt, double tol) {
    for (const auto& e : w.entities) {
        if (e.kind == WalkEntity::Kind::Segment) {
            // The profile's material is on the LEFT of p->q: the
            // outward normal is the right normal.
            const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
            const double len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len <= 1e-9) continue;
            const math::Vec2d n{d.y / len, -d.x / len};
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

// ===========================================================================
// THE CROSSED CORNER (G2-u2 m3, ADR-0038 D1.2/D2/D3) — the gate and
// the assembly. The transverse crossing beyond the band AT the
// pierced face's covered corner, the split-free variant: the outer
// loops re-authored around the notch, the exit faces shrunk, the
// tool's surviving walls re-windowed, the tool's joint vertical
// re-timed (the absorbed joint(i) — the vocabulary row this unit
// pays), the dead corner column dropped. Everything else that crosses
// keeps the CROSSED refusal with the family named.
// ===========================================================================

// One containment-band trigger (hoisted from the placement tree so
// the gate can read the sorted set).
struct Trigger {
    math::Vec2d origin, normal;
    double margin;
};

// The gate's product: the admitted family's geometry, or the full
// actionable refusal.
struct CrossedCorner {
    bool admitted{false};
    std::string refusal{};
    std::size_t i0{0}, i1{0};  // the crossing face-edges (i1 = i0+1 mod k)
    math::Vec2d corner{};      // the covered corner (t.verts[i1])
    math::Vec2d exit_p{}, exit_q{};  // the exits on i0 / i1
    std::size_t ent_p{0}, ent_q{0};  // the exit entities
    std::size_t joint_idx{0};        // the inner joint's index (J_b)
    math::Vec2d joint{};             // the inner joint's position
};

// The material-side slack of pt against entity i (positive inside):
// the left-normal distance for a segment, r - |pt - c| for an arc.
inline double profile_slack(const ToolWalk& w, std::size_t i,
                            math::Vec2d pt) {
    const WalkEntity& e = w.entities[i];
    if (e.kind == WalkEntity::Kind::Segment) {
        const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
        const double len = std::sqrt(d.x * d.x + d.y * d.y);
        const math::Vec2d ln{-d.y / len, d.x / len};
        return (pt.x - e.p.x) * ln.x + (pt.y - e.p.y) * ln.y;
    }
    return e.radius -
           std::hypot(pt.x - e.center.x, pt.y - e.center.y);
}

// One candidate exit: an entity of the walk crossing the face-edge
// `edge` within both spans, strictly interior (the convexity plus the
// covered corner guarantee exactly one — more is a defect the gate
// refuses).
struct ExitHit {
    std::size_t entity{};
    bool arc{false};
    math::Vec2d point{};
};

inline std::vector<ExitHit> exit_hits(const ToolWalk& w,
                                      const TargetInfo& t,
                                      std::size_t edge, double tol) {
    std::vector<ExitHit> out;
    const math::Vec2d a = t.verts[edge];
    const math::Vec2d b =
        t.verts[(edge + 1) % t.verts.size()];
    const math::Vec2d f{b.x - a.x, b.y - a.y};
    const auto cross2 = [](math::Vec2d u, math::Vec2d v) {
        return u.x * v.y - u.y * v.x;
    };
    const auto strictly_interior = [&](math::Vec2d p, math::Vec2d s0,
                                       math::Vec2d s1) {
        return std::hypot(p.x - s0.x, p.y - s0.y) > tol &&
               std::hypot(p.x - s1.x, p.y - s1.y) > tol;
    };
    for (std::size_t i = 0; i < w.entities.size(); ++i) {
        const WalkEntity& e = w.entities[i];
        if (e.kind == WalkEntity::Kind::Segment) {
            const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
            const double det = cross2(d, f);
            if (std::abs(det) < 1e-12) continue;  // parallel
            const math::Vec2d am{a.x - e.p.x, a.y - e.p.y};
            const double s = cross2(am, f) / det;
            const double u = cross2(am, d) / det;
            if (s < -1e-12 || s > 1.0 + 1e-12) continue;
            if (u < -1e-12 || u > 1.0 + 1e-12) continue;
            const math::Vec2d p{e.p.x + s * d.x, e.p.y + s * d.y};
            if (!strictly_interior(p, e.p, e.q)) continue;
            if (!strictly_interior(p, a, b)) continue;
            out.push_back({i, false, p});
        } else {
            // The arc's circle against the edge's line: the two
            // roots, each checked against the arc's own CCW span.
            const double fl2 = f.x * f.x + f.y * f.y;
            const math::Vec2d m{a.x - e.center.x, a.y - e.center.y};
            const double bq = m.x * f.x + m.y * f.y;
            const double cq = m.x * m.x + m.y * m.y - e.radius * e.radius;
            const double disc = bq * bq - fl2 * cq;
            if (disc < 0.0) continue;
            for (const double root : {(-bq - std::sqrt(disc)) / fl2,
                                      (-bq + std::sqrt(disc)) / fl2}) {
                if (root < -1e-12 || root > 1.0 + 1e-12) continue;
                const math::Vec2d p{a.x + root * f.x,
                                    a.y + root * f.y};
                const double ang = norm_two_pi(
                    std::atan2(p.y - e.center.y, p.x - e.center.x) -
                    e.psi_start);
                if (ang > e.sweep + 1e-12) continue;
                if (!strictly_interior(p, a, b)) continue;
                out.push_back({i, true, p});
            }
        }
    }
    return out;
}

// THE GATE (D1.2's admitted family): exactly two transverse crossings
// at ADJACENT face-edges, the shared corner strictly covered, the
// through z-span, exactly one strictly-interior exit per edge, both
// exits on SEGMENT entities, the notch's inner chain the 2-entity
// corner (the reversed walk from the i0-exit: the top face's loop
// walks the notch with the material on the left, which is the
// footprint's canonical walk REVERSED — the inner chain is always
// the backward one, and the 2-entity form is q == p-1), the inner
// joint strictly inside the face, and the pierced face's crossing
// co-edges LINE-authored (the re-spanning class). Every other
// crossing keeps the CROSSED refusal, the family named.
inline CrossedCorner crossed_corner_gate(
    const BRepBody& A, const ToolWalk& w, const TargetInfo& t,
    const std::vector<Trigger>& crossing,
    const std::vector<Trigger>& tangential, std::size_t top_face,
    double z_lo, double z_hi, double tol) {
    CrossedCorner cc;
    const std::size_t k = t.verts.size();
    const std::size_t n = w.entities.size();

    // The refusal's family, named (the actionable discipline: the
    // same CROSSES phrase the pin :952 taught, the family and the
    // landed door added).
    const auto refuse = [&](const std::string& family) {
        cc.admitted = false;
        cc.refusal =
            "exact_subtract: the tool's profile CROSSES the boundary "
            "of the pierced face (it pokes beyond the edge from (" +
            std::to_string(crossing[0].origin.x) + ", " +
            std::to_string(crossing[0].origin.y) + ") outward along (" +
            std::to_string(crossing[0].normal.x) + ", " +
            std::to_string(crossing[0].normal.y) + ") by " +
            std::to_string(crossing[0].margin) + ") - " + family +
            ". The CORNER variant (the transverse crossing at the "
            "pierced face's covered corner, the split-free shape) is "
            "landed (ADR-0038 D1.2, G2-u2 m3); this placement is out "
            "of that family. Remedy: place the tool so it covers "
            "exactly one corner of the pierced face, its two adjacent "
            "segment edges exiting through the face's two adjacent "
            "edges (the corner notch), or keep the profile strictly "
            "inside the pierced face, or accept the faceted path.";
        return cc;
    };

    if (!tangential.empty())
        return refuse("the crossing composes with a TANGENT border "
                      "(the parete's family, the zero-thickness "
                      "contact - ADR-0038 D1.3's registered seat)");
    if (crossing.size() != 2)
        return refuse("the crossing is not at ONE corner (the "
                      "mid-edge or the multi-corner shape - the "
                      "split family, ADR-0038 D4's queue)");

    // The crossing edges' indices in the face's loop (the origins
    // are the face's own verts, read back).
    std::size_t ia = k, ib = k;
    for (std::size_t i = 0; i < k; ++i) {
        if (crossing[0].origin.x == t.verts[i].x &&
            crossing[0].origin.y == t.verts[i].y)
            ia = i;
        if (crossing[1].origin.x == t.verts[i].x &&
            crossing[1].origin.y == t.verts[i].y)
            ib = i;
    }
    if (ia >= k || ib >= k)
        return refuse("the crossing edges do not read back from the "
                      "pierced face's own loop (an internal "
                      "inconsistency - report this as a kernel bug)");
    // Order: i1 = i0 + 1 (the loop's own order, the shared corner
    // verts[i1] between them).
    if ((ia + 1) % k == ib) {
        cc.i0 = ia;
        cc.i1 = ib;
    } else if ((ib + 1) % k == ia) {
        cc.i0 = ib;
        cc.i1 = ia;
    } else {
        return refuse("the two crossed edges are not adjacent (the "
                      "footprint spans the face - the split family, "
                      "ADR-0038 D4's queue)");
    }
    cc.corner = t.verts[cc.i1];

    // The corner strictly covered (the footprint's every entity's
    // material side by more than the band).
    for (std::size_t i = 0; i < n; ++i)
        if (profile_slack(w, i, cc.corner) <= tol)
            return refuse("the pierced face's corner is not strictly "
                          "inside the tool's profile (the crossing "
                          "misses the corner - the bite or the split "
                          "family, ADR-0038 D4's queue)");

    // The through z-span (the blind span composes: the composite is
    // queued behind the chain cap insertion — the m4 door landed the
    // SEAM-WIRED CIRCLE family, and the composite's L-shaped cap is
    // the CHAIN form the walk table still queues).
    if (!(z_lo <= t.z_bot + tol && z_hi >= t.z_top - tol)) {
        cc.admitted = false;
        cc.refusal =
            "exact_subtract: the BLIND-CROSSED COMPOSITE - the tool "
            "crosses the pierced face's corner AND terminates inside "
            "the target (its span z = [" + std::to_string(z_lo) +
            ", " + std::to_string(z_hi) +
            "] does not cover the pierced pair z = [" +
            std::to_string(t.z_bot) + ", " + std::to_string(t.z_top) +
            "]) - the notch with the internal cap is the composite "
            "family (ADR-0038 D4's queue): its L-shaped cap needs the "
            "CHAIN cap insertion — the per-entity rim projection the "
            "walk table queues (the blind's decoupling landed for the "
            "SEAM-WIRED CIRCLE family, G2-u2 m4; this cap is the "
            "chain form). Remedy: extend the extrusion distance so "
            "the tool passes through, or accept the faceted path.";
        return cc;
    }

    // The exits: exactly one strictly-interior exit per crossed edge.
    const std::vector<ExitHit> hp = exit_hits(w, t, cc.i0, tol);
    const std::vector<ExitHit> hq = exit_hits(w, t, cc.i1, tol);
    if (hp.size() != 1 || hq.size() != 1)
        return refuse("the footprint's boundary does not cross the "
                      "two edges exactly once each within their "
                      "spans (the split or the degenerate family, "
                      "ADR-0038 D4's queue)");
    if (hp[0].arc || hq[0].arc)
        return refuse("the exit is on an ARC entity (the circular "
                      "crossed - the arc class shares the blind's "
                      "carried-cap family, ADR-0038 D4's queue with "
                      "the reason sharpened by the m2 amendment)");
    cc.exit_p = hp[0].point;
    cc.ent_p = hp[0].entity;
    cc.exit_q = hq[0].point;
    cc.ent_q = hq[0].entity;

    // The notch's inner chain: the 2-entity corner — the reversed
    // walk from the i0-exit (the loop's own direction) reaches the
    // i1-exit's entity in one step: q == p - 1 (mod n).
    if ((cc.ent_p + n - cc.ent_q) % n != 1)
        return refuse("the notch's inner chain is not the "
                      "single-corner shape (the general crossed, "
                      "ADR-0038 D4's queue)");
    cc.joint_idx = cc.ent_q;
    cc.joint = w.entities[cc.ent_q].q;  // J_q = entity q's end joint

    // The inner joint strictly inside the face (every edge's
    // material side by more than the band).
    for (std::size_t i = 0; i < k; ++i) {
        const double d =
            (cc.joint.x - t.verts[i].x) * t.normals[i].x +
            (cc.joint.y - t.verts[i].y) * t.normals[i].y;
        if (d >= -tol)
            return refuse("the notch's inner corner is not strictly "
                          "inside the pierced face (the degenerate "
                          "band - ADR-0038 D4's queue)");
    }

    // The pierced face's crossing co-edges carry Segment p-curves
    // (the re-spanning class: the exit faces' twins of the same
    // edges are straight by the same fact — the planar side faces
    // project them as Line2).
    {
        const auto& loop =
            A.solid.shell().faces()[top_face].loops()[0];
        for (const std::size_t pos : {cc.i0, cc.i1}) {
            const auto* pc = loop.coedges()[pos].p_curve();
            if (pc == nullptr || pc->kind != PCurve::Kind::Segment)
                return refuse("the crossed face-edge is not "
                              "line-authored (the circular crossed, "
                              "ADR-0038 D4's queue)");
        }
    }

    cc.admitted = true;
    return cc;
}

// ---------------------------------------------------------------------------
// crossed_corner_assemble — the re-authoring (D2): the walk's own
// chaining discipline throughout. The kept co-edges COPY (the id
// remapped, the p-curve verbatim); the re-spanned and the new
// co-edges PROJECT their Line2 payloads (material-on-left, the
// factories' discipline); every flag derives from the walk's
// (start, end) against the edge's stored pair — the near endpoint's
// slot, the factory's own authoring.
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> crossed_corner_assemble(
    const BRepBody& A, const BRepBody& bp, const ToolWalk& w,
    const TargetInfo& t, const CrossedCorner& cc, double tol,
    const SubtractNaming& naming) {
    const std::size_t V_A = A.vertices.size();
    const std::size_t E_A = A.edges.size();
    const std::size_t F_A = A.face_count();
    const ProvenanceInfo prov = combined_provenance(A, bp);
    (void)tol;  // the gate consumed the band; the assembly is exact

    // ---- The structural lookups (edge references, never geometric
    // signatures — the house's own rule).
    const auto& top_face = A.solid.shell().faces()[t.top_face];
    const auto& bot_face = A.solid.shell().faces()[t.bottom_face];
    const EdgeId te_p = top_face.loops()[0].coedges()[cc.i0].edge();
    const EdgeId te_q = top_face.loops()[0].coedges()[cc.i1].edge();
    // The side faces: the OTHER users of the crossed top edges.
    const auto other_user = [&](EdgeId e, FaceId skip1,
                                FaceId skip2) -> std::size_t {
        for (std::size_t f = 0; f < A.face_count(); ++f) {
            if (f == skip1 || f == skip2) continue;
            for (const auto& loop : A.solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges())
                    if (ce.edge() == e) return f;
        }
        return F_A;
    };
    const std::size_t side_p =
        other_user(te_p, t.top_face, t.bottom_face);
    const std::size_t side_q =
        other_user(te_q, t.top_face, t.bottom_face);
    if (side_p >= F_A || side_q >= F_A || side_p == side_q)
        throw std::logic_error(
            "exact_subtract: the crossed assembly could not find the "
            "exit faces (the structural lookups failed) - report "
            "this as a kernel bug.");
    // The bottom twins: the edges shared by the bottom face and the
    // side faces.
    const auto bottom_twin = [&](std::size_t side) -> EdgeId {
        for (const auto& ce : bot_face.loops()[0].coedges())
            for (const auto& loop :
                 A.solid.shell().faces()[side].loops())
                for (const auto& ce2 : loop.coedges())
                    if (ce2.edge() == ce.edge()) return ce.edge();
        throw std::logic_error(
            "exact_subtract: the crossed assembly could not find the "
            "bottom twin of an exit face's edge - report this as a "
            "kernel bug.");
    };
    const EdgeId be_p = bottom_twin(side_p);
    const EdgeId be_q = bottom_twin(side_q);
    // The dead corner column: the edge whose both endpoints sit at
    // the corner's XY (the prism-class target's joint vertical).
    std::size_t dead_edge = E_A;
    std::size_t dead_v0 = V_A, dead_v1 = V_A;
    for (std::size_t e = 0; e < E_A; ++e) {
        const auto& ed = A.edges[e];
        const auto& p0 = A.vertices[ed.vertex(0)].position();
        const auto& p1 = A.vertices[ed.vertex(1)].position();
        if (std::abs(p0.x - p1.x) < 1e-12 &&
            std::abs(p0.y - p1.y) < 1e-12 &&
            std::abs(p0.x - cc.corner.x) < 1e-12 &&
            std::abs(p0.y - cc.corner.y) < 1e-12) {
            dead_edge = e;
            dead_v0 = ed.vertex(0);
            dead_v1 = ed.vertex(1);
            break;
        }
    }
    if (dead_edge >= E_A)
        throw std::invalid_argument(
            "exact_subtract: the CROSSED corner - the target carries "
            "no vertical edge at the covered corner (the family "
            "takes the prism-class target whose pierced pair shares "
            "the corner column; a tapered or cornerless target is "
            "out of the v1 class). Remedy: use a box or planar "
            "prism target, or accept the faceted path.");

    // ---- The maps: A's survivors compacted (the dead column
    // dropped), the born appended.
    std::vector<std::size_t> vmap(V_A, V_A);  // old -> new (V_A = dead)
    std::vector<std::size_t> emap(E_A, E_A);
    {
        std::size_t next = 0;
        for (std::size_t i = 0; i < V_A; ++i)
            if (i != dead_v0 && i != dead_v1) vmap[i] = next++;
        next = 0;
        for (std::size_t i = 0; i < E_A; ++i)
            if (i != dead_edge) emap[i] = next++;
    }
    const std::size_t v_base = V_A - 2;   // the survivors' count
    const std::size_t e_base = E_A - 1;   // + the joint vertical
    // The born vertices: 6/7 = exit_p at z_bot/z_top; 8/9 = the
    // joint; 10/11 = exit_q (the numbering table the tests pin).
    const std::size_t v_ex_p_lo = v_base + 0, v_ex_p_hi = v_base + 1;
    const std::size_t v_j_lo = v_base + 2, v_j_hi = v_base + 3;
    const std::size_t v_ex_q_lo = v_base + 4, v_ex_q_hi = v_base + 5;
    // The born edges: 12 = the rim on lat(p) at z_bot (J -> exit_p);
    // 13 = the rim on lat(q) at z_bot (exit_q -> J); 14 = the rim on
    // lat(p) at z_top (exit_p -> J); 15 = the rim on lat(q) at z_top
    // (J -> exit_q); 16/17 = the exit verticals at exit_p/exit_q
    // (bottom -> top, the seam's own convention).
    const std::size_t e_rim_p_lo = e_base + 1;
    const std::size_t e_rim_q_lo = e_base + 2;
    const std::size_t e_rim_p_hi = e_base + 3;
    const std::size_t e_rim_q_hi = e_base + 4;
    const std::size_t e_exit_p = e_base + 5;
    const std::size_t e_exit_q = e_base + 6;

    // The born exit vertex at a given quota, for a given exit's
    // entity side (the re-spanned top edges take the z_top pair, the
    // bottom twins the z_bot ones).
    const auto born_exit = [&](std::size_t ent, double z) {
        if (ent == cc.ent_p)
            return z > 0.5 * (t.z_bot + t.z_top) ? v_ex_p_hi
                                                 : v_ex_p_lo;
        return z > 0.5 * (t.z_bot + t.z_top) ? v_ex_q_hi : v_ex_q_lo;
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);

    // ---- A's surviving vertices (the copies carry the records; the
    // ids compact).
    for (std::size_t i = 0; i < V_A; ++i) {
        if (vmap[i] == V_A) continue;
        R->vertices.push_back(A.vertices[i]);
        R->vertices.back().set_id(
            static_cast<VertexId>(vmap[i]));
    }
    // ---- The born vertices (the exit pair, the joint pair, the
    // exit pair — the notch's own order).
    {
        const math::Vec3d ex_p_lo{cc.exit_p.x, cc.exit_p.y, t.z_bot};
        const math::Vec3d ex_p_hi{cc.exit_p.x, cc.exit_p.y, t.z_top};
        const math::Vec3d j_lo{cc.joint.x, cc.joint.y, t.z_bot};
        const math::Vec3d j_hi{cc.joint.x, cc.joint.y, t.z_top};
        const math::Vec3d ex_q_lo{cc.exit_q.x, cc.exit_q.y, t.z_bot};
        const math::Vec3d ex_q_hi{cc.exit_q.x, cc.exit_q.y, t.z_top};
        const std::pair<math::Vec3d, std::size_t> born[6] = {
            {ex_p_lo, v_ex_p_lo}, {ex_p_hi, v_ex_p_hi},
            {j_lo, v_j_lo},       {j_hi, v_j_hi},
            {ex_q_lo, v_ex_q_lo}, {ex_q_hi, v_ex_q_hi}};
        for (const auto& [p, id] : born) {
            Vertex v{p};
            v.set_id(static_cast<VertexId>(id));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- A's surviving edges, the endpoints remapped and the
    // corner endpoints of the four re-spanned edges substituted.
    const auto remap_vertex = [&](std::size_t old_v, std::size_t edge,
                                  double quota_hint) -> std::size_t {
        if (vmap[old_v] != V_A) return vmap[old_v];
        // The dead corner endpoint of a re-spanned edge: the born
        // exit at the edge's own quota.
        const auto& pos = A.vertices[old_v].position();
        const std::size_t ent =
            (edge == te_p || edge == be_p) ? cc.ent_p : cc.ent_q;
        (void)quota_hint;
        return born_exit(ent, pos.z);
    };
    for (std::size_t i = 0; i < E_A; ++i) {
        if (emap[i] == E_A) continue;
        R->edges.push_back(A.edges[i]);
        auto& ed = R->edges.back();
        ed.set_id(static_cast<EdgeId>(emap[i]));
        ed.set_vertex(0, static_cast<VertexId>(
                             remap_vertex(ed.vertex(0), i, 0)));
        ed.set_vertex(1, static_cast<VertexId>(
                             remap_vertex(ed.vertex(1), i, 0)));
    }
    // ---- The joint vertical (the absorbed tool's joint(i),
    // re-timed) and the born rims/exits.
    {
        const auto push = [&](std::size_t a, std::size_t b) {
            Edge e{static_cast<VertexId>(a),
                   static_cast<VertexId>(b)};
            e.set_id(static_cast<EdgeId>(R->edges.size()));
            e.set_provenance(prov);
            R->edges.push_back(e);
        };
        push(v_j_lo, v_j_hi);    // e_base + 0: the joint
        push(v_j_lo, v_ex_p_lo); // e_base + 1: rim_p@lo
        push(v_ex_q_lo, v_j_lo); // e_base + 2: rim_q@lo
        push(v_ex_p_hi, v_j_hi); // e_base + 3: rim_p@hi
        push(v_j_hi, v_ex_q_hi); // e_base + 4: rim_q@hi
        push(v_ex_p_lo, v_ex_p_hi);  // e_base + 5: exit_p
        push(v_ex_q_lo, v_ex_q_hi);  // e_base + 6: exit_q
    }

    // ---- The re-authoring helper: one face's outer loop rebuilt by
    // the walk's own chaining. The spec: per array position, either
    // COPY (the id remapped), RE-SPAN (the edge known, the corner
    // endpoint substituted), or a NEW edge (the rims / the exit
    // vertical / the joint, identified by their (start, end)
    // positions among the born set). The notch's rims insert after
    // the re-spanned co-edge that ENDS at a born exit; the dead
    // co-edge is replaced by the exit vertical of its side's exit.
    const auto pos3 = [&](std::size_t v) {
        return R->vertices[v].position();
    };
    // Which born edge runs between a and b (vertex ids, either
    // orientation), and whether the walk traverses it reversed —
    // the wall's CCW order may run either way.
    const auto born_edge_of = [&](std::size_t a,
                                  std::size_t b) {
        const std::pair<std::size_t, std::size_t> table[7] = {
            {v_j_lo, v_j_hi},                     // the joint
            {v_j_lo, v_ex_p_lo}, {v_ex_q_lo, v_j_lo},
            {v_ex_p_hi, v_j_hi}, {v_j_hi, v_ex_q_hi},
            {v_ex_p_lo, v_ex_p_hi}, {v_ex_q_lo, v_ex_q_hi}};
        for (std::size_t i = 0; i < 7; ++i) {
            if (table[i].first == a && table[i].second == b)
                return std::pair<std::size_t, bool>{e_base + i,
                                                    false};
            if (table[i].first == b && table[i].second == a)
                return std::pair<std::size_t, bool>{e_base + i,
                                                    true};
        }
        throw std::logic_error(
            "exact_subtract: the crossed assembly's wall walk lost "
            "the chaining (no born edge between the walk's "
            "endpoints) - report this as a kernel bug.");
    };
    const auto is_born_vertex = [&](std::size_t v) {
        return v >= v_base;
    };

    // Re-author one of A's faces (the top/bottom/side quartet).
    // `pierced` selects the notch-rims insertion (the top and bottom
    // faces carry the notch; the side faces carry the exit vertical).
    const auto reauthor_face = [&](std::size_t face_index,
                                   bool pierced) {
        const Face& src = A.solid.shell().faces()[face_index];
        const auto* surf = src.surface();
        TranslateReader rd;
        surf->accept(rd);
        const PlaneSurface* pl = rd.plane;
        if (pl == nullptr)
            throw std::logic_error(
                "exact_subtract: the crossed assembly met a non-planar "
                "face in a planar-class target - report this as a "
                "kernel bug.");
        const auto dom = [&](const math::Vec3d& p) {
            return plane_domain(*pl, p);
        };
        const auto& ces = src.loops()[0].coedges();
        const std::size_t kk = ces.size();
        Loop loop;
        // The walk's cursor: the first co-edge's start (the dead
        // corner substituted where the first co-edge is re-spanned).
        std::size_t cursor = 0;  // a vertex id in R's numbering
        {
            const auto& ce0 = ces[0];
            const std::size_t s0 =
                ce0.reversed()
                    ? static_cast<std::size_t>(A.edges[ce0.edge()]
                                                   .vertex(1))
                    : static_cast<std::size_t>(
                          A.edges[ce0.edge()].vertex(0));
            cursor = remap_vertex(s0, ce0.edge(), 0);
        }
        const auto emit = [&](std::size_t edge_id, std::size_t a,
                              std::size_t b, const PCurve* copy) {
            const bool fwd =
                R->edges[edge_id].vertex(0) ==
                        static_cast<VertexId>(a) &&
                R->edges[edge_id].vertex(1) ==
                    static_cast<VertexId>(b);
            const bool rev =
                R->edges[edge_id].vertex(0) ==
                        static_cast<VertexId>(b) &&
                R->edges[edge_id].vertex(1) ==
                    static_cast<VertexId>(a);
            if (!fwd && !rev)
                throw std::logic_error(
                    "exact_subtract: the crossed assembly's walk "
                    "lost the chaining (an edge that does not run "
                    "between the walk's endpoints) - report this as "
                    "a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(edge_id), rev};
            if (copy != nullptr) {
                ce.set_p_curve(*copy);
            } else {
                ce.set_p_curve(pcurve_segment(
                    dom(pos3(a)), dom(pos3(b)),
                    /*interior_left=*/true));
            }
            loop.add_coedge(ce);
            cursor = b;
        };
        for (std::size_t j = 0; j < kk; ++j) {
            const auto& ce = ces[j];
            const std::size_t old_edge = ce.edge();
            if (old_edge == dead_edge) {
                // The dead co-edge is replaced by the exit vertical
                // of this side's own exit (side_p's re-spanned edges
                // are te_p/be_p — the exit on the face-edge i0).
                const bool is_p_side = (face_index == side_p);
                const std::size_t lo =
                    is_p_side ? v_ex_p_lo : v_ex_q_lo;
                const std::size_t hi =
                    is_p_side ? v_ex_p_hi : v_ex_q_hi;
                if (cursor != lo && cursor != hi)
                    throw std::logic_error(
                        "exact_subtract: the crossed assembly's "
                        "walk reached the dead edge off the exit "
                        "vertical's ends - report this as a kernel "
                        "bug.");
                const std::size_t other = cursor == lo ? hi : lo;
                emit(is_p_side ? e_exit_p : e_exit_q, cursor, other,
                     nullptr);
                continue;
            }
            const bool respan =
                old_edge == te_p || old_edge == te_q ||
                old_edge == be_p || old_edge == be_q;
            // The step's (start, end): the factory's traversal with
            // the dead endpoint substituted (re-spanned) or verbatim
            // (kept).
            const std::size_t s_old = static_cast<std::size_t>(
                ce.reversed()
                    ? A.edges[old_edge].vertex(1)
                    : A.edges[old_edge].vertex(0));
            const std::size_t e_old = static_cast<std::size_t>(
                ce.reversed()
                    ? A.edges[old_edge].vertex(0)
                    : A.edges[old_edge].vertex(1));
            const std::size_t s_new =
                remap_vertex(s_old, old_edge, 0);
            const std::size_t e_new =
                remap_vertex(e_old, old_edge, 0);
            emit(emap[old_edge], s_new, e_new,
                 respan ? nullptr : ce.p_curve());
            // The notch's rims: after the re-spanned co-edge that
            // ends at a born exit (the pierced faces only — exactly
            // one such co-edge per pierced face: the top's i0 and
            // the bottom's twin that walks INTO the corner).
            if (pierced && is_born_vertex(e_new) &&
                (e_new == v_ex_p_lo || e_new == v_ex_p_hi ||
                 e_new == v_ex_q_lo || e_new == v_ex_q_hi)) {
                // From this exit through the joint to the other
                // exit, the walk's own direction; the emit derives
                // each flag from the stored pair (the stored rim
                // directions are the pierced faces' own traversals:
                // rim_p@lo J->exit_p, rim_q@lo exit_q->J, rim_p@hi
                // exit_p->J, rim_q@hi J->exit_q).
                const bool from_p =
                    e_new == v_ex_p_lo || e_new == v_ex_p_hi;
                const bool lo_quota =
                    e_new == v_ex_p_lo || e_new == v_ex_q_lo;
                const std::size_t jv = lo_quota ? v_j_lo : v_j_hi;
                const std::size_t other =
                    lo_quota ? (from_p ? v_ex_q_lo : v_ex_p_lo)
                             : (from_p ? v_ex_q_hi : v_ex_p_hi);
                const std::size_t rim_first =
                    from_p ? (lo_quota ? e_rim_p_lo : e_rim_p_hi)
                           : (lo_quota ? e_rim_q_lo : e_rim_q_hi);
                const std::size_t rim_second =
                    from_p ? (lo_quota ? e_rim_q_lo : e_rim_q_hi)
                           : (lo_quota ? e_rim_p_lo : e_rim_p_hi);
                emit(rim_first, cursor, jv, nullptr);
                emit(rim_second, jv, other, nullptr);
            }
        }
        Face face(std::make_shared<PlaneSurface>(*pl));
        // The factory's own mathematical plane, copied bit-identical
        // (the same origin/normal/frame — the copy constructor's own
        // guarantee); the loop is re-authored below.
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(face_index));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        // Replace A's copy at its own index (the faces keep their
        // ids — none dies).
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= face_index)
            faces_mut.resize(face_index + 1);
        faces_mut[face_index] = std::move(face);
    };

    // A's faces first (the copies for the kept, the re-authored for
    // the quartet), then the walls appended. The copies' co-edges
    // carry A's OWN edge ids — the renumbering (the dead column
    // compacted away) rewrites them: one rebuild pass over the
    // copied faces (surface, stamps and p-curves copied verbatim,
    // the edge references remapped; the quartet is replaced
    // wholesale by the re-authoring below).
    for (std::size_t f = 0; f < F_A; ++f)
        shell.add_face(A.solid.shell().faces()[f]);
    for (std::size_t f = 0; f < F_A; ++f) {
        // The quartet is re-authored wholesale below — its copies
        // (which reference the dead column's edge, by the family's
        // own shape) are replaced, never rebuilt.
        if (f == t.top_face || f == t.bottom_face ||
            f == side_p || f == side_q)
            continue;
        const Face& src = A.solid.shell().faces()[f];
        TranslateReader rd;
        src.surface()->accept(rd);
        Face face(std::make_shared<PlaneSurface>(*rd.plane));
        for (const auto& loop : src.loops()) {
            Loop nl;
            nl.set_outer(loop.is_outer());
            for (const auto& ce : loop.coedges()) {
                const std::size_t mapped =
                    emap[static_cast<std::size_t>(ce.edge())];
                if (mapped == E_A)
                    throw std::logic_error(
                        "exact_subtract: the crossed assembly met "
                        "the dead column's edge in a face outside "
                        "the re-authored quartet - report this as "
                        "a kernel bug.");
                CoEdge nc{static_cast<EdgeId>(mapped),
                          ce.reversed()};
                if (ce.p_curve() != nullptr)
                    nc.set_p_curve(*ce.p_curve());
                nl.add_coedge(nc);
            }
            face.add_loop(std::move(nl));
        }
        face.set_id(static_cast<FaceId>(f));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        shell.faces_mut()[f] = std::move(face);
    }
    reauthor_face(t.top_face, true);
    reauthor_face(t.bottom_face, true);
    reauthor_face(side_p, false);
    reauthor_face(side_q, false);

    // ---- The walls: the tool's surviving laterals, re-windowed to
    // the notch's rim span (the footprint's clip inside the target).
    const auto add_crossed_wall = [&](std::size_t ent,
                                      const Face* lateral) {
        const WalkEntity& e = w.entities[ent];
        const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
        const double len = std::sqrt(d.x * d.x + d.y * d.y);
        const math::Vec2d ln{-d.y / len, d.x / len};
        const bool is_p = ent == cc.ent_p;
        const math::Vec2d ex = is_p ? cc.exit_p : cc.exit_q;
        auto surf = std::make_shared<PlaneSurface>(
            math::Vec3d{cc.joint.x, cc.joint.y, t.z_bot},
            math::Vec3d{ln.x, ln.y, 0.0});
        const auto dom = [&](const math::Vec3d& p) {
            return plane_domain(*surf, p);
        };
        const math::Vec3d c_ex_lo{ex.x, ex.y, t.z_bot};
        const math::Vec3d c_ex_hi{ex.x, ex.y, t.z_top};
        const math::Vec3d c_j_lo{cc.joint.x, cc.joint.y, t.z_bot};
        const math::Vec3d c_j_hi{cc.joint.x, cc.joint.y, t.z_top};
        const math::Vec2d q1 = dom(c_ex_lo), q2 = dom(c_ex_hi);
        const math::Vec2d q3 = dom(c_j_hi), q4 = dom(c_j_lo);
        // The CCW order of the rectangle (the material-on-left
        // walk): the signed area picks the direction.
        const auto area = [](math::Vec2d p1, math::Vec2d p2,
                             math::Vec2d p3, math::Vec2d p4) {
            return (p1.x * p2.y - p2.x * p1.y) +
                   (p2.x * p3.y - p3.x * p2.y) +
                   (p3.x * p4.y - p4.x * p3.y) +
                   (p4.x * p1.y - p1.x * p4.y);
        };
        const bool fwd = area(q1, q2, q3, q4) > 0.0;
        const math::Vec2d order[4] = {fwd ? q1 : q4, fwd ? q2 : q3,
                                      fwd ? q3 : q2, fwd ? q4 : q1};
        const std::size_t v_of[4] = {
            is_p ? v_ex_p_lo : v_ex_q_lo,
            is_p ? v_ex_p_hi : v_ex_q_hi, v_j_hi, v_j_lo};
        const std::size_t v_order[4] = {
            fwd ? v_of[0] : v_of[3], fwd ? v_of[1] : v_of[2],
            fwd ? v_of[2] : v_of[1], fwd ? v_of[3] : v_of[0]};
        Loop loop;
        for (std::size_t s = 0; s < 4; ++s) {
            const std::size_t a = v_order[s];
            const std::size_t b = v_order[(s + 1) % 4];
            const auto [edge_id, rev] = born_edge_of(a, b);
            CoEdge ce{static_cast<EdgeId>(edge_id), rev};
            ce.set_p_curve(pcurve_segment(order[s], order[(s + 1) % 4],
                                          /*interior_left=*/true));
            loop.add_coedge(ce);
        }
        Face face(surf);
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        if (lateral != nullptr) {
            face.set_shape_id(lateral->shape_id());
            face.set_name(lateral->name());
        }
        shell.add_face(std::move(face));
    };
    // The tool's laterals for the two surviving entities (the
    // genealogical correspondence — the same lookup the stamping
    // consults).
    const auto lateral_of = [&](std::size_t i) -> const Face* {
        if (w.cap == nullptr) return nullptr;
        return tool_lateral_of(
            bp, w.cap->loops()[0].coedges()[i].edge());
    };
    // The surviving walls in the tool's canonical walk order (the
    // entity index ascending — the numbering table's own rule).
    const std::size_t wall_ents[2] = {std::min(cc.ent_p, cc.ent_q),
                                      std::max(cc.ent_p, cc.ent_q)};
    for (const std::size_t ent : wall_ents)
        add_crossed_wall(ent, lateral_of(ent));

    // ---- THE NAMES (D3): the joint vertical's birth name (the
    // extrude's own vocabulary — the first instance of the joint(i)
    // row, stamped from the tool's own feature id, no sub context
    // involved, exactly the wall's inheritance discipline); the born
    // rims and exits (the named mode: the sub's relational grammar,
    // the operands' own stamps the legs).
    {
        const Face* const wall_p = lateral_of(cc.ent_p);
        const Face* const wall_q = lateral_of(cc.ent_q);
        // THE JOINT: the tool's lateral's stamp carries the tool's
        // feature id; the path [kExtLatRole, i] declares the
        // extrude's vocabulary — the joint's name is the absorbed
        // entity's OWN, derived at the assembly site (the tool's
        // body does not stamp its verticals: v1's honest boundary;
        // the subtract authors the first instance).
        if (wall_p != nullptr &&
            wall_p->shape_id().subshape.depth == 2 &&
            wall_p->shape_id().subshape.path[0] ==
                naming::kExtLatRole) {
            const geometry::FeatureId tool_feature =
                wall_p->shape_id().feature;
            geometry::SubshapeRef ref{};
            ref = ref.push(naming::kExtJointRole)
                      .push(static_cast<std::uint32_t>(
                          cc.joint_idx));
            R->edges[e_base].set_shape_id(
                {tool_feature, ref, {}});
            R->edges[e_base].set_name(naming::render_name(
                tool_feature, ref, naming::kExtNaming.edges()));
        }
        // THE RIMS AND THE EXITS (the named mode): the operands'
        // legs read the operands' own stamps (the m4 machinery).
        const Face* const top_operand = &A.solid.shell().faces()[t.top_face];
        const Face* const bot_operand = &A.solid.shell().faces()[t.bottom_face];
        const Face* const side_p_operand =
            &A.solid.shell().faces()[side_p];
        const Face* const side_q_operand =
            &A.solid.shell().faces()[side_q];
        const bool stampable =
            naming.target != nullptr && naming.tool != nullptr &&
            wall_p != nullptr && wall_q != nullptr &&
            top_operand->shape_id().subshape.depth > 0 &&
            bot_operand->shape_id().subshape.depth > 0 &&
            side_p_operand->shape_id().subshape.depth > 0 &&
            side_q_operand->shape_id().subshape.depth > 0 &&
            wall_p->shape_id().subshape.depth > 0 &&
            wall_q->shape_id().subshape.depth > 0;
        if (stampable) {
            const auto operand_leg =
                [](geometry::SubshapeRef ref, const Face& operand) {
                    for (std::uint8_t k = 0;
                         k < operand.shape_id().subshape.depth; ++k)
                        ref = ref.push(
                            operand.shape_id().subshape.path[k]);
                    return ref;
                };
            struct RimSpec {
                std::size_t edge;
                std::uint32_t role;
                const Face* a_leg;
                const Face* b_leg;
            };
            const RimSpec rims[6] = {
                {e_rim_p_lo, naming::kSubIsectLoRole, bot_operand,
                 wall_p},
                {e_rim_q_lo, naming::kSubIsectLoRole, bot_operand,
                 wall_q},
                {e_rim_p_hi, naming::kSubIsectHiRole, top_operand,
                 wall_p},
                {e_rim_q_hi, naming::kSubIsectHiRole, top_operand,
                 wall_q},
                {e_exit_p, naming::kSubIsectRole, side_p_operand,
                 wall_p},
                {e_exit_q, naming::kSubIsectRole, side_q_operand,
                 wall_q},
            };
            for (const RimSpec& spec : rims) {
                const naming::OperandBinding sub_operands[2] = {
                    {spec.a_leg->shape_id().feature, naming.target},
                    {spec.b_leg->shape_id().feature, naming.tool},
                };
                const naming::OperationNaming sub_naming{
                    naming::FeatureKind::Subtract, {},
                    naming::kSubEdgeRoles, sub_operands};
                geometry::SubshapeRef ref =
                    geometry::SubshapeRef{}.push(spec.role);
                ref = operand_leg(ref, *spec.a_leg);
                ref = operand_leg(ref, *spec.b_leg);
                R->edges[spec.edge].set_shape_id(
                    {naming.feature, ref, {}});
                R->edges[spec.edge].set_name(
                    naming::render_name(naming.feature, ref,
                                        sub_naming.edges()));
            }
        }
    }

    // ---- The postconditions (hard, the family's own pins).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - a loop does not "
            "chain (an assembly defect in the crossed-corner "
            "builder; report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - the result is "
            "not a closed orientable manifold (the radial-edge XOR "
            "or the shell closure broke in the crossed-corner "
            "builder; report this as a kernel bug)");
    for (const auto& e : R->edges) {
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            throw std::runtime_error(
                "exact_subtract: postcondition failed - edge " +
                std::to_string(e.id()) +
                " is not manifold-with-XOR (an assembly defect; "
                "report this as a kernel bug)");
    }
    if (inner_loop_count(*R) != 0)
        throw std::runtime_error(
            "exact_subtract: postcondition failed - the crossed "
            "corner's pierced faces must be simply connected (H = 0; "
            "report this as a kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(A))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - chi = " +
            std::to_string(euler_characteristic(*R)) +
            " (the crossed corner's pinned value is the target's "
            "own chi - the dent family; report this as a kernel "
            "bug)");
    return R;
}

// ---------------------------------------------------------------------------
// blind_door_assemble — THE BLIND DOOR (ADR-0038 D1.1/D2 + the m4
// decoupling rule): the seam-wired circle blind, one cap strictly
// inside, the other side through. THE DECOUPLING THROUGHOUT: the
// carried cap keeps its NATIVE FRAME (the tool's own cap surface,
// copied bit-identical — the plane, the domain) and gains the
// INVERTED REGION BIT (the rim arcs authored hole=true, "the
// material now outside the profile" — the pierced faces' inner-loop
// precedent applied to a carried face); the WALK serves the topology
// — the wall is the through machinery's seam-wired window re-based
// to [z_floor, z_pierced] VERBATIM (u_sign = -1, region-on-right),
// the pierced face's inner loop is the G1 machinery unchanged, the
// cap's loop follows its OWN frame parity (the caps' own walks:
// cap(lo) the reversed cycle, cap(hi) the canonical — exactly the
// pierced faces' convention), and the RACCOLTO's XOR azimuth closes
// the wall's window and the cap's rim on the SAME seam pair with
// OPPOSITE directions. The rim is typed by the INTRA-TOOL SSI
// consultation (D2: the tool's cap × the tool's wall, the same body
// twice — the entry plane×cylinder ⊥ at the cap quota, the
// both-domain answer the m2 tests pinned as the family's guide).
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> blind_door_assemble(
    const BRepBody& A, const BRepBody& bp, const ToolWalk& w,
    const TargetInfo& t, bool cap_is_lo, double tol,
    const SubtractNaming& naming) {
    const ProvenanceInfo prov = combined_provenance(A, bp);
    (void)tol;  // the gate consumed the band; the assembly is exact

    // ---- The quotas: the floor (the carried cap's own) and the
    // pierced quota (the window's other end, the through side).
    // v = 0 is the window's LOW end: the floor for blind_lo, the
    // pierced face for blind_hi (the mirror read upside down — the
    // numbering table is the through case's own, re-based).
    const double z_floor = cap_is_lo ? w.z_lo : w.z_hi;
    const std::size_t pierced = cap_is_lo ? t.top_face : t.bottom_face;
    const double z_pierced = cap_is_lo ? t.z_top : t.z_bot;
    const double z_v0 = std::min(z_floor, z_pierced);
    const double z_v1 = std::max(z_floor, z_pierced);
    const double h = z_v1 - z_v0;

    // ---- THE INTRA-TOOL CONSULTATION (D2 — the insertion's guide):
    // the tool's near cap × the tool's wall, the same body twice.
    // The near cap: the plane whose normal points the carried cap's
    // own way (the -Z cap for cap_is_lo, the +Z cap otherwise); the
    // wall: the CylinderZSurface (the circle tool's single lateral).
    const PlaneSurface* cap_surf = nullptr;
    const Face* cap_face = nullptr;
    const CylinderZSurface* tool_wall = nullptr;
    for (const auto& f : bp.solid.shell().faces()) {
        const auto* surf = f.surface();
        if (surf == nullptr) continue;
        TranslateReader r;
        surf->accept(r);
        if (r.plane != nullptr) {
            const double nz = r.plane->normal_vec().z;
            if ((cap_is_lo && nz < -0.9) || (!cap_is_lo && nz > 0.9)) {
                cap_surf = r.plane;
                cap_face = &f;
            }
        } else if (r.cyl_z != nullptr) {
            tool_wall = r.cyl_z;
        }
    }
    if (cap_surf == nullptr || cap_face == nullptr || tool_wall == nullptr)
        throw std::logic_error(
            "exact_subtract: the blind door's intra-tool consultation "
            "could not find the carried cap x the wall of the same "
            "body (the tool's own faces) - report this as a kernel "
            "bug.");
    const PlaneCylinderSSI ans =
        ssi_plane_cylinder(*cap_surf, *tool_wall, tol, tol);
    if (ans.kind != PlaneCylinderSSI::Kind::Circle ||
        std::abs(ans.circle_center.x - w.circle_center.x) > tol ||
        std::abs(ans.circle_center.y - w.circle_center.y) > tol ||
        std::abs(ans.circle_center.z - z_floor) > tol ||
        std::abs(ans.circle_radius - w.circle_radius) > tol)
        throw std::logic_error(
            "exact_subtract: the blind door's intra-tool consultation "
            "disagrees with the walk (the cap x the wall must answer "
            "the circle at the cap quota, the wall's own radius) - "
            "report this as a kernel bug.");

    // The rim's parameters FROM the entry's both-domain answer: the
    // cap-side circle (the logical rim — its materialized form the
    // seam-wired arc pair below) and the wall-side quota (the Line2
    // v = const, the window's floor edge re-based by the walk).
    const auto cap_dom = [&](const math::Vec2d& xy) {
        return plane_domain(*cap_surf,
                            math::Vec3d{xy.x, xy.y, z_floor});
    };
    const math::Vec2d rim_center = cap_dom(w.circle_center);
    const double rim_radius = ans.circle_radius;

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);

    // ---- A verbatim (the ids preserved, the factory's own p-curves
    // and stamps intact).
    R->vertices = A.vertices;
    R->edges = A.edges;
    for (const auto& f : A.solid.shell().faces()) shell.add_face(f);

    const std::size_t n = w.entities.size();  // the circle: n = 2
    const std::size_t v_base = R->vertices.size();
    const std::size_t e_base = R->edges.size();
    const auto vid = [&](std::size_t i) { return static_cast<VertexId>(i); };
    const auto eid = [&](std::size_t i) { return static_cast<EdgeId>(i); };

    // The joints of the canonical walk: J_i = entity i's q.
    std::vector<math::Vec2d> J(n), P(n);
    for (std::size_t i = 0; i < n; ++i) {
        J[i] = w.entities[i].q;
        P[i] = w.entities[i].p;
    }

    // ---- The walk tables (the through case's circle layout, the
    // quotas re-based): 2i = J_i at z_v0, 2i+1 = J_i at z_v1;
    // e_base+i = entity i's curve at v=0 (J_(i-1) -> J_i), e_base+n+i
    // = its v=h copy, e_base+2n = the seam at J_0 (the +X meridian).
    for (std::size_t i = 0; i < n; ++i) {
        Vertex vb{math::Vec3d{J[i].x, J[i].y, z_v0}};
        vb.set_id(vid(v_base + 2 * i));
        vb.set_provenance(prov);
        R->vertices.push_back(vb);
        Vertex vt{math::Vec3d{J[i].x, J[i].y, z_v1}};
        vt.set_id(vid(v_base + 2 * i + 1));
        vt.set_provenance(prov);
        R->vertices.push_back(vt);
    }
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
    {
        // The seam: the vertical at J_0 (the seam meridian +X),
        // v = 0 -> v = h — the through case's own layout (2i/2i+1 =
        // J_i at the window's two ends).
        Edge ev{vid(v_base), vid(v_base + 1)};
        ev.set_id(eid(e_base + 2 * n));
        ev.set_provenance(prov);
        R->edges.push_back(ev);
    }

    // ---- THE WALL — the through machinery's seam-wired window,
    // re-based to [z_v0, z_v1] (the u_sign = -1 cavity frame, the
    // window [u0-2pi, u0] x [0, h], region-on-right on every Line2).
    const auto line_rr = [](math::Vec2d a, math::Vec2d b) {
        return pcurve_segment(a, b, /*interior_left=*/false);
    };
    {
        const double psi0 =
            std::atan2(J[0].y - w.circle_center.y,
                       J[0].x - w.circle_center.x);
        const double u0 = -psi0;
        auto surf = std::make_shared<CylinderZSurface>(
            w.circle_center, w.circle_radius, z_v0, -1);
        const double f0 = u0;
        const double fp = subtract_detail::kPi;
        const double ftp = subtract_detail::kTwoPi;
        const double fh = h;
        Loop loop;
        CoEdge s_down{eid(e_base + 2 * n), true};
        s_down.set_p_curve(line_rr({f0, fh}, {f0, 0.0}));
        CoEdge bot1{eid(e_base + 1), false};
        bot1.set_p_curve(line_rr({f0, 0.0}, {f0 - fp, 0.0}));
        CoEdge bot0{eid(e_base + 0), false};
        bot0.set_p_curve(
            line_rr({f0 - fp, 0.0}, {f0 - ftp, 0.0}));
        CoEdge s_up{eid(e_base + 2 * n), false};
        s_up.set_p_curve(
            line_rr({f0 - ftp, 0.0}, {f0 - ftp, fh}));
        CoEdge top0{eid(e_base + n + 0), true};
        top0.set_p_curve(
            line_rr({f0 - ftp, fh}, {f0 - fp, fh}));
        CoEdge top1{eid(e_base + n + 1), true};
        top1.set_p_curve(line_rr({f0 - fp, fh}, {f0, fh}));
        loop.add_coedge(s_down);
        loop.add_coedge(bot1);
        loop.add_coedge(bot0);
        loop.add_coedge(s_up);
        loop.add_coedge(top0);
        loop.add_coedge(top1);
        Face face(surf);
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        // The ABSORBED wall inherits the tool's lateral's stamp (the
        // wall is the extrude's face, not the subtract's).
        if (w.cap != nullptr) {
            const Face* lateral = subtract_detail::tool_lateral_of(
                bp, w.cap->loops()[0].coedges()[0].edge());
            if (lateral != nullptr) {
                face.set_shape_id(lateral->shape_id());
                face.set_name(lateral->name());
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE PIERCED FACE'S INNER LOOP — the G1 machinery
    // unchanged: the +Z face walks the canonical cycle, the -Z face
    // the reversed one (their own frames' parity), the bits flipped
    // (the material outside the profile: the arcs hole = true).
    {
        const auto& pf = A.solid.shell().faces()[pierced];
        TranslateReader pr;
        pf.surface()->accept(pr);
        const PlaneSurface* pl = pr.plane;
        const auto dom = [&](const math::Vec2d& xy) {
            return plane_domain(
                *pl, math::Vec3d{xy.x, xy.y, z_pierced});
        };
        Loop inner;
        inner.set_outer(false);
        if (cap_is_lo) {
            // The +Z pierced face: [entity 0 FWD, ..., entity n-1
            // FWD] on the v = h pair.
            for (std::size_t i = 0; i < n; ++i) {
                const auto& e = w.entities[i];
                CoEdge ce{eid(e_base + n + i), false};
                const math::Vec2d c = dom(e.center);
                const math::Vec2d s = dom(e.p);
                ce.set_p_curve(pcurve_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x), e.sweep));
                inner.add_coedge(ce);
            }
        } else {
            // The -Z pierced face: the reversed cycle [entity 0 REV,
            // entity n-1 REV, ..., entity 1 REV] on the v = 0 pair.
            const auto add = [&](std::size_t i) {
                const auto& e = w.entities[i];
                CoEdge ce{eid(e_base + i), true};
                const math::Vec2d c = dom(e.center);
                const math::Vec2d s = dom(e.q);  // the REVERSED start
                ce.set_p_curve(pcurve_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x), e.sweep));
                inner.add_coedge(ce);
            };
            add(0);
            for (std::size_t k = n - 1; k >= 1; --k) add(k);
        }
        shell.faces_mut()[pierced].add_loop(std::move(inner));
    }

    // ---- THE INTERNAL CAP — the m4 decoupling's own authoring: the
    // tool's cap face carried with its NATIVE FRAME (the surface
    // copied bit-identical — the same mathematical plane, the same
    // domain), the loop the cap's OWN walk (the caps' frame parity:
    // cap(lo) the reversed cycle, cap(hi) the canonical — the same
    // walks the extrude authored, re-based onto the born rim pair),
    // and the region bit INVERTED: the rim arcs carry hole = true —
    // "the material is now outside the profile" (D1.1), the walk's
    // own declaration that the material side flipped. The mass
    // machinery reads the bit (the outer loop's region side) and
    // the volume answers the golden the wall refused.
    {
        Loop loop;
        if (cap_is_lo) {
            // The -Z cap: the REVERSED cycle on the v = 0 pair —
            // [entity n-1 REV, ..., entity 0 REV] (the mirror
            // domain's only storable form: the CCW-only vocabulary
            // forces it, and the extrude's own cap(lo) walks it).
            for (std::size_t k = n; k-- > 0;) {
                const auto& e = w.entities[k];
                CoEdge ce{eid(e_base + k), true};
                const math::Vec2d s = cap_dom(J[k]);  // REV: start q
                ce.set_p_curve(pcurve_arc(
                    rim_center, rim_radius, /*hole=*/true,
                    std::atan2(s.y - rim_center.y,
                               s.x - rim_center.x),
                    e.sweep));
                loop.add_coedge(ce);
            }
        } else {
            // The +Z cap: the CANONICAL cycle on the v = h pair —
            // [entity 0 FWD, ..., entity n-1 FWD].
            for (std::size_t i = 0; i < n; ++i) {
                const auto& e = w.entities[i];
                CoEdge ce{eid(e_base + n + i), false};
                const math::Vec2d s = cap_dom(P[i]);  // FWD: start p
                ce.set_p_curve(pcurve_arc(
                    rim_center, rim_radius, /*hole=*/true,
                    std::atan2(s.y - rim_center.y,
                               s.x - rim_center.x),
                    e.sweep));
                loop.add_coedge(ce);
            }
        }
        Face face(std::make_shared<PlaneSurface>(*cap_surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        // The ABSORBED cap inherits the tool's own stamp (one face
        // in, one face out — the wall's own disposition).
        face.set_shape_id(cap_face->shape_id());
        face.set_name(cap_face->name());
        shell.add_face(std::move(face));
    }

    // ---- THE NAMES (D3): the floor rim — the grammar's FIRST
    // INTRA-BODY row (the carried cap x the wall of the same body,
    // the level absent: the cap's plane IS the level); the pierced
    // rim and the seam in the G1 forms; the stamping in the pass
    // that builds, the operands' own stamps the legs.
    {
        const Face* const wall_operand =
            (w.cap != nullptr)
                ? subtract_detail::tool_lateral_of(
                      bp, w.cap->loops()[0].coedges()[0].edge())
                : nullptr;
        const Face* const pierced_operand =
            &A.solid.shell().faces()[pierced];
        const bool stampable =
            naming.target != nullptr && naming.tool != nullptr &&
            wall_operand != nullptr &&
            wall_operand->shape_id().subshape.depth > 0 &&
            pierced_operand->shape_id().subshape.depth > 0 &&
            cap_face->shape_id().subshape.depth > 0;
        if (stampable) {
            const auto operand_leg =
                [](geometry::SubshapeRef ref, const Face& operand) {
                    for (std::uint8_t k = 0;
                         k < operand.shape_id().subshape.depth; ++k)
                        ref = ref.push(
                            operand.shape_id().subshape.path[k]);
                    return ref;
                };
            const auto stamp =
                [&](std::size_t index,
                    const geometry::SubshapeRef& ref,
                    const naming::OperandBinding* bindings,
                    std::size_t binding_count) {
                    const naming::OperationNaming sub_naming{
                        naming::FeatureKind::Subtract, {},
                        naming::kSubEdgeRoles,
                        {bindings, binding_count}};
                    R->edges[index].set_shape_id(
                        {naming.feature, ref, {}});
                    R->edges[index].set_name(naming::render_name(
                        naming.feature, ref, sub_naming.edges()));
                };

            // The pierced rim (the G1 form, levelled): the pierced
            // face's leg + the wall's, at the window's pierced end.
            {
                const naming::OperandBinding sub_operands[2] = {
                    {pierced_operand->shape_id().feature,
                     naming.target},
                    {wall_operand->shape_id().feature, naming.tool},
                };
                const std::uint32_t level =
                    cap_is_lo ? naming::kSubIsectHiRole
                              : naming::kSubIsectLoRole;
                const geometry::SubshapeRef rim =
                    operand_leg(operand_leg(
                                    geometry::SubshapeRef{}.push(level),
                                    *pierced_operand),
                                *wall_operand);
                const std::size_t lo0 = e_base + (cap_is_lo ? n : 0);
                stamp(lo0 + 0, rim, sub_operands, 2);
                stamp(lo0 + 1, rim, sub_operands, 2);
            }
            // The floor rim (the INTRA-BODY row, unlevelled): BOTH
            // legs the tool's own — the carried cap first, the wall
            // second (the declared slot order).
            {
                const naming::OperandBinding sub_operands[2] = {
                    {cap_face->shape_id().feature, naming.tool},
                    {wall_operand->shape_id().feature, naming.tool},
                };
                const geometry::SubshapeRef rim = operand_leg(
                    operand_leg(
                        geometry::SubshapeRef{}.push(
                            naming::kSubIsectIntraRole),
                        *cap_face),
                    *wall_operand);
                const std::size_t fl0 =
                    e_base + (cap_is_lo ? 0 : n);
                stamp(fl0 + 0, rim, sub_operands, 2);
                stamp(fl0 + 1, rim, sub_operands, 2);
            }
            // The seam (the G1 form): the wall's own.
            {
                const naming::OperandBinding sub_operands[2] = {
                    {pierced_operand->shape_id().feature,
                     naming.target},
                    {wall_operand->shape_id().feature, naming.tool},
                };
                const geometry::SubshapeRef seam = operand_leg(
                    geometry::SubshapeRef{}.push(naming::kSubSeamRole),
                    *wall_operand);
                stamp(e_base + 2 * n, seam, sub_operands, 2);
            }
        }
    }

    // ---- The postconditions (hard, the family's own pins).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - a loop does not "
            "chain (an assembly defect in the blind-door builder; "
            "report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - the result is not "
            "a closed orientable manifold (the radial-edge XOR or the "
            "shell closure broke in the blind-door builder; report "
            "this as a kernel bug)");
    for (const auto& e : R->edges) {
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            throw std::runtime_error(
                "exact_subtract: postcondition failed - edge " +
                std::to_string(e.id()) +
                " is not manifold-with-XOR (an assembly defect; "
                "report this as a kernel bug)");
    }
    if (inner_loop_count(*R) != 1)
        throw std::runtime_error(
            "exact_subtract: postcondition failed - the blind hole "
            "pierces exactly ONE face (H = 1; report this as a "
            "kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(A))
        throw std::runtime_error(
            "exact_subtract: postcondition failed - chi = " +
            std::to_string(euler_characteristic(*R)) +
            " (the blind hole's pinned value is the target's own "
            "chi - the dent family; report this as a kernel bug)");
    return R;
}

} // namespace subtract_detail

// ---------------------------------------------------------------------------
// exact_subtract
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> exact_subtract(const BRepBody& A,
                                                const BRepBody& B,
                                                math::Vec3d tool_offset,
                                                double tol,
                                                const SubtractNaming& naming) {
    // ==== THE CLASS CHECK (D7.1/D7.3) — pinned BEFORE the algorithm =
    //
    // A: convex, planar-faced, every face ⊥ or ∥ to the axis.
    {
        std::size_t pos_z = 0, neg_z = 0;
        for (const auto& f : A.solid.shell().faces()) {
            const auto* surf = f.surface();
            if (surf == nullptr || surf->family() != SurfaceFamily::Plane)
                throw std::invalid_argument(
                    "exact_subtract: the target carries a non-planar face "
                    "(family '" +
                    (surf ? std::string(family_name(surf->family()))
                          : std::string(family_name(SurfaceFamily::None))) +
                    "') - the G1 class takes convex PLANAR-faced targets (a "
                    "box, a planar prism); a curved target face meets the "
                    "tool in the general SSI table (G2's). Remedy: use a "
                    "planar-faced target, or accept the faceted path (the "
                    "SDF materialization stands).");
            subtract_detail::TranslateReader rr;
            surf->accept(rr);
            const double nz = rr.plane->normal_vec().z;
            if (std::abs(nz) > tol && std::abs(nz) < 1.0 - tol)
                throw std::invalid_argument(
                    "exact_subtract: the target has an OBLIQUE face (its "
                    "normal makes angle " +
                    std::to_string(std::acos(std::min(1.0, std::abs(nz)))) +
                    " rad with the tool's axis) - the oblique plane∩cylinder "
                    "SSI is an ellipse whose cylinder-domain p-curve is a "
                    "SINUSOID, not typifiable in the G1 vocabulary "
                    "(ADR-0032 D5: the G3 spline table). Remedy: reorient "
                    "the target's face (axis-aligned), or accept the "
                    "faceted path.");
            if (nz >= 1.0 - tol) ++pos_z;
            if (nz <= -(1.0 - tol)) ++neg_z;
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
        subtract_detail::TranslateReader pr;
        f.surface()->accept(pr);
        const math::Vec3d o = pr.plane->origin();
        const math::Vec3d n = pr.plane->normal_vec();
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
                (surf->family() != SurfaceFamily::Plane &&
                 surf->family() != SurfaceFamily::Cylinder))
                throw std::invalid_argument(
                    "exact_subtract: the tool is not a Z-axis prism of "
                    "the D6 class (a face of family '" +
                    (surf ? std::string(family_name(surf->family()))
                          : std::string(family_name(SurfaceFamily::None))) +
                    "') - the G1 exact subtract consumes the exact "
                    "extrude's prisms (a convex profile or a circle), "
                    "placed by the recipe's offset. Remedy: use an "
                    "extruded profile (or the fitted box) as the tool, "
                    "or accept the faceted path.");
            if (surf->family() == SurfaceFamily::Cylinder &&
                [&] { subtract_detail::TranslateReader tr;
                      surf->accept(tr); return tr.cyl_z == nullptr; }())
                throw std::invalid_argument(
                    "exact_subtract: the tool is not a Z-axis prism of "
                    "the D6 class (its cylinder's axis is Y - the "
                    "primitive factory's placement) - the G1 exact "
                    "subtract consumes the exact extrude's Z-axis "
                    "prisms. Remedy: model the tool as an extruded "
                    "circle profile, or accept the faceted path.");
            subtract_detail::TranslateReader tr;
            surf->accept(tr);
            const double nz =
                tr.plane != nullptr ? tr.plane->normal_vec().z
                                    : 0.0;  // cylinder walls are ∥ to the axis
            if (tr.plane != nullptr) {
                if (nz > 1.0 - tol) ++pos_z;
                else if (nz < -(1.0 - tol)) ++neg_z;
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
        if (f.surface()->family() != SurfaceFamily::Plane) continue;
        subtract_detail::TranslateReader pr;
        f.surface()->accept(pr);
        const math::Vec3d o = pr.plane->origin();
        const math::Vec3d n = pr.plane->normal_vec();
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
            subtract_detail::TranslateReader pr;
            f.surface()->accept(pr);
            const PlaneSurface* pl = pr.plane;
            const double nz = pl->normal_vec().z;
            if (nz >= 1.0 - tol) {
                t.top_face = i;
                t.z_top = pl->origin().z;
                have_top = true;
            } else if (nz <= -(1.0 - tol)) {
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
        subtract_detail::TranslateReader tr;
        top.surface()->accept(tr);
        const PlaneSurface* top_plane = tr.plane;
        double area2 = 0.0;
        for (const auto& ce : top.loops()[0].coedges()) {
            const VertexId vid = ce.start_vertex(A.edges);
            t.verts.push_back(
                {A.vertices[vid].position().x, A.vertices[vid].position().y});
        }
        for (std::size_t i = 0; i < t.verts.size(); ++i) {
            const math::Vec2d& a = t.verts[i];
            const math::Vec2d& b = t.verts[(i + 1) % t.verts.size()];
            area2 += static_cast<double>(a.x) * b.y -
                     static_cast<double>(b.x) * a.y;
            const math::Vec2d d{b.x - a.x, b.y - a.y};
            const double len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len <= 1e-9)
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
    const double z_lo = w.z_lo;  // the translated caps
    const double z_hi = w.z_hi;
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
        std::vector<math::Vec2d> axes = t.normals;
        for (const auto& e : w.entities) {
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
                const double len = std::sqrt(d.x * d.x + d.y * d.y);
                if (len > 1e-9)
                    axes.push_back({d.y / len, -d.x / len});
            } else {
                for (const auto& v : t.verts) {
                    const math::Vec2d d{v.x - e.center.x, v.y - e.center.y};
                    const double len = std::sqrt(d.x * d.x + d.y * d.y);
                    if (len > 1e-9)
                        axes.push_back({d.x / len, d.y / len});
                }
            }
        }
        double best_gap = -std::numeric_limits<double>::infinity();
        for (const auto& a : axes) {
            double fp_min = std::numeric_limits<double>::infinity();
            for (const auto& v : t.verts)
                fp_min = std::min(fp_min, a.x * v.x + a.y * v.y);
            const double gap =
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
        std::vector<subtract_detail::Trigger> crossing, tangential;
        for (std::size_t i = 0; i < t.verts.size(); ++i) {
            const math::Vec2d& n = t.normals[i];
            const double line = n.x * t.verts[i].x + n.y * t.verts[i].y;
            const double margin =
                subtract_detail::profile_support(w, n) - line;
            if (margin > tol)
                crossing.push_back({t.verts[i], n, margin});
            else if (margin > -tol)
                tangential.push_back({t.verts[i], n, margin});
        }
        const auto canonical_less =
            [](const subtract_detail::Trigger& a,
               const subtract_detail::Trigger& b) {
            if (a.origin.x != b.origin.x) return a.origin.x < b.origin.x;
            if (a.origin.y != b.origin.y) return a.origin.y < b.origin.y;
            if (a.normal.x != b.normal.x) return a.normal.x < b.normal.x;
            return a.normal.y < b.normal.y;
        };
        if (!crossing.empty()) {
            std::sort(crossing.begin(), crossing.end(), canonical_less);
            // ==== THE CROSSED CORNER (ADR-0038 D1.2, G2-u2 m3) ====
            // The transverse crossing beyond the band at the pierced
            // face's covered corner — the split-free variant — IS the
            // landed door; every other crossing family keeps the
            // CROSSED refusal, the family named (the message built by
            // the gate: the same CROSSES phrase the pin :952 taught).
            const subtract_detail::CrossedCorner cc =
                subtract_detail::crossed_corner_gate(
                    A, w, t, crossing, tangential, t.top_face,
                    z_lo, z_hi, tol);
            if (!cc.admitted)
                throw std::invalid_argument(cc.refusal);
            return subtract_detail::crossed_corner_assemble(
                A, *bp, w, t, cc, tol, naming);
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

    // (3) The z-bands: the blind door (one cap inside — the
    //     orientation decoupling landed, G2-u2 m4), the both-inside
    //     cavity, the through-hole.
    const bool blind_lo = z_lo > t.z_bot + tol && z_lo < t.z_top - tol;
    const bool blind_hi = z_hi > t.z_bot + tol && z_hi < t.z_top - tol;
    if (blind_lo && blind_hi)
        throw std::invalid_argument(
            "exact_subtract: the BLIND cavity - both the tool's caps "
            "lie strictly inside the target (z = [" +
            std::to_string(z_lo) + ", " + std::to_string(z_hi) +
            "] within the pierced pair z = [" + std::to_string(t.z_bot) +
            ", " + std::to_string(t.z_top) +
            "]) - the fully-internal cavity: the result would be a "
            "TWO-SHELL body (the outer shell and the cavity's own, "
            "chi = 2*2 = 4), and the single-shell derivation the "
            "report practices has no row for it (ADR-0038 D4's queue, "
            "the multi-shell honesty). Remedy: extend the extrusion "
            "distance so the tool passes through - or leaves exactly "
            "one cap inside (the blind hole family, landed G2-u2 m4) "
            "- or accept the faceted path.");
    if (blind_lo || blind_hi) {
        // ==== THE BLIND DOOR (ADR-0038 D1.1/D2 + the m4 rule) ====
        // Exactly one cap strictly inside, the other side through —
        // the SEAM-WIRED CIRCLE family the decoupling opened (the
        // carried cap keeps its native frame and gains the inverted
        // region bit; the walk serves the topology). The CHAIN
        // profile's blind is its own honest boundary: the per-entity
        // rim projection is the walk table's row away (ADR-0038 D4's
        // queue, no consumer today).
        if (!w.is_circle)
            throw std::invalid_argument(
                "exact_subtract: the BLIND hole - the tool terminates "
                "inside the target (its " +
                std::string(blind_lo ? "bottom" : "top") + " cap at z = " +
                std::to_string(blind_lo ? z_lo : z_hi) +
                " lies strictly between the pierced faces z = [" +
                std::to_string(t.z_bot) + ", " + std::to_string(t.z_top) +
                "]) and the tool is a CHAIN profile - the landed door is "
                "the SEAM-WIRED CIRCLE family (the SSI-guided cap "
                "insertion, ADR-0038 D1.1/D2 with the m4 decoupling "
                "rule); the chain's per-entity rim projection is the "
                "walk table's queued row (ADR-0038 D4's queue, no "
                "consumer today). Remedy: use a circular tool (the "
                "extruded circle profile), extend the extrusion "
                "distance so the tool passes through, or accept the "
                "faceted path.");
        return subtract_detail::blind_door_assemble(
            A, *bp, w, t, blind_lo, tol, naming);
    }
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
    const double h = t.z_top - t.z_bot;
    const ProvenanceInfo prov = combined_provenance(A, B);

    const auto vid = [&](std::size_t i) { return static_cast<VertexId>(i); };
    const auto eid = [&](std::size_t i) { return static_cast<EdgeId>(i); };

    // The joints of the canonical walk: J_i = entity i's q.
    std::vector<math::Vec2d> J(n), P(n);
    for (std::size_t i = 0; i < n; ++i) {
        J[i] = w.entities[i].q;
        P[i] = w.entities[i].p;  // == J[(i + n - 1) % n] (the guard proved it)
    }

    // ---- the new vertices: 2i = J_i at z_bot, 2i+1 = J_i at z_top
    for (std::size_t i = 0; i < n; ++i) {
        Vertex vb{math::Vec3d{J[i].x, J[i].y, t.z_bot}};
        vb.set_id(vid(v_base + 2 * i));
        vb.set_provenance(prov);
        R->vertices.push_back(vb);
        Vertex vt{math::Vec3d{J[i].x, J[i].y, t.z_top}};
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

    // ---- THE NAMES (ADR-0036 D1, H2-u6 morso 4) ----------------------
    // Stamped in the pass that builds, beside the walk indices the
    // construction already holds — zero extra passes, no state, the
    // morsi 2-3 lesson replicated at the first relational consumer.
    // The address plane is untouched by construction (B3's own bill:
    // the numbering above is exactly what it was; the names ride
    // BESIDE the ids).
    //
    //   * the ABSORBED WALL inherits the tool's own lateral's stamp
    //     (name + ShapeId, the copy of the operand's own): an entity
    //     that SURVIVES keeps its birth name — the wall is the
    //     extrude's face, not the subtract's (D1: identity is where a
    //     thing was born). No table, no render — nothing can drift.
    //     The kept and the pierced entities need no stamping here:
    //     the VERBATIM copy above carries their stamps (D1's rule for
    //     free, the m2 pin).
    //   * the BORN EDGES of the SEAM-WIRED case (the guide case's own
    //     class, the ADR D1 table's own class) carry the relational
    //     names: the rim circles — ONE logical name each, the
    //     seam-wired arc pair its MATERIALIZED form (ADR-0023: both
    //     halves answer to it), the bottom pair at @lo, the top pair
    //     at @hi — and the seam vertical. The legs flow from the
    //     OPERANDS' OWN STAMPS: the pierced faces' and the wall's
    //     feature ids and role paths, read here; the TABLES are the
    //     naming context's. The unnamed context, or an operand whose
    //     own stamps are unpopulated, leaves the born edges honestly
    //     EMPTY (the D4 queue's discipline, never a guess); the
    //     general chain's born edges read honestly empty too — the
    //     REGISTERED boundary (the morse-4 register in the ADR: the
    //     joints' verticals have no vocabulary row yet, and the
    //     segment walls' curves are Phase D's SSI entry; the extension
    //     is a table row away when a consumer arrives).
    const auto lateral_of = [&](std::size_t i) -> const Face* {
        if (w.cap == nullptr) return nullptr;
        return subtract_detail::tool_lateral_of(
            *bp, w.cap->loops()[0].coedges()[i].edge());
    };
    if (is_circle) {
        const Face* const wall_operand = lateral_of(0);
        const Face* const top_operand =
            &A.solid.shell().faces()[t.top_face];
        const Face* const bot_operand =
            &A.solid.shell().faces()[t.bottom_face];
        const bool stampable =
            naming.target != nullptr && naming.tool != nullptr &&
            wall_operand != nullptr &&
            wall_operand->shape_id().subshape.depth > 0 &&
            top_operand->shape_id().subshape.depth > 0 &&
            bot_operand->shape_id().subshape.depth > 0;
        if (stampable) {
            // The instance's operand bindings (the m2 test's own
            // shapes, now the OFFICIAL tables): the feature ids flow
            // from the operands' stamps, the tables from the context.
            const naming::OperandBinding sub_operands[2] = {
                {top_operand->shape_id().feature, naming.target},
                {wall_operand->shape_id().feature, naming.tool},
            };
            const naming::OperationNaming sub_naming{
                naming::FeatureKind::Subtract, {}, naming::kSubEdgeRoles,
                sub_operands};
            // The relational paths: [the entry] + the A leg + the B
            // leg — the operands' OWN subshape refs concatenated, the
            // composite the m2 goldens already pinned ([isect@hi,
            // box's top, ext's lat, 0] — depth 4, kMaxDepth = 8 with
            // room; ONLY integers in the path, the render the only
            // place the strings exist).
            const auto operand_leg =
                [](geometry::SubshapeRef ref, const Face& operand) {
                    for (std::uint8_t k = 0;
                         k < operand.shape_id().subshape.depth; ++k)
                        ref = ref.push(operand.shape_id().subshape.path[k]);
                    return ref;
                };
            const geometry::SubshapeRef rim_lo = operand_leg(
                operand_leg(geometry::SubshapeRef{}
                                .push(naming::kSubIsectLoRole),
                            *bot_operand),
                *wall_operand);
            const geometry::SubshapeRef rim_hi = operand_leg(
                operand_leg(geometry::SubshapeRef{}
                                .push(naming::kSubIsectHiRole),
                            *top_operand),
                *wall_operand);
            const geometry::SubshapeRef seam = operand_leg(
                geometry::SubshapeRef{}.push(naming::kSubSeamRole),
                *wall_operand);
            const auto stamp = [&](std::size_t index,
                                   const geometry::SubshapeRef& ref) {
                R->edges[index].set_shape_id(
                    {naming.feature, ref, {}});
                R->edges[index].set_name(naming::render_name(
                    naming.feature, ref, sub_naming.edges()));
            };
            stamp(e_base + 0, rim_lo);  // the bottom rim's -Y half
            stamp(e_base + 1, rim_lo);  // the bottom rim's +Y half
            stamp(e_base + 2, rim_hi);  // the top rim's -Y half
            stamp(e_base + 3, rim_hi);  // the top rim's +Y half
            stamp(e_base + 4, seam);    // the seam vertical at J_0
        }
    }

    // A local Line2 authoring helper (region-on-right throughout —
    // the convention this unit pins for the surviving walls).
    const auto line_rr = [](math::Vec2d a, math::Vec2d b) {
        return pcurve_segment(a, b, /*interior_left=*/false);
    };

    // ---- the walls ---------------------------------------------------
    // The ABSORBED wall inherits the tool's lateral's stamp (the
    // naming block above derives the correspondence once; the
    // inheritance is the stamp copy of the operand's own).
    const auto add_wall_face = [&](std::shared_ptr<SurfaceGeometry> surf,
                                   Loop loop, const Face* lateral) {
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        if (lateral != nullptr) {
            face.set_shape_id(lateral->shape_id());
            face.set_name(lateral->name());
        }
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
                const math::Vec2d d{e.q.x - e.p.x, e.q.y - e.p.y};
                const double len = std::sqrt(d.x * d.x + d.y * d.y);
                const math::Vec2d ln{-d.y / len, d.x / len};
                auto surf = std::make_shared<PlaneSurface>(
                    math::Vec3d{e.p.x, e.p.y, t.z_bot},
                    math::Vec3d{ln.x, ln.y, 0.0});
                const auto dom = [&](const math::Vec2d& xy, double z) {
                    return subtract_detail::plane_domain(
                        *surf, math::Vec3d{xy.x, xy.y, z});
                };  // Vec2d payload: the domain stores the exact
                    // world's own double (morso 3 — no conversion).
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
                add_wall_face(std::move(surf), std::move(loop),
                              lateral_of(i));
            } else {
                // The arc patch: CylinderZSurface on the CAVITY side
                // (u_sign = -1 — the frame rule), the window
                // [-psi_end, -psi_start] x [0, h].
                auto surf = std::make_shared<CylinderZSurface>(
                    e.center, e.radius, t.z_bot, -1);
                // The window payloads: the world's doubles, stored
                // as the domain's own doubles (morso 3 — the
                // quantizer retired with the float payload).
                const double u_p = -e.psi_start;
                const double u_q = -(e.psi_start + e.sweep);
                const double vh = h;
                CoEdge bot{eid(e_base + i), false};
                bot.set_p_curve(line_rr({u_p, 0.0}, {u_q, 0.0}));
                CoEdge up{eid(e_base + 2 * n + i), false};
                up.set_p_curve(line_rr({u_q, 0.0}, {u_q, vh}));
                CoEdge top{eid(e_base + n + i), true};
                top.set_p_curve(line_rr({u_q, vh}, {u_p, vh}));
                CoEdge down{eid(e_base + 2 * n + prev), true};
                down.set_p_curve(line_rr({u_p, vh}, {u_p, 0.0}));
                loop.add_coedge(bot);
                loop.add_coedge(up);
                loop.add_coedge(top);
                loop.add_coedge(down);
                add_wall_face(std::move(surf), std::move(loop),
                              lateral_of(i));
            }
        }
    } else {
        // THE SEAM-WIRED WALL (the ADR-0023 pattern, the patch between
        // the circles): the window [u0 - 2pi, u0] x [0, h] on the
        // REVERSED u, walked [seam down at u0 (REV), bottom west
        // (entity 1 then entity 0, both FWD), seam up at u0 - 2pi
        // (FWD), top east (entity 0 then entity 1, both REV)].
        const double psi0 = std::atan2(J[0].y - w.circle_center.y,
                                      J[0].x - w.circle_center.x);
        const double u0 = -psi0;
        auto surf = std::make_shared<CylinderZSurface>(
            w.circle_center, w.circle_radius, t.z_bot, -1);
        // The window payloads: the world's doubles, stored as the
        // domain's own doubles (morso 3 — the quantizer retired).
        const double f0 = u0;
        const double fp = subtract_detail::kPi;
        const double ftp = subtract_detail::kTwoPi;
        const double fh = h;
        Loop loop;
        CoEdge s_down{eid(e_base + 2 * n), true};
        s_down.set_p_curve(line_rr({f0, fh}, {f0, 0.0}));
        CoEdge bot1{eid(e_base + 1), false};
        bot1.set_p_curve(line_rr({f0, 0.0}, {f0 - fp, 0.0}));
        CoEdge bot0{eid(e_base + 0), false};
        bot0.set_p_curve(
            line_rr({f0 - fp, 0.0},
                    {f0 - ftp, 0.0}));
        CoEdge s_up{eid(e_base + 2 * n), false};
        s_up.set_p_curve(
            line_rr({f0 - ftp, 0.0},
                    {f0 - ftp, fh}));
        CoEdge top0{eid(e_base + n + 0), true};
        top0.set_p_curve(
            line_rr({f0 - ftp, fh},
                    {f0 - fp, fh}));
        CoEdge top1{eid(e_base + n + 1), true};
        top1.set_p_curve(
            line_rr({f0 - fp, fh}, {f0, fh}));
        loop.add_coedge(s_down);
        loop.add_coedge(bot1);
        loop.add_coedge(bot0);
        loop.add_coedge(s_up);
        loop.add_coedge(top0);
        loop.add_coedge(top1);
        add_wall_face(std::move(surf), std::move(loop), lateral_of(0));
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
        subtract_detail::TranslateReader tpr, bpr;
        top_f.surface()->accept(tpr);
        bot_f.surface()->accept(bpr);
        const PlaneSurface* top_plane = tpr.plane;
        const PlaneSurface* bot_plane = bpr.plane;
        const auto dom_top = [&](const math::Vec2d& xy) {
            return subtract_detail::plane_domain(
                *top_plane, math::Vec3d{xy.x, xy.y, t.z_top});
        };
        const auto dom_bot = [&](const math::Vec2d& xy) {
            return subtract_detail::plane_domain(
                *bot_plane, math::Vec3d{xy.x, xy.y, t.z_bot});
        };

        // The TOP inner loop: [entity 0 FWD, ..., entity n-1 FWD].
        Loop top_inner;
        top_inner.set_outer(false);
        for (std::size_t i = 0; i < n; ++i) {
            const auto& e = w.entities[i];
            CoEdge ce{eid(e_base + n + i), false};
            if (e.kind == subtract_detail::WalkEntity::Kind::Segment) {
                ce.set_p_curve(pcurve_segment(
                    dom_top(e.p), dom_top(e.q), /*interior_left=*/false));
            } else {
                const math::Vec2d c = dom_top(e.center);
                const math::Vec2d s = dom_top(e.p);
                ce.set_p_curve(pcurve_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x),
                    e.sweep));
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
                ce.set_p_curve(pcurve_segment(
                    dom_bot(e.q), dom_bot(e.p), /*interior_left=*/false));
            } else {
                const math::Vec2d c = dom_bot(e.center);
                const math::Vec2d s = dom_bot(e.q);  // the REVERSED start
                ce.set_p_curve(pcurve_arc(
                    c, e.radius, /*hole=*/true,
                    std::atan2(s.y - c.y, s.x - c.x),
                    e.sweep));
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
