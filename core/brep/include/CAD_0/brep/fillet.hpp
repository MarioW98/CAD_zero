// core/brep/include/CAD_0/brep/fillet.hpp
//
// THE FILLET, CLASS 1 — LO SPIGOLO D'ANGOLO DEL BOX (ADR-0039 D1.1,
// G2-u3 m2): the rolling ball on the box's convex 90° vertical edge,
// the cylindrical flank born in the edge's place. The first feature
// that SUBSTITUTES instead of subtracting: nothing is removed from
// the body's interior — the boundary itself is re-authored, the edge
// dies, the adjacent faces retreat to the tangent lines, the flank
// takes the corner's place.
//
// THE ROLLING BALL AS THE GEOMETRIC CONTRACT (D2): a CONSTRUCTION,
// never a solve — the exact path's whole discipline in one feature.
// The ball of radius r, inscribed in the material, tangent to both
// faces; its center rides the bisector locus at r/sin(α/2) from the
// edge (α the interior dihedral = π/2, the class's own angle); the
// swept envelope is a CYLINDER of radius r (CylinderZSurface's own
// family, the flank a quarter θ-window of it); the contact the exact
// tangency (the tangent lines at t = r·cot(α/2) = r — the 90°
// specialization exact in double, never r/tan(π/4)'s one-ULP read);
// the removed region the corner wedge, its area the closed form
// A(α,r) = r²·tan((π−α)/2) − (r²/2)(π−α), at π/2 the quarter corner
// r²(1−π/4). ZERO intersection solves: every quantity below is a
// closed form of the stored operands (the ball's center C = corner −
// r·(n_A + n_B), the tangent points the perpendicular feet C + r·n),
// and the SSI table's plane×cylinder ∥ entry is CONSUMED AT ITS
// TANGENT LIMIT BY CONSTRUCTION — the fillet places the tangent
// lines from the closed forms, it never solves the tangency (the
// entries are the vocabulary the payloads read, not solves the
// feature runs — D2's own words). The ε_line sentinel stays asleep:
// nothing transversal is solved, the Layer-2 bound on
// stored-vs-nominal is exact/0.
//
// THE ADMISSION, BEFORE THE CONSTRUCTION (the class's own, never a
// runtime geometry test): the body carries the FACTORY BOX's
// structural signature — 8 vertices, 12 edges, 6 planar single-loop
// faces, closed, orientable, χ = 2, every entity exact (the
// Layer-2 gate: an approximated body never enters) — and the
// reference resolves to one of its VERTICAL edges (the endpoints'
// (x,y) equal within the Linear band). Within that signature the
// edge is CONVEX 90° BY CONSTRUCTION (the admission (b) of ADR-0033
// D3: the ball rolls on the material side, inscribed): no runtime
// dihedral test runs, no convexity sign is computed — the class's
// guarantee IS the admission, and the tree's producers (the
// factories, the extrude's certificate, the union's convex
// fragments) cannot produce a non-convex carrier of the signature.
// The two adjacent faces must be PLANAR (the admission (a): both in
// the table's world) and perpendicular — their stored outward
// normals' dot reads exactly 0 (the axis-aligned signature's own
// arithmetic, not a tolerance decision; a skewed pair is the GENERAL
// DIHEDRAL family, registered for its first consumer — ADR-0039
// D1.1's "never smuggled").
//
// THE ORTHOGONAL PRISM JOINTS — the class's registered next
// instance (ADR-0039 D1.1's formula V = 2n+2, E = 3n+3, F = n+3):
// the same walk, landing with their first consumer (the joint(i)
// stamps exist today only on subtract results; the m2 vocabulary
// names the boundary rather than guessing past it).
//
// LE BANDE TIPIZZATE (D2, ADR-0035's Linear world — ε_L the
// operation's declared tolerance, ZERO new constants): the tangent
// lines at t = r must sit STRICTLY inside both adjacent faces
// (r < the face's reach from the edge − ε_L, the reach read from
// the face's own loop vertices in the inward direction), and both
// end faces must contain the quarter arcs (r < the corner's ring
// edges' extents − ε_L). Beyond the band: `fillet_radius_exceeds_face`
// with the offending bound NAMED IN THE PAYLOAD (the face, its
// reach, the ε_L — never a dry string without the number). THE
// MEMBER THIS BITE CAN MEET: exactly this one — the box class has
// no annulus (the rim class's `fillet_exits_annulus`) and no wall
// span (`fillet_eats_wall`); both stay REGISTERED (ADR-0033 D4's
// corpus) with their seats in m3's rim class, unimplemented here.
// The concave, the variable-radius and the three-face families stay
// MURO (ADR-0039 D1.3): refused by name, no writer, no queue seat.
//
// THE DECOUPLING RULE (ADR-0038 m4) IS THE ASSEMBLY'S LAW: the
// frame serves the geometry, the walk serves the topology. The four
// absorbed faces (the two adjacent laterals, the two end caps) keep
// their NATIVE frames — the same mathematical planes, the same
// p-curve vocabulary — and gain re-authored walks in the result's
// canonical direction, the region bits carrying the material side
// (every loop CCW in its domain, interior on the left); the born
// flank carries its OWN right-handed frame (CylinderZSurface,
// u_sign = +1: ∂u×∂v = +radial = the outward, the material lying
// inside the cylinder — the frame rule verified pointwise by the
// tests). The kept faces (front, left) are copied verbatim, their
// edge references remapped through the compaction.
//
// THE COUNTS (D1.1's table, the guide case box [-1,1]³, the edge
// `box#1:vertical2` at r = 0.5): the edge dies — 1 edge and its 2
// corner vertices vanish; the four ring edges at the dead corners
// RE-SPAN (absorbed); the flank is born with 4 boundary edges (2
// tangent lines + 2 quarter arcs) and 4 vertices:
//
//     V = 8 − 2 + 4 = 10    E = 12 − 1 + 4 = 15    F = 6 + 1 = 7
//     H = 0                 χ = 10 − 15 + 7 = 2 (Δχ = +2 −3 +1 = 0)
//
// The co-edge census closes: right 4 + back 4 + top 5 + bottom 5 +
// front 4 + left 4 + flank 4 = 30 incidences / 2 = 15 edges, every
// edge exactly two OPPOSITE co-edges — the output gate's own
// identity, verified as a postcondition.
//
// THE NUMBERING (the construction's own tables, frozen by the
// tests — the crossed-corner's own discipline): the survivors
// COMPACT (the dead column dropped), the born APPEND:
//
//   vertices   0..5   the surviving corners (the old 0,1,3,4,5,7)
//              6/7    T_A_lo / T_A_hi  (the first face's tangent)
//              8/9    T_B_lo / T_B_hi  (the second's)
//   edges      0..9   the kept ring edges and verticals (old 0..9)
//              10     the surviving far vertical (old 11)
//              11     e_tan_A  (6→7,  the first tangent, bottom→top)
//              12     e_tan_B  (8→9,  the second tangent, bottom→top)
//              13     e_arc_lo (8→6,  the low quarter arc)
//              14     e_arc_hi (9→7,  the high quarter arc)
//   faces      0..5   the box's own (the four absorbed re-authored)
//              6      the flank (born)
//
// THE NAMES (ADR-0039 D3 — the moment ADR-0036 exists for): the
// recipe's edge-reference is a NAME (this vocabulary; the m2 kernel
// surface takes the structural ShapeId, the recipe's string field
// lands with m4's format morse). The flank: `fil#F:flank[<A> ×
// <B>]` — the referenced edge's two adjacent faces IN THE EDGE'S
// OWN DECLARED CO-EDGE ORDER (the radial list's shell order —
// deterministic, input-order-independent); the guide case
// `fil#2:flank[box#1:back × box#1:right]` (the edge 10's first
// co-edge sits on face 3). The tangent wires: `fil#F:isect[<the
// landing face> × <the flank>]` — the level ABSENT, the tangency IS
// the level; the second operand the SAME feature's flank (the
// same-feature extension, born earlier in the construction's walk).
// The flank carries NO seam (a bounded quarter patch, not a periodic
// closure); the two quarter arcs read honestly UNNAMED (the
// vocabulary's own boundary, naming.hpp's table). The shrink faces
// and the re-spanned edges KEEP their names (the stamp copy): the
// operation's mark rides the tavola's absorbed rows, never a minted
// identity.
//
// THE VOLUME (D1.1's derivation, the classical form IS THE TEST):
// V = abc − ℓ·r²(1−π/4) — the guide case 8 − 2·0.25·(1−π/4) =
// 7.5 + π/8 ≈ 7.892699081698725, all_exact: the planar Green
// machinery and the cylindrical window integrals the mass path
// already carries answer it — the flank patch's mass and the
// analytic wedge the same arithmetic read twice (the m2 golden's
// own pin).
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/naming.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::brep {

// The operation's declared Linear tolerance (ADR-0035 D2.1: one ε_L
// per operation, explicit at the signature — the same declaration
// discipline as kSubtractTolerance). Consumed at the band gates
// below: the verticality classification, the tangent lines' strict
// interiority, the quarter arcs' containment.
inline constexpr double kFilletTolerance = 1e-6;

// The postcondition defect detector for the authored quarter arc's
// angular span: the closed form says π/2 exactly; the atan2
// roundings on the projected endpoints leave ≤ ~4·ε_mach ≈ 1e-15 at
// the O(1) angles this class reaches, so a span outside 1e-9 of π/2
// is a GROSS assembly defect (the wrong center, the wrong pair) —
// this is a defect detector, never a geometric decision: every
// geometric decision in this file reads the typed ε_L above.
inline constexpr double kFilletArcSpanDefectBand = 1e-9;

// The fillet's naming context — the fillet's own DAG id (the born
// names' creator part, 'fil#F') and the filleted body's own naming
// declaration (its role tables — the flank's operands render through
// them). The body's feature id flows from the body's own stamps (the
// found edge's creator); the tables are the caller's context, the
// exact_subtract precedent. The unnamed mode (body == nullptr, the
// default) leaves the born entities honestly empty — the inheritance
// (the shrink faces' names) flows in both modes (a stamp copy, no
// table involved).
struct FilletNaming {
    geometry::FeatureId feature{};
    const naming::OperationNaming* body{};
};

namespace fillet_detail {

// The plane payload reader (the typed dispatch — no dynamic_cast
// chains in the exact path).
struct PlaneReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    void visit(const PlaneSurface& p) override { plane = &p; }
};

// The plane's domain coordinates of a 3D point (the p-curve writers'
// own projection — factory.hpp's plane_domain, local to this header
// so the fillet adds no include edge).
inline math::Vec2d dom_of(const PlaneSurface& s, math::Vec3d p) {
    const math::Vec3d d = p - s.origin();
    return {d.dot(s.u_axis()), d.dot(s.v_axis())};
}

} // namespace fillet_detail

// fillet_edge — the rolling ball on the box's vertical edge, class 1
// of ADR-0039 D1.1 (plane×plane, the convex 90° corner of the
// factory box family). `edge_reference` is the structural name of the
// edge to fillet (e.g. box#1:vertical2's ShapeId — the pick's own
// answer, found against the body's stamps; never parsed, never an
// address); `radius` the constant ball radius r; `naming` the
// context above; `tol` the operation's declared Linear tolerance.
//
// LOUD on every refusal (the P2 culture, the bounds NAMED — ADR-0033
// D4's actionable-error discipline): the reference that finds no
// edge (or a seam-wired pair — a fillet asked on a seam is refused
// with the pair named), the body outside the class signature, the
// non-vertical or non-planar-adjacent edge, the skewed dihedral (the
// registered general family), the radius outside the typed band
// (`fillet_radius_exceeds_face`, the face and its reach carried).
// Returns the filleted body; the tavola is fillet_descent's own
// derivation (descent.hpp — the recipe is the source of truth, the
// tavola is cache).
inline std::unique_ptr<BRepBody> fillet_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double radius, const FilletNaming& naming = FilletNaming{},
    double tol = kFilletTolerance);

// ---------------------------------------------------------------------------
// The implementation (inline, the subtract's own pattern)
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> fillet_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double radius, const FilletNaming& naming, double tol) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("fillet_edge: " + what);
    };

    // ---- The radius's own gate (the band's arithmetic needs it sane
    // first — a non-positive radius has no ball).
    if (!(radius > 0.0) || !std::isfinite(radius))
        refuse("the radius must be finite and > 0, got " +
               std::to_string(radius) +
               " — a non-positive ball has no envelope and no tangent "
               "lines. Remedy: author a positive radius within the "
               "faces' reach (the band the admission derives).");

    // ---- The class signature: the FACTORY BOX's own (the admission
    // BEFORE the construction — the counts and the families, zero
    // dihedral arithmetic, zero convexity signs).
    {
        bool all_exact = true;
        for (const auto& v : body.vertices)
            if (!v.provenance().is_exact()) all_exact = false;
        for (const auto& e : body.edges)
            if (!e.provenance().is_exact()) all_exact = false;
        for (const auto& f : body.solid.shell().faces())
            if (!f.provenance().is_exact()) all_exact = false;
        if (!all_exact)
            refuse("the body carries approximated entities (the "
                   "Layer-2 record says so) - the exact fillet "
                   "consumes EXACT bodies only (the booleans' own "
                   "gate). Remedy: author the body on the exact path "
                   "(the factories), or accept the faceted path.");
        if (V0 != 8 || E0 != 12 || F0 != 6 ||
            euler_characteristic(body) != 2 || !is_closed_orientable(body))
            refuse("the body is outside the class signature (the "
                   "factory box's own: 8 vertices, 12 edges, 6 faces, "
                   "chi = 2, closed orientable; found " +
                   std::to_string(V0) + "/" + std::to_string(E0) +
                   "/" + std::to_string(F0) +
                   ") - the v1 admission is the box family's convex "
                   "90-degree corners (ADR-0039 D1.1). Remedy: fillet "
                   "a factory box's vertical edge.");
        for (const auto& f : body.solid.shell().faces()) {
            if (f.loops().size() != 1)
                refuse("a face carries " +
                       std::to_string(f.loops().size()) +
                       " loops - the class signature is the factory "
                       "box's single-loop faces (a pierced or notched "
                       "face is a later family's carrier). Remedy: "
                       "fillet a factory box's vertical edge.");
            const auto* surf = f.surface();
            fillet_detail::PlaneReader rd;
            if (surf == nullptr ||
                (surf->accept(rd), rd.plane == nullptr))
                refuse("a face carries a non-planar surface - the "
                       "class signature is the factory box's six "
                       "planes (the admission (a): both adjacent "
                       "faces in the table's world). Remedy: fillet "
                       "a factory box's vertical edge.");
        }
    }

    // ---- The reference's own find: the structural match against the
    // body's stamps (the by-name query's discipline — never parsed,
    // never an address; the generation field not compared, ADR-0036
    // D1's rule).
    std::size_t edge = E0;
    {
        std::vector<std::size_t> found;
        for (std::size_t i = 0; i < E0; ++i) {
            const auto& id = body.edges[i].shape_id();
            if (id.subshape.depth > 0 &&
                id.feature == edge_reference.feature &&
                id.subshape == edge_reference.subshape)
                found.push_back(i);
        }
        if (found.empty())
            refuse("the edge reference names no edge of this body "
                   "(the structural find against the stamps found no "
                   "match) - a reference that finds no row at all is "
                   "a closure violation, not a user error: resolve "
                   "the reference against the generation it was made "
                   "on (the resolver's own guard), or re-pick the "
                   "edge on this body.");
        if (found.size() > 1)
            refuse("the edge reference materializes as " +
                   std::to_string(found.size()) +
                   " edges (the seam-wired pair) - a fillet asked on "
                   "a seam is refused with the pair named (ADR-0033 "
                   "D3's own boundary). Remedy: reference a "
                   "non-periodic edge (the box's verticals are "
                   "single).");
        edge = found[0];
    }

    // ---- The verticality (the construction's own axis: the flank
    // is a CylinderZSurface, the Z-axis family).
    const auto vpos = [&](std::size_t v) -> const math::Vec3d& {
        return body.vertices[v].position();
    };
    const VertexId ev0 = body.edges[edge].vertex(0);
    const VertexId ev1 = body.edges[edge].vertex(1);
    const math::Vec3d& p0 =
        vpos(static_cast<std::size_t>(ev0));
    const math::Vec3d& p1 =
        vpos(static_cast<std::size_t>(ev1));
    if (std::abs(p0.x - p1.x) > tol || std::abs(p0.y - p1.y) > tol)
        refuse("the referenced edge is not vertical (its endpoints' "
               "(x, y) differ by more than the declared Linear "
               "tolerance " + std::to_string(tol) +
               ") - the v1 class takes the VERTICAL spigolo (the "
               "flank is the Z-axis cylinder family, the guide "
               "case's own edge). Remedy: reference one of the box's "
               "verticals (box#F:vertical0..3).");
    const std::size_t v_lo = p0.z <= p1.z
                                 ? static_cast<std::size_t>(ev0)
                                 : static_cast<std::size_t>(ev1);
    const std::size_t v_hi = p0.z <= p1.z
                                 ? static_cast<std::size_t>(ev1)
                                 : static_cast<std::size_t>(ev0);
    const math::Vec3d& c_lo = vpos(v_lo);
    const math::Vec3d& c_hi = vpos(v_hi);
    const double z_lo = c_lo.z, z_hi = c_hi.z;
    const double ell = z_hi - z_lo;
    if (ell <= tol)
        refuse("the referenced edge is degenerate (its length " +
               std::to_string(ell) + " is within the declared Linear "
               "tolerance) - a zero-length edge has no bisector "
               "locus. Remedy: report the body's construction (the "
               "factory never authors one).");

    // ---- The two adjacent faces, in the edge's own declared
    // co-edge order (the radial list's shell order — the flank's
    // operands' order, deterministic and input-order-independent).
    const auto& dead_coedges = body.edges[edge].coedges();
    if (dead_coedges.size() != 2)
        refuse("the referenced edge carries " +
               std::to_string(dead_coedges.size()) +
               " co-edges - the manifold class admits exactly two "
               "(the box's edges are manifold by construction). "
               "Remedy: report the body's construction.");
    const auto face_of_coedge = [&](const CoEdge* ce) -> std::size_t {
        for (std::size_t f = 0; f < F0; ++f)
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& other : loop.coedges())
                    if (&other == ce) return f;
        return F0;
    };
    const std::size_t face_A = face_of_coedge(dead_coedges[0]);
    const std::size_t face_B = face_of_coedge(dead_coedges[1]);
    if (face_A >= F0 || face_B >= F0 || face_A == face_B)
        refuse("the adjacent faces' structural lookup failed (the "
               "radial co-edges' owning faces) - report this as a "
               "kernel bug.");
    const Face& src_A = body.solid.shell().faces()[face_A];
    const Face& src_B = body.solid.shell().faces()[face_B];
    fillet_detail::PlaneReader rd_A, rd_B;
    src_A.surface()->accept(rd_A);
    src_B.surface()->accept(rd_B);
    if (rd_A.plane == nullptr || rd_B.plane == nullptr)
        refuse("an adjacent face is non-planar - the admission (a) "
               "takes both adjacent faces in the table's world "
               "(planes); the curved families are the rim class's "
               "(m3). Remedy: reference an edge between two planar "
               "faces (the box's verticals).");
    const math::Vec3d n_A = rd_A.plane->normal_vec();
    const math::Vec3d n_B = rd_B.plane->normal_vec();
    // The 90° dihedral: the axis-aligned signature's own exact
    // arithmetic (the stored unit normals' dot), never a tolerance
    // decision — the closed forms below (the bisector n_A + n_B, the
    // tangent distance r) are exact for the perpendicular pair and
    // wrong for anything else.
    if (n_A.dot(n_B) != 0.0)
        refuse("the adjacent faces' dihedral is not the class's 90 "
               "degrees (the stored normals' dot reads " +
               std::to_string(n_A.dot(n_B)) +
               " where the perpendicular pair reads exactly 0) - the "
               "GENERAL DIHEDRAL family is the same derivation one "
               "parameter wider, REGISTERED for its first consumer "
               "(a non-orthogonal prism joint), never smuggled "
               "(ADR-0039 D1.1). Remedy: fillet an orthogonal "
               "corner, or await the general-dihedral morse.");

    // ---- The caps: for each dead corner, the face that uses it and
    // is neither adjacent face (the box's own: the bottom at the low
    // corner, the top at the high one).
    const auto cap_of_vertex = [&](std::size_t dead_v) -> std::size_t {
        std::size_t cap = F0;
        for (std::size_t f = 0; f < F0; ++f) {
            if (f == face_A || f == face_B) continue;
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges()) {
                    const auto& e = body.edges[ce.edge()];
                    if (e.vertex(0) == dead_v ||
                        e.vertex(1) == dead_v) {
                        if (cap != F0 && cap != f)
                            refuse("the dead corner sits on more than "
                                   "one end face - the class signature "
                                   "is the box's (exactly one cap per "
                                   "corner). Remedy: report the "
                                   "body's construction.");
                        cap = f;
                    }
                }
        }
        return cap;
    };
    const std::size_t cap_lo = cap_of_vertex(v_lo);
    const std::size_t cap_hi = cap_of_vertex(v_hi);
    if (cap_lo >= F0 || cap_hi >= F0)
        refuse("a dead corner's end-face lookup failed - report "
               "this as a kernel bug.");

    // ---- LE BANDE TIPIZZATE (the Linear world's own ε_L, the
    // bounds named in the refusal): the tangent lines strictly
    // inside both adjacent faces; the quarter arcs contained by both
    // caps (the corner's ring edges' extents).
    const auto inward_reach = [&](const Face& face,
                                  const PlaneSurface& pl,
                                  const CoEdge* dead_ce) {
        // The inward direction: the LEFT of the dead co-edge's
        // traversal (the material side, read through the face's
        // outward normal) — the co-edge's own region bit.
        const math::Vec3d t = dead_ce->reversed()
                                  ? math::Vec3d{0, 0, -1}
                                  : math::Vec3d{0, 0, 1};
        const math::Vec3d inward = pl.normal_vec().cross(t);
        double reach = 0.0;
        for (const auto& loop : face.loops())
            for (const auto& ce : loop.coedges()) {
                const auto& e = body.edges[ce.edge()];
                for (const int endp : {0, 1}) {
                    const math::Vec3d q =
                        vpos(static_cast<std::size_t>(e.vertex(endp)));
                    reach = std::max(reach, (q - c_lo).dot(inward));
                }
            }
        return reach;
    };
    const double reach_A =
        inward_reach(src_A, *rd_A.plane, dead_coedges[0]);
    const double reach_B =
        inward_reach(src_B, *rd_B.plane, dead_coedges[1]);
    // The caps' containment: the corner's ring edges on each cap —
    // the two loop edges incident to the dead vertex — must extend
    // past the tangent points.
    const auto cap_reach = [&](std::size_t cap, std::size_t dead_v) {
        double reach = std::numeric_limits<double>::infinity();
        const auto& loop =
            body.solid.shell().faces()[cap].loops()[0].coedges();
        std::size_t incident = 0;
        for (const auto& ce : loop) {
            const auto& e = body.edges[ce.edge()];
            const bool has_dead =
                e.vertex(0) == dead_v || e.vertex(1) == dead_v;
            if (!has_dead || ce.edge() == edge) continue;
            ++incident;
            const std::size_t other =
                e.vertex(0) == dead_v
                    ? static_cast<std::size_t>(e.vertex(1))
                    : static_cast<std::size_t>(e.vertex(0));
            const math::Vec3d dq = vpos(other) - vpos(dead_v);
            reach = std::min(reach, std::sqrt(dq.length_sq()));
        }
        if (incident != 2)
            refuse("an end face's corner carries " +
                   std::to_string(incident) +
                   " incident ring edges (the box's own are two) - "
                   "the class signature is the box's. Remedy: report "
                   "the body's construction.");
        return reach;
    };
    const double reach_cap_lo = cap_reach(cap_lo, v_lo);
    const double reach_cap_hi = cap_reach(cap_hi, v_hi);

    const auto band_refusal = [&](const char* what, double reach) {
        refuse(std::string(what) +
               " (fillet_radius_exceeds_face): the radius " +
               std::to_string(radius) +
               " against the typed band's bound " +
               std::to_string(reach) + " - " + std::to_string(tol) +
               " (the face's own reach minus the declared Linear "
               "tolerance): the tangent line would exit the face, "
               "and the flank's closed forms would place it outside "
               "the material. Remedy: shrink the radius below " +
               std::to_string(reach - tol) +
               ", or restructure the body (the band is the face's "
               "own reach, ADR-0039 D2's derived decision).");
    };
    if (radius >= reach_A - tol)
        band_refusal("the tangent line exits the first adjacent face",
                     reach_A);
    if (radius >= reach_B - tol)
        band_refusal("the tangent line exits the second adjacent face",
                     reach_B);
    if (radius >= reach_cap_lo - tol)
        band_refusal("the quarter arc exits the low end face",
                     reach_cap_lo);
    if (radius >= reach_cap_hi - tol)
        band_refusal("the quarter arc exits the high end face",
                     reach_cap_hi);

    // ---- THE CONSTRUCTION — every quantity a closed form of the
    // stored operands, ZERO solves (D2's contract).
    // The ball's center on the bisector (the 90° closed form: the
    // corner offset by r along each inward normal — the sum
    // r·(n_A + n_B) IS the bisector locus at r/sin(π/4) = r·√2).
    const math::Vec3d corner = c_lo;  // (x, y) shared by the edge
    const math::Vec2d center{corner.x - radius * (n_A.x + n_B.x),
                             corner.y - radius * (n_A.y + n_B.y)};
    // The tangent points: the feet of the perpendiculars from the
    // center onto the two planes — at t = r·cot(π/4) = r from the
    // edge, exact (never r/tan(π/4)'s one-ULP read).
    const math::Vec2d tan_A{center.x + radius * n_A.x,
                            center.y + radius * n_A.y};
    const math::Vec2d tan_B{center.x + radius * n_B.x,
                            center.y + radius * n_B.y};
    // The flank's azimuth window: the quarter facing the corner,
    // from the azimuth of one normal to the other's, CCW (the
    // normals are perpendicular — the cross product picks the start).
    const double az_B = std::atan2(n_B.y, n_B.x);
    const double az_A = std::atan2(n_A.y, n_A.x);
    const double cross_BA = n_B.x * n_A.y - n_B.y * n_A.x;
    const bool b_at_u0 = cross_BA > 0.0;
    const double u0 = b_at_u0 ? az_B : az_A;
    const double u1 = u0 + kPi / 2.0;

    // ---- The maps: the survivors compact (the dead column
    // dropped), the born append (the numbering table above).
    std::vector<std::size_t> vmap(V0, V0), emap(E0, E0);
    {
        std::size_t next = 0;
        for (std::size_t i = 0; i < V0; ++i)
            if (i != v_lo && i != v_hi) vmap[i] = next++;
        next = 0;
        for (std::size_t i = 0; i < E0; ++i)
            if (i != edge) emap[i] = next++;
    }
    const std::size_t v_base = V0 - 2;
    const std::size_t v_TA_lo = v_base + 0, v_TA_hi = v_base + 1;
    const std::size_t v_TB_lo = v_base + 2, v_TB_hi = v_base + 3;
    const std::size_t e_base = E0 - 1;
    const std::size_t e_tan_A = e_base + 0;   // (v_TA_lo -> v_TA_hi)
    const std::size_t e_tan_B = e_base + 1;   // (v_TB_lo -> v_TB_hi)
    const std::size_t e_arc_lo = e_base + 2;  // (v_TB_lo -> v_TA_lo)
    const std::size_t e_arc_hi = e_base + 3;  // (v_TB_hi -> v_TA_hi)

    // The dead endpoint's substitute: the tangent vertex of the
    // adjacent face (A or B) that co-uses the re-spanned edge, at
    // the dead endpoint's own quota.
    const auto substitute = [&](std::size_t dead_v,
                                std::size_t re_spanned) -> std::size_t {
        for (const CoEdge* ce : body.edges[re_spanned].coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (f == face_A)
                return dead_v == v_lo ? v_TA_lo : v_TA_hi;
            if (f == face_B)
                return dead_v == v_lo ? v_TB_lo : v_TB_hi;
        }
        refuse("a re-spanned edge's co-user is neither adjacent face "
               "- the class signature is the box's (the corner's "
               "ring edges each border one of the two laterals). "
               "Remedy: report the body's construction.");
        return 0;  // unreachable — refuse() throws
    };
    // The re-spanning map: a surviving edge with a dead endpoint is
    // re-spanned (the endpoint substituted); its stored pair and its
    // p-curves on every face re-author below.
    const auto new_end = [&](std::size_t old_end, std::size_t e) {
        if (old_end != v_lo && old_end != v_hi) return vmap[old_end];
        return substitute(old_end, e);
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- The vertices: the survivors (the copies carry the
    // records), then the born tangent pairs (A first — the co-edge
    // order, the flank's own operands' order).
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        R->vertices.push_back(body.vertices[i]);
        R->vertices.back().set_id(static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[4] = {
            {tan_A.x, tan_A.y, z_lo}, {tan_A.x, tan_A.y, z_hi},
            {tan_B.x, tan_B.y, z_lo}, {tan_B.x, tan_B.y, z_hi}};
        const std::size_t ids[4] = {v_TA_lo, v_TA_hi, v_TB_lo,
                                    v_TB_hi};
        for (std::size_t i = 0; i < 4; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the endpoints remapped, the
    // dead column's substitutions applied), then the born four (the
    // tangent lines bottom->top, the seam's own convention; the
    // quarter arcs from the second face's tangent to the first's —
    // the deterministic B->A convention).
    for (std::size_t i = 0; i < E0; ++i) {
        if (emap[i] == E0) continue;
        R->edges.push_back(body.edges[i]);
        auto& e = R->edges.back();
        e.set_id(static_cast<EdgeId>(emap[i]));
        e.set_vertex(0, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(e.vertex(0)), i)));
        e.set_vertex(1, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(e.vertex(1)), i)));
    }
    {
        const std::pair<std::size_t, std::size_t> born[4] = {
            {v_TA_lo, v_TA_hi}, {v_TB_lo, v_TB_hi},
            {v_TB_lo, v_TA_lo}, {v_TB_hi, v_TA_hi}};
        for (const auto& [a, b] : born) {
            Edge e{static_cast<VertexId>(a),
                   static_cast<VertexId>(b)};
            e.set_id(static_cast<EdgeId>(R->edges.size()));
            e.set_provenance(prov);
            R->edges.push_back(e);
        }
    }

    // ---- The faces. The four absorbed (the laterals, the caps)
    // re-author their walks; the kept copy verbatim (the ids
    // remapped); the flank is born. The re-authoring is the
    // crossed-corner's own discipline: per co-edge, COPY (the id
    // remapped, the p-curve verbatim), RE-SPAN (the endpoints
    // substituted, the p-curve re-projected), or NEW (the tangent
    // line replacing the dead co-edge on the laterals; the quarter
    // arc inserted between the re-spanned pair on the caps) — every
    // flag derived from the walk's (start, end) against the stored
    // pair, every p-curve projected through the face's own native
    // frame (the decoupling rule: the frame serves the geometry, the
    // walk serves the topology).
    const auto pos3 = [&](std::size_t v) -> const math::Vec3d& {
        return R->vertices[v].position();
    };

    // The re-authored planar co-edge: the direction derived from the
    // stored pair (an exact integer comparison — never a distance
    // test), the p-curve projected through the face's own plane.
    const auto emit_planar = [&](Loop& loop, const PlaneSurface& pl,
                                 std::size_t e_id, std::size_t a,
                                 std::size_t b) {
        const auto& stored = R->edges[e_id];
        const bool fwd =
            stored.vertex(0) == static_cast<VertexId>(a) &&
            stored.vertex(1) == static_cast<VertexId>(b);
        const bool rev =
            stored.vertex(0) == static_cast<VertexId>(b) &&
            stored.vertex(1) == static_cast<VertexId>(a);
        if (!fwd && !rev)
            refuse("the re-authored walk lost the chaining (an edge "
                   "that does not run between the walk's endpoints) "
                   "- report this as a kernel bug.");
        CoEdge ce{static_cast<EdgeId>(e_id), rev};
        ce.set_p_curve(pcurve_segment(
            fillet_detail::dom_of(pl, pos3(a)),
            fillet_detail::dom_of(pl, pos3(b)),
            /*interior_left=*/true));
        loop.add_coedge(ce);
    };

    // One old co-edge re-emitted on a planar face: the id remapped,
    // the flag derived from the stored pair, the p-curve re-projected
    // where re-spanned, copied verbatim where kept.
    const auto reemit = [&](const PlaneSurface& pl,
                            const CoEdge& ce) {
        const std::size_t old_edge = ce.edge();
        if (emap[old_edge] == E0)
            refuse("the dead column's edge appeared in a face "
                   "outside the re-authored set - report this as a "
                   "kernel bug.");
        const std::size_t s_old = static_cast<std::size_t>(
            ce.reversed() ? body.edges[old_edge].vertex(1)
                          : body.edges[old_edge].vertex(0));
        const std::size_t e_old = static_cast<std::size_t>(
            ce.reversed() ? body.edges[old_edge].vertex(0)
                          : body.edges[old_edge].vertex(1));
        const std::size_t s_new = new_end(s_old, old_edge);
        const std::size_t e_new = new_end(e_old, old_edge);
        const bool re_spanned = s_old == v_lo || s_old == v_hi ||
                                e_old == v_lo || e_old == v_hi;
        CoEdge out{static_cast<EdgeId>(emap[old_edge]), false};
        const auto& stored = R->edges[emap[old_edge]];
        const bool fwd =
            stored.vertex(0) == static_cast<VertexId>(s_new) &&
            stored.vertex(1) == static_cast<VertexId>(e_new);
        const bool rev =
            stored.vertex(0) == static_cast<VertexId>(e_new) &&
            stored.vertex(1) == static_cast<VertexId>(s_new);
        if (!fwd && !rev)
            refuse("the re-authored walk lost the chaining (the "
                   "stored pair does not run between the walk's "
                   "endpoints) - report this as a kernel bug.");
        out.set_reversed(rev);
        if (re_spanned) {
            out.set_p_curve(pcurve_segment(
                fillet_detail::dom_of(pl, pos3(s_new)),
                fillet_detail::dom_of(pl, pos3(e_new)),
                /*interior_left=*/true));
        } else if (ce.p_curve() != nullptr) {
            out.set_p_curve(*ce.p_curve());
        }
        return out;
    };

    // The lateral's re-authoring: the dead co-edge replaced by this
    // face's tangent line (the dead co-edge's own walk direction
    // preserved), the re-spanned neighbors re-projected, the rest
    // copied verbatim.
    const auto reauthor_lateral = [&](std::size_t face_index,
                                      std::size_t tan_edge,
                                      const PlaneSurface& pl) {
        const Face& src =
            body.solid.shell().faces()[face_index];
        const auto& ces = src.loops()[0].coedges();
        Loop loop;
        for (const auto& ce : ces) {
            if (ce.edge() == edge) {
                // The tangent line, the dead co-edge's own walk
                // direction (the Z sense of its traversal — near
                // endpoint's slot, the factory's own authoring
                // rule, read through the walk's own vertices).
                const std::size_t d_start = static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[edge].vertex(1)
                        : body.edges[edge].vertex(0));
                const std::size_t d_end = static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[edge].vertex(0)
                        : body.edges[edge].vertex(1));
                const auto& te = R->edges[tan_edge];
                const std::size_t lo_id =
                    static_cast<std::size_t>(te.vertex(0));
                const std::size_t hi_id =
                    static_cast<std::size_t>(te.vertex(1));
                const std::size_t from =
                    vpos(d_start).z <= vpos(d_end).z ? lo_id
                                                     : hi_id;
                const std::size_t to =
                    vpos(d_start).z <= vpos(d_end).z ? hi_id
                                                     : lo_id;
                emit_planar(loop, pl, tan_edge, from, to);
                continue;
            }
            loop.add_coedge(reemit(pl, ce));
        }
        Face face(std::make_shared<PlaneSurface>(pl));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(face_index));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= face_index)
            faces_mut.resize(face_index + 1);
        faces_mut[face_index] = std::move(face);
    };

    // The cap's re-authoring: the two re-spanned ring edges
    // re-projected, the quarter arc inserted between them (the
    // walk's own direction — CCW in the cap's domain, the material
    // on the disc side, the Arc2's CCW-store discipline), the rest
    // copied verbatim.
    const auto reauthor_cap = [&](std::size_t face_index,
                                  std::size_t dead_v,
                                  std::size_t arc_edge) {
        const Face& src =
            body.solid.shell().faces()[face_index];
        fillet_detail::PlaneReader rd;
        src.surface()->accept(rd);
        const PlaneSurface& pl = *rd.plane;
        const auto& ces = src.loops()[0].coedges();
        Loop loop;
        for (std::size_t j = 0; j < ces.size(); ++j) {
            const auto& ce = ces[j];
            const std::size_t e_old = static_cast<std::size_t>(
                ce.reversed() ? body.edges[ce.edge()].vertex(0)
                              : body.edges[ce.edge()].vertex(1));
            loop.add_coedge(reemit(pl, ce));
            // The arc's insertion: after the re-spanned co-edge that
            // ENDS at the corner's substitute — between it and the
            // next co-edge's substituted start.
            if (e_old == dead_v) {
                const auto& next = ces[(j + 1) % ces.size()];
                const std::size_t n_old = static_cast<std::size_t>(
                    next.reversed()
                        ? body.edges[next.edge()].vertex(1)
                        : body.edges[next.edge()].vertex(0));
                const std::size_t n_new =
                    new_end(n_old, next.edge());
                // The walk's current position: the re-spanned
                // co-edge's END (reemit derived its flags from the
                // stored pair — the end is e_old's substitute).
                const std::size_t here =
                    new_end(e_old, ce.edge());
                // The quarter arc in the cap's own domain: center
                // the flank axis's foot, radius r, the CCW span from
                // the walk's start to its end (the material on the
                // disc side — the walk is the region's boundary).
                const math::Vec2d a2 =
                    fillet_detail::dom_of(pl, pos3(here));
                const math::Vec2d b2 =
                    fillet_detail::dom_of(pl, pos3(n_new));
                const math::Vec2d c2 = fillet_detail::dom_of(
                    pl, math::Vec3d{center.x, center.y,
                                    vpos(dead_v).z});
                const double a0 =
                    std::atan2(a2.y - c2.y, a2.x - c2.x);
                const double a1 =
                    std::atan2(b2.y - c2.y, b2.x - c2.x);
                double sweep = a1 - a0;
                if (sweep <= 0.0) sweep += 2.0 * kPi;
                if (std::abs(sweep - kPi / 2.0) >
                    kFilletArcSpanDefectBand)
                    refuse("the quarter arc's span reads " +
                           std::to_string(sweep) +
                           " (the 90-degree class's own is pi/2) - "
                           "the closed forms and the walk disagree; "
                           "report this as a kernel bug.");
                const auto& stored = R->edges[arc_edge];
                const bool fwd =
                    stored.vertex(0) ==
                            static_cast<VertexId>(here) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(n_new);
                const bool rev =
                    stored.vertex(0) ==
                            static_cast<VertexId>(n_new) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(here);
                if (!fwd && !rev)
                    refuse("the quarter arc's stored pair does not "
                           "run between the walk's endpoints - "
                           "report this as a kernel bug.");
                CoEdge arc{static_cast<EdgeId>(arc_edge), rev};
                arc.set_p_curve(pcurve_arc(
                    c2, radius, /*hole=*/false, a0, sweep));
                loop.add_coedge(arc);
            }
        }
        Face face(std::make_shared<PlaneSurface>(pl));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(face_index));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= face_index)
            faces_mut.resize(face_index + 1);
        faces_mut[face_index] = std::move(face);
    };

    // The four absorbed faces re-author; the kept copy verbatim.
    reauthor_lateral(face_A, e_tan_A, *rd_A.plane);
    reauthor_lateral(face_B, e_tan_B, *rd_B.plane);
    reauthor_cap(cap_lo, v_lo, e_arc_lo);
    reauthor_cap(cap_hi, v_hi, e_arc_hi);
    for (std::size_t f = 0; f < F0; ++f) {
        if (f == face_A || f == face_B || f == cap_lo ||
            f == cap_hi)
            continue;
        const Face& src = body.solid.shell().faces()[f];
        fillet_detail::PlaneReader rd;
        src.surface()->accept(rd);
        Face face(std::make_shared<PlaneSurface>(*rd.plane));
        for (const auto& loop : src.loops()) {
            Loop nl;
            nl.set_outer(loop.is_outer());
            for (const auto& ce : loop.coedges()) {
                const std::size_t mapped = emap[ce.edge()];
                if (mapped == E0)
                    refuse("the dead column's edge appeared in a "
                           "face outside the re-authored quartet - "
                           "report this as a kernel bug.");
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
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= f) faces_mut.resize(f + 1);
        faces_mut[f] = std::move(face);
    }

    // ---- The flank is born: the CylinderZSurface quarter window,
    // the walk CCW in the domain (the material — the filleted body's
    // interior — on the left; the outward +radial, the frame rule
    // (∂u×∂v)·n_out > 0 by construction, u_sign = +1). The walk is
    // the rectangle (u0,0) -> (u1,0) -> (u1,ell) -> (u0,ell) — the
    // low arc, the first tangent (at u1), the high arc, the second
    // tangent (at u0) — every p-curve a Line2 in the cylinder's own
    // domain: the flank patch IS the rectangle-window class the mass
    // machinery already integrates.
    {
        auto surf = std::make_shared<CylinderZSurface>(
            center, radius, z_lo, /*u_sign=*/+1);
        const std::size_t v_u0_lo = b_at_u0 ? v_TB_lo : v_TA_lo;
        const std::size_t v_u0_hi = b_at_u0 ? v_TB_hi : v_TA_hi;
        const std::size_t v_u1_lo = b_at_u0 ? v_TA_lo : v_TB_lo;
        const std::size_t v_u1_hi = b_at_u0 ? v_TA_hi : v_TB_hi;
        const std::size_t e_u1_tan =
            b_at_u0 ? e_tan_A : e_tan_B;
        const std::size_t e_u0_tan =
            b_at_u0 ? e_tan_B : e_tan_A;
        Loop loop;
        const auto emit_flank = [&](std::size_t e_id,
                                    std::size_t a, std::size_t b,
                                    double ua, double va,
                                    double ub, double vb) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the flank's walk lost the chaining - report "
                       "this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                math::Vec2d{ua, va}, math::Vec2d{ub, vb},
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        emit_flank(e_arc_lo, v_u0_lo, v_u1_lo, u0, 0.0, u1, 0.0);
        emit_flank(e_u1_tan, v_u1_lo, v_u1_hi, u1, 0.0, u1, ell);
        emit_flank(e_arc_hi, v_u1_hi, v_u0_hi, u1, ell, u0, ell);
        emit_flank(e_u0_tan, v_u0_hi, v_u0_lo, u0, ell, u0, 0.0);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0));
        face.set_provenance(prov);
        // The flank's name: fil#F:flank[<A> × <B>] — the operands
        // the referenced edge's adjacent faces, in the co-edge
        // order; the second binding of the isect rows below is the
        // SAME feature's flank (the same-feature extension, born
        // earlier in this walk).
        if (naming.body != nullptr &&
            src_A.shape_id().subshape.depth > 0 &&
            src_B.shape_id().subshape.depth > 0) {
            naming::OperandBinding fil_operands[2] = {
                {src_A.shape_id().feature, naming.body},
                {naming.feature, nullptr}};
            const naming::OperationNaming fil_decl{
                naming::FeatureKind::Fillet,
                naming::kFilletFaceRoles, naming::kFilletEdgeRoles,
                fil_operands};
            fil_operands[1].naming = &fil_decl;
            const auto leg = [](geometry::SubshapeRef ref,
                                const Face& operand) {
                for (std::uint8_t k = 0;
                     k < operand.shape_id().subshape.depth; ++k)
                    ref = ref.push(
                        operand.shape_id().subshape.path[k]);
                return ref;
            };
            geometry::SubshapeRef ref =
                geometry::SubshapeRef{}.push(naming::kFilFlankRole);
            ref = leg(ref, src_A);
            ref = leg(ref, src_B);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, fil_decl.faces()));
            // The tangent wires: fil#F:isect[<the landing face> ×
            // <the flank>] — the level ABSENT, the tangency IS the
            // level. The landing face of each tangent line is the
            // adjacent face it retires.
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {{e_tan_A, &src_A}, {e_tan_B, &src_B}};
            for (const auto& w : wires) {
                // The wire's path: the isect role, the LANDING
                // face's leg, then the FLANK's own role and its
                // two legs (the same-feature operand renders as a
                // full nested role — the renderer consumes the
                // flank's index, then its slots' operands).
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kFilFlankRole);
                wref = leg(wref, src_A);
                wref = leg(wref, src_B);
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(naming::render_name(
                    naming.feature, wref, fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE OUTPUT GATE (D2's identities, hard postconditions).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "fillet_edge: postcondition failed - a loop does not "
            "chain (an assembly defect in the fillet's re-authored "
            "walks; report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "fillet_edge: postcondition failed - the closed-"
            "orientable contract broke (every edge exactly two "
            "opposite co-edges; report this as a kernel bug)");
    if (R->vertex_count() != V0 - 2 + 4 ||
        R->edge_count() != E0 - 1 + 4 ||
        R->face_count() != F0 + 1)
        throw std::runtime_error(
            "fillet_edge: postcondition failed - the counts "
            "moved (the class's own arithmetic is V-2+4 / E-1+4 / "
            "F+1; report this as a kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(body))
        throw std::runtime_error(
            "fillet_edge: postcondition failed - chi moved (the "
            "fillet is chi-flat by construction, Delta-chi = "
            "+2 -3 +1 = 0; report this as a kernel bug)");
    return R;
}

} // namespace CAD_0::brep
