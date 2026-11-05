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
#include "CAD_0/brep/query.hpp"

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
// declarations (its role tables — the flank's operands render through
// them). The body's feature id flows from the body's own stamps (the
// found edge's creator); the tables are the caller's context, the
// exact_subtract precedent. The unnamed mode (body == nullptr, the
// default) leaves the born entities honestly empty — the inheritance
// (the shrink faces' names) flows in both modes (a stamp copy, no
// table involved).
//
// THE SECOND CREATOR (G2-u3 m3, the rim class): the blind body has
// TWO face creators — its box faces (box#1, the box tables) and its
// hole wall (ext#3, the extrude tables) — so the context gains the
// far pair: a leg whose creating feature id equals far_feature
// renders through far_body, every other leg through body. The box
// pure (m2) leaves far default{}: one table for every leg, the m2
// semantics untouched (additive, the m2 call sites unchanged).
struct FilletNaming {
    geometry::FeatureId feature{};
    const naming::OperationNaming* body{};
    geometry::FeatureId far_feature{};
    const naming::OperationNaming* far_body{};
};

// ---------------------------------------------------------------------------
// THE RECIPE — G2-u3 m4 (ADR-0039 D6 rung 4, the m4 rung's own words:
// "the FilletRecipe (the edge-name field, the radius)"): the record the
// FILE carries, named fields, never a raw struct — exactly two, nothing
// else: the edge-reference as the STRING (the grammar's own form,
// VERBATIM — "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", the pinned
// string ADR-0038 D3 wrote and ADR-0039 D3 registered as the recipe's
// field: never an address, never a dump; the m2/m3 kernel surfaces take
// the structural ShapeId, the recipe's string field lands HERE), and
// the radius as its own TYPED field (the constant ball radius r — the
// value class is the loader's, through the kernel factories, the same
// discipline the sketch constraints pay at their door). MAI un
// indirizzo, MAI un dump di struct: the padding does not exist in the
// file, and neither does the pointer — a raw image of this record
// would carry the string's heap address (the ABI), not its characters
// (the m4 padding counterproof's own subject).
// ---------------------------------------------------------------------------
struct FilletRecipe {
    std::string edge_reference{};  // the grammar's form, character for character
    double radius{0.0};            // the ball radius r (> 0, finite)
};

// fillet_reference_find — THE STRING'S OWN DOOR (G2-u3 m4): the
// recipe's edge-reference resolved against a body's stamps by RENDER
// equality — the whole string matched against the entity's own name,
// never parsed, never an address. The seam-wired pair (ADR-0023) is
// ONE logical name: the find returns the shared structural id, the
// pair taken whole (the m3 register's own rule). LOUD on the miss,
// with the remedy: the string named by name, the body's own edges
// offered as the candidates (never a mute "not found" — the m4
// re-run's class-4 answer is the RESOLVER's, above this door).
inline geometry::ShapeId fillet_reference_find(
    const BRepBody& body, const std::string& edge_reference) {
    for (EdgeId i = 0; i < body.edge_count(); ++i) {
        const EdgeInfo info = query_edge(body, i);
        if (!info.name.empty() && info.name == edge_reference)
            return info.id;
    }
    throw std::invalid_argument(
        "fillet_reference_find: the edge-reference '" + edge_reference +
        "' names no edge of this body - the reference is the grammar's "
        "own form, matched whole against the entity names (never "
        "parsed, never an address). Remedy: pick the edge from the "
        "body's own query (the by-name surface), or re-run against "
        "the body the reference was authored on.");
}

namespace fillet_detail {

// The plane payload reader (the typed dispatch — no dynamic_cast
// chains in the exact path).
struct PlaneReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    void visit(const PlaneSurface& p) override { plane = &p; }
};

// The Z-axis cylinder reader (the m3 rim class's wall).
struct WallReader : SurfaceVisitor {
    const CylinderZSurface* wall{nullptr};
    void visit(const CylinderZSurface& w) override { wall = &w; }
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

// fillet_rim_edge — THE FILLET, CLASS 2: IL RIM CIRCOLARE DEL FORO
// (ADR-0039 D1.2, G2-u3 m3): the rolling ball on the blind hole's
// top rim (plane x cylinder perpendicular, the CONVEX corner), the
// toroidal flank born in the rim's place. The same discipline as
// fillet_edge one class over: every quantity a CLOSED FORM of the
// stored operands, ZERO intersection solves — the ball's center
// rides the circle of radius a + r at depth r below the rim plane,
// the swept envelope the TorusZ quarter patch v in [pi/2, pi], the
// contact circles placed exactly (radius a + r at the rim quota on
// the pierced face; radius a at depth r on the wall) — the G2-u1
// table's toroidal rows (9/10: plane x torus, cylinder x torus
// coaxial) consumed as the VOCABULARY the payloads read (the SSI
// entries' own typing: Circle2/Line2 v = const), never run as
// solves: the table's surface unchanged, the register's own words.
// The epsilon_line sentinel stays asleep here too: nothing
// transversal is solved, the Layer-2 bound is exact/0.
//
// THE ADMISSION BEFORE THE CONSTRUCTION (the u2 signature, never a
// runtime geometry test): the body carries the BLIND DOOR's own —
// 12/17/8 with H = 1, chi = 2, closed orientable, every entity
// exact; the ONE pierced face (the inner loop) is a plane whose
// outward reads +Z (the guide orientation; the mirror family is
// registered for its first consumer); the ONE cylindrical face is
// the hole's wall (CylinderZ, the cavity frame u_sign = -1). Within
// that signature the referenced rim is CONVEX BY CONSTRUCTION; the
// floor rim (the carried cap x the wall) is the CONCAVE family's
// instance — refused by name with the queue's own entry, never
// sedated. LE BANDE TIPIZZATE (the class's own, the Linear world's
// epsilon_L): the tangent circle at a + r strictly inside the
// pierced face (r < the face's reach from the axis, minus a, minus
// epsilon_L; beyond: fillet_exits_annulus, the bound carried) and
// the tangency on the wall above the floor (r < the wall's height
// minus epsilon_L; beyond: fillet_eats_wall). The guide case
// r = 0.25 < a = 0.5: wide margin, NO degenerate seated — the
// r -> 0 and r -> a families stay MURO with their registered
// dispositions (0033 D4), and the vertex blend keeps its refused
// tail (never met, never named by this class's payload).
//
// THE ASSEMBLY IS THE DECOUPLING RULE'S (the m2 law, one family
// over): the pierced face keeps its NATIVE frame and its name
// (box#1:top) with the inner loop RE-AUTHORED at the contact circle
// (the blind door's own convention, the region bits carried); the
// wall keeps its NATIVE cavity frame and its name (ext#3:lat(0))
// with the window RE-TIMED to [floor, rim - r] and the seam
// RE-WIRED (the crossed corner's re-spanning); the flank is born
// with its own right-handed frame, the periodic seam riding its
// single loop TWICE (the wall's own pattern, ADR-0023). The counts:
// 14/20/9 with H = 1, chi = 2; the census 9/22/16 out+vanished; the
// co-edge census 20 x 2 = 40, every edge exactly two opposite
// co-edges — the contact circles walked OPPOSITE by their landing
// faces and the flank (the RACCOLTO's azimuth XOR).
inline std::unique_ptr<BRepBody> fillet_rim_edge(
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


// ---------------------------------------------------------------------------
// fillet_rim_edge — the implementation (the m2 pattern, one family over)
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> fillet_rim_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double radius, const FilletNaming& naming, double tol) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("fillet_rim_edge: " + what);
    };

    // ---- The radius's own gate (a non-positive radius has no ball).
    if (!(radius > 0.0) || !std::isfinite(radius))
        refuse("the radius must be finite and > 0, got " +
               std::to_string(radius) +
               " - a non-positive ball has no envelope and no contact "
               "circles. Remedy: author a positive radius within the "
               "class's bands (the admission derives).");

    // ---- The class signature: THE BLIND DOOR's own (the admission
    // BEFORE the construction - the u2 counts, zero convexity signs,
    // zero dihedral arithmetic).
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
                   "(the factories, the booleans), or accept the "
                   "faceted path.");
        if (V0 != 12 || E0 != 17 || F0 != 8 ||
            inner_loop_count(body) != 1 ||
            euler_characteristic(body) != 2 ||
            !is_closed_orientable(body))
            refuse("the body is outside the class signature (the "
                   "blind door's own: 12 vertices, 17 edges, 8 faces, "
                   "H = 1, chi = 2, closed orientable; found " +
                   std::to_string(V0) + "/" + std::to_string(E0) + "/" +
                   std::to_string(F0) +
                   ") - the v1 admission is the blind hole's convex "
                   "TOP rim (ADR-0039 D1.2, the u2 guide body). "
                   "Remedy: fillet the blind body's top rim.");
    }

    // ---- The signature's own two: the pierced face (the inner loop,
    // a +Z plane) and the wall (the single CylinderZ cavity frame).
    std::size_t pierced = F0, wall = F0;
    {
        const auto& faces = body.solid.shell().faces();
        for (std::size_t f = 0; f < F0; ++f) {
            const auto* surf = faces[f].surface();
            if (surf == nullptr)
                refuse("a face carries no surface - report this as a "
                       "kernel bug.");
            if (faces[f].loops().size() > 1) {
                if (pierced != F0)
                    refuse("two faces carry inner loops - the class "
                           "signature is the blind door's (exactly one "
                           "pierced face). Remedy: report the body's "
                           "construction.");
                pierced = f;
            }
            fillet_detail::WallReader wr;
            surf->accept(wr);
            if (wr.wall != nullptr) {
                if (wall != F0)
                    refuse("two cylindrical faces - the class "
                           "signature is the blind door's (exactly one "
                           "hole wall). Remedy: report the body's "
                           "construction.");
                wall = f;
            }
        }
        if (pierced == F0 || wall == F0)
            refuse("the signature's own two were not found (the "
                   "pierced face, the wall) - report this as a kernel "
                   "bug.");
        const auto& pf = body.solid.shell().faces()[pierced];
        fillet_detail::PlaneReader pr;
        pf.surface()->accept(pr);
        if (pr.plane == nullptr)
            refuse("the pierced face is not planar - the admission "
                   "(a) takes it in the table's world. Remedy: report "
                   "the body's construction.");
        if (pr.plane->normal_vec().z < 0.9)
            refuse("the pierced face's outward is not the guide "
                   "orientation (+Z; it reads " +
                   std::to_string(pr.plane->normal_vec().z) +
                   ") - the MIRROR family is registered for its "
                   "first consumer (ADR-0039 D1.2's own scope), never "
                   "smuggled. Remedy: fillet the body in the guide "
                   "orientation, or await the mirror morse.");
        if (pf.loops().size() != 2)
            refuse("the pierced face carries " +
                   std::to_string(pf.loops().size()) +
                   " loops - the blind door's own are two (the outer "
                   "box rim, the inner hole rim). Remedy: report the "
                   "body's construction.");
    }
    fillet_detail::WallReader wall_rd;
    body.solid.shell().faces()[wall].surface()->accept(wall_rd);
    if (wall_rd.wall == nullptr || wall_rd.wall->u_sign() != -1.0)
        refuse("the wall is not the cavity frame (CylinderZ, "
               "u_sign = -1) - the blind door's own authoring. "
               "Remedy: report the body's construction.");
    const CylinderZSurface& the_wall = *wall_rd.wall;
    const double a = the_wall.radius();
    const double cx = the_wall.axis_center().x;
    const double cy = the_wall.axis_center().y;
    const double z_floor = the_wall.z_base();

    // ---- The reference's own find: the structural match against the
    // body's stamps. The rim is the seam-wired ARC PAIR: ONE logical
    // name, TWO materialized edges (the morse-5 register's own rule:
    // the consumer that wants the rim takes the pair whole).
    std::vector<std::size_t> found;
    for (std::size_t i = 0; i < E0; ++i) {
        const auto& id = body.edges[i].shape_id();
        if (id.subshape.depth > 0 &&
            id.feature == edge_reference.feature &&
            id.subshape == edge_reference.subshape)
            found.push_back(i);
    }
    if (found.empty())
        refuse("the edge reference names no edge of this body (the "
               "structural find against the stamps found no match) - a "
               "reference that finds no row at all is a closure "
               "violation, not a user error: resolve the reference "
               "against the generation it was made on (the resolver's "
               "own guard), or re-pick the rim on this body.");
    if (found.size() == 1)
        refuse("the edge reference names a SINGLE edge (" +
               body.edges[found[0]].name() +
               ") - a fillet asked on a seam is refused with the edge "
               "named (ADR-0033 D3's own boundary). Remedy: reference "
               "the RIM (the seam-wired arc pair, e.g. the pierced "
               "rim's isect@hi), never the seam.");
    if (found.size() != 2)
        refuse("the edge reference materializes as " +
               std::to_string(found.size()) +
               " edges - the closure culture's own refusal. Remedy: "
               "report this as a kernel bug.");
    const std::size_t e_a = found[0], e_b = found[1];
    // The pair's joint check: the two arcs share their two endpoints
    // (the seam pair of the periodic circle). The joints carry the
    // rim's quota (level, above the floor).
    double z_rim = 0.0;
    VertexId j0 = 0, j1 = 0;
    {
        const auto shares = [&](VertexId v) {
            const bool in_a = body.edges[e_a].vertex(0) == v ||
                              body.edges[e_a].vertex(1) == v;
            const bool in_b = body.edges[e_b].vertex(0) == v ||
                              body.edges[e_b].vertex(1) == v;
            return in_a && in_b;
        };
        const VertexId cands[4] = {
            body.edges[e_a].vertex(0), body.edges[e_a].vertex(1),
            body.edges[e_b].vertex(0), body.edges[e_b].vertex(1)};
        std::vector<VertexId> joints;
        for (const VertexId v : cands) {
            if (!shares(v)) continue;
            bool seen = false;
            for (const VertexId u : joints)
                if (u == v) seen = true;
            if (!seen) joints.push_back(v);
        }
        if (joints.size() != 2)
            refuse("the two named edges do not share their joint "
                   "pair - not a seam-wired arc pair. Remedy: report "
                   "this as a kernel bug.");
        const math::Vec3d& pj = body.vertices[static_cast<std::size_t>(
                                     joints[0])]
                                     .position();
        const math::Vec3d& pk = body.vertices[static_cast<std::size_t>(
                                     joints[1])]
                                     .position();
        if (std::abs(pj.z - pk.z) > tol)
            refuse("the rim's joints are not level (the blind door's "
                   "own authoring) - report this as a kernel bug.");
        z_rim = pj.z;
        j0 = joints[0];
        j1 = joints[1];
    }

    // ---- The class's own gate: the two adjacent faces must be THE
    // pierced face and the wall (the CONVEX rim). Anything else is
    // the concave family's instance (the floor rim) - refused by
    // name with the queue's entry, never sedated.
    {
        const auto face_of_coedge = [&](const CoEdge* ce) {
            for (std::size_t f = 0; f < F0; ++f)
                for (const auto& loop :
                     body.solid.shell().faces()[f].loops())
                    for (const auto& other : loop.coedges())
                        if (&other == ce) return f;
            return F0;
        };
        const auto& dead = body.edges[e_a].coedges();
        if (dead.size() != 2)
            refuse("the referenced rim carries " +
                   std::to_string(dead.size()) +
                   " co-edges - the manifold class admits exactly two. "
                   "Remedy: report the body's construction.");
        const std::size_t f0 = face_of_coedge(dead[0]);
        const std::size_t f1 = face_of_coedge(dead[1]);
        const bool is_convex = (f0 == pierced && f1 == wall) ||
                                (f0 == wall && f1 == pierced);
        if (!is_convex)
            refuse("the referenced rim is not the convex top rim "
                   "(its adjacent faces are " +
                   body.solid.shell().faces()[f0].name() + " and " +
                   body.solid.shell().faces()[f1].name() +
                   ") - this is the CONCAVE family's instance "
                   "(fillet_out_of_class_pair: the reentrant corner, "
                   "the ball would roll on the void side and ADD "
                   "material - the inverted flank, ADR-0039 D1.3 "
                   "quoting ADR-0033 D4). The class waits in the queue "
                   "with its own unit, no date, its near-band "
                   "conditioning (r -> a) registered beside it. "
                   "Remedy: fillet the TOP rim (the pierced face's "
                   "own), or await the concave morse.");
    }

    // ---- LE BANDE TIPIZZATE (the class's own, the Linear world's
    // epsilon_L, the bounds named in the refusals): the tangent
    // circle at a + r strictly inside the pierced face (the reach
    // read from the face's own outer loop - the minimum distance
    // from the axis point, at the rim quota, to the loop's segments),
    // and the tangency on the wall above the floor.
    double reach = std::numeric_limits<double>::infinity();
    {
        const math::Vec3d P0{cx, cy, z_rim};
        for (const auto& ce :
             body.solid.shell().faces()[pierced].loops()[0].coedges()) {
            const Edge& e = body.edges[ce.edge()];
            const math::Vec3d& A = body.vertices[static_cast<std::size_t>(
                                        e.vertex(0))]
                                        .position();
            const math::Vec3d& B = body.vertices[static_cast<std::size_t>(
                                        e.vertex(1))]
                                        .position();
            const math::Vec3d AB = B - A;
            const double L2 = AB.dot(AB);
            double t = 0.0;
            if (L2 > 0.0) t = (P0 - A).dot(AB) / L2;
            t = std::max(0.0, std::min(1.0, t));
            const math::Vec3d Q = A + AB * t;
            reach = std::min(reach, std::sqrt((P0 - Q).dot(P0 - Q)));
        }
    }
    {
        const double bound_annulus = reach - a;
        const double bound_wall = z_rim - z_floor;
        const auto band_refusal = [](const char* what,
                                     const char* member, double bound,
                                     double the_radius, double the_tol) {
            const bool annulus =
                std::string(member) == "fillet_exits_annulus";
            return std::string(what) + " (" + std::string(member) +
                   "): the radius " + std::to_string(the_radius) +
                   " against the typed band's bound " +
                   std::to_string(bound) + " - " +
                   std::to_string(the_tol) +
                   " (the class's own reach minus the declared Linear "
                   "tolerance): the tangency would sit outside the " +
                   std::string(annulus ? "pierced face's annulus"
                                       : "wall's span") +
                   ", and the closed forms would place it outside the "
                   "material. Remedy: shrink the radius below " +
                   std::to_string(bound - the_tol) +
                   ", or restructure the body (the band is the class's "
                   "own, ADR-0039 D2's derived decision).";
        };
        if (radius >= bound_annulus - tol)
            refuse(band_refusal("the tangent circle exits the pierced face",
                                "fillet_exits_annulus", bound_annulus,
                                radius, tol));
        if (radius >= bound_wall - tol)
            refuse(band_refusal("the tangency passes the far end of the wall",
                                "fillet_eats_wall", bound_wall, radius, tol));
    }

    // ---- THE CONSTRUCTION - every quantity a closed form of the
    // stored operands, ZERO solves (D2's contract, one family over).
    const double R = a + radius;           // the tube's center radius
    const double z_c = z_rim - radius;     // the tube's center quota
    const double h_prime = z_c - z_floor;  // the wall's re-timed span
    // The seam meridian's domain coordinate in the wall's cavity
    // frame: f0 = -psi0, psi0 the +X joint's azimuth about the axis.
    double f0 = -std::atan2(0.0, 1.0);
    if (f0 == 0.0) f0 = 0.0;  // normalize -0.0
    const auto line_rr = [](math::Vec2d p, math::Vec2d q) {
        return pcurve_segment(p, q, /*interior_left=*/false);
    };

    // ---- The wall's structural pieces, read from its own loop: the
    // seam (the edge carried TWICE - the periodic identification's
    // signature, ADR-0023) and the floor pair (the two co-edges
    // between the seam's descending copy and its ascending one - the
    // blind door's own order).
    std::size_t e_wseam = E0;       // the seam's OLD id
    std::size_t e_floor_p = E0;     // the +Y floor half (bot1)
    std::size_t e_floor_m = E0;     // the -Y floor half (bot0)
    {
        const auto& ces =
            body.solid.shell().faces()[wall].loops()[0].coedges();
        const std::size_t n = ces.size();
        // The two indices of the seam's copies.
        std::size_t i1 = n, i2 = n;
        for (std::size_t k = 0; k < n; ++k) {
            std::size_t count = 0;
            for (std::size_t q = 0; q < n; ++q)
                if (ces[q].edge() == ces[k].edge()) ++count;
            if (count != 2) continue;  // not the seam
            if (i1 == n) {
                i1 = k;
            } else if (ces[k].edge() == ces[i1].edge()) {
                i2 = k;
                break;
            }
        }
        if (i2 == n) {
            // No edge carried twice: not the seam-wired layout.
        } else {
            e_wseam = ces[i1].edge();
            // From EACH seam copy, walk forward to the other copy:
            // the two co-edges between them are the floor pair (after
            // s_down, the blind door's order) or the dead rim pair
            // (after s_up). The floor pair is the one with NEITHER
            // dead edge, in walk order.
            for (const std::size_t start : {i1, i2}) {
                std::vector<const CoEdge*> between;
                for (std::size_t step = 1; step < n; ++step) {
                    const CoEdge& c = ces[(start + step) % n];
                    if (c.edge() == e_wseam) break;
                    between.push_back(&c);
                }
                if (between.size() == 2 &&
                    between[0]->edge() != e_a &&
                    between[0]->edge() != e_b &&
                    between[1]->edge() != e_a &&
                    between[1]->edge() != e_b) {
                    e_floor_p = between[0]->edge();
                    e_floor_m = between[1]->edge();
                    break;
                }
            }
        }
        if (e_wseam == E0 || e_wseam == e_a || e_wseam == e_b ||
            e_floor_p == E0 || e_floor_m == E0)
            refuse("the wall's own pieces were not found (the seam, "
                   "the floor pair) - report this as a kernel bug.");
    }

    // ---- The maps: the survivors compact (the dead pair dropped),
    // the born append. The dead rim's two joints vanish; its two
    // arcs are the dead columns.
    std::vector<std::size_t> vmap(V0, V0), emap(E0, E0);
    {
        std::size_t next = 0;
        for (std::size_t i = 0; i < V0; ++i)
            if (i != j0 && i != j1) vmap[i] = next++;
        next = 0;
        for (std::size_t i = 0; i < E0; ++i)
            if (i != e_a && i != e_b) emap[i] = next++;
    }
    // The born numbering (the construction's own tables, frozen by
    // the tests): 2i/2i+1 = joint i at the contact-parete (low) / at
    // the contact-piano (high) - the blind door's own layout.
    const std::size_t v_base = V0 - 2;
    const std::size_t v_S_lo = v_base + 0, v_S_hi = v_base + 1;
    const std::size_t v_A_lo = v_base + 2, v_A_hi = v_base + 3;
    const std::size_t e_base = E0 - 2;
    const std::size_t e_cp0 = e_base + 0;   // the -Y half, piano
    const std::size_t e_cp1 = e_base + 1;   // the +Y half, piano
    const std::size_t e_cw0 = e_base + 2;   // the -Y half, parete
    const std::size_t e_cw1 = e_base + 3;   // the +Y half, parete
    const std::size_t e_fseam = e_base + 4; // the flank's seam
    const std::size_t e_seam = emap[e_wseam];

    auto Rr = std::make_unique<BRepBody>();
    auto& shell = Rr->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- The vertices: the survivors (the copies carry the records),
    // then the born joint pairs (S at the parete/piano, then A).
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        Rr->vertices.push_back(body.vertices[i]);
        Rr->vertices.back().set_id(static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[4] = {
            {cx + a, cy, z_c}, {cx + R, cy, z_rim},
            {cx - a, cy, z_c}, {cx - R, cy, z_rim}};
        const std::size_t ids[4] = {v_S_lo, v_S_hi, v_A_lo, v_A_hi};
        for (std::size_t i = 0; i < 4; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            Rr->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the wall's seam RE-SPANNED to the
    // new high joint), then the born five.
    for (std::size_t i = 0; i < E0; ++i) {
        if (emap[i] == E0) continue;
        Rr->edges.push_back(body.edges[i]);
        auto& e = Rr->edges.back();
        e.set_id(static_cast<EdgeId>(emap[i]));
        // The kept copies remap through the compaction (the m2
        // discipline); the seam's HIGH endpoint is the dead rim's
        // joint - it has no image, and re-points at the parete
        // contact's joint (the LOW one, the floor's own, is kept).
        for (const int side : {0, 1}) {
            const std::size_t old_v = static_cast<std::size_t>(
                e.vertex(side));
            if (old_v == j0 || old_v == j1) continue;  // the seam's
            e.set_vertex(side, static_cast<VertexId>(vmap[old_v]));
        }
        if (i == e_wseam) {
            const VertexId lo = e.vertex(0), hi = e.vertex(1);
            const bool lo_first =
                body.vertices[static_cast<std::size_t>(lo)].position().z <
                body.vertices[static_cast<std::size_t>(hi)].position().z;
            if (lo_first)
                e.set_vertex(1, static_cast<VertexId>(v_S_lo));
            else
                e.set_vertex(0, static_cast<VertexId>(v_S_lo));
        }
    }
    {
        const std::pair<std::size_t, std::size_t> born[5] = {
            {v_A_hi, v_S_hi}, {v_S_hi, v_A_hi},
            {v_A_lo, v_S_lo}, {v_S_lo, v_A_lo},
            {v_S_hi, v_S_lo}};
        for (const auto& [x, y] : born) {
            Edge e{static_cast<VertexId>(x), static_cast<VertexId>(y)};
            e.set_id(static_cast<EdgeId>(Rr->edges.size()));
            e.set_provenance(prov);
            Rr->edges.push_back(e);
        }
    }

    // ---- THE WALL (absorbed, the window re-timed): its native cavity
    // frame kept bit-identical, the walk the blind door's own with
    // the contact-parete pair replacing the dead rim pair at v = h'.
    {
        auto surf = std::make_shared<CylinderZSurface>(
            the_wall.axis_center(), the_wall.radius(),
            the_wall.z_base(), -1);
        Loop loop;
        {
            CoEdge s_down{static_cast<EdgeId>(e_seam), true};
            s_down.set_p_curve(line_rr({f0, h_prime}, {f0, 0.0}));
            loop.add_coedge(s_down);
        }
        {
            CoEdge bot1{static_cast<EdgeId>(emap[e_floor_p]), false};
            bot1.set_p_curve(line_rr({f0, 0.0}, {f0 - kPi, 0.0}));
            loop.add_coedge(bot1);
        }
        {
            CoEdge bot0{static_cast<EdgeId>(emap[e_floor_m]), false};
            bot0.set_p_curve(
                line_rr({f0 - kPi, 0.0}, {f0 - kTwoPi, 0.0}));
            loop.add_coedge(bot0);
        }
        {
            CoEdge s_up{static_cast<EdgeId>(e_seam), false};
            s_up.set_p_curve(
                line_rr({f0 - kTwoPi, 0.0}, {f0 - kTwoPi, h_prime}));
            loop.add_coedge(s_up);
        }
        {
            CoEdge t0{static_cast<EdgeId>(e_cw0), true};
            t0.set_p_curve(line_rr({f0 - kTwoPi, h_prime},
                                   {f0 - kPi, h_prime}));
            loop.add_coedge(t0);
        }
        {
            CoEdge t1{static_cast<EdgeId>(e_cw1), true};
            t1.set_p_curve(
                line_rr({f0 - kPi, h_prime}, {f0, h_prime}));
            loop.add_coedge(t1);
        }
        const Face& src = body.solid.shell().faces()[wall];
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(wall));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= wall) faces_mut.resize(wall + 1);
        faces_mut[wall] = std::move(face);
    }

    // ---- THE PIERCED FACE (absorbed, the inner loop re-authored at
    // the contact circle - the blind door's own convention: the arcs
    // hole = true, the material outside the profile).
    {
        const Face& src = body.solid.shell().faces()[pierced];
        fillet_detail::PlaneReader pr;
        src.surface()->accept(pr);
        const PlaneSurface& pl = *pr.plane;
        const auto dom = [&pl](const math::Vec3d& P) {
            return fillet_detail::dom_of(pl, P);
        };
        const math::Vec2d c2 = dom(math::Vec3d{cx, cy, z_rim});
        Face face(std::make_shared<PlaneSurface>(pl));
        {
            Loop outer;
            for (const auto& ce : src.loops()[0].coedges()) {
                const std::size_t mapped = emap[ce.edge()];
                if (mapped == E0)
                    refuse("the dead column's edge appeared in the "
                           "pierced face's outer loop - report this as "
                           "a kernel bug.");
                CoEdge nc{static_cast<EdgeId>(mapped), ce.reversed()};
                if (ce.p_curve() != nullptr) nc.set_p_curve(*ce.p_curve());
                outer.add_coedge(nc);
            }
            outer.set_outer(src.loops()[0].is_outer());
            face.add_loop(std::move(outer));
        }
        {
            Loop inner;
            inner.set_outer(false);
            const math::Vec3d S_hi{cx + R, cy, z_rim};
            const math::Vec3d A_hi{cx - R, cy, z_rim};
            {
                // cp0: the -Y half, A_hi -> S_hi: a SEMICIRCLE, sweep
                // pi (the blind door's own arcs), hole = true.
                CoEdge ce{static_cast<EdgeId>(e_cp0), false};
                const math::Vec2d s = dom(A_hi);
                ce.set_p_curve(pcurve_arc(
                    c2, R, /*hole=*/true,
                    std::atan2(s.y - c2.y, s.x - c2.x), kPi));
                inner.add_coedge(ce);
            }
            {
                CoEdge ce{static_cast<EdgeId>(e_cp1), false};
                const math::Vec2d s = dom(S_hi);
                ce.set_p_curve(pcurve_arc(
                    c2, R, /*hole=*/true,
                    std::atan2(s.y - c2.y, s.x - c2.x), kPi));
                inner.add_coedge(ce);
            }
            face.add_loop(std::move(inner));
        }
        face.set_id(static_cast<FaceId>(pierced));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= pierced) faces_mut.resize(pierced + 1);
        faces_mut[pierced] = std::move(face);
    }

    // ---- THE KEPT FACES: verbatim (the ids remapped through the
    // compaction - the blind door copies its operand verbatim).
    for (std::size_t f = 0; f < F0; ++f) {
        if (f == pierced || f == wall) continue;
        const Face& src = body.solid.shell().faces()[f];
        fillet_detail::PlaneReader pr;
        src.surface()->accept(pr);
        if (pr.plane == nullptr)
            refuse("a kept face is not planar - the class signature is "
                   "the blind door's (six planes, one cylinder). "
                   "Remedy: report the body's construction.");
        Face face(std::make_shared<PlaneSurface>(*pr.plane));
        for (const auto& loop : src.loops()) {
            Loop nl;
            nl.set_outer(loop.is_outer());
            for (const auto& ce : loop.coedges()) {
                const std::size_t mapped = emap[ce.edge()];
                if (mapped == E0)
                    refuse("the dead column's edge appeared in a kept "
                           "face - report this as a kernel bug.");
                CoEdge nc{static_cast<EdgeId>(mapped), ce.reversed()};
                if (ce.p_curve() != nullptr) nc.set_p_curve(*ce.p_curve());
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

    // ---- THE FLANK IS BORN: the TorusZ quarter window, the walk CCW
    // in the domain (the periodic seam riding the single loop TWICE -
    // the wall's own pattern, ADR-0023).
    {
        auto surf = std::make_shared<TorusZSurface>(
            math::Vec2d{cx, cy}, R, radius, z_c);
        const double v_lo = kPi / 2.0, v_hi = kPi;
        Loop loop;
        // The walk is REGION-ON-RIGHT (interior_left = false), the
        // wall's own authoring: the flank's material lies INSIDE the
        // tube - BEHIND the surface, like the wall's own - so with the
        // right-handed frame the region rides the RIGHT of the walk.
        // The rectangle runs (0, pi/2) UP the seam's meridian to
        // (0, pi), RIGHT along the wall contact to (2pi, pi), DOWN to
        // (2pi, pi/2), and LEFT along the plane contact home: every
        // contact co-edge thereby rides OPPOSITE its landing face's
        // copy (the RACCOLTO's azimuth XOR), and the seam rides the
        // single loop TWICE.
        {
            CoEdge ce{static_cast<EdgeId>(e_fseam), false};
            ce.set_p_curve(line_rr({0.0, v_lo}, {0.0, v_hi}));
            loop.add_coedge(ce);
        }
        {
            CoEdge ce{static_cast<EdgeId>(e_cw1), false};
            ce.set_p_curve(line_rr({0.0, v_hi}, {kPi, v_hi}));
            loop.add_coedge(ce);
        }
        {
            CoEdge ce{static_cast<EdgeId>(e_cw0), false};
            ce.set_p_curve(line_rr({kPi, v_hi}, {kTwoPi, v_hi}));
            loop.add_coedge(ce);
        }
        {
            CoEdge ce{static_cast<EdgeId>(e_fseam), true};
            ce.set_p_curve(line_rr({kTwoPi, v_hi}, {kTwoPi, v_lo}));
            loop.add_coedge(ce);
        }
        {
            CoEdge ce{static_cast<EdgeId>(e_cp0), true};
            ce.set_p_curve(line_rr({kTwoPi, v_lo}, {kPi, v_lo}));
            loop.add_coedge(ce);
        }
        {
            CoEdge ce{static_cast<EdgeId>(e_cp1), true};
            ce.set_p_curve(line_rr({kPi, v_lo}, {0.0, v_lo}));
            loop.add_coedge(ce);
        }
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0));
        face.set_provenance(prov);
        // ---- THE NAMES (ADR-0039 D3, the m2 stamping one class over).
        if (naming.body != nullptr &&
            body.solid.shell().faces()[pierced].shape_id()
                    .subshape.depth > 0 &&
            body.solid.shell().faces()[wall].shape_id()
                    .subshape.depth > 0) {
            const Face& op_plane =
                body.solid.shell().faces()[pierced];
            const Face& op_wall = body.solid.shell().faces()[wall];
            // The second creator's rule: the leg whose feature equals
            // far_feature renders through far_body (m3), the others
            // through body (the m2 semantics).
            const auto table_of = [&](const Face& operand) {
                return (naming.far_body != nullptr &&
                        operand.shape_id().feature == naming.far_feature)
                           ? naming.far_body
                           : naming.body;
            };
            // THREE bindings (the subtract's intra-body pattern):
            // the pierced face (the box's tables), the wall (the
            // extrude's tables), the fillet itself (the flank's own
            // nested render) - the slots address them by index.
            naming::OperandBinding fil_operands[3] = {
                {op_plane.shape_id().feature, table_of(op_plane)},
                {op_wall.shape_id().feature, table_of(op_wall)},
                {naming.feature, nullptr}};
            const naming::OperationNaming fil_decl{
                naming::FeatureKind::Fillet,
                naming::kFilletFaceRoles, naming::kFilletEdgeRoles,
                fil_operands};
            fil_operands[2].naming = &fil_decl;
            const auto leg = [](geometry::SubshapeRef ref,
                                const Face& operand) {
                for (std::uint8_t k = 0;
                     k < operand.shape_id().subshape.depth; ++k)
                    ref = ref.push(
                        operand.shape_id().subshape.path[k]);
                return ref;
            };
            geometry::SubshapeRef ref =
                geometry::SubshapeRef{}.push(naming::kFilFlankRimRole);
            ref = leg(ref, op_plane);
            ref = leg(ref, op_wall);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, fil_decl.faces()));
            // The two contact circles (the tangent wires, the level
            // ABSENT) and the flank's own seam.
            const struct {
                std::size_t edge0, edge1;
                const Face* landing;
            } wires[2] = {
                {e_cp0, e_cp1, &op_plane},
                {e_cw0, e_cw1, &op_wall}};
            for (const auto& w : wires) {
                const bool on_wall = (w.landing == &op_wall);
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        on_wall ? naming::kFilIsectWallRole
                                : naming::kFilIsectPlaneRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kFilFlankRimRole);
                wref = leg(wref, op_plane);
                wref = leg(wref, op_wall);
                for (const std::size_t idx : {w.edge0, w.edge1}) {
                    Rr->edges[idx].set_shape_id(
                        {naming.feature, wref, {}});
                    Rr->edges[idx].set_name(naming::render_name(
                        naming.feature, wref, fil_decl.edges()));
                }
            }
            {
                geometry::SubshapeRef sref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilSeamRole);
                sref = sref.push(naming::kFilFlankRimRole);
                sref = leg(sref, op_plane);
                sref = leg(sref, op_wall);
                Rr->edges[e_fseam].set_shape_id(
                    {naming.feature, sref, {}});
                Rr->edges[e_fseam].set_name(naming::render_name(
                    naming.feature, sref, fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE OUTPUT GATE (the hard postconditions, the m2 set one
    // family over: the counts, the XOR, the chi, the H).
    link_radial_edges(*Rr);
    if (!loops_are_closed(*Rr))
        throw std::runtime_error(
            "fillet_rim_edge: postcondition failed - a loop does not "
            "chain (an assembly defect in the rim fillet's re-authored "
            "walks; report this as a kernel bug)");
    if (!is_closed_orientable(*Rr))
        throw std::runtime_error(
            "fillet_rim_edge: postcondition failed - the closed-"
            "orientable contract broke (every edge exactly two "
            "opposite co-edges; report this as a kernel bug)");
    for (const auto& e : Rr->edges) {
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            throw std::runtime_error(
                "fillet_rim_edge: postcondition failed - edge " +
                std::to_string(e.id()) +
                " is not manifold-with-XOR (an assembly defect; "
                "report this as a kernel bug)");
    }
    if (Rr->vertex_count() != V0 - 2 + 4 ||
        Rr->edge_count() != E0 - 2 + 5 ||
        Rr->face_count() != F0 + 1)
        throw std::runtime_error(
            "fillet_rim_edge: postcondition failed - the counts "
            "moved (the class's own arithmetic is V-2+4 / E-2+5 / "
            "F+1; report this as a kernel bug)");
    if (inner_loop_count(*Rr) != 1)
        throw std::runtime_error(
            "fillet_rim_edge: postcondition failed - the pierced face "
            "keeps exactly one inner loop (now the a+r circle; report "
            "this as a kernel bug)");
    if (euler_characteristic(*Rr) != euler_characteristic(body))
        throw std::runtime_error(
            "fillet_rim_edge: postcondition failed - chi moved (the "
            "fillet is chi-flat by construction; report this as a "
            "kernel bug)");
    return Rr;
}

} // namespace CAD_0::brep
