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
// runtime geometry test): the body carries THE BOX FAMILY CHAIN's
// structural signature — the factory box (k = 0) or a box carrying
// k previous VERTICAL fillets (V = 8+2k, E = 12+3k, F = 6+k, the
// F-6 non-planar faces CylinderZ flanks, the rest the box's six
// planes possibly shrunk), every face single-loop, closed,
// orientable, χ = 2, every entity exact (the Layer-2 gate: an
// approximated body never enters) — and the reference resolves to
// one of its VERTICAL edges (the endpoints' (x,y) equal within the
// Linear band; the previous links' KEPT STAMPS answer here — the
// chain consumes the current state, m6-correttiva-ter). Within
// that signature the edge is CONVEX 90° BY CONSTRUCTION (the
// admission (b) of ADR-0033 D3: the ball rolls on the material
// side, inscribed): no runtime dihedral test runs, no convexity
// sign is computed — the class's guarantee IS the admission, and
// the tree's producers (the factory and this function's own gated
// output) cannot produce a non-convex carrier of the signature.
// The two adjacent faces must be PLANAR (the admission (a): both
// in the table's world — the previous fillets' tangent wires,
// vertical but plane × cylinder, refuse here BY FAMILY) and
// perpendicular — their stored outward normals' dot reads exactly
// 0 (the axis-aligned signature's own arithmetic, not a tolerance
// decision; a skewed pair is the GENERAL DIHEDRAL family,
// registered for its first consumer — ADR-0039 D1.1's "never
// smuggled").
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

// ---------------------------------------------------------------------------
// THE MULTI RECIPE — G5 m5 (ADR-0043 D6.5, D9's fifth morso: "la ricetta
// multi-spigolo e la porta v0.8.0 (la mappa spigolo→raggio, i campi
// nominali, il migrator, i golden del formato)"): the many-edge record
// the FILE carries — the EDGE→RADIUS MAP, one entry per consumed spigolo,
// named fields, never a raw struct (the m4 padding doctrine one recipe
// over: a memcpy'd entry image would carry the string's heap address and
// the tail padding, the ABI, never the characters). The entry's two
// fields are the single recipe's own, verbatim: the edge-reference as
// the STRING (the grammar's form, matched whole, never parsed) and the
// radius its own TYPED field. THE LAW THE MAP OBEYS is not the struct's
// — it is the ANGLES LAW (D6.6/D1: the connected edge-corner graph
// forces one radius per component), read by the doors that carry the
// body (creation, the loader's pass 2, the save door's resolution) and
// NEVER by the kernel: the kernel's own admission (the body-state
// gates, fillet_ring_act's signature among them) stays the last line,
// its words unchanged.
// ---------------------------------------------------------------------------
struct MultiFilletEntry {
    std::string edge_reference{};  // the grammar's form, character for character
    double radius{0.0};            // the ball radius r (> 0, finite)
};

struct MultiFilletRecipe {
    // The consumed spigoli, one entry each, in the AUTHORED order (the
    // vertical passage replays in this order — the derivation's freedom
    // among the verticals, D5.1: "sequenziali o in un atto solo,
    // libertà dell'implementazione"; the ring's consumption is the ACT,
    // orderless by construction). The SAME reference twice is a loud
    // refusal at every door (the map's uniqueness — an edge consumed
    // twice is not a map).
    std::vector<MultiFilletEntry> entries{};
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

// The chain's flank reader (G5 m2, ADR-0043 D2.2/D9): the box family
// chain's curved faces are the fillet walks' own born flanks — the
// vertical walk's CylinderZ, this walk's CylinderX and CylinderY. One
// reader, three placements, the family ONE.
struct FlankReader : SurfaceVisitor {
    const CylinderZSurface* z{nullptr};
    const CylinderXSurface* x{nullptr};
    const CylinderYSurface* y{nullptr};
    void visit(const CylinderZSurface& s) override { z = &s; }
    void visit(const CylinderXSurface& s) override { x = &s; }
    void visit(const CylinderYSurface& s) override { y = &s; }
    bool any() const noexcept { return z != nullptr || x != nullptr ||
                                       y != nullptr; }
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

// fillet_ring_edge — THE RING FLANK (G5 m2, ADR-0043 D2.2/D9): the
// rolling ball on the box's RING edge (top/bottom x lateral — the
// horizontal spigolo, X- or Y-parallel), the CylinderX/CylinderY
// flanks' writer. The same discipline as fillet_edge ONE AXIS OVER
// ("gli spigoli del ring sono verticali NEL LORO frame — la stessa
// topologia, il piano di simmetria ruotato", D2.3): every quantity a
// CLOSED FORM of the stored operands, ZERO intersection solves — the
// ball's center on the bisector at r·cot(π/4) = r from the edge, the
// tangent lines placed exactly (on each adjacent face the line at
// distance r from the edge), the caps' quarter arcs centered at the
// axis's foot. The flank is born on the placement the edge names
// (X-parallel → CylinderXSurface, Y-parallel → CylinderYSurface —
// the m1 classes, right-handed by construction, the seam meridian at
// the window's edge, never crossed), the window the θ-quarter facing
// the corner × the edge's own span.
//
// THE ADMISSION: the box family chain (8+2k / 12+3k / 6+k, χ = 2,
// closed orientable, exact, single-loop faces), the curved faces the
// chain's flanks (CylinderZ, CylinderX or CylinderY — the three
// placements, the counts' k and the flanks' count agreeing). THE
// CORNER-FREEDOM WALL (D5.1/D6.1): both the edge's corners must be
// FREE — no previous fillet's flank bordering any edge incident to
// them; an occupied corner is refused LOUDLY with the occupying
// flank named, because the naked sequential there would refine
// cylinder against perpendicular cylinder — THE STEINMETZ QUARTIC
// the stage never consults — and the honest construction at an
// occupied corner is the CORNER ACT (the sphere tangent to the three
// flanks, the atomic decomposition), its own morsi's seat (D5.2,
// D9's morsi 3–4). LE BANDE TIPIZZATE ride the chain as the
// verticals' own (the shared face's reach TIGHTENED by the previous
// fillet's tangent line, the bound NAMED in the refusal).
//
// The counts by the same walk's arithmetic: V-2+4 / E-1+4 / F+1 —
// the golden 10/15/7 at k = 0 one axis over, the chain formula
// 8+2k/12+3k/6+k verified at k = 1, 2. The tavola is
// fillet_descent's own derivation (the same by-name diff, the
// vanished cause "replaced by the flank").
inline std::unique_ptr<BRepBody> fillet_ring_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double radius, const FilletNaming& naming = FilletNaming{},
    double tol = kFilletTolerance);

// fillet_ring_act — THE RING AND CORNERS ACT, INDIVISIBLE (G5 m4,
// ADR-0043 D5.1's second passage): the cube's eight ring edges and
// eight corner spheres consumed in ONE transaction, from the
// four-vertical state (16/24/10) to the fully rounded cube
// (24/48/26). The order of consumption is the derivation (D5.1):
// the verticals first, then this act — never the naked sequential.
inline std::unique_ptr<BRepBody> fillet_ring_act(
    const BRepBody& body, double radius,
    const FilletNaming& naming = FilletNaming{},
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

    // ---- The class signature: THE BOX FAMILY CHAIN's own (the
    // admission BEFORE the construction — the counts and the
    // families, zero dihedral arithmetic, zero convexity signs).
    // The m6-correttiva-ter widening: the carrier is the FACTORY
    // BOX (k = 0) or a box carrying k PREVIOUS VERTICAL FILLETS —
    // each link of this same walk added 2 vertices, 3 edges and 1
    // CylinderZ flank, so V = 8+2k, E = 12+3k, F = 6+k with the
    // SAME chi = 2, closed orientable, every entity exact, every
    // face single-loop; the F - 6 NON-planar faces must be
    // CylinderZ (the born flanks' own family — the only curved
    // faces the chain can produce; the counts' k and the flanks'
    // count must AGREE, the agreement itself the wall against the
    // counts-matching impostors: the orthogonal prism (all planes,
    // 0 flanks where k says 1) and the pierced carriers (inner
    // loops) refuse here). The convexity trust is the m2
    // admission's own: the producers of the signature are the
    // factory and THIS FUNCTION (its output gate re-verifies the
    // closure), and their planar-planar verticals are convex by
    // construction — the orthogonal prism joints stay REGISTERED
    // for their own carrier (ADR-0039 D1.1), never smuggled.
    std::size_t chain_k = 0;  // the previous vertical fillets
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
        const std::size_t kcand = F0 >= 6 ? F0 - 6
                                        : static_cast<std::size_t>(-1);
        const bool counts_match =
            kcand != static_cast<std::size_t>(-1) &&
            V0 == 8 + 2 * kcand && E0 == 12 + 3 * kcand;
        if (!counts_match ||
            euler_characteristic(body) != 2 || !is_closed_orientable(body))
            refuse("the body is outside the class signature (the "
                   "box family chain's own: 8+2k vertices, 12+3k "
                   "edges, 6+k faces for k previous vertical "
                   "fillets, chi = 2, closed orientable; found " +
                   std::to_string(V0) + "/" + std::to_string(E0) +
                   "/" + std::to_string(F0) +
                   ") - the v1 admission is the box family's convex "
                   "90-degree corners (ADR-0039 D1.1). Remedy: "
                   "fillet a factory box's vertical edge, or a "
                   "chain of them (the current state a previous "
                   "fillet left).");
        chain_k = kcand;
        std::size_t flanks = 0;
        for (const auto& f : body.solid.shell().faces()) {
            if (f.loops().size() != 1)
                refuse("a face carries " +
                       std::to_string(f.loops().size()) +
                       " loops - the class signature is the box "
                       "family's single-loop faces (a pierced or "
                       "notched face is a later family's carrier). "
                       "Remedy: fillet a factory box's vertical "
                       "edge.");
            const auto* surf = f.surface();
            fillet_detail::PlaneReader rd;
            const bool planar =
                surf != nullptr &&
                (surf->accept(rd), rd.plane != nullptr);
            if (!planar) {
                fillet_detail::WallReader wr;
                if (surf == nullptr ||
                    (surf->accept(wr), wr.wall == nullptr))
                    refuse("a face carries a surface outside the "
                           "class families (neither the box's plane "
                           "nor a previous fillet's CylinderZ "
                           "flank) - the chain's only curved faces "
                           "are the flanks this walk itself born. "
                           "Remedy: fillet a factory box's vertical "
                           "edge.");
                ++flanks;
            }
        }
        if (flanks != chain_k)
            refuse("the body carries " + std::to_string(flanks) +
                   " cylindrical faces where the chain's counts "
                   "declare " + std::to_string(chain_k) +
                   " - the signature's own arithmetic (the flanks "
                   "and the counts must agree; the orthogonal "
                   "prisms are REGISTERED for their own carrier, "
                   "ADR-0039 D1.1). Remedy: fillet a factory box's "
                   "vertical edge.");
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
        // The kept faces copy verbatim — the surface carried in its
        // OWN family (the m6-correttiva-ter chain: a previous
        // fillet's CylinderZ flank rides through here as its
        // native wall, value-copied exactly as the planes are —
        // never re-read through a plane reader). The admission
        // above already verified every face is a plane or a
        // CylinderZ flank, so the fallback is a kernel bug.
        std::shared_ptr<SurfaceGeometry> kept_surf;
        {
            fillet_detail::PlaneReader rd;
            src.surface()->accept(rd);
            if (rd.plane != nullptr)
                kept_surf = std::make_shared<PlaneSurface>(*rd.plane);
            else {
                fillet_detail::WallReader wr;
                src.surface()->accept(wr);
                if (wr.wall == nullptr)
                    refuse("a kept face's surface is outside the "
                           "admission's families - report this as a "
                           "kernel bug.");
                kept_surf =
                    std::make_shared<CylinderZSurface>(*wr.wall);
            }
        }
        Face face(std::move(kept_surf));
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
// fillet_ring_edge_corner_act — THE CORNER ACT (G5 m3, ADR-0043
// D5.1/D5.2/D6.2/D6.3/D9's third morso, its first half): the atomic
// decomposition LANDING at one corner. The trigger is the m2 wall's
// own seat — a ring fillet asked at a corner OCCUPIED by a previous
// VERTICAL fillet's CylinderZ flank — and the act is indivisible by
// construction (D5.1: "nessuno stato intermedio della decomposizione
// è chiuso"): the second flank at an occupied corner would have to
// refine cylinder against PERPENDICULAR cylinder (the Steinmetz
// quartic, D6.1's wall — never consulted, never solved), so the act
// consumes the corner WHOLE in one step:
//
//   * THE SPHERE is born (SphereSurface, the EXISTING family, a new
//     window): center C = the corner offset r inward along each of
//     the three planes' normals — the closed form of the operands;
//     radius r, EQUAL to the vertical flank's (the mismatch is
//     D6.6's own named refusal, fillet_corner_radii_mismatch — the
//     sphere tangent to three flanks exists only at the common
//     radius). The patch is the OTTANTO toward the corner: the
//     frame DERIVED (the constructor's own rule, no knob): pole =
//     the SHARED plane's outward normal (the plane both ring edges
//     lie on — the equator is then the vertical flank's coaxial
//     circle), azimuth = the outward normal of the first lateral
//     plane of the naming order [face_A, face_B, third] (the seam
//     meridian declared there — D5.2's own seat), second axis the
//     product. The window: the quarter u x [0, pi/2] with the apex
//     DEGENERATE at the pole (the tangency point on the shared
//     plane — the vertex where the two ring flanks' coaxial arcs
//     meet): 3 edges, 1 loop, the walk CCW in the domain (the
//     material on the left, the house law).
//   * THE THREE COAXIAL ARCS are born — the quarters of the three
//     contact circles (D3's own vocabulary, the LANDED
//     cylinder x sphere coassiale row consumed as GRAMMAR, never
//     run as a solve): the equator (plane through C perpendicular
//     to the vertical axis — the sphere x vertical-flank tangency,
//     Line2 v = const on the cylinder's own domain, Line2 v = pi/2
//     on the sphere's) and the two meridians (the planes
//     perpendicular to the ring axes — the sphere x ring-flank
//     tangencies). Every arc's endpoints are the TANGENCY POINTS on
//     the planes — D5.2's three vertices, placed exactly (the feet
//     of the perpendiculars from C), never solved.
//   * THE TWO RING FLANKS are born GIÀ RIFINITI (D5.2): the m2
//     walk's own closed forms one axis over, but the window's
//     corner end is the COAXIAL ARC (the contact circle's quarter),
//     never the full quarter span — the far end keeps the m2 cap
//     (the quarter arc on the far cap face), the corner end stops
//     at the sphere.
//   * THE VERTICAL FLANK is RI-FINESTRATO (D6.3): the cap arc at
//     the corner end DIES (replaced by the coaxial equator — the
//     ADR's own cause, "replaced by the corner sphere's tangent
//     arc") and the flank's window is re-authored in place (the two
//     tangent lines shortened to the coaxial plane, the top arc the
//     equator now) — the face SURVIVES with its name and its stamp:
//     the tavola's row reads Absorbed (the edit in place, the m6
//     rule generalized to revolution surfaces), NEVER a death and a
//     birth under the same name (the Born-sotto-falso the test
//     refuses).
//   * THE PLANES are re-authored at the tangent lines (D5.3): the
//     shared plane ends as the RECTANGLE of the two tangent lines
//     meeting at the sphere's tangency point (the ADR's own
//     closure: the dead cap arc's center IS the tangency point);
//     the laterals gain the flanks' tangent lines and the shortened
//     vertical tangent lines; the far caps gain the m2 quarter
//     arcs.
//
// THE COUNTS (the act's own arithmetic, the tavola's closure): on
// the k = 1 chain (10/15/7 — one vertical flank): V - 4 + 7, E - 3
// + 9, F + 3 -> 13/21/10, chi = 2 flat (the four dead vertices are
// the two far corners and the two corner-side tangent junctions;
// the three dead edges are the two ring edges and the corner's cap
// arc; the seven born vertices are the three tangency points and
// the two far tangent pairs; the nine born edges are the four
// tangent lines, the two far cap arcs and the three coaxial arcs;
// the three born faces are the two ring flanks and the sphere).
//
// THE WITNESS (D3's thesis, falsifiable): every quantity above is a
// CLOSED FORM of the stored operands — zero intersection solves,
// zero consults of the SSI table, zero new SSI lines (the quartic's
// wall stands: the flanks never meet each other, the sphere's arcs
// are authored at the tangency points, the windows stop at the
// coaxial planes). The diff touches no ssi.hpp/ssi.cpp line.
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> fillet_ring_edge_corner_act(
    const BRepBody& body, std::size_t edge, std::size_t v_occ,
    double radius, const FilletNaming& naming, double tol,
    std::size_t face_A, std::size_t face_B) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("fillet_ring_edge: " + what);
    };
    const auto vpos = [&](std::size_t v) -> const math::Vec3d& {
        return body.vertices[v].position();
    };
    const auto face_of_coedge = [&](const CoEdge* ce) -> std::size_t {
        for (std::size_t f = 0; f < F0; ++f)
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& other : loop.coedges())
                    if (&other == ce) return f;
        return F0;
    };
    const auto plane_of = [&](std::size_t f) -> const PlaneSurface* {
        fillet_detail::PlaneReader rd;
        const auto* s = body.solid.shell().faces()[f].surface();
        if (s != nullptr) s->accept(rd);
        return rd.plane;
    };

    // ---- The reference edge's own geometry (the walk's own
    // derivation, compact: the ring-ness, the adjacent planes, the
    // span — the trigger already verified the class admission, the
    // act re-reads what it consumes).
    const VertexId ev0 = body.edges[edge].vertex(0);
    const VertexId ev1 = body.edges[edge].vertex(1);
    const math::Vec3d& p0 = vpos(static_cast<std::size_t>(ev0));
    const math::Vec3d& p1 = vpos(static_cast<std::size_t>(ev1));
    const bool x_par =
        std::abs(p0.y - p1.y) <= tol && std::abs(p0.z - p1.z) <= tol;
    const bool y_par =
        std::abs(p0.x - p1.x) <= tol && std::abs(p0.z - p1.z) <= tol;
    if ((!x_par && !y_par) || (x_par && y_par))
        refuse("the corner act's reference lost its ring-ness (the "
               "axis-parallel test disagrees with the trigger's "
               "reading) - report this as a kernel bug.");
    const std::size_t v_far =
        static_cast<std::size_t>(ev0) == v_occ
            ? static_cast<std::size_t>(ev1)
            : static_cast<std::size_t>(ev0);
    if (static_cast<std::size_t>(ev0) != v_occ &&
        static_cast<std::size_t>(ev1) != v_occ)
        refuse("the corner act's occupied end is not the reference "
               "edge's endpoint - report this as a kernel bug.");
    const PlaneSurface* pl_A = plane_of(face_A);
    const PlaneSurface* pl_B = plane_of(face_B);
    if (pl_A == nullptr || pl_B == nullptr)
        refuse("the corner act's adjacent faces are not both planes "
               "- the trigger's own wall guarantees they are; report "
               "this as a kernel bug.");
    const math::Vec3d n_A = pl_A->normal_vec();
    const math::Vec3d n_B = pl_B->normal_vec();
    if (n_A.dot(n_B) != 0.0)
        refuse("the corner act's adjacent planes are not "
               "perpendicular - the class signature is the box "
               "family's. Remedy: report the body's construction.");

    // ---- THE OCCUPYING VERTICAL FLANK (the trigger found the
    // corner occupied; the act reads WHICH placement and derives
    // the corner's three planes from the body's own records).
    std::size_t vert_face = F0;
    const CylinderZSurface* vz = nullptr;
    for (std::size_t ei = 0; ei < E0 && vert_face == F0; ++ei) {
        if (ei == edge) continue;
        const auto& e = body.edges[ei];
        if (e.vertex(0) != static_cast<VertexId>(v_occ) &&
            e.vertex(1) != static_cast<VertexId>(v_occ))
            continue;
        for (const CoEdge* ce : e.coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (f >= F0) continue;
            const auto* s = body.solid.shell().faces()[f].surface();
            fillet_detail::PlaneReader rd;
            if (s != nullptr) s->accept(rd);
            if (s == nullptr || rd.plane == nullptr) {
                fillet_detail::FlankReader fr;
                s->accept(fr);
                if (fr.z == nullptr)
                    refuse("the corner is occupied by a RING flank "
                           "(CylinderX/CylinderY, not the vertical "
                           "placement) - the corner act takes the "
                           "vertical-first chain (the sphere tangent "
                           "to one vertical and two ring flanks, "
                           "D5.1's own decomposition); the "
                           "ring-occupied corner rides with the "
                           "twelve-spigoli morso (D9's second half). "
                           "Remedy: fillet the corner's vertical edge "
                           "first (fillet_edge), then the ring pair "
                           "fires the act.");
                vert_face = f;
                vz = fr.z;
                break;
            }
        }
    }
    if (vert_face == F0 || vz == nullptr)
        refuse("the corner act's occupying flank was not found at "
               "the occupied end - the trigger's own scan "
               "guarantees it; report this as a kernel bug.");
    // THE RADIUS MATCH (D6.6's own named wall): the sphere tangent
    // to all three flanks exists only at the common radius.
    if (vz->radius() != radius)
        refuse("the corner act's radii disagree "
               "(fillet_corner_radii_mismatch): the requested " +
               std::to_string(radius) +
               " against the vertical flank's " +
               std::to_string(vz->radius()) +
               " - the corner sphere is tangent to the three flanks "
               "only at their COMMON radius (the coaxial circles "
               "close the windows exactly there, D6.6). Remedy: "
               "fillet the corner's three edges at the same radius.");
    // The vertical flank's OWN lateral planes (the two planar
    // co-users of its loop's tangent lines, normals horizontal)
    // and its two caps (the cap arcs' planar co-users, normals
    // vertical): the corner's plane census — read through the
    // RADIAL structure (the co-user of an edge is the other face
    // its co-edges point into, never the walk's own reference).
    std::size_t lat_1 = F0, lat_2 = F0, cap_1 = F0, cap_2 = F0;
    for (const auto& ce :
         body.solid.shell().faces()[vert_face].loops()[0]
             .coedges()) {
        for (const CoEdge* rce :
             body.edges[ce.edge()].coedges()) {
            const std::size_t f = face_of_coedge(rce);
            if (f >= F0 || f == vert_face) continue;
            const PlaneSurface* pl = plane_of(f);
            if (pl == nullptr)
                refuse("the vertical flank borders a non-planar "
                       "face - the class signature is the box "
                       "family's. Remedy: report the body's "
                       "construction.");
            const bool vertical_normal =
                std::abs(pl->normal_vec().z) > 1.0 - 1e-9;
            if (vertical_normal) {
                if (cap_1 == F0) cap_1 = f;
                else if (cap_2 == F0 && f != cap_1) cap_2 = f;
            } else {
                if (lat_1 == F0) lat_1 = f;
                else if (lat_2 == F0 && f != lat_1) lat_2 = f;
            }
        }
    }
    if (lat_1 == F0 || lat_2 == F0 || cap_1 == F0 || cap_2 == F0)
        refuse("the vertical flank's plane census is incomplete "
               "(the two laterals and the two caps) - the class "
               "signature is the box family's. Remedy: report the "
               "body's construction.");
    // The SHARED plane (the ring pair's common adjacent face) and
    // the naming's THIRD plane (the vertical's lateral that is not
    // one of the reference's faces).
    const bool A_shared = std::abs(n_A.z) > 1.0 - 1e-9;
    const bool B_shared = std::abs(n_B.z) > 1.0 - 1e-9;
    if (A_shared == B_shared)
        refuse("the reference edge's planes are both vertical or "
               "both horizontal - the ring class takes one of each "
               "(the horizontal one the corner's shared plane). "
               "Remedy: report the body's construction.");
    const std::size_t shared_face = A_shared ? face_A : face_B;
    const PlaneSurface* pl_shared =
        A_shared ? pl_A : pl_B;
    const math::Vec3d n_shared = pl_shared->normal_vec();
    const PlaneSurface* pl_third =
        (lat_1 != face_A && lat_1 != face_B)
            ? plane_of(lat_1)
            : plane_of(lat_2);
    if (pl_third == nullptr)
        refuse("the corner's third plane was not found among the "
               "vertical flank's laterals - the class signature is "
               "the box family's. Remedy: report the body's "
               "construction.");
    const math::Vec3d n_third = pl_third->normal_vec();
    // THE CORNER and THE SPHERE CENTER (the closed forms: the
    // corner is the three planes' meet — {the shared plane, the
    // reference's lateral, the third plane} — each plane's own
    // offset n̂·origin along its axis; C the corner offset r inward
    // along each outward normal).
    const PlaneSurface* pl_lat = A_shared ? pl_B : pl_A;
    const math::Vec3d n_lat = A_shared ? n_B : n_A;
    const math::Vec3d corner = [&] {
        const PlaneSurface* pls[3] = {pl_shared, pl_lat, pl_third};
        const math::Vec3d nrm[3] = {n_shared, n_lat, n_third};
        math::Vec3d c{0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            const int k = std::abs(nrm[i].x) > 0.5
                              ? 0
                              : (std::abs(nrm[i].y) > 0.5 ? 1 : 2);
            const double comp =
                nrm[i].dot(pls[i]->origin()) / nrm[i][k];
            if (k == 0) c.x = comp;
            else if (k == 1) c.y = comp;
            else c.z = comp;
        }
        return c;
    }();
    const math::Vec3d C =
        corner - (n_shared + n_lat + n_third) * radius;
    // THE COHERENCE GUARD: the vertical flank's axis passes through
    // C's (x, y) — the coaxiality the closed forms demand (a
    // doctored body refuses loudly, never a wrong sphere).
    if (std::abs(vz->axis_center().x - C.x) > tol ||
        std::abs(vz->axis_center().y - C.y) > tol)
        refuse("the vertical flank's placement and the corner's "
               "planes disagree (the axis center reads (" +
               std::to_string(vz->axis_center().x) + ", " +
               std::to_string(vz->axis_center().y) +
               ") where the closed forms derive (" +
               std::to_string(C.x) + ", " + std::to_string(C.y) +
               ")) - the sphere's coaxial circles exist only at the "
               "agreement. Remedy: report the body's construction "
               "(the factory and the walks never disagree).");

    // ---- THE PARTNER RING EDGE (the corner's other ring edge —
    // the indivisible pair, D5.1: found through the corner's cap
    // arc, the vertical flank's own boundary at the corner).
    std::size_t e_cap = E0;   // the corner's cap arc (it dies)
    std::size_t v_occ2 = V0;  // the partner's corner-side end
    for (std::size_t ei = 0; ei < E0; ++ei) {
        if (ei == edge) continue;
        const auto& e = body.edges[ei];
        if (e.vertex(0) != static_cast<VertexId>(v_occ) &&
            e.vertex(1) != static_cast<VertexId>(v_occ))
            continue;
        bool borders_flank = false, borders_shared = false;
        for (const CoEdge* ce : e.coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (f == vert_face) borders_flank = true;
            if (f == shared_face) borders_shared = true;
        }
        // The cap arc borders the flank AND the shared plane (it
        // lies ON the corner's cap); the vertical tangent lines
        // border the flank and a lateral.
        if (!borders_flank || !borders_shared) continue;
        if (e_cap != E0)
            refuse("the corner carries more than one cap arc - the "
                   "class signature is the box family's. Remedy: "
                   "report the body's construction.");
        e_cap = ei;
        v_occ2 = static_cast<std::size_t>(
            e.vertex(0) == static_cast<VertexId>(v_occ)
                ? e.vertex(1)
                : e.vertex(0));
    }
    if (e_cap == E0 || v_occ2 == V0)
        refuse("the corner's cap arc was not found (the vertical "
               "flank's boundary at the occupied end) - the class "
               "signature is the box family's. Remedy: report the "
               "body's construction.");
    std::size_t edge2 = E0;    // the partner ring edge
    std::size_t face_A2 = F0, face_B2 = F0;
    for (std::size_t ei = 0; ei < E0; ++ei) {
        if (ei == edge || ei == e_cap) continue;
        const auto& e = body.edges[ei];
        if (e.vertex(0) != static_cast<VertexId>(v_occ2) &&
            e.vertex(1) != static_cast<VertexId>(v_occ2))
            continue;
        const math::Vec3d& q0 =
            vpos(static_cast<std::size_t>(e.vertex(0)));
        const math::Vec3d& q1 =
            vpos(static_cast<std::size_t>(e.vertex(1)));
        const bool px =
            std::abs(q0.y - q1.y) <= tol &&
            std::abs(q0.z - q1.z) <= tol;
        const bool py =
            std::abs(q0.x - q1.x) <= tol &&
            std::abs(q0.z - q1.z) <= tol;
        if (!px && !py) continue;  // the vertical tangent line
        const auto& ces = e.coedges();
        if (ces.size() != 2) continue;
        const std::size_t fa = face_of_coedge(ces[0]);
        const std::size_t fb = face_of_coedge(ces[1]);
        if (plane_of(fa) == nullptr || plane_of(fb) == nullptr)
            continue;
        if (edge2 != E0)
            refuse("the corner carries more than one partner ring "
                   "edge - the class signature is the box family's. "
                   "Remedy: report the body's construction.");
        edge2 = ei;
        face_A2 = fa;
        face_B2 = fb;
    }
    if (edge2 == E0)
        refuse("the corner's partner ring edge was not found - the "
               "corner act consumes the indivisible pair (D5.1: a "
               "ring flank without its sphere has untyped borders; "
               "the gate refuses the half act by definition). "
               "Remedy: report the body's construction (the box "
               "family always carries the pair).");
    // The partner's plane census: {the shared plane, the third}.
    {
        const bool a2_shared = face_A2 == shared_face;
        const bool b2_shared = face_B2 == shared_face;
        const std::size_t other =
            a2_shared ? face_B2 : (b2_shared ? face_A2 : F0);
        if (!a2_shared && !b2_shared)
            refuse("the partner ring edge does not border the "
                   "corner's shared plane - the class signature is "
                   "the box family's. Remedy: report the body's "
                   "construction.");
        const PlaneSurface* pl_o = plane_of(other);
        if (pl_o == nullptr ||
            std::abs(pl_o->normal_vec().dot(n_third) - 1.0) > 1e-9)
            refuse("the partner ring edge's other plane is not the "
                   "corner's third plane - the class signature is "
                   "the box family's. Remedy: report the body's "
                   "construction.");
    }
    const std::size_t v_far2 = static_cast<std::size_t>(
        body.edges[edge2].vertex(0) ==
                static_cast<VertexId>(v_occ2)
            ? body.edges[edge2].vertex(1)
            : body.edges[edge2].vertex(0));
    // THE FAR CORNERS' FREEDOM (the m2 wall's own check, one corner
    // over: the partner's far corner must be free too).
    for (const std::size_t fv : {v_far, v_far2}) {
        for (std::size_t ei = 0; ei < E0; ++ei) {
            if (ei == edge || ei == edge2) continue;
            const auto& e = body.edges[ei];
            if (e.vertex(0) != static_cast<VertexId>(fv) &&
                e.vertex(1) != static_cast<VertexId>(fv))
                continue;
            for (const CoEdge* ce : e.coedges()) {
                const std::size_t f = face_of_coedge(ce);
                if (f >= F0) continue;
                if (plane_of(f) == nullptr)
                    refuse("the far corner at (" +
                           std::to_string(vpos(fv).x) + ", " +
                           std::to_string(vpos(fv).y) + ", " +
                           std::to_string(vpos(fv).z) +
                           ") is occupied by a previous fillet's "
                           "flank - the corner act takes ONE corner "
                           "(the single-corner morso, D9's third); "
                           "the multi-corner chain is the ring "
                           "act's own (the twelve-spigoli morso). "
                           "Remedy: fillet corners whose ring edges' "
                           "far ends are free.");
            }
        }
    }

    // ---- THE TYPED BANDS (the m2 walk's own, per ring edge: the
    // tangent lines within the faces, the far arcs within the far
    // caps — the corner end's band is the coherence guard above and
    // the span guards below, never a face-exit test).
    const auto cap_of_vertex = [&](std::size_t dead_v,
                                   std::size_t fa,
                                   std::size_t fb) -> std::size_t {
        std::size_t cap = F0;
        for (std::size_t f = 0; f < F0; ++f) {
            if (f == fa || f == fb) continue;
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges()) {
                    const auto& e = body.edges[ce.edge()];
                    if (e.vertex(0) == dead_v ||
                        e.vertex(1) == dead_v) {
                        if (cap != F0 && cap != f)
                            refuse("a far corner sits on more than "
                                   "one end face - the class "
                                   "signature is the box's. Remedy: "
                                   "report the body's construction.");
                        cap = f;
                    }
                }
        }
        return cap;
    };
    const std::size_t cap_far1 =
        cap_of_vertex(v_far, face_A, face_B);
    const std::size_t cap_far2 =
        cap_of_vertex(v_far2, face_A2, face_B2);
    if (cap_far1 >= F0 || cap_far2 >= F0)
        refuse("a far corner's end-face lookup failed - report "
               "this as a kernel bug.");
    const auto cap_reach = [&](std::size_t cap, std::size_t dead_v) {
        double reach = std::numeric_limits<double>::infinity();
        for (const auto& ce :
             body.solid.shell().faces()[cap].loops()[0].coedges()) {
            const auto& e = body.edges[ce.edge()];
            const bool has_dead =
                e.vertex(0) == dead_v || e.vertex(1) == dead_v;
            if (!has_dead) continue;
            const std::size_t other =
                e.vertex(0) == dead_v
                    ? static_cast<std::size_t>(e.vertex(1))
                    : static_cast<std::size_t>(e.vertex(0));
            const math::Vec3d dq = vpos(other) - vpos(dead_v);
            reach = std::min(reach, std::sqrt(dq.length_sq()));
        }
        return reach;
    };
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
    if (radius >= cap_reach(cap_far1, v_far) - tol)
        band_refusal("the quarter arc exits the reference's far "
                     "end face",
                     cap_reach(cap_far1, v_far));
    if (radius >= cap_reach(cap_far2, v_far2) - tol)
        band_refusal("the quarter arc exits the partner's far end "
                     "face",
                     cap_reach(cap_far2, v_far2));

    // ---- THE SPHERE'S FRAME (D5.2's own derivation, no knob):
    // pole = the SHARED plane's outward normal (the equator is the
    // vertical flank's coaxial circle), azimuth = the reference's
    // lateral plane's outward normal (the seam meridian declared
    // there — the first lateral of the naming order), second axis
    // the product. The three tangency points: the feet of the
    // perpendiculars from C to the three planes.
    const math::Vec3d pole = n_shared;
    const math::Vec3d e0 = n_lat;
    const math::Vec3d e1 = e0.cross(pole);
    const math::Vec3d T_shared = C + pole * radius;
    const math::Vec3d T_lat = C + e0 * radius;
    const math::Vec3d T_third = C + n_third * radius;
    // The third plane's tangency azimuth in the sphere's frame (a
    // quarter from the seam, the sign the derived product's own).
    const double u_third =
        std::atan2(n_third.dot(e1), n_third.dot(e0));
    if (std::abs(std::abs(u_third) - kPi / 2.0) >
        kFilletArcSpanDefectBand)
        refuse("the sphere's tangency azimuths are not a "
               "perpendicular pair (the third plane's tangency "
               "reads " + std::to_string(u_third) +
               " where the class's own is +-pi/2) - the closed "
               "forms and the frame disagree; report this as a "
               "kernel bug.");
    const double u_s0 = std::min(0.0, u_third);
    const double u_s1 = std::max(0.0, u_third);

    // ---- THE TWO RING FLANKS' GEOMETRY (the m2 walk's own closed
    // forms, per edge: the cross-plane center on the bisector, the
    // tangent points as the feet of the perpendiculars, the
    // azimuth window the perpendicular-pair discrimination — the
    // correttiva's own structure — and the axial window ENDING AT
    // THE COAXIAL PLANE (C's own coordinate along the axis: the
    // refinement the sphere demands, never the full quarter span).
    const auto mod2pi = [](double a) {
        while (a < 0.0) a += 2.0 * kPi;
        while (a >= 2.0 * kPi) a -= 2.0 * kPi;
        return a;
    };
    struct FlankGeom {
        int axis{-1};
        math::Vec2d center2{};
        double u0{0}, u1{0};
        bool a_at_u0{false};
        double a_base{0}, a_act{0}, a_far_end{0};
        math::Vec3d tan3_a_far{}, tan3_b_far{};
        math::Vec3d tan3_a_c{}, tan3_b_c{};
    };
    const auto flank_geom = [&](std::size_t eid,
                                std::size_t fa, std::size_t fb,
                                std::size_t v_far_end) -> FlankGeom {
        FlankGeom g;
        const math::Vec3d& q0 =
            vpos(static_cast<std::size_t>(
                body.edges[eid].vertex(0)));
        const math::Vec3d& q1 =
            vpos(static_cast<std::size_t>(
                body.edges[eid].vertex(1)));
        const bool px = std::abs(q0.y - q1.y) <= tol &&
                        std::abs(q0.z - q1.z) <= tol;
        g.axis = px ? 0 : 1;
        const auto AXf = [&](const math::Vec3d& p) -> double {
            return g.axis == 0 ? p.x : p.y;
        };
        const auto cp = [&](const math::Vec3d& d) -> math::Vec2d {
            return g.axis == 0
                       ? math::Vec2d{d.y, d.z}
                       : math::Vec2d{d.x, d.z};
        };
        const auto az = [&](const math::Vec2d& d) -> double {
            return g.axis == 0 ? std::atan2(-d.x, d.y)
                               : std::atan2(-d.y, d.x);
        };
        const math::Vec3d n_a = plane_of(fa)->normal_vec();
        const math::Vec3d n_b = plane_of(fb)->normal_vec();
        const math::Vec2d n2_a = cp(n_a), n2_b = cp(n_b);
        // The corner's cross-plane (the edge's own fixed
        // coordinates — the three planes' meet read in the frame).
        // The center: the closed form on the bisector (the two
        // adjacent planes' cross-plane corner, r inward along
        // each). The corner's cross-plane coordinates ARE the two
        // planes' meet — read from the corner itself.
        const math::Vec2d c2 = cp(corner);
        // The two planes' cross-plane normals pick the components.
        const math::Vec2d ctr2{c2.x - radius * (n2_a.x + n2_b.x),
                               c2.y - radius * (n2_a.y + n2_b.y)};
        g.center2 = ctr2;
        // The tangent points (2D, the outward-normal feet).
        const math::Vec2d t2_a{ctr2.x + radius * n2_a.x,
                               ctr2.y + radius * n2_a.y};
        const math::Vec2d t2_b{ctr2.x + radius * n2_b.x,
                               ctr2.y + radius * n2_b.y};
        // The azimuth window (the correttiva's structure: both
        // flags read BEFORE the refusal).
        const double uA = az(n2_a), uB = az(n2_b);
        const bool uA_leads =
            std::abs(mod2pi(uB - uA) - kPi / 2.0) <=
            kFilletArcSpanDefectBand;
        const bool uB_leads =
            std::abs(mod2pi(uA - uB) - kPi / 2.0) <=
            kFilletArcSpanDefectBand;
        if (!uA_leads && !uB_leads)
            refuse("the flank's tangent azimuths are not a "
                   "perpendicular pair (the class's own are a "
                   "quarter apart) - the closed forms and the walk "
                   "disagree; report this as a kernel bug.");
        g.a_at_u0 = uA_leads;
        g.u0 = uA_leads ? uA : uB;
        g.u1 = g.u0 + kPi / 2.0;
        // The axial window: the far end's coordinate to C's own
        // (the coaxial plane), the base the lower end.
        const double a_act = AXf(C);
        const double a_far = AXf(vpos(v_far_end));
        g.a_act = a_act;
        g.a_far_end = a_far;
        g.a_base = std::min(a_act, a_far);
        // The tangent points in 3D: at the far end (the m2's own
        // cap side) and at the coaxial plane (the C side — the
        // refinement: the tangent lines END at the sphere's
        // tangency points, never the corner).
        const auto tan3 = [&](const math::Vec2d& t2,
                              double a) -> math::Vec3d {
            return g.axis == 0
                       ? math::Vec3d{a, t2.x, t2.y}
                       : math::Vec3d{t2.x, a, t2.y};
        };
        g.tan3_a_far = tan3(t2_a, a_far);
        g.tan3_b_far = tan3(t2_b, a_far);
        g.tan3_a_c = C + n_a * radius;
        g.tan3_b_c = C + n_b * radius;
        return g;
    };
    const FlankGeom G1 =
        flank_geom(edge, face_A, face_B, v_far);
    const FlankGeom G2 =
        flank_geom(edge2, face_A2, face_B2, v_far2);
    if (G1.axis == G2.axis)
        refuse("the corner's two ring edges are parallel - the "
               "class signature is the box family's perpendicular "
               "pair. Remedy: report the body's construction.");

    // ---- THE MAPS: the survivors compact, the born append (the
    // vertical walk's own table, the act's counts its own).
    std::vector<std::size_t> vmap(V0, V0), emap(E0, E0);
    {
        std::size_t next = 0;
        for (std::size_t i = 0; i < V0; ++i)
            if (i != v_occ && i != v_far && i != v_occ2 &&
                i != v_far2)
                vmap[i] = next++;
        next = 0;
        for (std::size_t i = 0; i < E0; ++i)
            if (i != edge && i != edge2 && i != e_cap)
                emap[i] = next++;
    }
    const std::size_t v_base = V0 - 4;
    const std::size_t v_tA1_far = v_base + 0;
    const std::size_t v_tB1_far = v_base + 1;
    const std::size_t v_tA2_far = v_base + 2;
    const std::size_t v_tB2_far = v_base + 3;
    const std::size_t v_T_shared = v_base + 4;
    const std::size_t v_T_lat = v_base + 5;
    const std::size_t v_T_third = v_base + 6;
    const std::size_t e_base = E0 - 3;
    const std::size_t e_tanA1 = e_base + 0;   // the reference's
    const std::size_t e_tanB1 = e_base + 1;   // tangent lines
    const std::size_t e_capfar1 = e_base + 2;  // its far cap arc
    const std::size_t e_coax1 = e_base + 3;   // its coaxial arc
    const std::size_t e_tanA2 = e_base + 4;   // the partner's
    const std::size_t e_tanB2 = e_base + 5;   // tangent lines
    const std::size_t e_capfar2 = e_base + 6;  // its far cap arc
    const std::size_t e_coax2 = e_base + 7;   // its coaxial arc
    const std::size_t e_equator = e_base + 8;  // the vertical's

    // The tangent-point vertex OF A FACE (the act's three: the
    // shared plane's, the reference's lateral's, the third's).
    const std::size_t lat_face = A_shared ? face_B : face_A;
    const std::size_t third_face =
        face_A2 == shared_face ? face_B2 : face_A2;
    const auto t_of_face = [&](std::size_t f) -> std::size_t {
        if (f == shared_face) return v_T_shared;
        if (f == lat_face) return v_T_lat;
        if (f == third_face) return v_T_third;
        refuse("a tangency point was asked of a face outside the "
               "corner's three planes - report this as a kernel "
               "bug.");
        return 0;  // unreachable — refuse() throws
    };
    const auto far_t1_of_face = [&](std::size_t f) -> std::size_t {
        if (f == face_A) return v_tA1_far;
        if (f == face_B) return v_tB1_far;
        refuse("the reference's far tangent was asked of a face "
               "outside its two - report this as a kernel bug.");
        return 0;  // unreachable — refuse() throws
    };
    const auto far_t2_of_face = [&](std::size_t f) -> std::size_t {
        if (f == face_A2) return v_tA2_far;
        if (f == face_B2) return v_tB2_far;
        refuse("the partner's far tangent was asked of a face "
               "outside its two - report this as a kernel bug.");
        return 0;  // unreachable — refuse() throws
    };
    // The re-spanning (the m2's own rule, generalized: a dead
    // endpoint's replacement is the tangent vertex of the plane —
    // among the owning edge's pair — that the re-spanned edge
    // borders).
    const auto substitute = [&](std::size_t dead_v,
                                std::size_t e) -> std::size_t {
        for (const CoEdge* ce : body.edges[e].coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (dead_v == v_far && (f == face_A || f == face_B))
                return far_t1_of_face(f);
            if (dead_v == v_occ && (f == face_A || f == face_B))
                return t_of_face(f);
            if (dead_v == v_far2 &&
                (f == face_A2 || f == face_B2))
                return far_t2_of_face(f);
            if (dead_v == v_occ2 &&
                (f == face_A2 || f == face_B2))
                return t_of_face(f);
        }
        refuse("a re-spanned edge's co-user is outside the act's "
               "own faces - the class signature is the box "
               "family's. Remedy: report the body's construction.");
        return 0;  // unreachable — refuse() throws
    };
    const auto new_end = [&](std::size_t old_end, std::size_t e) {
        if (old_end != v_occ && old_end != v_far &&
            old_end != v_occ2 && old_end != v_far2)
            return vmap[old_end];
        return substitute(old_end, e);
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- The vertices: the survivors, then the seven born (the
    // two flanks' far tangent pairs first — the operands' order —
    // then the three tangency points, the shared plane's first).
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        R->vertices.push_back(body.vertices[i]);
        R->vertices.back().set_id(
            static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[7] = {
            G1.tan3_a_far, G1.tan3_b_far,
            G2.tan3_a_far, G2.tan3_b_far,
            T_shared, T_lat, T_third};
        const std::size_t ids[7] = {
            v_tA1_far, v_tB1_far, v_tA2_far, v_tB2_far,
            v_T_shared, v_T_lat, v_T_third};
        for (std::size_t i = 0; i < 7; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the endpoints remapped through
    // the re-spanning), then the nine born — the two flanks' four
    // tangent lines (A then B, the reference then the partner), the
    // two far cap arcs (the B->A convention), the two coaxial arcs
    // (the A->B convention, the sphere's meridians), the equator
    // (the laterals' pair — the vertical flank's new cap).
    for (std::size_t i = 0; i < E0; ++i) {
        if (emap[i] == E0) continue;
        R->edges.push_back(body.edges[i]);
        auto& e = R->edges.back();
        e.set_id(static_cast<EdgeId>(emap[i]));
        e.set_vertex(0, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(
                                e.vertex(0)), i)));
        e.set_vertex(1, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(
                                e.vertex(1)), i)));
    }
    {
        const std::pair<std::size_t, std::size_t> born[9] = {
            {v_tA1_far, t_of_face(face_A)},
            {v_tB1_far, t_of_face(face_B)},
            {v_tB1_far, v_tA1_far},
            {t_of_face(face_A), t_of_face(face_B)},
            {v_tA2_far, t_of_face(face_A2)},
            {v_tB2_far, t_of_face(face_B2)},
            {v_tB2_far, v_tA2_far},
            {t_of_face(face_A2), t_of_face(face_B2)},
            {v_T_lat, v_T_third}};
        for (const auto& [a, b] : born) {
            Edge e{static_cast<VertexId>(a),
                   static_cast<VertexId>(b)};
            e.set_id(static_cast<EdgeId>(R->edges.size()));
            e.set_provenance(prov);
            R->edges.push_back(e);
        }
    }

    // ---- The p-curve writers (the m2's own, verbatim in every
    // part the corner does not name).
    const auto pos3 = [&](std::size_t v) -> const math::Vec3d& {
        return R->vertices[v].position();
    };
    const auto emit_planar = [&](Loop& loop,
                                 const PlaneSurface& pl,
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
            refuse("the re-authored walk lost the chaining (an "
                   "edge that does not run between the walk's "
                   "endpoints) - report this as a kernel bug.");
        CoEdge ce{static_cast<EdgeId>(e_id), rev};
        ce.set_p_curve(pcurve_segment(
            fillet_detail::dom_of(pl, pos3(a)),
            fillet_detail::dom_of(pl, pos3(b)),
            /*interior_left=*/true));
        loop.add_coedge(ce);
    };
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
        const bool re_spanned =
            s_old == v_occ || s_old == v_far ||
            s_old == v_occ2 || s_old == v_far2 ||
            e_old == v_occ || e_old == v_far ||
            e_old == v_occ2 || e_old == v_far2;
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
    // The re-authoring of a planar face whose walk crosses the act:
    // the dead ring edges' spans replaced by the flanks' tangent
    // lines (the dead co-edge's own walk direction), the dead cap
    // arc's span COLLAPSED at the tangency point (the two lines
    // meet there — D5.3's rectangle corner), the survivors
    // re-emitted through the maps.
    const auto reauthor_corner_plane =
        [&](std::size_t face_index) {
            const Face& src =
                body.solid.shell().faces()[face_index];
            fillet_detail::PlaneReader rd;
            src.surface()->accept(rd);
            const PlaneSurface& pl = *rd.plane;
            Loop loop;
            for (const auto& ce : src.loops()[0].coedges()) {
                const std::size_t e_old = ce.edge();
                if (e_old == edge || e_old == edge2) {
                    const bool first = e_old == edge;
                    const std::size_t fa =
                        first ? face_A : face_A2;
                    const std::size_t vfv =
                        first ? v_far : v_far2;
                    const std::size_t tan_e =
                        fa == face_index
                            ? (first ? e_tanA1 : e_tanA2)
                            : (first ? e_tanB1 : e_tanB2);
                    const std::size_t d_start =
                        static_cast<std::size_t>(
                            ce.reversed()
                                ? body.edges[e_old].vertex(1)
                                : body.edges[e_old].vertex(0));
                    const bool far_to_corner =
                        d_start == vfv;
                    const auto far_of =
                        [&](std::size_t f) {
                            return first
                                       ? far_t1_of_face(f)
                                       : far_t2_of_face(f);
                        };
                    const std::size_t from =
                        far_to_corner
                            ? far_of(face_index)
                            : t_of_face(face_index);
                    const std::size_t to =
                        far_to_corner
                            ? t_of_face(face_index)
                            : far_of(face_index);
                    emit_planar(loop, pl, tan_e, from, to);
                    continue;
                }
                if (e_old == e_cap) continue;  // collapsed at T
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
    reauthor_corner_plane(shared_face);
    reauthor_corner_plane(lat_face);
    reauthor_corner_plane(third_face);

    // ---- The far caps (the m2's reauthor_cap, verbatim in every
    // part: the quarter arc inserted at the dead far corner, the
    // re-spanned edges riding the maps).
    const auto reauthor_far_cap = [&](std::size_t face_index,
                                      std::size_t dead_v,
                                      std::size_t arc_edge,
                                      const FlankGeom& G,
                                      std::size_t fa,
                                      std::size_t fb) {
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
                ce.reversed()
                    ? body.edges[ce.edge()].vertex(0)
                    : body.edges[ce.edge()].vertex(1));
            loop.add_coedge(reemit(pl, ce));
            if (e_old == dead_v) {
                const auto& next = ces[(j + 1) % ces.size()];
                const std::size_t n_old =
                    static_cast<std::size_t>(
                        next.reversed()
                            ? body.edges[next.edge()].vertex(1)
                            : body.edges[next.edge()].vertex(0));
                const std::size_t n_new =
                    new_end(n_old, next.edge());
                const std::size_t here =
                    new_end(e_old, ce.edge());
                const math::Vec2d a2 =
                    fillet_detail::dom_of(pl, pos3(here));
                const math::Vec2d b2 =
                    fillet_detail::dom_of(pl, pos3(n_new));
                const math::Vec3d foot3 =
                    G.axis == 0
                        ? math::Vec3d{G.a_far_end,
                                      G.center2.x, G.center2.y}
                        : math::Vec3d{G.center2.x,
                                      G.a_far_end, G.center2.y};
                const math::Vec2d c2 =
                    fillet_detail::dom_of(pl, foot3);
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
    reauthor_far_cap(cap_far1, v_far, e_capfar1, G1,
                     face_A, face_B);
    reauthor_far_cap(cap_far2, v_far2, e_capfar2, G2,
                     face_A2, face_B2);

    // ---- THE VERTICAL FLANK'S RI-FINESTRAMENTO (D6.3): the face
    // SURVIVES — the same index, the same name, the same stamp, the
    // same surface (value-copied) — its loop re-authored in place:
    // the two tangent lines SHORTENED to the coaxial plane (their
    // corner ends re-spanned to the laterals' tangency points), the
    // corner's cap arc REPLACED by the equator (the p-curve the old
    // arc's own u-span at the coaxial v — the closed forms' own
    // agreement), the far cap arc carried verbatim. The window is
    // the loop: the surface never changes, the edit is real.
    {
        const Face& src =
            body.solid.shell().faces()[vert_face];
        const auto& ces = src.loops()[0].coedges();
        if (ces.size() != 4)
            refuse("the vertical flank's loop carries " +
                   std::to_string(ces.size()) +
                   " co-edges (the class's own are four) - report "
                   "this as a kernel bug.");
        const double v_coax = C.z - vz->z_base();
        Loop loop;
        for (const auto& ce : ces) {
            const std::size_t e_old = ce.edge();
            const std::size_t s_old = static_cast<std::size_t>(
                ce.reversed()
                    ? body.edges[e_old].vertex(1)
                    : body.edges[e_old].vertex(0));
            const std::size_t e_old_end =
                static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[e_old].vertex(0)
                        : body.edges[e_old].vertex(1));
            if (e_old == e_cap) {
                // THE EQUATOR: the old arc's u-span at the coaxial
                // v, the endpoints the laterals' tangency points.
                if (ce.p_curve() == nullptr ||
                    ce.p_curve()->kind != PCurve::Kind::Segment)
                    refuse("the corner's cap arc carries no "
                           "segment p-curve on the flank's domain - "
                           "report this as a kernel bug.");
                const double ua = ce.p_curve()->segment.a.x;
                const double ub = ce.p_curve()->segment.b.x;
                const std::size_t from_new =
                    s_old == v_occ ? v_T_lat : v_T_third;
                const std::size_t to_new =
                    e_old_end == v_occ ? v_T_lat : v_T_third;
                const auto& stored = R->edges[e_equator];
                const bool fwd =
                    stored.vertex(0) ==
                            static_cast<VertexId>(from_new) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(to_new);
                const bool rev =
                    stored.vertex(0) ==
                            static_cast<VertexId>(to_new) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(from_new);
                if (!fwd && !rev)
                    refuse("the equator's stored pair does not run "
                           "between the tangency points - report "
                           "this as a kernel bug.");
                CoEdge out{static_cast<EdgeId>(e_equator), rev};
                out.set_p_curve(pcurve_segment(
                    math::Vec2d{ua, v_coax},
                    math::Vec2d{ub, v_coax},
                    /*interior_left=*/true));
                loop.add_coedge(out);
                continue;
            }
            // A tangent line (shortened) or the far cap arc
            // (carried verbatim): the survivors' co-edges with the
            // re-spanned corner ends and FRESH domain p-curves —
            // the u the old p-curve's own, the v-span read off the
            // NEW walk's endpoints (the maps' own arithmetic).
            const bool is_tangent =
                s_old == v_occ || s_old == v_occ2 ||
                e_old_end == v_occ || e_old_end == v_occ2;
            if (!is_tangent) {
                if (emap[e_old] == E0)
                    refuse("the vertical flank's loop lost a co-edge "
                           "to the dead column - report this as a "
                           "kernel bug.");
                CoEdge out{static_cast<EdgeId>(emap[e_old]),
                           ce.reversed()};
                if (ce.p_curve() != nullptr)
                    out.set_p_curve(*ce.p_curve());
                loop.add_coedge(out);
                continue;
            }
            if (ce.p_curve() == nullptr ||
                ce.p_curve()->kind != PCurve::Kind::Segment ||
                ce.p_curve()->segment.a.x !=
                    ce.p_curve()->segment.b.x)
                refuse("the vertical flank's tangent line carries "
                       "no u-const segment p-curve on the flank's "
                       "domain - report this as a kernel bug.");
            const double u_t = ce.p_curve()->segment.a.x;
            // The v-span: the FAR end unchanged, the CORNER end the
            // coaxial v — and which end is the corner's is a fact of
            // the TOPOLOGY (the walk's start sits on the occupied
            // vertex), never of the v-ordering: at a top corner the
            // far end is the span's minimum, at a bottom corner its
            // maximum, and the act owes BOTH orientations the same
            // closed form (the eight-corner sweep's own catch — the
            // min/max reading authored [0, v_coax] at the bottom,
            // the corner's own dead span, instead of [v_coax, far]).
            const bool a_is_corner =
                (s_old == v_occ || s_old == v_occ2);
            const double v_span_far =
                a_is_corner ? ce.p_curve()->segment.b.y
                            : ce.p_curve()->segment.a.y;
            const std::size_t s_new =
                new_end(s_old, e_old);
            const std::size_t e_new =
                new_end(e_old_end, e_old);
            const auto& stored = R->edges[emap[e_old]];
            const bool fwd =
                stored.vertex(0) ==
                        static_cast<VertexId>(s_new) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(e_new);
            const bool rev =
                stored.vertex(0) ==
                        static_cast<VertexId>(e_new) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(s_new);
            if (!fwd && !rev)
                refuse("the shortened tangent line's stored pair "
                       "does not run between the walk's endpoints - "
                       "report this as a kernel bug.");
            CoEdge out{static_cast<EdgeId>(emap[e_old]), rev};
            out.set_p_curve(pcurve_segment(
                a_is_corner ? math::Vec2d{u_t, v_coax}
                            : math::Vec2d{u_t, v_span_far},
                a_is_corner ? math::Vec2d{u_t, v_span_far}
                            : math::Vec2d{u_t, v_coax},
                /*interior_left=*/true));
            loop.add_coedge(out);
        }
        Face face(std::make_shared<CylinderZSurface>(*vz));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(vert_face));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= vert_face)
            faces_mut.resize(vert_face + 1);
        faces_mut[vert_face] = std::move(face);
    }

    // ---- The kept faces: the copies carry the surfaces in their
    // OWN families (the planes and the chain's flanks — never a
    // re-read through a foreign reader), the co-edges through the
    // maps.
    for (std::size_t f = 0; f < F0; ++f) {
        if (f == shared_face || f == lat_face || f == third_face ||
            f == cap_far1 || f == cap_far2 || f == vert_face)
            continue;
        const Face& src = body.solid.shell().faces()[f];
        std::shared_ptr<SurfaceGeometry> kept_surf;
        {
            fillet_detail::PlaneReader rd;
            src.surface()->accept(rd);
            if (rd.plane != nullptr)
                kept_surf =
                    std::make_shared<PlaneSurface>(*rd.plane);
            else {
                fillet_detail::FlankReader fr;
                src.surface()->accept(fr);
                if (fr.z != nullptr)
                    kept_surf =
                        std::make_shared<CylinderZSurface>(*fr.z);
                else if (fr.x != nullptr)
                    kept_surf =
                        std::make_shared<CylinderXSurface>(*fr.x);
                else if (fr.y != nullptr)
                    kept_surf =
                        std::make_shared<CylinderYSurface>(*fr.y);
                else
                    refuse("a kept face's surface is outside the "
                           "admission's families - report this as "
                           "a kernel bug.");
            }
        }
        Face face(std::move(kept_surf));
        for (const auto& loop : src.loops()) {
            Loop nl;
            nl.set_outer(loop.is_outer());
            for (const auto& ce : loop.coedges()) {
                const std::size_t mapped = emap[ce.edge()];
                if (mapped == E0)
                    refuse("the dead column's edge appeared in a "
                           "face outside the re-authored set - "
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

    // ---- THE TWO RING FLANKS ARE BORN, GIÀ RIFINITI (D5.2): the
    // m2's own rectangle-window class — the u-quarter facing the
    // corner × the axial span from the far cap to the COAXIAL
    // PLANE (never the full quarter: the corner end stops at the
    // sphere, the arc there the coaxial meridian).
    // The walk, CCW in the flank's own domain, the material on the
    // left (the m2's own): the LOW arc (whichever end it belongs
    // to), the tangent at u1, the HIGH arc, the tangent at u0 —
    // the arcs are the far cap's and the coaxial meridian's, the
    // v-window [vA, vB] the span from the far cap to the coaxial
    // plane.
    const auto emit_born_flank = [&](Loop& loop,
                                     const FlankGeom& G,
                                     std::size_t e_tan_a,
                                     std::size_t e_tan_b,
                                     std::size_t e_far_arc,
                                     std::size_t e_coax,
                                     std::size_t v_far_a,
                                     std::size_t v_far_b,
                                     std::size_t v_c_a,
                                     std::size_t v_c_b) {
        const double v_far_abs = G.a_far_end - G.a_base;
        const double v_act_abs = G.a_act - G.a_base;
        const double vA = std::min(v_far_abs, v_act_abs);
        const double vB = std::max(v_far_abs, v_act_abs);
        const bool far_is_low = v_far_abs < v_act_abs;
        const std::size_t v_u0_far =
            G.a_at_u0 ? v_far_a : v_far_b;
        const std::size_t v_u1_far =
            G.a_at_u0 ? v_far_b : v_far_a;
        const std::size_t v_u0_act =
            G.a_at_u0 ? v_c_a : v_c_b;
        const std::size_t v_u1_act =
            G.a_at_u0 ? v_c_b : v_c_a;
        const auto emit = [&](std::size_t e_id, std::size_t a,
                              std::size_t b, double ua, double va,
                              double ub, double vb) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the born flank's walk lost the chaining - "
                       "report this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                math::Vec2d{ua, va}, math::Vec2d{ub, vb},
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        // The arcs' domain: u0 -> u1 at the LOW v and u1 -> u0 at
        // the HIGH v (the CCW rectangle), each arc's 3D walk
        // reading its OWN face (the far cap's quarter, the coaxial
        // meridian's quarter) — the p-curve here is the flank's
        // OWN domain rectangle, the class's own form.
        const std::size_t e_low_arc =
            far_is_low ? e_far_arc : e_coax;
        const std::size_t e_high_arc =
            far_is_low ? e_coax : e_far_arc;
        const std::size_t v_low_u0 =
            far_is_low ? v_u0_far : v_u0_act;
        const std::size_t v_low_u1 =
            far_is_low ? v_u1_far : v_u1_act;
        const std::size_t v_high_u0 =
            far_is_low ? v_u0_act : v_u0_far;
        const std::size_t v_high_u1 =
            far_is_low ? v_u1_act : v_u1_far;
        const std::size_t e_u1_tan =
            G.a_at_u0 ? e_tan_b : e_tan_a;
        const std::size_t e_u0_tan =
            G.a_at_u0 ? e_tan_a : e_tan_b;
        emit(e_low_arc, v_low_u0, v_low_u1, G.u0, vA, G.u1, vA);
        emit(e_u1_tan, v_low_u1, v_high_u1, G.u1, vA, G.u1, vB);
        emit(e_high_arc, v_high_u1, v_high_u0, G.u1, vB, G.u0,
             vB);
        emit(e_u0_tan, v_high_u0, v_low_u0, G.u0, vB, G.u0, vA);
    };

    // The naming context (the m2's own: the fillet's declaration
    // with the body's binding and the same-feature extension).
    const Face& src_A_ref =
        body.solid.shell().faces()[face_A];
    const Face& src_B_ref =
        body.solid.shell().faces()[face_B];
    const Face& src_A2_ref =
        body.solid.shell().faces()[face_A2];
    const Face& src_B2_ref =
        body.solid.shell().faces()[face_B2];
    const Face& src_third_ref =
        body.solid.shell().faces()[third_face];
    const Face& src_vert_ref =
        body.solid.shell().faces()[vert_face];
    const bool named =
        naming.body != nullptr &&
        src_A_ref.shape_id().subshape.depth > 0 &&
        src_B_ref.shape_id().subshape.depth > 0 &&
        src_A2_ref.shape_id().subshape.depth > 0 &&
        src_B2_ref.shape_id().subshape.depth > 0 &&
        src_third_ref.shape_id().subshape.depth > 0 &&
        src_vert_ref.shape_id().subshape.depth > 0;
    naming::OperandBinding fil_operands[2] = {
        {src_A_ref.shape_id().feature, naming.body},
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
    // The flanks' own refs (the m2's grammar, each edge's own
    // operands).
    const auto flank_ref_of = [&](const Face& fa,
                                  const Face& fb) {
        geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(
                naming::kFilFlankRole);
        ref = leg(ref, fa);
        ref = leg(ref, fb);
        return ref;
    };
    // The corner's own ref (D2.5: the three adjacent planes in the
    // picker's order — the reference's A, its B, then the third).
    const auto corner_ref = [&]() {
        geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(
                naming::kFilCornerRole);
        ref = leg(ref, src_A_ref);
        ref = leg(ref, src_B_ref);
        ref = leg(ref, src_third_ref);
        return ref;
    }();

    // F1 — the reference's flank.
    {
        std::shared_ptr<SurfaceGeometry> surf;
        if (G1.axis == 0)
            surf = std::make_shared<CylinderXSurface>(
                radius, G1.a_base, G1.center2.x, G1.center2.y);
        else
            surf = std::make_shared<CylinderYSurface>(
                radius, G1.a_base, G1.center2.x, G1.center2.y);
        Loop loop;
        emit_born_flank(loop, G1, e_tanA1, e_tanB1, e_capfar1,
                        e_coax1, v_tA1_far, v_tB1_far,
                        t_of_face(face_A), t_of_face(face_B));
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0));
        face.set_provenance(prov);
        if (named) {
            const geometry::SubshapeRef ref =
                flank_ref_of(src_A_ref, src_B_ref);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, fil_decl.faces()));
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {{e_tanA1, &src_A_ref},
                          {e_tanB1, &src_B_ref}};
            for (const auto& w : wires) {
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kFilFlankRole);
                wref = leg(wref, src_A_ref);
                wref = leg(wref, src_B_ref);
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(
                    naming::render_name(naming.feature, wref,
                                        fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }
    // F2 — the partner's flank (the other placement).
    {
        std::shared_ptr<SurfaceGeometry> surf;
        if (G2.axis == 0)
            surf = std::make_shared<CylinderXSurface>(
                radius, G2.a_base, G2.center2.x, G2.center2.y);
        else
            surf = std::make_shared<CylinderYSurface>(
                radius, G2.a_base, G2.center2.x, G2.center2.y);
        Loop loop;
        emit_born_flank(loop, G2, e_tanA2, e_tanB2, e_capfar2,
                        e_coax2, v_tA2_far, v_tB2_far,
                        t_of_face(face_A2), t_of_face(face_B2));
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0 + 1));
        face.set_provenance(prov);
        if (named) {
            const geometry::SubshapeRef ref =
                flank_ref_of(src_A2_ref, src_B2_ref);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, fil_decl.faces()));
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {{e_tanA2, &src_A2_ref},
                          {e_tanB2, &src_B2_ref}};
            for (const auto& w : wires) {
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kFilFlankRole);
                wref = leg(wref, src_A2_ref);
                wref = leg(wref, src_B2_ref);
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(
                    naming::render_name(naming.feature, wref,
                                        fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE SPHERE IS BORN — the ottanto (D5.2): the existing
    // family, a new window. The loop: the equator (the vertical
    // flank's coaxial quarter — the base at v = pi/2) and the two
    // meridians (the ring flanks' coaxial quarters — u = const,
    // meeting at the pole, the tangency point on the shared plane,
    // where the window's v = 0 side degenerates). Three edges,
    // three vertices, one loop, the walk CCW in the domain.
    {
        auto surf = std::make_shared<SphereSurface>(
            radius, C, pole, e0);
        // The meridians' seats: u = 0 is the seam (the reference's
        // flank arc — e0 is its lateral's normal); u = u_third the
        // partner's.
        const bool third_is_low = u_third < 0.0;
        const std::size_t e_mer_low =
            third_is_low ? e_coax2 : e_coax1;
        const std::size_t e_mer_high =
            third_is_low ? e_coax1 : e_coax2;
        const std::size_t v_base_low =
            third_is_low ? v_T_third : v_T_lat;
        const std::size_t v_base_high =
            third_is_low ? v_T_lat : v_T_third;
        Loop loop;
        const auto emit = [&](std::size_t e_id, std::size_t a,
                              std::size_t b, double ua, double va,
                              double ub, double vb) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the sphere's walk lost the chaining - "
                       "report this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                math::Vec2d{ua, va}, math::Vec2d{ub, vb},
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        // The walk: the sphere's own parametrization carries
        // ∂u×∂v INWARD (−n̂ by construction — the family's form),
        // so the material-on-the-left law reads the CLOCKWISE
        // domain walk here, the cylinders' own CCW mirrored: the
        // equator from the seam's side to the third's, the
        // meridians closing at the pole (the shared plane's
        // tangency point).
        emit(e_equator, v_base_high, v_base_low, u_s1, kPi / 2.0,
             u_s0, kPi / 2.0);
        emit(e_mer_low, v_base_low, v_T_shared, u_s0, kPi / 2.0,
             u_s0, 0.0);
        emit(e_mer_high, v_T_shared, v_base_high, u_s1, 0.0,
             u_s1, kPi / 2.0);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0 + 2));
        face.set_provenance(prov);
        if (named) {
            face.set_shape_id({naming.feature, corner_ref, {}});
            face.set_name(naming::render_name(
                naming.feature, corner_ref, fil_decl.faces()));
            // The coaxial wires (D2.5: isect[corner × flank], the
            // level absent — the coaxial tangency IS the level):
            // each arc's ref = the corner's own path then the
            // flank's (the same-feature extension, both legs the
            // fillet's own born faces); the equator's second leg
            // the VERTICAL flank's own stored path.
            const struct {
                std::size_t edge;
                const Face* flank_owner_a;
                const Face* flank_owner_b;
                bool vertical;
            } coax[3] = {
                {e_coax1, &src_A_ref, &src_B_ref, false},
                {e_coax2, &src_A2_ref, &src_B2_ref, false},
                {e_equator, nullptr, nullptr, true}};
            for (const auto& w : coax) {
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectCoaxRole);
                for (std::uint8_t k = 0;
                     k < corner_ref.depth; ++k)
                    wref = wref.push(corner_ref.path[k]);
                if (w.vertical) {
                    const geometry::SubshapeRef vf =
                        src_vert_ref.shape_id().subshape;
                    for (std::uint8_t k = 0; k < vf.depth; ++k)
                        wref = wref.push(vf.path[k]);
                } else {
                    wref = wref.push(naming::kFilFlankRole);
                    wref = leg(wref, *w.flank_owner_a);
                    wref = leg(wref, *w.flank_owner_b);
                }
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(
                    naming::render_name(naming.feature, wref,
                                        fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE OUTPUT GATE (the act's own arithmetic, hard
    // postconditions).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "fillet_ring_edge: the corner act's postcondition "
            "failed - a loop does not chain (an assembly defect in "
            "the act's re-authored walks; report this as a kernel "
            "bug)");
    if (!is_closed_orientable(*R)) {
        // The named diagnosis (the house's own culture): WHICH
        // edge breaks the radial contract, its endpoints and its
        // co-edge count — never a bare throw.
        std::string diag;
        for (std::size_t i = 0; i < R->edges.size(); ++i) {
            const std::size_t n = R->edges[i].coedges().size();
            if (n == 2) {
                const bool r0 = R->edges[i].coedges()[0]->reversed();
                const bool r1 = R->edges[i].coedges()[1]->reversed();
                if (r0 != r1) continue;
                diag += " edge " + std::to_string(i) + " (two " +
                        std::string(r0 ? "reversed" : "forward") +
                        " co-edges)";
            } else {
                diag += " edge " + std::to_string(i) + " (" +
                        std::to_string(n) + " co-edges)";
            }
            const auto& e = R->edges[i];
            diag += " [" +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(0))]
                        .position().x) + ", " +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(0))]
                        .position().y) + ", " +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(0))]
                        .position().z) + "] -> [" +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(1))]
                        .position().x) + ", " +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(1))]
                        .position().y) + ", " +
                    std::to_string(R->vertices
                        [static_cast<std::size_t>(e.vertex(1))]
                        .position().z) + "]";
            break;
        }
        throw std::runtime_error(
            "fillet_ring_edge: the corner act's postcondition "
            "failed - the closed-orientable contract broke (" +
            (diag.empty() ? std::string("the orientation walk")
                          : diag) +
            "; report this as a kernel bug)");
    }
    if (R->vertex_count() != V0 - 4 + 7 ||
        R->edge_count() != E0 - 3 + 9 ||
        R->face_count() != F0 + 3)
        throw std::runtime_error(
            "fillet_ring_edge: the corner act's postcondition "
            "failed - the counts moved (the act's own arithmetic "
            "is V-4+7 / E-3+9 / F+3; report this as a kernel "
            "bug)");
    if (euler_characteristic(*R) != euler_characteristic(body))
        throw std::runtime_error(
            "fillet_ring_edge: the corner act's postcondition "
            "failed - chi moved (the act is chi-flat by "
            "construction, Delta-chi = +3 -6 +3 = 0; report "
            "this as a kernel bug)");
    return R;
}

// ---------------------------------------------------------------------------
// fillet_ring_edge — the implementation (the m6ter walk ONE AXIS OVER,
// G5 m2, ADR-0043 D2.2/D9)
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> fillet_ring_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double radius, const FilletNaming& naming, double tol) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("fillet_ring_edge: " + what);
    };

    // ---- The radius's own gate (the class's own, verbatim).
    if (!(radius > 0.0) || !std::isfinite(radius))
        refuse("the radius must be finite and > 0, got " +
               std::to_string(radius) +
               " — a non-positive ball has no envelope and no tangent "
               "lines. Remedy: author a positive radius within the "
               "faces' reach (the band the admission derives).");

    // ---- The class signature: THE BOX FAMILY CHAIN (the same counts
    // gate as the vertical walk's; the curved faces the chain's THREE
    // flank placements — the verticals' CylinderZ and this walk's
    // CylinderX/CylinderY, one family, the counts' k agreeing).
    std::size_t chain_k = 0;
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
        const std::size_t kcand = F0 >= 6 ? F0 - 6
                                        : static_cast<std::size_t>(-1);
        const bool counts_match =
            kcand != static_cast<std::size_t>(-1) &&
            V0 == 8 + 2 * kcand && E0 == 12 + 3 * kcand;
        if (!counts_match ||
            euler_characteristic(body) != 2 || !is_closed_orientable(body))
            refuse("the body is outside the class signature (the "
                   "box family chain's own: 8+2k vertices, 12+3k "
                   "edges, 6+k faces for k previous fillets, chi = 2, "
                   "closed orientable; found " +
                   std::to_string(V0) + "/" + std::to_string(E0) +
                   "/" + std::to_string(F0) +
                   ") - the ring class takes the box family's "
                   "horizontal 90-degree corners (ADR-0043 D2.2). "
                   "Remedy: fillet a factory box's ring edge (one of "
                   "box#F:top0..3 / bottom0..3), or a chain of them "
                   "(the current state a previous fillet left).");
        chain_k = kcand;
        std::size_t flanks = 0;
        for (const auto& f : body.solid.shell().faces()) {
            if (f.loops().size() != 1)
                refuse("a face carries " +
                       std::to_string(f.loops().size()) +
                       " loops - the class signature is the box "
                       "family's single-loop faces (a pierced or "
                       "notched face is a later family's carrier). "
                       "Remedy: fillet a factory box's ring edge.");
            const auto* surf = f.surface();
            fillet_detail::PlaneReader rd;
            const bool planar =
                surf != nullptr &&
                (surf->accept(rd), rd.plane != nullptr);
            if (!planar) {
                fillet_detail::FlankReader fr;
                if (surf == nullptr ||
                    (surf->accept(fr), !fr.any()))
                    refuse("a face carries a surface outside the "
                           "class families (neither the box's plane "
                           "nor a previous fillet's flank - the "
                           "chain's curved faces are CylinderZ, "
                           "CylinderX and CylinderY, the three "
                           "placements this walk and the vertical "
                           "walk born). Remedy: fillet a factory "
                           "box's ring edge.");
                ++flanks;
            }
        }
        if (flanks != chain_k)
            refuse("the body carries " + std::to_string(flanks) +
                   " cylindrical faces where the chain's counts "
                   "declare " + std::to_string(chain_k) +
                   " - the signature's own arithmetic (the flanks "
                   "and the counts must agree). Remedy: fillet a "
                   "factory box's ring edge.");
    }

    // ---- The reference's own find (the vertical walk's, verbatim).
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
                   "non-periodic edge (the box's ring edges are "
                   "single).");
        edge = found[0];
    }

    // ---- THE RING-NESS (the construction's own axis: the flank is a
    // CylinderXSurface or a CylinderYSurface, the m1 placements).
    const auto vpos = [&](std::size_t v) -> const math::Vec3d& {
        return body.vertices[v].position();
    };
    const VertexId ev0 = body.edges[edge].vertex(0);
    const VertexId ev1 = body.edges[edge].vertex(1);
    const math::Vec3d& p0 =
        vpos(static_cast<std::size_t>(ev0));
    const math::Vec3d& p1 =
        vpos(static_cast<std::size_t>(ev1));
    const bool x_par =
        std::abs(p0.y - p1.y) <= tol && std::abs(p0.z - p1.z) <= tol;
    const bool y_par =
        std::abs(p0.x - p1.x) <= tol && std::abs(p0.z - p1.z) <= tol;
    const bool z_par =
        std::abs(p0.x - p1.x) <= tol && std::abs(p0.y - p1.y) <= tol;
    if (z_par)
        refuse("the referenced edge is vertical (the class's own "
               "world is fillet_edge's: the CylinderZ flank, the "
               "ADR-0039 walk). Remedy: reference one of the box's "
               "ring edges (box#F:top0..3 / bottom0..3), or call "
               "fillet_edge for the verticals.");
    if (!x_par && !y_par)
        refuse("the referenced edge is not axis-parallel (its "
               "endpoints differ in more than one coordinate by "
               "more than the declared Linear tolerance " +
               std::to_string(tol) +
               ") - the ring class takes the box family's "
               "axis-aligned horizontal spigoli. Remedy: reference "
               "one of the box's ring edges.");
    const int axis = x_par ? 0 : 1;  // 0: X-parallel, 1: Y-parallel
    // The axial coordinate and the axis's unit direction.
    const auto AX = [&](const math::Vec3d& p) -> double {
        return axis == 0 ? p.x : p.y;
    };
    const math::Vec3d axis_dir = axis == 0
                                      ? math::Vec3d{1, 0, 0}
                                      : math::Vec3d{0, 1, 0};
    const std::size_t v_lo = AX(p0) <= AX(p1)
                                 ? static_cast<std::size_t>(ev0)
                                 : static_cast<std::size_t>(ev1);
    const std::size_t v_hi = AX(p0) <= AX(p1)
                                 ? static_cast<std::size_t>(ev1)
                                 : static_cast<std::size_t>(ev0);
    const math::Vec3d& c_lo = vpos(v_lo);
    const math::Vec3d& c_hi = vpos(v_hi);
    const double a_lo = AX(c_lo), a_hi = AX(c_hi);
    const double ell = a_hi - a_lo;
    if (ell <= tol)
        refuse("the referenced edge is degenerate (its length " +
               std::to_string(ell) + " is within the declared Linear "
               "tolerance) - a zero-length edge has no bisector "
               "locus. Remedy: report the body's construction (the "
               "factory never authors one).");

    // ---- The two adjacent faces, in the edge's own declared
    // co-edge order (the vertical walk's, verbatim).
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
        refuse("an adjacent face is non-planar - the ring class (a) "
               "takes both adjacent faces in the table's world "
               "(planes); a plane x cylinder adjacency is the rim "
               "class's (m3), a flank x flank adjacency the corner "
               "act's (D5.2). Remedy: reference an edge between two "
               "planar faces (the box's ring edges).");
    const math::Vec3d n_A = rd_A.plane->normal_vec();
    const math::Vec3d n_B = rd_B.plane->normal_vec();
    if (n_A.dot(n_B) != 0.0)
        refuse("the adjacent faces' dihedral is not the class's 90 "
               "degrees (the stored normals' dot reads " +
               std::to_string(n_A.dot(n_B)) +
               " where the perpendicular pair reads exactly 0) - "
               "the GENERAL DIHEDRAL family is the same derivation "
               "one parameter wider, REGISTERED for its first "
               "consumer, never smuggled (ADR-0039 D1.1). Remedy: "
               "fillet an orthogonal corner, or await the "
               "general-dihedral morso.");

    // ---- THE CORNER-FREEDOM WALL AND THE CORNER ACT'S TRIGGER
    // (D5.1/D6.1 — the m2 gate; G5 m3, ADR-0043 D5.2/D9's third
    // morso): a previous fillet's flank at a corner borders an
    // edge incident to it (the re-spanned edge, the tangent wire);
    // the naked sequential there would refine cylinder against
    // PERPENDICULAR cylinder — the Steinmetz quartic the stage
    // never consults — and the honest construction is the CORNER
    // ACT (the sphere tangent to the three flanks, the atomic
    // decomposition), its own door since m3. The scan reads both
    // ends: free corners continue the naked walk; a corner
    // occupied by a VERTICAL flank (the CylinderZ placement) fires
    // the act — fillet_ring_edge_corner_act consumes the corner
    // WHOLE (the indivisible pair, D5.1); a corner occupied by a
    // RING flank, or an edge with two occupied corners, stays the
    // wall's refusal (the twelve-spigoli morso's seat, D9's second
    // half).
    std::size_t occ_face = F0;
    std::size_t occ_v = V0;
    for (const std::size_t dead_v : {v_lo, v_hi}) {
        for (std::size_t ei = 0; ei < E0; ++ei) {
            if (ei == edge) continue;
            const auto& e = body.edges[ei];
            if (e.vertex(0) != static_cast<VertexId>(dead_v) &&
                e.vertex(1) != static_cast<VertexId>(dead_v))
                continue;
            for (const CoEdge* ce : e.coedges()) {
                const std::size_t f = face_of_coedge(ce);
                if (f >= F0)
                    refuse("an incident edge's co-user lookup failed "
                           "- report this as a kernel bug.");
                if (f == occ_face && dead_v == occ_v) continue;
                const auto* s =
                    body.solid.shell().faces()[f].surface();
                fillet_detail::PlaneReader rd;
                if (s != nullptr) s->accept(rd);
                if (s == nullptr || rd.plane == nullptr) {
                    if (occ_face == F0) {
                        occ_face = f;
                        occ_v = dead_v;
                        continue;
                    }
                    const auto& occ =
                        body.solid.shell().faces()[f];
                    refuse("the corner at (" +
                           std::to_string(vpos(dead_v).x) + ", " +
                           std::to_string(vpos(dead_v).y) + ", " +
                           std::to_string(vpos(dead_v).z) +
                           ") is occupied by a previous fillet's "
                           "flank (the curved face '" + occ.name() +
                           "' borders an edge incident to it) - the "
                           "ring fillet takes FREE corners only: at "
                           "an occupied corner the second flank would "
                           "have to refine against the first "
                           "(cylinder x perpendicular cylinder, the "
                           "Steinmetz quartic) and the honest "
                           "construction is the CORNER ACT - the "
                           "sphere tangent to the three flanks, the "
                           "atomic decomposition of D5.1 (ADR-0043 "
                           "D5.2/D9). Remedy: fillet ring edges "
                           "whose corners are free (the "
                           "non-adjacent chain), or fillet the "
                           "corner's vertical edge first and let the "
                           "ring pair fire the corner act; the "
                           "ring-occupied corner and the two-corner "
                           "edge ride with the twelve-spigoli "
                           "morso.");
                }
            }
        }
    }
    if (occ_face != F0) {
        fillet_detail::FlankReader fr;
        body.solid.shell().faces()[occ_face].surface()->accept(fr);
        if (fr.z != nullptr)
            return fillet_ring_edge_corner_act(
                body, edge, occ_v, radius, naming, tol, face_A,
                face_B);
        refuse("the corner is occupied by a RING flank "
               "(CylinderX/CylinderY, not the vertical placement) - "
               "the corner act takes the vertical-first chain (the "
               "sphere tangent to one vertical and two ring flanks, "
               "D5.1's own decomposition); the ring-occupied corner "
               "rides with the twelve-spigoli morso (D9's second "
               "half). Remedy: fillet the corner's vertical edge "
               "first (fillet_edge), then the ring pair fires the "
               "act.");
    }


    // ---- The caps: for each dead corner, the face that uses it and
    // is neither adjacent face (the box's own: the side planes at the
    // edge's two ends — the ring walk's "top/bottom", one axis over).
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
    {
        // The cap is a plane (the corner-freedom wall above already
        // guarantees it — every face at a free corner is the box's
        // own; the check stays loud anyway, never a silent deref).
        fillet_detail::PlaneReader rd_lo, rd_hi;
        body.solid.shell().faces()[cap_lo].surface()->accept(rd_lo);
        body.solid.shell().faces()[cap_hi].surface()->accept(rd_hi);
        if (rd_lo.plane == nullptr || rd_hi.plane == nullptr)
            refuse("an end face is non-planar - the class signature "
                   "is the box's. Remedy: report the body's "
                   "construction.");
    }

    // ---- LE BANDE TIPIZZATE (the vertical walk's, one axis over:
    // the inward direction read through the traversal's own axial
    // sense — the stored pair's direction times the co-edge's flag).
    const auto inward_reach = [&](const Face& face,
                                  const PlaneSurface& pl,
                                  const CoEdge* dead_ce) {
        const std::size_t e0v = static_cast<std::size_t>(
            body.edges[edge].vertex(0));
        const std::size_t e1v = static_cast<std::size_t>(
            body.edges[edge].vertex(1));
        const double stored_sense =
            AX(vpos(e1v)) >= AX(vpos(e0v)) ? 1.0 : -1.0;
        const double walk_sense =
            dead_ce->reversed() ? -stored_sense : stored_sense;
        const math::Vec3d t = axis_dir * walk_sense;
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
    // stored operands, ZERO solves (D2.2's contract, one axis over).
    // The cross-plane basis: (y, z) for the X-parallel placement,
    // (x, z) for the Y-parallel; the class's azimuth convention (the
    // m1 frames: the seam +Z for CylinderX, +X for CylinderY, the
    // second axis â x ê₀ making the azimuth CLOCKWISE in the basis
    // order — the atan2 signs below are the classes' own).
    const auto cp = [&](const math::Vec3d& d) -> math::Vec2d {
        return axis == 0 ? math::Vec2d{d.y, d.z}
                         : math::Vec2d{d.x, d.z};
    };
    const auto az = [&](const math::Vec2d& d) -> double {
        return axis == 0 ? std::atan2(-d.x, d.y)
                         : std::atan2(-d.y, d.x);
    };
    // The ball's center on the bisector (the 90° closed form, read
    // in the cross-plane): the corner offset by r along each inward
    // normal — the sum r·(n_A + n_B) IS the bisector locus.
    const math::Vec2d corner2 = cp(c_lo);
    const math::Vec2d n2_A = cp(n_A), n2_B = cp(n_B);
    const math::Vec2d center2{corner2.x -
                                  radius * (n2_A.x + n2_B.x),
                              corner2.y -
                                  radius * (n2_A.y + n2_B.y)};
    // The tangent points: the feet of the perpendiculars from the
    // center onto the two planes, at t = r from the edge, exact.
    const math::Vec2d tan2_A{center2.x + radius * n2_A.x,
                             center2.y + radius * n2_A.y};
    const math::Vec2d tan2_B{center2.x + radius * n2_B.x,
                             center2.y + radius * n2_B.y};
    // The tangent points in 3D (the axial coordinate free — the
    // tangent lines run the edge's own span).
    const auto tan3 = [&](const math::Vec2d& t2,
                          double a) -> math::Vec3d {
        return axis == 0 ? math::Vec3d{a, t2.x, t2.y}
                         : math::Vec3d{t2.x, a, t2.y};
    };
    // The flank's azimuth window: the quarter facing the corner,
    // from one face's tangent to the other's, in the CLASS's own
    // convention (the m1 frames are right-handed; the window never
    // crosses the seam meridian — the tangents sit at the cardinal
    // azimuths, a quarter apart).
    const auto mod2pi = [](double a) {
        while (a < 0.0) a += 2.0 * kPi;
        while (a >= 2.0 * kPi) a -= 2.0 * kPi;
        return a;
    };
    const double uA = az(n2_A), uB = az(n2_B);
    // The perpendicular pair's own discrimination, both flags read
    // BEFORE the refusal: every path to the use below is
    // initialized (the refuse branch throws and never falls
    // through — the structure answers the clang column's
    // -Wsometimes-uninitialized, never a flag).
    const bool uA_leads =
        std::abs(mod2pi(uB - uA) - kPi / 2.0) <=
        kFilletArcSpanDefectBand;  // B rides u1 = u0 + pi/2
    const bool uB_leads =
        std::abs(mod2pi(uA - uB) - kPi / 2.0) <=
        kFilletArcSpanDefectBand;  // A rides u1, B the u0
    if (!uA_leads && !uB_leads)
        refuse("the tangent azimuths are not a perpendicular pair "
               "(the class's own are a quarter apart) - the closed "
               "forms and the walk disagree; report this as a "
               "kernel bug.");
    const bool a_at_u0 = uA_leads;
    const double u0 = a_at_u0 ? uA : uB;
    const double u1 = u0 + kPi / 2.0;
    // The corner-facing quarter, guarded: the bisector's azimuth
    // lies inside the window (the corner is at r·(n_A + n_B) from
    // the axis — the bisector IS the corner direction).
    {
        const double db = mod2pi(az(
            math::Vec2d{n2_A.x + n2_B.x, n2_A.y + n2_B.y}) - u0);
        if (db > kPi / 2.0 + kFilletArcSpanDefectBand &&
            db < 2.0 * kPi - kFilletArcSpanDefectBand)
            refuse("the flank's window does not face the corner (the "
                   "bisector's azimuth reads outside the quarter) - "
                   "the closed forms and the walk disagree; report "
                   "this as a kernel bug.");
    }

    // ---- The maps: the survivors compact (the dead column
    // dropped), the born append (the vertical walk's own table).
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
               "other edges each border one of the two laterals). "
               "Remedy: report the body's construction.");
        return 0;  // unreachable — refuse() throws
    };
    const auto new_end = [&](std::size_t old_end, std::size_t e) {
        if (old_end != v_lo && old_end != v_hi) return vmap[old_end];
        return substitute(old_end, e);
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- The vertices: the survivors, then the born tangent pairs
    // (A first — the co-edge order, the flank's operands' order).
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        R->vertices.push_back(body.vertices[i]);
        R->vertices.back().set_id(static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[4] = {
            tan3(tan2_A, a_lo), tan3(tan2_A, a_hi),
            tan3(tan2_B, a_lo), tan3(tan2_B, a_hi)};
        const std::size_t ids[4] = {v_TA_lo, v_TA_hi, v_TB_lo,
                                    v_TB_hi};
        for (std::size_t i = 0; i < 4; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the endpoints remapped), then
    // the born four (the tangent lines lo->hi along the axis; the
    // quarter arcs from the second face's tangent to the first's —
    // the deterministic B->A convention, the vertical walk's own).
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

    // ---- The faces (the vertical walk's re-authoring, verbatim in
    // every part the axis does not name: per co-edge COPY, RE-SPAN
    // or NEW, every p-curve projected through the face's own native
    // frame — the decoupling rule).
    const auto pos3 = [&](std::size_t v) -> const math::Vec3d& {
        return R->vertices[v].position();
    };
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
                // direction (the axial sense of its traversal —
                // near endpoint's slot, the factory's own authoring
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
                    AX(vpos(d_start)) <= AX(vpos(d_end)) ? lo_id
                                                         : hi_id;
                const std::size_t to =
                    AX(vpos(d_start)) <= AX(vpos(d_end)) ? hi_id
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
            if (e_old == dead_v) {
                const auto& next = ces[(j + 1) % ces.size()];
                const std::size_t n_old = static_cast<std::size_t>(
                    next.reversed()
                        ? body.edges[next.edge()].vertex(1)
                        : body.edges[next.edge()].vertex(0));
                const std::size_t n_new =
                    new_end(n_old, next.edge());
                const std::size_t here =
                    new_end(e_old, ce.edge());
                // The quarter arc in the cap's own domain: center
                // the FLANK AXIS's foot at the dead corner's own
                // axial quota, radius r, the CCW span from the
                // walk's start to its end (the material on the
                // disc side).
                const math::Vec2d a2 =
                    fillet_detail::dom_of(pl, pos3(here));
                const math::Vec2d b2 =
                    fillet_detail::dom_of(pl, pos3(n_new));
                const math::Vec3d foot3 =
                    axis == 0
                        ? math::Vec3d{AX(vpos(dead_v)), center2.x,
                                      center2.y}
                        : math::Vec3d{center2.x,
                                      AX(vpos(dead_v)), center2.y};
                const math::Vec2d c2 =
                    fillet_detail::dom_of(pl, foot3);
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

    reauthor_lateral(face_A, e_tan_A, *rd_A.plane);
    reauthor_lateral(face_B, e_tan_B, *rd_B.plane);
    reauthor_cap(cap_lo, v_lo, e_arc_lo);
    reauthor_cap(cap_hi, v_hi, e_arc_hi);
    for (std::size_t f = 0; f < F0; ++f) {
        if (f == face_A || f == face_B || f == cap_lo ||
            f == cap_hi)
            continue;
        const Face& src = body.solid.shell().faces()[f];
        // The kept faces copy verbatim — the surface carried in its
        // OWN family (the chain's flanks ride through as their
        // native walls: CylinderZ, CylinderX or CylinderY,
        // value-copied exactly as the planes are — never re-read
        // through a plane reader).
        std::shared_ptr<SurfaceGeometry> kept_surf;
        {
            fillet_detail::PlaneReader rd;
            src.surface()->accept(rd);
            if (rd.plane != nullptr)
                kept_surf = std::make_shared<PlaneSurface>(*rd.plane);
            else {
                fillet_detail::FlankReader fr;
                src.surface()->accept(fr);
                if (fr.z != nullptr)
                    kept_surf =
                        std::make_shared<CylinderZSurface>(*fr.z);
                else if (fr.x != nullptr)
                    kept_surf =
                        std::make_shared<CylinderXSurface>(*fr.x);
                else if (fr.y != nullptr)
                    kept_surf =
                        std::make_shared<CylinderYSurface>(*fr.y);
                else
                    refuse("a kept face's surface is outside the "
                           "admission's families - report this as a "
                           "kernel bug.");
            }
        }
        Face face(std::move(kept_surf));
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

    // ---- The flank is born: the placement's own class (X-parallel
    // → CylinderXSurface, Y-parallel → CylinderYSurface — the m1
    // classes, right-handed by construction, NO sign knob), the
    // walk CCW in the domain (the material on the left, the frame
    // rule): the rectangle (u0,0) -> (u1,0) -> (u1,ell) ->
    // (u0,ell) — the low arc, the first tangent (at u1), the high
    // arc, the second tangent (at u0) — every p-curve a Line2 in
    // the cylinder's own domain: the flank patch IS the
    // rectangle-window class the mass machinery already integrates.
    {
        std::shared_ptr<SurfaceGeometry> surf;
        if (axis == 0)
            surf = std::make_shared<CylinderXSurface>(
                radius, a_lo, center2.x, center2.y);
        else
            surf = std::make_shared<CylinderYSurface>(
                radius, a_lo, center2.x, center2.y);
        const std::size_t v_u0_lo = a_at_u0 ? v_TA_lo : v_TB_lo;
        const std::size_t v_u0_hi = a_at_u0 ? v_TA_hi : v_TB_hi;
        const std::size_t v_u1_lo = a_at_u0 ? v_TB_lo : v_TA_lo;
        const std::size_t v_u1_hi = a_at_u0 ? v_TB_hi : v_TA_hi;
        const std::size_t e_u1_tan = a_at_u0 ? e_tan_B : e_tan_A;
        const std::size_t e_u0_tan = a_at_u0 ? e_tan_A : e_tan_B;
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
        // The flank's name: fil#F:flank[<A> × <B>] — the SAME
        // grammar as the verticals' (D2.5: "nessuna estensione
        // necessaria"), the operands the referenced edge's adjacent
        // faces, in the co-edge order.
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
            // level (the vertical walk's own wires, verbatim).
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {{e_tan_A, &src_A}, {e_tan_B, &src_B}};
            for (const auto& w : wires) {
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

    // ---- THE OUTPUT GATE (the vertical walk's identities, hard
    // postconditions).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "fillet_ring_edge: postcondition failed - a loop does not "
            "chain (an assembly defect in the fillet's re-authored "
            "walks; report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "fillet_ring_edge: postcondition failed - the closed-"
            "orientable contract broke (every edge exactly two "
            "opposite co-edges; report this as a kernel bug)");
    if (R->vertex_count() != V0 - 2 + 4 ||
        R->edge_count() != E0 - 1 + 4 ||
        R->face_count() != F0 + 1)
        throw std::runtime_error(
            "fillet_ring_edge: postcondition failed - the counts "
            "moved (the class's own arithmetic is V-2+4 / E-1+4 / "
            "F+1; report this as a kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(body))
        throw std::runtime_error(
            "fillet_ring_edge: postcondition failed - chi moved (the "
            "fillet is chi-flat by construction, Delta-chi = "
            "+2 -3 +1 = 0; report this as a kernel bug)");
    return R;
}

// ---------------------------------------------------------------------------
// fillet_ring_act — THE RING AND CORNERS ACT, INDIVISIBLE (G5 m4,
// ADR-0043 D5.1's second passage, D9's fourth morso): the cube's
// eight ring edges and eight corner spheres consumed in ONE
// transaction. The order of consumption is the derivation (D5.1):
// the vertical passage MUST come first (the vertical walk's
// capitelli are the living top/bottom planes), and the ring CANNOT
// be taken sequentially — the second flank at an occupied corner
// would refine cylinder against PERPENDICULAR cylinder (the
// Steinmetz quartic, D6.1's wall) — while a corner sphere without
// its three flanks has an unbounded patch and a ring flank without
// its two spheres has untyped ends: the HALF act is a body the gate
// refuses, and that IS the indivisibility. So this act lands the
// whole remaining cube at once:
//
//   * THE EIGHT SPHERES (D5.2) at the centers (±(1−r), ±(1−r),
//     ±(1−r)) — the octant windows on the EXISTING SphereSurface
//     family, the SAME frame class the m3 act landed (pole the
//     shared plane's outward normal — the equator is the vertical
//     flank's coaxial circle — azimuth the x-plane's normal, the
//     second axis the product; the walk CLOCKWISE in the domain,
//     the family's inward cross-product normal mirrored, the m3
//     pin), the three arcs per sphere: the equator (the vertical
//     flank's coaxial quarter) and the two meridians (the ring
//     flanks' coaxial quarters), the three tangency points (the
//     feet of the perpendiculars from C to the three planes —
//     D7.4's golden positions).
//   * THE EIGHT RING FLANKS born REFINED AT BOTH ENDS (D5.1): the
//     m2/m3 closed forms one axis over, the azimuth window the
//     quarter facing the corner, the axial window the FULL
//     [−(1−r), (1−r)] — never a cap arc: both ends close at the
//     coaxial planes where the spheres take over.
//   * THE SIX PLANES re-authored as the RECTANGLES of the tangent
//     lines (D5.3): [−(1−r), (1−r)]² in each plane, the four
//     corners the spheres' tangency points (the dead cap arcs'
//     centers — the closure of the construction).
//   * THE FOUR VERTICAL FLANKS re-windowed AGAIN (D6.3): both cap
//     arcs dead (replaced by the corner spheres' equators — the
//     cause verbatim), the two tangent lines shortened to the
//     coaxial planes z = ±(1−r): the face SURVIVES with its name
//     and its stamp, the tavola's row Absorbed, never
//     Born-sotto-falso.
//
// THE ARITHMETIC (D7.1's pre-pinned verdict): from the four-vertical
// state 16/24/10 (the formula 8+2k/12+3k/6+k at k=4): the sixteen
// junction vertices all die (the tangent lines' ends on the
// capitelli), the eight ring edges and the eight cap arcs die, the
// eight vertical tangent lines are absorbed, the ten faces are
// absorbed; born: twenty-four tangency vertices, sixteen tangent
// lines, twenty-four arcs, eight ring flanks, eight spheres —
// 24/48/26, H = 0, χ = 2 (D2.3's own census: 96 incidences / 2).
//
// THE WITNESS (D3's thesis): every quantity above is a CLOSED FORM
// of the stored operands — the sphere centers from the three
// planes, the tangency points as the feet of the perpendiculars,
// the arcs as the quarters of the coaxial contact circles, the
// planes' rectangles from the tangent lines. ZERO intersection
// solves, ZERO SSI consults (the quartic never touched, zero
// ssi.hpp/ssi.cpp lines in the diff), the windows stop at the
// coaxial planes and the flanks never meet each other.
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> fillet_ring_act(
    const BRepBody& body, double radius,
    const FilletNaming& naming, double tol) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("fillet_ring_act: " + what);
    };

    // ---- THE RADIUS'S OWN GATE, then THE STATE'S (the order of
    // consumption is the derivation: this act takes the cube AFTER
    // the four vertical fillets — the formula 8+2k/12+3k/6+k at
    // k = 4; anything else is the naked sequential that dies at the
    // corner, and the refusal names the door it came from).
    if (!(radius > 0.0) || !std::isfinite(radius))
        refuse("the radius must be finite and > 0, got " +
               std::to_string(radius) +
               " — a non-positive ball has no envelope and no "
               "tangent lines. Remedy: author a positive radius.");
    if (V0 != 16 || E0 != 24 || F0 != 10)
        refuse("the ring act takes the cube after the FOUR VERTICAL "
               "FILLETS (the consumption order of D5.1): the state "
               "is V=" +
               std::to_string(V0) + " / E=" + std::to_string(E0) +
               " / F=" + std::to_string(F0) +
               " where the chain formula 8+2k/12+3k/6+k at k=4 "
               "reads 16/24/10 — the naked sequential on the ring "
               "dies at the occupied corner (the Steinmetz quartic, "
               "D6.1) and the corner act of m3 takes ONE corner "
               "only. Remedy: fillet the four verticals first "
               "(fillet_edge, gated at every step), then this act; "
               "for a single corner use fillet_ring_edge (the m3 "
               "door).");
    // ---- THE TANGENTIAL COLLAPSE (D7.2's own wall): at r = 1 the
    // core degenerates (a = 0), the strips die and the closed form
    // sees the inscribed sphere — the LIMIT is the formula's
    // coherence, never a body: the strips must survive.
    if (2.0 * radius >= 2.0 - tol)
        refuse("the tangential collapse: the radius " +
               std::to_string(radius) +
               " empties the strips (the core's side a = 2-2r "
               "degenerates and the six planes die with it) — the "
               "closed form of D7.2 sees the limit r=1 as the "
               "inscribed SPHERE (4/3)*pi, the BODY does not (the "
               "plane faces would have no interior). Remedy: author "
               "a radius below 1 (the strip condition r_i + r_j < "
               "the face's extension, D2.4).");

    // ---- THE STRUCTURAL CENSUS (the box family at k = 4, the
    // same trust the vertical walk places in its own signature:
    // the producers are the factory and the gated chain).
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
                   "consumes EXACT bodies only. Remedy: author the "
                   "body on the exact path (the factories).");
        if (euler_characteristic(body) != 2 ||
            !is_closed_orientable(body) || !loops_are_closed(body))
            refuse("the body is not a closed orientable "
                   "single-shell body - the act's signature is the "
                   "box family chain. Remedy: report the body's "
                   "construction.");
        for (const auto& f : body.solid.shell().faces())
            if (f.loops().size() != 1)
                refuse("a face carries more than one loop - the "
                       "box family chain's faces are simply "
                       "connected. Remedy: report the body's "
                       "construction.");
    }
    const auto vpos = [&](std::size_t v) -> const math::Vec3d& {
        return body.vertices[v].position();
    };
    const auto face_of_coedge = [&](const CoEdge* ce) -> std::size_t {
        for (std::size_t f = 0; f < F0; ++f)
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& other : loop.coedges())
                    if (&other == ce) return f;
        return F0;
    };
    const auto plane_of = [&](std::size_t f) -> const PlaneSurface* {
        fillet_detail::PlaneReader rd;
        const auto* s = body.solid.shell().faces()[f].surface();
        if (s != nullptr) s->accept(rd);
        return rd.plane;
    };
    const auto z_flank_of = [&](std::size_t f)
        -> const CylinderZSurface* {
        fillet_detail::FlankReader fr;
        const auto* s = body.solid.shell().faces()[f].surface();
        if (s != nullptr) s->accept(fr);
        return fr.z;
    };

    // ---- THE FACE CENSUS: six planes + four CylinderZ, the radii
    // COHERENT (D6.6's own name: the eight spheres exist only at
    // the common radius).
    std::vector<std::size_t> z_faces, plane_faces;
    for (std::size_t f = 0; f < F0; ++f) {
        if (plane_of(f) != nullptr) plane_faces.push_back(f);
        else if (z_flank_of(f) != nullptr) z_faces.push_back(f);
        else
            refuse("face " + std::to_string(f) +
                   " is neither a plane nor a CylinderZ flank - "
                   "the signature is the box family chain at k=4. "
                   "Remedy: report the body's construction.");
    }
    if (plane_faces.size() != 6 || z_faces.size() != 4)
        refuse("the face census reads " +
               std::to_string(plane_faces.size()) + " planes and " +
               std::to_string(z_faces.size()) +
               " CylinderZ flanks (the class's own are six and "
               "four). Remedy: report the body's construction.");
    for (const std::size_t f : z_faces) {
        const CylinderZSurface* z = z_flank_of(f);
        if (z->radius() != radius)
            refuse("the corner radii disagree "
                   "(fillet_corner_radii_mismatch): the requested " +
                   std::to_string(radius) +
                   " against the vertical flank's " +
                   std::to_string(z->radius()) +
                   " - the corner spheres are tangent to the three "
                   "flanks only at their COMMON radius (the coaxial "
                   "circles close the windows exactly there, "
                   "D6.6/D2.4). Remedy: fillet the whole cube at "
                   "the same radius (the mixed-radius chain is the "
                   "partial world of D7.3, never this act).");
    }

    // ---- THE EDGE CENSUS: eight vertical tangent lines, eight
    // cap arcs, eight ring edges — every edge classified, the
    // counts pinned (the classification itself the wall against
    // the counts-matching impostors).
    std::vector<std::size_t> ring_edges, cap_arcs, z_tangents;
    for (std::size_t ei = 0; ei < E0; ++ei) {
        const auto& e = body.edges[ei];
        const std::size_t a = static_cast<std::size_t>(e.vertex(0));
        const std::size_t b = static_cast<std::size_t>(e.vertex(1));
        const math::Vec3d& pa = vpos(a);
        const math::Vec3d& pb = vpos(b);
        // The co-users' kinds.
        const auto& ces = e.coedges();
        if (ces.size() != 2)
            refuse("edge " + std::to_string(ei) + " carries " +
                   std::to_string(ces.size()) +
                   " co-edges - the chain's edges are radial "
                   "twos. Remedy: report the body's construction.");
        const std::size_t f0 = face_of_coedge(ces[0]);
        const std::size_t f1 = face_of_coedge(ces[1]);
        const bool f0_z = z_flank_of(f0) != nullptr;
        const bool f1_z = z_flank_of(f1) != nullptr;
        const bool f0_plane = plane_of(f0) != nullptr;
        const bool f1_plane = plane_of(f1) != nullptr;
        const bool vertical =
            std::abs(pa.x - pb.x) <= tol &&
            std::abs(pa.y - pb.y) <= tol;
        const bool horizontal =
            std::abs(pa.z - pb.z) <= tol;
        if (f0_z && f1_plane && vertical) z_tangents.push_back(ei);
        else if (f0_plane && f1_z && vertical)
            z_tangents.push_back(ei);
        else if ((f0_z && f1_plane) || (f0_plane && f1_z))
            cap_arcs.push_back(ei);
        else if (f0_plane && f1_plane && horizontal &&
                 (std::abs(pa.x - pb.x) <= tol ||
                  std::abs(pa.y - pb.y) <= tol))
            ring_edges.push_back(ei);
        else
            refuse("edge " + std::to_string(ei) +
                   " is outside the chain's three kinds (the "
                   "vertical tangent, the cap arc, the ring edge) "
                   "- the signature is the box family chain at "
                   "k=4. Remedy: report the body's construction.");
    }
    if (z_tangents.size() != 8 || cap_arcs.size() != 8 ||
        ring_edges.size() != 8)
        refuse("the edge census reads " +
               std::to_string(z_tangents.size()) + " tangent " +
               "lines, " + std::to_string(cap_arcs.size()) +
               " cap arcs and " + std::to_string(ring_edges.size()) +
               " ring edges (the class's own are eight each). "
               "Remedy: report the body's construction.");

    // ---- THE CLOSED FORMS' OWN TABLES. The eight corners in a
    // deterministic order (the sign triples, + first); the six
    // planes indexed by (axis, sign); the ring edges read with
    // their two planes and their axis.
    const auto sgn = [](double v) -> int {
        return v >= 0.0 ? 1 : -1;
    };
    // The corners: c = 4*(σx<0) + 2*(σy<0) + (σz<0).
    struct CornerInfo {
        int sx, sy, sz;
        math::Vec3d C{};
        math::Vec3d T_x{}, T_y{}, T_z{};
        std::size_t v_Tx{0}, v_Ty{0}, v_Tz{0};
        std::size_t face_x{0}, face_y{0}, face_z{0};
        std::size_t e_eq{0}, e_mer_x{0}, e_mer_y{0};
        std::size_t f_sphere{0};
        double u_third{0.0}, u_s0{0.0}, u_s1{0.0};
    };
    CornerInfo corners[8];
    for (int c = 0; c < 8; ++c) {
        CornerInfo& ci = corners[c];
        ci.sx = (c & 4) ? -1 : 1;
        ci.sy = (c & 2) ? -1 : 1;
        ci.sz = (c & 1) ? -1 : 1;
    }
    // The planes indexed (axis, sign): plane_at[axis][si].
    std::size_t plane_at[3][2] = {{F0, F0}, {F0, F0}, {F0, F0}};
    double plane_coord[3][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    for (const std::size_t f : plane_faces) {
        const PlaneSurface* pl = plane_of(f);
        const math::Vec3d n = pl->normal_vec();
        const int axis = std::abs(n.x) > 0.5
                             ? 0
                             : (std::abs(n.y) > 0.5 ? 1 : 2);
        const int si = (n[axis] > 0.0) ? 1 : 0;
        if (plane_at[axis][si] != F0)
            refuse("two planes share the same axis and side - the "
                   "signature is the box family. Remedy: report "
                   "the body's construction.");
        plane_at[axis][si] = f;
        // The plane's own coordinate on its axis (the payload's,
        // never the world's: a translated or resized cube is the
        // same cube to the closed forms).
        plane_coord[axis][si] = n.dot(pl->origin()) / n[axis];
    }
    for (int k = 0; k < 3; ++k)
        if (plane_at[k][0] == F0 || plane_at[k][1] == F0)
            refuse("a plane's axis census is incomplete - the "
                   "signature is the box family. Remedy: report "
                   "the body's construction.");
    // ---- THE BOX'S OWN FRAME, derived from the six planes (the
    // m3 act's own derivation, one act over): the center and the
    // half-extent per axis, the cube checked (the family's law,
    // D1: the cube, all of it, nothing else). The closed forms
    // below ride THESE — the world origin is nobody's coordinate
    // (the acceptance's translated-cube audit caught the old
    // ±1/±s readings: every corner sat at the unit box's spots).
    math::Vec3d center{0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k)
        center[k] =
            0.5 * (plane_coord[k][0] + plane_coord[k][1]);
    const double ext_half[3] = {
        0.5 * (plane_coord[0][1] - plane_coord[0][0]),
        0.5 * (plane_coord[1][1] - plane_coord[1][0]),
        0.5 * (plane_coord[2][1] - plane_coord[2][0])};
    if (std::abs(ext_half[0] - ext_half[1]) > tol ||
        std::abs(ext_half[1] - ext_half[2]) > tol)
        refuse("the six planes are not a cube's (the half-extents "
               "read " + std::to_string(ext_half[0]) + ", " +
               std::to_string(ext_half[1]) + ", " +
               std::to_string(ext_half[2]) +
               ") - the act's family is the CUBE (ADR-0043 D1), "
               "the box of equal sides. Remedy: fillet a cube.");
    const double ext = ext_half[0];
    const double s = ext - radius;
    // The corners' closed forms, payload-derived: the sphere at
    // the corner offset r inward along each plane's outward
    // normal, the tangency points the feet of the perpendiculars.
    for (int c = 0; c < 8; ++c) {
        CornerInfo& ci = corners[c];
        ci.C = {center.x + ci.sx * s, center.y + ci.sy * s,
                center.z + ci.sz * s};
        ci.T_x = {center.x + ci.sx * ext, ci.C.y, ci.C.z};
        ci.T_y = {ci.C.x, center.y + ci.sy * ext, ci.C.z};
        ci.T_z = {ci.C.x, ci.C.y, center.z + ci.sz * ext};
    }
    for (int c = 0; c < 8; ++c) {
        CornerInfo& ci = corners[c];
        ci.face_x = plane_at[0][ci.sx > 0 ? 1 : 0];
        ci.face_y = plane_at[1][ci.sy > 0 ? 1 : 0];
        ci.face_z = plane_at[2][ci.sz > 0 ? 1 : 0];
    }
    // The ring edges read: the axis, the two planes (one ⊥z, one
    // lateral), the corners at the two ends (the sign along the
    // axis), the flank's placement data.
    struct RingInfo {
        std::size_t edge{0};
        int axis{-1};  // 0: x̂-parallel, 1: ŷ-parallel
        std::size_t face_z{0}, face_lat{0};
        int sz{0}, slat{0};       // the planes' signs
        int s_lat_axis{0};        // which axis the lateral is (0/1)
        std::size_t corner_lo{0}, corner_hi{0};
        std::size_t e_tan_z{0}, e_tan_lat{0};
        std::size_t f_flank{0};
        double u0{0.0}, u1{0.0};
        bool a_at_u0{false};      // the ⊥z plane rides u0
    };
    RingInfo rings[8];
    const auto mod2pi = [](double a) {
        while (a < 0.0) a += 2.0 * kPi;
        while (a >= 2.0 * kPi) a -= 2.0 * kPi;
        return a;
    };
    if (ring_edges.size() != 8)
        refuse("the ring census is not eight - report this as a "
               "kernel bug.");
    for (std::size_t ri = 0; ri < 8; ++ri) {
        RingInfo& r = rings[ri];
        r.edge = ring_edges[ri];
        const auto& e = body.edges[r.edge];
        const math::Vec3d& pa =
            vpos(static_cast<std::size_t>(e.vertex(0)));
        const math::Vec3d& pb =
            vpos(static_cast<std::size_t>(e.vertex(1)));
        const bool x_par = std::abs(pa.y - pb.y) <= tol &&
                           std::abs(pa.z - pb.z) <= tol;
        r.axis = x_par ? 0 : 1;
        const auto& ces = e.coedges();
        const std::size_t fa = face_of_coedge(ces[0]);
        const std::size_t fb = face_of_coedge(ces[1]);
        for (const std::size_t f : {fa, fb}) {
            const PlaneSurface* pl = plane_of(f);
            if (pl == nullptr)
                refuse("a ring edge borders a non-planar face - "
                       "the signature is the box family. Remedy: "
                       "report the body's construction.");
            const math::Vec3d n = pl->normal_vec();
            if (std::abs(n.z) > 0.5) {
                r.face_z = f;
                r.sz = static_cast<int>(n.z > 0.0 ? 1 : -1);
            } else if (std::abs(n.x) > 0.5) {
                r.face_lat = f;
                r.slat = static_cast<int>(n.x > 0.0 ? 1 : -1);
                r.s_lat_axis = 0;
            } else {
                r.face_lat = f;
                r.slat = static_cast<int>(n.y > 0.0 ? 1 : -1);
                r.s_lat_axis = 1;
            }
        }
        if (r.axis == r.s_lat_axis)
            refuse("a ring edge's lateral plane is parallel to its "
                   "own axis - the signature is the box family. "
                   "Remedy: report the body's construction.");
        // The corners at the ends: the sign along the axis at each
        // endpoint (the endpoints sit at center ± (ext−r) along
        // the axis — the payload's frame, never the world's).
        // The corner index at each endpoint, from the signs of the
        // endpoint's POSITION (the junction's three signs ARE the
        // corner's, the junctions being the re-spanned tips).
        const auto corner_index_of_pos =
            [&](const math::Vec3d& p) -> int {
            return (sgn(p.x - center.x) < 0 ? 4 : 0) +
                   (sgn(p.y - center.y) < 0 ? 2 : 0) +
                   (sgn(p.z - center.z) < 0 ? 1 : 0);
        };
        // The edge's own direction: the endpoint on the axis'
        // NEGATIVE side is the walk's first (a deterministic
        // corner_lo/corner_hi for the born geometry).
        const math::Vec3d& p_lo =
            (r.axis == 0 ? pa.x - center.x : pa.y - center.y)
                    < 0.0
                ? pa
                : pb;
        const math::Vec3d& p_hi =
            (r.axis == 0 ? pb.x - center.x : pb.y - center.y)
                    < 0.0
                ? pa
                : pb;
        r.corner_lo = corner_index_of_pos(p_lo);
        r.corner_hi = corner_index_of_pos(p_hi);
        // The azimuth window (the m2/m3 discrimination: the ⊥z
        // plane and the lateral are a perpendicular pair in the
        // flank's cross-plane).
        const auto az = [&](int axis_, double dx, double dy) {
            return axis_ == 0 ? std::atan2(-dx, dy)
                              : std::atan2(-dy, dx);
        };
        double n2_zx, n2_zy, n2_lx, n2_ly;
        if (r.axis == 0) {
            // cross-plane (y, z): the z-plane's normal is ±ẑ →
            // (0, ±1); the lateral (⊥y) → (±1, 0).
            n2_zx = 0.0;
            n2_zy = r.sz > 0 ? 1.0 : -1.0;
            n2_lx = r.s_lat_axis == 1 ? static_cast<double>(r.slat)
                                      : 0.0;
            n2_ly = 0.0;
        } else {
            // cross-plane (x, z).
            n2_zx = 0.0;
            n2_zy = r.sz > 0 ? 1.0 : -1.0;
            n2_lx = r.s_lat_axis == 0 ? static_cast<double>(r.slat)
                                      : 0.0;
            n2_ly = 0.0;
        }
        const double uA = az(r.axis, n2_zx, n2_zy);
        const double uB = az(r.axis, n2_lx, n2_ly);
        const bool uA_leads =
            std::abs(mod2pi(uB - uA) - kPi / 2.0) <=
            kFilletArcSpanDefectBand;
        const bool uB_leads =
            std::abs(mod2pi(uA - uB) - kPi / 2.0) <=
            kFilletArcSpanDefectBand;
        if (!uA_leads && !uB_leads)
            refuse("a ring flank's tangent azimuths are not a "
                   "perpendicular pair - the closed forms and the "
                   "walk disagree; report this as a kernel bug.");
        r.a_at_u0 = uA_leads;
        r.u0 = uA_leads ? uA : uB;
        r.u1 = r.u0 + kPi / 2.0;
    }

    // ---- THE MAPS: no vertex survives (the sixteen junctions all
    // die at the spheres' tangent points), the eight vertical
    // tangent lines survive (absorbed, shortened), the eight ring
    // edges and the eight cap arcs die.
    std::vector<std::size_t> vmap(V0, V0), emap(E0, E0);
    {
        std::size_t next = 0;
        for (std::size_t i = 0; i < E0; ++i) {
            bool dead = false;
            for (const std::size_t r : ring_edges)
                if (r == i) dead = true;
            for (const std::size_t a : cap_arcs)
                if (a == i) dead = true;
            if (!dead) emap[i] = next++;
        }
    }
    // The born ids: 24 vertices (three tangency points per corner),
    // 40 edges (16 ring tangent lines + 24 arcs), 16 faces (8 ring
    // flanks + 8 spheres).
    for (int c = 0; c < 8; ++c) {
        corners[c].v_Tx = static_cast<std::size_t>(3 * c + 0);
        corners[c].v_Ty = static_cast<std::size_t>(3 * c + 1);
        corners[c].v_Tz = static_cast<std::size_t>(3 * c + 2);
    }
    const std::size_t e_tan_base = E0 - 16;
    for (std::size_t ri = 0; ri < 8; ++ri) {
        rings[ri].e_tan_z = e_tan_base + 2 * ri;
        rings[ri].e_tan_lat = e_tan_base + 2 * ri + 1;
    }
    const std::size_t e_arc_base = E0 - 16 + 16;
    for (int c = 0; c < 8; ++c) {
        corners[c].e_eq = e_arc_base + 3 * c + 0;
        corners[c].e_mer_x = e_arc_base + 3 * c + 1;
        corners[c].e_mer_y = e_arc_base + 3 * c + 2;
    }
    for (std::size_t ri = 0; ri < 8; ++ri)
        rings[ri].f_flank = F0 + ri;
    for (int c = 0; c < 8; ++c)
        corners[c].f_sphere = F0 + 8 + static_cast<std::size_t>(c);
    // The tangency-point vertex OF A PLANE at a corner: the foot on
    // the plane with the given axis.
    const auto t_of_plane_axis = [&](int c, int axis)
        -> std::size_t {
        return axis == 0 ? corners[c].v_Tx
               : (axis == 1 ? corners[c].v_Ty : corners[c].v_Tz);
    };
    // The corner index of a JUNCTION (the old endpoint's three
    // signs ARE the corner's).
    const auto corner_of_pos = [&](const math::Vec3d& p) -> int {
        return (sgn(p.x - center.x) < 0 ? 4 : 0) +
               (sgn(p.y - center.y) < 0 ? 2 : 0) +
               (sgn(p.z - center.z) < 0 ? 1 : 0);
    };
    // The NEW endpoint of an old junction: the tangency point on
    // the axis' plane of the junction's corner (the junction on
    // the plane (axis, σ) maps to T_axis(corner(junction))).
    const auto new_end = [&](std::size_t old_v, int axis) {
        const int c = corner_of_pos(vpos(old_v));
        return t_of_plane_axis(c, axis);
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- THE VERTICES: the twenty-four tangency points (D7.4's
    // golden positions, the feet of the perpendiculars).
    for (int c = 0; c < 8; ++c) {
        const math::Vec3d born[3] = {corners[c].T_x, corners[c].T_y,
                                     corners[c].T_z};
        const std::size_t ids[3] = {corners[c].v_Tx,
                                    corners[c].v_Ty,
                                    corners[c].v_Tz};
        for (int k = 0; k < 3; ++k) {
            Vertex v{born[k]};
            v.set_id(static_cast<VertexId>(ids[k]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- THE EDGES: the eight vertical tangent lines SURVIVE,
    // their ends re-spanned from the capitelli junctions (z = ±1)
    // to the coaxial planes' tangency points (z = ±(1−r)) — the
    // same edge, the same two co-edges, the window edit read here
    // on the stored pairs. Then the forty born.
    for (std::size_t i = 0; i < E0; ++i) {
        if (emap[i] == E0) continue;
        R->edges.push_back(body.edges[i]);
        auto& e = R->edges.back();
        e.set_id(static_cast<EdgeId>(emap[i]));
        for (int side = 0; side < 2; ++side) {
            const std::size_t old_end = static_cast<std::size_t>(
                e.vertex(side));
            // The junction's re-spanning axis: the Z tangent lies
            // on ONE lateral plane (read through the radial
            // structure); its ends map to the lateral's own
            // tangency points.
            std::size_t lateral = F0;
            for (const CoEdge* ce : body.edges[i].coedges()) {
                const std::size_t f = face_of_coedge(ce);
                if (plane_of(f) != nullptr &&
                    std::abs(plane_of(f)->normal_vec().z) < 0.5)
                    lateral = f;
            }
            if (lateral == F0)
                refuse("a vertical tangent line borders no lateral "
                       "plane - report this as a kernel bug.");
            const PlaneSurface* pl = plane_of(lateral);
            const int axis = std::abs(pl->normal_vec().x) > 0.5
                                 ? 0
                                 : 1;
            e.set_vertex(side, static_cast<VertexId>(
                                    new_end(old_end, axis)));
        }
    }
    {
        // The sixteen ring tangent lines: per ring edge, the line
        // on the ⊥z plane and the line on the lateral (the stored
        // pairs run from the axis' − side to its + side).
        for (std::size_t ri = 0; ri < 8; ++ri) {
            const RingInfo& r = rings[ri];
            const std::pair<std::size_t, std::size_t> born[2] = {
                {t_of_plane_axis(static_cast<int>(r.corner_lo), 2),
                 t_of_plane_axis(static_cast<int>(r.corner_hi), 2)},
                {t_of_plane_axis(static_cast<int>(r.corner_lo),
                                 r.s_lat_axis),
                 t_of_plane_axis(static_cast<int>(r.corner_hi),
                                 r.s_lat_axis)}};
            const std::size_t ids[2] = {r.e_tan_z, r.e_tan_lat};
            for (int k = 0; k < 2; ++k) {
                Edge e{static_cast<VertexId>(born[k].first),
                       static_cast<VertexId>(born[k].second)};
                e.set_id(static_cast<EdgeId>(ids[k]));
                e.set_provenance(prov);
                R->edges.push_back(e);
            }
        }
        // The twenty-four arcs: per corner the equator (T_x ↔ T_y,
        // the vertical flank's coaxial quarter) and the two
        // meridians (T_z ↔ T_y coassiale with the x̂-parallel
        // flank, T_z ↔ T_x with the ŷ-parallel one — the m3 act's
        // own stored-pair convention).
        for (int c = 0; c < 8; ++c) {
            const CornerInfo& ci = corners[c];
            const std::pair<std::size_t, std::size_t> born[3] = {
                {ci.v_Tx, ci.v_Ty},
                {ci.v_Tz, ci.v_Ty},
                {ci.v_Tz, ci.v_Tx}};
            const std::size_t ids[3] = {ci.e_eq, ci.e_mer_x,
                                        ci.e_mer_y};
            for (int k = 0; k < 3; ++k) {
                Edge e{static_cast<VertexId>(born[k].first),
                       static_cast<VertexId>(born[k].second)};
                e.set_id(static_cast<EdgeId>(ids[k]));
                e.set_provenance(prov);
                R->edges.push_back(e);
            }
        }
    }

    // ---- THE SIX PLANES RE-AUTHORED (D5.3): each walk follows the
    // OLD loop's own order with the edges substituted in place —
    // every ring edge becomes ITS tangent line on this plane, every
    // cap arc becomes the vertical tangent of the flank it
    // belonged to (the line that closes the rectangle's side), the
    // junctions become the spheres' tangency points (the dead arcs'
    // centers — the closure of D5.3's own derivation).
    const auto pos3 = [&](std::size_t v) -> const math::Vec3d& {
        return R->vertices[v].position();
    };
    const auto emit_planar = [&](Loop& loop,
                                 const PlaneSurface& pl,
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
            refuse("the re-authored plane walk lost the chaining: "
                   "the edge " + std::to_string(e_id) +
                   " (stored " +
                   std::to_string(static_cast<std::size_t>(
                       stored.vertex(0))) + " -> " +
                   std::to_string(static_cast<std::size_t>(
                       stored.vertex(1))) +
                   ") does not run between the walk's points (" +
                   std::to_string(a) + " -> " +
                   std::to_string(b) + ") - report this as a "
                   "kernel bug.");
        CoEdge ce{static_cast<EdgeId>(e_id), rev};
        ce.set_p_curve(pcurve_segment(
            fillet_detail::dom_of(pl, pos3(a)),
            fillet_detail::dom_of(pl, pos3(b)),
            /*interior_left=*/true));
        loop.add_coedge(ce);
    };

    for (int k = 0; k < 3; ++k) {
        for (int si = 0; si < 2; ++si) {
            const std::size_t pf = plane_at[k][si];
            const Face& src =
                body.solid.shell().faces()[pf];
            const PlaneSurface& pl = *plane_of(pf);
            Loop loop;
            for (const auto& ce : src.loops()[0].coedges()) {
                const std::size_t e_old = ce.edge();
                const std::size_t s_old = static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[e_old].vertex(1)
                        : body.edges[e_old].vertex(0));
                const std::size_t e_old_end =
                    static_cast<std::size_t>(
                        ce.reversed()
                            ? body.edges[e_old].vertex(0)
                            : body.edges[e_old].vertex(1));
                const int c_from = corner_of_pos(vpos(s_old));
                const int c_to = corner_of_pos(vpos(e_old_end));
                bool is_ring = false;
                const RingInfo* ring = nullptr;
                for (const RingInfo& r : rings)
                    if (r.edge == e_old) {
                        is_ring = true;
                        ring = &r;
                    }
                if (is_ring) {
                    // The ring's tangent on THIS plane: the ⊥z
                    // line on a z-plane, the lateral line on a
                    // lateral.
                    const std::size_t e_new =
                        (k == 2) ? ring->e_tan_z
                                 : ring->e_tan_lat;
                    const int t_axis =
                        (k == 2) ? 2 : ring->s_lat_axis;
                    emit_planar(loop, pl, e_new,
                                t_of_plane_axis(c_from, t_axis),
                                t_of_plane_axis(c_to, t_axis));
                    continue;
                }
                bool is_z_tan = false;
                for (const std::size_t t : z_tangents)
                    if (t == e_old) is_z_tan = true;
                bool is_cap = false;
                for (const std::size_t a : cap_arcs)
                    if (a == e_old) is_cap = true;
                if (is_cap) {
                    // On the z-planes the cap arc COLLAPSES at the
                    // tangency point: the two neighboring ring
                    // tangents MEET there (D5.3's rectangle corner
                    // — the m3 act's own pattern); on the laterals
                    // a cap arc cannot appear (the capitelli are
                    // the z-planes' own loops).
                    if (k != 2)
                        refuse("a lateral plane's loop carries a "
                               "cap arc - the capitelli live on "
                               "the z-planes; report this as a "
                               "kernel bug.");
                    continue;
                }
                if (!is_z_tan)
                    refuse("a plane's co-edge is outside the act's "
                           "kinds - report this as a kernel bug.");
                if (k == 2)
                    refuse("a z-plane's loop carries a vertical "
                           "tangent - report this as a kernel "
                           "bug.");
                // The tangent IS its own survivor: the window edit
                // (the shortened line) read on the stored pair.
                if (emap[e_old] == E0)
                    refuse("the vertical tangent died in the map - "
                           "report this as a kernel bug.");
                const int lat_axis = k;
                emit_planar(loop, pl, emap[e_old],
                            t_of_plane_axis(c_from, lat_axis),
                            t_of_plane_axis(c_to, lat_axis));
            }
            Face face(std::make_shared<PlaneSurface>(pl));
            face.add_loop(std::move(loop));
            face.set_id(static_cast<FaceId>(pf));
            face.set_shape_id(src.shape_id());
            face.set_name(src.name());
            face.set_provenance(src.provenance());
            auto& faces_mut = shell.faces_mut();
            if (faces_mut.size() <= pf) faces_mut.resize(pf + 1);
            faces_mut[pf] = std::move(face);
        }
    }

    // ---- THE FOUR VERTICAL FLANKS RE-WINDOWED (D6.3, the m3
    // edit both ends over): the face SURVIVES with its name and
    // stamp; both cap arcs die (replaced by the corner spheres'
    // equators), the two tangent lines shorten to the coaxial
    // planes z = ±(1−r). The p-curves ride the flank's own
    // parametrization: the tangents keep their u (read from the
    // stored p-curves), the equators take the dead arcs' u-span at
    // the coaxial v — the closed forms' own agreement.
    for (const std::size_t zf : z_faces) {
        const CylinderZSurface* vz = z_flank_of(zf);
        const Face& src =
            body.solid.shell().faces()[zf];
        const auto& ces = src.loops()[0].coedges();
        if (ces.size() != 4)
            refuse("a vertical flank's loop carries " +
                   std::to_string(ces.size()) +
                   " co-edges (the class's own are four) - report "
                   "this as a kernel bug.");
        const int sx = sgn(vz->axis_center().x - center.x);
        const int sy = sgn(vz->axis_center().y - center.y);
        Loop loop;
        for (const auto& ce : ces) {
            const std::size_t e_old = ce.edge();
            const std::size_t s_old = static_cast<std::size_t>(
                ce.reversed()
                    ? body.edges[e_old].vertex(1)
                    : body.edges[e_old].vertex(0));
            const std::size_t e_old_end =
                static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[e_old].vertex(0)
                        : body.edges[e_old].vertex(1));
            bool is_cap = false;
            for (const std::size_t a : cap_arcs)
                if (a == e_old) is_cap = true;
            if (is_cap) {
                // The equator of the corner this arc belongs to
                // (the arc's junctions sit on ONE z side).
                const int szc =
                    sgn(vpos(s_old).z - center.z);
                const int c = (sx < 0 ? 4 : 0) +
                              (sy < 0 ? 2 : 0) +
                              (szc < 0 ? 1 : 0);
                if (ce.p_curve() == nullptr ||
                    ce.p_curve()->kind != PCurve::Kind::Segment)
                    refuse("the vertical flank's cap arc carries "
                           "no segment p-curve - report this as a "
                           "kernel bug.");
                const double ua = ce.p_curve()->segment.a.x;
                const double ub = ce.p_curve()->segment.b.x;
                const double v_eq =
                    center.z + static_cast<double>(szc) * s
                    - vz->z_base();
                // The walk's OWN direction: each arc endpoint sits
                // on ONE lateral plane (the ±1 coordinate that is
                // not z) and maps to THAT plane's tangency point —
                // the equator is emitted along the old walk's
                // sense, never blindly forward.
                const auto touch_of_junction =
                    [&](const math::Vec3d& pj) -> std::size_t {
                    const int ax =
                        std::abs(std::abs(pj.x - center.x) - ext)
                                <= tol
                            ? 0
                            : 1;
                    return t_of_plane_axis(c, ax);
                };
                const std::size_t from =
                    touch_of_junction(vpos(s_old));
                const std::size_t to =
                    touch_of_junction(vpos(e_old_end));
                const auto& stored =
                    R->edges[corners[c].e_eq];
                const bool fwd =
                    stored.vertex(0) ==
                            static_cast<VertexId>(from) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(to);
                const bool rev =
                    stored.vertex(0) ==
                            static_cast<VertexId>(to) &&
                    stored.vertex(1) ==
                        static_cast<VertexId>(from);
                if (!fwd && !rev)
                    refuse("the equator's stored pair does not run "
                           "between the tangency points - report "
                           "this as a kernel bug.");
                CoEdge out{static_cast<EdgeId>(corners[c].e_eq),
                           rev};
                out.set_p_curve(pcurve_segment(
                    fwd ? math::Vec2d{ua, v_eq}
                        : math::Vec2d{ub, v_eq},
                    fwd ? math::Vec2d{ub, v_eq}
                        : math::Vec2d{ua, v_eq},
                    /*interior_left=*/true));
                loop.add_coedge(out);
                continue;
            }
            // A tangent line: shortened to the coaxial planes,
            // the u the stored p-curve's own, the ends the two
            // corners' tangency points on the tangent's lateral.
            if (ce.p_curve() == nullptr ||
                ce.p_curve()->kind != PCurve::Kind::Segment ||
                ce.p_curve()->segment.a.x !=
                    ce.p_curve()->segment.b.x)
                refuse("the vertical flank's tangent line carries "
                       "no u-const segment p-curve - report this "
                       "as a kernel bug.");
            const double u_t = ce.p_curve()->segment.a.x;
            const double v_lo =
                std::min(ce.p_curve()->segment.a.y,
                         ce.p_curve()->segment.b.y);
            const bool a_low =
                ce.p_curve()->segment.a.y <=
                ce.p_curve()->segment.b.y;
            // The lateral this tangent lies on (its plane
            // co-user).
            std::size_t lateral = F0;
            for (const CoEdge* c2 :
                 body.edges[e_old].coedges()) {
                const std::size_t f = face_of_coedge(c2);
                if (plane_of(f) != nullptr)
                    lateral = f;
            }
            if (lateral == F0)
                refuse("a vertical tangent line borders no plane "
                       "- report this as a kernel bug.");
            const int lat_axis =
                std::abs(plane_of(lateral)->normal_vec().x) > 0.5
                    ? 0
                    : 1;
            const int c_from = corner_of_pos(vpos(s_old));
            const int c_to = corner_of_pos(vpos(e_old_end));
            if (emap[e_old] == E0)
                refuse("a vertical tangent died in the map - "
                       "report this as a kernel bug.");
            const auto& stored = R->edges[emap[e_old]];
            const std::size_t from =
                t_of_plane_axis(c_from, lat_axis);
            const std::size_t to =
                t_of_plane_axis(c_to, lat_axis);
            const bool fwd =
                stored.vertex(0) ==
                        static_cast<VertexId>(from) &&
                stored.vertex(1) ==
                    static_cast<VertexId>(to);
            const bool rev =
                stored.vertex(0) ==
                        static_cast<VertexId>(to) &&
                stored.vertex(1) ==
                    static_cast<VertexId>(from);
            if (!fwd && !rev)
                refuse("the shortened tangent's stored pair does "
                       "not run between the tangency points - "
                       "report this as a kernel bug.");
            CoEdge out{static_cast<EdgeId>(emap[e_old]), rev};
            // The v-span: the direction the old p-curve's own
            // (the walk's sense preserved), the ends at the
            // coaxial planes z = ±(1−r).
            const double v_lo_new =
                center.z - s - vz->z_base();
            const double v_hi_new =
                center.z + s - vz->z_base();
            out.set_p_curve(pcurve_segment(
                a_low ? math::Vec2d{u_t, v_lo_new}
                      : math::Vec2d{u_t, v_hi_new},
                a_low ? math::Vec2d{u_t, v_hi_new}
                      : math::Vec2d{u_t, v_lo_new},
                /*interior_left=*/true));
            (void)v_lo;
            loop.add_coedge(out);
        }
        Face face(std::make_shared<CylinderZSurface>(*vz));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(zf));
        face.set_shape_id(src.shape_id());
        face.set_name(src.name());
        face.set_provenance(src.provenance());
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= zf) faces_mut.resize(zf + 1);
        faces_mut[zf] = std::move(face);
    }

    // ---- THE NAMING CONTEXT (the m2/m3 grammar verbatim: the
    // fillet's declaration with the body's binding and the
    // same-feature extension).
    const bool named =
        naming.body != nullptr &&
        body.solid.shell()
                .faces()[plane_at[0][0]]
                .shape_id()
                .subshape.depth > 0;
    naming::OperandBinding fil_operands[2] = {
        {body.solid.shell()
             .faces()[plane_at[0][0]]
             .shape_id()
             .feature,
         naming.body},
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
    const auto flank_ref_of = [&](const Face& fa,
                                  const Face& fb) {
        geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(
                naming::kFilFlankRole);
        ref = leg(ref, fa);
        ref = leg(ref, fb);
        return ref;
    };
    const auto corner_ref_of = [&](const Face& fz,
                                   const Face& fx,
                                   const Face& fy) {
        geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(
                naming::kFilCornerRole);
        ref = leg(ref, fz);
        ref = leg(ref, fx);
        ref = leg(ref, fy);
        return ref;
    };
    const Face& F0_face = body.solid.shell().faces()[0];

    // ---- THE EIGHT RING FLANKS BORN (the m2/m3 closed forms, the
    // axial window the FULL [−(1−r), (1−r)] — both ends close at
    // the coaxial planes where the spheres take over, never a cap
    // arc). The walk CCW in the flank's own domain, the material
    // on the left: (u0, vA) → (u1, vA) the low arc, the tangent at
    // u1, the high arc, the tangent at u0.
    const double v_lo = -s, v_hi = s;
    for (std::size_t ri = 0; ri < 8; ++ri) {
        const RingInfo& r = rings[ri];
        // THE SURFACE'S AXIAL BASE IS THE BOX'S OWN ORIGIN on the
        // flank's axis (the edge's midpoint, v = 0) — the window's
        // ±s bounds are written in THAT coordinate, and the coaxial
        // planes at x = ±s are where the arcs close. A base at v_lo
        // would double-shift the window (evaluate maps v to base + v:
        // [−2s, 0] instead of [−s, +s]) — the p-curves coherent in
        // their own domain but landing HALF A SPAN off the 3D edges,
        // invisible to the divergence-volume (it walks the 3D curves,
        // which are placed right) and fatal to every domain reader
        // (the tessellation, the SSI's p-curve matching, the chain
        // consumers). The eight-corner acceptance's boundary audit
        // caught it: every ring flank's evaluated midpoint sat r·â
        // off its edge's true midpoint.
        std::shared_ptr<SurfaceGeometry> surf;
        if (r.axis == 0)
            surf = std::make_shared<CylinderXSurface>(
                radius, center.x,
                center.y + static_cast<double>(r.slat) * s,
                center.z + static_cast<double>(r.sz) * s);
        else
            surf = std::make_shared<CylinderYSurface>(
                radius, center.y,
                center.x + static_cast<double>(r.slat) * s,
                center.z + static_cast<double>(r.sz) * s);
        // The low/high arc edges and the tangent edges at the two
        // azimuth ends: the walk maps the u0-side tangent to the
        // plane that rides u0 (a_at_u0: the ⊥z plane at u0).
        const int z_axis_of_flank = 2;
        const std::size_t e_u0_tan =
            r.a_at_u0 ? r.e_tan_z : r.e_tan_lat;
        const std::size_t e_u1_tan =
            r.a_at_u0 ? r.e_tan_lat : r.e_tan_z;
        const std::size_t v_u0_lo =
            r.a_at_u0
                ? t_of_plane_axis(static_cast<int>(r.corner_lo), 2)
                : t_of_plane_axis(static_cast<int>(r.corner_lo),
                                 r.s_lat_axis);
        const std::size_t v_u1_lo =
            r.a_at_u0
                ? t_of_plane_axis(static_cast<int>(r.corner_lo),
                                 r.s_lat_axis)
                : t_of_plane_axis(static_cast<int>(r.corner_lo), 2);
        const std::size_t v_u0_hi =
            r.a_at_u0
                ? t_of_plane_axis(static_cast<int>(r.corner_hi), 2)
                : t_of_plane_axis(static_cast<int>(r.corner_hi),
                                 r.s_lat_axis);
        const std::size_t v_u1_hi =
            r.a_at_u0
                ? t_of_plane_axis(static_cast<int>(r.corner_hi),
                                 r.s_lat_axis)
                : t_of_plane_axis(static_cast<int>(r.corner_hi), 2);
        // The arc edges at the two axial ends: which corner's
        // meridian runs at vA (lo) and vB (hi) — the arc at the
        // axis' − end belongs to corner_lo, the + end to corner_hi;
        // the u0-side meridian is the one on the plane that rides
        // u0 (m3's convention: the meridian at the seam's azimuth
        // is the flank's coaxial arc on the ⊥z plane).
        const int c_lo = static_cast<int>(r.corner_lo);
        const int c_hi = static_cast<int>(r.corner_hi);
        const std::size_t e_arc_lo =
            r.axis == 0 ? corners[c_lo].e_mer_x
                        : corners[c_lo].e_mer_y;
        const std::size_t e_arc_hi =
            r.axis == 0 ? corners[c_hi].e_mer_x
                        : corners[c_hi].e_mer_y;
        Loop loop;
        const auto emit = [&](std::size_t e_id, std::size_t a,
                              std::size_t b, double ua, double va,
                              double ub, double vb) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the born ring flank's walk lost the "
                       "chaining - report this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                math::Vec2d{ua, va}, math::Vec2d{ub, vb},
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        (void)z_axis_of_flank;
        emit(e_arc_lo, v_u0_lo, v_u1_lo, r.u0, v_lo, r.u1, v_lo);
        emit(e_u1_tan, v_u1_lo, v_u1_hi, r.u1, v_lo, r.u1, v_hi);
        emit(e_arc_hi, v_u1_hi, v_u0_hi, r.u1, v_hi, r.u0, v_hi);
        emit(e_u0_tan, v_u0_hi, v_u0_lo, r.u0, v_hi, r.u0, v_lo);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(r.f_flank));
        face.set_provenance(prov);
        if (named) {
            const Face& fz =
                body.solid.shell().faces()[r.face_z];
            const Face& fl =
                body.solid.shell().faces()[r.face_lat];
            const geometry::SubshapeRef ref =
                flank_ref_of(fz, fl);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, fil_decl.faces()));
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {
                {r.e_tan_lat, &fl}, {r.e_tan_z, &fz}};
            for (const auto& w : wires) {
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kFilFlankRole);
                wref = leg(wref, fl);
                wref = leg(wref, fz);
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(
                    naming::render_name(naming.feature, wref,
                                        fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE EIGHT SPHERES BORN (D5.2, the m3 act's own class):
    // the octant windows, the frame DERIVED (pole the z-plane's
    // outward normal — the equator is the vertical flank's coaxial
    // circle — azimuth the x-plane's outward normal, the second
    // axis the product), the walk CLOCKWISE in the domain (the
    // family's inward cross-product normal mirrored — the m3 pin).
    for (int c = 0; c < 8; ++c) {
        CornerInfo& ci = corners[c];
        const Face& fz = body.solid.shell().faces()[ci.face_z];
        const Face& fx = body.solid.shell().faces()[ci.face_x];
        const Face& fy = body.solid.shell().faces()[ci.face_y];
        const math::Vec3d pole =
            plane_of(ci.face_z)->normal_vec();
        const math::Vec3d e0_axis =
            plane_of(ci.face_x)->normal_vec();
        const math::Vec3d e1_axis = e0_axis.cross(pole);
        // The y-plane's tangency azimuth (a quarter from the
        // seam).
        const math::Vec3d n_y = plane_of(ci.face_y)->normal_vec();
        ci.u_third = std::atan2(n_y.dot(e1_axis),
                                n_y.dot(e0_axis));
        if (std::abs(std::abs(ci.u_third) - kPi / 2.0) >
            kFilletArcSpanDefectBand)
            refuse("the corner sphere's tangency azimuths are not "
                   "a perpendicular pair - report this as a "
                   "kernel bug.");
        ci.u_s0 = std::min(0.0, ci.u_third);
        ci.u_s1 = std::max(0.0, ci.u_third);
        auto surf = std::make_shared<SphereSurface>(
            radius, ci.C, pole, e0_axis);
        const bool third_is_low = ci.u_third < 0.0;
        // The meridians' seats: u = 0 (the seam, the x-plane's
        // side — the ŷ-parallel flank's coaxial arc) and u =
        // u_third (the x̂-parallel flank's).
        const std::size_t e_mer_low =
            third_is_low ? ci.e_mer_x : ci.e_mer_y;
        const std::size_t e_mer_high =
            third_is_low ? ci.e_mer_y : ci.e_mer_x;
        const std::size_t v_base_low =
            third_is_low ? ci.v_Ty : ci.v_Tx;
        const std::size_t v_base_high =
            third_is_low ? ci.v_Tx : ci.v_Ty;
        Loop loop;
        const auto emit = [&](std::size_t e_id, std::size_t a,
                              std::size_t b, double ua, double va,
                              double ub, double vb) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the sphere's walk lost the chaining - "
                       "report this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                math::Vec2d{ua, va}, math::Vec2d{ub, vb},
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        emit(ci.e_eq, v_base_high, v_base_low, ci.u_s1, kPi / 2.0,
             ci.u_s0, kPi / 2.0);
        emit(e_mer_low, v_base_low, ci.v_Tz, ci.u_s0, kPi / 2.0,
             ci.u_s0, 0.0);
        emit(e_mer_high, ci.v_Tz, v_base_high, ci.u_s1, 0.0,
             ci.u_s1, kPi / 2.0);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(ci.f_sphere));
        face.set_provenance(prov);
        if (named) {
            const geometry::SubshapeRef cref =
                corner_ref_of(fz, fx, fy);
            face.set_shape_id({naming.feature, cref, {}});
            face.set_name(naming::render_name(
                naming.feature, cref, fil_decl.faces()));
            const struct {
                std::size_t edge;
                const Face* f_a;
                const Face* f_b;
                bool vertical;
            } coax[3] = {
                {ci.e_mer_x, &fx, &fy, false},
                {ci.e_mer_y, &fx, &fy, false},
                {ci.e_eq, nullptr, nullptr, true}};
            for (const auto& w : coax) {
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kFilIsectCoaxRole);
                for (std::uint8_t k = 0; k < cref.depth; ++k)
                    wref = wref.push(cref.path[k]);
                if (w.vertical) {
                    // The equator's second leg: the vertical
                    // flank's own stored path.
                    std::size_t vflank_here = F0;
                    for (const std::size_t zf : z_faces) {
                        const CylinderZSurface* z2 =
                            z_flank_of(zf);
                        const int zx =
                            sgn(z2->axis_center().x - center.x);
                        const int zy =
                            sgn(z2->axis_center().y - center.y);
                        if (zx == ci.sx && zy == ci.sy)
                            vflank_here = zf;
                    }
                    if (vflank_here == F0)
                        refuse("the corner's vertical flank was "
                               "not found - report this as a "
                               "kernel bug.");
                    const geometry::SubshapeRef vf =
                        body.solid.shell()
                            .faces()[vflank_here]
                            .shape_id()
                            .subshape;
                    for (std::uint8_t k = 0; k < vf.depth; ++k)
                        wref = wref.push(vf.path[k]);
                } else {
                    wref = wref.push(naming::kFilFlankRole);
                    wref = leg(wref, *w.f_a);
                    wref = leg(wref, *w.f_b);
                }
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(
                    naming::render_name(naming.feature, wref,
                                        fil_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }
    (void)F0_face;

    // ---- THE OUTPUT GATE (the act's own arithmetic, hard
    // postconditions, the named diagnosis the house's culture).
    link_radial_edges(*R);
    if (!loops_are_closed(*R)) {
        std::string diag;
        for (std::size_t f = 0;
             f < R->solid.shell().faces().size() && diag.empty();
             ++f) {
            const auto& loop =
                R->solid.shell().faces()[f].loops()[0];
            const auto& ces = loop.coedges();
            for (std::size_t j = 0; j < ces.size(); ++j) {
                const auto& ce = ces[j];
                const auto& e =
                    R->edges[static_cast<std::size_t>(
                        ce.edge())];
                const std::size_t end_a = static_cast<std::size_t>(
                    ce.reversed() ? e.vertex(0) : e.vertex(1));
                const auto& next = ces[(j + 1) % ces.size()];
                const auto& e2 =
                    R->edges[static_cast<std::size_t>(
                        next.edge())];
                const std::size_t start_b =
                    static_cast<std::size_t>(
                        next.reversed() ? e2.vertex(1)
                                        : e2.vertex(0));
                if (end_a != start_b) {
                    diag = " face " + std::to_string(f) + " ('" +
                           R->solid.shell().faces()[f].name() +
                           "'): co-edge " + std::to_string(j) +
                           " ends at " + std::to_string(end_a) +
                           " but the next starts at " +
                           std::to_string(start_b);
                    break;
                }
            }
        }
        throw std::runtime_error(
            "fillet_ring_act: the postcondition failed - a loop "
            "does not chain (" +
            (diag.empty() ? std::string("an assembly defect")
                          : diag) +
            "; report this as a kernel bug)");
    }
    if (!is_closed_orientable(*R)) {
        std::string diag;
        for (std::size_t i = 0; i < R->edges.size(); ++i) {
            const std::size_t n = R->edges[i].coedges().size();
            if (n == 2) {
                const bool r0 =
                    R->edges[i].coedges()[0]->reversed();
                const bool r1 =
                    R->edges[i].coedges()[1]->reversed();
                if (r0 != r1) continue;
                diag += " edge " + std::to_string(i) +
                        " (two " +
                        std::string(r0 ? "reversed"
                                       : "forward") +
                        " co-edges)";
            } else {
                diag += " edge " + std::to_string(i) + " (" +
                        std::to_string(n) + " co-edges)";
            }
            break;
        }
        throw std::runtime_error(
            "fillet_ring_act: the postcondition failed - the "
            "closed-orientable contract broke (" +
            (diag.empty() ? std::string("the orientation walk")
                          : diag) +
            "; report this as a kernel bug)");
    }
    if (R->vertex_count() != 24 || R->edge_count() != 48 ||
        R->face_count() != 26)
        throw std::runtime_error(
            "fillet_ring_act: the postcondition failed - the "
            "counts moved (the act's own arithmetic is "
            "V-16+24 / E-16+40 / F+16 from 16/24/10; report "
            "this as a kernel bug)");
    if (euler_characteristic(*R) != 2)
        throw std::runtime_error(
            "fillet_ring_act: the postcondition failed - chi "
            "moved (the act is chi-flat by construction, "
            "Delta-chi = +8 -16 +8 = 0; report this as a kernel "
            "bug)");
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
