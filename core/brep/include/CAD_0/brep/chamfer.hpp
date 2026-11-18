// core/brep/include/CAD_0/brep/chamfer.hpp
//
// THE CHAMFER, STAGE 1, MORSE 1 — LO SPIGOLO VERTICALE SINGOLO DEL BOX
// (ADR-0045 D1/D2/D8.1, the line G6): the flat bevel — the second
// feature that SUBSTITUTES instead of subtracting, every contact a
// LINE, the miter's line registered for its own morsi (D4: the corner
// act, the ring act — never smuggled here).
//
// THE CONTRACT (D2, the falsifiable thesis): NESSUNA riga SSI nuova,
// ZERO intersection solves — the bevel is born on the EXISTING
// PlaneSurface family (no new surface class, no seam, no periodicity),
// the setback lines are AUTHORED at their closed-form positions (the
// plane×plane row consumed as GRAMMAR, never run as a solve), and the
// four born vertices are closed forms of the stored operands (the
// setback feet: the corner offset by the setback along each adjacent
// face's inward direction — sb_A = corner - d*n_B, sb_B = corner -
// d*n_A, the same feet the fillet's tangent lines sit on). Touching
// ssi.hpp/ssi.cpp for this stage is a defect, not a choice (D2's STOP
// criterion, verbatim).
//
// THE ADMISSION (D1/D5, before the construction — the counts and the
// families, zero dihedral arithmetic): this morse's carrier is THE
// FACTORY BOX alone (k = 0: 8 vertices, 12 edges, 6 faces, chi = 2,
// closed orientable, every entity exact, every face a single-loop
// PLANE). The CHAIN's widening (a box carrying k previous vertical
// chamfers, 8+2k / 12+3k / 6+k, the bevel-stamp wall against the
// all-planar impostors — the hexagonal prism 12/18/8 among them) is
// the NEXT morse's own seat (D8.2, the fillet's own history: the m2
// admitted the box alone, the m6ter widened the chain). A body
// carrying curved faces (a fillet's flank among them) refuses HERE
// by signature — the mixed corner is the crease merge's family
// (D5.3, the seat ADR-0039 D4), never sedated.
//
// LE BANDE TIPIZZATE (D5.1, the Linear world's own eps_L, the bound
// NAMED in the refusal): the setback line strictly inside both
// adjacent faces (d < the face's inward reach - eps), the chamfer
// segment's feet strictly inside the caps' ring edges (d < the ring
// edge's length - eps) — the single edge's own band. THE STATE BAND
// d < 1 (the twelve-edge strip law, the six planes dying to points at
// the rhombic-dodecahedron collapse) is the MULTI-EDGE mode's wall
// (D5.1's second half): documented here, tested at its own morsi —
// this single-edge walk cannot meet it (one bevel never reaches the
// opposite geometry within its own band).
//
// THE 17/11 LESSON IS LAW (D4.4): every span end of every born or
// re-windowed face is read from the TOPOLOGY (the walk's junctions,
// the stored pairs), NEVER from the coordinate ordering — and the
// closed forms read the box's frame FROM THE PAYLOAD (the corner, the
// normals, the spans from the stored operands — the m6 correttiva's
// own cure: a translated or resized box is the SAME closed forms, the
// world's origin and the unit box's +-1 never consulted).
//
// The counts by the walk's own arithmetic (the fillet's table, one
// family under — the born edges all LINES where the fillet's were
// lines-and-arcs): V - 2 + 4 / E - 1 + 4 / F + 1 — the golden
// 10/15/7 at d = 0.5, chi = 2, H = 0, Delta-chi = +2 - 3 + 1 = 0;
// the volume V = 8 - d^2 (the removed region the right triangular
// prism of legs d and height the edge's own span — ADR-0045 D6.1).
// The tavola is chamfer_descent's own derivation (descent.hpp — the
// recipe is the source of truth, the tavola is cache), the vanished
// cause verbatim: "replaced by the bevel".
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

// The chamfer's own declared Linear tolerance (the fillet's constant
// one feature over — the same 1e-6, its own name).
inline constexpr double kChamferTolerance = 1e-6;

// The naming context (the FilletNaming discipline, the m2 single-table
// form): the chamfer's own DAG id (the born names' creator part,
// 'che#2') and the carrier's creating kind (its role tables — the
// bevel's operands render through them). Without the context (the
// default, the unnamed mode) the born entities read honestly empty —
// the shrink faces' inheritance flows in both modes.
struct ChamferNaming {
    geometry::FeatureId feature{};
    const naming::OperationNaming* body{};
};

namespace chamfer_detail {

// The plane payload reader (the typed dispatch — the fillet_detail
// precedent: local to this header so the chamfer adds no include
// edge).
struct PlaneReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    void visit(const PlaneSurface& p) override { plane = &p; }
};

// The plane's domain coordinates of a 3D point (the p-curve writers'
// own projection — factory.hpp's plane_domain, fillet.hpp's dom_of,
// local here for the same reason).
inline math::Vec2d dom_of(const PlaneSurface& s, math::Vec3d p) {
    const math::Vec3d d = p - s.origin();
    return {d.dot(s.u_axis()), d.dot(s.v_axis())};
}

// The CCW correction (the 17/11 law's own rule at the frame
// level, G6 m3's sweep the finder): the walk's orientation
// MEASURED in the actual (u, v) domain — the signed area of the
// p-curves — and reversed when the domain's axes mirror the
// construction's assumption (the plane frame's u flips with the
// bisector's octant: a walk authored for +u reads CLOCKWISE where
// the frame chose -u, and the radial two-opposite contract
// breaks). The landed positive-octant walks measure positive: the
// correction a NO-OP there, the digests immovable.
inline void ensure_ccw(Loop& loop) {
    double area2 = 0.0;
    for (const auto& ce : loop.coedges()) {
        const auto* pc = ce.p_curve();
        if (pc == nullptr || pc->kind != PCurve::Kind::Segment)
            return;  // defensive: non-segment walks untouched
        area2 += pc->segment.a.x * pc->segment.b.y -
                 pc->segment.a.y * pc->segment.b.x;
    }
    if (area2 >= 0.0) return;
    Loop fixed;
    const auto& ces = loop.coedges();
    for (std::size_t i = ces.size(); i-- > 0;) {
        const auto& ce = ces[i];
        CoEdge c{ce.edge(), !ce.reversed()};
        const auto* pc = ce.p_curve();
        if (pc != nullptr && pc->kind == PCurve::Kind::Segment)
            c.set_p_curve(pcurve_segment(
                pc->segment.b, pc->segment.a,
                /*interior_left=*/true));
        fixed.add_coedge(c);
    }
    loop = std::move(fixed);
}

// ---------------------------------------------------------------------------
// The chain carrier's admission (the block chamfer_edge landed in
// G6 m2, shared verbatim with the corner act — the same signature,
// the same walls, the same words): the counts' chain formula, the
// bevel-stamp wall against the all-planar impostors, the families
// and the loops. Returns k, the bevel-stamped faces the counts
// declare.
// ---------------------------------------------------------------------------
template <class Refuse>
std::size_t admit_chain_carrier(const BRepBody& body,
                                std::size_t V0, std::size_t E0,
                                std::size_t F0, Refuse&& refuse) {
    bool all_exact = true;
    for (const auto& v : body.vertices)
        if (!v.provenance().is_exact()) all_exact = false;
    for (const auto& e : body.edges)
        if (!e.provenance().is_exact()) all_exact = false;
    for (const auto& f : body.solid.shell().faces())
        if (!f.provenance().is_exact()) all_exact = false;
    if (!all_exact)
        refuse("the body carries approximated entities (the "
               "Layer-2 record says so) - the exact chamfer "
               "consumes EXACT bodies only (the booleans' own "
               "gate). Remedy: author the body on the exact path "
               "(the factories), or accept the faceted path.");
    const std::size_t kcand =
        F0 >= 6 ? F0 - 6 : static_cast<std::size_t>(-1);
    const bool counts_match =
        kcand != static_cast<std::size_t>(-1) &&
        V0 == 8 + 2 * kcand && E0 == 12 + 3 * kcand;
    if (!counts_match ||
        euler_characteristic(body) != 2 ||
        !is_closed_orientable(body))
        refuse("the body is outside the class signature (the "
               "chamfer chain's own: 8+2k vertices, 12+3k "
               "edges, 6+k faces for k previous vertical "
               "chamfers, chi = 2, closed orientable; found " +
               std::to_string(V0) + "/" + std::to_string(E0) +
               "/" + std::to_string(F0) +
               ") - the carrier may carry ONLY the chain's own "
               "vertical bevels: the RING chamfers and the "
               "corner MITER are their own morsi's seats (D4, "
               "the heart of the stage), and a curved face is "
               "the mixed corner's family (D5.3, the crease "
               "merge's seat, ADR-0039 D4). Remedy: chamfer a "
               "factory box's vertical edge, or a chain of "
               "them.");
    std::size_t bevels = 0;
    for (const auto& f : body.solid.shell().faces()) {
        if (f.loops().size() != 1)
            refuse("a face carries " +
                   std::to_string(f.loops().size()) +
                   " loops - the class signature is the box "
                   "family's single-loop faces (a pierced or "
                   "notched face is a later family's carrier). "
                   "Remedy: chamfer a factory box's vertical "
                   "edge.");
        const auto* surf = f.surface();
        chamfer_detail::PlaneReader rd;
        const bool planar =
            surf != nullptr &&
            (surf->accept(rd), rd.plane != nullptr);
        if (!planar)
            refuse("a face carries a surface outside the class "
                   "families (the carrier's world is the box's "
                   "six planes and the chain's own flat "
                   "bevels; a curved face is the mixed "
                   "corner's family - the crease merge's "
                   "seat, ADR-0039 D4, ADR-0045 D5.3). "
                   "Remedy: chamfer a factory box's vertical "
                   "edge, or a chain of them.");
        // The bevel's own stamp form (the m1 walk's writer):
        // the bevel role under the two box-face legs.
        const auto& sub = f.shape_id().subshape;
        if (sub.depth == 3 &&
            sub.path[0] == naming::kCheBevelRole &&
            sub.path[1] < 6 && sub.path[2] < 6)
            ++bevels;
    }
    if (bevels != kcand)
        refuse("the body carries " + std::to_string(bevels) +
               " bevel-stamped faces where the chain's counts "
               "declare " + std::to_string(kcand) +
               " - the signature's own arithmetic (the "
               "bevel-stamp wall: the chamfer chain's faces "
               "are all planes, and the counts alone admit the "
               "ALL-PLANAR IMPOSTORS - the HEXAGONAL PRISM is "
               "12/18/8, every plane; the identity is where a "
               "thing was BORN, and only the chain's own "
               "bevels were born bevels). Remedy: chamfer a "
               "factory box's vertical edge, or a chain of "
               "them.");
    return kcand;
}

} // namespace chamfer_detail

// ---------------------------------------------------------------------------
// chamfer_edge — the flat bevel on the box's vertical edge (ADR-0045
// D1.1/D8.1, the class the fillet's v1 walks one family over).
// `edge_reference` is the structural name of the edge to chamfer
// (e.g. box#1:vertical2's ShapeId — the pick's own answer, found
// against the body's stamps; never parsed, never an address);
// `setback` the constant setback d; `naming` the context above;
// `tol` the operation's declared Linear tolerance.
//
// LOUD on every refusal (the P2 culture, the bounds NAMED): the
// reference that finds no edge (or a seam-wired pair), the body
// outside the class signature (this morse: the factory box alone),
// the non-vertical or non-planar-adjacent edge, the skewed dihedral
// (the registered general family), the setback outside the typed band
// (`chamfer_setback_exceeds_face`, the face and its reach carried).
// Returns the chamfered body.
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> chamfer_edge(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double setback, const ChamferNaming& naming = ChamferNaming{},
    double tol = kChamferTolerance) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("chamfer_edge: " + what);
    };

    // ---- The setback's own gate (the band's arithmetic needs it
    // sane first — a non-positive setback has no bevel plane).
    if (!(setback > 0.0) || !std::isfinite(setback))
        refuse("the setback must be finite and > 0, got " +
               std::to_string(setback) +
               " — a non-positive setback has no bevel plane and no "
               "setback lines. Remedy: author a positive setback "
               "within the faces' reach (the band the admission "
               "derives).");

    // ---- The class signature (the admission BEFORE the
    // construction — the shared chain-carrier block, G6 m2's own
    // words: the counts' chain formula 8+2k/12+3k/6+k, the
    // bevel-stamp wall against the all-planar impostors, the
    // families and the loops; the corner act shares it verbatim).
    const std::size_t chain_k =
        chamfer_detail::admit_chain_carrier(body, V0, E0, F0,
                                            refuse);

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
                   " edges (the seam-wired pair) - a chamfer asked on "
                   "a seam is refused with the pair named (ADR-0033 "
                   "C3's own boundary). Remedy: reference a "
                   "non-periodic edge (the box's verticals are "
                   "single).");
        edge = found[0];
    }

    // ---- The verticality (the construction's own axis: the bevel's
    // plane is vertical, the setback lines Z-parallel).
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
               "bevel is the vertical plane family, the guide case's "
               "own edge; the ring edges are the corner act's own "
               "morsi, ADR-0045 D4/D8). Remedy: reference one of the "
               "box's verticals (box#F:vertical0..3).");
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
               "plane. Remedy: report the body's construction (the "
               "factory never authors one).");

    // ---- The two adjacent faces, in the edge's own declared
    // co-edge order (the radial list's shell order — the bevel's
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
    chamfer_detail::PlaneReader rd_A, rd_B;
    src_A.surface()->accept(rd_A);
    src_B.surface()->accept(rd_B);
    if (rd_A.plane == nullptr || rd_B.plane == nullptr)
        refuse("an adjacent face is non-planar - the admission (a) "
               "takes both adjacent faces in the table's world "
               "(planes); the curved families are the rim class's "
               "(the fillet's m3) and the mixed corner's (D5.3, the "
               "crease merge's seat). Remedy: reference an edge "
               "between two planar faces (the box's verticals).");
    const math::Vec3d n_A = rd_A.plane->normal_vec();
    const math::Vec3d n_B = rd_B.plane->normal_vec();
    // The 90 degree dihedral: the axis-aligned signature's own exact
    // arithmetic (the stored unit normals' dot), never a tolerance
    // decision — the closed forms below (the bisector normal, the
    // setback distance d) are exact for the perpendicular pair and
    // wrong for anything else.
    if (n_A.dot(n_B) != 0.0)
        refuse("the adjacent faces' dihedral is not the class's 90 "
               "degrees (the stored normals' dot reads " +
               std::to_string(n_A.dot(n_B)) +
               " where the perpendicular pair reads exactly 0) - the "
               "GENERAL DIHEDRAL family is the same derivation one "
               "parameter wider, REGISTERED with the fillet's own "
               "stage 2 seat (a non-orthogonal prism joint), never "
               "smuggled (ADR-0039 D1.1, ADR-0045 D1). Remedy: "
               "chamfer an orthogonal corner, or await the "
               "general-dihedral morse.");

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

    // ---- LE BANDE TIPIZZATE (the Linear world's own eps_L, the
    // bounds named in the refusal): the setback line strictly inside
    // both adjacent faces; the chamfer segment's feet contained by
    // the caps' ring edges (the corner's extents).
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
    // past the setback feet.
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
               " (chamfer_setback_exceeds_face): the setback " +
               std::to_string(setback) +
               " against the typed band's bound " +
               std::to_string(reach) + " - " + std::to_string(tol) +
               " (the face's own reach minus the declared Linear "
               "tolerance): the setback line would exit the face, "
               "and the bevel's closed forms would place it outside "
               "the material. Remedy: shrink the setback below " +
               std::to_string(reach - tol) +
               ", or restructure the body (the band is the face's "
               "own reach, ADR-0045 D5.1's derived decision — the "
               "single edge's band; the multi-edge strip law d < 1 "
               "is the ring act's own wall).");
    };
    if (setback >= reach_A - tol)
        band_refusal("the setback line exits the first adjacent face",
                     reach_A);
    if (setback >= reach_B - tol)
        band_refusal("the setback line exits the second adjacent face",
                     reach_B);
    if (setback >= reach_cap_lo - tol)
        band_refusal("the chamfer segment exits the low end face",
                     reach_cap_lo);
    if (setback >= reach_cap_hi - tol)
        band_refusal("the chamfer segment exits the high end face",
                     reach_cap_hi);

    // ---- THE STAGE BAND (D5.1's derived wall, ALIVE with the
    // multi-edge state — G6 m2): in the chain (the carrier carries
    // k >= 1 bevels) every new setback must sit under 1 — the
    // strips of the opposing chamfers on every face must not meet
    // (2d < 2), the non-adjacent prisms' regions stay empty sse
    // d < 1 (the derivation: the tripla {x+y > 2−d, x+z > 2−d,
    // x−z > 2−d} dà 2x > 4−2d, e con x ≤ 1 è vuota sse d ≤ 1), and
    // at d = 1 the six planes die to points (the tangential
    // collapse, the rhombic dodecahedron) — the stage refuses the
    // limit ("the body is not the limit", the fillet's r=1
    // precedent). The SINGLE edge's own band stays the m1 tests'
    // (a lone chamfer at 1.5 within its faces' reach is the
    // single-edge mode's own geometry, its tests IMMOBILE); the
    // TWO READINGS ride the payload: the faces' own reach — the
    // m1 band, satisfied — and the stage's 1, the binding one.
    if (chain_k >= 1 && setback >= 1.0 - tol)
        refuse("the setback crosses the STAGE band "
               "(chamfer_setback_exceeds_face): the setback " +
               std::to_string(setback) +
               " against the stage's derived bound 1.000000 - " +
               std::to_string(tol) +
               " (THE TWO READINGS: the faces' own current reach " +
               std::to_string(std::min(reach_A, reach_B)) +
               " would admit it - the single edge's m1 band, "
               "satisfied - but the multi-edge state's strips must "
               "not meet, 2d < 2, the non-adjacent prisms' regions "
               "empty sse d < 1, and at d = 1 the six planes die "
               "to points - the tangential collapse, ADR-0045 "
               "D5.1's derivation; the body is not the limit, the "
               "fillet's r=1 precedent). Remedy: keep the chain's "
               "setbacks below 0.999999 (the uniform twelve-edge "
               "mode's own derivation is the ring act's morse, "
               "D4).");

    // ---- THE CONSTRUCTION — every quantity a closed form of the
    // stored operands, ZERO solves (D2's contract). The setback feet:
    // on each adjacent face at distance d from the edge, along the
    // OTHER face's inward direction (sb_A = corner - d*n_B — the
    // same feet the fillet's tangent lines sit on: at equal setback
    // and radius the two features share their four born POSITIONS,
    // the boundary between them the different geometry — the straight
    // segment where the fillet rolled its quarter arc). The bevel
    // plane: the bisector (n_A + n_B)/sqrt(2) through the point
    // corner - (d/2)(n_A + n_B) — the plane whose two landing lines
    // ARE the setback lines (the miter's own plane, D2.3's section).
    const math::Vec3d corner = c_lo;  // (x, y) shared by the edge
    const math::Vec2d sb_A{corner.x - setback * n_B.x,
                           corner.y - setback * n_B.y};
    const math::Vec2d sb_B{corner.x - setback * n_A.x,
                           corner.y - setback * n_A.y};
    const math::Vec3d bis{(n_A.x + n_B.x), (n_A.y + n_B.y), 0.0};
    const double bis_len = std::sqrt(bis.length_sq());
    const math::Vec3d bis_unit = bis * (1.0 / bis_len);
    const math::Vec3d bevel_point{
        corner.x - 0.5 * setback * (n_A.x + n_B.x),
        corner.y - 0.5 * setback * (n_A.y + n_B.y),
        0.5 * (z_lo + z_hi)};

    // ---- The maps: the survivors compact (the dead column
    // dropped), the born append (the numbering table below).
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
    const std::size_t v_SA_lo = v_base + 0, v_SA_hi = v_base + 1;
    const std::size_t v_SB_lo = v_base + 2, v_SB_hi = v_base + 3;
    const std::size_t e_base = E0 - 1;
    const std::size_t e_sb_A = e_base + 0;   // (v_SA_lo -> v_SA_hi)
    const std::size_t e_sb_B = e_base + 1;   // (v_SB_lo -> v_SB_hi)
    const std::size_t e_seg_lo = e_base + 2; // (v_SB_lo -> v_SA_lo)
    const std::size_t e_seg_hi = e_base + 3; // (v_SB_hi -> v_SA_hi)

    // The dead endpoint's substitute: the setback foot of the
    // adjacent face (A or B) that co-uses the re-spanned edge, at the
    // dead endpoint's own quota.
    const auto substitute = [&](std::size_t dead_v,
                                std::size_t re_spanned) -> std::size_t {
        for (const CoEdge* ce : body.edges[re_spanned].coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (f == face_A)
                return dead_v == v_lo ? v_SA_lo : v_SA_hi;
            if (f == face_B)
                return dead_v == v_lo ? v_SB_lo : v_SB_hi;
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
    // records), then the born setback pairs (A first — the co-edge
    // order, the bevel's own operands' order).
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        R->vertices.push_back(body.vertices[i]);
        R->vertices.back().set_id(static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[4] = {
            {sb_A.x, sb_A.y, z_lo}, {sb_A.x, sb_A.y, z_hi},
            {sb_B.x, sb_B.y, z_lo}, {sb_B.x, sb_B.y, z_hi}};
        const std::size_t ids[4] = {v_SA_lo, v_SA_hi, v_SB_lo,
                                    v_SB_hi};
        for (std::size_t i = 0; i < 4; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the endpoints remapped, the
    // dead column's substitutions applied), then the born four (the
    // setback lines bottom->top, the chamfer segments from the
    // second face's foot to the first's — the deterministic B->A
    // convention, the fillet's own).
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
            {v_SA_lo, v_SA_hi}, {v_SB_lo, v_SB_hi},
            {v_SB_lo, v_SA_lo}, {v_SB_hi, v_SA_hi}};
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
    // remapped); the bevel is born. The re-authoring is the
    // crossed-corner's own discipline: per co-edge, COPY (the id
    // remapped, the p-curve verbatim), RE-SPAN (the endpoints
    // substituted, the p-curve re-projected), or NEW (the setback
    // line replacing the dead co-edge on the laterals; the chamfer
    // segment — a LINE, never an arc — inserted between the
    // re-spanned pair on the caps) — every flag derived from the
    // walk's (start, end) against the stored pair, every p-curve
    // projected through the face's own native frame (the decoupling
    // rule: the frame serves the geometry, the walk serves the
    // topology).
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
            chamfer_detail::dom_of(pl, pos3(a)),
            chamfer_detail::dom_of(pl, pos3(b)),
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
                chamfer_detail::dom_of(pl, pos3(s_new)),
                chamfer_detail::dom_of(pl, pos3(e_new)),
                /*interior_left=*/true));
        } else if (ce.p_curve() != nullptr) {
            out.set_p_curve(*ce.p_curve());
        }
        return out;
    };

    // The lateral's re-authoring: the dead co-edge replaced by this
    // face's setback line (the dead co-edge's own walk direction
    // preserved — THE 17/11 LESSON'S OWN RULE: the direction read
    // through the WALK's vertices, the near endpoint's slot, never a
    // coordinate ordering), the re-spanned neighbors re-projected,
    // the rest copied verbatim.
    const auto reauthor_lateral = [&](std::size_t face_index,
                                      std::size_t sb_edge,
                                      const PlaneSurface& pl) {
        const Face& src =
            body.solid.shell().faces()[face_index];
        const auto& ces = src.loops()[0].coedges();
        Loop loop;
        for (const auto& ce : ces) {
            if (ce.edge() == edge) {
                // The setback line, the dead co-edge's own walk
                // direction (the Z sense of its traversal — the
                // factory's own authoring rule, read through the
                // walk's own vertices).
                const std::size_t d_start = static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[edge].vertex(1)
                        : body.edges[edge].vertex(0));
                const std::size_t d_end = static_cast<std::size_t>(
                    ce.reversed()
                        ? body.edges[edge].vertex(0)
                        : body.edges[edge].vertex(1));
                const auto& se = R->edges[sb_edge];
                const std::size_t lo_id =
                    static_cast<std::size_t>(se.vertex(0));
                const std::size_t hi_id =
                    static_cast<std::size_t>(se.vertex(1));
                const std::size_t from =
                    vpos(d_start).z <= vpos(d_end).z ? lo_id
                                                     : hi_id;
                const std::size_t to =
                    vpos(d_start).z <= vpos(d_end).z ? hi_id
                                                     : lo_id;
                emit_planar(loop, pl, sb_edge, from, to);
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
    // re-projected, the CHAMFER SEGMENT (a Line2 in the cap's own
    // domain — the flat bevel's own boundary, never an arc) inserted
    // between them, the rest copied verbatim.
    const auto reauthor_cap = [&](std::size_t face_index,
                                  std::size_t dead_v,
                                  std::size_t seg_edge) {
        const Face& src =
            body.solid.shell().faces()[face_index];
        chamfer_detail::PlaneReader rd;
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
            // The segment's insertion: after the re-spanned co-edge
            // that ENDS at the corner's substitute — between it and
            // the next co-edge's substituted start (the walk's own
            // junctions, the 17/11 lesson's rule verbatim).
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
                emit_planar(loop, pl, seg_edge, here, n_new);
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
    reauthor_lateral(face_A, e_sb_A, *rd_A.plane);
    reauthor_lateral(face_B, e_sb_B, *rd_B.plane);
    reauthor_cap(cap_lo, v_lo, e_seg_lo);
    reauthor_cap(cap_hi, v_hi, e_seg_hi);
    for (std::size_t f = 0; f < F0; ++f) {
        if (f == face_A || f == face_B || f == cap_lo ||
            f == cap_hi)
            continue;
        const Face& src = body.solid.shell().faces()[f];
        // The kept faces copy verbatim — the admission above already
        // verified every face is a plane, so anything else here is a
        // kernel bug.
        chamfer_detail::PlaneReader rd;
        src.surface()->accept(rd);
        if (rd.plane == nullptr)
            refuse("a kept face's surface is outside the "
                   "admission's families - report this as a kernel "
                   "bug.");
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

    // ---- The bevel is born: the PlaneSurface bisector window, the
    // walk the CCW rectangle in the bevel's own (u, v) domain (the
    // material on the left, the frame right-handed around the
    // outward bisector by construction) — every p-curve a Line2
    // projected through the bevel's own frame (dom_of — the m6
    // correttiva's cure: the window in the domain IS the projection
    // of the actual 3D feet, never a hand-computed span). The CCW
    // order is read from the FEET'S OWN domain coordinates (which
    // face's foot carries the lower v — a fact of the payload, never
    // of the world's axes).
    {
        auto surf = std::make_shared<PlaneSurface>(bevel_point,
                                                   bis_unit);
        const PlaneSurface& pl = *surf;
        const math::Vec2d dA_lo =
            chamfer_detail::dom_of(pl, pos3(v_SA_lo));
        const math::Vec2d dB_lo =
            chamfer_detail::dom_of(pl, pos3(v_SB_lo));
        const bool a_lower = dA_lo.y < dB_lo.y;
        // The CCW rectangle: at the low end from the HIGH-v foot to
        // the LOW-v one, up the LOW-v setback line, at the high end
        // from the LOW-v foot to the HIGH-v one, down the HIGH-v
        // setback line — the interior on the left at every leg.
        const std::size_t v_low_lo = a_lower ? v_SA_lo : v_SB_lo;
        const std::size_t v_low_hi = a_lower ? v_SA_hi : v_SB_hi;
        const std::size_t v_high_lo = a_lower ? v_SB_lo : v_SA_lo;
        const std::size_t v_high_hi = a_lower ? v_SB_hi : v_SA_hi;
        const std::size_t e_low_line = a_lower ? e_sb_A : e_sb_B;
        const std::size_t e_high_line = a_lower ? e_sb_B : e_sb_A;
        Loop loop;
        const auto emit_bevel = [&](std::size_t e_id,
                                    std::size_t a, std::size_t b) {
            const auto& stored = R->edges[e_id];
            const bool fwd =
                stored.vertex(0) == static_cast<VertexId>(a) &&
                stored.vertex(1) == static_cast<VertexId>(b);
            const bool rev =
                stored.vertex(0) == static_cast<VertexId>(b) &&
                stored.vertex(1) == static_cast<VertexId>(a);
            if (!fwd && !rev)
                refuse("the bevel's walk lost the chaining - report "
                       "this as a kernel bug.");
            CoEdge ce{static_cast<EdgeId>(e_id), rev};
            ce.set_p_curve(pcurve_segment(
                chamfer_detail::dom_of(pl, pos3(a)),
                chamfer_detail::dom_of(pl, pos3(b)),
                /*interior_left=*/true));
            loop.add_coedge(ce);
        };
        emit_bevel(e_seg_lo, v_high_lo, v_low_lo);
        emit_bevel(e_low_line, v_low_lo, v_low_hi);
        emit_bevel(e_seg_hi, v_low_hi, v_high_hi);
        emit_bevel(e_high_line, v_high_hi, v_high_lo);
        // THE CCW CORRECTION (the sweep's find, G6 m3): the frame's
        // u flips with the bisector's octant — the walk above is
        // authored for +u and reads CLOCKWISE where the frame chose
        // -u (the negative-x verticals). Measured in the actual
        // domain, reversed only when mirrored: a no-op on the
        // landed positive-octant goldens, their digests immovable.
        chamfer_detail::ensure_ccw(loop);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(F0));
        face.set_provenance(prov);
        // The bevel's name: che#F:bevel[<A> x <B>] — the operands
        // the referenced edge's adjacent faces, in the co-edge
        // order; the second binding of the isect rows below is the
        // SAME feature's bevel (the same-feature extension, born
        // earlier in this walk).
        if (naming.body != nullptr &&
            src_A.shape_id().subshape.depth > 0 &&
            src_B.shape_id().subshape.depth > 0) {
            naming::OperandBinding che_operands[2] = {
                {src_A.shape_id().feature, naming.body},
                {naming.feature, nullptr}};
            const naming::OperationNaming che_decl{
                naming::FeatureKind::Chamfer,
                naming::kChamferFaceRoles, naming::kChamferEdgeRoles,
                che_operands};
            che_operands[1].naming = &che_decl;
            const auto leg = [](geometry::SubshapeRef ref,
                                const Face& operand) {
                for (std::uint8_t k = 0;
                     k < operand.shape_id().subshape.depth; ++k)
                    ref = ref.push(
                        operand.shape_id().subshape.path[k]);
                return ref;
            };
            geometry::SubshapeRef ref =
                geometry::SubshapeRef{}.push(naming::kCheBevelRole);
            ref = leg(ref, src_A);
            ref = leg(ref, src_B);
            face.set_shape_id({naming.feature, ref, {}});
            face.set_name(naming::render_name(
                naming.feature, ref, che_decl.faces()));
            // The setback wires: che#F:isect[<the landing face> x
            // <the bevel>] — the level ABSENT, the setback IS the
            // level. The landing face of each setback line is the
            // adjacent face it retires.
            const struct {
                std::size_t edge;
                const Face* landing;
            } wires[2] = {{e_sb_A, &src_A}, {e_sb_B, &src_B}};
            for (const auto& w : wires) {
                // The wire's path: the isect role, the LANDING
                // face's leg, then the BEVEL's own role and its
                // two legs (the same-feature operand renders as a
                // full nested role — the renderer consumes the
                // bevel's index, then its slots' operands).
                geometry::SubshapeRef wref =
                    geometry::SubshapeRef{}.push(
                        naming::kCheIsectRole);
                wref = leg(wref, *w.landing);
                wref = wref.push(naming::kCheBevelRole);
                wref = leg(wref, src_A);
                wref = leg(wref, src_B);
                R->edges[w.edge].set_shape_id(
                    {naming.feature, wref, {}});
                R->edges[w.edge].set_name(naming::render_name(
                    naming.feature, wref, che_decl.edges()));
            }
        }
        shell.add_face(std::move(face));
    }

    // ---- THE OUTPUT GATE (the identities, hard postconditions).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "chamfer_edge: postcondition failed - a loop does not "
            "chain (an assembly defect in the chamfer's re-authored "
            "walks; report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "chamfer_edge: postcondition failed - the closed-"
            "orientable contract broke (every edge exactly two "
            "opposite co-edges; report this as a kernel bug)");
    if (R->vertex_count() != V0 - 2 + 4 ||
        R->edge_count() != E0 - 1 + 4 ||
        R->face_count() != F0 + 1)
        throw std::runtime_error(
            "chamfer_edge: postcondition failed - the counts "
            "moved (the class's own arithmetic is V-2+4 / E-1+4 / "
            "F+1; report this as a kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(body))
        throw std::runtime_error(
            "chamfer_edge: postcondition failed - chi moved (the "
            "chamfer is chi-flat by construction, Delta-chi = "
            "+2 -3 +1 = 0; report this as a kernel bug)");
    return R;
}
// ---------------------------------------------------------------------------
// chamfer_corner_act — THE CORNER ACT AT ONE CORNER (G6 m3, ADR-0045
// D4/D8.3, the heart of the stage): the miter as an atomic act, on
// the stamp of fillet_ring_edge_corner_act (G5 m3, ADR-0043 D5). The
// reference is ONE RING edge (X- or Y-parallel) whose corner end
// carries a previous VERTICAL chamfer's bevel; the act consumes the
// corner WHOLE: the TWO ring edges convergent there (the reference
// and its partner, found through the cap segment's junctions — the
// 17/11 law: every span's end read from the TOPOLOGY, never an
// ordering of coordinates), the vertical bevel's CAP SEGMENT at the
// corner (dead, the cause verbatim "replaced by the miter line"),
// and the corner's two feet; the MITER M = corner − (d/2)(1,1,1) is
// born a VERTEX (never a face — D2's concurrence, the three planes
// sx·x+sy·y = c_xy, sx·x+sz·z = c_xz, sy·y+sz·z = c_yz meeting in
// one point, the determinant 2 ≠ 0), the three MITER LINES born
// (each from a retreat point to M), the two RING BEVELS born GIÀ
// RIFINITI, and the vertical bevel RE-WINDOWED in place (Absorbed,
// never Born-sotto-falso — its stamp and name survive the edit).
//
// THE FAR ENDS, derived before the code and written here (D8.3's
// own demand): the act's ring bevels span their edges' WHOLE
// extent — the bevel plane does not taper, and at the far end it
// cuts the UNMITERED corner the same plane reaches: each ring
// bevel is a PENTAGON (two setback lines, two miter lines at the
// corner end, ONE FAR-END SEGMENT lying on the far corner's third
// — lateral — face), the far corner's vertex dies with its two
// incident edges RE-SPANNED to the far feet, and the far lateral
// face absorbs the segment (its own corner cut). The ADR's D4.2
// hexagon is the BOTH-corners composition — additive: the second
// act at the same vertical's other corner re-windows the pentagon
// again; the one-corner act's own shape is the pentagon (the
// amendment this morse writes into D4.2).
//
// THE COUNTS (the tavola's closure, from the k=1 chain 10/15/7):
// dead 4 vertices (the two corner feet and the two far corners)
// and 3 edges (the two ring edges and the cap segment); born 8
// vertices (M, the three retreats, the four far feet), 9 edges
// (the three miter lines, the four setback lines, the two far-end
// segments) and 2 faces (the ring bevels); re-spanned 6 edges (the
// two setback wires, the two far verticals, the two far ring
// edges); kept 1 face (the far cap). V-4+8 / E-3+9 / F+2 ->
// 14/21/9, chi = 2 flat. The census: the cap 4 + the far cap 5 +
// the two near laterals 4 + the two far laterals 5 + the vertical
// bevel 5 + the two ring bevels 5 = 42 incidences / 2 = 21.
//
// THE VOLUME, closed form (the oracle's own derivation, the
// inclusion-exclusion at the corner in the retreat coordinates
// u = a−|x|, v, w): the three prisms 3·d² minus the three pairwise
// wedges 3·(d³/3) plus the triple d³/4 (the tetra-split, det/6:
// d³/6 + d³/12) — V = 8 − 3d² + (3/4)d³ for the unit box, the
// per-corner account of the twelve-spiegoli R_c = (3/4)d²(2−d) =
// d²/2 + (d² − (3/4)d³) the coherence identity (the corner's
// vertical half-prism plus its ring share).
//
// THE WITNESS (D2's thesis, falsifiable): every quantity above is
// a CLOSED FORM of the stored operands — the miter by the corner's
// signs and planes, the retreats and the far feet by the setback
// along the faces' own normals, the walks by the junctions. ZERO
// intersection solves, ZERO SSI consults, ZERO new families (the
// miter is a vertex; every born face a PlaneSurface). The diff
// touches no ssi.hpp/ssi.cpp line.
//
// LOUD on every refusal (the bounds and the values NAMED): the
// reference that finds no edge; the non-ring reference; the
// corner without a vertical bevel (or ambiguous between two);
// the setback mismatch against the vertical bevel's own
// (chamfer_corner_setback_mismatch, both values in the payload —
// D5.4's law: the miter closes at the corner's octant only at the
// common setback); the band (chamfer_setback_exceeds_face, the
// reach carried, and the STAGE band d < 1 with the two readings —
// the multi-edge state's strip law, D5.1).
// ---------------------------------------------------------------------------
inline std::unique_ptr<BRepBody> chamfer_corner_act(
    const BRepBody& body, const geometry::ShapeId& edge_reference,
    double setback, const ChamferNaming& naming = ChamferNaming{},
    double tol = kChamferTolerance) {
    const std::size_t V0 = body.vertices.size();
    const std::size_t E0 = body.edges.size();
    const std::size_t F0 = body.face_count();

    const auto refuse = [](const std::string& what) {
        throw std::invalid_argument("chamfer_corner_act: " + what);
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
        chamfer_detail::PlaneReader rd;
        const auto* s = body.solid.shell().faces()[f].surface();
        if (s != nullptr) s->accept(rd);
        return rd.plane;
    };

    // ---- The setback's own gate.
    if (!(setback > 0.0) || !std::isfinite(setback))
        refuse("the setback must be finite and > 0, got " +
               std::to_string(setback) +
               " — a non-positive setback has no bevel plane. "
               "Remedy: author a positive setback within the "
               "faces' reach (the band the admission derives).");

    // ---- The class signature (the shared chain-carrier block).
    const std::size_t chain_k =
        chamfer_detail::admit_chain_carrier(body, V0, E0, F0,
                                            refuse);
    if (chain_k < 1)
        refuse("the carrier carries no vertical bevel (k = 0) - "
               "the act's corner is the THREE planes' concurrence "
               "(the vertical bevel and the two ring bevels, D4.1). "
               "Remedy: chamfer a vertical edge first "
               "(chamfer_edge), then the act.");

    // ---- The reference's own find: the structural match against
    // the CURRENT state's stamps (never parsed, never an address).
    std::size_t edge = E0;
    for (std::size_t i = 0; i < E0; ++i) {
        const auto& id = body.edges[i].shape_id();
        if (id.feature != edge_reference.feature) continue;
        if (id.subshape != edge_reference.subshape) continue;
        if (edge != E0)
            refuse("the reference names more than one edge of "
                   "this body (the structural match is ambiguous) "
                   "- report this as a kernel bug.");
        edge = i;
    }
    if (edge == E0)
        refuse("the reference names no edge of this body (the "
               "structural find against the CURRENT state's "
               "stamps) - the edge may be dead (consumed by an "
               "earlier chamfer) or foreign to this body. Remedy: "
               "query the body's edges and reference a live RING "
               "edge at a bevel-occupied corner.");

    // ---- THE RING-NESS (the act's own axis test: the reference
    // is a RING edge — X- or Y-parallel; the vertical walk is
    // chamfer_edge's own door, the wires are nobody's).
    const VertexId ev0 = body.edges[edge].vertex(0);
    const VertexId ev1 = body.edges[edge].vertex(1);
    const math::Vec3d& p0 = vpos(static_cast<std::size_t>(ev0));
    const math::Vec3d& p1 = vpos(static_cast<std::size_t>(ev1));
    const bool vertical =
        std::abs(p0.x - p1.x) <= tol && std::abs(p0.y - p1.y) <= tol;
    const bool x_par =
        std::abs(p0.y - p1.y) <= tol && std::abs(p0.z - p1.z) <= tol;
    const bool y_par =
        std::abs(p0.x - p1.x) <= tol && std::abs(p0.z - p1.z) <= tol;
    if (vertical)
        refuse("the reference is a VERTICAL edge - the corner act "
               "takes a RING edge (X- or Y-parallel) at a corner "
               "occupied by a vertical bevel; the vertical walk is "
               "chamfer_edge's own door. Remedy: reference one of "
               "the corner's two ring edges.");
    if (!x_par && !y_par)
        refuse("the reference is neither X- nor Y-parallel (a "
               "setback wire or a diagonal) - the corner act's "
               "world is the box's ring. Remedy: reference one of "
               "the corner's two ring edges.");

    // ---- THE CORNER (the occupied end): the endpoint whose
    // incident faces include a BEVEL-STAMPED face — the vertical
    // chamfer's own (the m2 stamp predicate). Exactly one: a bare
    // ring pair has no miter to close, and a reference between two
    // vertical bevels is the twelve-spigoli composition's own seat.
    const auto bevel_face_at = [&](std::size_t v) -> std::size_t {
        for (std::size_t ei = 0; ei < E0; ++ei) {
            const auto& e = body.edges[ei];
            if (e.vertex(0) != static_cast<VertexId>(v) &&
                e.vertex(1) != static_cast<VertexId>(v))
                continue;
            for (const CoEdge* ce : e.coedges()) {
                const std::size_t f = face_of_coedge(ce);
                if (f >= F0) continue;
                const auto& sub = body.solid.shell()
                                      .faces()[f]
                                      .shape_id()
                                      .subshape;
                if (sub.depth == 3 &&
                    sub.path[0] == naming::kCheBevelRole &&
                    sub.path[1] < 6 && sub.path[2] < 6)
                    return f;
            }
        }
        return F0;
    };
    std::size_t v_occ = V0, v_far = V0, vert_face = F0;
    {
        const std::size_t ends[2] = {
            static_cast<std::size_t>(ev0),
            static_cast<std::size_t>(ev1)};
        for (const std::size_t u : ends) {
            const std::size_t f = bevel_face_at(u);
            if (f == F0) continue;
            if (v_occ != V0)
                refuse("the reference runs between TWO vertical "
                       "bevels (both ends occupied) - the corner "
                       "is ambiguous, and the two-bevel ring is "
                       "the twelve-spigoli composition's own seat "
                       "(D9's second half). Remedy: act at a "
                       "corner whose far end is free, or await "
                       "the ring act's morse.");
            v_occ = u;
            vert_face = f;
        }
    }
    if (v_occ == V0)
        refuse("the reference's corner carries NO vertical bevel - "
               "the act's corner is the THREE planes' concurrence "
               "(the vertical bevel and the two ring bevels, D4.1), "
               "and a bare ring pair has no miter to close. "
               "Remedy: chamfer the corner's vertical edge first "
               "(chamfer_edge), then the act.");
    v_far = static_cast<std::size_t>(ev0) == v_occ
                ? static_cast<std::size_t>(ev1)
                : static_cast<std::size_t>(ev0);

    // ---- THE VERTICAL BEVEL'S OWN SETBACK, read from its stored
    // plane (the offset the m1 walk wrote) — THE MATCH (D5.4's
    // named wall): the miter closes at the corner's octant only at
    // the common setback.
    const PlaneSurface* pl_V = plane_of(vert_face);
    if (pl_V == nullptr)
        refuse("the corner's vertical bevel is not a plane - the "
               "class signature is the box family's; report this "
               "as a kernel bug.");
    // The corner's lateral planes are read below; the bevel's
    // offset is derived from its plane and the corner (the signs
    // squared to 1: (n_A+n_B)·corner − d_v = sqrt(2)·(n̂·o)).
    // (Filled after the faces are identified.)

    // ---- THE CORNER'S TOPOLOGICAL DECOMPOSITION (the 17/11 law):
    // the cap segment (the vertical bevel's non-vertical edge at
    // the occupied end), the partner foot (its other end), the
    // partner ring edge (the ring edge at the partner foot), and
    // the two setback wires (the vertical edges of the bevel at
    // the two feet).
    const auto is_vertical_edge = [&](std::size_t ei) {
        const auto& e = body.edges[ei];
        const math::Vec3d& a =
            vpos(static_cast<std::size_t>(e.vertex(0)));
        const math::Vec3d& b =
            vpos(static_cast<std::size_t>(e.vertex(1)));
        return std::abs(a.x - b.x) <= tol &&
               std::abs(a.y - b.y) <= tol;
    };
    const auto edge_on_face = [&](std::size_t ei,
                                  std::size_t f) {
        for (const CoEdge* ce : body.edges[ei].coedges())
            if (face_of_coedge(ce) == f) return true;
        return false;
    };
    std::size_t cap_seg = E0, wire_ref = E0;
    for (std::size_t ei = 0; ei < E0; ++ei) {
        if (ei == edge) continue;
        const auto& e = body.edges[ei];
        if (e.vertex(0) != static_cast<VertexId>(v_occ) &&
            e.vertex(1) != static_cast<VertexId>(v_occ))
            continue;
        if (!edge_on_face(ei, vert_face)) continue;
        if (is_vertical_edge(ei)) {
            wire_ref = ei;
        } else {
            if (cap_seg != E0)
                refuse("the corner carries more than one cap "
                       "segment on the vertical bevel - the class "
                       "signature is the chain's own; report this "
                       "as a kernel bug.");
            cap_seg = ei;
        }
    }
    if (cap_seg == E0 || wire_ref == E0)
        refuse("the corner's cap segment or setback wire was not "
               "found on the vertical bevel - the class signature "
               "is the chain's own (the m1 walk's own born pair at "
               "each end); report this as a kernel bug.");
    const std::size_t v_part =
        static_cast<std::size_t>(
            body.edges[cap_seg].vertex(0)) == v_occ
            ? static_cast<std::size_t>(
                  body.edges[cap_seg].vertex(1))
            : static_cast<std::size_t>(
                  body.edges[cap_seg].vertex(0));
    std::size_t part_edge = E0, wire_part = E0;
    for (std::size_t ei = 0; ei < E0; ++ei) {
        if (ei == cap_seg) continue;
        const auto& e = body.edges[ei];
        if (e.vertex(0) != static_cast<VertexId>(v_part) &&
            e.vertex(1) != static_cast<VertexId>(v_part))
            continue;
        if (edge_on_face(ei, vert_face)) {
            if (!is_vertical_edge(ei))
                refuse("the partner foot carries a second cap "
                       "segment - the class signature is the "
                       "chain's own; report this as a kernel bug.");
            wire_part = ei;
            continue;
        }
        const math::Vec3d& a =
            vpos(static_cast<std::size_t>(e.vertex(0)));
        const math::Vec3d& b =
            vpos(static_cast<std::size_t>(e.vertex(1)));
        const bool ex = std::abs(a.y - b.y) <= tol &&
                        std::abs(a.z - b.z) <= tol;
        const bool ey = std::abs(a.x - b.x) <= tol &&
                        std::abs(a.z - b.z) <= tol;
        if (ex != ey) part_edge = ei;  // exactly one parallelism
    }
    if (part_edge == E0 || wire_part == E0)
        refuse("the partner ring edge or its wire was not found at "
               "the partner foot - the class signature is the "
               "chain's own; report this as a kernel bug.");
    const std::size_t v_far_part =
        static_cast<std::size_t>(
            body.edges[part_edge].vertex(0)) == v_part
            ? static_cast<std::size_t>(
                  body.edges[part_edge].vertex(1))
            : static_cast<std::size_t>(
                  body.edges[part_edge].vertex(0));

    // ---- THE FACES' ROLES AND THE CORNER'S SIGNS (the payload's
    // own octant — the outward normals, never a coordinate
    // ordering).
    const auto& ref_ces = body.edges[edge].coedges();
    if (ref_ces.size() != 2)
        refuse("the reference edge's radial structure is not the "
               "two-co-edge class - report this as a kernel bug.");
    const std::size_t face_A = face_of_coedge(ref_ces[0]);
    const std::size_t face_B = face_of_coedge(ref_ces[1]);
    const auto& part_ces = body.edges[part_edge].coedges();
    const std::size_t face_A2 = face_of_coedge(part_ces[0]);
    const std::size_t face_B2 = face_of_coedge(part_ces[1]);
    const PlaneSurface* pl_A = plane_of(face_A);
    const PlaneSurface* pl_B = plane_of(face_B);
    if (pl_A == nullptr || pl_B == nullptr)
        refuse("the reference's adjacent faces are not both planes "
               "- the class signature is the box family's; report "
               "this as a kernel bug.");
    const bool A_is_cap =
        std::abs(pl_A->normal_vec().z) > 1.0 - 1e-9;
    const bool B_is_cap =
        std::abs(pl_B->normal_vec().z) > 1.0 - 1e-9;
    if (A_is_cap == B_is_cap)
        refuse("the reference's faces are both caps or both "
               "laterals - the ring class takes one of each (the "
               "cap the corner's shared plane). Remedy: report the "
               "body's construction.");
    const std::size_t cap_face = A_is_cap ? face_A : face_B;
    const std::size_t lat_ref = A_is_cap ? face_B : face_A;
    const std::size_t lat_part =
        face_A2 == cap_face ? face_B2 : face_A2;
    if ((face_A2 != cap_face && face_B2 != cap_face) ||
        lat_part == cap_face || lat_part == lat_ref)
        refuse("the partner ring edge does not share the corner's "
               "cap face, or its laterals coincide - the class "
               "signature is the box's own corner (two ring edges "
               "on one cap). Remedy: report the body's "
               "construction.");
    const PlaneSurface* pl_cap = plane_of(cap_face);
    const PlaneSurface* pl_latr = plane_of(lat_ref);
    const PlaneSurface* pl_latp = plane_of(lat_part);
    if (pl_cap == nullptr || pl_latr == nullptr ||
        pl_latp == nullptr)
        refuse("the corner's planes are not all planes - the class "
               "signature is the box family's; report this as a "
               "kernel bug.");
    const math::Vec3d n_cap = pl_cap->normal_vec();
    const math::Vec3d n_latr = pl_latr->normal_vec();
    const math::Vec3d n_latp = pl_latp->normal_vec();
    const auto unit_axis = [&](const math::Vec3d& n,
                               const char* what) -> double {
        const double len = std::sqrt(n.length_sq());
        if (std::abs(len - 1.0) > 1e-9 ||
            std::abs(std::abs(n.x) + std::abs(n.y) +
                     std::abs(n.z) - 1.0) > 1e-9)
            refuse(std::string("the corner's ") + what +
                   " normal is not a unit axis - the class "
                   "signature is the box family's. Remedy: report "
                   "the body's construction.");
        return n.x + n.y + n.z;  // ±1, the axis's own sign
    };
    // THE SIGNS from the AXES, never the roles (the 17/11 law at
    // the octant level): whichever lateral is x-aligned carries
    // sx, the other sy — the reference's role (X- or Y-parallel)
    // permutes the two laterals, and the corner's planes do not.
    const bool latr_is_x =
        std::abs(n_latr.x) > 1.0 - 1e-9;
    const double sx = unit_axis(
        latr_is_x ? n_latr : n_latp, "the x lateral");
    const double sy = unit_axis(
        latr_is_x ? n_latp : n_latr, "the y lateral");
    // The cap's own axis check (the sign itself unneeded below —
    // the corner's z read from the plane, the retreats from the
    // normal vectors).
    unit_axis(n_cap, "cap");
    // THE CORNER, from the planes' own coordinates — EACH COORDINATE
    // FROM ITS OWN AXIS'S PLANE (the x lateral carries x, the y
    // lateral y, the cap z: a lateral's origin is ANY point on its
    // plane, the other coordinates arbitrary — reading the corner's
    // x off the y lateral's origin is the sweep's own 17/11 lesson,
    // the axis never the role).
    const math::Vec3d corner{
        latr_is_x ? pl_latr->origin().x : pl_latp->origin().x,
        latr_is_x ? pl_latp->origin().y : pl_latr->origin().y,
        pl_cap->origin().z};

    // ---- THE SETBACK MATCH (D5.4's named wall, now that the
    // corner is known): the vertical bevel's plane
    // (n_A+n_B)·p = (n_A+n_B)·corner − d_v.
    {
        const math::Vec3d n_V = pl_V->normal_vec();
        const double rhs = sx * corner.x + sy * corner.y;
        const double d_v =
            rhs - std::sqrt(2.0) * (n_V.x * pl_V->origin().x +
                                    n_V.y * pl_V->origin().y);
        if (!(d_v > 0.0 && d_v < 2.0))
            refuse("the vertical bevel's plane is not the class's "
                   "bisector (the offset read " +
                   std::to_string(d_v) +
                   ") - report this as a kernel bug.");
        if (std::abs(d_v - setback) > tol)
            refuse("the corner's setbacks disagree "
                   "(chamfer_corner_setback_mismatch): the "
                   "requested " +
                   std::to_string(setback) +
                   " against the vertical bevel's own " +
                   std::to_string(d_v) +
                   " - the miter is the THREE planes' concurrence "
                   "and closes inside the corner's octant only at "
                   "the COMMON setback (at unequal setbacks the "
                   "concurrence point exits the octant save the "
                   "triangular inequalities, and the closure "
                   "degenerates into the asymmetric window — the "
                   "flat vertex blend, out of the exact world, "
                   "D5.4). Remedy: chamfer the corner's three "
                   "edges at the same setback.");
    }

    // ---- THE FAR JUNCTIONS (the topology of each far corner:
    // the vertical and the ring edge that meet the consumed edge
    // there, and the far lateral — the third plane).
    const auto far_junctions = [&](std::size_t v_far_x,
                                   std::size_t consumed,
                                   std::size_t near_lat,
                                   std::size_t& far_vert,
                                   std::size_t& far_ring,
                                   std::size_t& far_lat) {
        far_vert = E0;
        far_ring = E0;
        for (std::size_t ei = 0; ei < E0; ++ei) {
            if (ei == consumed) continue;
            const auto& e = body.edges[ei];
            if (e.vertex(0) != static_cast<VertexId>(v_far_x) &&
                e.vertex(1) != static_cast<VertexId>(v_far_x))
                continue;
            if (is_vertical_edge(ei)) {
                far_vert = ei;
                continue;
            }
            const math::Vec3d& a =
                vpos(static_cast<std::size_t>(e.vertex(0)));
            const math::Vec3d& b =
                vpos(static_cast<std::size_t>(e.vertex(1)));
            const bool ex = std::abs(a.y - b.y) <= tol &&
                            std::abs(a.z - b.z) <= tol;
            const bool ey = std::abs(a.x - b.x) <= tol &&
                            std::abs(a.z - b.z) <= tol;
            if (ex != ey) far_ring = ei;
        }
        if (far_vert == E0 || far_ring == E0)
            refuse("a far corner's junctions are not the box's own "
                   "(a vertical and a ring edge) - the class "
                   "signature; report this as a kernel bug.");
        far_lat = F0;
        for (const CoEdge* ce : body.edges[far_vert].coedges()) {
            const std::size_t f = face_of_coedge(ce);
            if (f != near_lat) far_lat = f;
        }
        if (far_lat == F0 || far_lat == near_lat)
            refuse("the far corner's lateral was not found - the "
                   "class signature is the box's own corner; "
                   "report this as a kernel bug.");
    };
    std::size_t far_vert_r = E0, far_ring_r = E0, far_lat_r = F0;
    std::size_t far_vert_p = E0, far_ring_p = E0, far_lat_p = F0;
    far_junctions(v_far, edge, lat_ref, far_vert_r, far_ring_r,
                  far_lat_r);
    far_junctions(v_far_part, part_edge, lat_part, far_vert_p,
                  far_ring_p, far_lat_p);

    // ---- LE BANDE (the typed Linear eps_L world): the faces'
    // own CURRENT reaches (the m1 read: the extent from the edge,
    // the payload's own vertices), then THE STAGE BAND d < 1
    // (D5.1's derived wall — the act IS the multi-edge state, its
    // k ≥ 1 by construction), the two readings in the payload.
    const auto reach_of = [&](std::size_t f,
                              const math::Vec3d& p_on_edge,
                              const math::Vec3d& along) -> double {
        // The face's extent from the edge's line, along the
        // in-face direction (the max over the face's own
        // vertices, the sign chosen by the vertices themselves).
        const math::Vec3d cand[2] = {along, along * -1.0};
        double best = -1.0;
        for (const auto& c : cand) {
            double m = 0.0;
            for (const auto& loop :
                 body.solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges()) {
                    const auto& e = body.edges[ce.edge()];
                    for (const std::size_t vi :
                         {static_cast<std::size_t>(e.vertex(0)),
                          static_cast<std::size_t>(e.vertex(1))}) {
                        const math::Vec3d& q = vpos(vi);
                        m = std::max(m, (q - p_on_edge).dot(c));
                    }
                }
            best = std::max(best, m);
        }
        return best;
    };
    {
        // The in-face directions: the face's normal crossed with
        // the edge's direction (the two laterals' z-axis and the
        // cap's x/y — derived, never assumed).
        const math::Vec3d edge_dir =
            (p1 - p0) * (1.0 / std::sqrt((p1 - p0).length_sq()));
        const math::Vec3d in_cap =
            n_cap.cross(edge_dir * (x_par ? 1.0 : -1.0));
        const double reach_cap =
            reach_of(cap_face, p0, in_cap);
        const double reach_latr =
            reach_of(lat_ref, p0, n_latr.cross(edge_dir));
        const math::Vec3d part_dir =
            (vpos(v_far_part) - vpos(v_part)) *
            (1.0 / std::sqrt((vpos(v_far_part) - vpos(v_part))
                                 .length_sq()));
        const double reach_latp =
            reach_of(lat_part, vpos(v_part),
                     n_latp.cross(part_dir));
        const auto band_refusal = [&](const char* what,
                                      double reach) {
            refuse(std::string("the setback crosses the band "
                               "(chamfer_setback_exceeds_face): ") +
                   what + " — the setback " +
                   std::to_string(setback) +
                   " against the face's own current reach " +
                   std::to_string(reach) + " - " +
                   std::to_string(tol) +
                   " (the reach read from the CURRENT face's own "
                   "vertices, the previous chamfers' retreats "
                   "included — the m1 machinery's own law, the "
                   "strip law of D1.3). Remedy: author a setback "
                   "within the face's reach.");
        };
        if (setback >= reach_cap - tol)
            band_refusal("the setback line exits the cap face",
                         reach_cap);
        if (setback >= reach_latr - tol)
            band_refusal(
                "the setback line exits the reference's lateral",
                reach_latr);
        if (setback >= reach_latp - tol)
            band_refusal(
                "the setback line exits the partner's lateral",
                reach_latp);
        if (setback >= 1.0 - tol)
            refuse("the setback crosses the STAGE band "
                   "(chamfer_setback_exceeds_face): the setback " +
                   std::to_string(setback) +
                   " against the stage's derived bound 1.000000 - " +
                   std::to_string(tol) +
                   " (THE TWO READINGS: the faces' own current "
                   "reach " +
                   std::to_string(std::min(
                       reach_cap,
                       std::min(reach_latr, reach_latp))) +
                   " would admit it - the ring pair's own "
                   "face bands, satisfied - but the multi-edge "
                   "state's strips must not meet, 2d < 2, and at "
                   "d = 1 the six planes die to points - the "
                   "tangential collapse, ADR-0045 D5.1's "
                   "derivation; the body is not the limit, the "
                   "fillet's r=1 precedent). Remedy: keep the "
                   "chain's setbacks below 0.999999 (the uniform "
                   "twelve-edge mode's own derivation is the ring "
                   "act's morse, D4).");
    }

    // ---- THE CONSTRUCTION — every quantity a closed form of the
    // stored operands (D2's contract): the miter, the retreats,
    // the far feet.
    const math::Vec3d M =
        corner - (n_latr + n_latp + n_cap) * (0.5 * setback);
    const math::Vec3d P_lr =
        corner - (n_latp + n_cap) * setback;
    const math::Vec3d P_lp =
        corner - (n_latr + n_cap) * setback;
    const math::Vec3d P_cap =
        corner - (n_latr + n_latp) * setback;
    const math::Vec3d& p_far = vpos(v_far);
    const math::Vec3d far_rc = p_far - n_latr * setback;
    const math::Vec3d far_rl = p_far - n_cap * setback;
    const math::Vec3d& p_far_p = vpos(v_far_part);
    const math::Vec3d far_pc = p_far_p - n_latp * setback;
    const math::Vec3d far_pl = p_far_p - n_cap * setback;

    // ---- The maps: the survivors compact (the four dead dropped),
    // the born append (the numbering table: M first, then the
    // three retreats, then the four far feet; the miter lines
    // first among the born edges, then the setback lines, then
    // the far-end segments).
    std::vector<std::size_t> vmap(V0, V0), emap(E0, E0);
    {
        const std::size_t dead_v[4] = {v_occ, v_part, v_far,
                                       v_far_part};
        std::size_t next = 0;
        for (std::size_t i = 0; i < V0; ++i) {
            bool dead = false;
            for (const std::size_t d : dead_v)
                if (i == d) dead = true;
            if (!dead) vmap[i] = next++;
        }
        const std::size_t dead_e[3] = {edge, part_edge, cap_seg};
        next = 0;
        for (std::size_t i = 0; i < E0; ++i) {
            bool dead = false;
            for (const std::size_t d : dead_e)
                if (i == d) dead = true;
            if (!dead) emap[i] = next++;
        }
    }
    const std::size_t v_base = V0 - 4;
    const std::size_t v_M = v_base + 0;
    const std::size_t v_P_lr = v_base + 1;
    const std::size_t v_P_lp = v_base + 2;
    const std::size_t v_P_cap = v_base + 3;
    const std::size_t v_frc = v_base + 4;
    const std::size_t v_frl = v_base + 5;
    const std::size_t v_fpc = v_base + 6;
    const std::size_t v_fpl = v_base + 7;
    const std::size_t e_base = E0 - 3;
    const std::size_t e_mVR = e_base + 0;  // (P_lr -> M)
    const std::size_t e_mVB = e_base + 1;  // (P_lp -> M)
    const std::size_t e_mRB = e_base + 2;  // (P_cap -> M)
    const std::size_t e_src = e_base + 3;  // (frc -> P_cap)
    const std::size_t e_srl = e_base + 4;  // (frl -> P_lr)
    const std::size_t e_spc = e_base + 5;  // (fpc -> P_cap)
    const std::size_t e_spl = e_base + 6;  // (fpl -> P_lp)
    const std::size_t e_ffr = e_base + 7;  // (frl -> frc)
    const std::size_t e_ffp = e_base + 8;  // (fpl -> fpc)

    // The per-edge substitution (the 17/11 law: each re-spanned
    // edge's new end read from ITS OWN junction's role — the
    // wires retreat along their own laterals, the far verticals
    // land on the reference/partner laterals' feet, the far ring
    // edges on the cap's feet).
    const auto is_dead_v = [&](std::size_t v) {
        return v == v_occ || v == v_part || v == v_far ||
               v == v_far_part;
    };
    const auto new_end = [&](std::size_t old_v,
                             std::size_t ei) -> std::size_t {
        if (!is_dead_v(old_v)) return vmap[old_v];
        if (ei == wire_ref) return v_P_lr;
        if (ei == wire_part) return v_P_lp;
        if (ei == far_vert_r) return v_frl;
        if (ei == far_vert_p) return v_fpl;
        if (ei == far_ring_r) return v_frc;
        if (ei == far_ring_p) return v_fpc;
        refuse("a dead vertex on an edge outside the re-spanned "
               "set - report this as a kernel bug.");
        return 0;  // unreachable — refuse() throws
    };

    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();

    // ---- The vertices: the survivors, then the born eight.
    for (std::size_t i = 0; i < V0; ++i) {
        if (vmap[i] == V0) continue;
        R->vertices.push_back(body.vertices[i]);
        R->vertices.back().set_id(static_cast<VertexId>(vmap[i]));
    }
    {
        const math::Vec3d born[8] = {M,    P_lr,  P_lp, P_cap,
                                     far_rc, far_rl, far_pc,
                                     far_pl};
        const std::size_t ids[8] = {v_M,   v_P_lr, v_P_lp, v_P_cap,
                                    v_frc, v_frl,  v_fpc,  v_fpl};
        for (std::size_t i = 0; i < 8; ++i) {
            Vertex v{born[i]};
            v.set_id(static_cast<VertexId>(ids[i]));
            v.set_provenance(prov);
            R->vertices.push_back(v);
        }
    }

    // ---- The edges: the survivors (the endpoints remapped), then
    // the born nine.
    for (std::size_t i = 0; i < E0; ++i) {
        if (emap[i] == E0) continue;
        R->edges.push_back(body.edges[i]);
        auto& e = R->edges.back();
        e.set_id(static_cast<EdgeId>(emap[i]));
        e.set_vertex(0, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(e.vertex(0)),
                            i)));
        e.set_vertex(1, static_cast<VertexId>(new_end(
                            static_cast<std::size_t>(e.vertex(1)),
                            i)));
    }
    {
        const std::pair<std::size_t, std::size_t> born[9] = {
            {v_P_lr, v_M},   {v_P_lp, v_M},   {v_P_cap, v_M},
            {v_frc, v_P_cap}, {v_frl, v_P_lr}, {v_fpc, v_P_cap},
            {v_fpl, v_P_lp}, {v_frl, v_frc},  {v_fpl, v_fpc}};
        for (const auto& [a, b] : born) {
            Edge e{static_cast<VertexId>(a),
                   static_cast<VertexId>(b)};
            e.set_id(static_cast<EdgeId>(R->edges.size()));
            e.set_provenance(prov);
            R->edges.push_back(e);
        }
    }

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
            refuse("the re-authored walk lost the chaining - "
                   "report this as a kernel bug.");
        CoEdge ce{static_cast<EdgeId>(e_id), rev};
        ce.set_p_curve(pcurve_segment(
            chamfer_detail::dom_of(pl, pos3(a)),
            chamfer_detail::dom_of(pl, pos3(b)),
            /*interior_left=*/true));
        loop.add_coedge(ce);
    };

    // ---- The re-authoring (the 17/11 law verbatim: the walk on
    // the old loop's own cyclic order, every span's end read from
    // the junctions — the dead runs replaced by the born chains,
    // the dead vertices' substitutes bridged, never an ordering
    // of coordinates). Kept co-edges copy their p-curves verbatim;
    // re-spanned ones re-project; the born bridges project fresh.
    const auto is_dead_edge = [&](std::size_t ei) {
        return ei == edge || ei == part_edge || ei == cap_seg;
    };
    const auto reauthor = [&](std::size_t face_index,
                              const PlaneSurface& pl,
                              const std::vector<std::size_t>&
                                  born_here) {
        const Face& src =
            body.solid.shell().faces()[face_index];
        const auto& ces = src.loops()[0].coedges();
        Loop loop;
        std::vector<bool> used(born_here.size(), false);
        const auto bridge = [&](std::size_t from_v,
                                std::size_t to_v) {
            std::size_t cur = from_v;
            for (std::size_t guard = 0; cur != to_v; ++guard) {
                if (guard > born_here.size())
                    refuse("the re-authoring lost the bridge (a "
                           "born chain that does not reach the "
                           "next surviving co-edge) - report this "
                           "as a kernel bug.");
                bool advanced = false;
                for (std::size_t bi = 0; bi < born_here.size();
                     ++bi) {
                    if (used[bi]) continue;
                    const auto& be = R->edges[born_here[bi]];
                    const std::size_t b0 = static_cast<std::size_t>(
                        be.vertex(0));
                    const std::size_t b1 = static_cast<std::size_t>(
                        be.vertex(1));
                    if (b0 == cur) {
                        emit_planar(loop, pl, born_here[bi], cur,
                                    b1);
                        used[bi] = true;
                        cur = b1;
                        advanced = true;
                        break;
                    }
                    if (b1 == cur) {
                        emit_planar(loop, pl, born_here[bi], cur,
                                    b0);
                        used[bi] = true;
                        cur = b0;
                        advanced = true;
                        break;
                    }
                }
                if (!advanced)
                    refuse("the re-authoring lost the bridge (no "
                           "born edge at the walk's position) - "
                           "report this as a kernel bug.");
            }
        };
        std::size_t first_start = V0 * 2;
        std::size_t cur = V0 * 2;
        for (const auto& ce : ces) {
            const std::size_t ei = ce.edge();
            if (is_dead_edge(ei)) continue;
            const std::size_t s_old = static_cast<std::size_t>(
                ce.reversed()
                    ? body.edges[ei].vertex(1)
                    : body.edges[ei].vertex(0));
            const std::size_t e_old = static_cast<std::size_t>(
                ce.reversed()
                    ? body.edges[ei].vertex(0)
                    : body.edges[ei].vertex(1));
            const std::size_t s_new = new_end(s_old, ei);
            const std::size_t e_new = new_end(e_old, ei);
            if (cur == V0 * 2) {
                first_start = s_new;
                cur = s_new;
            } else if (cur != s_new) {
                bridge(cur, s_new);
            }
            CoEdge out{static_cast<EdgeId>(emap[ei]), false};
            const auto& stored = R->edges[emap[ei]];
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
                refuse("the re-authored walk lost the chaining "
                       "(the stored pair does not run between "
                       "the walk's endpoints) - report this as a "
                       "kernel bug.");
            out.set_reversed(rev);
            if (is_dead_v(s_old) || is_dead_v(e_old)) {
                out.set_p_curve(pcurve_segment(
                    chamfer_detail::dom_of(pl, pos3(s_new)),
                    chamfer_detail::dom_of(pl, pos3(e_new)),
                    /*interior_left=*/true));
            } else if (ce.p_curve() != nullptr) {
                out.set_p_curve(*ce.p_curve());
            }
            loop.add_coedge(out);
            cur = e_new;
        }
        if (cur == V0 * 2)
            refuse("the face's walk is entirely dead - the class "
                   "signature; report this as a kernel bug.");
        if (cur != first_start) bridge(cur, first_start);
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

    // ---- The faces: the kept copy verbatim, the six touched
    // re-authored, the two ring bevels born.
    for (std::size_t f = 0; f < F0; ++f) {
        const Face& src = body.solid.shell().faces()[f];
        std::vector<std::size_t> born_here;
        if (f == cap_face) {
            born_here = {e_src, e_spc};
        } else if (f == lat_ref) {
            born_here = {e_srl};
        } else if (f == lat_part) {
            born_here = {e_spl};
        } else if (f == far_lat_r) {
            born_here = {e_ffr};
        } else if (f == far_lat_p) {
            born_here = {e_ffp};
        } else if (f == vert_face) {
            born_here = {e_mVR, e_mVB};
        }
        bool touched = !born_here.empty();
        if (!touched)
            for (const auto& ce : src.loops()[0].coedges()) {
                const std::size_t ei = ce.edge();
                if (is_dead_edge(ei)) {
                    touched = true;
                    break;
                }
                const auto& e = body.edges[ei];
                if (is_dead_v(static_cast<std::size_t>(
                        e.vertex(0))) ||
                    is_dead_v(static_cast<std::size_t>(
                        e.vertex(1)))) {
                    touched = true;
                    break;
                }
            }
        if (!touched) {
            // KEPT: the record and the walk verbatim, the edge ids
            // remapped (nothing re-spanned here — the p-curves
            // copy whole).
            const PlaneSurface* pl = plane_of(f);
            if (pl == nullptr)
                refuse("a kept face is not a plane - the class "
                       "signature; report this as a kernel bug.");
            Face face(std::make_shared<PlaneSurface>(*pl));
            for (const auto& loop : src.loops()) {
                Loop out;
                for (const auto& ce : loop.coedges()) {
                    CoEdge c{static_cast<EdgeId>(emap[ce.edge()]),
                             ce.reversed()};
                    if (ce.p_curve() != nullptr)
                        c.set_p_curve(*ce.p_curve());
                    out.add_coedge(c);
                }
                face.add_loop(std::move(out));
            }
            face.set_id(static_cast<FaceId>(f));
            face.set_shape_id(src.shape_id());
            face.set_name(src.name());
            face.set_provenance(src.provenance());
            auto& faces_mut = shell.faces_mut();
            if (faces_mut.size() <= f) faces_mut.resize(f + 1);
            faces_mut[f] = std::move(face);
        } else {
            const PlaneSurface* pl = plane_of(f);
            if (pl == nullptr)
                refuse("a re-authored face is not a plane - the "
                       "class signature; report this as a kernel "
                       "bug.");
            reauthor(f, *pl, born_here);
        }
    }

    // ---- The born edges' naming (the act's own vocabulary): the
    // two ring bevels, their setback wires and far-end segments
    // (the m1 isect grammar), and the three miter lines (the new
    // role — the bevel×bevel concurrence, the legs the bevels' own
    // paths, rendered under the act's binding as the fillet's
    // coaxial wires' own pattern: the PATH the faithful identity).
    const Face& src_A_ref = body.solid.shell().faces()[face_A];
    const Face& src_B_ref = body.solid.shell().faces()[face_B];
    const Face& src_A2 = body.solid.shell().faces()[face_A2];
    const Face& src_B2 = body.solid.shell().faces()[face_B2];
    const Face& src_V = body.solid.shell().faces()[vert_face];
    const Face& src_flr =
        body.solid.shell().faces()[far_lat_r];
    const Face& src_flp =
        body.solid.shell().faces()[far_lat_p];
    const bool named =
        naming.body != nullptr &&
        src_A_ref.shape_id().subshape.depth > 0 &&
        src_B_ref.shape_id().subshape.depth > 0 &&
        src_A2.shape_id().subshape.depth > 0 &&
        src_B2.shape_id().subshape.depth > 0 &&
        src_V.shape_id().subshape.depth > 0 &&
        src_flr.shape_id().subshape.depth > 0 &&
        src_flp.shape_id().subshape.depth > 0;
    naming::OperandBinding che_operands[2] = {
        {src_A_ref.shape_id().feature, naming.body},
        {naming.feature, nullptr}};
    const naming::OperationNaming che_decl{
        naming::FeatureKind::Chamfer,
        naming::kChamferFaceRoles, naming::kChamferEdgeRoles,
        che_operands};
    che_operands[1].naming = &che_decl;
    const auto leg = [](geometry::SubshapeRef ref,
                        const Face& operand) {
        for (std::uint8_t k = 0;
             k < operand.shape_id().subshape.depth; ++k)
            ref = ref.push(
                operand.shape_id().subshape.path[k]);
        return ref;
    };
    const auto bevel_ref_of = [&](const Face& fa,
                                  const Face& fb) {
        geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(
                naming::kCheBevelRole);
        ref = leg(ref, fa);
        ref = leg(ref, fb);
        return ref;
    };
    const auto stamp_bevel = [&](std::size_t f_new, const Face& fa,
                                 const Face& fb) {
        if (!named) return;
        const geometry::SubshapeRef ref =
            bevel_ref_of(fa, fb);
        auto& face = shell.faces_mut()[f_new];
        face.set_shape_id({naming.feature, ref, {}});
        face.set_name(naming::render_name(
            naming.feature, ref, che_decl.faces()));
    };
    const auto stamp_wire = [&](std::size_t e_new,
                                const Face& landing,
                                const Face& fa, const Face& fb) {
        if (!named) return;
        geometry::SubshapeRef wref =
            geometry::SubshapeRef{}.push(
                naming::kCheIsectRole);
        wref = leg(wref, landing);
        wref = wref.push(naming::kCheBevelRole);
        wref = leg(wref, fa);
        wref = leg(wref, fb);
        R->edges[e_new].set_shape_id(
            {naming.feature, wref, {}});
        R->edges[e_new].set_name(naming::render_name(
            naming.feature, wref, che_decl.edges()));
    };
    const auto stamp_miter = [&](std::size_t e_new,
                                 const geometry::SubshapeRef& r1,
                                 const geometry::SubshapeRef& r2) {
        if (!named) return;
        geometry::SubshapeRef wref =
            geometry::SubshapeRef{}.push(
                naming::kCheMiterRole);
        for (std::uint8_t k = 0; k < r1.depth; ++k)
            wref = wref.push(r1.path[k]);
        for (std::uint8_t k = 0; k < r2.depth; ++k)
            wref = wref.push(r2.path[k]);
        R->edges[e_new].set_shape_id(
            {naming.feature, wref, {}});
        R->edges[e_new].set_name(naming::render_name(
            naming.feature, wref, che_decl.edges()));
    };

    // ---- The two ring bevels are born GIÀ RIFINITI (the
    // pentagon: two setback lines, two miter lines at the corner
    // end, the far-end segment), the walk CCW in the bevel's own
    // domain (the material on the left — the signed area read
    // from the feet's own domain coordinates, the orientation
    // corrected deterministically when the octant mirrors it).
    const auto born_bevel = [&](std::size_t f_new,
                                const math::Vec3d& n_1,
                                const math::Vec3d& n_2,
                                const math::Vec3d& p_near,
                                const math::Vec3d& p_far_e,
                                std::size_t e_sb_cap,
                                std::size_t e_sb_lat,
                                std::size_t e_m_cap,
                                std::size_t e_m_lat,
                                std::size_t e_far,
                                std::size_t v_cap_near,
                                std::size_t v_cap_far,
                                std::size_t v_lat_near,
                                std::size_t v_lat_far) {
        const math::Vec3d bis = n_1 + n_2;
        const math::Vec3d bis_unit =
            bis * (1.0 / std::sqrt(bis.length_sq()));
        const math::Vec3d mid =
            (p_near + p_far_e) * 0.5;
        auto surf = std::make_shared<PlaneSurface>(
            mid - bis * (0.5 * setback), bis_unit);
        const PlaneSurface& pl = *surf;
        // The pentagon's own cyclic chain: the cap's far foot ->
        // the cap retreat -> M -> the lateral retreat -> the
        // lateral far foot (the boundary's own order).
        const std::size_t chain[5][3] = {
            {e_sb_cap, v_cap_far, v_cap_near},
            {e_m_cap, v_cap_near, v_M},
            {e_m_lat, v_M, v_lat_near},
            {e_sb_lat, v_lat_near, v_lat_far},
            {e_far, v_lat_far, v_cap_far}};
        Loop loop;
        for (const auto& c : chain)
            emit_planar(loop, pl, c[0], c[1], c[2]);
        // THE CCW CORRECTION (the shared rule, the frame's octant
        // never assumed): measured in the actual domain, reversed
        // only when mirrored.
        chamfer_detail::ensure_ccw(loop);
        Face face(std::move(surf));
        face.add_loop(std::move(loop));
        face.set_id(static_cast<FaceId>(f_new));
        face.set_provenance(prov);
        auto& faces_mut = shell.faces_mut();
        if (faces_mut.size() <= f_new)
            faces_mut.resize(f_new + 1);
        faces_mut[f_new] = std::move(face);
    };
    // R (the reference's ring bevel): the cap and the reference's
    // lateral; the corner end = the occupied end.
    born_bevel(F0, n_cap, n_latr, vpos(v_occ), vpos(v_far),
               e_src, e_srl, e_mRB, e_mVR, e_ffr, v_P_cap, v_frc,
               v_P_lr, v_frl);
    // B (the partner's): the cap and the partner's lateral.
    born_bevel(F0 + 1, n_cap, n_latp, vpos(v_part),
               vpos(v_far_part), e_spc, e_spl, e_mRB, e_mVB,
               e_ffp, v_P_cap, v_fpc, v_P_lp, v_fpl);
    stamp_bevel(F0, src_A_ref, src_B_ref);
    stamp_bevel(F0 + 1, src_A2, src_B2);
    // The setback wires and the far-end segments: the m1 isect
    // grammar (the landing face first, the bevel nested).
    stamp_wire(e_src, src_A_ref, src_A_ref, src_B_ref);
    stamp_wire(e_srl, src_B_ref, src_A_ref, src_B_ref);
    stamp_wire(e_spc, src_A2, src_A2, src_B2);
    stamp_wire(e_spl, src_B2, src_A2, src_B2);
    stamp_wire(e_ffr, src_flr, src_A_ref, src_B_ref);
    stamp_wire(e_ffp, src_flp, src_A2, src_B2);
    // The three miter lines: the carrier's bevel first where it
    // is a party (V×R, V×B), the act's two in their declared order
    // (R×B) — the first leg pushed verbatim from the bevel's own
    // stamp (the carrier's che#2 path, faithful), the second built
    // from the ring bevel's operand legs.
    {
        const geometry::SubshapeRef ref_V =
            src_V.shape_id().subshape;
        const geometry::SubshapeRef ref_R =
            bevel_ref_of(src_A_ref, src_B_ref);
        const geometry::SubshapeRef ref_B =
            bevel_ref_of(src_A2, src_B2);
        stamp_miter(e_mVR, ref_V, ref_R);
        stamp_miter(e_mVB, ref_V, ref_B);
        stamp_miter(e_mRB, ref_R, ref_B);
    }

    // ---- THE OUTPUT GATE (the act's own arithmetic, hard
    // postconditions).
    link_radial_edges(*R);
    if (!loops_are_closed(*R))
        throw std::runtime_error(
            "chamfer_corner_act: postcondition failed - a loop "
            "does not chain (an assembly defect in the act's "
            "re-authored walks; report this as a kernel bug)");
    if (!is_closed_orientable(*R))
        throw std::runtime_error(
            "chamfer_corner_act: postcondition failed - the "
            "closed-orientable contract broke (every edge exactly "
            "two opposite co-edges; report this as a kernel bug)");
    if (R->vertex_count() != V0 - 4 + 8 ||
        R->edge_count() != E0 - 3 + 9 ||
        R->face_count() != F0 + 2)
        throw std::runtime_error(
            "chamfer_corner_act: postcondition failed - the "
            "counts moved (the act's own arithmetic is V-4+8 / "
            "E-3+9 / F+2; report this as a kernel bug)");
    if (euler_characteristic(*R) != euler_characteristic(body))
        throw std::runtime_error(
            "chamfer_corner_act: postcondition failed - chi moved "
            "(the act is chi-flat by construction, Delta-chi = "
            "(+4) + (-6) + (+2) = 0 counting the net; report this "
            "as a kernel bug)");
    return R;
}

} // namespace CAD_0::brep
