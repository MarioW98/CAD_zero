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
    // construction — the counts and the families, zero dihedral
    // arithmetic, zero convexity signs). This morse's carrier: THE
    // FACTORY BOX (k = 0) — 8/12/6, chi = 2, closed orientable,
    // every entity exact, every face a single-loop PLANE. The chain's
    // widening is the next morse's seat (the header's own words); a
    // curved face anywhere (a fillet's flank among them) refuses
    // here — the mixed corner is the crease merge's family (D5.3),
    // never met, never sedated.
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
                   "Layer-2 record says so) - the exact chamfer "
                   "consumes EXACT bodies only (the booleans' own "
                   "gate). Remedy: author the body on the exact path "
                   "(the factories), or accept the faceted path.");
        if (V0 != 8 || E0 != 12 || F0 != 6 ||
            euler_characteristic(body) != 2 ||
            !is_closed_orientable(body))
            refuse("the body is outside the class signature (this "
                   "morse's carrier is the FACTORY BOX alone: 8 "
                   "vertices, 12 edges, 6 faces, chi = 2, closed "
                   "orientable; found " +
                   std::to_string(V0) + "/" + std::to_string(E0) +
                   "/" + std::to_string(F0) +
                   ") - the CHAIN (a box carrying k previous "
                   "vertical chamfers, 8+2k/12+3k/6+k) is the next "
                   "morse's own seat (ADR-0045 D8.2, the fillet's "
                   "own history: m2 the box, the m6ter the chain), "
                   "and a body carrying curved faces is the mixed "
                   "corner's family (D5.3, the crease merge's seat, "
                   "ADR-0039 D4). Remedy: chamfer a factory box's "
                   "vertical edge.");
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
                       "families (the box's six planes are this "
                       "morse's whole world; a curved face is the "
                       "mixed corner's family - the crease merge's "
                       "seat, ADR-0039 D4, ADR-0045 D5.3). Remedy: "
                       "chamfer a factory box's vertical edge.");
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

} // namespace CAD_0::brep
