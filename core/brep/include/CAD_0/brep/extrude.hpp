// core/brep/include/CAD_0/brep/extrude.hpp
//
// THE EXACT EXTRUDE — ADR-0032 D6 (G1 unit 3): the sketch profile
// becomes a prism BREP with true faces. The typed profile chain IS
// the trimming: the lateral faces come from the entities (D6's
// mapping table), the caps' outer loop IS the chain itself, entity
// for entity — the 2D kernel's region semantics and the cap's
// parameter-domain region are the same mathematics (D2.1's one
// opinion, made real: segment→Line2, arc→Arc2, circle→Circle2 on
// the hole rims — this unit is the vocabulary's first Circle2
// writer).
//
// THE MAPPING (D6's table, fixed):
//
//   | Profile entity            | Lateral face   | Trimming              |
//   |---------------------------|----------------|-----------------------|
//   | Segment (length ℓ)       | Plane          | Rectangle, 4 Line2    |
//   | Arc (radius r, span Δθ)  | Cylindrical    | Δθ × h, 4 Line2       |
//   | Circle (closed, solid)   | Full cylinder  | ADR-0023 seam-wired   |
//   | Circle (closed, hole)    | Full cylinder  | seam-wired, u flipped |
//
// THE PINNED COUNTS (derived in D6, pinned by the tests):
//   * a closed chain of n entities → V = 2n, E = 3n, F = n+2,
//     χ = 2 for every n — the box is the n = 4 case (8/12/6);
//   * the solid circle profile → the ADR-0023 seam counts
//     4V/5E/3F, χ = 2 (the factory cylinder's pattern VERBATIM,
//     on the Z axis through the circle's centre);
//   * the MULTI-CHAIN extension (the ring — a tested flow of the
//     SDF extrude): the outer chain as above + one wall and one
//     inner cap loop per hole: each hole h contributes
//     (2n_h, 3n_h, n_h + the loop pair), H = 2·(holes) and
//     χ = 2 − 2·(holes) — the tube reads χ = 0, genus 1.
//
// AUTHORING CONVENTIONS (pcurve.hpp's, second real use — where the
// first use forced a correction, this one forces a documented
// boundary):
//
//   * DIRECTION — every p-curve runs in its co-edge's traversal
//     direction (the start/end are the co-edge's vertex images; on
//     periodic walls the u candidate may shift by ±2π — the seam
//     discipline the factories established);
//   * REGION SIDE — the LATERAL faces are authored material-on-left
//     in their own domains (the window/rectangle IS the face; the
//     factories' discipline, interior_left = true throughout). The
//     CAPS keep the SKETCH'S OWN BITS verbatim (interior_left /
//     hole as authored): the cap's region is the profile region,
//     and the entity's declared side is where the material lies —
//     the same mathematics, no translation. A cap loop therefore
//     reads CCW-outer/CW-inner ONLY for material-left chains; for
//     material-right chains (a code-authored clockwise contour) the
//     walk adapts and the bits carry the region;
//   * FRAMES — every surface's (u,v) frame is right-handed around
//     the face's outward normal (analytic.hpp's rule). This forces
//     CylinderZSurface's u-sign: +1 on the solid side (the fillet
//     arcs, the solid circle's wall — outward +radial), −1 on the
//     hole side (the hole walls — outward −radial, the material
//     outside). The hole rims' caps carry Circle2 trim curves: a
//     closed curve has no traversal direction to pin, exactly the
//     vocabulary's honest answer for a full-circle rim.
//
// THE CLASS BOUNDARY (loud, actionable — the vocabulary's own limit,
// the same honest shape as ADR-0032 D5.3's oblique-SSI exclusion):
// a hole-side ARC (hole = true — a notch in the outer contour or a
// bite in a hole) makes the exact B-Rep REFUSE. The typed 2D
// vocabulary stores CCW sweeps only, and a notch arc's cap trimming
// runs CW in BOTH cap domains (the mirror and the rotation both
// preserve the walk's handedness against the vocabulary's) — the
// p-curve is not expressible in G1. The SDF materialization stands
// (the display path never changes); the caller records the refusal.
// All-segment contours of EITHER winding and all fillet-side arcs
// are in class.
//
// NUMBERING (ADR-0005's discipline, the factories' rule: stable ids
// by construction order — the tables the tests pin):
//
//   Single chain of n entities (segments/arcs), joints
//   J_i = entity i's end point (entity i runs J(i-1) → J_i, mod n):
//     vertices : 2i = J_i at z_lo, 2i+1 = J_i at z_hi
//     edges    : i = entity i's bottom curve (authored direction),
//                n+i = its top curve, 2n+i = the vertical at J_i
//     faces    : i = entity i's lateral face, n = the -Z cap,
//                n+1 = the +Z cap
//   Solid circle (centre C, radius r):
//     vertices : 0/1 = bottom seam/antipode (seam = +X from C),
//                2/3 = the top pair
//     edges    : 0/1 = the bottom rim halves (0→π via +Y, π→2π via
//                -Y), 2/3 = the top rim halves, 4 = the seam
//     faces    : 0 = the wall, 1 = the -Z cap, 2 = the +Z cap
//   Multi-chain: the outer chain first (the tables above), then each
//   hole chain in sketch order (the same per-chain layout; a hole
//   circle contributes 2 vertices [bottom/top seam], 3 edges [the
//   two closed rims + the seam] and 1 wall face), then the caps.
//   The -Z cap is added before the +Z cap; on each cap the outer
//   loop is the first loop, the holes follow in sketch order.
//
// THE NAMING (ADR-0036 D1, H2-u6 morso 3 — the walk roles made
// NORMATIVE): the tables above are the role tables' own walk, and
// the vocabulary is naming.hpp's kExtFaceRoles/kExtEdgeRoles —
// lat(i) for chain entity i's lateral, cap(lo)/cap(hi) for the
// region's two lids, the seam for the seam-wired vertical
// (ADR-0023's own e_seam). Multi-chain bodies prefix the chain
// group on the CHAIN-OWNED roles ('c0:lat(i)', 'c1:seam' — the
// group is the numbering table's own multi-chain layout: the outer
// first — group 0 — then each hole in sketch order); the caps are
// the extruded region's own and NEVER carry the group. The names
// are stamped IN the pass that builds, beside the walk indices the
// construction already holds (zero extra passes); the walk's other
// edges — the bottom/top curves, the verticals, the rims — are
// outside D1's v1 extrude vocabulary and read honestly EMPTY. The
// tavola (the born-of genealogy) derives in descent.hpp's
// extrude_descent, from these same tables.
//
// Layer-2 (ADR-0032 D2.4): the extrude writes exact/0 on every
// subshape — the factories' discipline (the ProvenanceInfo default;
// a constructed point is the mathematical ideal, the burden sits on
// the path that would sample). The NAME sits BESIDE the Layer-2
// record (ADR-0036 D6: identity is not arithmetic honesty — no
// mother touched).
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::brep {

namespace extrude_detail {

// The periodic domain's constants, DOUBLE since morso 3 (ADR-0035
// D4): they feed the p-curve payloads, the BREP parametric domain's
// own world — the sketch kernel keeps its float constants in its own
// world (mathematical constants, not mirrored layouts).
inline constexpr double kPi = 3.14159265358979;
inline constexpr double kTwoPi = 6.28318530717958647692;

// One chain's per-entity walk data, resolved once: the joints, the
// authored material side, the per-entity curve endpoints.
struct ChainWalk {
    // The joint AFTER entity i (entity i runs from start[i] to
    // end[i]; end[i] == start[i+1 mod n]).
    std::vector<math::Vec2f> start;
    std::vector<math::Vec2f> end;
    // The chain's uniform authored material side (left = the canvas
    // discipline; right = a code-authored clockwise contour).
    bool material_left{true};
    std::size_t n{0};
};

// The endpoints of entity i in its own authored direction (the
// sketch kernel's own arc points — one opinion, one code).
inline void entity_endpoints(const sdf::SketchEntity& e,
                             math::Vec2f& a, math::Vec2f& b) {
    switch (e.kind) {
    case sdf::SketchEntity::Kind::Segment:
        a = e.segment.a;
        b = e.segment.b;
        return;
    case sdf::SketchEntity::Kind::Arc:
        a = sdf::arc_start_point(e.arc);
        b = sdf::arc_end_point(e.arc);
        return;
    case sdf::SketchEntity::Kind::Circle:
        a = e.circle.center;
        b = e.circle.center;
        return;
    }
}

// The chain's signed enclosed area, in walk order (segments: the
// shoelace term; arcs: the circular-segment term
// [r(C×(u_e−u_s)) + r²Δ]/2; a lone circle: ±πr² by its role sign —
// the magnitude is what the outer/hole ranking consumes). Derived
// from ∮(x dy − y dx)/2 over the authored walk.
inline double chain_signed_area(const sdf::SketchChain& c) {
    if (c.entities.size() == 1 &&
        c.entities[0].kind == sdf::SketchEntity::Kind::Circle)
        return kPi * c.entities[0].circle.radius *
               c.entities[0].circle.radius;
    double twice = 0.0;
    for (const auto& e : c.entities) {
        switch (e.kind) {
        case sdf::SketchEntity::Kind::Segment:
            twice += e.segment.a.x * e.segment.b.y -
                     e.segment.a.y * e.segment.b.x;
            break;
        case sdf::SketchEntity::Kind::Arc: {
            const auto& arc = e.arc;
            const math::Vec2f u_s{std::cos(arc.start_angle),
                                  std::sin(arc.start_angle)};
            const float end_angle = arc.start_angle + arc.sweep;
            const math::Vec2f u_e{std::cos(end_angle),
                                  std::sin(end_angle)};
            const double cross = double(arc.center.x) * (u_e.y - u_s.y) -
                                double(arc.center.y) * (u_e.x - u_s.x);
            twice += double(arc.radius) * cross +
                     double(arc.radius) * double(arc.radius) * arc.sweep;
            break;
        }
        case sdf::SketchEntity::Kind::Circle:
            break;  // unreachable: refused with the chain structure
        }
    }
    return twice * 0.5;
}

// THE SKETCH FRONTIER (ADR-0035 D4): the cap loop co-edge for one
// entity, authored in the cap's own plane domain BY PROJECTION (the
// frame stays the surface's — no second frame copy). This is THE
// named site where sketch entities enter the BREP parametric
// domain: the conversion is float→double, EXACT (every float32 is
// exactly a double — the payload carries the sketch's own numbers
// bit-for-bit), ONE direction (there is no bridge back; the lossy
// direction died with the demotions), and EXPLICIT — every scalar
// crosses as a named static_cast<double> below, never an implicit
// narrowing or widening that hides the boundary. `walk_forward`
// selects the traversal vs the entity's authored direction; the
// SIDE BIT is the authored one (the cap's region is the profile
// region — the same mathematics).
inline CoEdge cap_coedge(const PlaneSurface& cap,
                         const sdf::SketchEntity& e, EdgeId edge,
                         bool walk_forward) {
    math::Vec2f a, b;
    entity_endpoints(e, a, b);
    const auto dom = [&cap](const math::Vec2f& p) {
        // The projection computes in double; the payload IS double
        // (the BREP domain's own world — morso 3 removed the
        // demotion that lived here since ADR-0034 D1's table row).
        const math::Vec3d d =
            math::Vec3d{p.x, p.y, 0.0} - cap.origin();
        return math::Vec2d{d.dot(cap.u_axis()), d.dot(cap.v_axis())};
    };
    CoEdge ce{edge, !walk_forward};
    switch (e.kind) {
    case sdf::SketchEntity::Kind::Segment: {
        const math::Vec2d s = walk_forward ? dom(a) : dom(b);
        const math::Vec2d t = walk_forward ? dom(b) : dom(a);
        ce.set_p_curve(
            pcurve_segment(s, t, e.segment.interior_left));
        return ce;
    }
    case sdf::SketchEntity::Kind::Arc: {
        // The walk's start point in the domain fixes the start angle
        // (atan2); the sweep is the authored span. On the +Z cap
        // (the rotation frame) a forward walk starts at the arc's
        // own start; on the -Z cap (the mirror frame) a reversed
        // walk does — both read CCW in the domain, the only
        // direction the vocabulary stores. The arc's radius and
        // sweep cross the frontier as the EXACT promotions of the
        // sketch's own floats.
        const math::Vec2d c_dom = dom(e.arc.center);
        const math::Vec2d s_pt = walk_forward ? dom(a) : dom(b);
        const math::Vec2d d = s_pt - c_dom;
        const double start = std::atan2(d.y, d.x);
        ce.set_p_curve(pcurve_arc(c_dom,
                                  static_cast<double>(e.arc.radius),
                                  /*hole=*/false, start,
                                  static_cast<double>(e.arc.sweep)));
        return ce;
    }
    case sdf::SketchEntity::Kind::Circle:
        // A full circle is authored as ONE closed trim curve: the
        // Circle2, direction-free (the co-edge's orientation flag
        // still obeys the radial XOR). Unreachable for the outer
        // loop (the solid circle goes through the seam-wired path);
        // this is the hole rim's authoring.
        ce.set_p_curve(
            pcurve_circle(dom(e.circle.center),
                          static_cast<double>(e.circle.radius),
                          e.circle.hole));
        return ce;
    }
    return ce;
}

} // namespace extrude_detail

// make_prism — the exact extrude (ADR-0032 D6). Preconditions, in
// the house's actionable culture:
//   * distance finite and > 0; direction +1/-1 (the SlabZ contract —
//     the same messages, the same remedy);
//   * the chains CERTIFY: the ProfileSDF certificate runs here (the
//     extrude's own precondition, ADR-0030 D2 revision 1 — an open
//     or crossed chain refuses with the certificate's message);
//   * the exact class (this unit's boundary, above): no hole-side
//     arcs — a notch/bite arc's cap trimming is not expressible in
//     the typed 2D vocabulary (CCW sweeps only); the refusal names
//     the entity and the remedy.
//
// The optional `feature` (ADR-0036 D1, H2-u6 morso 3 — the
// factories' own parameter, the same discipline): the DAG node's
// FeatureId, stamped into every entity's ShapeId beside the walk
// role. Defaulted: the standalone callers and the scene's hybrid
// materialization read ext#0 — the token is the pin there, the id
// is the caller's to supply (the DAG identity flows with the
// resolver's morso, D6 rung 4).
inline std::unique_ptr<BRepBody> make_prism(
    const std::vector<sdf::SketchChain>& chains, double distance,
    int direction, geometry::FeatureId feature = {}) {
    // ---- the slab contract (SlabZ's own checks, verbatim in
    // substance) -----------------------------------------------
    if (!std::isfinite(distance) || distance <= 0.0)
        throw std::invalid_argument(
            "make_prism: distance must be finite and > 0, got " +
            std::to_string(distance) +
            " — a non-positive or non-finite extrusion distance "
            "degenerates the slab (empty or NaN: no solid to "
            "extrude). Remedy: extrude with a positive distance.");
    if (direction != 1 && direction != -1)
        throw std::invalid_argument(
            "make_prism: direction must be +1 or -1 (±Z), got " +
            std::to_string(direction) +
            " — the MVP extrudes along the world Z axis only "
            "(ADR-0030 D2). Remedy: pass +1 or -1.");

    // ---- the certificate: ONE opinion with the SDF path ---------
    // (ProfileSDF's constructor runs the full construction
    // certificate — an open chain, a crossing, a degenerate span
    // each refuse HERE with the kernel's own actionable message.
    // The fields are not consumed afterwards: the p-curves are
    // authored from the sketch's own numbers — the same numbers the
    // certificate validated.)
    const sdf::ProfileSDF profile_certificate{chains};
    (void)profile_certificate;

    // ---- the naming (ADR-0036 D1, H2-u6 morso 3): the walk roles --
    // The views: single-chain bodies render the feature form;
    // multi-chain bodies render the group form for the CHAIN-OWNED
    // roles (the numbering table's own multi-chain layout — the
    // outer first, then the holes in sketch order), while the caps
    // stay on the feature view in every body (the region's own,
    // never a chain's). The chain-group counter walks the
    // construction's own sequence: the outer takes 0, each hole the
    // next — read at CALL time by the stamping sites below.
    const bool multi_chain = chains.size() > 1;
    const naming::RoleRegistry lateral_view =
        multi_chain ? naming::kExtNaming.grouped_faces()
                    : naming::kExtNaming.faces();
    const naming::RoleRegistry seam_view =
        multi_chain ? naming::kExtNaming.grouped_edges()
                    : naming::kExtNaming.edges();
    std::uint32_t chain_group = 0;

    // The stamping, in the pass that builds (the morse-2 lesson
    // replicated: the walk role is already in hand — the group, the
    // chain entity's index, the cap's level — zero extra passes, no
    // state, the name a pure function of what the construction
    // holds). The path encoding is the register's own: [<group>?,
    // role index, the role's components] — the group, when present,
    // the path's FIRST component.
    const auto stamp_lateral = [&](Face& face, std::size_t i) {
        geometry::SubshapeRef ref{};
        if (multi_chain)
            ref = ref.push(chain_group);
        ref = ref.push(naming::kExtLatRole)
                  .push(static_cast<std::uint32_t>(i));
        face.set_shape_id({feature, ref, {}});
        face.set_name(
            naming::render_name(feature, ref, lateral_view));
    };
    const auto stamp_seam = [&](Edge& seam_edge) {
        geometry::SubshapeRef ref{};
        if (multi_chain)
            ref = ref.push(chain_group);
        ref = ref.push(naming::kExtSeamRole);
        seam_edge.set_shape_id({feature, ref, {}});
        seam_edge.set_name(naming::render_name(feature, ref, seam_view));
    };

    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // ---- the class checks (the vocabulary's honest boundary) ----
    for (std::size_t c = 0; c < chains.size(); ++c) {
        const auto& ents = chains[c].entities;
        bool saw_left = false, saw_right = false;
        for (std::size_t i = 0; i < ents.size(); ++i) {
            const auto& e = ents[i];
            bool left = true;
            switch (e.kind) {
            case sdf::SketchEntity::Kind::Segment:
                left = e.segment.interior_left;
                break;
            case sdf::SketchEntity::Kind::Arc:
                if (e.arc.hole)
                    throw std::invalid_argument(
                        "make_prism: a hole-side arc (hole = true) is "
                        "outside the exact extrude's class — entity " +
                        std::to_string(i) + " of chain " +
                        std::to_string(c) + " (centre (" +
                        std::to_string(e.arc.center.x) + ", " +
                        std::to_string(e.arc.center.y) + "), radius " +
                        std::to_string(e.arc.radius) +
                        "): the typed 2D vocabulary stores CCW sweeps "
                        "only, and a notch arc's cap trimming runs CW "
                        "in BOTH cap domains — the p-curve is not "
                        "expressible in G1 (the same boundary shape "
                        "as ADR-0032 D5.3's oblique SSI). Remedy: "
                        "author the profile with the material on the "
                        "left of the walk (the canvas's discipline), "
                        "or wait for the G3 spline p-curves.");
                left = true;
                break;
            case sdf::SketchEntity::Kind::Circle:
                left = !e.circle.hole;
                break;
            }
            if (left) saw_left = true;
            else saw_right = true;
        }
        if (saw_left && saw_right)
            throw std::invalid_argument(
                "make_prism: the entities of chain " +
                std::to_string(c) +
                " declare mixed material sides — a certified chain "
                "bounds its region from one side (the certificate "
                "implies uniformity; a mix here is a kernel bug or "
                "an uncertifiable payload). Remedy: check the "
                "orientation bits.");
    }

    // ---- the outer/hole classification --------------------------
    // A single chain IS the outer (a lone circle or contour). With
    // several chains the certificate guarantees exactly one outer
    // boundary (every curve lies on the region's boundary, the
    // curves are disjoint, the region is bounded): the outer is the
    // chain enclosing the largest area — strictly, since the holes
    // live inside it.
    std::size_t outer = 0;
    if (chains.size() > 1) {
        double best = -1.0;
        for (std::size_t c = 0; c < chains.size(); ++c) {
            const double area = std::abs(
                extrude_detail::chain_signed_area(chains[c]));
            if (area > best) {
                best = area;
                outer = c;
            }
        }
        // The MAX must be unique: the outer boundary strictly
        // contains every hole, so its enclosed area is strictly
        // larger. Two holes MAY tie (two identical circles) — only
        // a tie AT the max is the bug class.
        std::size_t at_max = 0;
        for (std::size_t c = 0; c < chains.size(); ++c)
            if (std::abs(extrude_detail::chain_signed_area(
                    chains[c])) == best)
                ++at_max;
        if (at_max != 1)
            throw std::invalid_argument(
                "make_prism: no unique outer chain (" +
                std::to_string(at_max) +
                " chains enclose the maximal area " +
                std::to_string(best) +
                ") — the certified multi-chain structure has one "
                "outer boundary strictly larger than every hole; a "
                "tie at the max is a kernel bug. Remedy: report the "
                "sketch.");
    }

    // The extrusion span (SlabZ's own convention: direction +1 spans
    // z ∈ [0, d], direction -1 spans z ∈ [-d, 0]).
    const double z_lo = direction < 0 ? -distance : 0.0;
    const double z_hi = direction < 0 ? 0.0 : distance;
    const double h = distance;

    const auto vid = [](std::size_t i) {
        return static_cast<VertexId>(i);
    };
    const auto eid = [](std::size_t i) {
        return static_cast<EdgeId>(i);
    };

    // The cap plane domains, created once (their frames are fixed by
    // the constructor's rule: -Z → (u,v) = (x,−y), the mirror;
    // +Z → (u,v) = (−x,−y), the rotation).
    const PlaneSurface cap_lo{math::Vec3d{0.0, 0.0, z_lo},
                              math::Vec3d{0.0, 0.0, -1.0}};
    const PlaneSurface cap_hi{math::Vec3d{0.0, 0.0, z_hi},
                              math::Vec3d{0.0, 0.0, 1.0}};

    // The cap loops, collected while the chains are walked, then
    // appended to the two cap faces in order (outer first, holes in
    // sketch order).
    std::vector<std::vector<CoEdge>> cap_lo_loops;
    std::vector<std::vector<CoEdge>> cap_hi_loops;

    // ---- one general chain (segments/arcs): the OUTER contour -----
    // (The certificate's mathematics makes this the outer boundary's
    // own builder: a multi-entity HOLE is inexpressible in the
    // max-composition kernel — the hole chain's entities' regions
    // would have to intersect to the hole's complement, and an
    // intersection of half-planes is convex while a complement is
    // not. Holes are CIRCLES — one disc-complement entity each; the
    // hole loop below enforces it loudly.)
    const auto add_contour = [&](std::size_t ci) {
        const auto& ents = chains[ci].entities;
        const std::size_t n = ents.size();

        extrude_detail::ChainWalk walk;
        walk.n = n;
        walk.start.resize(n);
        walk.end.resize(n);
        walk.material_left = true;
        for (std::size_t i = 0; i < n; ++i) {
            extrude_detail::entity_endpoints(ents[i], walk.start[i],
                                             walk.end[i]);
            if (ents[i].kind == sdf::SketchEntity::Kind::Segment)
                walk.material_left =
                    walk.material_left && ents[i].segment.interior_left;
            else if (ents[i].kind == sdf::SketchEntity::Kind::Arc)
                walk.material_left =
                    walk.material_left && !ents[i].arc.hole;
        }
        // (A hole-side arc already refused above; a mix refused too.
        // material_left now reflects the uniform authored side.)

        const std::size_t v_base = body->vertices.size();
        const std::size_t e_base = body->edges.size();

        // Vertices: 2i = J_i at z_lo, 2i+1 = J_i at z_hi, where
        // J_i = entity i's end point.
        for (std::size_t i = 0; i < n; ++i) {
            const math::Vec2f& j = walk.end[i];
            Vertex vb{math::Vec3d{j.x, j.y, z_lo}};
            vb.set_id(vid(v_base + 2 * i));
            body->vertices.push_back(vb);
            Vertex vt{math::Vec3d{j.x, j.y, z_hi}};
            vt.set_id(vid(v_base + 2 * i + 1));
            body->vertices.push_back(vt);
        }

        // Edges: i = the bottom curve of entity i (authored
        // direction: from J(i-1) to J_i); n+i = the top curve;
        // 2n+i = the vertical at J_i. Pushed in ID ORDER (the
        // factories' rule: id == vector index — the co-edges and the
        // query index edges by id).
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t prev = (i + n - 1) % n;
            Edge eb{vid(v_base + 2 * prev), vid(v_base + 2 * i)};
            eb.set_id(eid(e_base + i));
            body->edges.push_back(eb);
        }
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t prev = (i + n - 1) % n;
            Edge et{vid(v_base + 2 * prev + 1), vid(v_base + 2 * i + 1)};
            et.set_id(eid(e_base + n + i));
            body->edges.push_back(et);
        }
        for (std::size_t i = 0; i < n; ++i) {
            Edge ev{vid(v_base + 2 * i), vid(v_base + 2 * i + 1)};
            ev.set_id(eid(e_base + 2 * n + i));
            body->edges.push_back(ev);
        }

        // The lateral faces. The material-on-left walk: entity i
        // forward (J(i-1) → J_i) when the chain is material-left,
        // reversed (J_i → J(i-1)) when material-right — the loop is
        // then [bottom, up-at-the-end, top-reversed,
        // down-at-the-start] and the p-curves carry the FACE'S own
        // region (the rectangle/window), material on the left.
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t prev = (i + n - 1) % n;
            const auto& e = ents[i];
            const bool fwd = walk.material_left;
            const std::size_t e_bot = e_base + i;
            const std::size_t e_top = e_base + n + i;
            const std::size_t e_v_end = e_base + 2 * n + (fwd ? i : prev);
            const std::size_t e_v_start =
                e_base + 2 * n + (fwd ? prev : i);

            if (e.kind == sdf::SketchEntity::Kind::Segment) {
                // The plane through the segment, the OUTWARD normal
                // the sketch kernel's own Halfplane2D derives (one
                // opinion — no second normal formula).
                const sdf::Halfplane2D hp(e.segment.a, e.segment.b,
                                          e.segment.interior_left);
                const math::Vec2f o2 = fwd ? walk.start[i] : walk.end[i];
                auto surf = std::make_shared<PlaneSurface>(
                    math::Vec3d{o2.x, o2.y, z_lo},
                    math::Vec3d{hp.outward_normal().x,
                                hp.outward_normal().y, 0.0});
                Face face(surf);
                Loop loop;
                const auto add = [&](std::size_t edge, bool reversed) {
                    loop.add_coedge(detail::line_coedge(
                        eid(edge), reversed, *surf, body->vertices,
                        body->edges));
                };
                add(e_bot, !fwd);
                add(e_v_end, false);
                add(e_top, fwd);
                add(e_v_start, true);
                face.add_loop(std::move(loop));
                face.set_id(static_cast<FaceId>(
                    shell.faces().size()));
                // The NAME in the same pass: entity i's lateral —
                // kExtFaceRoles' lat(i), the group prefix on
                // multi-chain bodies (the outer chain's own group).
                stamp_lateral(face, i);
                shell.add_face(std::move(face));
            } else {
                // The cylindrical patch: the window [α, α+Δ] × [0,h]
                // in the surface's (u,v), u running the walk's
                // azimuth direction (the frame rule's sign).
                const auto& arc = e.arc;
                // THE SKETCH FRONTIER (ADR-0035 D4): the sketch
                // payload's angles enter the BREP domain as the
                // EXACT promotions of the authored floats — one
                // direction, lossless, right here; the span is
                // stored in the domain's own double.
                const double u0 = fwd ? static_cast<double>(arc.start_angle)
                                      : static_cast<double>(arc.start_angle +
                                                            arc.sweep);
                const double u1 = fwd ? static_cast<double>(arc.start_angle +
                                                           arc.sweep)
                                      : static_cast<double>(arc.start_angle);
                // The sketch payload promoted to the exact world's
                // parameters (lossless, ADR-0034).
                auto surf = std::make_shared<CylinderZSurface>(
                    math::Vec2d{arc.center.x, arc.center.y},
                    arc.radius, z_lo, fwd ? +1 : -1);
                Face face(surf);
                Loop loop;
                // The window payloads: the world's double span,
                // stored as the domain's own double (morso 3: the
                // quantizer retired with the float payload).
                const double vh = h;
                const auto wall_line = [&](math::Vec2d a, math::Vec2d b) {
                    return pcurve_segment(a, b,
                                          /*interior_left=*/true);
                };
                CoEdge bot{eid(e_bot), !fwd};
                bot.set_p_curve(wall_line({u0, 0.0}, {u1, 0.0}));
                CoEdge up{eid(e_v_end), false};
                up.set_p_curve(wall_line({u1, 0.0}, {u1, vh}));
                CoEdge top{eid(e_top), fwd};
                top.set_p_curve(wall_line({u1, vh}, {u0, vh}));
                CoEdge down{eid(e_v_start), true};
                down.set_p_curve(wall_line({u0, vh}, {u0, 0.0}));
                loop.add_coedge(bot);
                loop.add_coedge(up);
                loop.add_coedge(top);
                loop.add_coedge(down);
                face.add_loop(std::move(loop));
                face.set_id(static_cast<FaceId>(
                    shell.faces().size()));
                // The NAME in the same pass: entity i's lateral —
                // the cylindrical patch's own lat(i), the group
                // prefix on multi-chain bodies.
                stamp_lateral(face, i);
                shell.add_face(std::move(face));
            }
        }

        // The cap loops: the chain itself, typed entity for entity,
        // the authored bits kept verbatim. The traversal keeps the
        // radial-edge XOR with the lateral faces' usage: the -Z cap
        // walks each entity OPPOSITE to its material-on-left walk,
        // the +Z cap walks it IN that direction — so a material-left
        // chain reads authored-cycle on the +Z cap and reversed
        // cycle on the -Z cap, and a material-right chain the other
        // way around. The cycle ORDER follows the traversal (the
        // loop must chain end-to-start either way).
        {
            std::vector<CoEdge> lo_loop, hi_loop;
            for (std::size_t k = 0; k < n; ++k) {
                const std::size_t i =
                    walk.material_left ? n - 1 - k : k;
                lo_loop.push_back(extrude_detail::cap_coedge(
                    cap_lo, ents[i], eid(e_base + i),
                    /*walk_forward=*/!walk.material_left));
            }
            for (std::size_t k = 0; k < n; ++k) {
                const std::size_t i =
                    walk.material_left ? k : n - 1 - k;
                hi_loop.push_back(extrude_detail::cap_coedge(
                    cap_hi, ents[i], eid(e_base + n + i),
                    /*walk_forward=*/walk.material_left));
            }
            cap_lo_loops.push_back(std::move(lo_loop));
            cap_hi_loops.push_back(std::move(hi_loop));
        }
    };

    // ---- one solid circle: the seam-wired pattern VERBATIM ------
    const auto add_solid_circle = [&](std::size_t ci) {
        const auto& circ = chains[ci].entities[0].circle;
        // THE SKETCH FRONTIER (ADR-0035 D4): the sketch circle's own
        // numbers enter the BREP parametric domain here, promoted
        // ONCE — float→double is exact by construction, one
        // direction, no bridge back. Everything downstream reads
        // these doubles.
        const math::Vec2d c{static_cast<double>(circ.center.x),
                            static_cast<double>(circ.center.y)};
        const double r = static_cast<double>(circ.radius);
        const std::size_t v_base = body->vertices.size();
        const std::size_t e_base = body->edges.size();

        const math::Vec3d seam_lo{c.x + r, c.y, z_lo};
        const math::Vec3d seam_hi{c.x + r, c.y, z_hi};
        const math::Vec3d anti_lo{c.x - r, c.y, z_lo};
        const math::Vec3d anti_hi{c.x - r, c.y, z_hi};
        Vertex v0{seam_lo};
        v0.set_id(vid(v_base));
        Vertex v1{anti_lo};
        v1.set_id(vid(v_base + 1));
        Vertex v2{seam_hi};
        v2.set_id(vid(v_base + 2));
        Vertex v3{anti_hi};
        v3.set_id(vid(v_base + 3));
        body->vertices.push_back(v0);
        body->vertices.push_back(v1);
        body->vertices.push_back(v2);
        body->vertices.push_back(v3);

        Edge rim_bot_a{vid(v_base), vid(v_base + 1)};      // θ 0→π
        rim_bot_a.set_id(eid(e_base));
        Edge rim_bot_b{vid(v_base + 1), vid(v_base)};      // θ π→2π
        rim_bot_b.set_id(eid(e_base + 1));
        Edge rim_top_a{vid(v_base + 2), vid(v_base + 3)};  // θ 0→π
        rim_top_a.set_id(eid(e_base + 2));
        Edge rim_top_b{vid(v_base + 3), vid(v_base + 2)};  // θ π→2π
        rim_top_b.set_id(eid(e_base + 3));
        Edge seam{vid(v_base), vid(v_base + 2)};
        seam.set_id(eid(e_base + 4));
        // The SEAM's NAME in the same pass (ADR-0036 D1 + morse-3:
        // the ADR-0023 convention's own e_seam, kExtEdgeRoles' bare
        // 'seam', born of the chain's entity 0 — the tavola's FROM
        // carries the genealogy, the wire's pair is the name's ONE
        // materialized form). The rim halves are outside v1's
        // vocabulary: honestly unnamed.
        stamp_seam(seam);
        body->edges.push_back(rim_bot_a);
        body->edges.push_back(rim_bot_b);
        body->edges.push_back(rim_top_a);
        body->edges.push_back(rim_top_b);
        body->edges.push_back(seam);

        // The wall: the slit window [0,2π]×[0,h] walked CCW from the
        // seam's RIGHT copy — six Line2, chained with no periodic
        // jump, material on the left (the strip).
        {
            auto surf = std::make_shared<CylinderZSurface>(
                math::Vec2d{c.x, c.y}, r, z_lo, +1);
            Face face(surf);
            Loop loop;
            const double vh = h;
            const auto wall_line = [](math::Vec2d a, math::Vec2d b) {
                return pcurve_segment(a, b,
                                      /*interior_left=*/true);
            };
            CoEdge s_fwd{eid(e_base + 4), false};  // up the right
            s_fwd.set_p_curve(
                wall_line({extrude_detail::kTwoPi, 0.0},
                          {extrude_detail::kTwoPi, vh}));
            CoEdge top_b_rev{eid(e_base + 3), true};
            top_b_rev.set_p_curve(wall_line(
                {extrude_detail::kTwoPi, vh}, {extrude_detail::kPi, vh}));
            CoEdge top_a_rev{eid(e_base + 2), true};
            top_a_rev.set_p_curve(wall_line(
                {extrude_detail::kPi, vh}, {0.0, vh}));
            CoEdge s_rev{eid(e_base + 4), true};  // down the left
            s_rev.set_p_curve(wall_line({0.0, vh}, {0.0, 0.0}));
            CoEdge bot_a_fwd{eid(e_base), false};
            bot_a_fwd.set_p_curve(wall_line(
                {0.0, 0.0}, {extrude_detail::kPi, 0.0}));
            CoEdge bot_b_fwd{eid(e_base + 1), false};
            bot_b_fwd.set_p_curve(wall_line(
                {extrude_detail::kPi, 0.0},
                {extrude_detail::kTwoPi, 0.0}));
            loop.add_coedge(s_fwd);
            loop.add_coedge(top_b_rev);
            loop.add_coedge(top_a_rev);
            loop.add_coedge(s_rev);
            loop.add_coedge(bot_a_fwd);
            loop.add_coedge(bot_b_fwd);
            face.add_loop(std::move(loop));
            face.set_id(static_cast<FaceId>(shell.faces().size()));
            // The NAME in the same pass: the circle case's SINGLE
            // wall — D1's own "the circle special case's single
            // lat(0) wall", born of the chain's one entity.
            stamp_lateral(face, 0);
            shell.add_face(std::move(face));
        }

        // The caps' loops: the disc region, two π-sweep Arc2 each,
        // authored in the caps' own domains by projection (the walk
        // keeps the XOR: the wall walks the bottom rim east and the
        // top rim west; the -Z cap walks the bottom rim in the
        // authored direction's opposite, the +Z cap the top's).
        const auto dom_lo = [&](const math::Vec2d& p) {
            const math::Vec3d d =
                math::Vec3d{p.x, p.y, 0.0} - cap_lo.origin();
            return math::Vec2d{d.dot(cap_lo.u_axis()),
                               d.dot(cap_lo.v_axis())};
        };
        const auto dom_hi = [&](const math::Vec2d& p) {
            const math::Vec3d d =
                math::Vec3d{p.x, p.y, 0.0} - cap_hi.origin();
            return math::Vec2d{d.dot(cap_hi.u_axis()),
                               d.dot(cap_hi.v_axis())};
        };
        const auto arc_at = [](const math::Vec2d& center, double radius,
                               const math::Vec2d& from) {
            const math::Vec2d d = from - center;
            return pcurve_arc(center, radius, /*hole=*/false,
                              std::atan2(d.y, d.x),
                              extrude_detail::kPi);
        };
        {
            // -Z cap: [rim_bot_a rev (antipode→seam), rim_bot_b rev
            // (seam→antipode)] — 3D CW, CCW in the mirror domain.
            std::vector<CoEdge> loop;
            CoEdge a{eid(e_base), true};
            a.set_p_curve(arc_at(dom_lo(c), r, dom_lo({c.x - r, c.y})));
            CoEdge b{eid(e_base + 1), true};
            b.set_p_curve(arc_at(dom_lo(c), r, dom_lo({c.x + r, c.y})));
            loop.push_back(a);
            loop.push_back(b);
            cap_lo_loops.push_back(std::move(loop));
        }
        {
            // +Z cap: [rim_top_b fwd (antipode→seam), rim_top_a fwd
            // (seam→antipode)] — 3D CCW, CCW in the rotation domain.
            std::vector<CoEdge> loop;
            CoEdge b{eid(e_base + 3), false};
            b.set_p_curve(arc_at(dom_hi(c), r, dom_hi({c.x - r, c.y})));
            CoEdge a{eid(e_base + 2), false};
            a.set_p_curve(arc_at(dom_hi(c), r, dom_hi({c.x + r, c.y})));
            loop.push_back(b);
            loop.push_back(a);
            cap_hi_loops.push_back(std::move(loop));
        }
    };

    // ---- one hole circle: the seam-wired wall on the flipped u --
    // (The ONLY hole class the kernel can express: a single circle
    // chain with hole = true — the disc complement is one entity's
    // region. The wall is seam-wired on the REVERSED u; each cap
    // carries the rim as ONE closed trim curve, the Circle2.)
    const auto add_hole_circle = [&](std::size_t ci) {
        const auto& circ = chains[ci].entities[0].circle;
        // THE SKETCH FRONTIER (ADR-0035 D4): as above — the sketch
        // circle's own numbers, promoted once, exactly, one way.
        const math::Vec2d c{static_cast<double>(circ.center.x),
                            static_cast<double>(circ.center.y)};
        const double r = static_cast<double>(circ.radius);
        const std::size_t v_base = body->vertices.size();
        const std::size_t e_base = body->edges.size();

        const math::Vec3d seam_lo{c.x + r, c.y, z_lo};
        const math::Vec3d seam_hi{c.x + r, c.y, z_hi};
        Vertex vb{seam_lo};
        vb.set_id(vid(v_base));
        Vertex vt{seam_hi};
        vt.set_id(vid(v_base + 1));
        body->vertices.push_back(vb);
        body->vertices.push_back(vt);

        Edge rim_bot{vid(v_base), vid(v_base)};       // closed curve
        rim_bot.set_id(eid(e_base));
        Edge rim_top{vid(v_base + 1), vid(v_base + 1)};
        rim_top.set_id(eid(e_base + 1));
        Edge seam{vid(v_base), vid(v_base + 1)};
        seam.set_id(eid(e_base + 2));
        // The SEAM's NAME in the same pass: the hole's own seam —
        // 'c<group>:seam' on multi-chain bodies (this chain's
        // group, the counter's current value), the closed rims
        // honestly unnamed (outside v1's vocabulary).
        stamp_seam(seam);
        body->edges.push_back(rim_bot);
        body->edges.push_back(rim_top);
        body->edges.push_back(seam);

        // The wall: the window [-2π,0]×[0,h] on the REVERSED u (the
        // hole's outward normal is -radial — the frame rule), walked
        // CCW: up the right (u=0), west, down the left (u=-2π),
        // east. Four Line2; the seam's pair lives in this one loop.
        {
            auto surf = std::make_shared<CylinderZSurface>(
                math::Vec2d{c.x, c.y}, r, z_lo, -1);
            Face face(surf);
            Loop loop;
            const double vh = h;
            const auto wall_line = [](math::Vec2d a, math::Vec2d b) {
                return pcurve_segment(a, b,
                                      /*interior_left=*/true);
            };
            CoEdge s_fwd{eid(e_base + 2), false};
            s_fwd.set_p_curve(wall_line({0.0, 0.0}, {0.0, vh}));
            CoEdge top{eid(e_base + 1), false};
            top.set_p_curve(wall_line(
                {0.0, vh}, {-extrude_detail::kTwoPi, vh}));
            CoEdge s_rev{eid(e_base + 2), true};
            s_rev.set_p_curve(wall_line(
                {-extrude_detail::kTwoPi, vh}, {-extrude_detail::kTwoPi, 0.0}));
            CoEdge bot{eid(e_base), false};
            bot.set_p_curve(wall_line(
                {-extrude_detail::kTwoPi, 0.0}, {0.0, 0.0}));
            loop.add_coedge(s_fwd);
            loop.add_coedge(top);
            loop.add_coedge(s_rev);
            loop.add_coedge(bot);
            face.add_loop(std::move(loop));
            face.set_id(static_cast<FaceId>(shell.faces().size()));
            // The NAME in the same pass: the hole's wall — the
            // disc-complement entity's own lat(0), the group prefix
            // riding the chain's own group.
            stamp_lateral(face, 0);
            shell.add_face(std::move(face));
        }

        // The caps' INNER loops: ONE co-edge on the closed rim, the
        // Circle2 with the authored hole bit (the annulus is the
        // entity's region — the first Circle2 writer). The co-edge
        // is REVERSED (the wall used the rim forward): the XOR.
        // The center crosses the projection in double (the cap's
        // own frame); the radius is the sketch's own float, promoted
        // EXACTLY at the frontier (ADR-0035 D4).
        {
            const math::Vec3d d =
                math::Vec3d{c.x, c.y, 0.0} - cap_lo.origin();
            const math::Vec2d center{d.dot(cap_lo.u_axis()),
                                     d.dot(cap_lo.v_axis())};
            CoEdge ce{eid(e_base), true};
            ce.set_p_curve(pcurve_circle(center, r,
                                         /*hole=*/true));
            cap_lo_loops.push_back(std::vector<CoEdge>{ce});
        }
        {
            const math::Vec3d d =
                math::Vec3d{c.x, c.y, 0.0} - cap_hi.origin();
            const math::Vec2d center{d.dot(cap_hi.u_axis()),
                                     d.dot(cap_hi.v_axis())};
            CoEdge ce{eid(e_base + 1), true};
            ce.set_p_curve(pcurve_circle(center, r,
                                         /*hole=*/true));
            cap_hi_loops.push_back(std::vector<CoEdge>{ce});
        }
    };

    // ---- walk the chains: the outer first, then the holes -------
    const auto is_circle_chain = [](const sdf::SketchChain& c) {
        return c.entities.size() == 1 &&
               c.entities[0].kind == sdf::SketchEntity::Kind::Circle;
    };
    {
        const auto& oc = chains[outer];
        if (is_circle_chain(oc))
            add_solid_circle(outer);
        else
            add_contour(outer);
        // The outer chain was group 0; the holes (in sketch order)
        // take the next groups — the numbering table's own
        // multi-chain layout, read at call time by the stamps.
        ++chain_group;
    }
    for (std::size_t c = 0; c < chains.size(); ++c) {
        if (c == outer) continue;
        if (!is_circle_chain(chains[c]))
            throw std::invalid_argument(
                "make_prism: a multi-entity hole chain is outside the "
                "kernel's expressible class — chain " +
                std::to_string(c) +
                " bounds a hole with segments/arcs, but the 2D "
                "profile composes regions by INTERSECTION (max of "
                "fields): an intersection of half-planes is convex "
                "and cannot be a hole's complement. Holes are circles "
                "— one disc-complement entity each (the sketch "
                "canvas's own authoring class). Remedy: model the "
                "hole as a hole-side circle chain, or as part of the "
                "outer contour's shape.");
        add_hole_circle(c);
        ++chain_group;
    }

    // ---- the caps: two plane faces, the loops in walk order ------
    // The FIRST loop of each cap is the outer one (the outer chain
    // was walked first); every subsequent loop is a hole — the
    // positional H term, ADR-0021: "subsequent loops are holes".
    // The caps' NAMES (the region's own two lids — feature-owned,
    // no group in any body): kExtFaceRoles' cap(lo)/cap(hi), the
    // role index the level's own table row.
    const auto add_cap_face = [&](const PlaneSurface& surf_proto,
                                  std::vector<std::vector<CoEdge>>& loops,
                                  std::uint32_t role_index) {
        Face face(std::make_shared<PlaneSurface>(surf_proto));
        for (std::size_t li = 0; li < loops.size(); ++li) {
            Loop l;
            l.set_outer(li == 0);
            for (const auto& ce : loops[li]) l.add_coedge(ce);
            face.add_loop(std::move(l));
        }
        face.set_id(static_cast<FaceId>(shell.faces().size()));
        const geometry::SubshapeRef ref =
            geometry::SubshapeRef{}.push(role_index);
        face.set_shape_id({feature, ref, {}});
        face.set_name(naming::render_name(
            feature, ref, naming::kExtNaming.faces()));
        shell.add_face(std::move(face));
    };
    add_cap_face(cap_lo, cap_lo_loops, naming::kExtCapLoRole);
    add_cap_face(cap_hi, cap_hi_loops, naming::kExtCapHiRole);

    link_radial_edges(*body);
    return body;
}

} // namespace CAD_0::brep
