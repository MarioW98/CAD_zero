// core/brep/include/CAD_0/brep/factory.hpp
//
// Factory functions for creating B-Rep primitive bodies.
//
// These functions build a complete BRepBody with topology (vertices,
// edges, loops, faces) and analytic surface geometry, then call
// `link_radial_edges(*body)` to populate the radial-edge list mandated
// by ADR-0012 and documented in ADR-0021 §4.
//
// Layer-2 contract (ADR-0032 D2.4 + D8, G1 unit 1): factory bodies
// are EXACT mathematics — every entity's provenance is exact with
// linear tolerance 0, the ProvenanceInfo default these constructors
// write (an ideal constructed point needs no declaration; a SAMPLING
// path must declare its bound — the bridge does, and per-path tests
// pin both sides of that asymmetry).
//
// THE TRIMMING CONTRACT (ADR-0032 D1.1/D2, G1 unit 2): every
// co-edge of every factory body carries its typed p-curve in the
// OWNING face's surface domain — the face is (surface, region of
// the domain), and the loop of p-curves IS the region's boundary
// (D2.3). Authoring discipline, per the conventions pinned in
// pcurve.hpp:
//
//   * DIRECTION — the p-curve runs in the co-edge's traversal
//     direction (its start/end are the projected start/end
//     vertices, or the seam structure where the surface is
//     periodic);
//   * REGION SIDE — outer loops are authored CCW in the domain,
//     material on the LEFT (interior_left = true / hole = false on
//     every factory p-curve); inner loops — none in the G1
//     primitive class — would be authored CW, material still on
//     the left;
//   * the seam-wired periodic faces (sphere, cylinder wall) slit
//     their window open: the seam's forward copy walks the window's
//     RIGHT edge (u = 2π), the reversed copy the LEFT edge (u = 0),
//     and the loop's p-curves chain endpoint-to-endpoint in the
//     domain (across the poles' degenerate edges, modulo the
//     period — the sphere; contiguously — the cylinder wall).
//
// The plane-face p-curves are authored by PROJECTION (the co-edge's
// 3D endpoints through PlaneSurface's (u,v) frame — one inverse, no
// hand-copied corners); the sphere's and cylinder's seam/rim
// p-curves are authored as the domain constants they are (the slit
// window's edges), pinned bit-for-bit by the unit-2 goldens and
// cross-checked against the surfaces' evaluate() by the unit-2
// conventions tests.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <memory>

namespace CAD_0::brep {

namespace detail {

// The periodic domain's constants, shared by the seam authoring
// here and pinned bit-for-bit by the unit-2 goldens (the sketch
// kernel's own kTwoPi carries the same value — mathematical
// constants, not mirrored layouts). These stay FLOAT: they feed the
// p-curve payloads (the sketch kernel's domain, ADR-0032 D2.1; the
// H1 boundary converts world double -> domain float at the
// authoring site, one direction, declared here).
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kTwoPi = 6.28318530717958647692f;

// The domain coordinates of a 3D point in a plane's (u,v) frame —
// the inverse of PlaneSurface::evaluate (a frame the surface now
// exposes for exactly this: one projection, no second frame copy).
// THE BOUNDARY CONVERSION (ADR-0034): the projection computes in
// double (the exact world), the payload converts to float here —
// the p-curve vocabulary's own domain, one direction, the only
// place it happens.
inline math::Vec2f plane_domain(const PlaneSurface& s, math::Vec3d p) {
    const math::Vec3d d = p - s.origin();
    return {static_cast<float>(d.dot(s.u_axis())),
            static_cast<float>(d.dot(s.v_axis()))};
}

// A co-edge with its Line2 p-curve authored by projection through
// the owning face's plane frame: the p-curve's start/end ARE the
// co-edge's start/end vertices' domain images (the DIRECTION
// convention, by construction). interior_left = true is the
// factories' authoring discipline (outer loops CCW, material on
// the left) — a discipline the unit-2 conventions tests verify
// against the regions, so a reordered loop fails loudly there,
// never silently here.
inline CoEdge line_coedge(EdgeId e, bool reversed, const PlaneSurface& s,
                          const std::vector<Vertex>& vertices,
                          const std::vector<Edge>& edges) {
    CoEdge ce{e, reversed};
    ce.set_p_curve(sdf::sketch_segment(
        plane_domain(s, vertices[ce.start_vertex(edges)].position()),
        plane_domain(s, vertices[ce.end_vertex(edges)].position()),
        /*interior_left=*/true));
    return ce;
}

} // namespace detail

// Create a box B-Rep body centered at origin with given half-extents.
// 6 faces (planes), 12 edges, 8 vertices. Euler characteristic χ = 2.
//
// Vertex numbering (matches the legacy factory, kept stable for tests):
//
//     7-------6
//    /|      /|
//   3-------2 |
//   | 4-----|-5
//   |/      |/
//   0-------1
//
// Edge numbering:
//   bottom ring : 0:0→1  1:1→2  2:2→3  3:3→0
//   top ring    : 4:4→5  5:5→6  6:6→7  7:7→4
//   verticals   : 8:0→4  9:1→5 10:2→6 11:3→7
//
// Face numbering and outward normals (loops chosen so the cross product
// (v1-v0) × (v2-v0) of the first three loop vertices points in the
// outward direction — this gives every edge the opposite coedge
// orientation contract required by ADR-0022 §5):
//   0: bottom  (-Z)   loop 0→3→2→1→0    (edges 3,2,1,0 reversed)
//   1: top     (+Z)   loop 4→5→6→7→4    (edges 4,5,6,7 forward)
//   2: front   (-Y)   loop 0→1→5→4→0    (edges 0,9 forward; 4,8 reversed)
//   3: back    (+Y)   loop 2→3→7→6→2    (edges 2,11 forward; 6,10 reversed)
//   4: left    (-X)   loop 3→0→4→7→3    (edges 3,8 forward; 7,11 reversed)
//   5: right   (+X)   loop 1→2→6→5→1    (edges 1,10 forward; 5,9 reversed)
//
inline std::unique_ptr<BRepBody> make_box(math::Vec3d half_extents) {
    auto body = std::make_unique<BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);

    // 8 vertices
    const double x = half_extents.x, y = half_extents.y, z = half_extents.z;
    body->vertices = {
        Vertex{{-x, -y, -z}}, Vertex{{ x, -y, -z}},
        Vertex{{ x,  y, -z}}, Vertex{{-x,  y, -z}},
        Vertex{{-x, -y,  z}}, Vertex{{ x, -y,  z}},
        Vertex{{ x,  y,  z}}, Vertex{{-x,  y,  z}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<VertexId>(i));

    // 12 edges (see numbering in the header comment).
    body->edges = {
        Edge{0,1}, Edge{1,2}, Edge{2,3}, Edge{3,0},  // bottom ring
        Edge{4,5}, Edge{5,6}, Edge{6,7}, Edge{7,4},  // top ring
        Edge{0,4}, Edge{1,5}, Edge{2,6}, Edge{3,7},  // verticals
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    // Helper: append a planar face with a single 4-coedge outer loop,
    // every co-edge carrying its Line2 p-curve — the projected
    // rectangle of the face's region in the plane's (u,v) domain
    // (unit 2's trimming contract; the pins live in
    // test_g1_unit2_pcurves.cpp).
    auto add_plane_face = [&shell, &body](
        math::Vec3d origin, math::Vec3d normal, FaceId id,
        EdgeId e0, bool r0,
        EdgeId e1, bool r1,
        EdgeId e2, bool r2,
        EdgeId e3, bool r3
    ) {
        auto surf = std::make_shared<PlaneSurface>(origin, normal);
        Face face(surf);
        Loop loop;
        loop.add_coedge(detail::line_coedge(e0, r0, *surf,
                                            body->vertices, body->edges));
        loop.add_coedge(detail::line_coedge(e1, r1, *surf,
                                            body->vertices, body->edges));
        loop.add_coedge(detail::line_coedge(e2, r2, *surf,
                                            body->vertices, body->edges));
        loop.add_coedge(detail::line_coedge(e3, r3, *surf,
                                            body->vertices, body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // Face 0: bottom (-Z), loop 0→3→2→1→0.
    //   Cross-product check: (v3-v0) × (v2-v0) for v0=(-x,-y,-z),
    //   v3=(-x,+y,-z), v2=(+x,+y,-z) gives (0,0,-4) = -Z = outward.
    add_plane_face({0, 0, -z}, {0, 0, -1}, 0,
        /*e0*/ 3, true,   // 0→3  (edge 3: 3→0 reversed)
        /*e1*/ 2, true,   // 3→2  (edge 2: 2→3 reversed)
        /*e2*/ 1, true,   // 2→1  (edge 1: 1→2 reversed)
        /*e3*/ 0, true);  // 1→0  (edge 0: 0→1 reversed)

    // Face 1: top (+Z), loop 4→5→6→7→4.
    add_plane_face({0, 0, z}, {0, 0, 1}, 1,
        /*e0*/ 4, false,  // 4→5
        /*e1*/ 5, false,  // 5→6
        /*e2*/ 6, false,  // 6→7
        /*e3*/ 7, false); // 7→4

    // Face 2: front (-Y), loop 0→1→5→4→0.
    add_plane_face({0, -y, 0}, {0, -1, 0}, 2,
        /*e0*/ 0, false,  // 0→1  (edge 0: 0→1)
        /*e1*/ 9, false,  // 1→5  (edge 9: 1→5)
        /*e2*/ 4, true,   // 5→4  (edge 4: 4→5 reversed)
        /*e3*/ 8, true);  // 4→0  (edge 8: 0→4 reversed)

    // Face 3: back (+Y), loop 2→3→7→6→2.
    add_plane_face({0, y, 0}, {0, 1, 0}, 3,
        /*e0*/ 2, false,  // 2→3  (edge 2: 2→3)
        /*e1*/ 11, false, // 3→7  (edge 11: 3→7)
        /*e2*/ 6, true,   // 7→6  (edge 6: 6→7 reversed)
        /*e3*/ 10, true); // 6→2  (edge 10: 2→6 reversed)

    // Face 4: left (-X), loop 3→0→4→7→3.
    add_plane_face({-x, 0, 0}, {-1, 0, 0}, 4,
        /*e0*/ 3, false,  // 3→0  (edge 3: 3→0)
        /*e1*/ 8, false,  // 0→4  (edge 8: 0→4)
        /*e2*/ 7, true,   // 4→7  (edge 7: 7→4 reversed)
        /*e3*/ 11, true); // 7→3  (edge 11: 3→7 reversed)

    // Face 5: right (+X), loop 1→2→6→5→1.
    add_plane_face({x, 0, 0}, {1, 0, 0}, 5,
        /*e0*/ 1, false,  // 1→2  (edge 1: 1→2)
        /*e1*/ 10, false, // 2→6  (edge 10: 2→6)
        /*e2*/ 5, true,   // 6→5  (edge 5: 5→6 reversed)
        /*e3*/ 9, true);  // 5→1  (edge 9: 1→5 reversed)

    // ADR-0021 §4: populate the radial-edge list now that every loop is
    // finalized. After this call, the CoEdge* pointers in each
    // Edge::coedges_ are valid for the lifetime of `body`.
    link_radial_edges(*body);

    return body;
}

// Create a sphere B-Rep body with a seam wire — ADR-0023 §1.
//
// Topology: 2V / 1E / 1F, χ = 2. The single seam edge (a meridian from
// north pole to south pole) is used twice by the single spherical face:
// once forward (north→south) and once reversed (south→north). This is
// a valid closed manifold — the seam edge has 2 co-edges, both on the
// same face, with opposite orientations.
//
// Vertex layout:
//   v_north  = (0, +R, 0)   (north pole)
//   v_south  = (0, -R, 0)   (south pole)
//
// Edge:
//   e_seam: v_north → v_south   (the meridian at u=0)
//
// Face loop: [e_seam forward, e_seam reversed]
//
// The trimming (G1 unit 2): the face's region is the slit rectangle
// [0,2π]×[0,π] of the (u,v) domain (u = azimuth, v = polar from the
// north pole). The CCW boundary walk is bottom-east (the degenerate
// north-pole edge), right seam UP, top-west (the degenerate
// south-pole edge), left seam DOWN — the degenerate pole edges carry
// no co-edges, so the 3D loop is the seam pair, and the p-curves
// chain across the poles modulo the 2π period:
//   seam fwd (north→south): the RIGHT edge, Line2 (2π,0) → (2π,π);
//   seam rev (south→north): the LEFT edge, Line2 (0,π) → (0,0).
// Both carry material on the left (the rectangle's interior).
inline std::unique_ptr<BRepBody> make_sphere(double radius, math::Vec3d center = {}) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // 2 vertices: north and south poles.
    body->vertices = {
        Vertex{center + math::Vec3d{0,  radius, 0}},  // v_north (id 0)
        Vertex{center + math::Vec3d{0, -radius, 0}},  // v_south (id 1)
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<VertexId>(i));

    // 1 edge: the seam meridian from north to south.
    body->edges = { Edge{0, 1} };
    body->edges[0].set_id(0);

    // 1 face: the spherical surface. Its loop uses the seam edge twice
    // — once forward (north→south) and once reversed (south→north) —
    // forming a closed loop that bounds the entire spherical surface.
    Face face(std::make_shared<SphereSurface>(radius, center));
    Loop loop;
    {
        CoEdge seam_fwd{0, false};  // north → south, the RIGHT edge
        seam_fwd.set_p_curve(sdf::sketch_segment(
            math::Vec2f{detail::kTwoPi, 0.0f},
            math::Vec2f{detail::kTwoPi, detail::kPi},
            /*interior_left=*/true));
        CoEdge seam_rev{0, true};   // south → north, the LEFT edge
        seam_rev.set_p_curve(sdf::sketch_segment(
            math::Vec2f{0.0f, detail::kPi},
            math::Vec2f{0.0f, 0.0f},
            /*interior_left=*/true));
        loop.add_coedge(seam_fwd);
        loop.add_coedge(seam_rev);
    }
    face.add_loop(std::move(loop));
    face.set_id(0);
    shell.add_face(std::move(face));

    link_radial_edges(*body);
    return body;
}

// Create a cylinder B-Rep body with a seam wire — ADR-0023 §1.
//
// Topology: 4V / 5E / 3F, χ = 2. The lateral face's loop uses the seam
// edge twice (forward and reversed), same pattern as the sphere.
//
// Vertex layout (radius R, height H, axis along Y, centered at origin):
//   v_top1 = (+R, +H/2, 0)   v_top2 = (-R, +H/2, 0)
//   v_bot1 = (+R, -H/2, 0)   v_bot2 = (-R, -H/2, 0)
//
// Edges (the "sides" name which 3D half-arc each rim edge is — data
// the P-CURVES pin since unit 2, not stored on the Edge itself; the
// two top edges SWAPPED sides at unit 2's frame correction, the
// analytic.hpp handedness fix — u now runs the opposite azimuth):
//   0: e_top1  v_top1 → v_top2   (top rim arc, −Z side)
//   1: e_top2  v_top2 → v_top1   (top rim arc, +Z side)
//   2: e_bot1  v_bot1 → v_bot2   (bottom rim arc, +Z side)
//   3: e_bot2  v_bot2 → v_bot1   (bottom rim arc, −Z side)
//   4: e_seam  v_bot1 → v_top1   (vertical seam at u=0)
//
// Faces:
//   0: lateral (CylinderSurface)  loop: [e_bot1 rev, e_bot2 rev, e_seam fwd, e_top2 rev, e_top1 rev, e_seam rev]
//   1: top cap (PlaneSurface, +Y)  loop: [e_top1 fwd, e_top2 fwd]
//   2: bot cap (PlaneSurface, -Y)  loop: [e_bot1 fwd, e_bot2 fwd]
//
// The trimming (G1 unit 2), in the corrected (u,v) domain — u the
// azimuth of evaluate(u,v) = (R cos u, v, −R sin u), so u=0 is the
// seam meridian through v_top1/v_bot1 and u=π the antipode:
//
//   * the WALL's region is the window [0,2π]×[−H/2,+H/2]; its CCW
//     boundary walk is bottom-EAST, right-seam-UP, top-WEST,
//     left-seam-DOWN — exactly the loop below, in walk order from
//     the seam: right seam up (u=2π), top west (2π→π→0), left seam
//     down (u=0), bottom east (0→π→2π). The six Line2 p-curves
//     chain contiguously, no periodic jump (the rim arcs span the
//     full window); each carries material on the left (the strip).
//   * the CAPS' regions are discs in their own plane domains — the
//     rim's full circle walked CCW as two π-sweep Arc2 (the caps'
//     frames put v_top1/v_bot1 at domain angle −π/2 and the antipode
//     at +π/2; both caps read the same angles, see the golden pins).
inline std::unique_ptr<BRepBody> make_cylinder(double radius, double height) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    const double h = height * 0.5;
    // 4 vertices: 2 on the top rim, 2 on the bottom rim (seam + antipode).
    body->vertices = {
        Vertex{{ radius,  h, 0}},  // v_top1 (id 0)
        Vertex{{-radius,  h, 0}},  // v_top2 (id 1)
        Vertex{{ radius, -h, 0}},  // v_bot1 (id 2)
        Vertex{{-radius, -h, 0}},  // v_bot2 (id 3)
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<VertexId>(i));

    // 5 edges (see numbering in the header comment).
    body->edges = {
        Edge{0, 1},  // e_top1: v_top1 → v_top2   (−Z side)
        Edge{1, 0},  // e_top2: v_top2 → v_top1   (+Z side)
        Edge{2, 3},  // e_bot1: v_bot1 → v_bot2   (+Z side)
        Edge{3, 2},  // e_bot2: v_bot2 → v_bot1   (−Z side)
        Edge{2, 0},  // e_seam: v_bot1 → v_top1   (at u=0)
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    // Face 0: the wall — the slit rectangle's CCW walk in the (u,v)
    // window [0,2π]×[−h,h], starting up the RIGHT seam copy:
    //   (2π,−h) → (2π,h) → (π,h) → (0,h) → (0,−h) → (π,−h) → (2π,−h)
    // — six Line2, chained with no jump, material on the left.
    {
        Face face(std::make_shared<CylinderSurface>(radius, height));
        Loop loop;
        const auto wall_line = [](math::Vec2f a, math::Vec2f b) {
            return sdf::sketch_segment(a, b, /*interior_left=*/true);
        };
        // The window payloads: the exact world's double h quantized to
        // the p-curve domain's float, one direction (ADR-0034).
        const float vh = static_cast<float>(h);
        CoEdge seam_fwd{4, false};  // v_bot1 → v_top1: the RIGHT edge
        seam_fwd.set_p_curve(wall_line({detail::kTwoPi, -vh},
                                       {detail::kTwoPi, vh}));
        CoEdge top2_rev{1, true};   // v_top1 → v_top2: +Z half, west
        top2_rev.set_p_curve(wall_line({detail::kTwoPi, vh},
                                       {detail::kPi, vh}));
        CoEdge top1_rev{0, true};   // v_top2 → v_top1: −Z half, west
        top1_rev.set_p_curve(wall_line({detail::kPi, vh}, {0.0f, vh}));
        CoEdge seam_rev{4, true};   // v_top1 → v_bot1: the LEFT edge
        seam_rev.set_p_curve(wall_line({0.0f, vh}, {0.0f, -vh}));
        CoEdge bot2_rev{3, true};   // v_bot1 → v_bot2: −Z half, east
        bot2_rev.set_p_curve(wall_line({0.0f, -vh},
                                       {detail::kPi, -vh}));
        CoEdge bot1_rev{2, true};   // v_bot2 → v_bot1: +Z half, east
        bot1_rev.set_p_curve(wall_line({detail::kPi, -vh},
                                       {detail::kTwoPi, -vh}));
        loop.add_coedge(seam_fwd);
        loop.add_coedge(top2_rev);
        loop.add_coedge(top1_rev);
        loop.add_coedge(seam_rev);
        loop.add_coedge(bot2_rev);
        loop.add_coedge(bot1_rev);
        face.add_loop(std::move(loop));
        face.set_id(0);
        shell.add_face(std::move(face));
    }

    // The caps: planar disc regions, the rim circle walked CCW as two
    // π-sweep arcs in each cap's own (u,v) frame. The frames
    // ((−Z,−X) top, (Z,−X) bottom) put v_top1/v_bot1 (the seam-side
    // rim vertex, +X in 3D) at domain angle −π/2 and the antipode at
    // +π/2 — the same table for both caps, pinned by the goldens.
    const auto add_cap_face = [&shell, radius](math::Vec3d origin,
                                               math::Vec3d normal,
                                               FaceId id, EdgeId e_first,
                                               EdgeId e_second) {
        Face face(std::make_shared<PlaneSurface>(origin, normal));
        Loop loop;
        CoEdge first{e_first, false};
        first.set_p_curve(sdf::sketch_arc(math::Vec2f{0, 0}, static_cast<float>(radius),
                                          /*hole=*/false,
                                          -0.5f * detail::kPi,
                                          detail::kPi));
        CoEdge second{e_second, false};
        second.set_p_curve(sdf::sketch_arc(math::Vec2f{0, 0}, static_cast<float>(radius),
                                           /*hole=*/false,
                                           0.5f * detail::kPi,
                                           detail::kPi));
        loop.add_coedge(first);
        loop.add_coedge(second);
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // Face 1: top cap (PlaneSurface, +Y normal): [e_top1, e_top2].
    add_cap_face({0, h, 0}, {0, 1, 0}, 1, 0, 1);

    // Face 2: bottom cap (PlaneSurface, -Y normal): [e_bot1, e_bot2].
    add_cap_face({0, -h, 0}, {0, -1, 0}, 2, 2, 3);

    link_radial_edges(*body);
    return body;
}

// Create a regular tetrahedron B-Rep body — ADR-0022 §5.
//
// 4 vertices, 6 edges, 4 triangular faces, 1 shell. χ = 4 − 6 + 4 = 2.
// Every vertex has valence 3; every edge is shared by exactly 2 faces.
//
// Vertex layout (regular tetrahedron, edge length 2√2, centroid origin):
//
//   v0 = ( 1,  1,  1)
//   v1 = (-1, -1,  1)
//   v2 = (-1,  1, -1)
//   v3 = ( 1, -1, -1)
//
// Edge numbering (canonical: low-vertex-id first):
//   0: v0—v1    1: v0—v2    2: v0—v3
//   3: v1—v2    4: v1—v3    5: v2—v3
//
// Face numbering and outward normals (loops chosen so the cross
// product of the first two edge vectors points outward from the body
// centroid at the origin — gives every edge the opposite coedge
// orientation contract required by ADR-0022 §5):
//   0: opposite v0 (face v1—v2—v3), outward normal ≈ (-1,-1,-1)/√3
//   1: opposite v1 (face v0—v3—v2), outward normal ≈ (+1,+1,-1)/√3
//   2: opposite v2 (face v0—v1—v3), outward normal ≈ (+1,-1,+1)/√3
//   3: opposite v3 (face v0—v2—v1), outward normal ≈ (-1,+1,+1)/√3
//
inline std::unique_ptr<BRepBody> make_tetrahedron() {
    auto body = std::make_unique<BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);

    // 4 vertices
    body->vertices = {
        Vertex{{ 1.0,  1.0,  1.0}},
        Vertex{{-1.0, -1.0,  1.0}},
        Vertex{{-1.0,  1.0, -1.0}},
        Vertex{{ 1.0, -1.0, -1.0}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<VertexId>(i));

    // 6 edges (see numbering in the header comment).
    body->edges = {
        Edge{0,1}, Edge{0,2}, Edge{0,3},
        Edge{1,2}, Edge{1,3}, Edge{2,3},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    // Helper: append a triangular planar face with a single 3-coedge
    // outer loop. The plane is built from the face centroid (used as
    // the plane origin) and the outward normal (face_centroid - body_centroid,
    // with body_centroid = origin for this tetrahedron). Every co-edge
    // carries its Line2 p-curve by projection (unit 2's trimming
    // contract — the fourth factory joins the three primitives).
    auto add_tri_face = [&shell, &body](
        FaceId id,
        EdgeId e0, bool r0,
        EdgeId e1, bool r1,
        EdgeId e2, bool r2,
        math::Vec3d origin, math::Vec3d normal
    ) {
        auto surf = std::make_shared<PlaneSurface>(origin, normal);
        Face face(surf);
        Loop loop;
        loop.add_coedge(detail::line_coedge(e0, r0, *surf,
                                            body->vertices, body->edges));
        loop.add_coedge(detail::line_coedge(e1, r1, *surf,
                                            body->vertices, body->edges));
        loop.add_coedge(detail::line_coedge(e2, r2, *surf,
                                            body->vertices, body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // Face 0: opposite v0 → outward loop v1→v2→v3→v1.
    //   Cross-product check: (v2-v1) × (v3-v1) = (0,2,-2) × (2,0,-2)
    //   = (-4,-4,-4) = -X-Y-Z direction = away from v0. ✓
    //   walk: edge 3 (v1→v2) forward, edge 5 (v2→v3) forward,
    //         edge 4 (v1→v3) reversed → v3→v1.
    //   Face centroid: ((-1-1+1)/3, (-1+1-1)/3, (1-1-1)/3) = (-1/3, -1/3, -1/3).
    {
        const math::Vec3d fc{-1.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0};
        add_tri_face(0,
            /*e0*/ 3, false,  // v1→v2  (edge 3: v1→v2)
            /*e1*/ 5, false,  // v2→v3  (edge 5: v2→v3)
            /*e2*/ 4, true,   // v3→v1  (edge 4: v1→v3 reversed)
            fc, fc.normalized());
    }

    // Face 1: opposite v1 → outward loop v0→v3→v2→v0.
    //   Cross-product check: (v3-v0) × (v2-v0) = (0,-2,-2) × (-2,0,-2)
    //   = (4, 4, -4) = +X+Y-Z direction = away from v1. ✓
    //   walk: edge 2 (v0→v3) forward, edge 5 (v2→v3) reversed → v3→v2,
    //         edge 1 (v0→v2) reversed → v2→v0.
    //   Face centroid: ((1+1-1)/3, (1-1-1)/3, (1+1-1)/3) = (1/3, -1/3, 1/3).
    //   Wait — this face contains v0,v2,v3 (not v0,v3,v2 in that order).
    //   Recompute centroid: (1 + (-1) + 1)/3 = 1/3 for x;
    //                       (1 + 1 + (-1))/3 = 1/3 for y;
    //                       (1 + (-1) + (-1))/3 = -1/3 for z.
    //   Centroid: (1/3, 1/3, -1/3). Outward normal: same direction.
    {
        const math::Vec3d fc{1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0};
        add_tri_face(1,
            /*e0*/ 2, false,  // v0→v3  (edge 2: v0→v3)
            /*e1*/ 5, true,   // v3→v2  (edge 5: v2→v3 reversed)
            /*e2*/ 1, true,   // v2→v0  (edge 1: v0→v2 reversed)
            fc, fc.normalized());
    }

    // Face 2: opposite v2 → outward loop v0→v1→v3→v0.
    //   Cross-product check: (v1-v0) × (v3-v0) = (-2,-2,0) × (0,-2,-2)
    //   = (4, -4, 4) = +X-Y+Z direction = away from v2. ✓
    //   walk: edge 0 (v0→v1) forward, edge 4 (v1→v3) forward,
    //         edge 2 (v0→v3) reversed → v3→v0.
    //   Face centroid: (1 + (-1) + 1)/3 = 1/3 for x;
    //                  (1 + (-1) + (-1))/3 = -1/3 for y;
    //                  (1 + 1 + (-1))/3 = 1/3 for z.
    //   Centroid: (1/3, -1/3, 1/3).
    {
        const math::Vec3d fc{1.0 / 3.0, -1.0 / 3.0, 1.0 / 3.0};
        add_tri_face(2,
            /*e0*/ 0, false,  // v0→v1  (edge 0: v0→v1)
            /*e1*/ 4, false,  // v1→v3  (edge 4: v1→v3)
            /*e2*/ 2, true,   // v3→v0  (edge 2: v0→v3 reversed)
            fc, fc.normalized());
    }

    // Face 3: opposite v3 → outward loop v0→v2→v1→v0.
    //   Cross-product check: (v2-v0) × (v1-v0) = (-2,0,-2) × (-2,-2,0)
    //   = (-4, 4, 4) = -X+Y+Z direction = away from v3. ✓
    //   walk: edge 1 (v0→v2) forward, edge 3 (v1→v2) reversed → v2→v1,
    //         edge 0 (v0→v1) reversed → v1→v0.
    //   Face centroid: (1 + (-1) + (-1))/3 = -1/3 for x;
    //                  (1 + 1 + (-1))/3 = 1/3 for y;
    //                  (1 + (-1) + 1)/3 = 1/3 for z.
    //   Centroid: (-1/3, 1/3, 1/3).
    {
        const math::Vec3d fc{-1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
        add_tri_face(3,
            /*e0*/ 1, false,  // v0→v2  (edge 1: v0→v2)
            /*e1*/ 3, true,   // v2→v1  (edge 3: v1→v2 reversed)
            /*e2*/ 0, true,   // v1→v0  (edge 0: v0→v1 reversed)
            fc, fc.normalized());
    }

    link_radial_edges(*body);
    return body;
}

} // namespace CAD_0::brep
