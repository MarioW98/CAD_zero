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
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <memory>

namespace CAD_0::brep {

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
inline std::unique_ptr<BRepBody> make_box(math::Vec3f half_extents) {
    auto body = std::make_unique<BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);

    // 8 vertices
    const float x = half_extents.x, y = half_extents.y, z = half_extents.z;
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

    // Helper: append a planar face with a single 4-coedge outer loop.
    auto add_plane_face = [&shell](
        math::Vec3f origin, math::Vec3f normal, FaceId id,
        EdgeId e0, bool r0,
        EdgeId e1, bool r1,
        EdgeId e2, bool r2,
        EdgeId e3, bool r3
    ) {
        Face face(std::make_shared<PlaneSurface>(origin, normal));
        Loop loop;
        loop.add_coedge(CoEdge{e0, r0});
        loop.add_coedge(CoEdge{e1, r1});
        loop.add_coedge(CoEdge{e2, r2});
        loop.add_coedge(CoEdge{e3, r3});
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
inline std::unique_ptr<BRepBody> make_sphere(float radius, math::Vec3f center = {}) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // 2 vertices: north and south poles.
    body->vertices = {
        Vertex{center + math::Vec3f{0,  radius, 0}},  // v_north (id 0)
        Vertex{center + math::Vec3f{0, -radius, 0}},  // v_south (id 1)
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
    loop.add_coedge(CoEdge{0, false});  // north → south
    loop.add_coedge(CoEdge{0, true});   // south → north
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
// Edges:
//   0: e_top1  v_top1 → v_top2   (top rim arc, +Z side)
//   1: e_top2  v_top2 → v_top1   (top rim arc, -Z side)
//   2: e_bot1  v_bot1 → v_bot2   (bottom rim arc, +Z side)
//   3: e_bot2  v_bot2 → v_bot1   (bottom rim arc, -Z side)
//   4: e_seam  v_bot1 → v_top1   (vertical seam at angle 0)
//
// Faces:
//   0: lateral (CylinderSurface)  loop: [e_bot1 rev, e_bot2 rev, e_seam fwd, e_top2 rev, e_top1 rev, e_seam rev]
//   1: top cap (PlaneSurface, +Y)  loop: [e_top1 fwd, e_top2 fwd]
//   2: bot cap (PlaneSurface, -Y)  loop: [e_bot1 fwd, e_bot2 fwd]
//
inline std::unique_ptr<BRepBody> make_cylinder(float radius, float height) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    const float h = height * 0.5f;
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
        Edge{0, 1},  // e_top1: v_top1 → v_top2
        Edge{1, 0},  // e_top2: v_top2 → v_top1
        Edge{2, 3},  // e_bot1: v_bot1 → v_bot2
        Edge{3, 2},  // e_bot2: v_bot2 → v_bot1
        Edge{2, 0},  // e_seam: v_bot1 → v_top1
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    // Helper: append a planar face with a single outer loop.
    auto add_plane_face = [&shell](
        math::Vec3f origin, math::Vec3f normal, FaceId id,
        std::initializer_list<std::pair<EdgeId, bool>> coedges
    ) {
        Face face(std::make_shared<PlaneSurface>(origin, normal));
        Loop loop;
        for (const auto& [eid, rev] : coedges) {
            loop.add_coedge(CoEdge{eid, rev});
        }
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // Face 0: lateral (CylinderSurface).
    // Loop walks: v_bot1 →(bot1 rev: bot2→bot1? no — bot1 is v_bot1→v_bot2,
    //   reversed is v_bot2→v_bot1. But we start at v_bot1. So we need
    //   bot2 rev first? Let me re-derive carefully.
    //
    //   Start at v_bot1 (id 2). Walk the lateral boundary:
    //   - Bottom rim reversed (because the lateral face's bottom edge
    //     goes opposite to the bottom cap's loop):
    //       e_bot1 reversed: v_bot2 → v_bot1  (no, that arrives at v_bot1)
    //       e_bot2 reversed: v_bot1 → v_bot2  (departs v_bot1)
    //   Hmm, let me just walk and verify closure at the end.
    //
    //   Lateral loop (6 co-edges):
    //     1. e_bot1 reversed:  v_bot2 → v_bot1  (arrive v_bot1)
    //     2. e_bot2 reversed:  v_bot1 → v_bot2  (depart v_bot1, arrive v_bot2)
    //   Wait — that's not right. Let me re-derive from scratch.
    //
    //   The lateral face boundary, walking CCW from outside (radially
    //   outward), starting at v_bot1:
    //     - Go UP along the seam: v_bot1 → v_top1   (e_seam forward)
    //     - Go along the top rim (reversed relative to top cap):
    //       v_top1 → v_top2  (e_top2 reversed, because e_top2 is v_top2→v_top1)
    //       v_top2 → v_top1  (e_top1 reversed, because e_top1 is v_top1→v_top2)
    //     - Go DOWN along the seam: v_top1 → v_bot1  (e_seam reversed)
    //     - Go along the bottom rim (reversed relative to bottom cap):
    //       v_bot1 → v_bot2  (e_bot2 reversed, because e_bot2 is v_bot2→v_bot1)
    //       v_bot2 → v_bot1  (e_bot1 reversed, because e_bot1 is v_bot1→v_bot2)
    //   Back at v_bot1. ✓ 6 co-edges.
    {
        Face face(std::make_shared<CylinderSurface>(radius, height));
        Loop loop;
        loop.add_coedge(CoEdge{4, false});  // e_seam fwd: v_bot1 → v_top1
        loop.add_coedge(CoEdge{1, true});  // e_top2 rev: v_top1 → v_top2
        loop.add_coedge(CoEdge{0, true});  // e_top1 rev: v_top2 → v_top1
        loop.add_coedge(CoEdge{4, true});  // e_seam rev: v_top1 → v_bot1
        loop.add_coedge(CoEdge{3, true});  // e_bot2 rev: v_bot1 → v_bot2
        loop.add_coedge(CoEdge{2, true});  // e_bot1 rev: v_bot2 → v_bot1
        face.add_loop(std::move(loop));
        face.set_id(0);
        shell.add_face(std::move(face));
    }

    // Face 1: top cap (PlaneSurface, +Y normal).
    // Loop: [e_top1 forward (v_top1→v_top2), e_top2 forward (v_top2→v_top1)]
    add_plane_face({0, h, 0}, {0, 1, 0}, 1,
        {{0, false}, {1, false}});

    // Face 2: bottom cap (PlaneSurface, -Y normal).
    // Loop: [e_bot1 forward (v_bot1→v_bot2), e_bot2 forward (v_bot2→v_bot1)]
    add_plane_face({0, -h, 0}, {0, -1, 0}, 2,
        {{2, false}, {3, false}});

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
        Vertex{{ 1.0f,  1.0f,  1.0f}},
        Vertex{{-1.0f, -1.0f,  1.0f}},
        Vertex{{-1.0f,  1.0f, -1.0f}},
        Vertex{{ 1.0f, -1.0f, -1.0f}},
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
    // with body_centroid = origin for this tetrahedron).
    auto add_tri_face = [&shell](
        FaceId id,
        EdgeId e0, bool r0,
        EdgeId e1, bool r1,
        EdgeId e2, bool r2,
        math::Vec3f origin, math::Vec3f normal
    ) {
        Face face(std::make_shared<PlaneSurface>(origin, normal));
        Loop loop;
        loop.add_coedge(CoEdge{e0, r0});
        loop.add_coedge(CoEdge{e1, r1});
        loop.add_coedge(CoEdge{e2, r2});
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
        const math::Vec3f fc{-1.0f/3.0f, -1.0f/3.0f, -1.0f/3.0f};
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
        const math::Vec3f fc{1.0f/3.0f, 1.0f/3.0f, -1.0f/3.0f};
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
        const math::Vec3f fc{1.0f/3.0f, -1.0f/3.0f, 1.0f/3.0f};
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
        const math::Vec3f fc{-1.0f/3.0f, 1.0f/3.0f, 1.0f/3.0f};
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
