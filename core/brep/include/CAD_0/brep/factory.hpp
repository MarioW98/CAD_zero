// core/brep/include/CAD_0/brep/factory.hpp
//
// Factory functions for creating B-Rep primitive bodies.
//
// These functions build a complete BRepBody with topology (vertices,
// edges, loops, faces) and analytic surface geometry, then call
// `link_radial_edges(*body)` to populate the radial-edge list mandated
// by ADR-0012 and documented in ADR-0021 §4.
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
// Face numbering and outward normals:
//   0: bottom  (-Z)   loop 0→1→2→3→0    (edges 0,1,2,3 forward)
//   1: top     (+Z)   loop 4→5→6→7→4    (edges 4,5,6,7 forward)
//   2: front   (-Y)   loop 0→1→5→4→0    (edges 0,9,4 rev,8 rev)
//   3: back    (+Y)   loop 2→3→7→6→2    (edges 2,11,6 rev,10 rev)
//   4: left    (-X)   loop 3→0→4→7→3    (edges 3,8,7 rev,11 rev)
//   5: right   (+X)   loop 1→2→6→5→1    (edges 1,10,5 rev,9 rev)
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

    // Face 0: bottom (-Z), loop 0→1→2→3→0.
    add_plane_face({0, 0, -z}, {0, 0, -1}, 0,
        /*e0*/ 0, false,  // 0→1
        /*e1*/ 1, false,  // 1→2
        /*e2*/ 2, false,  // 2→3
        /*e3*/ 3, false); // 3→0

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

// Create a sphere B-Rep body (single face, no edges for MVP — just
// the surface geometry, tessellated later).
inline std::unique_ptr<BRepBody> make_sphere(float radius, math::Vec3f center = {}) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // A sphere has a single face with a spherical surface. The seam
    // wire that turns this into a proper closed shell is Phase D step
    // (b); for now, the Euler characteristic is documented as the
    // known MVP limitation in ADR-0021 §5.
    Face face(std::make_shared<SphereSurface>(radius, center));
    face.set_id(0);
    shell.add_face(std::move(face));

    link_radial_edges(*body);
    return body;
}

// Create a cylinder B-Rep body (single cylindrical face + 2 planar caps).
inline std::unique_ptr<BRepBody> make_cylinder(float radius, float height) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // Lateral face
    Face lateral(std::make_shared<CylinderSurface>(radius, height));
    lateral.set_id(0);
    shell.add_face(std::move(lateral));

    // Bottom cap
    Face bottom(std::make_shared<PlaneSurface>(
        math::Vec3f{0, -height * 0.5f, 0}, math::Vec3f{0, -1, 0}));
    bottom.set_id(1);
    shell.add_face(std::move(bottom));

    // Top cap
    Face top(std::make_shared<PlaneSurface>(
        math::Vec3f{0, height * 0.5f, 0}, math::Vec3f{0, 1, 0}));
    top.set_id(2);
    shell.add_face(std::move(top));

    link_radial_edges(*body);
    return body;
}

} // namespace CAD_0::brep
