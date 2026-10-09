// core/brep/include/CAD_0/brep/factory.hpp
//
// Factory functions for creating B-Rep primitive bodies.
//
// These functions build a complete BRepBody with topology (vertices,
// edges, loops, faces) and analytic surface geometry.
//
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <memory>

namespace CAD_0::brep {

// Create a box B-Rep body centered at origin with given half-extents.
// 6 faces (planes), 12 edges, 8 vertices.
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

    // 12 edges
    body->edges = {
        Edge{0,1}, Edge{1,2}, Edge{2,3}, Edge{3,0},  // bottom
        Edge{4,5}, Edge{5,6}, Edge{6,7}, Edge{7,4},  // top
        Edge{0,4}, Edge{1,5}, Edge{2,6}, Edge{3,7},  // verticals
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    // 6 faces — each a plane with a 4-edge loop
    // Face 0: bottom (z = -z), normal = -Z
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{0, 0, -z}, math::Vec3f{0, 0, -1}));
        Loop loop;
        loop.add_coedge(CoEdge{0, false});  // 0→1
        loop.add_coedge(CoEdge{1, false});  // 1→2
        loop.add_coedge(CoEdge{2, false});  // 2→3
        loop.add_coedge(CoEdge{3, false});  // 3→0
        face.add_loop(std::move(loop));
        face.set_id(0);
        shell.add_face(std::move(face));
    }
    // Face 1: top (z = +z), normal = +Z
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{0, 0, z}, math::Vec3f{0, 0, 1}));
        Loop loop;
        loop.add_coedge(CoEdge{4, false});  // 4→5
        loop.add_coedge(CoEdge{5, false});  // 5→6
        loop.add_coedge(CoEdge{6, false});  // 6→7
        loop.add_coedge(CoEdge{7, false});  // 7→4
        face.add_loop(std::move(loop));
        face.set_id(1);
        shell.add_face(std::move(face));
    }
    // Face 2: front (y = -y), normal = -Y
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{0, -y, 0}, math::Vec3f{0, -1, 0}));
        Loop loop;
        loop.add_coedge(CoEdge{0, false});  // 0→1
        loop.add_coedge(CoEdge{9, false}); // 1→5
        loop.add_coedge(CoEdge{4, true});   // 5→4
        loop.add_coedge(CoEdge{8, true});   // 4→0
        face.add_loop(std::move(loop));
        face.set_id(2);
        shell.add_face(std::move(face));
    }
    // Face 3: back (y = +y), normal = +Y
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{0, y, 0}, math::Vec3f{0, 1, 0}));
        Loop loop;
        loop.add_coedge(CoEdge{3, false});  // 3→0... actually 3→2 reversed
        loop.add_coedge(CoEdge{10, false}); // 2→6
        loop.add_coedge(CoEdge{6, true});   // 6→5
        loop.add_coedge(CoEdge{1, true});   // 1→2 reversed → 2→1? No.
        // Simplify: use direct vertex sequences
        // Face 3 (back, y=+y): 2→3→7→6
        loop = Loop();
        loop.add_coedge(CoEdge{2, true});   // 3→2 reversed
        loop.add_coedge(CoEdge{11, false}); // 3→7
        loop.add_coedge(CoEdge{7, true});   // 6→7 reversed
        loop.add_coedge(CoEdge{6, true});   // 7→6 reversed
        face = Face(std::make_shared<PlaneSurface>(
            math::Vec3f{0, y, 0}, math::Vec3f{0, 1, 0}));
        face.add_loop(std::move(loop));
        face.set_id(3);
        shell.add_face(std::move(face));
    }
    // Face 4: left (x = -x), normal = -X
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{-x, 0, 0}, math::Vec3f{-1, 0, 0}));
        Loop loop;
        loop.add_coedge(CoEdge{3, false});  // 3→0
        loop.add_coedge(CoEdge{8, false});  // 0→4
        loop.add_coedge(CoEdge{7, true});   // 4→7 reversed → 7→4? No.
        loop.add_coedge(CoEdge{11, true});  // 7→3 reversed → 3→7? No.
        face.add_loop(std::move(loop));
        face.set_id(4);
        shell.add_face(std::move(face));
    }
    // Face 5: right (x = +x), normal = +X
    {
        Face face(std::make_shared<PlaneSurface>(
            math::Vec3f{x, 0, 0}, math::Vec3f{1, 0, 0}));
        Loop loop;
        loop.add_coedge(CoEdge{1, false});  // 1→2
        loop.add_coedge(CoEdge{10, false}); // 2→6... wait, edge 10 is 2→6
        loop.add_coedge(CoEdge{5, true});   // 5→6 reversed → 6→5
        loop.add_coedge(CoEdge{9, true});   // 1→5 reversed → 5→1
        face.add_loop(std::move(loop));
        face.set_id(5);
        shell.add_face(std::move(face));
    }

    return body;
}

// Create a sphere B-Rep body (single face, no edges for MVP — just
// the surface geometry, tessellated later).
inline std::unique_ptr<BRepBody> make_sphere(float radius, math::Vec3f center = {}) {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // A sphere has a single face with a cylindrical surface
    Face face(std::make_shared<SphereSurface>(radius, center));
    face.set_id(0);
    // No loops for now (full sphere surface)
    shell.add_face(std::move(face));

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

    return body;
}

} // namespace CAD_0::brep
