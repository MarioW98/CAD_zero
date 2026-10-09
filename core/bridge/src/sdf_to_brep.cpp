// core/bridge/src/sdf_to_brep.cpp
#include "CAD_0/bridge/sdf_to_brep.hpp"

#include <cmath>
#include <map>
#include <sstream>

namespace CAD_0::bridge {

// ---------------------------------------------------------------------------
// Faceted conversion: each triangle → Face with PlaneSurface
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep_faceted(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts) {

    auto body = std::make_unique<brep::BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    // Extract mesh
    sdf::TriangleMesh mesh = sdf::marching_cubes(
        sdf_body, opts.march_resolution, opts.compute_normals);

    if (mesh.positions.empty() || mesh.indices.empty()) {
        return body;
    }

    // Create vertices
    body->vertices.reserve(mesh.positions.size());
    for (const auto& p : mesh.positions) {
        body->vertices.push_back(brep::Vertex{p});
    }
    for (std::size_t i = 0; i < body->vertices.size(); ++i) {
        body->vertices[i].set_id(static_cast<brep::VertexId>(i));
    }

    // Create edges: for each triangle, create 3 edges (deduplicated by vertex pair)
    // Use a map to avoid duplicate edges
    std::map<std::pair<brep::VertexId, brep::VertexId>, brep::EdgeId> edge_map;
    auto get_or_create_edge = [&](brep::VertexId v0, brep::VertexId v1) -> brep::EdgeId {
        auto key = std::make_pair(std::min(v0, v1), std::max(v0, v1));
        auto it = edge_map.find(key);
        if (it != edge_map.end()) return it->second;
        brep::EdgeId eid = static_cast<brep::EdgeId>(body->edges.size());
        body->edges.push_back(brep::Edge{v0, v1});
        body->edges.back().set_id(eid);
        edge_map[key] = eid;
        return eid;
    };

    // Create faces: each triangle → Face with PlaneSurface
    const std::size_t num_tris = mesh.indices.size() / 3;
    for (std::size_t t = 0; t < num_tris; ++t) {
        const auto i0 = mesh.indices[t * 3 + 0];
        const auto i1 = mesh.indices[t * 3 + 1];
        const auto i2 = mesh.indices[t * 3 + 2];

        const auto& p0 = mesh.positions[i0];
        const auto& p1 = mesh.positions[i1];
        const auto& p2 = mesh.positions[i2];

        // Compute face normal
        math::Vec3f normal = (p1 - p0).cross(p2 - p0).normalized();
        if (normal.length() < 1e-9f) continue;  // degenerate triangle

        // Create edges
        brep::EdgeId e0 = get_or_create_edge(i0, i1);
        brep::EdgeId e1 = get_or_create_edge(i1, i2);
        brep::EdgeId e2 = get_or_create_edge(i2, i0);

        // Determine edge direction (reversed or not)
        bool r0 = (i0 > i1);
        bool r1 = (i1 > i2);
        bool r2 = (i2 > i0);

        // Create face with plane surface
        auto surf = std::make_shared<brep::PlaneSurface>(p0, normal);
        brep::Face face(surf);
        face.set_id(static_cast<brep::FaceId>(t));

        // Create loop
        brep::Loop loop;
        loop.add_coedge(brep::CoEdge{e0, r0});
        loop.add_coedge(brep::CoEdge{e1, r1});
        loop.add_coedge(brep::CoEdge{e2, r2});
        face.add_loop(std::move(loop));

        shell.add_face(std::move(face));
    }

    return body;
}

// ---------------------------------------------------------------------------
// Primitive fitting: detect known SDF types
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep_primitive(
    const sdf::SDFBody& sdf_body) {

    const sdf::SDFNode* root = sdf_body.root();
    if (!root) return nullptr;

    std::string desc = root->describe();

    // Check for sphere: description must START with "sphere(" (not contain it)
    if (desc.substr(0, 7) == "sphere(") {
        auto pos = desc.find("r=");
        if (pos != std::string::npos) {
            float r = std::stof(desc.substr(pos + 2));
            return brep::make_sphere(r);
        }
    }

    // Check for box
    if (desc.substr(0, 4) == "box(") {
        auto pos = desc.find("e=");
        if (pos != std::string::npos) {
            std::string vals = desc.substr(pos + 2);
            float ex = std::stof(vals);
            auto c1 = vals.find(",");
            if (c1 != std::string::npos) {
                float ey = std::stof(vals.substr(c1 + 1));
                auto c2 = vals.find(",", c1 + 1);
                if (c2 != std::string::npos) {
                    float ez = std::stof(vals.substr(c2 + 1));
                    return brep::make_box({ex, ey, ez});
                }
            }
        }
    }

    // Check for cylinder
    if (desc.substr(0, 9) == "cylinder(") {
        auto pos = desc.find("r=");
        if (pos != std::string::npos) {
            float r = std::stof(desc.substr(pos + 2));
            auto hpos = desc.find("h=");
            if (hpos != std::string::npos) {
                float h = std::stof(desc.substr(hpos + 2));
                return brep::make_cylinder(r, h);
            }
        }
    }

    // Not a recognized primitive
    return nullptr;
}

// ---------------------------------------------------------------------------
// High-level: try primitive first, fall back to faceted
// ---------------------------------------------------------------------------
std::unique_ptr<brep::BRepBody> sdf_to_brep(
    const sdf::SDFBody& sdf_body,
    const SdfToBrepOptions& opts) {

    // Try primitive fitting first
    if (opts.fit_primitives) {
        auto body = sdf_to_brep_primitive(sdf_body);
        if (body) return body;
    }

    // Fall back to faceted conversion
    return sdf_to_brep_faceted(sdf_body, opts);
}

} // namespace CAD_0::bridge
