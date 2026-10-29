// core/bridge/src/brep_to_sdf.cpp
#include "CAD_0/bridge/brep_to_sdf.hpp"

#include <cmath>

namespace CAD_0::bridge {

// Mesh-based SDF node: evaluates signed distance to a triangle mesh.
// This is a simple brute-force implementation — for production, use a BVH.
class MeshSDF final : public sdf::SDFNode {
public:
    explicit MeshSDF(sdf::TriangleMesh mesh) : mesh_(std::move(mesh)) {
        // Precompute triangle data
        const std::size_t n = mesh_.indices.size() / 3;
        triangles_.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            Triangle tri;
            tri.v0 = mesh_.positions[mesh_.indices[i * 3 + 0]];
            tri.v1 = mesh_.positions[mesh_.indices[i * 3 + 1]];
            tri.v2 = mesh_.positions[mesh_.indices[i * 3 + 2]];
            tri.normal = (tri.v1 - tri.v0).cross(tri.v2 - tri.v0).normalized();
            triangles_.push_back(tri);
        }
    }

    sdf::SDFSample sample(const math::Vec3f& p) const noexcept override {
        if (triangles_.empty()) return {1e30f, {}};

        float min_dist = 1e30f;
        math::Vec3f best_normal{0, 1, 0};
        bool inside = false;

        for (const auto& tri : triangles_) {
            float d = point_triangle_distance(p, tri);
            if (std::abs(d) < std::abs(min_dist)) {
                min_dist = d;
                best_normal = tri.normal;
            }
            // Check if inside (ray casting — simplified)
            if (tri.normal.dot(p - tri.v0) < 0) {
                // Point is on the inside side of this triangle
            }
        }

        // Determine sign: if point is "behind" most normals, it's inside
        // This is a simplification — a proper winding check is needed
        int inside_count = 0;
        for (const auto& tri : triangles_) {
            if (tri.normal.dot(p - tri.v0) < 0) inside_count++;
        }
        inside = (static_cast<std::size_t>(inside_count) > triangles_.size() / 2);

        float signed_dist = inside ? -min_dist : min_dist;
        return {signed_dist, inside ? -best_normal : best_normal};
    }

    float lipschitz() const noexcept override { return 1.0f; }

    math::Bboxf bounds() const noexcept override {
        math::Bboxf b;
        for (const auto& p : mesh_.positions) b.expand(p);
        return b;
    }

    std::string describe() const override {
        return "mesh_sdf(" + std::to_string(triangles_.size()) + " tris)";
    }

    std::unique_ptr<sdf::SDFNode> clone() const override {
        return std::make_unique<MeshSDF>(*this);
    }

private:
    struct Triangle {
        math::Vec3f v0, v1, v2, normal;
    };

    sdf::TriangleMesh mesh_;
    std::vector<Triangle> triangles_;

    // Point-to-triangle distance (from Ericson, Real-Time Collision Detection)
    static float point_triangle_distance(const math::Vec3f& p, const Triangle& tri) {
        const math::Vec3f ab = tri.v1 - tri.v0;
        const math::Vec3f ac = tri.v2 - tri.v0;
        const math::Vec3f ap = p - tri.v0;

        float d1 = ab.dot(ap);
        float d2 = ac.dot(ap);
        if (d1 <= 0 && d2 <= 0) return ap.length();  // barycentric (1,0,0)

        const math::Vec3f bp = p - tri.v1;
        float d3 = ab.dot(bp);
        float d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3) return bp.length();  // barycentric (0,1,0)

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            float v = d1 / (d1 - d3);
            return (tri.v0 + ab * v - p).length();  // edge ab
        }

        const math::Vec3f cp = p - tri.v2;
        float d5 = ab.dot(cp);
        float d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6) return cp.length();  // barycentric (0,0,1)

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            float w = d2 / (d2 - d6);
            return (tri.v0 + ac * w - p).length();  // edge ac
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return (tri.v1 + (tri.v2 - tri.v1) * w - p).length();  // edge bc
        }

        // Inside face region
        float denom = 1.0f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        math::Vec3f closest = tri.v0 + ab * v + ac * w;
        return (closest - p).length();
    }
};

std::unique_ptr<sdf::SDFBody> brep_to_sdf(const brep::BRepBody& body) {
    // Detect known primitive types
    const auto& faces = body.solid.shell().faces();
    if (faces.empty()) return std::make_unique<sdf::SDFBody>();

    // Check if it's a sphere (1 face of type "sphere")
    if (faces.size() == 1) {
        const auto* surf = faces[0].surface();
        if (surf && std::string(surf->type_name()) == "sphere") {
            const auto* sph = dynamic_cast<const brep::SphereSurface*>(surf);
            if (sph) {
                return std::make_unique<sdf::SDFBody>(sdf::make_sphere(sph->radius()));
            }
        }
    }

    // Check if it's a cylinder (3 faces: cylinder + 2 planes)
    if (faces.size() == 3) {
        const auto* surf = faces[0].surface();
        if (surf && std::string(surf->type_name()) == "cylinder") {
            const auto* cyl = dynamic_cast<const brep::CylinderSurface*>(surf);
            if (cyl) {
                return std::make_unique<sdf::SDFBody>(
                    sdf::make_cylinder(cyl->radius(), cyl->height()));
            }
        }
    }

    // Check if it's a box (6 plane faces, 8 vertices, 12 edges)
    if (faces.size() == 6 && body.vertex_count() == 8 && body.edge_count() == 12) {
        // Compute half-extents from vertex positions
        auto b = body.bounds();
        math::Vec3f half = {(b.max.x - b.min.x) * 0.5f,
                             (b.max.y - b.min.y) * 0.5f,
                             (b.max.z - b.min.z) * 0.5f};
        return std::make_unique<sdf::SDFBody>(sdf::make_box(half));
    }

    // Fall back to mesh-based SDF
    brep::TessellationOptions opts;
    opts.resolution = 16;
    auto mesh = brep::tessellate_body(body, opts);
    return mesh_to_sdf(mesh);
}

std::unique_ptr<sdf::SDFBody> mesh_to_sdf(const sdf::TriangleMesh& mesh) {
    if (mesh.positions.empty()) return std::make_unique<sdf::SDFBody>();
    // Clone the mesh into the SDF node
    sdf::TriangleMesh mesh_copy = mesh;
    return std::make_unique<sdf::SDFBody>(
        std::make_unique<MeshSDF>(std::move(mesh_copy)));
}

} // namespace CAD_0::bridge
