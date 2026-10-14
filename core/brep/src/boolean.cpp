// core/brep/src/boolean.cpp
//
// Implementation of bool_intersect via Sutherland-Hodgman polygon
// clipping — ADR-0023 §2.
//
// The algorithm:
//   1. For each face F_A of A, extract its polygon vertices from the
//      outer loop.
//   2. Clip the polygon against every face plane of B (keep the part
//      inside B's half-space).
//   3. If the clipped polygon is non-empty, add it as a new face.
//   4. Repeat for B's faces against A.
//   5. Weld vertices (grid snapping for determinism).
//   6. Build edges from face loops.
//   7. link_radial_edges.
//
// Determinism (ADR-0014): all FP operations use dot, subtract, divide,
// lerp — deterministic under -ffp-contract=off. Vertex welding uses
// grid-snapped integer keys, independent of insertion order.
//
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace CAD_0::brep {

namespace {

using Polygon = std::vector<math::Vec3f>;

// Grid resolution for deterministic vertex welding. Points within
// 1e-6 of each other snap to the same grid cell and are welded.
constexpr float kWeldGrid = 1e-6f;

struct SnappedKey {
    std::int64_t x, y, z;
    bool operator<(const SnappedKey& o) const noexcept {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
    bool operator==(const SnappedKey& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

using SnappedKeySet = std::set<SnappedKey>;

SnappedKey snap(const math::Vec3f& p) noexcept {
    return {
        static_cast<std::int64_t>(std::llround(p.x / kWeldGrid)),
        static_cast<std::int64_t>(std::llround(p.y / kWeldGrid)),
        static_cast<std::int64_t>(std::llround(p.z / kWeldGrid)),
    };
}

// Extract the outer loop polygon of a face as a list of 3D points.
// Returns empty if the face has no loops or the surface is not a plane.
Polygon extract_face_polygon(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices
) {
    Polygon poly;
    if (face.loops().empty()) return poly;
    const auto* surf = face.surface();
    if (!surf || std::string(surf->type_name()) != "plane") return poly;
    const auto& loop = face.loops()[0];
    poly.reserve(loop.coedges().size());
    for (const auto& ce : loop.coedges()) {
        const VertexId vid = ce.start_vertex(edges);
        if (vid < vertices.size()) {
            poly.push_back(vertices[vid].position());
        }
    }
    return poly;
}

// Sutherland-Hodgman clip: keep the part of `poly` that is inside the
// half-space defined by (plane_origin, plane_normal). "Inside" means
// dot(p - origin, normal) <= tolerance.
Polygon clip_polygon(
    const Polygon& poly,
    math::Vec3f plane_origin,
    math::Vec3f plane_normal
) {
    if (poly.empty()) return {};
    Polygon result;
    result.reserve(poly.size() + 1);
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& curr = poly[i];
        const auto& next = poly[(i + 1) % n];
        const float d_curr = (curr - plane_origin).dot(plane_normal);
        const float d_next = (next - plane_origin).dot(plane_normal);
        const bool curr_inside = d_curr <= kWeldGrid;
        const bool next_inside = d_next <= kWeldGrid;
        if (curr_inside) {
            result.push_back(curr);
        }
        if (curr_inside != next_inside) {
            const float denom = d_curr - d_next;
            if (std::abs(denom) > 1e-12f) {
                const float t = d_curr / denom;
                result.push_back(curr + (next - curr) * t);
            }
        }
    }
    return result;
}

// Get the plane (origin + outward normal) of a face. Only works for
// PlaneSurface. The origin is taken from the face's first loop vertex
// (NOT the stored PlaneSurface origin, which goes stale if vertices
// are moved). The normal comes from the stored PlaneSurface (it does
// not change under translation).
bool get_face_plane(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices,
    math::Vec3f& origin,
    math::Vec3f& normal
) {
    const auto* surf = face.surface();
    if (!surf) return false;
    if (std::string(surf->type_name()) != "plane") return false;
    const auto* plane = dynamic_cast<const PlaneSurface*>(surf);
    if (!plane) return false;
    normal = plane->normal_vec();
    // Use the first vertex of the outer loop as the plane origin.
    // Any point on the plane works; using a vertex ensures the plane
    // tracks actual vertex positions, not a stale stored origin.
    if (face.loops().empty() || face.loops()[0].coedges().empty()) {
        origin = plane->origin();
        return true;
    }
    const VertexId vid = face.loops()[0].coedges()[0].start_vertex(edges);
    if (vid < vertices.size()) {
        origin = vertices[vid].position();
    } else {
        origin = plane->origin();
    }
    return true;
}

// A clipped face: the surface (a new PlaneSurface from the original
// face's plane parameters) and the clipped polygon vertices.
struct ClippedFace {
    std::shared_ptr<PlaneSurface> surface;
    Polygon polygon;
};

} // anonymous namespace

std::unique_ptr<BRepBody> bool_intersect(const BRepBody& A, const BRepBody& B) {
    // Validate: every face must be planar.
    auto validate_planar = [](const BRepBody& body) -> bool {
        for (const auto& face : body.solid.shell().faces()) {
            const auto* surf = face.surface();
            if (!surf) return false;
            if (std::string(surf->type_name()) != "plane") return false;
        }
        return true;
    };
    if (!validate_planar(A) || !validate_planar(B)) {
        throw std::invalid_argument(
            "bool_intersect: only planar-faced polyhedra are supported "
            "(curved-surface boolean is deferred — ADR-0023 §2)");
    }

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;

    const auto& shell_A = A.solid.shell();
    const auto& shell_B = B.solid.shell();

    // Clip A's faces against B's half-spaces.
    for (const auto& face_A : shell_A.faces()) {
        Polygon poly = extract_face_polygon(face_A, A.edges, A.vertices);
        if (poly.size() < 3) continue;
        for (const auto& face_B : shell_B.faces()) {
            math::Vec3f origin, normal;
            if (!get_face_plane(face_B, B.edges, B.vertices, origin, normal)) continue;
            poly = clip_polygon(poly, origin, normal);
            if (poly.size() < 3) break;
        }
        if (poly.size() >= 3) {
            math::Vec3f o, n;
            get_face_plane(face_A, A.edges, A.vertices, o, n);
            clipped_faces.push_back({std::make_shared<PlaneSurface>(poly[0], n), poly});
        }
    }
    // Clip B's faces against A's half-spaces.
    for (const auto& face_B : shell_B.faces()) {
        Polygon poly = extract_face_polygon(face_B, B.edges, B.vertices);
        if (poly.size() < 3) continue;
        for (const auto& face_A : shell_A.faces()) {
            math::Vec3f origin, normal;
            if (!get_face_plane(face_A, A.edges, A.vertices, origin, normal)) continue;
            poly = clip_polygon(poly, origin, normal);
            if (poly.size() < 3) break;
        }
        if (poly.size() >= 3) {
            // Deduplicate: skip if this face is coplanar and coincident
            // with an existing clipped face. Two faces are duplicates if
            // their vertex sets (as snapped keys) are identical.
            SnappedKeySet poly_keys;
            for (const auto& p : poly) poly_keys.insert(snap(p));
            bool is_dup = false;
            for (const auto& existing : clipped_faces) {
                if (existing.polygon.size() != poly.size()) continue;
                SnappedKeySet existing_keys;
                for (const auto& p : existing.polygon) existing_keys.insert(snap(p));
                if (existing_keys == poly_keys) {
                    is_dup = true;
                    break;
                }
            }
            if (!is_dup) {
                math::Vec3f o, n;
                get_face_plane(face_B, B.edges, B.vertices, o, n);
                clipped_faces.push_back({std::make_shared<PlaneSurface>(poly[0], n), poly});
            }
        }
    }

    if (clipped_faces.empty()) {
        // A ∩ B is empty — return an empty body (shell marked closed
        // but with no faces; is_closed_manifold will return false due
        // to the V/E/F > 0 guard).
        return result;
    }

    // Weld vertices via grid snapping.
    std::map<SnappedKey, VertexId> weld_map;
    auto weld_vertex = [&](const math::Vec3f& pos) -> VertexId {
        SnappedKey key = snap(pos);
        auto it = weld_map.find(key);
        if (it != weld_map.end()) return it->second;
        VertexId id = static_cast<VertexId>(result->vertices.size());
        result->vertices.push_back(Vertex{pos});
        result->vertices.back().set_id(id);
        weld_map.emplace(key, id);
        return id;
    };

    // Weld edges by canonical (v0 < v1) key.
    struct EdgeKey {
        VertexId a, b;
        bool operator<(const EdgeKey& o) const noexcept {
            if (a != o.a) return a < o.a;
            return b < o.b;
        }
    };
    std::map<EdgeKey, EdgeId> edge_map;
    auto get_or_create_edge = [&](VertexId v0, VertexId v1) -> EdgeId {
        EdgeKey key{v0 < v1 ? v0 : v1, v0 < v1 ? v1 : v0};
        auto it = edge_map.find(key);
        if (it != edge_map.end()) return it->second;
        EdgeId eid = static_cast<EdgeId>(result->edges.size());
        result->edges.push_back(Edge{key.a, key.b});
        result->edges.back().set_id(eid);
        edge_map.emplace(key, eid);
        return eid;
    };

    // Build faces from clipped polygons.
    FaceId next_face_id = 0;
    for (const auto& cf : clipped_faces) {
        if (cf.polygon.size() < 3) continue;

        std::vector<VertexId> vids;
        vids.reserve(cf.polygon.size());
        for (const auto& p : cf.polygon) {
            vids.push_back(weld_vertex(p));
        }

        Face face(cf.surface);
        Loop loop;
        for (std::size_t i = 0; i < vids.size(); ++i) {
            VertexId v0 = vids[i];
            VertexId v1 = vids[(i + 1) % vids.size()];
            EdgeId eid = get_or_create_edge(v0, v1);
            bool reversed = (v0 > v1);
            loop.add_coedge(CoEdge{eid, reversed});
        }
        face.add_loop(std::move(loop));
        face.set_id(next_face_id++);
        shell.add_face(std::move(face));
    }

    link_radial_edges(*result);
    return result;
}

std::unique_ptr<BRepBody> bool_union(const BRepBody& A, const BRepBody& B) {
    (void)A; (void)B;
    throw std::runtime_error(
        "bool_union: not implemented — requires polygon subtraction "
        "(non-convex clipping), deferred to Phase D step (c+) — ADR-0023 §2");
}

std::unique_ptr<BRepBody> bool_subtract(const BRepBody& A, const BRepBody& B) {
    (void)A; (void)B;
    throw std::runtime_error(
        "bool_subtract: not implemented — requires polygon subtraction "
        "(non-convex clipping), deferred to Phase D step (c+) — ADR-0023 §2");
}

} // namespace CAD_0::brep
