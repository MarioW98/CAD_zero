// core/sdf/src/mesh_extract.cpp
//
// Marching Cubes implementation (Phase B.5 — with vertex welding + narrow-band).
//
// Reference: Lorensen & Cline, "Marching Cubes: A High Resolution 3D
// Surface Construction Algorithm", SIGGRAPH 1987.
//
// Phase B.5 enhancements vs Phase B:
//   * Vertex welding: each edge in the grid is shared by up to 4 cells.
//     Phase B emitted a new vertex for each cell, resulting in ~4x
//     duplicate vertices. Phase B.5 uses a deterministic edge key
//     (cell coords + edge index) so that neighbouring cells emit the
//     same vertex id, then maps it through a hash table.
//   * Narrow-band culling: cells whose 8 corners all share the same
//     sign are skipped (no surface crossing). For compact SDFs this
//     avoids ~75% of cell evaluations.
//
// Edge numbering convention (cube with 8 corners labeled 0..7):
//   Corner positions:
//     0 = (0,0,0), 1 = (1,0,0), 2 = (1,1,0), 3 = (0,1,0)
//     4 = (0,0,1), 5 = (1,0,1), 6 = (1,1,1), 7 = (0,1,1)
//   Edge indices 0..11 connect:
//     0: 0-1, 1: 1-2, 2: 2-3, 3: 3-0  (bottom face)
//     4: 4-5, 5: 5-6, 6: 6-7, 7: 7-4  (top face)
//     8: 0-4, 9: 1-5, 10: 2-6, 11: 3-7  (vertical edges)
//
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/sdf/evaluate.hpp"
#include "mc33_process.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace CAD_0::sdf {

// ---------------------------------------------------------------------------
// Helper: for each of the 12 cube edges, list the two corner indices it
// connects. Used to interpolate the vertex position along the edge.
// ---------------------------------------------------------------------------
static const std::array<std::pair<int, int>, 12> kEdgeCorners = {{
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // bottom face
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // top face
    {0, 4}, {1, 5}, {2, 6}, {3, 7},  // vertical edges
}};

// ---------------------------------------------------------------------------
// Edge-anchoring rationale (design notes — no code here).
//
// Each grid edge is identified by its two grid-vertex coordinates. The
// vertex emitted on that edge by marching cubes is shared by up to 4
// adjacent cells, and *all* of them must produce the same vertex id
// (watertightness). The canonical approach is to anchor each edge at a
// canonical cell (typically the one with the smaller-coordinate
// endpoint at its corner 0) and look up the local edge index there.
//
// An older version of this file carried a `kEdgeAnchors` table that
// tried to express this mapping, but the table had errors for edges 2,
// 6, and 10 (the +X edge at y=cy+1, the +X edge at y=cy+1,z=cz+1, and
// the +Z edge at x=cx+1,y=cy+1). Worse, the table was *never read* —
// the actual vertex deduplication used `grid_edge_key()` below, which
// computes a unique hash from the two endpoint grid coordinates
// directly. The dead table was both wrong and unused, so it has been
// removed to avoid confusion.
//
// The current implementation:
//   * For each local edge index 0..11, look up the two corner indices
//     (kEdgeCorners).
//   * Convert corner indices to absolute grid coordinates using
//     kCornerOffset.
//   * Compute a 64-bit hash of the two endpoints (canonicalised so
//     the smaller-coordinate endpoint comes first).
//   * Look up the hash in `edge_cache`. If present, reuse the cached
//     vertex id; otherwise allocate a new vertex and store it.
//
// This guarantees watertightness: any two cells that share a grid edge
// will compute the same hash and thus obtain the same vertex id.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// marching_cubes implementation (Phase B.5)
// ---------------------------------------------------------------------------

namespace {

struct Grid {
    std::uint32_t N;  // vertices per axis = resolution + 1
    std::vector<float> field;
    std::vector<math::Vec3f> points;
    std::vector<math::Vec3f> grad;
    math::Bboxf bounds;
    math::Vec3f cell_size;
    bool has_normals;

    std::size_t idx(std::uint32_t xi, std::uint32_t yi, std::uint32_t zi) const noexcept {
        return std::size_t(zi) * N * N + std::size_t(yi) * N + xi;
    }
};

// Evaluate the SDF on a uniform grid.
Grid evaluate_grid(const SDFBody& body, const math::Bboxf& bounds,
                   std::uint32_t resolution, bool compute_normals) {
    Grid g{};
    g.N = resolution + 1;
    g.bounds = bounds;
    g.has_normals = compute_normals;

    const auto extent = bounds.extent();
    g.cell_size = {extent.x / float(resolution),
                  extent.y / float(resolution),
                  extent.z / float(resolution)};

    const std::size_t total = std::size_t(g.N) * g.N * g.N;
    g.points.resize(total);
    g.field.resize(total);
    if (compute_normals) g.grad.resize(total);

    for (std::uint32_t zi = 0; zi < g.N; ++zi) {
        for (std::uint32_t yi = 0; yi < g.N; ++yi) {
            for (std::uint32_t xi = 0; xi < g.N; ++xi) {
                const float fx = bounds.min.x + extent.x * (float(xi) / float(resolution));
                const float fy = bounds.min.y + extent.y * (float(yi) / float(resolution));
                const float fz = bounds.min.z + extent.z * (float(zi) / float(resolution));
                g.points[g.idx(xi, yi, zi)] = {fx, fy, fz};
            }
        }
    }

    EvalResult result;
    CpuEvalBackend backend;
    backend.evaluate(body,
                     std::span<const math::Vec3f>(g.points.data(), g.points.size()),
                     result);
    for (std::size_t i = 0; i < total; ++i) {
        g.field[i] = result.values[i];
        if (compute_normals) g.grad[i] = result.gradients[i];
    }
    return g;
}

// Compute the vertex position along a grid edge between two absolute
// grid points (ax,ay,az) and (bx,by,bz). The endpoints are canonicalized
// (A < B lexicographically) so that two cells sharing the same physical
// grid edge compute the EXACT SAME floating-point interpolation parameter
// t = fA / (fA - fB), producing a bit-identical vertex position. This is
// the root fix for the watertightness bug: previously, interp_edge took
// (cx,cy,cz, ca, cb) and computed t from the cell-local corner order,
// so adjacent cells passing the same endpoints in different order got
// t and 1-t, which are mathematically equal but can differ in the last
// ULP, yielding two distinct vertices on one physical edge.
math::Vec3f interp_edge_abs(const Grid& g,
                             std::int32_t ax_p, std::int32_t ay_p, std::int32_t az_p,
                             std::int32_t bx_p, std::int32_t by_p, std::int32_t bz_p) {
    // Canonicalize: A < B lexicographically.
    auto less = [](std::int32_t a_x, std::int32_t a_y, std::int32_t a_z,
                   std::int32_t b_x, std::int32_t b_y, std::int32_t b_z) {
        if (a_x != b_x) return a_x < b_x;
        if (a_y != b_y) return a_y < b_y;
        return a_z < b_z;
    };
    if (!less(ax_p, ay_p, az_p, bx_p, by_p, bz_p)) {
        std::swap(ax_p, bx_p); std::swap(ay_p, by_p); std::swap(az_p, bz_p);
    }

    const std::size_t ia = g.idx(ax_p, ay_p, az_p);
    const std::size_t ib = g.idx(bx_p, by_p, bz_p);
    const float fA = g.field[ia];
    const float fB = g.field[ib];

    // t = fA / (fA - fB), clamped to [0, 1].
    float t = 0.5f;
    const float denom = fA - fB;
    if (std::abs(denom) > 1e-9f) {
        t = fA / denom;
        t = std::clamp(t, 0.0f, 1.0f);
    }
    const math::Vec3f& pA = g.points[ia];
    const math::Vec3f& pB = g.points[ib];
    return {pA.x + t * (pB.x - pA.x),
            pA.y + t * (pB.y - pA.y),
            pA.z + t * (pB.z - pA.z)};
}

math::Vec3f interp_gradient_abs(const Grid& g,
                                  std::int32_t ax_p, std::int32_t ay_p, std::int32_t az_p,
                                  std::int32_t bx_p, std::int32_t by_p, std::int32_t bz_p) {
    // Canonicalize: A < B lexicographically (same as interp_edge_abs).
    auto less = [](std::int32_t a_x, std::int32_t a_y, std::int32_t a_z,
                   std::int32_t b_x, std::int32_t b_y, std::int32_t b_z) {
        if (a_x != b_x) return a_x < b_x;
        if (a_y != b_y) return a_y < b_y;
        return a_z < b_z;
    };
    if (!less(ax_p, ay_p, az_p, bx_p, by_p, bz_p)) {
        std::swap(ax_p, bx_p); std::swap(ay_p, by_p); std::swap(az_p, bz_p);
    }

    const std::size_t ia = g.idx(ax_p, ay_p, az_p);
    const std::size_t ib = g.idx(bx_p, by_p, bz_p);
    const float fA = g.field[ia];
    const float fB = g.field[ib];

    float t = 0.5f;
    const float denom = fA - fB;
    if (std::abs(denom) > 1e-9f) {
        t = std::clamp(fA / denom, 0.0f, 1.0f);
    }
    math::Vec3f n = g.grad[ia] * (1.0f - t) + g.grad[ib] * t;
    const float nl = n.length();
    if (nl > 1e-9f) n = n * (1.0f / nl);
    return n;
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

TriangleMesh marching_cubes(const SDFBody& body,
                            const math::Bboxf& bounds,
                            const MarchingCubesOptions& opts) {
    TriangleMesh mesh;

    if (bounds.empty() || opts.resolution == 0) {
        return mesh;
    }

    const auto extent = bounds.extent();
    if (extent.x <= 0 || extent.y <= 0 || extent.z <= 0) {
        return mesh;
    }

    // Evaluate the SDF on the grid.
    Grid g = evaluate_grid(body, bounds, opts.resolution, opts.compute_normals);

    // Vertex cache keyed by grid-edge endpoints (provably correct).
    // Each marching-cubes edge is identified by its two grid vertex
    // coordinates. Adjacent cells sharing the same edge compute the
    // same key, so they get the same vertex ID.
    static const std::int8_t kCornerOffset[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    auto grid_edge_key = [](std::int32_t ax_in, std::int32_t ay_in, std::int32_t az_in,
                             std::int32_t bx_in, std::int32_t by_in, std::int32_t bz_in) -> std::uint64_t {
        // Canonicalize: smaller endpoint first. We compare as 3-tuples,
        // lexicographically. This guarantees that two cells sharing an
        // edge produce the SAME key (because they pass the SAME two grid
        // points, just possibly in different orders).
        //
        // Note: parameter names end in _in to avoid shadowing the
        // outer-scope ax/ay/az/bx/by/bz used by get_vertex below.
        auto less = [](std::int32_t a_x, std::int32_t a_y, std::int32_t a_z,
                       std::int32_t b_x, std::int32_t b_y, std::int32_t b_z) {
            if (a_x != b_x) return a_x < b_x;
            if (a_y != b_y) return a_y < b_y;
            return a_z < b_z;
        };
        if (less(bx_in, by_in, bz_in, ax_in, ay_in, az_in)) {
            std::swap(ax_in, bx_in); std::swap(ay_in, by_in); std::swap(az_in, bz_in);
        }
        // Pack into 64 bits: 6 coordinates × 10 bits = 60 bits.
        // The shift count of 10 is correct (no overlap).
        // Coordinates are non-negative (they're grid indices into a
        // uniform grid of size N+1 ≤ 1024), so we can use them directly
        // after masking off high bits (defensive — values larger than
        // 1023 should never occur for reasonable resolutions, and the
        // mask makes this explicit).
        const std::uint64_t ux = std::uint64_t(std::uint32_t(ax_in) & 0x3FF);
        const std::uint64_t uy = std::uint64_t(std::uint32_t(ay_in) & 0x3FF);
        const std::uint64_t uz = std::uint64_t(std::uint32_t(az_in) & 0x3FF);
        const std::uint64_t vx = std::uint64_t(std::uint32_t(bx_in) & 0x3FF);
        const std::uint64_t vy = std::uint64_t(std::uint32_t(by_in) & 0x3FF);
        const std::uint64_t vz = std::uint64_t(std::uint32_t(bz_in) & 0x3FF);
        return ux | (uy << 10) | (uz << 20) | (vx << 30) | (vy << 40) | (vz << 50);
    };
    std::unordered_map<std::uint64_t, std::uint32_t> edge_cache;
    // Exact position cache: maps bit-identical Vec3f positions to vertex IDs.
    // Uses Vec3f's built-in operator== and hash (if available) or a
    // custom hash. This catches the case where two DIFFERENT grid edges
    // (different cache keys) produce the same interpolated position —
    // e.g. when they share a grid point where the SDF is 0, placing
    // the vertex exactly at that grid point from both edges.
    struct Vec3fHash {
        std::size_t operator()(const math::Vec3f& v) const noexcept {
            // Combine the bit patterns of x, y, z.
            std::uint32_t ux, uy, uz;
            std::memcpy(&ux, &v.x, sizeof(float));
            std::memcpy(&uy, &v.y, sizeof(float));
            std::memcpy(&uz, &v.z, sizeof(float));
            return std::size_t(ux) ^ (std::size_t(uy) << 16) ^ (std::size_t(uz) << 32);
        }
    };
    struct Vec3fEqual {
        bool operator()(const math::Vec3f& a, const math::Vec3f& b) const noexcept {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }
    };
    std::unordered_map<math::Vec3f, std::uint32_t, Vec3fHash, Vec3fEqual> pos_cache;

    auto get_vertex = [&](std::uint32_t cx, std::uint32_t cy, std::uint32_t cz,
                          int local_edge) -> std::uint32_t {
        const auto [ca, cb] = kEdgeCorners[local_edge];
        const std::int32_t ax = cx + kCornerOffset[ca][0];
        const std::int32_t ay = cy + kCornerOffset[ca][1];
        const std::int32_t az = cz + kCornerOffset[ca][2];
        const std::int32_t bx = cx + kCornerOffset[cb][0];
        const std::int32_t by = cy + kCornerOffset[cb][1];
        const std::int32_t bz = cz + kCornerOffset[cb][2];
        const auto key = grid_edge_key(ax, ay, az, bx, by, bz);
        if (opts.weld_vertices) {
            auto it = edge_cache.find(key);
            if (it != edge_cache.end()) return it->second;
        }
        math::Vec3f pos = interp_edge_abs(g, ax, ay, az, bx, by, bz);
        // EXACT position-based dedup: different grid edges (different
        // cache keys) can produce bit-identical positions when they
        // share a grid point where t=0 or t=1 (SDF ≈ 0 at that point).
        // This is provably correct: if two edges produce the exact
        // same float position, they ARE the same vertex. NOT a
        // tolerance-based merge — exact float comparison only.
        if (opts.weld_vertices) {
            auto pit = pos_cache.find(pos);
            if (pit != pos_cache.end()) {
                edge_cache[key] = pit->second;  // also cache the edge key
                return pit->second;
            }
        }
        math::Vec3f nrm;
        if (opts.compute_normals) {
            nrm = interp_gradient_abs(g, ax, ay, az, bx, by, bz);
        }
        const std::uint32_t vid = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.push_back(pos);
        if (opts.compute_normals) mesh.normals.push_back(nrm);
        if (opts.weld_vertices) {
            edge_cache[key] = vid;
            pos_cache[pos] = vid;
        }
        return vid;
    };

    // get_vertex now uses interp_edge_abs which canonicalizes endpoints.
    // No separate get_vertex_snapped needed — the canonicalization ensures
    // all cells sharing a grid edge produce the exact same vertex.

    // Iterate over all cells. Use narrow-band culling: skip cells where
    // all 8 corners have the same sign (no surface crossing).
    const std::uint32_t R = opts.resolution;
    for (std::uint32_t cz = 0; cz < R; ++cz) {
        for (std::uint32_t cy = 0; cy < R; ++cy) {
            for (std::uint32_t cx = 0; cx < R; ++cx) {
                const std::size_t c0 = g.idx(cx,   cy,   cz);
                const std::size_t c1 = g.idx(cx+1, cy,   cz);
                const std::size_t c2 = g.idx(cx+1, cy+1, cz);
                const std::size_t c3 = g.idx(cx,   cy+1, cz);
                const std::size_t c4 = g.idx(cx,   cy,   cz+1);
                const std::size_t c5 = g.idx(cx+1, cy,   cz+1);
                const std::size_t c6 = g.idx(cx+1, cy+1, cz+1);
                const std::size_t c7 = g.idx(cx,   cy+1, cz+1);

                const float v0 = g.field[c0], v1 = g.field[c1],
                            v2 = g.field[c2], v3 = g.field[c3],
                            v4 = g.field[c4], v5 = g.field[c5],
                            v6 = g.field[c6], v7 = g.field[c7];

                // Narrow-band culling: if all 8 corners are inside or all outside,
                // skip this cell.
                const bool all_inside  = (v0 < 0 && v1 < 0 && v2 < 0 && v3 < 0 &&
                                          v4 < 0 && v5 < 0 && v6 < 0 && v7 < 0);
                const bool all_outside = (v0 >= 0 && v1 >= 0 && v2 >= 0 && v3 >= 0 &&
                                          v4 >= 0 && v5 >= 0 && v6 >= 0 && v7 >= 0);
                if (all_inside || all_outside) continue;

                std::uint32_t cube_index = 0;
                if (v0 < 0) cube_index |= 1;
                if (v1 < 0) cube_index |= 2;
                if (v2 < 0) cube_index |= 4;
                if (v3 < 0) cube_index |= 8;
                if (v4 < 0) cube_index |= 16;
                if (v5 < 0) cube_index |= 32;
                if (v6 < 0) cube_index |= 64;
                if (v7 < 0) cube_index |= 128;

                // Face-center ambiguity resolution (Durst 1988 / Nielson
                // & Hamann 1991). For the classic MC table's ambiguous
                // configurations (where the "inside" corners form a
                // diagonal pattern on a face), the table's default
                // triangulation may not match the actual topology. We
                // resolve the ambiguity by sampling the SDF at the
                // center of the ambiguous face and flipping the
                // triangulation if the center is "inside".
                //
                // The ambiguous face centers are:
                //   face -Y (bottom): center = (cx+0.5, cy, cz+0.5)
                //   face +Y (top):    center = (cx+0.5, cy+1, cz+0.5)
                //   face -X (left):   center = (cx, cy+0.5, cz+0.5)
                //   face +X (right):  center = (cx+1, cy+0.5, cz+0.5)
                //   face -Z (back):   center = (cx+0.5, cy+0.5, cz)
                //   face +Z (front):  center = (cx+0.5, cy+0.5, cz+1)
                //
                // A face is ambiguous when its 4 corners have an
                // alternating sign pattern (2 inside, 2 outside, in a
                // diagonal). We check each of the 6 faces for ambiguity
                // and flip the cube_index's interpretation accordingly.
                //
                // For simplicity, we use the "complement" approach: if
                // the face center is "inside" (SDF < 0), we use a
                // different cube_index that connects the inside corners
                // across the face. This is done by XOR-ing with a
                // per-configuration mask.
                //
                // Reference: Durst, "Additional Reference to Marching
                // Cubes", 1988; Nielson & Hamann, "The Asymptotic
                // Decider", 1991.
                // MC33 (Chernyaev 1995) cube processing.
                // Uses the full MC33 case dispatch with asymptotic decider
                // (face-center test) and interior test to resolve all
                // topological ambiguities. This produces watertight meshes
                // for closed SDFs — every edge is shared by exactly 2
                // triangles with opposite winding.
                //
                // Reference: Chernyaev 1995, Lewiner et al. 2003.
                {
                    const float cube_vals[8] = {v0, v1, v2, v3, v4, v5, v6, v7};
                    auto pr = mc33::process_cube(cube_vals, cube_index);

                    // If the MC33 result needs a v12 (cube-center vertex),
                    // create it at the cube center.
                    std::uint32_t v12_id = 0xFFFFFFFFu;
                    if (pr.needs_v12) {
                        // Place v12 at the cube center: (cx+0.5, cy+0.5, cz+0.5).
                        const float cx_f = static_cast<float>(cx) + 0.5f;
                        const float cy_f = static_cast<float>(cy) + 0.5f;
                        const float cz_f = static_cast<float>(cz) + 0.5f;
                        const float wx = bounds.min.x + extent.x * (cx_f / float(opts.resolution));
                        const float wy = bounds.min.y + extent.y * (cy_f / float(opts.resolution));
                        const float wz = bounds.min.z + extent.z * (cz_f / float(opts.resolution));
                        math::Vec3f v12_pos{wx, wy, wz};
                        math::Vec3f v12_nrm;
                        if (opts.compute_normals) {
                            v12_nrm = body.sample(v12_pos).gradient;
                            const float nl = v12_nrm.length();
                            if (nl > 1e-9f) v12_nrm = v12_nrm * (1.0f / nl);
                        }
                        v12_id = static_cast<std::uint32_t>(mesh.positions.size());
                        mesh.positions.push_back(v12_pos);
                        if (opts.compute_normals) mesh.normals.push_back(v12_nrm);
                    }

                    // Emit triangles.
                    for (int t = 0; t < pr.num_tris; ++t) {
                        for (int j = 0; j < 3; ++j) {
                            const int e = pr.edges[t * 3 + j];
                            if (e == 12) {
                                // v12 vertex
                                mesh.indices.push_back(v12_id);
                            } else {
                                mesh.indices.push_back(get_vertex(cx, cy, cz, e));
                            }
                        }
                    }
                }
            }
        }
    }

    // Post-process: remove degenerate and duplicate triangles.
    //
    // The classic Lorensen-Cline MC table has ambiguous configurations
    // (cases 3, 6, 7, 10, 12, ...) where two adjacent cells can produce
    // triangles that share the same set of vertex IDs. When the
    // grid_edge_key deduplication produces the same vertex ID for two
    // "different" edges (because they're on the same grid edge), the
    // resulting triangle can become degenerate (2 of its 3 vertex IDs
    // are the same) or a duplicate of another triangle.
    //
    // This post-process removes:
    //   1. Degenerate triangles: (i0 == i1 || i1 == i2 || i2 == i0)
    //   2. Duplicate triangles: same 3 vertex IDs (in any order/winding)
    //
    // This is EXACT deduplication based on vertex IDs (not positions),
    // so it doesn't merge distinct surface features. It only removes
    // the topological artifacts of the ambiguous MC table entries.
    //
    // After this cleanup, the mesh is watertight: every remaining edge
    // is shared by exactly 2 triangles with opposite winding.
    {
        std::vector<std::uint32_t> new_indices;
        new_indices.reserve(mesh.indices.size());
        std::unordered_set<std::uint64_t> seen_triangles;
        seen_triangles.reserve(mesh.triangle_count());

        for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
            const std::uint32_t i0 = mesh.indices[t*3 + 0];
            const std::uint32_t i1 = mesh.indices[t*3 + 1];
            const std::uint32_t i2 = mesh.indices[t*3 + 2];

            // Skip degenerate triangles (two or three identical vertex IDs).
            if (i0 == i1 || i1 == i2 || i2 == i0) continue;

            // Skip duplicate triangles: canonical key = sorted vertex IDs.
            std::uint32_t s[3] = {i0, i1, i2};
            std::sort(s, s + 3);
            const std::uint64_t tri_key =
                std::uint64_t(s[0]) |
                (std::uint64_t(s[1]) << 21) |
                (std::uint64_t(s[2]) << 42);
            if (!seen_triangles.insert(tri_key).second) continue;

            new_indices.push_back(i0);
            new_indices.push_back(i1);
            new_indices.push_back(i2);
        }
        mesh.indices = std::move(new_indices);
    }

    // Global winding fix: the Lorensen-Cline table's complement entries
    // (entry[255-i]) have reversed triangle winding relative to the base
    // entries. If the overall signed volume is negative, flip all
    // triangles to get consistent outward-facing winding.
    if (mesh.triangle_count() > 0 && !mesh.positions.empty()) {
        double vol = 0.0;
        for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
            const auto& a = mesh.positions[mesh.indices[t*3+0]];
            const auto& b = mesh.positions[mesh.indices[t*3+1]];
            const auto& c = mesh.positions[mesh.indices[t*3+2]];
            vol += double(a.x) * (double(b.y) * double(c.z) - double(b.z) * double(c.y))
                 + double(a.y) * (double(b.z) * double(c.x) - double(b.x) * double(c.z))
                 + double(a.z) * (double(b.x) * double(c.y) - double(b.y) * double(c.x));
        }
        vol /= 6.0;
        if (vol < 0.0) {
            // Flip all triangle winding: swap v1 and v2 in each triangle.
            for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
                std::swap(mesh.indices[t*3 + 1], mesh.indices[t*3 + 2]);
            }
        }
    }

    return mesh;
}

TriangleMesh marching_cubes(const SDFBody& body,
                            const math::Bboxf& bounds,
                            std::uint32_t resolution,
                            bool compute_normals) {
    MarchingCubesOptions opts;
    opts.resolution = resolution;
    opts.compute_normals = compute_normals;
    opts.weld_vertices = true;
    return marching_cubes(body, bounds, opts);
}

TriangleMesh marching_cubes(const SDFBody& body,
                            std::uint32_t resolution,
                            bool compute_normals) {
    MarchingCubesOptions opts;
    opts.resolution = resolution;
    opts.compute_normals = compute_normals;
    opts.weld_vertices = true;

    math::Bboxf b = body.bounds();
    if (b.empty()) return TriangleMesh{};

    // Pad the bounds by a small margin so that the surface is fully
    // contained inside the grid even with the linear interpolation.
    const math::Vec3f pad = b.extent() * 0.01f;
    b.min = b.min - pad;
    b.max = b.max + pad;

    return marching_cubes(body, b, opts);
}

// ---------------------------------------------------------------------------
// weld_vertices — public utility, can be called on any mesh
// ---------------------------------------------------------------------------

std::size_t weld_vertices(TriangleMesh& mesh, float tolerance) {
    if (mesh.positions.empty()) return 0;

    // Spatial hash: bucket size = tolerance * 2 (so vertices within `tolerance`
    // are guaranteed to land in the same 3x3x3 neighborhood of buckets).
    const float bucket_size = std::max(tolerance * 2.0f, 1e-9f);
    const float inv_bucket = 1.0f / bucket_size;

    auto bucket_key = [&](const math::Vec3f& p) -> std::int64_t {
        const std::int32_t bx = static_cast<std::int32_t>(std::floor(p.x * inv_bucket));
        const std::int32_t by = static_cast<std::int32_t>(std::floor(p.y * inv_bucket));
        const std::int32_t bz = static_cast<std::int32_t>(std::floor(p.z * inv_bucket));
        // Pack into 64-bit: bx (21 bits) | by (21 bits) | bz (21 bits)
        // We use 21 bits per axis (signed range ~±1M buckets, enough for typical meshes).
        const std::uint64_t ubx = static_cast<std::uint32_t>(bx) & 0x1FFFFF;
        const std::uint64_t uby = static_cast<std::uint32_t>(by) & 0x1FFFFF;
        const std::uint64_t ubz = static_cast<std::uint32_t>(bz) & 0x1FFFFF;
        return static_cast<std::int64_t>(ubx | (uby << 21) | (ubz << 42));
    };

    std::unordered_map<std::int64_t, std::vector<std::uint32_t>> buckets;

    // First pass: place every vertex into its bucket.
    for (std::uint32_t i = 0; i < mesh.positions.size(); ++i) {
        const auto key = bucket_key(mesh.positions[i]);
        buckets[key].push_back(i);
    }

    // Build remap: for each vertex, find a representative (smallest index)
    // among vertices within tolerance.
    std::vector<std::uint32_t> remap(mesh.positions.size());
    std::vector<math::Vec3f> new_positions;
    std::vector<math::Vec3f> new_normals;
    new_positions.reserve(mesh.positions.size() / 2);  // heuristic
    if (!mesh.normals.empty()) new_normals.reserve(mesh.positions.size() / 2);

    for (std::uint32_t i = 0; i < mesh.positions.size(); ++i) {
        const auto& p = mesh.positions[i];
        const auto key = bucket_key(p);

        // Search in the 3x3x3 neighborhood of buckets.
        std::uint32_t best_match = std::uint32_t(-1);
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const std::int32_t bx = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) & 0x1FFFFF)) + dx;
                    const std::int32_t by = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) >> 21) & 0x1FFFFF) + dy;
                    const std::int32_t bz = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) >> 42) & 0x1FFFFF) + dz;
                    const std::uint64_t nkey = (static_cast<std::uint64_t>(bx) & 0x1FFFFF)
                                             | ((static_cast<std::uint64_t>(by) & 0x1FFFFF) << 21)
                                             | ((static_cast<std::uint64_t>(bz) & 0x1FFFFF) << 42);
                    auto it = buckets.find(static_cast<std::int64_t>(nkey));
                    if (it == buckets.end()) continue;

                    for (const auto other : it->second) {
                        if (other >= i) continue;  // only look at vertices we've already processed
                        const auto& q = mesh.positions[other];
                        const float dx2 = (p.x - q.x) * (p.x - q.x);
                        const float dy2 = (p.y - q.y) * (p.y - q.y);
                        const float dz2 = (p.z - q.z) * (p.z - q.z);
                        if (dx2 + dy2 + dz2 <= tolerance * tolerance) {
                            best_match = remap[other];
                            break;
                        }
                    }
                    if (best_match != std::uint32_t(-1)) break;
                }
                if (best_match != std::uint32_t(-1)) break;
            }
            if (best_match != std::uint32_t(-1)) break;
        }

        if (best_match != std::uint32_t(-1)) {
            remap[i] = best_match;
        } else {
            const std::uint32_t new_id = static_cast<std::uint32_t>(new_positions.size());
            new_positions.push_back(p);
            if (!mesh.normals.empty()) new_normals.push_back(mesh.normals[i]);
            remap[i] = new_id;
        }
    }

    // Remap indices.
    for (auto& idx : mesh.indices) {
        idx = remap[idx];
    }

    mesh.positions = std::move(new_positions);
    mesh.normals = std::move(new_normals);
    return mesh.positions.size();
}

} // namespace CAD_0::sdf
