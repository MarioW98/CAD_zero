// core/bridge/src/brep_to_sdf.cpp
//
// H2-u4 MORSO 1 — THE SIGN OF THE BRIDGE: the generalized winding
// number, the honest refusal of open meshes, the discrimination.
//
// What was here before (the class named, for the record): a
// MAJORITY-OF-NORMALS VOTE — the old sample() counted the triangles
// whose plane put the query on the inner side and called the point
// inside when more than half did (brep_to_sdf.cpp:45-51 as it stood,
// its own comment confessing "a proper winding check is needed").
// The vote is wrong for concave bodies (the concavity's own faces
// outvote the truth), wrong at moderate outside distances (the
// ADR-0026 non-goals record said so on the page), and it computed in
// float. It is gone; the winding number below replaces it, and the
// tests pin the discrimination (the naive counterproof, red where the
// truth is green).
#include "CAD_0/bridge/brep_to_sdf.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace CAD_0::bridge {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFourPi = 4.0 * kPi;

// ---------------------------------------------------------------------------
// The signed solid angle of one triangle, van Oosterom–Strackee (1983).
//
//     tan(Ω/2) = a·(b×c) / ( |a||b||c| + (a·b)|c| + (a·c)|b| + (b·c)|a| )
//
// with a, b, c the vertices RELATIVE to the query point. This atan
// form is the numerically stable one: near the degenerate cases (the
// query on the triangle's plane, Ω → ±π) the numerator and the
// denominator stay well-conditioned and atan2 answers the angle
// without the catastrophic cancellation the bare mixed product
// a·(b×c)/r³ suffers when opposite contributions nearly cancel. The
// bare mixed product is NOT used here — the stability is the point
// of the formula's existence.
//
// Returns Ω ∈ (−2π, 2π], signed by the triangle's orientation. All
// double — the sign path's law (H1/ADR-0034): no float anywhere
// between the query and the classification.
double solid_angle(const math::Vec3d& a, const math::Vec3d& b,
                   const math::Vec3d& c) {
    const double la = a.length();
    const double lb = b.length();
    const double lc = c.length();
    const double num = a.dot(b.cross(c));
    const double den = la * lb * lc + a.dot(b) * lc + a.dot(c) * lb +
                       b.dot(c) * la;
    return 2.0 * std::atan2(num, den);
}

// Kahan-compensated summation, one term at a time. Deterministic: the
// compensation is a pure function of the summation ORDER, and the
// order is the index buffer's own — same bytes in, same bytes out
// (ADR-0014). The compensation keeps the winding sum's rounding at
// O(u) instead of growing with the cancellation the signed angles can
// carry, which is part of what makes the band below honest at F's
// scale. The final value folds the last pending correction back in
// (sum − comp): comp holds the LAST addition's rounding, the one the
// loop had no next term to hand it to.
inline void kahan_add(double& sum, double& comp, double term) {
    const double y = term - comp;
    const double t = sum + y;
    comp = (t - sum) - y;
    sum = t;
}

// ---------------------------------------------------------------------------
// THE SIGN BAND — derived, not estimated (the ADR-0035 D2.4 vocabulary).
//
// The mathematical fact first: for a CLOSED, consistently oriented
// mesh the winding number is EXACTLY integer off the surface — 1
// inside a single outward shell, 0 outside, k inside k nested shells,
// −1 for a globally inverted one — and genuinely fractional only ON
// the surface. w is a step function: the transition does not widen
// with distance or curvature; only arithmetic noise moves the
// computed value off its integer.
//
// The derivation (first principles, the D2.3 way):
//   * per triangle, Ω = 2·atan2(N, D); N and D are each ~40 double
//     flops of the RELATIVE vertex vectors (9 sub, 9 mul for the
//     cross, 5 for the triple product, 3 square roots, 3 dots, 7 for
//     D — counted), so each carries a relative error ≤ γ₄₀,
//     γₘ = m·u/(1−m·u), u = 2⁻⁵³;
//   * atan2's angle error is bounded by the argument error
//     INDEPENDENT of the angle: δθ = (N·δD − D·δN)/(N²+D²), so
//     |δθ| ≤ γ₄₀ by AM–GM (2|N||D| ≤ N²+D²) — no shape, size or
//     distance term survives;
//   * so each solid angle is off by at most 2γ₄₀, and the Kahan sum
//     holds the summation itself at O(u) more;
//   * dividing by 4π:
//
//         ε_w(F) = 2·F·γ₄₀/(4π) + 16u  ≈  F·7.1·10⁻¹⁶
//
// F = 10⁶ triangles → ε_w ≈ 7·10⁻¹⁰. The band would need F > 7·10¹⁴
// triangles to reach the classification boundary at 0.5 — the
// classification below is total: no query can fall between the band
// and the integer undecided.
//
// The declared kind: DIMENSIONLESS (w counts how many times the
// surface winds around the query — a pure number, no length, no
// angle), the per-mesh derived band beside ADR-0035's determinant
// band 4·ε_L·L: same discipline — derived where it is used, from the
// mesh's own F, never a shared anonymous constant.
//
// THE TIE-BREAK (ADR-0035 Family 3 — deterministic, never a coin
// flip): |w − round(w)| > ε_w means the computed w is GENUINELY
// fractional — not arithmetic noise but geometry: the query sits ON
// the surface within the mesh's own float32 storage honesty (a
// float32 vertex is exact in double, but the surface it names is only
// ~10⁻⁷-relative honest). The global evidence is exhausted exactly
// there; the LOCAL evidence decides: the side of the NEAREST
// triangle, in double, from the same stored bytes. Deterministic —
// same bytes in, same answer out — and honest: at |distance| ≤ the
// storage noise, inside and outside are both wrong by less than the
// data can tell.
constexpr double kUnitRoundoff = 1.0 / 9007199254740992.0; // 2^-53
constexpr int kSolidAngleFlops = 40; // counted above
double sign_band(std::size_t triangle_count) {
    const double m = static_cast<double>(kSolidAngleFlops);
    const double gamma = m * kUnitRoundoff / (1.0 - m * kUnitRoundoff);
    return (2.0 * static_cast<double>(triangle_count) * gamma) / kFourPi +
           16.0 * kUnitRoundoff;
}

// ---------------------------------------------------------------------------
// The watertight validation — O(E), once, at load, never per query.
//
// A mesh carries a well-defined winding number iff its triangles form
// a closed 2-chain: every undirected edge appears the same number of
// times in both directions. For an ordinary manifold that is "every
// edge shared by exactly two oppositely-oriented triangles". Two
// named failure classes, both refused:
//   * FREE EDGE — an undirected edge with total count 1: it belongs
//     to a single triangle, it is the boundary of a hole;
//   * UNBALANCED EDGE — an undirected edge whose two directions carry
//     different counts (total ≥ 2): the triangles disagree about the
//     orientation, the chain is not a cycle, and the winding number
//     is not guaranteed integer anywhere.
// Degenerate index triples (a repeated vertex — zero area, zero solid
// angle) are skipped by the check and by the winding sum alike: they
// carry no surface, and a hole they would mask shows up as the
// neighbours' free edges.
//
// Determinism of the REPORT: the culprits are sorted by (lo, hi) and
// the rim triangle is the lowest index touching the edge — the same
// bytes name the same culprits on every compiler (the unordered_map
// is a counting device only; its iteration order never reaches the
// message).
constexpr std::size_t kMaxCulpritEdges = 4; // named in the error, kept readable

struct EdgeCulprit {
    std::uint32_t lo, hi;      // the edge's vertex indices, ordered
    std::uint32_t tri;         // a triangle touching it (the hole's rim)
};

struct EdgeAudit {
    std::size_t free_edges = 0;
    std::size_t unbalanced_edges = 0;
    std::vector<EdgeCulprit> free_culprits;       // sorted, ≤ kMaxCulpritEdges
    std::vector<EdgeCulprit> unbalanced_culprits; // sorted, ≤ kMaxCulpritEdges
};

EdgeAudit audit_edges(const sdf::TriangleMesh& mesh) {
    EdgeAudit audit;
    const std::size_t n = mesh.indices.size() / 3;

    // Pass 1: count the directed edges. Key: (min<<32)|max; the pair
    // holds (lo→hi count, hi→lo count).
    std::unordered_map<std::uint64_t, std::pair<int, int>> edges;
    edges.reserve(mesh.indices.size());
    for (std::size_t t = 0; t < n; ++t) {
        const std::uint32_t tri[3] = {mesh.indices[3 * t + 0],
                                      mesh.indices[3 * t + 1],
                                      mesh.indices[3 * t + 2]};
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0])
            continue; // degenerate: no surface, no edges
        for (int e = 0; e < 3; ++e) {
            const std::uint32_t a = tri[e];
            const std::uint32_t b = tri[(e + 1) % 3];
            const std::uint64_t lo = std::min(a, b);
            const std::uint64_t hi = std::max(a, b);
            auto& cnt = edges[(lo << 32) | hi];
            if (a < b) ++cnt.first;
            else       ++cnt.second;
        }
    }

    // Pass 2: classify. Culprit lists sorted by (lo, hi) — the
    // deterministic first-culprits, independent of hash order.
    std::vector<std::pair<std::uint64_t, bool>> bad; // key, is_free
    for (const auto& [key, cnt] : edges) {
        const int total = cnt.first + cnt.second;
        if (total < 2) {
            ++audit.free_edges;
            bad.emplace_back(key, true);
        } else if (cnt.first != cnt.second) {
            ++audit.unbalanced_edges;
            bad.emplace_back(key, false);
        }
    }
    std::sort(bad.begin(), bad.end());
    for (const auto& [key, is_free] : bad) {
        EdgeCulprit c{static_cast<std::uint32_t>(key >> 32),
                      static_cast<std::uint32_t>(key & 0xFFFFFFFFu), 0};
        if (is_free) {
            if (audit.free_culprits.size() < kMaxCulpritEdges)
                audit.free_culprits.push_back(c);
        } else {
            if (audit.unbalanced_culprits.size() < kMaxCulpritEdges)
                audit.unbalanced_culprits.push_back(c);
        }
    }

    // Pass 3 (only when there is something to name): find the rim
    // triangle — the LOWEST triangle index touching the culprit edge.
    // That triangle borders the hole; the error names it.
    const auto name_rim = [&mesh, n](EdgeCulprit& c) {
        for (std::size_t t = 0; t < n; ++t) {
            const std::uint32_t tri[3] = {mesh.indices[3 * t + 0],
                                          mesh.indices[3 * t + 1],
                                          mesh.indices[3 * t + 2]};
            if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0])
                continue;
            for (int e = 0; e < 3; ++e) {
                const std::uint32_t a = tri[e];
                const std::uint32_t b = tri[(e + 1) % 3];
                if ((std::min(a, b) == c.lo && std::max(a, b) == c.hi)) {
                    c.tri = static_cast<std::uint32_t>(t);
                    return;
                }
            }
        }
    };
    for (auto& c : audit.free_culprits) name_rim(c);
    for (auto& c : audit.unbalanced_culprits) name_rim(c);
    return audit;
}

[[noreturn]] void throw_not_watertight(const EdgeAudit& audit,
                                       std::size_t triangle_count) {
    std::ostringstream os;
    os << "mesh_to_sdf: the mesh is NOT WATERTIGHT - the generalized "
          "winding number is defined only on closed, consistently "
          "oriented meshes, and this mesh (" << triangle_count
       << " triangles) does not form a closed shell. What is open: "
       << audit.free_edges << " free edge(s) (edges belonging to "
          "exactly ONE triangle - they are the boundary of the hole)";
    if (audit.unbalanced_edges > 0)
        os << ", and " << audit.unbalanced_edges
           << " unbalanced edge(s) (their two directions carry "
              "different triangle counts - the orientation disagrees "
              "across them)";
    os << ". First culprits:";
    for (const auto& c : audit.free_culprits)
        os << " free edge (v" << c.lo << ", v" << c.hi << "), rim "
              "triangle " << c.tri << ";";
    for (const auto& c : audit.unbalanced_culprits)
        os << " unbalanced edge (v" << c.lo << ", v" << c.hi << "), "
              "touched by triangle " << c.tri << ";";
    os << " Remedy: fill the hole (the free edges are its rim) or fix "
          "the exporter / weld duplicated vertices so every edge is "
          "shared by exactly two triangles with opposite directions; "
          "or convert a recognized primitive (box, sphere, cylinder), "
          "which needs no mesh at all.";
    throw std::invalid_argument(os.str());
}

// ---------------------------------------------------------------------------
// MeshSDF — the mesh-based SDF node.
//
// PERFORMANCE, HONEST (morso 1's declared ceiling): each query is
// O(F) — one pass over the triangles for the winding sum and the
// nearest distance together. The case that drives this bridge today
// (hand-built test meshes, F in the hundreds) carries it fine, and
// the green comes FIRST. The hierarchical winding number over a BVH
// (with the bound on a cluster's solid-angle contribution DECLARED in
// the ADR-0035 taxonomy, as this band is) is MORSO 2 — registered
// here, on the record: never the optimization before the correctness.
class MeshSDF final : public sdf::SDFNode {
public:
    explicit MeshSDF(sdf::TriangleMesh mesh) : mesh_(std::move(mesh)) {
        const std::size_t n = mesh_.indices.size() / 3;
        triangles_.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::uint32_t i0 = mesh_.indices[3 * i + 0];
            const std::uint32_t i1 = mesh_.indices[3 * i + 1];
            const std::uint32_t i2 = mesh_.indices[3 * i + 2];
            if (i0 == i1 || i1 == i2 || i2 == i0)
                continue; // degenerate: no surface (the audit skips it too)
            Triangle tri;
            // The display boundary, one direction, declared: the
            // float32 positions enter the double world exactly (every
            // float32 is a double) — the sign path stores and computes
            // in double from here on.
            tri.v0 = to_d(mesh_.positions[i0]);
            tri.v1 = to_d(mesh_.positions[i1]);
            tri.v2 = to_d(mesh_.positions[i2]);
            // The outward normal in double — the tie-break's local
            // evidence and the answer's gradient.
            const math::Vec3d cross =
                (tri.v1 - tri.v0).cross(tri.v2 - tri.v0);
            const double len = cross.length();
            tri.normal = (len > 0.0) ? cross / len : math::Vec3d{0, 1, 0};
            triangles_.push_back(tri);
        }
        band_ = sign_band(triangles_.size());
    }

    sdf::SDFSample sample(const math::Vec3f& p) const noexcept override {
        if (triangles_.empty()) return {1e30f, {}};

        // The query enters double exactly, once — no float anywhere
        // in the sign path (the H2-u7 lesson made law).
        const math::Vec3d q = to_d(p);

        // One pass: the winding sum (Kahan) and the nearest triangle
        // (the distance + the tie-break's local evidence) together.
        double w_sum = 0.0, w_comp = 0.0;
        double best_d2 = std::numeric_limits<double>::infinity();
        const Triangle* best = nullptr;
        for (const Triangle& tri : triangles_) {
            kahan_add(w_sum, w_comp,
                      solid_angle(tri.v0 - q, tri.v1 - q, tri.v2 - q));
            const math::Vec3d closest = closest_point_on_triangle(q, tri);
            const double d2 = (closest - q).length_sq();
            if (d2 < best_d2) {
                best_d2 = d2;
                best = &tri;
            }
        }
        const double w = (w_sum - w_comp) / kFourPi;
        const double d_min = std::sqrt(best_d2);

        // THE CLASSIFICATION, WITH ITS BAND (see sign_band above).
        const double wr = std::round(w);
        bool inside;
        if (std::abs(w - wr) <= band_) {
            // The integer is confident: 0 outside, ±k inside (k
            // nested shells; the sign honors a globally inverted
            // mesh — its material is where w rounds to nonzero).
            inside = (wr != 0.0);
        } else {
            // Genuinely fractional: the query sits ON the surface
            // within the data's own honesty. The deterministic local
            // tie-break — the nearest triangle's side, in double.
            inside = (best->normal.dot(q - best->v0) < 0.0);
        }

        const double signed_d = inside ? -d_min : d_min;
        // The declared demotion at the answer: the SDF's own world is
        // float32 (the display boundary, ADR-0034 D1) — one cast,
        // here. The gradient is the OUTWARD normal of the nearest
        // triangle on BOTH sides (the field's own convention — the
        // primitives answer p̂ inside and out, and ∇f of a signed
        // distance points outward everywhere).
        return {static_cast<float>(signed_d), to_f(best->normal)};
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
    static math::Vec3d to_d(const math::Vec3f& v) {
        return math::Vec3d{static_cast<double>(v.x), static_cast<double>(v.y),
                           static_cast<double>(v.z)};
    }
    static math::Vec3f to_f(const math::Vec3d& v) {
        return math::Vec3f{static_cast<float>(v.x), static_cast<float>(v.y),
                           static_cast<float>(v.z)};
    }

    struct Triangle {
        math::Vec3d v0, v1, v2, normal;
    };

    // Point-to-triangle closest point (Ericson, Real-Time Collision
    // Detection, section 5.1.5) — in DOUBLE: at large model scale the
    // float version's `v0 + ab*v` re-quantizes the closest point onto
    // the float grid and the returned distance dies with it (the
    // far-from-origin test pins exactly this death on the old code).
    static math::Vec3d closest_point_on_triangle(const math::Vec3d& p,
                                                 const Triangle& tri) {
        const math::Vec3d ab = tri.v1 - tri.v0;
        const math::Vec3d ac = tri.v2 - tri.v0;
        const math::Vec3d ap = p - tri.v0;

        const double d1 = ab.dot(ap);
        const double d2 = ac.dot(ap);
        if (d1 <= 0.0 && d2 <= 0.0) return tri.v0;  // barycentric (1,0,0)

        const math::Vec3d bp = p - tri.v1;
        const double d3 = ab.dot(bp);
        const double d4 = ac.dot(bp);
        if (d3 >= 0.0 && d4 <= d3) return tri.v1;  // barycentric (0,1,0)

        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            const double v = d1 / (d1 - d3);
            return tri.v0 + ab * v;  // edge ab
        }

        const math::Vec3d cp = p - tri.v2;
        const double d5 = ab.dot(cp);
        const double d6 = ac.dot(cp);
        if (d6 >= 0.0 && d5 <= d6) return tri.v2;  // barycentric (0,0,1)

        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            const double w = d2 / (d2 - d6);
            return tri.v0 + ac * w;  // edge ac
        }

        const double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return tri.v1 + (tri.v2 - tri.v1) * w;  // edge bc
        }

        // Inside face region
        const double denom = 1.0 / (va + vb + vc);
        const double v = vb * denom;
        const double w = vc * denom;
        return tri.v0 + ab * v + ac * w;
    }

    sdf::TriangleMesh mesh_;
    std::vector<Triangle> triangles_;
    double band_{0.0};  // the sign band, derived from this mesh's F
};

} // namespace

std::unique_ptr<sdf::SDFBody> brep_to_sdf(const brep::BRepBody& body) {
    // Detect known primitive types
    const auto& faces = body.solid.shell().faces();
    if (faces.empty()) return std::make_unique<sdf::SDFBody>();

    // Check if it's a sphere (1 face of type "sphere")
    if (faces.size() == 1) {
        const auto* surf = faces[0].surface();
        // H1 (ADR-0034 D3): the typed dispatch — the family enum, the
        // visitor for the payload; the SDF consumes the double world
        // through the declared double→float boundary.
        if (surf && surf->family() == brep::SurfaceFamily::Sphere) {
            struct SphereReader : brep::SurfaceVisitor {
                const brep::SphereSurface* found{nullptr};
                void visit(const brep::SphereSurface& s) override {
                    found = &s;
                }
            } reader;
            surf->accept(reader);
            if (reader.found) {
                return std::make_unique<sdf::SDFBody>(
                    sdf::make_sphere(static_cast<float>(
                        reader.found->radius())));
            }
        }
    }

    // Check if it's a cylinder (3 faces: cylinder + 2 planes)
    if (faces.size() == 3) {
        const auto* surf = faces[0].surface();
        if (surf && surf->family() == brep::SurfaceFamily::Cylinder) {
            struct CylReader : brep::SurfaceVisitor {
                const brep::CylinderSurface* found{nullptr};
                void visit(const brep::CylinderSurface& s) override {
                    found = &s;
                }
            } reader;
            surf->accept(reader);
            if (reader.found) {
                return std::make_unique<sdf::SDFBody>(sdf::make_cylinder(
                    static_cast<float>(reader.found->radius()),
                    static_cast<float>(reader.found->height())));
            }
        }
    }

    // Check if it's a box (6 plane faces, 8 vertices, 12 edges)
    if (faces.size() == 6 && body.vertex_count() == 8 && body.edge_count() == 12) {
        // Compute half-extents from vertex positions
        // THE DISPLAY BOUNDARY (ADR-0034): the double bounds read
        // into the SDF's float world, one direction, here.
        auto b = body.bounds();
        math::Vec3f half = {static_cast<float>((b.max.x - b.min.x) * 0.5),
                            static_cast<float>((b.max.y - b.min.y) * 0.5),
                            static_cast<float>((b.max.z - b.min.z) * 0.5)};
        return std::make_unique<sdf::SDFBody>(sdf::make_box(half));
    }

    // Fall back to the mesh bridge — which is WATERTIGHT ONLY
    // (H2-u4): tessellate_body's per-face grids are NOT welded into a
    // closed shell (each face tessellates independently over its own
    // parametric domain), so mesh_to_sdf's validation refuses them BY
    // NAME: the free edges, the rim triangles, the remedy. The old
    // fallback answered these bodies with a majority-vote sign that
    // misread moderate outside distances as inside (the ADR-0026
    // non-goals record said so on the page); the honest refusal
    // replaces the quiet wrong answer. A watertight tessellator is
    // the registered follow-up on the other side of this contract.
    brep::TessellationOptions opts;
    opts.resolution = 16;
    auto mesh = brep::tessellate_body(body, opts);
    return mesh_to_sdf(mesh);
}

std::unique_ptr<sdf::SDFBody> mesh_to_sdf(const sdf::TriangleMesh& mesh) {
    if (mesh.positions.empty()) return std::make_unique<sdf::SDFBody>();
    // THE WATERTIGHT CONTRACT, enforced at load — O(E), once, never
    // per query. The refusal names everything (see
    // throw_not_watertight).
    const EdgeAudit audit = audit_edges(mesh);
    if (audit.free_edges > 0 || audit.unbalanced_edges > 0)
        throw_not_watertight(audit, mesh.indices.size() / 3);
    // Clone the mesh into the SDF node
    sdf::TriangleMesh mesh_copy = mesh;
    return std::make_unique<sdf::SDFBody>(
        std::make_unique<MeshSDF>(std::move(mesh_copy)));
}

} // namespace CAD_0::bridge
