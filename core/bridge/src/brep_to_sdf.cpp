// core/bridge/src/brep_to_sdf.cpp
//
// H2-u4 MORSO 1 — THE SIGN OF THE BRIDGE: the generalized winding
// number, the honest refusal of open meshes, the discrimination.
//
// H2-u4 MORSO 2 — THE BVH OF THE WINDING NUMBER: the hierarchical
// approximation with its bound declared. Morso 1 answered every query
// in O(F) — honest, green, and a wall at grid scale (Q queries on an
// F-triangle mesh is O(F·Q), and Q grows with the detail). This morso
// abates the wall the only way this repo tolerates an approximation:
// with its bound DERIVED and DECLARED (ε_w, ADR-0035 D2.5), the
// traversal obeying the bound — never a distance felt by sentiment —
// and the classification composed honestly with morso 1's sign band.
// The exact O(F) evaluation survives, verbatim, as the reference the
// tests measure the bound against (and the counterproof: budget zero
// answers with the exact machine, no approximation anywhere).
//
// What was here before morso 1 (the class named, for the record): a
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

#include "CAD_0/bridge/detail/winding_test_hooks.hpp"

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
// order is fixed — the index buffer's own for the exact path, the
// tree's own for the hierarchical path — same bytes in, same bytes
// out (ADR-0014). The compensation keeps the winding sum's rounding at
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
//         ε_w,noise(F) = 2·F·γ₄₀/(4π) + 16u  ≈  F·7.1·10⁻¹⁶
//
// F = 10⁶ triangles → ε_w,noise ≈ 7·10⁻¹⁰. The band would need
// F > 7·10¹⁴ triangles to reach the classification boundary at 0.5 —
// the classification below is total: no query can fall between the
// band and the integer undecided.
//
// The declared kind: DIMENSIONLESS (w counts how many times the
// surface winds around the query — a pure number, no length, no
// angle), the per-mesh derived band beside ADR-0035's determinant
// band 4·ε_L·L: same discipline — derived where it is used, from the
// mesh's own F, never a shared anonymous constant. (The symbol was
// ε_w(F) at morso 1; D2.5 renamed it ε_w,noise(F) — one symbol per
// meaning, the ε_w name now belongs to the winding family's
// approximation budget.)
//
// THE TIE-BREAK (ADR-0035 Family 3 — deterministic, never a coin
// flip): |w − round(w)| > the composed band means the computed w is
// GENUINELY fractional — not arithmetic noise but geometry: the query
// sits ON the surface within the mesh's own float32 storage honesty
// (a float32 vertex is exact in double, but the surface it names is
// only ~10⁻⁷-relative honest). The global evidence is exhausted exactly
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
// THE WINDING APPROXIMATION BUDGET ε_w — declared (ADR-0035 D2.5).
//
// The hierarchical traversal below may approximate a far node's solid
// angle sum with its dipole proxy, and every such approximation is
// BOUNDED (ε_node, derived at the traversal). The tolerance has TWO
// members, both derived from the classification's own geometry — the
// boundary sits at |w − round(w)| = ½, and the discipline is
// DECIMAL ORDERS of the half-gap, one per role:
//
//   * ε_w = ½/100 = 5·10⁻³ — the PER-NODE threshold: a node may be
//     proxied at all only when its own derived bound is under one
//     HUNDREDTH of the half-gap. The role, measured: the threshold
//     is the room's allocator — at ½/100 a single approximation is
//     confined two decimal orders below the classification boundary
//     (and an order below the tie-break ladder's coarsest fractional
//     evidence, ⅛ = 0.125, a 25× margin), and the accumulation room
//     flows to the MANY far nodes whose bounds are orders smaller
//     instead of being eaten by a few mid-field clusters.
//   * the ACCUMULATION CAP 9/10·½ = 0.45 — the composition's ceiling:
//     the sum of the bounds of every proxied node stops one tenth
//     short of the boundary, leaving that tenth to the noise band
//     ε_w,noise(F) and the tie-break's fractional evidence. The
//     composed classification band ε_w,noise + B_accum never reaches
//     ½, so off the surface the rounding is total (both paths round
//     to the same integer — the classification invariance, DERIVED);
//     on a face (w = ½) the fractional evidence survives the
//     composed band (½ − 0.45 − ε_w,noise > 0) and the
//     deterministic tie-break fires, as in morso 1.
//
// The traversal rule, verbatim: approximate a node SE E SOLO SE
// ε_node ≤ ε_w AND ε_node ≤ (cap − B_accum) — the bound decides, the
// code obeys; when the accumulation can eat the margin, the
// traversal DESCENDS, it never decides by chance.
constexpr double kWindingBudget = 0.005;       // ε_w = ½/100, per node
constexpr double kWindingAccumulationCap = 0.45; // 9/10·½, composed

// The proxy's own arithmetic, counted for its rounding budget: the
// far-node dipole below is 3 sub (g) + 5 (d²) + 1 (sqrt) + 3 (d³, 1/)
// + 5 (moment·g) + 5 (Kahan) = 22 double flops → relative error
// γ₂₂ on a term of magnitude ≤ A/(d−r)², folded INTO ε_node so the
// accumulated bound carries the approximation AND the proxy's own
// rounding — nothing about a proxied node is left unbudgeted.
constexpr int kProxyFlops = 22;

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

// The display boundary's two directions, named once (ADR-0034 D1):
// float32→double is EXACT (every float32 is a double) — the sign
// path's input; double→float32 is the answer's declared demotion.
math::Vec3d to_double(const math::Vec3f& v) {
    return math::Vec3d{static_cast<double>(v.x), static_cast<double>(v.y),
                       static_cast<double>(v.z)};
}
math::Vec3f to_float(const math::Vec3d& v) {
    return math::Vec3f{static_cast<float>(v.x), static_cast<float>(v.y),
                       static_cast<float>(v.z)};
}

// ---------------------------------------------------------------------------
// The triangle store, shared by the exact reference and the hierarchy.
//
// The display boundary, one direction, declared: the float32
// positions enter the double world exactly (every float32 is a
// double) — the sign path stores and computes in double from here on.
// Degenerate index triples are skipped (no surface — the audit skips
// them too).
struct Triangle {
    math::Vec3d v0, v1, v2, normal;
};

std::vector<Triangle> build_triangles(const sdf::TriangleMesh& mesh) {
    std::vector<Triangle> triangles;
    const std::size_t n = mesh.indices.size() / 3;
    triangles.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint32_t i0 = mesh.indices[3 * i + 0];
        const std::uint32_t i1 = mesh.indices[3 * i + 1];
        const std::uint32_t i2 = mesh.indices[3 * i + 2];
        if (i0 == i1 || i1 == i2 || i2 == i0)
            continue; // degenerate: no surface (the audit skips it too)
        Triangle tri;
        tri.v0 = to_double(mesh.positions[i0]);
        tri.v1 = to_double(mesh.positions[i1]);
        tri.v2 = to_double(mesh.positions[i2]);
        // The outward normal in double — the tie-break's local
        // evidence and the answer's gradient.
        const math::Vec3d cross =
            (tri.v1 - tri.v0).cross(tri.v2 - tri.v0);
        const double len = cross.length();
        tri.normal = (len > 0.0) ? cross / len : math::Vec3d{0, 1, 0};
        triangles.push_back(tri);
    }
    return triangles;
}

// Point-to-triangle closest point (Ericson, Real-Time Collision
// Detection, section 5.1.5) — in DOUBLE: at large model scale the
// float version's `v0 + ab*v` re-quantizes the closest point onto
// the float grid and the returned distance dies with it (the
// far-from-origin test pins exactly this death on the old code).
math::Vec3d closest_point_on_triangle(const math::Vec3d& p,
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

// ---------------------------------------------------------------------------
// THE AABB TREE OF THE WINDING NUMBER (morso 2, B1) — deterministic.
//
// Structure: a binary AABB tree over the triangles, built ONCE per
// mesh at load (after the watertight check — morso 1's order
// intact). The split is the TRUE MEDIAN with a FIXED tie-break by
// triangle index: each node sorts its range by
// (centroid[split axis], original triangle index) — a TOTAL order —
// and halves it. std::sort over a total order yields the same
// arrangement on every standard library (GCC's and Clang's alike),
// so two builds of the same mesh give the IDENTICAL tree, byte for
// byte (ADR-0014). No nth_element: its arrangement of EQUAL keys is
// unspecified and compiler-dependent.
//
// Per node, the cluster's aggregate (all double, all deterministic):
//   * the AABB — exact min/max over the cluster's vertices, the
//     exact-distance pruning's tool;
//   * the bounding sphere (center, radius): center = the
//     area-weighted triangle centroid, radius = max |vertex − center|
//     (the sphere contains every triangle: a triangle lies in the
//     convex hull of its vertices, and balls are convex);
//   * the cluster's area A and its VECTOR AREA (the dipole moment)
//     m = Σ Aᵢ·n̂ᵢ = ½·Σ (v₁−v₀)×(v₂−v₀) — the far-field proxy's
//     data. A closed cluster has m = 0 (∮ n̂ dA = 0): its proxy is
//     EXACTLY zero outside — the dipole is the right first moment
//     for this field, and closed sub-shells come out exact.
constexpr std::size_t kLeafTriangles = 32;

struct BvhNode {
    math::Vec3d box_min, box_max; // the cluster's AABB
    math::Vec3d center;           // bounding-sphere center (area-weighted)
    double radius = 0.0;          // bounding-sphere radius (vertices)
    double area = 0.0;            // Σ Aᵢ over the cluster
    math::Vec3d moment;           // Σ Aᵢ·n̂ᵢ (the dipole)
    std::uint32_t left = 0, right = 0;        // internal children
    std::uint32_t tri_begin = 0, tri_end = 0; // leaf range into order
    bool is_leaf = false;
};

// What one traversal answers: the (possibly approximated) winding
// value w = ΣΩ/4π, the accumulated bound B_accum = Σ ε_node over the
// approximated nodes, and the nearest triangle with its squared
// distance — exact: distance pruning never drops a candidate.
struct TraverseResult {
    double w = 0.0;
    double bound = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    const Triangle* best = nullptr;
};

class WindingBVH {
public:
    WindingBVH() = default; // the empty tree (the empty mesh)

    // Builds the hierarchy over `order` — a permutation of the
    // triangle indices. The build is a PURE function of the triangle
    // bytes: same bytes in, same tree out, on every compiler.
    WindingBVH(const std::vector<Triangle>& triangles) {
        const std::size_t n = triangles.size();
        order_.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            order_[i] = static_cast<std::uint32_t>(i);

        // The sort keys, precomputed once: the triangle centroids.
        centroids_.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            centroids_[i] = (triangles[i].v0 + triangles[i].v1 +
                             triangles[i].v2) /
                            3.0;

        nodes_.reserve(2 * (n / kLeafTriangles + 1) + 1);
        build(triangles, 0, n);
        centroids_.clear();
        centroids_.shrink_to_fit();
    }

    // THE QUERY (B1/B2) — one traversal, two jobs, both bounded:
    //
    //   * the WINDING SUM: at each node, the traversal decides with
    //     the BOUND, never with a felt distance — approximate the
    //     node with its dipole proxy iff ε_node ≤ the REMAINING
    //     budget (ε_node derived below); once a node is proxied its
    //     subtree is closed for winding. The accumulated bound never
    //     exceeds the caller's budget: a node whose bound the budget
    //     cannot carry is DESCENDED, not approximated by hope.
    //   * the NEAREST TRIANGLE: exact, always. A node whose AABB is
    //     farther than the best distance so far cannot contain the
    //     nearest triangle and is pruned — pruning never changes the
    //     argmin's value, so the distance the SDF answers is the
    //     exact morso 1 distance, bit for bit.
    //
    // Determinism (ADR-0014): the tree is fixed, the child order is
    // fixed (near-first, ties to the left), the arithmetic is
    // contraction-free (-ffp-contract=off) double — same bytes in,
    // same bytes out. The query is const and allocates nothing:
    // ready for ADR-0009's grid parallelism the day that ADR
    // declares it; today the shipped world stays serial (the
    // ADR-0009 amendment: everything runs the single-threaded
    // fallback), and no parallelism is added here in secret.
    TraverseResult traverse(const std::vector<Triangle>& triangles,
                            const math::Vec3d& q, double budget) const {
        TraverseResult r;
        if (nodes_.empty()) return r;
        State s{triangles, q, budget, kWindingAccumulationCap, 0.0, 0.0,
                std::numeric_limits<double>::infinity(), nullptr};
        visit(0, true, 0.0, s); // the root is never distance-pruned
        r.w = (s.w_sum - s.w_comp) / kFourPi;
        r.bound = kWindingAccumulationCap - s.remaining;
        r.best_d2 = s.best_d2;
        r.best = s.best;
        return r;
    }

private:
    struct State {
        const std::vector<Triangle>& triangles;
        math::Vec3d q;
        double threshold; // ε_w: the per-node threshold (the caller's)
        double remaining; // the accumulation cap minus B_accum
        double w_sum, w_comp; // Kahan over the contributions
        double best_d2;
        const Triangle* best;
    };

    static double axis_coord(const math::Vec3d& v, int axis) {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }

    // ε_node — the node's total approximation bound, DERIVED
    // (ADR-0035 D2.5), in WINDING units (w = ΣΩ/4π, the
    // classification's own world, where the budget ε_w lives). The
    // exact cluster sum and the dipole proxy both integrate the same
    // point-charge field G_x(y) = (y−x)/|y−x|³ over the cluster's
    // triangles (Ω = ∫ n̂·G_x dA — the flux form of the van
    // Oosterom–Strackee angle; the identity is mathematical, the
    // atan2 is its stable evaluation). The proxy evaluates the field
    // ONCE, at the sphere center c; the error is
    //   |∫ n̂·(G_x(y) − G_x(c)) dA| ≤ A·max_{y∈cluster}|G_x(y) − G_x(c)|.
    // On the ball B(c, r) the field's Jacobian is bounded:
    //   |∂Gᵢ/∂yⱼ| = |δᵢⱼ/r̃³ − 3rᵢrⱼ/r̃⁵| ≤ 4/r̃³,  r̃ = |y−x| ≥ d−r,
    // (row-sum; the segment [c, y] stays inside the ball, outside the
    // singularity), so |G_x(y) − G_x(c)| ≤ 4r/(d−r)³ and
    //
    //     ε_node^Ω,approx = 4·A·r/(d−r)³
    //
    // — which reads as the solid angle the bounding sphere subtends
    // (Ω_B = 2π(1−√(1−(r/d)²)) ≥ πr²/d²) scaled by the cluster's own
    // shape and proximity:
    //     ε_node^Ω,approx = Ω_B(x) · (4A/(πrd)) · (1−r/d)⁻³.
    // In winding units (÷4π), with the proxy's own rounding folded
    // in (γ₂₂ on a term ≤ A/(d−r)², so the bound carries everything
    // about the proxied node):
    //
    //     ε_node = (A/4π) · ( 4r/(d−r)³ + γ₂₂/(d−r)² )
    //
    // d ≤ r (the query inside the node's bounding sphere) means the
    // bound is vacuous: the node is never approximated there.
    static double node_bound(const BvhNode& n, double d) {
        const double gap = d - n.radius;
        if (gap <= 0.0)
            return std::numeric_limits<double>::infinity();
        const double gamma = static_cast<double>(kProxyFlops) *
                             kUnitRoundoff /
                             (1.0 - static_cast<double>(kProxyFlops) *
                                        kUnitRoundoff);
        return (n.area / kFourPi) *
               (4.0 * n.radius / (gap * gap * gap) + gamma / (gap * gap));
    }

    static double box_dist_sq(const BvhNode& n, const math::Vec3d& q) {
        const double dx = std::max(std::max(n.box_min.x - q.x, 0.0),
                                   q.x - n.box_max.x);
        const double dy = std::max(std::max(n.box_min.y - q.y, 0.0),
                                   q.y - n.box_max.y);
        const double dz = std::max(std::max(n.box_min.z - q.z, 0.0),
                                   q.z - n.box_max.z);
        return dx * dx + dy * dy + dz * dz;
    }

    void visit(std::uint32_t idx, bool winding_open, double box_d2,
               State& s) const {
        const BvhNode& n = nodes_[idx];

        // The winding decision — the bound decides, the code obeys:
        // per-node threshold AND the accumulation's room. A node
        // whose bound fails either is descended, never hoped.
        if (winding_open && s.remaining > 0.0) {
            const math::Vec3d g = n.center - s.q;
            const double d = g.length();
            if (d > n.radius) {
                const double eps = node_bound(n, d);
                if (eps <= s.threshold && eps <= s.remaining) {
                    // THE PROXY: the cluster's dipole — its vector
                    // area dotted with the point-charge field at the
                    // sphere center: Ω_proxy = m·(c−q)/|c−q|³. The
                    // far contribution compresses into its compact
                    // sum (the classic hierarchical winding number
                    // form); a closed cluster's moment is zero and
                    // its proxy is exactly zero outside.
                    const double d2 = g.length_sq();
                    const double inv_d3 = 1.0 / (d2 * d);
                    kahan_add(s.w_sum, s.w_comp, n.moment.dot(g) * inv_d3);
                    s.remaining -= eps;
                    winding_open = false;
                }
            }
        }

        // The distance decision — exact pruning, never a lost argmin.
        // box_d2 arrives from the parent (boxes nest: the parent's
        // distance is a valid lower bound for the child's, and a
        // pruned parent's subtree is pruned whole).
        const bool dist_pruned = box_d2 >= s.best_d2;
        if (!winding_open && dist_pruned) return;

        if (n.is_leaf) {
            for (std::uint32_t k = n.tri_begin; k < n.tri_end; ++k) {
                const Triangle& t =
                    s.triangles[order_[k]];
                if (winding_open)
                    kahan_add(s.w_sum, s.w_comp,
                              solid_angle(t.v0 - s.q, t.v1 - s.q,
                                          t.v2 - s.q));
                if (!dist_pruned) {
                    const math::Vec3d closest =
                        closest_point_on_triangle(s.q, t);
                    const double d2 = (closest - s.q).length_sq();
                    if (d2 < s.best_d2) {
                        s.best_d2 = d2;
                        s.best = &t;
                    }
                }
            }
            return;
        }

        if (dist_pruned) {
            // Both children serve the winding only (the distance is
            // already pruned for the whole subtree): the order is
            // fixed left-first, and the parent's box_d2 — a valid
            // lower bound for theirs — passes down, so no child
            // recomputes what the pruning already decided.
            visit(n.left, winding_open, box_d2, s);
            visit(n.right, winding_open, box_d2, s);
            return;
        }

        // Children, near-first (deterministic: the smaller box
        // distance first, ties to the left) — the tighter best_d2
        // prunes the far sibling harder. The distances computed
        // here are the ones the children would compute themselves:
        // they pass down instead.
        const double dl = box_dist_sq(nodes_[n.left], s.q);
        const double dr = box_dist_sq(nodes_[n.right], s.q);
        if (dl <= dr) {
            visit(n.left, winding_open, dl, s);
            visit(n.right, winding_open, dr, s);
        } else {
            visit(n.right, winding_open, dr, s);
            visit(n.left, winding_open, dl, s);
        }
    }

    std::uint32_t build(const std::vector<Triangle>& triangles,
                        std::size_t begin, std::size_t end) {
        const std::uint32_t idx = static_cast<std::uint32_t>(nodes_.size());
        nodes_.emplace_back();
        BvhNode& node = nodes_[idx];

        // The aggregate over order_[begin, end) — index order within
        // the range, deterministic.
        math::Vec3d bmin{std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity()};
        math::Vec3d bmax{-std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()};
        double area = 0.0;
        math::Vec3d moment{0, 0, 0};
        math::Vec3d area_centroid{0, 0, 0};
        for (std::size_t k = begin; k < end; ++k) {
            const Triangle& t = triangles[order_[k]];
            bmin = math::Vec3d{std::min(bmin.x, std::min(t.v0.x,
                                 std::min(t.v1.x, t.v2.x))),
                               std::min(bmin.y, std::min(t.v0.y,
                                 std::min(t.v1.y, t.v2.y))),
                               std::min(bmin.z, std::min(t.v0.z,
                                 std::min(t.v1.z, t.v2.z)))};
            bmax = math::Vec3d{std::max(bmax.x, std::max(t.v0.x,
                                 std::max(t.v1.x, t.v2.x))),
                               std::max(bmax.y, std::max(t.v0.y,
                                 std::max(t.v1.y, t.v2.y))),
                               std::max(bmax.z, std::max(t.v0.z,
                                 std::max(t.v1.z, t.v2.z)))};
            const math::Vec3d cr = (t.v1 - t.v0).cross(t.v2 - t.v0);
            const double a = 0.5 * cr.length(); // the triangle's area
            area += a;
            moment = moment + cr * 0.5;         // Σ Aᵢ·n̂ᵢ (the dipole)
            area_centroid = area_centroid + centroids_[order_[k]] * a;
        }
        node.box_min = bmin;
        node.box_max = bmax;
        node.area = area;
        node.moment = moment;
        node.center = (area > 0.0) ? area_centroid / area
                                   : math::Vec3d{0, 0, 0};
        node.radius = 0.0;
        for (std::size_t k = begin; k < end; ++k) {
            const Triangle& t = triangles[order_[k]];
            node.radius = std::max(
                node.radius,
                std::max((t.v0 - node.center).length(),
                         std::max((t.v1 - node.center).length(),
                                  (t.v2 - node.center).length())));
        }

        if (end - begin <= kLeafTriangles) {
            node.is_leaf = true;
            node.tri_begin = static_cast<std::uint32_t>(begin);
            node.tri_end = static_cast<std::uint32_t>(end);
            return idx;
        }

        // The split axis: the AABB's largest extent, ties to the
        // lowest axis index — deterministic.
        const math::Vec3d ext = bmax - bmin;
        int axis = 0;
        if (ext.y > ext[axis]) axis = 1;
        if (ext.z > ext[axis]) axis = 2;

        // The TRUE median with the FIXED tie-break: a total order on
        // (centroid[axis], original index) — the sorted arrangement
        // is unique, on every standard library, so the tree is
        // byte-identical across compilers (ADR-0014).
        std::sort(order_.begin() + static_cast<std::ptrdiff_t>(begin),
                  order_.begin() + static_cast<std::ptrdiff_t>(end),
                  [this, axis](std::uint32_t a, std::uint32_t b) {
                      const double ca = axis_coord(
                          centroids_[a], axis);
                      const double cb = axis_coord(
                          centroids_[b], axis);
                      if (ca != cb) return ca < cb;
                      return a < b;
                  });

        const std::size_t mid = begin + (end - begin) / 2;
        // The children are built AFTER the node's slot exists — the
        // reference `node` is invalidated by the recursion's
        // push_backs, so re-index.
        const std::uint32_t l =
            build(triangles, begin, mid);
        const std::uint32_t r =
            build(triangles, mid, end);
        nodes_[idx].left = l;
        nodes_[idx].right = r;
        return idx;
    }

    std::vector<BvhNode> nodes_;
    std::vector<std::uint32_t> order_;     // the sorted triangle permutation
    std::vector<math::Vec3d> centroids_;   // the sort keys (build-time only)
};

// ---------------------------------------------------------------------------
// The classification and the answer — one implementation for the
// exact path and the hierarchy, the composed band the only difference.
//
// THE COMPOSITION (B2, derived — ADR-0035 D2.5): morso 1 classifies
// with the arithmetic band ε_w,noise(F); the hierarchical path's
// winding value carries additionally the accumulated approximation
// bound B_accum (≤ ε_w, the traversal's own accounting). The composed
// classification band is the honest sum:
//
//     ε_class = ε_w,noise(F) + B_accum
//
// Off the surface w is exactly integer (morso 1's fact), so both
// paths round to the same integer whenever ε_w,noise + ε_w < ½ —
// 0.051 ≪ 0.5, a 10× margin: THE CLASSIFICATION IS IDENTICAL to the
// exact one, derived, not hoped. ON the surface (w = ½ on a face, ¼
// on an edge, ⅛ at a corner) the fractional evidence survives the
// composed band (½ > 2·(ε_w,noise + ε_w) with a 4× margin) and the
// deterministic local tie-break decides — the same nearest-triangle
// side, in double, as morso 1.
sdf::SDFSample finish_sample(double w, double bound, double best_d2,
                             const Triangle* best, const math::Vec3d& q,
                             double noise_band) {
    const double band = noise_band + bound; // the composed band
    const double wr = std::round(w);
    bool inside;
    if (std::abs(w - wr) <= band) {
        // The integer is confident: 0 outside, ±k inside (k nested
        // shells; the sign honors a globally inverted mesh — its
        // material is where w rounds to nonzero).
        inside = (wr != 0.0);
    } else {
        // Genuinely fractional beyond the composed band: the query
        // sits ON the surface within the data's own honesty. The
        // deterministic local tie-break — the nearest triangle's
        // side, in double.
        inside = (best->normal.dot(q - best->v0) < 0.0);
    }

    const double d_min = std::sqrt(best_d2);
    const double signed_d = inside ? -d_min : d_min;
    // The declared demotion at the answer: the SDF's own world is
    // float32 (the display boundary, ADR-0034 D1) — one cast, here.
    // The gradient is the OUTWARD normal of the nearest triangle on
    // BOTH sides (the field's own convention — the primitives answer
    // p̂ inside and out, and ∇f of a signed distance points outward
    // everywhere).
    return {static_cast<float>(signed_d), to_float(best->normal)};
}

// Morso 1's exact evaluation, VERBATIM — one pass, index order, the
// winding sum (Kahan) and the nearest triangle together. Kept as the
// reference the corpus measures the bound against, the speed number's
// baseline, and the counterproof's ground truth.
sdf::SDFSample exact_sample_impl(const std::vector<Triangle>& triangles,
                                 const math::Vec3d& q, double band) {
    double w_sum = 0.0, w_comp = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    const Triangle* best = nullptr;
    for (const Triangle& tri : triangles) {
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
    return finish_sample(w, 0.0, best_d2, best, q, band);
}

// ---------------------------------------------------------------------------
// MeshSDF — the mesh-based SDF node.
//
// PERFORMANCE, THE MORSO 2 LEDGER: morso 1's ceiling was O(F) per
// query, declared and green. The hierarchy above lifts it: the query
// is now O(log F + visited) — the winding sum through the ε_w-bounded
// proxy chain, the nearest triangle through exact AABB pruning. The
// exact O(F) evaluation survives as the test seam's reference; the
// discrimination (the bound proven point by point, the classification
// invariant, the determinism byte-identical, the counterproof at
// budget zero) lives in tests/unit/bridge/test_h2_u4_m2_bvh.cpp.
class MeshSDF final : public sdf::SDFNode {
public:
    explicit MeshSDF(sdf::TriangleMesh mesh) : mesh_(std::move(mesh)) {
        triangles_ = build_triangles(mesh_);
        band_ = sign_band(triangles_.size());
        if (!triangles_.empty())
            bvh_ = WindingBVH(triangles_);
    }

    sdf::SDFSample sample(const math::Vec3f& p) const noexcept override {
        if (triangles_.empty()) return {1e30f, {}};

        // The query enters double exactly, once — no float anywhere
        // in the sign path (the H2-u7 lesson made law).
        const math::Vec3d q = to_double(p);
        const TraverseResult r =
            bvh_.traverse(triangles_, q, kWindingBudget);
        return finish_sample(r.w, r.bound, r.best_d2, r.best, q, band_);
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
    sdf::TriangleMesh mesh_;
    std::vector<Triangle> triangles_;
    WindingBVH bvh_;
    double band_{0.0};  // the sign band ε_w,noise(F), derived from this mesh's F
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
    // per query, and BEFORE the hierarchy's build (the morso 1 order
    // intact: nothing the BVH needs exists past a refused mesh). The
    // refusal names everything (see throw_not_watertight).
    const EdgeAudit audit = audit_edges(mesh);
    if (audit.free_edges > 0 || audit.unbalanced_edges > 0)
        throw_not_watertight(audit, mesh.indices.size() / 3);
    // Clone the mesh into the SDF node
    sdf::TriangleMesh mesh_copy = mesh;
    return std::make_unique<sdf::SDFBody>(
        std::make_unique<MeshSDF>(std::move(mesh_copy)));
}

// ---------------------------------------------------------------------------
// The test seam (detail/winding_test_hooks.hpp) — the same machinery,
// the budget from the caller. ONE implementation: the seam forwards
// to the internals above, it never re-implements them.
struct WindingBVHForTests::Impl {
    std::vector<Triangle> triangles; // the exact reference's store
    WindingBVH bvh;                  // empty mesh → an empty tree
    double band = 0.0;               // ε_w,noise(F)
};

WindingBVHForTests::WindingBVHForTests(const sdf::TriangleMesh& mesh)
    : impl_(std::make_unique<Impl>()) {
    // The SAME contract as mesh_to_sdf, both doors: audit first, the
    // actionable refusal, then the build.
    if (mesh.positions.empty()) {
        impl_->bvh = WindingBVH(impl_->triangles);
        return;
    }
    const EdgeAudit audit = audit_edges(mesh);
    if (audit.free_edges > 0 || audit.unbalanced_edges > 0)
        throw_not_watertight(audit, mesh.indices.size() / 3);
    impl_->triangles = build_triangles(mesh);
    impl_->band = sign_band(impl_->triangles.size());
    impl_->bvh = WindingBVH(impl_->triangles);
}

WindingBVHForTests::WindingBVHForTests(WindingBVHForTests&&) noexcept =
    default;
WindingBVHForTests& WindingBVHForTests::operator=(
    WindingBVHForTests&&) noexcept = default;
WindingBVHForTests::~WindingBVHForTests() = default;

WindingQueryResult WindingBVHForTests::winding(const math::Vec3d& q,
                                               double threshold) const {
    const TraverseResult r =
        impl_->bvh.traverse(impl_->triangles, q, threshold);
    return {r.w, r.bound};
}

sdf::SDFSample WindingBVHForTests::sample(const math::Vec3f& q,
                                          double threshold) const {
    if (impl_->triangles.empty()) return {1e30f, {}};
    const math::Vec3d qd = to_double(q);
    const TraverseResult r =
        impl_->bvh.traverse(impl_->triangles, qd, threshold);
    return finish_sample(r.w, r.bound, r.best_d2, r.best, qd, impl_->band);
}

sdf::SDFSample WindingBVHForTests::exact_sample(
    const math::Vec3f& q) const {
    if (impl_->triangles.empty()) return {1e30f, {}};
    return exact_sample_impl(impl_->triangles, to_double(q), impl_->band);
}

double WindingBVHForTests::exact_winding(const math::Vec3d& q) const {
    double w_sum = 0.0, w_comp = 0.0;
    for (const Triangle& tri : impl_->triangles)
        kahan_add(w_sum, w_comp,
                  solid_angle(tri.v0 - q, tri.v1 - q, tri.v2 - q));
    return (w_sum - w_comp) / kFourPi;
}

std::size_t WindingBVHForTests::triangle_count() const {
    return impl_->triangles.size();
}

double winding_sign_band_for_tests(std::size_t triangle_count) {
    return sign_band(triangle_count);
}

double winding_budget_for_tests() { return kWindingBudget; }

double winding_accumulation_cap_for_tests() {
    return kWindingAccumulationCap;
}

} // namespace CAD_0::bridge
