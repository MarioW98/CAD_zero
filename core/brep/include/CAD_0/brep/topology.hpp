// core/brep/include/CAD_0/brep/topology.hpp
//
// B-Rep topology — radial-edge structure (Weiler 1986, see ADR-0012).
//
// The full entity set, ownership rules, ID scheme, radial-edge population
// contract, and Euler invariant are documented in ADR-0021. This header
// is the canonical declaration; the implementation of the post-pass
// helpers lives in core/brep/src/topology.cpp.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"
#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/brep/pcurve.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::brep {

class Vertex;
class Edge;
class CoEdge;
class Loop;
class Face;
class Shell;
class Solid;
class BRepBody;

using VertexId = std::uint64_t;
using EdgeId = std::uint64_t;
using FaceId = std::uint64_t;
using ShellId = std::uint64_t;

// ---------------------------------------------------------------------------
// Layer-2 honesty — tolerance + provenance (ADR-0032 D2.4 + D8)
// ---------------------------------------------------------------------------
//
// Every curve-carrying entity (vertex, edge, face) declares whether
// its geometry is exact mathematics or a sample within a declared
// linear bound. "Is this edge exact?" is a QUERY, not an
// investigation — the field is the enforcement point:
//
//   * factories write exact (closed forms, tolerance 0) — the
//     default below, which is the mathematical ideal of a
//     constructed point;
//   * the faceted bridge writes approximated with the grid tolerance
//     it already declares (ADR-0026's declared-error culture, now a
//     first-class field);
//   * the booleans propagate the worst of their operands — a
//     fragment of an approximated operand is a sample's descendant;
//     the tolerance is inherited, never diluted;
//   * no code path, UI surface, or export may present approximated
//     geometry as exact (D8) — checked by tests, per path.
//
// The default is exact/0 deliberately: an ideal constructed point
// needs no declaration, while a SAMPLING path must declare its bound
// — the burden sits on the path that would otherwise lie, and the
// per-path tests are the discipline that keeps it there.
enum class Provenance : std::uint8_t {
    Exact,        // closed-form mathematics; linear tolerance 0 by pairing
    Approximated  // a sample within the declared linear tolerance (mm)
};

// The paired record: provenance kind + linear tolerance bound (mm —
// the ToleranceAccumulator precedent of ADR-0004's math layer; the
// kind is always Linear here, so a raw double carries it without a
// redundant kind field).
struct ProvenanceInfo {
    Provenance kind{Provenance::Exact};
    double linear_tolerance{0.0};  // mm; 0 iff Exact — validated

    ProvenanceInfo() = default;

    // Validates the pairing (the house's construction-time
    // discipline — loud, never a silent clamp):
    //   * Approximated demands a finite tolerance > 0: a sample with
    //     a zero or non-finite bound would claim an exactness it does
    //     not have — the lie this field exists to prevent;
    //   * Exact demands tolerance == 0: the pairing IS the contract
    //     (exact | approximated(tol), never a fuzzy middle).
    // Throws std::invalid_argument naming the remedy.
    ProvenanceInfo(Provenance k, double tol);

    static ProvenanceInfo exact() noexcept { return {}; }
    static ProvenanceInfo approximated(double tol) {
        return ProvenanceInfo(Provenance::Approximated, tol);
    }

    bool is_exact() const noexcept { return kind == Provenance::Exact; }
};

// Vertex — a 3D point with an ID
class Vertex {
public:
    Vertex() = default;
    explicit Vertex(math::Vec3f pos) : position_(pos) {}
    const math::Vec3f& position() const noexcept { return position_; }
    void set_position(math::Vec3f p) noexcept { position_ = p; }
    VertexId id() const noexcept { return id_; }
    void set_id(VertexId id) noexcept { id_ = id; }
    // Layer-2 (ADR-0032 D2.4): exact/0 unless a sampling path
    // declares its bound.
    const ProvenanceInfo& provenance() const noexcept { return provenance_; }
    void set_provenance(ProvenanceInfo p) noexcept { provenance_ = p; }
private:
    math::Vec3f position_{};
    VertexId id_{0};
    ProvenanceInfo provenance_{};
};

// Edge — connects two vertices, has radial co-edges.
//
// `coedges_` is the radial-edge list mandated by ADR-0012 §Decision.
// The pointers are **non-owning** (ADR-0021 §3): each CoEdge is owned
// by the Loop that uses it. `link_radial_edges(BRepBody&)` is the only
// sanctioned way to populate this list (ADR-0021 §4).
//
// The list stores `const CoEdge*` because the Edge never mutates the
// CoEdge instances it references — it only reads them (for orientation,
// adjacency queries, future radial walking). The owning Loop retains
// exclusive mutation rights.
class Edge {
public:
    Edge() = default;
    Edge(VertexId v0, VertexId v1) : v_{v0, v1} {}
    VertexId vertex(int i) const noexcept { return v_[i]; }
    void set_vertex(int i, VertexId vid) noexcept { v_[i] = vid; }
    EdgeId id() const noexcept { return id_; }
    void set_id(EdgeId id) noexcept { id_ = id; }
    const std::vector<const CoEdge*>& coedges() const noexcept { return coedges_; }
    void add_coedge(const CoEdge* ce) { coedges_.push_back(ce); }
    void clear_coedges() noexcept { coedges_.clear(); }
    bool is_manifold() const noexcept { return coedges_.size() == 2; }
    // Layer-2 (ADR-0032 D2.4 + D8): "is this edge exact?" — the
    // query the exact-kernel rungs consume.
    const ProvenanceInfo& provenance() const noexcept { return provenance_; }
    void set_provenance(ProvenanceInfo p) noexcept { provenance_ = p; }
private:
    VertexId v_[2]{0, 0};
    EdgeId id_{0};
    std::vector<const CoEdge*> coedges_;  // ADR-0012 radial-edge list; ADR-0021 ownership
    ProvenanceInfo provenance_{};
};

// CoEdge — oriented use of an Edge within a Loop
class CoEdge {
public:
    CoEdge() = default;
    CoEdge(EdgeId edge, bool reversed) : edge_(edge), reversed_(reversed) {}
    EdgeId edge() const noexcept { return edge_; }
    void set_edge(EdgeId e) noexcept { edge_ = e; }
    bool reversed() const noexcept { return reversed_; }
    void set_reversed(bool r) noexcept { reversed_ = r; }
    VertexId start_vertex(const std::vector<Edge>& edges) const;
    VertexId end_vertex(const std::vector<Edge>& edges) const;

    // The p-curve (ADR-0032 D2.2): this co-edge's use of its edge,
    // typed in the OWNING face's surface parameter domain, alongside
    // the 3D edge reference above — the pair that defines an exact
    // B-Rep. The type is the sketch kernel's own (pcurve.hpp, D2.1):
    // a trimming loop and the sketch chain that produced it are the
    // same code and cannot diverge. Conventions (direction, region
    // side, faceted world) are pinned in pcurve.hpp.
    //
    // Returns nullptr when the co-edge carries no p-curve — the
    // faceted world (bridge output, plane-per-triangle) never does;
    // the provenance field distinguishes the worlds, never a guess.
    // Unit 1 plants the field: no existing path writes a p-curve
    // (the writers arrive with G1 units 3–4), and a test pins that.
    const PCurve* p_curve() const noexcept {
        return p_curve_ ? &*p_curve_ : nullptr;
    }
    void set_p_curve(PCurve c) noexcept { p_curve_ = std::move(c); }

private:
    EdgeId edge_{0};
    bool reversed_{false};
    std::optional<PCurve> p_curve_{};  // ADR-0032 D2.2; empty in the faceted world
};

// Loop — closed sequence of co-edges bounding a face
class Loop {
public:
    Loop() = default;
    const std::vector<CoEdge>& coedges() const noexcept { return coedges_; }
    // Mutable access (the boolean copy path, G2 unit 2: the tool's
    // faces are copied wholesale and their co-edges' edge ids
    // REMAPPED to the result's numbering — the faces_mut precedent).
    std::vector<CoEdge>& coedges_mut() noexcept { return coedges_; }
    void add_coedge(CoEdge ce) { coedges_.push_back(std::move(ce)); }
    bool is_outer() const noexcept { return is_outer_; }
    void set_outer(bool outer) noexcept { is_outer_ = outer; }
private:
    std::vector<CoEdge> coedges_;
    bool is_outer_{true};
};

// SurfaceGeometry — abstract base for analytic surfaces
class SurfaceGeometry {
public:
    virtual ~SurfaceGeometry() = default;
    virtual math::Vec3f evaluate(float u, float v) const = 0;
    virtual math::Vec3f normal(float u, float v) const = 0;
    virtual const char* type_name() const = 0;
    virtual math::Bboxf bounds() const = 0;
};

// Face — bounded region of a surface
class Face {
public:
    Face() = default;
    explicit Face(std::shared_ptr<SurfaceGeometry> surf) : surface_(std::move(surf)) {}
    const SurfaceGeometry* surface() const noexcept { return surface_.get(); }
    void set_surface(std::shared_ptr<SurfaceGeometry> surf) { surface_ = std::move(surf); }
    const std::vector<Loop>& loops() const noexcept { return loops_; }
    // Mutable access (the boolean copy path, G2 unit 2 — the
    // coedges_mut note above).
    std::vector<Loop>& loops_mut() noexcept { return loops_; }
    void add_loop(Loop loop) { loops_.push_back(std::move(loop)); }
    FaceId id() const noexcept { return id_; }
    void set_id(FaceId id) noexcept { id_ = id; }
    bool forward() const noexcept { return forward_; }
    void set_forward(bool f) noexcept { forward_ = f; }
    // Layer-2 (ADR-0032 D2.4 + D8): a faceted face declares its
    // sampling bound; an analytic face reads exact/0.
    const ProvenanceInfo& provenance() const noexcept { return provenance_; }
    void set_provenance(ProvenanceInfo p) noexcept { provenance_ = p; }
private:
    std::shared_ptr<SurfaceGeometry> surface_;
    std::vector<Loop> loops_;
    FaceId id_{0};
    bool forward_{true};
    ProvenanceInfo provenance_{};
};

// Shell — set of connected faces
class Shell {
public:
    Shell() = default;
    const std::vector<Face>& faces() const noexcept { return faces_; }
    std::vector<Face>& faces_mut() { return faces_; }
    void add_face(Face face) { faces_.push_back(std::move(face)); }
    ShellId id() const noexcept { return id_; }
    void set_id(ShellId id) noexcept { id_ = id; }
    bool is_closed() const noexcept { return is_closed_; }
    void set_closed(bool c) noexcept { is_closed_ = c; }
private:
    std::vector<Face> faces_;
    ShellId id_{0};
    bool is_closed_{false};
};

// Solid — closed shell forming a volume
class Solid {
public:
    Solid() = default;
    explicit Solid(Shell shell) : shell_(std::move(shell)) {}
    const Shell& shell() const noexcept { return shell_; }
    Shell& shell_mut() { return shell_; }
    void set_shell(Shell shell) { shell_ = std::move(shell); }
private:
    Shell shell_;
};

// BRepBody — public-facing B-Rep body
class BRepBody {
public:
    BRepBody() = default;
    Solid solid;
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;

    std::size_t vertex_count() const noexcept { return vertices.size(); }
    std::size_t edge_count() const noexcept { return edges.size(); }
    std::size_t face_count() const noexcept {
        return solid.shell().faces().size();
    }
    math::Bboxf bounds() const noexcept {
        math::Bboxf b;
        for (const auto& v : vertices) b.expand(v.position());
        return b;
    }
    ~BRepBody() = default;
};

// Inline implementations
inline VertexId CoEdge::start_vertex(const std::vector<Edge>& edges) const {
    if (edge_ >= edges.size()) return 0;
    return reversed_ ? edges[edge_].vertex(1) : edges[edge_].vertex(0);
}
inline VertexId CoEdge::end_vertex(const std::vector<Edge>& edges) const {
    if (edge_ >= edges.size()) return 0;
    return reversed_ ? edges[edge_].vertex(0) : edges[edge_].vertex(1);
}

// ---------------------------------------------------------------------------
// Post-pass helpers — ADR-0021 §4 and §5
// ---------------------------------------------------------------------------

// link_radial_edges — populate Edge::coedges_ from the (Face, Loop, CoEdge)
// hierarchy. Idempotent: clears every edge's radial list before rebuilding.
// Must be called after every body mutation that adds or removes a face
// (factories call it once at the end of construction).
//
// The registered CoEdge* pointers are valid as long as the owning Loop is
// not reallocated (ADR-0021 §3). After this call returns, factories must
// not add further co-edges to existing loops.
void link_radial_edges(BRepBody& body);

// euler_characteristic — χ = V − E + F − H for the body's single
// shell, with H the INNER-LOOP count (inner_loop_count below): every
// loop beyond the first of each face, H = L − F over the total loop
// count L. For a closed manifold solid of disk-faced bodies (H = 0)
// the result is the classical V − E + F (e.g. make_box output: 2).
//
// DERIVATION (ADR-0032 D3, from first principles; Mäntylä 1988
// carries the generalized Euler-Poincaré relation for B-Reps with
// holed faces). A closed surface decomposed into F faces, where face
// f has b(f) boundary loops, has
//
//     χ = V − E + Σ_f (2 − b(f))
//
// — the face-wise characteristics glued along edges and vertices. A
// disk face (one loop) contributes 2 − 1 = 1: the classical
// V − E + F. An ANNULUS face (outer loop + one hole) contributes
// 2 − 2 = 0: a pierced face is not a disk, and counting it as one
// inflates χ by exactly its hole count. With L the total loop count
// and H = L − F the inner-loop count:
//
//     χ = V − E + F − H = 2(S − G)      (Euler-Poincaré, loop term)
//
// THE COUNTEREXAMPLE (why the extension is not optional): the
// annulus as ONE face — outer square loop (4 edges) + inner square
// loop (4 edges) — has V = 8, E = 8, F = 1, H = 1. The disk-faced
// formula reads 8 − 8 + 1 = 1, wrong: an annulus has χ = 0. The
// extended formula reads 8 − 8 + 1 − 1 = 0, right. The guide case's
// subtract produces exactly these holed faces (box minus a
// through-hole: 12V / 17E / 7F / H=2 → χ = 0, genus 1, ADR-0032
// D7.5), agreeing with the faceted report's χ = 0 as it must — same
// topology, better geometry.
//
// BACKWARD COMPATIBILITY (proven by tests, not asserted): every body
// the tree produces today is disk-faced — the bridge's triangles,
// the factories' faces (the seam-wired sphere/cylinder faces are
// disks precisely because the seam wire slits the periodic surface
// open), the booleans' convex fragments — so H = 0 and every
// existing pinned value is unchanged. A periodic surface WITHOUT a
// seam wire (no sanctioned path produces one) would still
// miscount: the seam wire is what makes a periodic face a disk.
//
// The count is POSITIONAL — every loop beyond each face's first,
// independent of Loop::is_outer bookkeeping: a face's boundary
// components are its loops, whatever a flag claims.
//
// Returns a signed 64-bit integer to avoid overflow on large meshes.
std::int64_t euler_characteristic(const BRepBody& body) noexcept;

// inner_loop_count — H, the inner-loop term of the extended
// euler_characteristic: Σ_f max(0, loops(f) − 1). A disk-faced body
// reads 0 (the backward-compatibility proof's own query); the
// annulus reads 1; the guide case's two pierced faces read 2 (one
// inner loop each). Pure integer arithmetic — ADR-0014 honored.
std::int64_t inner_loop_count(const BRepBody& body) noexcept;

// combined_provenance — the worst of two operands' Layer-2 records
// (ADR-0032 D2.4 + D8, hoisted to the post-pass helpers by G1 unit 4
// so the exact subtract and the polygon booleans share ONE
// definition): if ANY entity of either operand is approximated, the
// combination is approximated with the MAXIMUM of the declared
// bounds — a fragment of an approximated operand is a sample's
// descendant; the tolerance is inherited, never diluted, never
// averaged, never silently reset to exact. Both operands fully
// exact → exact. Overstating a bound is conservative honesty;
// understating it is the lie the provenance field exists to prevent.
ProvenanceInfo combined_provenance(const BRepBody& a,
                                    const BRepBody& b);

// loops_are_closed — for every loop of every face, check that the end
// vertex of co-edge i equals the start vertex of co-edge (i+1) mod n.
// Returns false on the first violation. Used by tests as a regression
// guard (ADR-0021 §6).
bool loops_are_closed(const BRepBody& body) noexcept;

// ---------------------------------------------------------------------------
// Query operators — ADR-0022 §2
// ---------------------------------------------------------------------------
//
// Free functions, not methods. Read-only. Pure integer + pointer
// bookkeeping; no floating-point evaluation (ADR-0014 honored).
//
// All return `std::vector` by value; for the MVP scale (cube = 12
// entries, tetrahedron = 6) the copy is trivial. Switch to
// `std::span<const T>` (C++26) when the kernel starts handling real
// CAD bodies (10^4+ edges).
//

// faces_around_edge — the faces that use this edge, in radial order
// (the order in which their co-edges were registered on the edge).
// Empty if `edge_id` is out of range. For a manifold solid, always
// size 2.
std::vector<FaceId> faces_around_edge(const BRepBody& body, EdgeId edge_id);

// edges_around_vertex — the edges incident on this vertex, in no
// particular order. Radial sort needs curve tangents and is deferred
// to Phase D step (c).
std::vector<EdgeId> edges_around_vertex(const BRepBody& body, VertexId vertex_id);

// faces_around_vertex — the faces touching this vertex, deduplicated.
// Order matches the edge incidence order (each edge contributes its
// face set in turn).
std::vector<FaceId> faces_around_vertex(const BRepBody& body, VertexId vertex_id);

// opposite_coedge — for a manifold edge, the other co-edge on the same
// edge. Returns nullptr if the edge is non-manifold (3+ co-edges) or
// the input co-edge is not registered radially on its edge.
//
// For non-manifold edges the function returns the first non-input
// entry in radial order; callers needing the full radial ring should
// access `body.edges[ce->edge()].coedges()` directly.
const CoEdge* opposite_coedge(const BRepBody& body, const CoEdge* ce) noexcept;

// is_closed_orientable — the STRUCTURAL closed-orientable-manifold
// contract (ADR-0022 §7 invariants 0–3, as amended by ADR-0027 D6 —
// Phase F):
//
//   0. V > 0, E > 0, F > 0, shell.is_closed()
//   1. Every edge has exactly 2 co-edges (manifoldness).
//   2. For every edge, the two co-edges have OPPOSITE orientations
//      (one forward, one reversed) — the orientability invariant.
//      A duplicated interface ring (same orientation on both co-edges)
//      fails this check immediately.
//   3. loops_are_closed(body) — every loop's co-edges form a cycle.
//
// These are the ASSEMBLY-DETECTING invariants: everything a correct
// builder of closed orientable manifolds can get wrong in assembly.
// What this predicate deliberately does NOT decide is the input's
// TOPOLOGY CLASSIFICATION — χ (sphere-like χ=2, torus-like χ=0,
// multi-shell χ>2) is a property of the modeled input, not of
// assembly correctness. It is REPORTED via euler_characteristic()
// and genus_from_chi(), never asserted here. The first genus > 0
// body of the repository (the ADR-0027 pierced box) passes this
// predicate exactly as it should.
bool is_closed_orientable(const BRepBody& body) noexcept;

// genus_from_chi — the REPORTED (never asserted) genus of a closed
// orientable body (ADR-0027 D6, acceptance correction 1 — the
// blocking rule). genus = (2 − χ)/2 is derived ONLY when χ ≤ 2 AND χ
// is even; otherwise std::nullopt. The clauses, so nobody has to
// guess why:
//
//   * χ = 2 − 2g holds per connected component of a closed
//     orientable surface, so χ ≤ 2 and even reads as ONE shell of
//     genus (2 − χ)/2 (χ = 0 → genus 1, the pierced box; χ = −2 →
//     genus 2, a double torus).
//   * χ > 2 means multiple shells (χ = 2s − 2g): the components
//     counter is deferred (ADR-0027 out of scope), so the genus is
//     NOT derived — callers report "multi-shell, genus n/d" — and a
//     negative "genus" can never be displayed as if it were one.
//   * an odd χ cannot come from closed orientable components (each
//     has even χ, hence so does any sum): an odd χ under a passing
//     is_closed_orientable is an assembly defect the structural
//     invariants cannot see — deriving a half-integer genus would be
//     an assertion, not a report.
//
// Pure integer arithmetic — ADR-0014 honored, like the rest of the
// post-pass helpers.
std::optional<std::int64_t> genus_from_chi(std::int64_t chi) noexcept;

// is_closed_manifold — COMPATIBILITY WRAPPER (ADR-0022 §7 as amended
// by ADR-0027 D6):
//
//     is_closed_orientable(body) && euler_characteristic(body) == 2
//
// i.e. the exact historical 4-invariant contract, structural
// invariants AND spherical topology. Kept so every pre-Phase-F call
// site — the bridge's pre-amendment postcondition, the bridge and
// boolean test guards — keeps its exact current meaning: nothing
// that passed starts failing, nothing that failed starts passing.
// A correctly extracted pierced box (genus 1) FAILS this wrapper —
// that is the wrapper telling the truth about its own contract, not
// an error.
//
// DEPRECATED for new code (kept indefinitely for compatibility):
// prefer is_closed_orientable() + an explicit euler_characteristic()
// assertion + genus_from_chi().
//
// Strict enough to reject:
//   - periodic-surface sphere/torus (no seam wire → V=E=0)
//   - two faces glued back-to-back (co-edges same orientation)
//   - non-connected shells (χ ≠ 2)
//   - any topology defect from a broken boolean op
bool is_closed_manifold(const BRepBody& body) noexcept;

// wireframe_edges — one (v0, v1) pair per edge, canonicalised so that
// v0 < v1. Useful as input for a future wireframe renderer; pure
// topology here.
std::vector<std::pair<VertexId, VertexId>> wireframe_edges(const BRepBody& body);

// vertex_valence — number of edges incident on the vertex.
// Convenience wrapper around edges_around_vertex(body, v).size().
std::size_t vertex_valence(const BRepBody& body, VertexId vertex_id);

} // namespace CAD_0::brep
