// core/brep/src/topology.cpp
//
// Implementation of:
//   * ADR-0021 post-pass helpers (link_radial_edges, euler_characteristic,
//     loops_are_closed);
//   * ADR-0032 D3 (G1 unit 1): the inner-loop term H of the extended
//     euler_characteristic (χ = V − E + F − H) and its query
//     inner_loop_count;
//   * ADR-0032 D2.4 (G1 unit 1): the ProvenanceInfo pairing validation;
//   * ADR-0022 query operators (faces_around_edge, edges_around_vertex,
//     faces_around_vertex, opposite_coedge, wireframe_edges,
//     vertex_valence);
//   * the closed-manifold contract as amended by ADR-0027 D6 (Phase F):
//     is_closed_orientable (structural invariants 0–3), genus_from_chi
//     (the reported genus, correction-1 rule), and the is_closed_manifold
//     compatibility wrapper (structural ∧ χ == 2 — the exact historical
//     meaning).
//
// All functions here are integer ID arithmetic + raw pointer bookkeeping,
// except the ProvenanceInfo constructor's validation. No floating-point
// evaluation happens here, so the determinism contract (ADR-0014) is
// untouched.
//
#include "CAD_0/brep/topology.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace CAD_0::brep {

ProvenanceInfo::ProvenanceInfo(Provenance k, double tol)
    : kind(k), linear_tolerance(tol) {
    // The pairing validation (ADR-0032 D2.4): loud, actionable, never a
    // silent clamp — the two lies the field exists to prevent are a
    // sample claiming exactness and an exact entity hedging with a
    // tolerance it does not have.
    if (k == Provenance::Approximated && !(std::isfinite(tol) && tol > 0.0)) {
        throw std::invalid_argument(
            "ProvenanceInfo: approximated geometry demands a finite linear "
            "tolerance > 0 (mm), got " + std::to_string(tol) +
            " — a sample with a zero or non-finite bound claims an "
            "exactness it does not have: that is the lie the provenance "
            "field exists to prevent (ADR-0032 D2.4/D8). Remedy: declare "
            "the sampling bound (e.g. the bridge's grid tolerance), or "
            "construct exact().");
    }
    if (k == Provenance::Exact && tol != 0.0) {
        throw std::invalid_argument(
            "ProvenanceInfo: exact geometry pairs with tolerance 0, got " +
            std::to_string(tol) +
            " — the pairing IS the contract (exact | approximated(tol), "
            "never a fuzzy middle). Remedy: construct exact(), or declare "
            "the geometry approximated with its real bound.");
    }
}

void link_radial_edges(BRepBody& body) {
    // Clear every existing radial entry (idempotent — safe to re-run after
    // a body mutation).
    for (auto& edge : body.edges) {
        edge.clear_coedges();
    }

    // Iterate every face of every shell. The current model has a single
    // shell per body (BRepBody::solid), but the loop structure here is
    // ready for the multi-shell future without rewrite.
    auto& shell = body.solid.shell_mut();
    for (auto& face : shell.faces_mut()) {
        for (auto& loop : face.loops()) {
            for (auto& coedge : loop.coedges()) {
                const EdgeId eid = coedge.edge();
                if (eid < body.edges.size()) {
                    body.edges[eid].add_coedge(&coedge);
                }
            }
        }
    }
}

ProvenanceInfo combined_provenance(const BRepBody& a, const BRepBody& b) {
    // The worst-of rule (ADR-0032 D2.4 + D8, unit 1's boolean
    // discipline, hoisted here by unit 4 so every exact consumer
    // shares the ONE definition — the no-drift rule at the entity
    // level). Pure record arithmetic, no geometry.
    bool any_approximated = false;
    double max_tol = 0.0;
    const auto consider = [&](const ProvenanceInfo& p) {
        if (!p.is_exact()) {
            any_approximated = true;
            max_tol = std::max(max_tol, p.linear_tolerance);
        }
    };
    for (const auto& v : a.vertices) consider(v.provenance());
    for (const auto& e : a.edges) consider(e.provenance());
    for (const auto& f : a.solid.shell().faces()) consider(f.provenance());
    for (const auto& v : b.vertices) consider(v.provenance());
    for (const auto& e : b.edges) consider(e.provenance());
    for (const auto& f : b.solid.shell().faces()) consider(f.provenance());
    return any_approximated ? ProvenanceInfo::approximated(max_tol)
                             : ProvenanceInfo::exact();
}

std::int64_t inner_loop_count(const BRepBody& body) noexcept {
    // H = L − F (ADR-0032 D3): every loop beyond the first of each
    // face is a boundary component the disk-faced formula V − E + F
    // silently assumes away. The count is positional — a face's
    // boundary components are its loops, whatever the is_outer flag
    // claims — and a loop-less face (no sanctioned path produces one)
    // contributes 0 rather than a nonsensical −1.
    std::int64_t H = 0;
    for (const auto& face : body.solid.shell().faces()) {
        const std::int64_t b =
            static_cast<std::int64_t>(face.loops().size());
        if (b > 1) {
            H += b - 1;
        }
    }
    return H;
}

std::int64_t euler_characteristic(const BRepBody& body) noexcept {
    // χ = V − E + F − H (ADR-0032 D3): the Euler-Poincaré loop term
    // the guide case's pierced faces force. The derivation, the
    // annulus counterexample, and the backward-compatibility contract
    // (every body the tree produces today is disk-faced, H = 0, no
    // pinned value moves) live in the header docstring; the tests
    // prove the compatibility body by body, golden by golden.
    const std::int64_t V = static_cast<std::int64_t>(body.vertex_count());
    const std::int64_t E = static_cast<std::int64_t>(body.edge_count());
    const std::int64_t F = static_cast<std::int64_t>(body.face_count());
    const std::int64_t H = inner_loop_count(body);
    return V - E + F - H;
}

bool loops_are_closed(const BRepBody& body) noexcept {
    const auto& shell = body.solid.shell();
    const auto& edges = body.edges;
    for (const auto& face : shell.faces()) {
        for (const auto& loop : face.loops()) {
            const auto& coedges = loop.coedges();
            if (coedges.empty()) continue;
            const std::size_t n = coedges.size();
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t j = (i + 1) % n;
                const VertexId end_i   = coedges[i].end_vertex(edges);
                const VertexId start_j = coedges[j].start_vertex(edges);
                if (end_i != start_j) {
                    return false;
                }
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// ADR-0022 query operators
// ---------------------------------------------------------------------------

std::vector<FaceId> faces_around_edge(const BRepBody& body, EdgeId edge_id) {
    std::vector<FaceId> result;
    if (edge_id >= body.edges.size()) {
        return result;
    }
    const auto& radial = body.edges[edge_id].coedges();
    // Each CoEdge* points into the Loop that owns it. We need to walk
    // back from the CoEdge to the FaceId that contains its loop. Since
    // we don't store a parent pointer on Loop, we scan faces/loops and
    // match by pointer identity.
    const auto& shell = body.solid.shell();
    for (const CoEdge* ce : radial) {
        bool found = false;
        for (FaceId fid = 0; fid < shell.faces().size(); ++fid) {
            const auto& face = shell.faces()[fid];
            for (const auto& loop : face.loops()) {
                const auto& coedges = loop.coedges();
                // A CoEdge* in the radial list points into the storage
                // of `loop.coedges_`. Match by pointer range.
                if (!coedges.empty()) {
                    const CoEdge* base = coedges.data();
                    const CoEdge* end   = base + coedges.size();
                    if (ce >= base && ce < end) {
                        result.push_back(fid);
                        found = true;
                        break;
                    }
                }
            }
            if (found) break;
        }
    }
    return result;
}

std::vector<EdgeId> edges_around_vertex(const BRepBody& body, VertexId vertex_id) {
    std::vector<EdgeId> result;
    if (vertex_id >= body.vertices.size()) {
        return result;
    }
    for (EdgeId eid = 0; eid < body.edges.size(); ++eid) {
        const auto& edge = body.edges[eid];
        if (edge.vertex(0) == vertex_id || edge.vertex(1) == vertex_id) {
            result.push_back(eid);
        }
    }
    return result;
}

std::vector<FaceId> faces_around_vertex(const BRepBody& body, VertexId vertex_id) {
    std::vector<FaceId> result;
    if (vertex_id >= body.vertices.size()) {
        return result;
    }
    const auto& shell = body.solid.shell();
    // For each face, check whether any of its loops touches this vertex.
    // A loop touches the vertex if any of its co-edges (start or end)
    // lands on it.
    for (FaceId fid = 0; fid < shell.faces().size(); ++fid) {
        const auto& face = shell.faces()[fid];
        bool touches = false;
        for (const auto& loop : face.loops()) {
            for (const auto& ce : loop.coedges()) {
                if (ce.start_vertex(body.edges) == vertex_id ||
                    ce.end_vertex(body.edges)   == vertex_id) {
                    touches = true;
                    break;
                }
            }
            if (touches) break;
        }
        if (touches) {
            result.push_back(fid);
        }
    }
    return result;
}

const CoEdge* opposite_coedge(const BRepBody& body, const CoEdge* ce) noexcept {
    if (ce == nullptr) return nullptr;
    const EdgeId eid = ce->edge();
    if (eid >= body.edges.size()) return nullptr;
    const auto& radial = body.edges[eid].coedges();
    for (const CoEdge* other : radial) {
        if (other != ce) {
            return other;
        }
    }
    return nullptr;
}

bool is_closed_orientable(const BRepBody& body) noexcept {
    // Invariant 0: a closed manifold solid must have at least one
    // vertex, one edge, and one face. Without this guard, the
    // periodic-surface sphere (1 face, 0 edges, 0 vertices) would
    // pass vacuously (ADR-0022 §5 regression guard).
    if (body.vertex_count() == 0) return false;
    if (body.edge_count()   == 0) return false;
    if (body.face_count()   == 0) return false;
    if (!body.solid.shell().is_closed()) return false;

    // Invariant 1: every edge has exactly 2 co-edges (manifoldness —
    // extends Edge::is_manifold() to the whole body).
    for (const auto& edge : body.edges) {
        if (edge.coedges().size() != 2) {
            return false;
        }
    }

    // Invariant 2: for every edge, the two co-edges have OPPOSITE
    // orientations (one forward, one reversed). This is what distinguishes
    // an orientable closed solid from two faces glued back-to-back.
    // A duplicated interface ring (same orientation on both co-edges)
    // fails this check immediately.
    for (const auto& edge : body.edges) {
        const auto& ce = edge.coedges();
        if (ce.size() != 2) return false;
        if (ce[0]->reversed() == ce[1]->reversed()) {
            return false;  // same orientation → not orientable
        }
    }

    // Invariant 3: every loop of every face is closed (end vertex of
    // co-edge i equals start vertex of co-edge (i+1) mod n).
    if (!loops_are_closed(body)) return false;

    // ADR-0027 D6: this is where the STRUCTURAL contract ends. The
    // input's topological classification (χ, genus — sphere-like,
    // torus-like, multi-shell) is NOT an assembly correctness
    // property: it is reported by the callers via
    // euler_characteristic() and genus_from_chi(), never asserted
    // here. The pre-Phase-F invariant 4 (χ == 2) lives on inside the
    // is_closed_manifold compatibility wrapper below, with its exact
    // historical meaning.
    return true;
}

std::optional<std::int64_t> genus_from_chi(std::int64_t chi) noexcept {
    // ADR-0027 D6, acceptance correction 1 (the blocking rule):
    // genus = (2 − χ)/2 is derived ONLY for χ ≤ 2 and even.
    //
    //   * χ ≤ 2, even → one shell of genus (2 − χ)/2 (χ = 2 → 0,
    //     χ = 0 → 1 — the pierced box, χ = −2 → 2);
    //   * χ > 2 → multiple shells (χ = 2s − 2g); the components
    //     counter is deferred, so the genus is not derived — callers
    //     report "multi-shell, n/d" and a negative genus is never
    //     displayed as if it were real;
    //   * odd χ → cannot be a sum of closed orientable components'
    //     (even) characteristics; under a passing is_closed_orientable
    //     it is an assembly defect the structural invariants cannot
    //     see — a half-integer genus would be an assertion, not a
    //     report, so it is not derived either.
    if (chi > 2 || (chi % 2) != 0) return std::nullopt;
    return (2 - chi) / 2;
}

bool is_closed_manifold(const BRepBody& body) noexcept {
    // Compatibility wrapper (ADR-0022 §7 as amended by ADR-0027 D6):
    // the exact historical 4-invariant contract — structural
    // invariants AND spherical topology (χ == 2). Zero silent
    // weakening: every pre-Phase-F call site keeps its meaning, and a
    // correctly extracted genus-1 body fails here exactly as it did
    // before the amendment (that is this wrapper's contract, not an
    // extraction error). New code prefers is_closed_orientable() +
    // explicit χ assertions + genus_from_chi().
    return is_closed_orientable(body) && euler_characteristic(body) == 2;
}

std::vector<std::pair<VertexId, VertexId>> wireframe_edges(const BRepBody& body) {
    std::vector<std::pair<VertexId, VertexId>> result;
    result.reserve(body.edges.size());
    for (const auto& edge : body.edges) {
        VertexId a = edge.vertex(0);
        VertexId b = edge.vertex(1);
        if (a > b) std::swap(a, b);
        result.emplace_back(a, b);
    }
    return result;
}

std::size_t vertex_valence(const BRepBody& body, VertexId vertex_id) {
    return edges_around_vertex(body, vertex_id).size();
}

} // namespace CAD_0::brep
