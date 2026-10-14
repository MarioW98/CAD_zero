// core/brep/src/topology.cpp
//
// Implementation of the ADR-0021 post-pass helpers:
//   * link_radial_edges  — populate Edge::coedges_ from the (Face, Loop,
//     CoEdge) hierarchy;
//   * euler_characteristic — χ = V − E + F for the body's single shell;
//   * loops_are_closed — every loop is a closed cycle of co-edges.
//
// These helpers are intentionally integer/pointer only: no floating-point
// evaluation happens here, so the determinism contract (ADR-0014) is
// untouched.
//
#include "CAD_0/brep/topology.hpp"

namespace CAD_0::brep {

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

std::int64_t euler_characteristic(const BRepBody& body) noexcept {
    const std::int64_t V = static_cast<std::int64_t>(body.vertex_count());
    const std::int64_t E = static_cast<std::int64_t>(body.edge_count());
    const std::int64_t F = static_cast<std::int64_t>(body.face_count());
    return V - E + F;
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

} // namespace CAD_0::brep
