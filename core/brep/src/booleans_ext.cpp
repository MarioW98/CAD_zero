// core/brep/src/booleans_ext.cpp
//
// The extended booleans' implementation — ADR-0033 D5.2 (G2 unit 2).
// The contract, the derivations and the pinned dispositions live in
// booleans_ext.hpp's header; this file carries the machinery: the
// class gates (the entry contract), the face kit (the cap, the zone,
// the disk, the wall band), and the operations. Pure functions
// throughout (ADR-0014): the same operands assemble the same bytes.
#include "CAD_0/brep/booleans_ext.hpp"

#include "CAD_0/brep/curve.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::brep {

// The sphere-target branch of exact_subtract — declared in
// subtract.hpp, dispatched there for the sphere-class targets.
std::unique_ptr<BRepBody> exact_subtract_sphere_target(
    const BRepBody& A, const BRepBody& B, math::Vec3f tool_offset,
    float tol);

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717958647692f;

std::string num(float v) { return std::to_string(v); }

enum class OpKind : std::uint8_t { Union, Intersect, Subtract };

const char* op_name(OpKind k) {
    switch (k) {
    case OpKind::Union: return "exact_union";
    case OpKind::Intersect: return "exact_intersect";
    case OpKind::Subtract: return "exact_subtract";
    }
    return "?";
}

// --- the operand records --------------------------------------------------

struct SphereRec {
    math::Vec3f center{};
    float radius{0.0f};
    math::Vec3f pole{}, azimuth{};   // the stored (re-framed) frame
    const Face* face{nullptr};
    bool budgeted{false};            // typed cuts beyond the seam
};

const SphereSurface* single_sphere_face(const BRepBody& b) {
    if (b.face_count() != 1) return nullptr;
    return dynamic_cast<const SphereSurface*>(
        b.solid.shell().faces()[0].surface());
}

std::size_t distinct_loop_edges(const Face& f) {
    std::set<EdgeId> ids;
    for (const auto& loop : f.loops())
        for (const auto& ce : loop.coedges()) ids.insert(ce.edge());
    return ids.size();
}

SphereRec read_sphere_face(const Face& f) {
    SphereRec r;
    const auto* sp = dynamic_cast<const SphereSurface*>(f.surface());
    r.center = sp->origin();
    r.radius = sp->radius();
    r.pole = sp->pole().normalized();
    r.azimuth = sp->azimuth().normalized();
    r.face = &f;
    r.budgeted = distinct_loop_edges(f) > 1;
    return r;
}

// The sphere-CLASS body (the subtract target): exactly ONE sphere
// face; every other face a plane or CylinderZ whose every edge either
// appears in the sphere face's loops or is that face's own seam pair.
bool is_sphere_class_body(const BRepBody& A, std::string& why,
                          FaceId& sphere_face) {
    bool any_sphere = false;
    for (std::size_t i = 0; i < A.face_count(); ++i) {
        const Face& f = A.solid.shell().faces()[i];
        const std::string fam =
            f.surface() ? f.surface()->type_name() : std::string("(none)");
        if (fam == "sphere") {
            if (any_sphere) {
                why = "the target carries more than one sphere face (" +
                      std::to_string(A.face_count()) + " faces)";
                return false;
            }
            any_sphere = true;
            sphere_face = static_cast<FaceId>(i);
            continue;
        }
        if (fam != "plane" && fam != "cylinder") {
            why = "the target carries a '" + fam + "' face";
            return false;
        }
    }
    if (!any_sphere) {
        why = "the target carries no sphere face";
        return false;
    }
    std::set<EdgeId> sphere_edges;
    for (const auto& ce :
         A.solid.shell().faces()[static_cast<std::size_t>(sphere_face)]
             .loops()[0]
             .coedges())
        sphere_edges.insert(ce.edge());
    for (std::size_t i = 0; i < A.face_count(); ++i) {
        if (static_cast<FaceId>(i) == sphere_face) continue;
        const Face& f = A.solid.shell().faces()[i];
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges()) {
                if (sphere_edges.count(ce.edge()) > 0) continue;
                std::size_t count = 0;
                for (const auto& l2 : f.loops())
                    for (const auto& c2 : l2.coedges())
                        if (c2.edge() == ce.edge()) ++count;
                if (count != 2) {
                    why = "a companion face carries an edge neither "
                          "shared with the sphere face nor its own seam";
                    return false;
                }
            }
    }
    return true;
}

// The provenance gate — the honesty chain, the unit-1 gate's words.
void demand_exact(const BRepBody& b, const char* what, const char* op) {
    const auto consider = [&](const ProvenanceInfo& p) {
        if (!p.is_exact())
            throw std::invalid_argument(
                std::string(op) + ": the " + what +
                "'s materialization is not exact (provenance "
                "approximated, tolerance " +
                std::to_string(p.linear_tolerance) +
                ") - the exact table consults exact faces (ADR-0033 "
                "D1.1's gate; the booleans consume exactness, the "
                "honesty chain), and answering an approximated operand "
                "in closed form would present sampled geometry as "
                "exact - the lie the Layer-2 provenance exists to "
                "prevent. Remedy: re-materialize the operand exactly "
                "(a factory primitive or an in-class operation), or "
                "take the faceted path (the bridge's own world, "
                "ADR-0026).");
    };
    for (const auto& v : b.vertices) consider(v.provenance());
    for (const auto& e : b.edges) consider(e.provenance());
    for (const auto& f : b.solid.shell().faces()) consider(f.provenance());
}

// --- the face kit ---------------------------------------------------------

// An authored rim half-arc: the edge id + its 3D midpoint (the
// matching key for the face on the other side of the rim).
struct RimHalf {
    EdgeId edge{0};
    math::Vec3f mid{};
};

PCurve lat_piece(float u_a, float u_b, float v, bool left = true) {
    return sdf::sketch_segment(math::Vec2f{u_a, v}, math::Vec2f{u_b, v},
                               /*interior_left=*/left);
}

PCurve mer_piece(float u, float v_a, float v_b, bool left = true) {
    return sdf::sketch_segment(math::Vec2f{u, v_a}, math::Vec2f{u, v_b},
                               /*interior_left=*/left);
}

// THE CAP — the disk around the pole, v in [0, v0], the slit-rectangle
// walk [seam_R up, lat W1, lat W2, seam_L down] (the factory sphere's
// own pattern, the window shortened). The edges seam (P<->S), latW1
// (S->A), latW2 (A->S) are authored cap-forward. Returns the two rim
// halves (W1, W2) for the sharing face.
//
// THE `rr` VARIANT (region-on-right) — the XOR derivation's other
// encoding, for the PIERCED pairing: the paired plane's authored walk
// (the CCW inner loop) opposes the cap's honest walk exactly when
// pole = -n_paired; a pierced tool face's normal points INTO the kept
// cap (pole = +n_paired), so the cap walks the REVERSED cycle with
// the flipped bits — the G1 cavity wall's own trick, the same
// requirement seen from the sphere's side.
std::array<RimHalf, 2> add_cap_face(Shell& shell, const SphereSurface& surf,
                                    float v0, VertexId P, VertexId S,
                                    VertexId A, EdgeId seam, EdgeId latW1,
                                    EdgeId latW2, const ProvenanceInfo& prov,
                                    bool rr = false) {
    Face face(std::make_shared<SphereSurface>(surf));
    Loop loop;
    if (!rr) {
        CoEdge seam_r{seam, false};  // P -> S, the RIGHT edge
        seam_r.set_p_curve(mer_piece(kTwoPi, 0.0f, v0));
        CoEdge w1{latW1, false};     // S -> A
        w1.set_p_curve(lat_piece(kTwoPi, kPi, v0));
        CoEdge w2{latW2, false};     // A -> S
        w2.set_p_curve(lat_piece(kPi, 0.0f, v0));
        CoEdge seam_l{seam, true};   // S -> P, the LEFT edge
        seam_l.set_p_curve(mer_piece(0.0f, v0, 0.0f));
        loop.add_coedge(seam_r);
        loop.add_coedge(w1);
        loop.add_coedge(w2);
        loop.add_coedge(seam_l);
    } else {
        // The REVERSED cycle, the region on the right: P -> S up the
        // LEFT edge, S -> A through the +e1 half (latW2's arc,
        // traversed reversed), A -> S through the -e1 half (latW1,
        // reversed), S -> P down the RIGHT edge.
        CoEdge seam_l{seam, false};  // P -> S, the LEFT edge
        seam_l.set_p_curve(mer_piece(0.0f, 0.0f, v0, /*left=*/false));
        CoEdge w2{latW2, true};      // S -> A (the edge authored A->S)
        w2.set_p_curve(lat_piece(0.0f, kPi, v0, /*left=*/false));
        CoEdge w1{latW1, true};      // A -> S (the edge authored S->A)
        w1.set_p_curve(lat_piece(kPi, kTwoPi, v0, /*left=*/false));
        CoEdge seam_r{seam, true};   // S -> P, the RIGHT edge
        seam_r.set_p_curve(mer_piece(kTwoPi, v0, 0.0f, /*left=*/false));
        loop.add_coedge(seam_l);
        loop.add_coedge(w2);
        loop.add_coedge(w1);
        loop.add_coedge(seam_r);
    }
    face.add_loop(std::move(loop));
    face.set_id(static_cast<FaceId>(shell.faces().size()));
    face.set_provenance(prov);
    shell.add_face(std::move(face));
    return {RimHalf{latW1, surf.evaluate(1.5f * kPi, v0)},
            RimHalf{latW2, surf.evaluate(0.5f * kPi, v0)}};
}

// THE CAP LOOP on SHARED rim halves: the same domain walk, the co-edge
// flags derived from the edges' own vertex order so the traversal
// chains P -> S -> A -> S -> P while the shared edges read opposite
// from the authoring face (the XOR). The halves are matched to this
// frame's W1/W2 by midpoint.
void add_cap_face_shared(Shell& shell, const BRepBody& R,
                         const SphereSurface& surf, float v0, VertexId P,
                         VertexId S, VertexId A, EdgeId seam,
                         const RimHalf& half1, const RimHalf& half2,
                         const ProvenanceInfo& prov) {
    const math::Vec3f m_w1 = surf.evaluate(1.5f * kPi, v0);
    const float d1 = (m_w1 - half1.mid).dot(m_w1 - half1.mid);
    const float d2 = (m_w1 - half2.mid).dot(m_w1 - half2.mid);
    const EdgeId e_w1 = d1 <= d2 ? half1.edge : half2.edge;
    const EdgeId e_w2 = d1 <= d2 ? half2.edge : half1.edge;
    const Edge& ed1 = R.edges[static_cast<std::size_t>(e_w1)];
    const Edge& ed2 = R.edges[static_cast<std::size_t>(e_w2)];

    Face face(std::make_shared<SphereSurface>(surf));
    Loop loop;
    CoEdge seam_r{seam, false};
    seam_r.set_p_curve(mer_piece(kTwoPi, 0.0f, v0));
    CoEdge w1{e_w1, ed1.vertex(0) != S};
    w1.set_p_curve(lat_piece(kTwoPi, kPi, v0));
    CoEdge w2{e_w2, ed2.vertex(0) != A};
    w2.set_p_curve(lat_piece(kPi, 0.0f, v0));
    CoEdge seam_l{seam, true};
    seam_l.set_p_curve(mer_piece(0.0f, v0, 0.0f));
    loop.add_coedge(seam_r);
    loop.add_coedge(w1);
    loop.add_coedge(w2);
    loop.add_coedge(seam_l);
    face.add_loop(std::move(loop));
    face.set_id(static_cast<FaceId>(shell.faces().size()));
    face.set_provenance(prov);
    shell.add_face(std::move(face));
}

// THE ZONE — the annulus v in [v1, v2], the slit-rectangle walk
// [seam_R up, top W, seam_L down, bottom E] (the factory cylinder
// wall's pattern on the sphere's domain). The rim vertices N0/A0 at
// v1 (the u=0 / u=pi points), S0/A0p at v2; the six edges: seam
// (N0<->S0), the top halves (S0->A0', A0'->S0), the bottom halves
// (N0->A0, A0->N0) — all authored zone-forward.
std::array<RimHalf, 4> add_zone_face(Shell& shell, const SphereSurface& surf,
                                     float v1, float v2, VertexId N0,
                                     VertexId A0, VertexId S0, VertexId A0p,
                                     EdgeId seam, EdgeId topW1, EdgeId topW2,
                                     EdgeId botE1, EdgeId botE2,
                                     const ProvenanceInfo& prov) {
    Face face(std::make_shared<SphereSurface>(surf));
    Loop loop;
    CoEdge seam_r{seam, false};  // N0 -> S0, the RIGHT edge
    seam_r.set_p_curve(mer_piece(kTwoPi, v1, v2));
    CoEdge tw1{topW1, false};    // S0 -> A0'
    tw1.set_p_curve(lat_piece(kTwoPi, kPi, v2));
    CoEdge tw2{topW2, false};    // A0' -> S0
    tw2.set_p_curve(lat_piece(kPi, 0.0f, v2));
    CoEdge seam_l{seam, true};   // S0 -> N0, the LEFT edge
    seam_l.set_p_curve(mer_piece(0.0f, v2, v1));
    CoEdge be1{botE1, false};    // N0 -> A0
    be1.set_p_curve(lat_piece(0.0f, kPi, v1));
    CoEdge be2{botE2, false};    // A0 -> N0
    be2.set_p_curve(lat_piece(kPi, kTwoPi, v1));
    loop.add_coedge(seam_r);
    loop.add_coedge(tw1);
    loop.add_coedge(tw2);
    loop.add_coedge(seam_l);
    loop.add_coedge(be1);
    loop.add_coedge(be2);
    face.add_loop(std::move(loop));
    face.set_id(static_cast<FaceId>(shell.faces().size()));
    face.set_provenance(prov);
    shell.add_face(std::move(face));
    return {RimHalf{topW1, surf.evaluate(1.5f * kPi, v2)},
            RimHalf{topW2, surf.evaluate(0.5f * kPi, v2)},
            RimHalf{botE1, surf.evaluate(0.5f * kPi, v1)},
            RimHalf{botE2, surf.evaluate(1.5f * kPi, v1)}};
}

// THE WALL BAND — the unified authoring: the window [u_L, u_R] x
// [0, h] with u_R = 2pi (u_sign +1) or 0 (u_sign -1), the CCW walk
// [seam_R up, top west (2 arcs), seam_L down, bottom east (2 arcs)],
// material on the LEFT (the frame rule: both signs' frames are
// right-handed around their own outward normals — the G1's cavity
// wall used the region-on-right encoding for ITS partner, the pierced
// planes; this unit's partners are the sphere faces, whose walks
// demand the CCW cycle — the XOR derivation in the header). The rim
// co-edges MATCH to the pre-authored halves by midpoint when the
// pointers are non-null (the flags derived from the edges' vertex
// order), else the band authors the rim edges fresh. Returns the
// band's own four rim halves (top1, top2, bot1, bot2).
std::array<RimHalf, 4> add_band_face(Shell& shell, BRepBody& R,
                                     const math::Vec2f& center, float radius,
                                     float z_base, float h, int u_sign,
                                     VertexId Sb, VertexId Ab, VertexId St,
                                     VertexId At, EdgeId seam,
                                     const RimHalf* top1, const RimHalf* top2,
                                     const RimHalf* bot1, const RimHalf* bot2,
                                     const ProvenanceInfo& prov) {
    const float u_R = (u_sign > 0) ? kTwoPi : 0.0f;
    const float u_L = u_R - kTwoPi;
    auto surf = std::make_shared<CylinderZSurface>(center, radius, z_base,
                                                   u_sign);

    // The fresh-edge authoring (the walks are band-forward).
    EdgeId e_top1 = 0, e_top2 = 0, e_bot1 = 0, e_bot2 = 0;
    if (top1 == nullptr || top2 == nullptr) {
        const EdgeId a = [&] {
            Edge e{St, At};
            e.set_id(static_cast<EdgeId>(R.edges.size()));
            e.set_provenance(prov);
            R.edges.push_back(e);
            return e.id();
        }();
        const EdgeId b = [&] {
            Edge e{At, St};
            e.set_id(static_cast<EdgeId>(R.edges.size()));
            e.set_provenance(prov);
            R.edges.push_back(e);
            return e.id();
        }();
        e_top1 = a;
        e_top2 = b;
    }
    if (bot1 == nullptr || bot2 == nullptr) {
        const EdgeId a = [&] {
            Edge e{Sb, Ab};
            e.set_id(static_cast<EdgeId>(R.edges.size()));
            e.set_provenance(prov);
            R.edges.push_back(e);
            return e.id();
        }();
        const EdgeId b = [&] {
            Edge e{Ab, Sb};
            e.set_id(static_cast<EdgeId>(R.edges.size()));
            e.set_provenance(prov);
            R.edges.push_back(e);
            return e.id();
        }();
        e_bot1 = a;
        e_bot2 = b;
    }

    // The four rim walks: their midpoints (for the matching) and their
    // start vertices (for the flags).
    struct RimWalk {
        EdgeId matched{0};
        bool fresh{false};
        VertexId start{};
        float mid_u{}, mid_v{};
    };
    RimWalk walks[4] = {
        {/*topW1*/ 0, false, St, u_R - 0.5f * kPi, h},
        {/*topW2*/ 0, false, At, u_L + 0.5f * kPi, h},
        {/*botE1*/ 0, false, Sb, u_L + 0.5f * kPi, 0.0f},
        {/*botE2*/ 0, false, Ab, u_R - 0.5f * kPi, 0.0f},
    };
    const RimHalf* pairs[2][2] = {{top1, top2}, {bot1, bot2}};
    for (int side = 0; side < 2; ++side) {
        const RimHalf* h1 = pairs[side][0];
        const RimHalf* h2 = pairs[side][1];
        const int i0 = side * 2;        // topW1 / botE1
        const int i1 = side * 2 + 1;    // topW2 / botE2
        if (h1 == nullptr || h2 == nullptr) {
            walks[i0].matched = (side == 0) ? e_top1 : e_bot1;
            walks[i1].matched = (side == 1) ? e_bot2 : e_top2;
            walks[i0].fresh = walks[i1].fresh = true;
            continue;
        }
        const math::Vec3f m0 = surf->evaluate(walks[i0].mid_u,
                                              walks[i0].mid_v);
        const float d1 = (m0 - h1->mid).dot(m0 - h1->mid);
        const float d2 = (m0 - h2->mid).dot(m0 - h2->mid);
        walks[i0].matched = d1 <= d2 ? h1->edge : h2->edge;
        walks[i1].matched = d1 <= d2 ? h2->edge : h1->edge;
    }

    Face face(surf);
    Loop loop;
    const auto coedge = [&](const RimWalk& w, const math::Vec2f& a,
                            const math::Vec2f& b) {
        const Edge& ed = R.edges[static_cast<std::size_t>(w.matched)];
        CoEdge ce{w.matched, !w.fresh && ed.vertex(0) != w.start};
        ce.set_p_curve(sdf::sketch_segment(a, b, /*interior_left=*/true));
        return ce;
    };
    loop.add_coedge(coedge(walks[0], math::Vec2f{u_R, h},
                           math::Vec2f{u_R - kPi, h}));      // topW1
    loop.add_coedge(coedge(walks[1], math::Vec2f{u_R - kPi, h},
                           math::Vec2f{u_L, h}));            // topW2
    {
        CoEdge s_down{seam, true};  // St -> Sb
        s_down.set_p_curve(mer_piece(u_L, h, 0.0f));
        loop.add_coedge(s_down);
    }
    loop.add_coedge(coedge(walks[2], math::Vec2f{u_L, 0.0f},
                           math::Vec2f{u_L + kPi, 0.0f}));   // botE1
    loop.add_coedge(coedge(walks[3], math::Vec2f{u_L + kPi, 0.0f},
                           math::Vec2f{u_R, 0.0f}));          // botE2
    {
        CoEdge s_up{seam, false};   // Sb -> St, the RIGHT edge
        s_up.set_p_curve(mer_piece(u_R, 0.0f, h));
        loop.add_coedge(s_up);
    }
    face.add_loop(std::move(loop));
    face.set_id(static_cast<FaceId>(shell.faces().size()));
    face.set_provenance(prov);
    shell.add_face(std::move(face));

    return {RimHalf{walks[0].matched,
                    surf->evaluate(walks[0].mid_u, walks[0].mid_v)},
            RimHalf{walks[1].matched,
                    surf->evaluate(walks[1].mid_u, walks[1].mid_v)},
            RimHalf{walks[2].matched,
                    surf->evaluate(walks[2].mid_u, walks[2].mid_v)},
            RimHalf{walks[3].matched,
                    surf->evaluate(walks[3].mid_u, walks[3].mid_v)}};
}

// THE RIM LOOP on a plane — the two CCW pi-arcs of a rim circle
// already split at the antipodal points S, A, matched to the authored
// rim halves by midpoint. The arcs carry the `hole` bit (false: the
// disc material, the factory caps; true: the pierced face's inner
// loop, the material outside the circle — the G1 flipped-bits
// convention).
Loop rim_loop_on_plane(const BRepBody& R, const PlaneSurface& pl,
                       const math::Vec3f& rim_center, float radius,
                       VertexId S, VertexId A, const RimHalf& half1,
                       const RimHalf& half2, bool hole) {
    const auto dom = [&](const math::Vec3f& p) {
        const math::Vec3f d = p - pl.origin();
        return math::Vec2f{d.dot(pl.u_axis()), d.dot(pl.v_axis())};
    };
    const math::Vec2f cdom = dom(rim_center);
    const math::Vec2f sdom =
        dom(R.vertices[static_cast<std::size_t>(S)].position());
    const float theta_S = std::atan2(sdom.y - cdom.y, sdom.x - cdom.x);
    const float theta_A = theta_S + kPi;

    const math::Vec3f mid1 = pl.evaluate(
        cdom.x + radius * std::cos(theta_S + 0.5f * kPi),
        cdom.y + radius * std::sin(theta_S + 0.5f * kPi));
    const float d11 = (mid1 - half1.mid).dot(mid1 - half1.mid);
    const float d12 = (mid1 - half2.mid).dot(mid1 - half2.mid);
    const EdgeId e_arc1 = d11 <= d12 ? half1.edge : half2.edge;
    const EdgeId e_arc2 = d11 <= d12 ? half2.edge : half1.edge;

    Loop loop;
    {
        const Edge& ed1 = R.edges[static_cast<std::size_t>(e_arc1)];
        CoEdge a1{e_arc1, ed1.vertex(0) != S};
        a1.set_p_curve(sdf::sketch_arc(cdom, radius, hole, theta_S, kPi));
        loop.add_coedge(a1);
    }
    {
        const Edge& ed2 = R.edges[static_cast<std::size_t>(e_arc2)];
        CoEdge a2{e_arc2, ed2.vertex(0) != A};
        a2.set_p_curve(sdf::sketch_arc(cdom, radius, hole, theta_A, kPi));
        loop.add_coedge(a2);
    }
    return loop;
}

// A vertex/edge appender — the numbering discipline (ids == the
// vector indices, appended in the authored order).
struct Builder {
    BRepBody* R{nullptr};
    ProvenanceInfo prov{};

    VertexId vertex(const math::Vec3f& p) {
        Vertex v{p};
        v.set_id(static_cast<VertexId>(R->vertices.size()));
        v.set_provenance(prov);
        R->vertices.push_back(v);
        return v.id();
    }
    EdgeId edge(VertexId a, VertexId b) {
        Edge e{a, b};
        e.set_id(static_cast<EdgeId>(R->edges.size()));
        e.set_provenance(prov);
        R->edges.push_back(e);
        return e.id();
    }
};

// The hard postconditions (the assembly culture, with the H term).
void postconditions(const BRepBody& R, std::int64_t chi_expected,
                    const char* op) {
    link_radial_edges(const_cast<BRepBody&>(R));
    if (!loops_are_closed(R))
        throw std::runtime_error(
            std::string(op) +
            ": postcondition failed - a loop does not chain (an "
            "assembly defect; report this as a kernel bug)");
    if (!is_closed_orientable(R))
        throw std::runtime_error(
            std::string(op) +
            ": postcondition failed - the result is not a closed "
            "orientable manifold (the radial-edge XOR or the shell "
            "closure broke; report this as a kernel bug)");
    for (const auto& e : R.edges) {
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            throw std::runtime_error(
                std::string(op) + ": postcondition failed - edge " +
                std::to_string(e.id()) +
                " is not manifold-with-XOR (an assembly defect; report "
                "this as a kernel bug)");
    }
    if (euler_characteristic(R) != chi_expected)
        throw std::runtime_error(
            std::string(op) + ": postcondition failed - chi = " +
            std::to_string(euler_characteristic(R)) + " (the pinned "
            "value is " + std::to_string(chi_expected) +
            "; report this as a kernel bug)");
}

bool sphere_key_less(const SphereRec& a, const SphereRec& b) {
    if (a.center.x != b.center.x) return a.center.x < b.center.x;
    if (a.center.y != b.center.y) return a.center.y < b.center.y;
    if (a.center.z != b.center.z) return a.center.z < b.center.z;
    return a.radius < b.radius;
}

// --- the sphere×sphere pair ------------------------------------------------

enum class PairKind : std::uint8_t {
    Overlap,
    Coincident,
    ConcentricUnequal,
    ExternalTangent,
    InternalTangent,
    Nested,
    Separate
};

PairKind classify_pair(const SphereRec& a, const SphereRec& b, float tol) {
    const math::Vec3f w = b.center - a.center;
    const float D = w.length();
    const float R1 = a.radius, R2 = b.radius;
    if (D <= tol)
        return std::abs(R1 - R2) <= tol ? PairKind::Coincident
                                        : PairKind::ConcentricUnequal;
    if (D > R1 + R2 + tol) return PairKind::Separate;
    if (std::abs(D - (R1 + R2)) <= tol) return PairKind::ExternalTangent;
    if (std::abs(D - std::abs(R1 - R2)) <= tol && R1 != R2)
        return PairKind::InternalTangent;
    if (D < std::abs(R1 - R2) - tol) return PairKind::Nested;
    return PairKind::Overlap;
}

// Build the two-cap body (the union's far caps / the lens's near
// caps): the poles p1/p2 (unit, outward from each center into its own
// kept cap), v01/v02 the cap latitudes. The shared azimuth is the
// radical plane's own u axis — the antipodal poles make the pair's
// u-directions oppose (the XOR derivation in the header).
std::unique_ptr<BRepBody> build_two_caps(const SphereRec& s1,
                                         const SphereRec& s2,
                                         const math::Vec3f& u, float h,
                                         const math::Vec3f& p1,
                                         const math::Vec3f& p2, float v01,
                                         float v02,
                                         const ProvenanceInfo& prov,
                                         const char* op) {
    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    Builder bd{R.get(), prov};

    const math::Vec3f cc = s1.center + u * h;
    const PlaneSurface radical(cc, u);
    const math::Vec3f e0 = radical.u_axis();

    const SphereSurface s1f(s1.radius, s1.center, p1, e0);
    const SphereSurface s2f(s2.radius, s2.center, p2, e0);

    const VertexId P1 = bd.vertex(s1f.evaluate(0.0f, 0.0f));
    const VertexId P2 = bd.vertex(s2f.evaluate(0.0f, 0.0f));
    const VertexId Sv = bd.vertex(s1f.evaluate(0.0f, v01));
    const VertexId Av = bd.vertex(s1f.evaluate(kPi, v01));

    const EdgeId seam1 = bd.edge(P1, Sv);
    const EdgeId seam2 = bd.edge(P2, Sv);
    const EdgeId latW1 = bd.edge(Sv, Av);
    const EdgeId latW2 = bd.edge(Av, Sv);

    const auto halves =
        add_cap_face(shell, s1f, v01, P1, Sv, Av, seam1, latW1, latW2, prov);
    add_cap_face_shared(shell, *R, s2f, v02, P2, Sv, Av, seam2, halves[0],
                        halves[1], prov);

    postconditions(*R, 2, op);
    return R;
}

// The bitten sphere: the far cap of s1 (pole -u, the kept side away
// from s2) + the radical-plane disk. V=3 (the pole, S, A), E=3 (the
// seam + the two rim halves), F=2, H=0, chi=2.
std::unique_ptr<BRepBody> build_bitten(const SphereRec& s1,
                                       const math::Vec3f& u, float h,
                                       const ProvenanceInfo& prov) {
    auto R = std::make_unique<BRepBody>();
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    Builder bd{R.get(), prov};

    const math::Vec3f cc = s1.center + u * h;
    // The disk: the radical plane, the outward +u (the material on
    // the -u side). The cap's azimuth is the disk's own u axis (the
    // unified rule: -sign(pole.n) * u_ax, here pole.n = -1).
    const PlaneSurface disk_plane(cc, u);
    const SphereSurface capf(s1.radius, s1.center, u * -1.0f,
                             disk_plane.u_axis());
    const float v0 = static_cast<float>(
        std::acos(std::clamp(-double(h) / double(s1.radius), -1.0, 1.0)));

    const VertexId P = bd.vertex(capf.evaluate(0.0f, 0.0f));
    const VertexId S = bd.vertex(capf.evaluate(0.0f, v0));
    const VertexId A = bd.vertex(capf.evaluate(kPi, v0));
    const EdgeId seam = bd.edge(P, S);
    const EdgeId latW1 = bd.edge(S, A);
    const EdgeId latW2 = bd.edge(A, S);

    const auto halves =
        add_cap_face(shell, capf, v0, P, S, A, seam, latW1, latW2, prov);
    const float rho =
        std::sqrt(std::max(0.0f, s1.radius * s1.radius - h * h));
    Face disk(std::make_shared<PlaneSurface>(disk_plane));
    disk.add_loop(rim_loop_on_plane(*R, disk_plane, cc, rho, S, A,
                                    halves[0], halves[1],
                                    /*hole=*/false));
    disk.set_id(static_cast<FaceId>(shell.faces().size()));
    disk.set_provenance(prov);
    shell.add_face(std::move(disk));

    postconditions(*R, 2, "exact_subtract");
    return R;
}

// The sphere-pair op: the dispositions and the assemblies.
// `body_a`/`body_b` are the ALREADY-TRANSLATED operands.
std::unique_ptr<BRepBody> sphere_pair_op(const BRepBody& body_a,
                                         const BRepBody& body_b, OpKind op,
                                         float tol) {
    const char* name = op_name(op);
    SphereRec r1 = read_sphere_face(body_a.solid.shell().faces()[0]);
    SphereRec r2 = read_sphere_face(body_b.solid.shell().faces()[0]);
    if (r1.budgeted || r2.budgeted)
        throw std::invalid_argument(
            std::string(name) +
            ": a sphere operand carries typed cuts beyond the seam - "
            "the frame budget (one frame = one axis family per face, "
            "ADR-0033 D1.1); the sphere-pair class takes FULL spheres "
            "(a trimmed survivor is the prism path's operand). "
            "Remedy: pass the factory sphere, or accept the faceted "
            "path.");

    const BRepBody* bodies[2] = {&body_a, &body_b};
    if (!sphere_key_less(r1, r2)) {
        std::swap(r1, r2);
        std::swap(bodies[0], bodies[1]);
    }
    const bool a_is_first = (bodies[0] == &body_a);
    const auto copy_of = [&](const SphereRec& pick) {
        auto R = std::make_unique<BRepBody>(
            ((&pick == &r1) == a_is_first) ? body_a : body_b);
        link_radial_edges(*R);
        return R;
    };

    switch (classify_pair(r1, r2, tol)) {
    case PairKind::Coincident:
        if (op == OpKind::Subtract)
            throw std::invalid_argument(
                "exact_subtract: the spheres are COINCIDENT (equal "
                "centers, equal radii) - the co-radial dedup rule's "
                "trigger (the coplanar rule's analog): one surface is "
                "kept, the other absorbed, and A minus A is the EMPTY "
                "solid (the swallow disposition, G1's own). Remedy: "
                "subtract a DIFFERENT body, or skip the operation (the "
                "empty result).");
        return copy_of(r1);
    case PairKind::ConcentricUnequal:
    case PairKind::Nested:
        if (op == OpKind::Subtract) return copy_of(r1);
        return copy_of((r1.radius >= r2.radius) == (op == OpKind::Union)
                           ? r1
                           : r2);
    case PairKind::InternalTangent:
        if (op == OpKind::Subtract)
            throw std::invalid_argument(
                "exact_subtract: the spheres touch at ONE POINT "
                "(internal tangency, D = |R1-R2|) - the removed cavity "
                "would touch the target's boundary at the contact, and "
                "the result pinches there (a non-manifold point on the "
                "cavity's wall; the topology flips). Remedy: nest the "
                "tool strictly inside (a real cavity is not authored "
                "in this class), overlap the boundaries properly, or "
                "accept the faceted path.");
        return copy_of((r1.radius >= r2.radius) == (op == OpKind::Union)
                           ? r1
                           : r2);
    case PairKind::ExternalTangent:
        if (op == OpKind::Subtract) {
            auto R = std::make_unique<BRepBody>(body_a);
            link_radial_edges(*R);
            return R;
        }
        throw std::invalid_argument(
            std::string(name) +
            ": the spheres touch at ONE POINT (external tangency, D = "
            "R1+R2) - the result's boundary would be non-manifold at "
            "the contact (two balls touching do not form one closed "
            "orientable surface; the topology flips). Remedy: move the "
            "operands apart or overlap them by more than the declared "
            "tolerance (" +
            num(tol) + "), or keep them as separate bodies.");
    case PairKind::Separate:
        if (op == OpKind::Subtract) {
            auto R = std::make_unique<BRepBody>(body_a);
            link_radial_edges(*R);
            return R;
        }
        if (op == OpKind::Union)
            throw std::invalid_argument(
                std::string(name) +
                ": the spheres are SEPARATE (D > R1+R2) - the "
                "disconnected union is not a single body, and the "
                "author class models separate bodies as separate "
                "bodies. Remedy: move the operands to overlap, or keep "
                "both (the tree's own answer).");
        throw std::invalid_argument(
            std::string(name) +
            ": the spheres are SEPARATE (D > R1+R2) - the intersection "
            "is the EMPTY solid. Remedy: move the operands to overlap, "
            "or accept the empty result by not intersecting.");
    case PairKind::Overlap:
        break;
    }

    const math::Vec3f w = r2.center - r1.center;
    const float D = w.length();
    const math::Vec3f u = w * (1.0f / D);
    const float h = (D * D + r1.radius * r1.radius -
                     r2.radius * r2.radius) /
                    (2.0f * D);
    const ProvenanceInfo prov = combined_provenance(body_a, body_b);

    if (op == OpKind::Subtract) return build_bitten(r1, u, h, prov);

    if (op == OpKind::Union) {
        const float v01 = static_cast<float>(
            std::acos(std::clamp(-double(h) / double(r1.radius),
                                 -1.0, 1.0)));
        const float v02 = static_cast<float>(
            std::acos(std::clamp((double(h) - double(D)) /
                                     double(r2.radius),
                                 -1.0, 1.0)));
        return build_two_caps(r1, r2, u, h, u * -1.0f, u, v01, v02, prov,
                              name);
    }
    const float v01 = static_cast<float>(
        std::acos(std::clamp(double(h) / double(r1.radius), -1.0, 1.0)));
    const float v02 = static_cast<float>(
        std::acos(std::clamp((double(D) - double(h)) / double(r2.radius),
                             -1.0, 1.0)));
    return build_two_caps(r1, r2, u, h, u, u * -1.0f, v01, v02, prov, name);
}

// === PART B: the prism tool, the coaxial path, the plane family ===

struct ToolWall {
    const Face* face{nullptr};
    const PlaneSurface* plane{nullptr};
    const CylinderZSurface* cyl{nullptr};
};

struct PrismTool {
    float z_lo{0.0f}, z_hi{0.0f};
    bool is_circle{false};
    math::Vec2f circle_center{};
    float circle_radius{0.0f};
    std::vector<ToolWall> walls{};
    const PlaneSurface* cap_hi{nullptr};
    const PlaneSurface* cap_lo{nullptr};
};

// The D6 prism signature (the G1 tool checks, compact): every face a
// plane or a Z-cylinder, exactly one +Z cap and one -Z cap, vertical
// laterals, single loops, half-space convex.
PrismTool read_prism_tool(const BRepBody& Bp, float tol) {
    PrismTool t;
    std::size_t pos_z = 0, neg_z = 0;
    for (const auto& f : Bp.solid.shell().faces()) {
        const auto* surf = f.surface();
        const std::string fam =
            surf ? std::string(surf->type_name()) : std::string("none");
        if (surf == nullptr || (fam != "plane" && fam != "cylinder"))
            throw std::invalid_argument(
                "exact_*: the tool is not a Z-axis prism of the D6 "
                "class (a face of family '" +
                fam +
                "') - the extended booleans consume the exact "
                "extrude's prisms (a convex profile or a circle) and "
                "the fitted box, placed by the recipe's offset. "
                "Remedy: use an extruded profile (or the box) as the "
                "tool, or accept the faceted path.");
        if (fam == "cylinder" &&
            dynamic_cast<const CylinderZSurface*>(surf) == nullptr)
            throw std::invalid_argument(
                "exact_*: the tool is not a Z-axis prism of the D6 "
                "class (its cylinder's axis is Y - the primitive "
                "factory's placement). Remedy: model the tool as an "
                "extruded circle profile, or accept the faceted path.");
        if (f.loops().size() != 1)
            throw std::invalid_argument(
                "exact_*: the tool is not CONVEX (a face carries inner "
                "loops - a ring profile's tool: the caps' holes make it "
                "non-convex). Remedy: use a convex tool profile, or "
                "accept the faceted path.");
        const auto* pl = dynamic_cast<const PlaneSurface*>(surf);
        if (pl != nullptr) {
            const float nz = pl->normal_vec().z;
            if (nz > 1.0f - tol) {
                ++pos_z;
                t.cap_hi = pl;
                t.z_hi = pl->origin().z;
            } else if (nz < -(1.0f - tol)) {
                ++neg_z;
                t.cap_lo = pl;
                t.z_lo = pl->origin().z;
            } else if (std::abs(nz) > tol) {
                throw std::invalid_argument(
                    "exact_*: the tool has an OBLIQUE face - not a "
                    "prism of the D6 class. Remedy: use an extruded "
                    "profile as the tool, or accept the faceted path.");
            } else {
                ToolWall w;
                w.face = &f;
                w.plane = pl;
                t.walls.push_back(w);
            }
        } else {
            ToolWall w;
            w.face = &f;
            w.cyl = dynamic_cast<const CylinderZSurface*>(surf);
            t.walls.push_back(w);
        }
    }
    if (pos_z != 1 || neg_z != 1)
        throw std::invalid_argument(
            "exact_*: the tool is not a Z-axis prism of the D6 class "
            "(it has " +
            std::to_string(pos_z) + " +Z faces and " +
            std::to_string(neg_z) +
            " -Z faces; a prism has exactly one of each - its caps). "
            "Remedy: use an extruded profile (or the fitted box) as "
            "the tool, or accept the faceted path.");
    for (const auto& f : Bp.solid.shell().faces()) {
        const auto* pl = dynamic_cast<const PlaneSurface*>(f.surface());
        if (pl == nullptr) continue;
        const math::Vec3f o = pl->origin();
        const math::Vec3f n = pl->normal_vec();
        for (const auto& v : Bp.vertices) {
            if ((v.position() - o).dot(n) > tol)
                throw std::invalid_argument(
                    "exact_*: the tool is not CONVEX (a vertex lies "
                    "outside a face's half-space - a reflex profile "
                    "corner). Remedy: use a convex tool profile, or "
                    "accept the faceted path.");
        }
    }
    // The circle profile: ONE cylinder wall spanning the full period.
    std::size_t cyl_walls = 0;
    for (const auto& w : t.walls)
        if (w.cyl != nullptr) ++cyl_walls;
    if (cyl_walls == 1 && t.walls.size() == 1) {
        const auto* cz = t.walls[0].cyl;
        t.is_circle = true;
        t.circle_center = cz->axis_center();
        t.circle_radius = cz->radius();
    }
    return t;
}

// The frame-budget gate for a trimmed survivor: the new cut's family
// axis vs the sphere face's stored pole (the budget's own record).
void demand_budget(const SphereRec& sph, const math::Vec3f& family_axis,
                   float tol, const char* name) {
    if (!sph.budgeted) return;
    const float c = std::abs(sph.pole.dot(family_axis.normalized()));
    if (c < 1.0f - tol)
        throw std::invalid_argument(
            std::string(name) +
            ": the frame budget - the target's sphere face carries "
            "typed cuts beyond the seam, so its frame is budgeted to "
            "ONE axis family (ADR-0033 D1.1), and the new cut's family "
            "(axis (" +
            num(family_axis.x) + ", " + num(family_axis.y) + ", " +
            num(family_axis.z) +
            ")) is not parallel to the budgeted one (the stored pole). "
            "One frame = one axis family per face: the incompatible "
            "family finds the wall - out-of-table, the accepted "
            "boundary (option (i)). Remedy: reorient the operands (cut "
            "parallel to the budgeted axis), re-cut from the ORIGINAL "
            "full sphere in one operation, accept the faceted path, or "
            "await the multi-cut unit.");
    throw std::invalid_argument(
        std::string(name) +
        ": the frame budget - the target's sphere face carries typed "
        "cuts beyond the seam, and although the new cut's family is "
        "parallel to the budgeted one, the MULTI-CUT cascade on a "
        "trimmed survivor (narrowing the kept band, re-timing the "
        "companion faces) is not in this unit's class - the honest "
        "scope, not a silent guess. Remedy: re-cut from the ORIGINAL "
        "full sphere with the composed intent (the coaxial family "
        "admits one tool: cut with the larger radius directly), or "
        "accept the faceted path, or await the multi-cut unit.");
}

// The coaxial circle-prism assembly: the bead (subtract), the rivet
// (union), the core (intersect).
std::unique_ptr<BRepBody> sphere_prism_coaxial(const SphereRec& sph,
                                               const PrismTool& tool,
                                               OpKind op, float tol) {
    const char* name = op_name(op);
    const float R = sph.radius;
    const float r = tool.circle_radius;
    const float Cz = sph.center.z;

    // The covering (the "trafora" class): the tool's z-span must cover
    // the sphere's, else a cap lies inside - the blind family.
    if (!(tool.z_lo <= Cz - R + tol && tool.z_hi >= Cz + R - tol))
        throw std::invalid_argument(
            std::string(name) +
            ": the BLIND tool - a cap lies strictly inside the "
            "sphere's span (the tool spans [z_lo, z_hi] = [" +
            num(tool.z_lo) + ", " + num(tool.z_hi) +
            "], the sphere [Cz-R, Cz+R] = [" + num(Cz - R) + ", " +
            num(Cz + R) +
            "]) - the internal cap face class needs the SSI-driven "
            "cap insertion, not the pass-through latitude pair this "
            "path builds (the G1 blind disposition). Remedy: extend "
            "the extrusion distance so the tool passes through, or "
            "accept the faceted path.");

    if (r >= R - tol) {
        if (r <= R + tol)
            throw std::invalid_argument(
                std::string(name) +
                ": the tool's wall is tangent to the sphere along its "
                "equator (r = R) - the tool contains the sphere and "
                "the exact result is the EMPTY solid (the swallow "
                "disposition, G1's own). Remedy: use a smaller tool "
                "radius, or accept the empty result by not "
                "subtracting.");
        throw std::invalid_argument(
            std::string(name) +
            ": the tool's cylinder swallows the sphere (r = " +
            num(r) + " > R = " + num(R) +
            ") - the exact result is the EMPTY solid. Remedy: use a "
            "smaller tool, or accept the empty result by not "
            "subtracting.");
    }

    const float w = std::sqrt(std::max(0.0f, R * R - r * r));
    const float v1 = static_cast<float>(
        std::acos(std::clamp(double(w) / double(R), -1.0, 1.0)));
    const float v2 = kPi - v1;
    const math::Vec2f cxy{sph.center.x, sph.center.y};

    // THE CONVENTION (the unit's XOR derivation): the sphere's
    // re-frame azimuth = u_sign * (the wall's u=0 meridian, +X) - the
    // wall's increasing-u direction at the seam.
    const int usign = (op == OpKind::Subtract) ? -1 : +1;
    const math::Vec3f azimuth{float(usign), 0.0f, 0.0f};
    const math::Vec3f axis{0.0f, 0.0f, 1.0f};

    auto out = std::make_unique<BRepBody>();
    auto& shell = out->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();
    Builder bd{out.get(), prov};

    // The latitude split points: the +X and -X meridians (the wall's
    // own u=0/u=pi; the sphere's azimuth carries the sign, so its
    // u=0 meridian sits on the -X side for the bead - the SAME two 3D
    // points either way).
    const math::Vec3f n_p{cxy.x + r, cxy.y, Cz + w};   // +X, north
    const math::Vec3f n_m{cxy.x - r, cxy.y, Cz + w};   // -X, north
    const math::Vec3f s_p{cxy.x + r, cxy.y, Cz - w};   // +X, south
    const math::Vec3f s_m{cxy.x - r, cxy.y, Cz - w};   // -X, south

    if (op == OpKind::Subtract) {
        // THE BEAD: the equatorial zone + the cavity wall band.
        // V=4, E=6, F=2, H=0, chi=0 (genus 1).
        const SphereSurface zonef(R, sph.center, axis, azimuth);
        const VertexId N0 = bd.vertex(n_m);   // zone u=0 at v1
        const VertexId A0 = bd.vertex(n_p);   // zone u=pi at v1
        const VertexId S0 = bd.vertex(s_m);   // zone u=0 at v2
        const VertexId A0p = bd.vertex(s_p);  // zone u=pi at v2
        const EdgeId seam = bd.edge(N0, S0);
        const EdgeId topW1 = bd.edge(S0, A0p);
        const EdgeId topW2 = bd.edge(A0p, S0);
        const EdgeId botE1 = bd.edge(N0, A0);
        const EdgeId botE2 = bd.edge(A0, N0);
        const auto zh = add_zone_face(shell, zonef, v1, v2, N0, A0, S0,
                                      A0p, seam, topW1, topW2, botE1,
                                      botE2, prov);
        // The wall band (u_sign = -1, the cavity; the CCW cycle of the
        // window [-2pi, 0]): its rim co-edges MATCH the zone's halves,
        // its own seam is the +X vertical (a NEW edge - the meridian
        // and the ruling are different curves).
        const EdgeId wseam = bd.edge(A0p, A0);  // +X south -> +X north
        add_band_face(shell, *out, cxy, r, /*z_base=*/Cz - w,
                      /*h=*/2.0f * w, /*u_sign=*/-1,
                      /*Sb=*/A0p, /*Ab=*/S0, /*St=*/A0, /*At=*/N0,
                      wseam, /*top=*/&zh[2], &zh[3], /*bot=*/&zh[0],
                      &zh[1], prov);
        postconditions(*out, 0, name);
        return out;
    }

    if (op == OpKind::Intersect) {
        // THE CORE: the two polar caps + the middle wall band.
        // V=6, E=7, F=3, H=0, chi=2.
        const SphereSurface capP(R, sph.center, axis, azimuth);
        const SphereSurface capM(R, sph.center, axis * -1.0f, azimuth);
        const VertexId Pp = bd.vertex(sph.center + math::Vec3f{0, 0, R});
        const VertexId Pm = bd.vertex(sph.center - math::Vec3f{0, 0, R});
        const VertexId Np = bd.vertex(n_p);
        const VertexId Nm = bd.vertex(n_m);
        const VertexId Sp = bd.vertex(s_p);
        const VertexId Sm = bd.vertex(s_m);
        const EdgeId seamP = bd.edge(Pp, Np);
        const EdgeId latP1 = bd.edge(Np, Nm);
        const EdgeId latP2 = bd.edge(Nm, Np);
        const EdgeId seamM = bd.edge(Pm, Sp);
        const EdgeId latM1 = bd.edge(Sp, Sm);
        const EdgeId latM2 = bd.edge(Sm, Sp);
        const auto hP =
            add_cap_face(shell, capP, v1, Pp, Np, Nm, seamP, latP1, latP2,
                         prov);
        const auto hM =
            add_cap_face(shell, capM, v1, Pm, Sp, Sm, seamM, latM1, latM2,
                         prov);
        const EdgeId wseam = bd.edge(Sp, Np);  // +X vertical
        add_band_face(shell, *out, cxy, r, Cz - w, 2.0f * w, +1,
                      /*Sb=*/Sp, /*Ab=*/Sm, /*St=*/Np, /*At=*/Nm, wseam,
                      /*top=*/&hP[0], &hP[1], /*bot=*/&hM[0], &hM[1], prov);
        postconditions(*out, 2, name);
        return out;
    }

    // THE RIVET (union): the zone + two wall bands + two end disks.
    // V=8, E=11, F=5, H=0, chi=2.
    const SphereSurface zonef(R, sph.center, axis, azimuth);
    // The zone's u=0/u=pi points with the +X azimuth (the bead's
    // negated frame swaps them; here u=0 IS the +X meridian).
    const VertexId N0 = bd.vertex(n_p);   // zone u=0 at v1
    const VertexId A0 = bd.vertex(n_m);   // zone u=pi at v1
    const VertexId S0 = bd.vertex(s_p);   // zone u=0 at v2
    const VertexId A0p = bd.vertex(s_m);  // zone u=pi at v2
    const EdgeId zseam = bd.edge(N0, S0);
    const EdgeId topW1 = bd.edge(S0, A0p);
    const EdgeId topW2 = bd.edge(A0p, S0);
    const EdgeId botE1 = bd.edge(N0, A0);
    const EdgeId botE2 = bd.edge(A0, N0);
    const auto zh = add_zone_face(shell, zonef, v1, v2, N0, A0, S0, A0p,
                                  zseam, topW1, topW2, botE1, botE2, prov);
    // The end-rim split points (the wall's +-X meridians).
    const VertexId Etp = bd.vertex(math::Vec3f{cxy.x + r, cxy.y, tool.z_hi});
    const VertexId Etm = bd.vertex(math::Vec3f{cxy.x - r, cxy.y, tool.z_hi});
    const VertexId Ebp = bd.vertex(math::Vec3f{cxy.x + r, cxy.y, tool.z_lo});
    const VertexId Ebm = bd.vertex(math::Vec3f{cxy.x - r, cxy.y, tool.z_lo});
    // The upper band: [Cz+w, z_hi]; its bottom rim = the NORTH latitude
    // (the zone's botE halves), its top rim fresh (the end disk
    // matches later). The band's u=0 meridian is +X: Sb = the +X
    // north point (the zone's N0).
    const EdgeId useam = bd.edge(N0, Etp);  // +X vertical
    const auto uh = add_band_face(shell, *out, cxy, r, Cz + w,
                                  tool.z_hi - (Cz + w), +1,
                                  /*Sb=*/N0, /*Ab=*/A0, /*St=*/Etp,
                                  /*At=*/Etm, useam,
                                  /*top=*/nullptr, nullptr,
                                  /*bot=*/&zh[2], &zh[3], prov);
    // The lower band: [z_lo, Cz-w]; its top rim = the SOUTH latitude
    // (the zone's topW halves); its St is the +X south point (S0).
    const EdgeId lseam = bd.edge(Ebp, S0);  // +X vertical
    const auto lh = add_band_face(shell, *out, cxy, r, tool.z_lo,
                                  (Cz - w) - tool.z_lo, +1,
                                  /*Sb=*/Ebp, /*Ab=*/Ebm, /*St=*/S0,
                                  /*At=*/A0p, lseam,
                                  /*top=*/&zh[0], &zh[1],
                                  /*bot=*/nullptr, nullptr, prov);
    // The end disks on the tool's cap planes (the outward +-Z).
    {
        const PlaneSurface pl(math::Vec3f{cxy.x, cxy.y, tool.z_hi},
                              math::Vec3f{0, 0, 1});
        Face disk(std::make_shared<PlaneSurface>(pl));
        disk.add_loop(rim_loop_on_plane(
            *out, pl, math::Vec3f{cxy.x, cxy.y, tool.z_hi}, r, Etp, Etm,
            uh[0], uh[1], /*hole=*/false));
        disk.set_id(static_cast<FaceId>(shell.faces().size()));
        disk.set_provenance(prov);
        shell.add_face(std::move(disk));
    }
    {
        const PlaneSurface pl(math::Vec3f{cxy.x, cxy.y, tool.z_lo},
                              math::Vec3f{0, 0, -1});
        Face disk(std::make_shared<PlaneSurface>(pl));
        disk.add_loop(rim_loop_on_plane(
            *out, pl, math::Vec3f{cxy.x, cxy.y, tool.z_lo}, r, Ebp, Ebm,
            lh[2], lh[3], /*hole=*/false));
        disk.set_id(static_cast<FaceId>(shell.faces().size()));
        disk.set_provenance(prov);
        shell.add_face(std::move(disk));
    }
    postconditions(*out, 2, name);
    return out;
}

// One cap around `pole` bounded by the plane at the a-hat offset t,
// paired with a plane whose normal is `paired_n` (the disk's -pole,
// or the pierced tool face's +pole). THE AZIMUTH RULE (the unit's
// derivation): azimuth = -sign(pole . paired_n) * u_ax(paired plane).
struct CapPieces {
    VertexId P{0}, S{0}, A{0};
    EdgeId seam{0}, W1{0}, W2{0};
    std::array<RimHalf, 2> halves{};
    SphereSurface surf{1.0f, math::Vec3f{}};  // replaced by build
    float v0{0.0f};
};

CapPieces build_cap_pieces(Builder& bd, const SphereRec& sph,
                           const math::Vec3f& a_hat,
                           const math::Vec3f& pole, float t,
                           const math::Vec3f& paired_n,
                           const math::Vec3f& paired_u_ax) {
    const float sgn = (pole.dot(paired_n) > 0.0f) ? -1.0f : 1.0f;
    const math::Vec3f e0 = paired_u_ax * sgn;
    CapPieces c;
    c.surf = SphereSurface(sph.radius, sph.center, pole, e0);
    c.v0 = static_cast<float>(std::acos(std::clamp(
        double(pole.dot(a_hat)) * double(t) / double(sph.radius),
        -1.0, 1.0)));
    c.P = bd.vertex(c.surf.evaluate(0.0f, 0.0f));
    c.S = bd.vertex(c.surf.evaluate(0.0f, c.v0));
    c.A = bd.vertex(c.surf.evaluate(kPi, c.v0));
    c.seam = bd.edge(c.P, c.S);
    c.W1 = bd.edge(c.S, c.A);
    c.W2 = bd.edge(c.A, c.S);
    return c;
}

// The plane-family assembly: the cutting faces all parallel to a_hat;
// the tool's interior band [T_lo, T_hi] on that axis; the kept pieces
// per op (the truncation semantics).
std::unique_ptr<BRepBody> sphere_prism_planes(
    const BRepBody& A, const BRepBody& Bp, const SphereRec& sph,
    const PrismTool& tool, OpKind op, float tol,
    const std::vector<const PlaneSurface*>& cutting, const char* name) {
    const float R = sph.radius;
    const math::Vec3f a_hat =
        cutting[0]->normal_vec().normalized();

    // The tool's a-hat band, from ALL the a-hat-parallel plane faces
    // (walls and caps alike — the caps when a_hat is Z).
    float T_hi = std::numeric_limits<float>::infinity();
    float T_lo = -std::numeric_limits<float>::infinity();
    const auto consider = [&](const PlaneSurface* pl) {
        const math::Vec3f n = pl->normal_vec();
        const float s = (pl->origin() - sph.center).dot(n);
        if (n.dot(a_hat) > 1.0f - tol) T_hi = std::min(T_hi, s);
        if (n.dot(a_hat) < -(1.0f - tol)) T_lo = std::max(T_lo, -s);
    };
    for (const auto& w : tool.walls)
        if (w.plane != nullptr) consider(w.plane);
    if (tool.cap_hi != nullptr) consider(tool.cap_hi);
    if (tool.cap_lo != nullptr) consider(tool.cap_lo);

    const bool lo_cuts = T_lo > -R + tol;   // a kept piece below
    const bool hi_cuts = T_hi < R - tol;    // a kept piece above

    // The SUBTRACT / UNION sphere pieces: the far caps. The
    // INTERSECT's: the near band (a cap or the zone).
    const bool want_lo = (op == OpKind::Intersect) ? lo_cuts : lo_cuts;
    const bool want_hi = (op == OpKind::Intersect) ? hi_cuts : hi_cuts;
    (void)want_lo;
    (void)want_hi;

    if (op == OpKind::Subtract && !lo_cuts && !hi_cuts)
        throw std::invalid_argument(
            std::string(name) +
            ": the tool's band swallows the sphere on the cut family "
            "(the tool contains it) - the exact result is the EMPTY "
            "solid. Remedy: use a smaller tool, or accept the empty "
            "result by not subtracting.");
    if (op == OpKind::Intersect && !lo_cuts && !hi_cuts) {
        // The band covers the sphere: the tool contains it - the
        // sphere verbatim (the contains disposition; the dispatch
        // below would have caught a miss).
        auto out = std::make_unique<BRepBody>(A);
        link_radial_edges(*out);
        return out;
    }

    auto out = std::make_unique<BRepBody>();
    auto& shell = out->solid.shell_mut();
    shell.set_closed(true);
    const ProvenanceInfo prov = ProvenanceInfo::exact();
    Builder bd{out.get(), prov};

    // Author one kept cap (pole = -/+ a_hat at the bound t), and its
    // pairing: the disk (subtract/intersect) or the pierced tool face
    // (union).
    const auto add_kept_cap = [&](const math::Vec3f& pole, float t,
                                  const PlaneSurface* pierced) {
        // The paired plane's normal and u axis: the disk's -pole (an
        // authored plane), or the pierced tool face's own frame.
        math::Vec3f paired_n, paired_u;
        if (pierced != nullptr) {
            paired_n = pierced->normal_vec().normalized();
            paired_u = pierced->u_axis();
        } else {
            const PlaneSurface pl(sph.center + pole * -1.0f * t,
                                  pole * -1.0f);
            paired_n = pole * -1.0f;
            paired_u = pl.u_axis();
        }
        CapPieces c = build_cap_pieces(bd, sph, a_hat, pole, t, paired_n,
                                       paired_u);
        c.halves = add_cap_face(shell, c.surf, c.v0, c.P, c.S, c.A, c.seam,
                                c.W1, c.W2, prov,
                                /*rr=*/pierced != nullptr);
        return c;
    };

    if (op == OpKind::Union) {
        // The far caps (pierced pairings) + the tool's faces.
        std::vector<CapPieces> caps;
        std::vector<std::pair<const PlaneSurface*, CapPieces*>> by_plane;
        if (lo_cuts) {
            // The cap around -a_hat at T_lo; the pierced face there is
            // the tool face whose plane sits at t = T_lo with the
            // outward -a_hat.
            const PlaneSurface* pf = nullptr;
            for (const auto* pl : cutting)
                if (pl->normal_vec().dot(a_hat) < -(1.0f - tol)) pf = pl;
            caps.push_back(add_kept_cap(a_hat * -1.0f, T_lo, pf));
            by_plane.push_back({pf, &caps.back()});
        }
        if (hi_cuts) {
            const PlaneSurface* pf = nullptr;
            for (const auto* pl : cutting)
                if (pl->normal_vec().dot(a_hat) > 1.0f - tol) pf = pl;
            caps.push_back(add_kept_cap(a_hat, T_hi, pf));
            by_plane.push_back({pf, &caps.back()});
        }
        // Append the tool's entities, re-based; the cutting faces gain
        // their rim inner loops.
        const std::size_t V_base = out->vertices.size();
        const std::size_t E_base = out->edges.size();
        const std::size_t F_base = shell.faces().size();
        for (const auto& v : Bp.vertices) {
            Vertex nv = v;
            nv.set_id(static_cast<VertexId>(out->vertices.size()));
            out->vertices.push_back(nv);
        }
        for (const auto& e : Bp.edges) {
            Edge ne = e;
            ne.set_id(static_cast<EdgeId>(out->edges.size()));
            out->edges.push_back(ne);
        }
        for (std::size_t fi = 0; fi < Bp.face_count(); ++fi) {
            const Face& f = Bp.solid.shell().faces()[fi];
            Face nf(f);  // the copy SHARES the surface's ownership
            for (auto& loop : nf.loops_mut())
                for (auto& ce : loop.coedges_mut())
                    ce.set_edge(static_cast<EdgeId>(ce.edge() + E_base));
            nf.set_id(static_cast<FaceId>(shell.faces().size()));
            shell.add_face(std::move(nf));
        }
        // The inner loops on the copied cutting faces: each cutting
        // face's own circle, split at the paired cap's S/A, the arcs
        // carrying hole=true (the G1 flipped-bits convention).
        for (auto& [pl, cap] : by_plane) {
            if (pl == nullptr) continue;
            const float s = (pl->origin() - sph.center)
                                .dot(pl->normal_vec());
            const float t = pl->normal_vec().dot(a_hat) > 0.0f ? s : -s;
            const float rho =
                std::sqrt(std::max(0.0f, R * R - t * t));
            const math::Vec3f cc = sph.center + a_hat * t;
            Loop inner = rim_loop_on_plane(*out, *pl, cc, rho, cap->S,
                                           cap->A, cap->halves[0],
                                           cap->halves[1],
                                           /*hole=*/true);
            inner.set_outer(false);
            // Find the copied face's index (the tool's order).
            for (std::size_t fi = 0; fi < Bp.face_count(); ++fi) {
                if (Bp.solid.shell().faces()[fi].surface() ==
                    static_cast<const SurfaceGeometry*>(pl)) {
                    shell.faces_mut()[F_base + fi].add_loop(inner);
                    break;
                }
            }
        }
        (void)V_base;
        postconditions(*out, 2, name);
        return out;
    }

    if (op == OpKind::Subtract) {
        // The far caps + the cut disks. The straddling band SPLITS the
        // sphere when both sides are kept: the two cap solids are two
        // shells (chi = 2 each) — the honest answer, the subtract's
        // own (the union would have refused the disconnection).
        const std::int64_t pieces = (lo_cuts ? 1 : 0) + (hi_cuts ? 1 : 0);
        const std::int64_t chi_expected = 2 * pieces;
        if (lo_cuts) {
            const CapPieces c = add_kept_cap(a_hat * -1.0f, T_lo, nullptr);
            const float rho =
                std::sqrt(std::max(0.0f, R * R - T_lo * T_lo));
            const math::Vec3f cc = sph.center + a_hat * T_lo;
            const PlaneSurface pl(cc, a_hat);  // the outward +a_hat
            Face disk(std::make_shared<PlaneSurface>(pl));
            disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, c.S, c.A,
                                            c.halves[0], c.halves[1],
                                            /*hole=*/false));
            disk.set_id(static_cast<FaceId>(shell.faces().size()));
            disk.set_provenance(prov);
            shell.add_face(std::move(disk));
        }
        if (hi_cuts) {
            const CapPieces c = add_kept_cap(a_hat, T_hi, nullptr);
            const float rho =
                std::sqrt(std::max(0.0f, R * R - T_hi * T_hi));
            const math::Vec3f cc = sph.center + a_hat * T_hi;
            const PlaneSurface pl(cc, a_hat * -1.0f);
            Face disk(std::make_shared<PlaneSurface>(pl));
            disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, c.S, c.A,
                                            c.halves[0], c.halves[1],
                                            /*hole=*/false));
            disk.set_id(static_cast<FaceId>(shell.faces().size()));
            disk.set_provenance(prov);
            shell.add_face(std::move(disk));
        }
        postconditions(*out, chi_expected, name);
        return out;
    }

    // INTERSECT: the near band — a single cap (one bound interior) or
    // the zone + two disks (both).
    if (lo_cuts && hi_cuts) {
        // The zone: v in [v1, v2] with v1 = arccos(T_hi/R) (the +a_hat
        // side), v2 = arccos(T_lo/R). The azimuth pairs with the
        // v1-side disk (normal +a_hat): pole.n = +1 -> -u_ax.
        const PlaneSurface pl_hi(sph.center + a_hat * T_hi, a_hat);
        const math::Vec3f e0 = pl_hi.u_axis() * -1.0f;
        const SphereSurface zonef(R, sph.center, a_hat, e0);
        const float v1 = static_cast<float>(
            std::acos(std::clamp(double(T_hi) / double(R), -1.0, 1.0)));
        const float v2 = static_cast<float>(
            std::acos(std::clamp(double(T_lo) / double(R), -1.0, 1.0)));
        const VertexId N0 = bd.vertex(zonef.evaluate(0.0f, v1));
        const VertexId A0 = bd.vertex(zonef.evaluate(kPi, v1));
        const VertexId S0 = bd.vertex(zonef.evaluate(0.0f, v2));
        const VertexId A0p = bd.vertex(zonef.evaluate(kPi, v2));
        const EdgeId seam = bd.edge(N0, S0);
        const EdgeId topW1 = bd.edge(S0, A0p);
        const EdgeId topW2 = bd.edge(A0p, S0);
        const EdgeId botE1 = bd.edge(N0, A0);
        const EdgeId botE2 = bd.edge(A0, N0);
        const auto zh = add_zone_face(shell, zonef, v1, v2, N0, A0, S0,
                                      A0p, seam, topW1, topW2, botE1,
                                      botE2, prov);
        // The disks at both bounds (the outward pointing OUT of the
        // band: -a_hat at T_lo, +a_hat at T_hi).
        {
            const float rho =
                std::sqrt(std::max(0.0f, R * R - T_lo * T_lo));
            const math::Vec3f cc = sph.center + a_hat * T_lo;
            const PlaneSurface pl(cc, a_hat * -1.0f);
            Face disk(std::make_shared<PlaneSurface>(pl));
            disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, N0, A0,
                                            zh[2], zh[3],
                                            /*hole=*/false));
            disk.set_id(static_cast<FaceId>(shell.faces().size()));
            disk.set_provenance(prov);
            shell.add_face(std::move(disk));
        }
        {
            const float rho =
                std::sqrt(std::max(0.0f, R * R - T_hi * T_hi));
            const math::Vec3f cc = sph.center + a_hat * T_hi;
            const PlaneSurface pl(cc, a_hat);
            Face disk(std::make_shared<PlaneSurface>(pl));
            disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, S0, A0p,
                                            zh[0], zh[1],
                                            /*hole=*/false));
            disk.set_id(static_cast<FaceId>(shell.faces().size()));
            disk.set_provenance(prov);
            shell.add_face(std::move(disk));
        }
        postconditions(*out, 2, name);
        return out;
    }
    if (lo_cuts) {
        // The cap around +a_hat at T_lo ({t >= T_lo}).
        const CapPieces c = add_kept_cap(a_hat, T_lo, nullptr);
        const float rho = std::sqrt(std::max(0.0f, R * R - T_lo * T_lo));
        const math::Vec3f cc = sph.center + a_hat * T_lo;
        const PlaneSurface pl(cc, a_hat * -1.0f);
        Face disk(std::make_shared<PlaneSurface>(pl));
        disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, c.S, c.A,
                                        c.halves[0], c.halves[1],
                                        /*hole=*/false));
        disk.set_id(static_cast<FaceId>(shell.faces().size()));
        disk.set_provenance(prov);
        shell.add_face(std::move(disk));
        postconditions(*out, 2, name);
        return out;
    }
    // The cap around -a_hat at T_hi ({t <= T_hi}).
    {
        const CapPieces c = add_kept_cap(a_hat * -1.0f, T_hi, nullptr);
        const float rho = std::sqrt(std::max(0.0f, R * R - T_hi * T_hi));
        const math::Vec3f cc = sph.center + a_hat * T_hi;
        const PlaneSurface pl(cc, a_hat);
        Face disk(std::make_shared<PlaneSurface>(pl));
        disk.add_loop(rim_loop_on_plane(*out, pl, cc, rho, c.S, c.A,
                                        c.halves[0], c.halves[1],
                                        /*hole=*/false));
        disk.set_id(static_cast<FaceId>(shell.faces().size()));
        disk.set_provenance(prov);
        shell.add_face(std::move(disk));
        postconditions(*out, 2, name);
        return out;
    }
}

// The sphere x prism dispatch: the pair enumeration (the entry
// contract), the dispositions, the paths.
std::unique_ptr<BRepBody> sphere_prism_op(const BRepBody& A,
                                          const BRepBody& Bp, OpKind op,
                                          float tol) {
    const char* name = op_name(op);
    std::string why;
    FaceId sface = 0;
    if (!is_sphere_class_body(A, why, sface))
        throw std::invalid_argument(
            std::string(name) + ": " + why +
            " - the sphere class takes the single-sphere body (the "
            "factory sphere or this unit's own survivors). Remedy: "
            "pass the factory sphere as the target, or accept the "
            "faceted path.");
    const SphereRec sph =
        read_sphere_face(A.solid.shell().faces()[static_cast<std::size_t>(
            sface)]);
    const PrismTool tool = read_prism_tool(Bp, tol);
    const float R = sph.radius;

    // (1) The cylinder walls: the interaction, then the coaxial or
    // the Viviani wall.
    for (const auto& w : tool.walls) {
        if (w.cyl == nullptr) continue;
        const math::Vec2f ac = w.cyl->axis_center();
        const float rho_c =
            std::hypot(sph.center.x - ac.x, sph.center.y - ac.y);
        const float r = w.cyl->radius();
        const bool z_over = !(sph.center.z + R < tool.z_lo - tol ||
                              sph.center.z - R > tool.z_hi + tol);
        if (!(std::abs(rho_c - r) <= R + tol && z_over)) continue;
        if (rho_c > tol)
            throw std::invalid_argument(
                std::string(name) +
                ": cylinder×sphere non-coaxial (the sphere's center "
                "sits " +
                num(rho_c) +
                " off the tool's axis): the intersection is the "
                "Viviani-class quartic, and the p-curves are spherical "
                "sinusoids in BOTH domains - typed in neither, the "
                "both-domain criterion's own exhibit (ADR-0033 D2). "
                "Remedy: reorient the operands (coaxial), accept the "
                "faceted path (the bridge's own world), or await G3's "
                "splines.");
        if (!tool.is_circle)
            throw std::invalid_argument(
                std::string(name) +
                ": the tool's ARC wall cuts the sphere coaxially - a "
                "PARTIAL cylinder: the latitude circles would bound a "
                "region the arc's own window does not cover (the kept "
                "spherical piece is not latitude-bounded), outside "
                "this unit's class. Remedy: use the full circle "
                "profile (the bead family) or the segment profile "
                "(the truncation family), or accept the faceted path.");
        demand_budget(sph, math::Vec3f{0, 0, 1}, tol, name);
        return sphere_prism_coaxial(sph, tool, op, tol);
    }

    // (2) The plane faces: the offset classification.
    bool miss = false, contains_all = true, tangent_contain = false;
    std::vector<const PlaneSurface*> cutting;
    const auto classify = [&](const PlaneSurface* pl) {
        const float s =
            (pl->origin() - sph.center).dot(pl->normal_vec());
        if (s <= -R + tol) {
            miss = true;
            contains_all = false;
        } else if (s >= R - tol) {
            if (s <= R + tol) tangent_contain = true;
        } else {
            cutting.push_back(pl);
            contains_all = false;
        }
    };
    for (const auto& w : tool.walls)
        if (w.plane != nullptr) classify(w.plane);
    if (tool.cap_hi != nullptr) classify(tool.cap_hi);
    if (tool.cap_lo != nullptr) classify(tool.cap_lo);
    // The cylinder walls that do not interact: their radial relation.
    for (const auto& w : tool.walls) {
        if (w.cyl == nullptr) continue;
        const math::Vec2f ac = w.cyl->axis_center();
        const float rho_c =
            std::hypot(sph.center.x - ac.x, sph.center.y - ac.y);
        const float r = w.cyl->radius();
        if (rho_c - R >= r - tol) {
            miss = true;          // the sphere outside the cylinder
            contains_all = false;
        } else if (rho_c + R > r + tol) {
            contains_all = false; // the radial middle without a cut
        }
    }

    if (miss) {
        if (op == OpKind::Subtract) {
            auto out = std::make_unique<BRepBody>(A);
            link_radial_edges(*out);
            return out;
        }
        if (op == OpKind::Union)
            throw std::invalid_argument(
                std::string(name) +
                ": the tool is DISJOINT from the sphere (or touches it "
                "within the declared tolerance) - the disconnected "
                "union is not a single body. Remedy: move the operands "
                "to overlap, or keep both (the tree's own answer).");
        throw std::invalid_argument(
            std::string(name) +
            ": the tool is DISJOINT from the sphere (or touches it "
            "within the declared tolerance) - the intersection is the "
            "EMPTY solid. Remedy: move the operands to overlap, or "
            "accept the empty result by not intersecting.");
    }
    if (contains_all) {
        if (op == OpKind::Subtract) {
            if (tangent_contain) {
                // The touch removes nothing (measure zero).
                auto out = std::make_unique<BRepBody>(A);
                link_radial_edges(*out);
                return out;
            }
            throw std::invalid_argument(
                std::string(name) +
                ": the tool SWALLOWS the sphere (every face's "
                "half-space contains it) - the exact result is the "
                "EMPTY solid. Remedy: use a smaller tool, or accept "
                "the empty result by not subtracting.");
        }
        if (op == OpKind::Union) {
            auto out = std::make_unique<BRepBody>(Bp);
            link_radial_edges(*out);
            return out;
        }
        auto out = std::make_unique<BRepBody>(A);
        link_radial_edges(*out);
        return out;
    }

    // (3) The cutting set: the family gate.
    for (std::size_t i = 0; i < cutting.size(); ++i)
        for (std::size_t j = i + 1; j < cutting.size(); ++j) {
            const float c = std::abs(cutting[i]->normal_vec()
                                         .dot(cutting[j]->normal_vec()));
            if (c < 1.0f - tol)
                throw std::invalid_argument(
                    std::string(name) +
                    ": the frame budget - the tool's cutting faces "
                    "span TWO axis families (one at (" +
                    num(cutting[i]->normal_vec().x) + ", " +
                    num(cutting[i]->normal_vec().y) + ", " +
                    num(cutting[i]->normal_vec().z) + "), another at (" +
                    num(cutting[j]->normal_vec().x) + ", " +
                    num(cutting[j]->normal_vec().y) + ", " +
                    num(cutting[j]->normal_vec().z) +
                    ")): one frame = one axis family per face "
                    "(ADR-0033 D1.1) - two general-direction cuts of "
                    "one sphere have no common typed frame, and the "
                    "boundary says so once, for every entry. Remedy: "
                    "reorient the tool so one family cuts (the "
                    "truncation or the slab), accept the faceted "
                    "path, or await the multi-cut unit.");
        }
    const math::Vec3f a_hat = cutting[0]->normal_vec().normalized();
    demand_budget(sph, a_hat, tol, name);
    return sphere_prism_planes(A, Bp, sph, tool, op, tol, cutting, name);
}

} // namespace

// ---------------------------------------------------------------------------
// The public entries
// ---------------------------------------------------------------------------

std::unique_ptr<BRepBody> exact_union(const BRepBody& A, const BRepBody& B,
                                      math::Vec3f tool_offset, float tol) {
    demand_exact(A, "target", "exact_union");
    demand_exact(B, "tool", "exact_union");
    auto bp = std::make_unique<BRepBody>(B);
    translate_body(*bp, tool_offset);

    const SphereSurface* sa = single_sphere_face(A);
    const SphereSurface* sb = single_sphere_face(*bp);
    if (sa != nullptr && sb != nullptr)
        return sphere_pair_op(A, *bp, OpKind::Union, tol);
    if (sa != nullptr) return sphere_prism_op(A, *bp, OpKind::Union, tol);
    if (sb != nullptr)
        throw std::invalid_argument(
            "exact_union: the prism TARGET with a sphere tool - the "
            "extended class takes the SPHERE as the target (the prism "
            "is the tool, the D6 class). Remedy: swap the operands, "
            "or accept the faceted path.");
    throw std::invalid_argument(
        "exact_union: a prism×prism pair - the polyhedral union is "
        "Phase D's LANDED opinion (bool_union on the convex polyhedra, "
        "ADR-0024/0025) and the exact extension does not duplicate a "
        "landed opinion (the ssi table's own rule for plane×plane). "
        "Remedy: call bool_union, or await the SSI-driven general "
        "union of a future phase.");
}

std::unique_ptr<BRepBody> exact_intersect(const BRepBody& A,
                                          const BRepBody& B,
                                          math::Vec3f tool_offset,
                                          float tol) {
    demand_exact(A, "target", "exact_intersect");
    demand_exact(B, "tool", "exact_intersect");
    auto bp = std::make_unique<BRepBody>(B);
    translate_body(*bp, tool_offset);

    const SphereSurface* sa = single_sphere_face(A);
    const SphereSurface* sb = single_sphere_face(*bp);
    if (sa != nullptr && sb != nullptr)
        return sphere_pair_op(A, *bp, OpKind::Intersect, tol);
    if (sa != nullptr)
        return sphere_prism_op(A, *bp, OpKind::Intersect, tol);
    if (sb != nullptr)
        throw std::invalid_argument(
            "exact_intersect: the prism TARGET with a sphere tool - "
            "the extended class takes the SPHERE as the target (the "
            "prism is the tool, the D6 class). Remedy: swap the "
            "operands, or accept the faceted path.");
    throw std::invalid_argument(
        "exact_intersect: a prism×prism pair - the polyhedral "
        "intersection is Phase D's LANDED opinion (bool_intersect on "
        "the convex polyhedra, ADR-0024/0025); the exact extension "
        "does not duplicate a landed opinion. Remedy: call "
        "bool_intersect, or await the SSI-driven general intersect of "
        "a future phase.");
}

std::unique_ptr<BRepBody> exact_subtract_sphere_target(
    const BRepBody& A, const BRepBody& B, math::Vec3f tool_offset,
    float tol) {
    demand_exact(A, "target", "exact_subtract");
    demand_exact(B, "tool", "exact_subtract");
    auto bp = std::make_unique<BRepBody>(B);
    translate_body(*bp, tool_offset);

    if (single_sphere_face(*bp) != nullptr)
        return sphere_pair_op(A, *bp, OpKind::Subtract, tol);
    return sphere_prism_op(A, *bp, OpKind::Subtract, tol);
}

} // namespace CAD_0::brep
