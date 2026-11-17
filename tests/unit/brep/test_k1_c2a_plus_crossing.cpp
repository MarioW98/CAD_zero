// tests/unit/brep/test_k1_c2a_plus_crossing.cpp
//
// K1 C2a — THE PLUS-SHAPED CROSSING'S OWN CONTRACT (the registered
// defect the second cohort caught, now the correction's own suite):
// the overlay walk's fragment was WRONG on the plus-shaped coplanar
// crossing — observed a pentagon of area 5.48 against the plus's
// 6.72, another member an 8-gon of 11.01, LARGER than both operands
// — and the area invariant refused honestly (overlay_region_
// irregular). THE ROOT CAUSE (the correction's own analysis):
// insert_intersection inserted every crossing at the HEAD of its
// edge's chain, so two crossings on ONE edge (the plus: A's top
// edge × B's left and right edges) sat in COMPUTATION order, not
// geometric order — the entry/exit alternation inverted on the
// edge and the walk paired the wrong crossings.
//
// THE CONTRACT, PINNED HERE BEFORE THE CURE (Phase 2's own law):
// the plus belongs to the admitted coplanar-overlay domain — the
// tops are EXACTLY coplanar (the ε_C admission's cleanest case),
// the crossing pattern is four interior transversal points, the
// union region is ONE simply-connected plus (12 outline vertices,
// genus 0), and the honest answer is the merged region with:
//   * the area = |A| + |B| − |A∩B| by THIS FILE'S OWN oracles;
//   * every boundary vertex ON ∂A ∪ ∂B (the fragment ownership);
//   * the closed, orientable, radial, χ = 2 body;
//   * the volume by the independent form;
//   * the operand reversal and the rebuild bit-identical.
// THE INDEPENDENT ORACLE: this file's OWN Sutherland–Hodgman
// convex clip (zero production code) for the 2D intersection, the
// shoelace areas, the AABB inclusion-exclusion for the axis-aligned
// volumes, and the z-matched closed form V = V(A)+V(B)−|A∩B|·2h for
// the rotated members — never the production walk, clip, classifier
// or pool.
//
// THE CORPUS (the correction's own adversarial diet):
//   1. the exact red verbatim (the cohort's fuzz member, preserved);
//   2. the δ band both sides: |δ| ≤ ε_L the plus overlay (the tops
//      within the composite window, welded), |δ| > ε_L the stepped
//      transversal body (the plus geometry has NO coplanar side
//      pairs — the mixed-class seam never enters);
//   3. the operand reversal;
//   4. the translation (the same pair at (5, −3, 2));
//   5. the in-plane rotation (30°, 45°, 37° — the crossing pattern
//      rotated, the walk's projected frame doing the work);
//   6. the scales 0.1× and 10× (the SAME absolute ε_L);
//   7. the near-coincident crossings (the thin tongue: the two
//      crossings on one edge 2·10⁻³ .. 2·10⁻⁵ apart — all ≫ ε_L,
//      the resolution's own derived margin);
//   8. the coincident crossing (the rotated corner exactly ON A's
//      top edge — the vertex-on-edge degenerate): the observed and
//      pinned disposition is the honest overlay_region_irregular
//      refusal (the endpoint-crossing class is a different seat,
//      registered);
//   9. the repeated execution: the census/area digest FNV-pinned,
//      bit-identical forever (ADR-0014).
#include <doctest/doctest.h>

#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/detail/boolean_test_hooks.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/math/vec.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace geo = CAD_0::geometry;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

// The pinned FNV-1a of the corpus digest (filled at the correction's
// own first green run; never moves again).
constexpr std::uint64_t kPinnedCorpusHash = 0x4ad12b37ba16f7a7ull;

// THE DECLARED TOLERANCES — the consumers' own, never a private one.
const double kEpsL = b::kBoolLinear.value;  // 1e-6, the ε_C window's
                                            // linear member

// ---------------------------------------------------------------------------
// THE INDEPENDENT 2D ORACLE — this file's own mathematics
// ---------------------------------------------------------------------------

double my_shoelace(const std::vector<Vec2d>& p) {
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2d& u = p[i];
        const Vec2d& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * a;
}

// MY OWN Sutherland–Hodgman: clip the convex polygon `subject` by
// the convex polygon `clip` (each clip edge's inside half-plane) —
// zero production code, the classical algorithm by name.
std::vector<Vec2d> my_convex_clip(const std::vector<Vec2d>& subject,
                                  const std::vector<Vec2d>& clip) {
    std::vector<Vec2d> out = subject;
    const std::size_t n = clip.size();
    for (std::size_t i = 0; i < n && out.size() >= 3; ++i) {
        const Vec2d& a = clip[i];
        const Vec2d& b = clip[(i + 1) % n];
        // CCW clip: the inside is to the LEFT of a→b.
        const auto side = [&](const Vec2d& p) {
            return (b.x - a.x) * (p.y - a.y)
                 - (b.y - a.y) * (p.x - a.x);
        };
        std::vector<Vec2d> next;
        for (std::size_t j = 0; j < out.size(); ++j) {
            const Vec2d& cur = out[j];
            const Vec2d& prv = out[(j + out.size() - 1) % out.size()];
            const double sc = side(cur), sp = side(prv);
            const bool cin = sc >= 0.0, pin = sp >= 0.0;
            if (cin) {
                if (!pin) {  // entering: emit the crossing first
                    const double t = sp / (sp - sc);
                    next.push_back(Vec2d{prv.x + t * (cur.x - prv.x),
                                         prv.y + t * (cur.y - prv.y)});
                }
                next.push_back(cur);
            } else if (pin) {  // leaving: emit the crossing
                const double t = sp / (sp - sc);
                next.push_back(Vec2d{prv.x + t * (cur.x - prv.x),
                                     prv.y + t * (cur.y - prv.y)});
            }
        }
        out = std::move(next);
    }
    return out;
}

// |A ∩ B| for convex polygons — my clip, my shoelace.
double my_intersection_area(const std::vector<Vec2d>& a,
                            const std::vector<Vec2d>& b) {
    const auto inter = my_convex_clip(a, b);
    if (inter.size() < 3) return 0.0;
    return std::abs(my_shoelace(inter));
}

// The union area by inclusion–exclusion — MY form.
double my_union_area(const std::vector<Vec2d>& a,
                     const std::vector<Vec2d>& b) {
    return std::abs(my_shoelace(a)) + std::abs(my_shoelace(b))
         - my_intersection_area(a, b);
}

// The perimeter — the audit's own tube charge (the derived band's
// linear term rides the EXACT rims: the operands' vertices are
// original bytes, no canonicalization ever touches them).
double my_perimeter(const std::vector<Vec2d>& p) {
    double l = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2d& u = p[i];
        const Vec2d& v = p[(i + 1) % p.size()];
        l += std::hypot(v.x - u.x, v.y - u.y);
    }
    return l;
}

// The boundary ownership: every point of the merged region lies on
// ∂A ∪ ∂B (within the declared ε_L — the crossings are computed ON
// the edges, the original vertices are the polygons' own).
bool my_point_on_boundary(const Vec2d& p,
                          const std::vector<Vec2d>& poly) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2d& a = poly[i];
        const Vec2d& b = poly[(i + 1) % poly.size()];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 0.0
                 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2
                 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        const double ex = p.x - (a.x + t * dx);
        const double ey = p.y - (a.y + t * dy);
        if (ex * ex + ey * ey <= kEpsL * kEpsL) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// The authoring (the cohort-2 suite's own discipline: the p-curve
// trimmings authored with the factory's own helper — the exact
// world's invariant)
// ---------------------------------------------------------------------------

std::unique_ptr<b::BRepBody> make_box_at(Vec3d e, Vec3d c) {
    auto body = std::make_unique<b::BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);
    const double x = e.x, y = e.y, z = e.z;
    const double cx = c.x, cy = c.y, cz = c.z;
    body->vertices = {
        b::Vertex{{cx-x,cy-y,cz-z}}, b::Vertex{{cx+x,cy-y,cz-z}},
        b::Vertex{{cx+x,cy+y,cz-z}}, b::Vertex{{cx-x,cy+y,cz-z}},
        b::Vertex{{cx-x,cy-y,cz+z}}, b::Vertex{{cx+x,cy-y,cz+z}},
        b::Vertex{{cx+x,cy+y,cz+z}}, b::Vertex{{cx-x,cy+y,cz+z}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));
    body->edges = {
        b::Edge{0,1}, b::Edge{1,2}, b::Edge{2,3}, b::Edge{3,0},
        b::Edge{4,5}, b::Edge{5,6}, b::Edge{6,7}, b::Edge{7,4},
        b::Edge{0,4}, b::Edge{1,5}, b::Edge{2,6}, b::Edge{3,7},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));
    auto add_plane_face = [&shell, &body](
        Vec3d origin, Vec3d normal, b::FaceId id,
        b::EdgeId e0, bool r0, b::EdgeId e1, bool r1,
        b::EdgeId e2, bool r2, b::EdgeId e3, bool r3) {
        auto surf = std::make_shared<b::PlaneSurface>(origin, normal);
        b::Face face(surf);
        b::Loop loop;
        loop.add_coedge(b::detail::line_coedge(e0, r0, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e1, r1, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e2, r2, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e3, r3, *surf,
                                               body->vertices,
                                               body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };
    add_plane_face({cx,cy,cz-z},{0,0,-1},0, 3,true,2,true,1,true,0,true);
    add_plane_face({cx,cy,cz+z},{0,0,1},1, 4,false,5,false,6,false,7,false);
    add_plane_face({cx,cy-y,cz},{0,-1,0},2, 0,false,9,false,4,true,8,true);
    add_plane_face({cx,cy+y,cz},{0,1,0},3, 2,false,11,false,6,true,10,true);
    add_plane_face({cx-x,cy,cz},{-1,0,0},4, 3,false,8,false,7,true,11,true);
    add_plane_face({cx+x,cy,cz},{1,0,0},5, 1,false,10,false,5,true,9,true);
    b::link_radial_edges(*body);
    return body;
}

// The IN-PLANE ROTATED box (the corpus's rotation members): the
// unit-box topology under a rotation about Z — every face a planar
// parallelogram-ish quad, convex, the winding preserved (a rotation
// is orientation-preserving). The p-curve discipline the same.
std::unique_ptr<b::BRepBody> rot_box(Vec3d half, Vec3d center,
                                     double ang) {
    const double ca = std::cos(ang), sa = std::sin(ang);
    auto body = std::make_unique<b::BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);
    const auto R = [&](double sx, double sy, double sz) {
        const double lx = sx * half.x, ly = sy * half.y;
        return Vec3d{center.x + lx * ca - ly * sa,
                     center.y + lx * sa + ly * ca,
                     center.z + sz * half.z};
    };
    body->vertices = {
        b::Vertex{R(-1,-1,-1)}, b::Vertex{R(1,-1,-1)},
        b::Vertex{R(1,1,-1)},   b::Vertex{R(-1,1,-1)},
        b::Vertex{R(-1,-1,1)},  b::Vertex{R(1,-1,1)},
        b::Vertex{R(1,1,1)},    b::Vertex{R(-1,1,1)},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));
    body->edges = {
        b::Edge{0,1}, b::Edge{1,2}, b::Edge{2,3}, b::Edge{3,0},
        b::Edge{4,5}, b::Edge{5,6}, b::Edge{6,7}, b::Edge{7,4},
        b::Edge{0,4}, b::Edge{1,5}, b::Edge{2,6}, b::Edge{3,7},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));
    // Each face's plane: the rotated quad's own frame (origin at the
    // quad's center, the normal by the right-hand rule over its
    // edges — computed from the vertices, never assumed).
    auto add_face = [&shell, &body](
        const Vec3d& v0, const Vec3d& v1, const Vec3d& v2,
        const Vec3d& v3, b::FaceId id,
        b::EdgeId e0, bool r0, b::EdgeId e1, bool r1,
        b::EdgeId e2, bool r2, b::EdgeId e3, bool r3) {
        const Vec3d n =
            (v1 - v0).cross(v2 - v0);  // CCW quad: the outward
        const double nl =
            1.0 / std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        const Vec3d nn{n.x * nl, n.y * nl, n.z * nl};
        const Vec3d o{0.25 * (v0.x + v1.x + v2.x + v3.x),
                      0.25 * (v0.y + v1.y + v2.y + v3.y),
                      0.25 * (v0.z + v1.z + v2.z + v3.z)};
        auto surf = std::make_shared<b::PlaneSurface>(o, nn);
        b::Face face(surf);
        b::Loop loop;
        loop.add_coedge(b::detail::line_coedge(e0, r0, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e1, r1, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e2, r2, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e3, r3, *surf,
                                               body->vertices,
                                               body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };
    const auto& V = body->vertices;
    const auto P = [](const b::Vertex& v) { return v.position(); };
    // make_box's own (edge, reversed) tables, and each face's plane
    // computed from ITS OWN traversal cycle (the cycles derived from
    // the tables: bottom (0,3,2,1), top (4,5,6,7), −Y (0,1,5,4),
    // +Y (2,3,7,6), −X (3,0,4,7), +X (1,2,6,5)).
    add_face(P(V[0]), P(V[3]), P(V[2]), P(V[1]), 0,
             3, true, 2, true, 1, true, 0, true);   // bottom
    add_face(P(V[4]), P(V[5]), P(V[6]), P(V[7]), 1,
             4, false, 5, false, 6, false, 7, false);  // top
    add_face(P(V[0]), P(V[1]), P(V[5]), P(V[4]), 2,
             0, false, 9, false, 4, true, 8, true);   // -Y-ish
    add_face(P(V[2]), P(V[3]), P(V[7]), P(V[6]), 3,
             2, false, 11, false, 6, true, 10, true);  // +Y-ish
    add_face(P(V[3]), P(V[0]), P(V[4]), P(V[7]), 4,
             3, false, 8, false, 7, true, 11, true);  // -X-ish
    add_face(P(V[1]), P(V[2]), P(V[6]), P(V[5]), 5,
             1, false, 10, false, 5, true, 9, true);  // +X-ish
    b::link_radial_edges(*body);
    return body;
}

// ---------------------------------------------------------------------------
// The verification helpers (the cohort-2 suite's own independent
// forms, verbatim in spirit: Newell for the areas/normals, the
// divergence theorem for the planar volumes — never the production
// mass read, which rightly refuses the untrimmed boolean world)
// ---------------------------------------------------------------------------

std::pair<Vec3d, double> face_normal_area(const b::BRepBody& body,
                                          b::FaceId fid) {
    const auto& face = body.solid.shell().faces()[fid];
    Vec3d n{0.0, 0.0, 0.0};
    for (const auto& lp : face.loops()) {
        for (const auto& ce : lp.coedges()) {
            const auto& e = body.edges[ce.edge()];
            const Vec3d p =
                body.vertices[static_cast<std::size_t>(e.vertex(0))]
                    .position();
            const Vec3d q =
                body.vertices[static_cast<std::size_t>(e.vertex(1))]
                    .position();
            const Vec3d a = ce.reversed() ? q : p;
            const Vec3d b = ce.reversed() ? p : q;
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
    }
    const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    return {n, len / 2.0};
}

double poly_volume(const b::BRepBody& body) {
    double total = 0.0;
    for (b::FaceId fid = 0; fid < body.face_count(); ++fid) {
        const auto [n, twice] = face_normal_area(body, fid);
        if (twice < 1e-18) continue;
        const double inv = 1.0 / std::sqrt(n.x * n.x + n.y * n.y
                                           + n.z * n.z);
        const Vec3d nn{n.x * inv, n.y * inv, n.z * inv};
        Vec3d ax = (std::abs(nn.x) < 0.9) ? Vec3d{1, 0, 0}
                                          : Vec3d{0, 0, 1};
        Vec3d e1 = nn.cross(ax);
        const double e1n =
            1.0 / std::sqrt(e1.x * e1.x + e1.y * e1.y + e1.z * e1.z);
        e1 = {e1.x * e1n, e1.y * e1n, e1.z * e1n};
        const Vec3d e2 = nn.cross(e1);
        const auto& face = body.solid.shell().faces()[fid];
        for (const auto& lp : face.loops()) {
            std::vector<Vec3d> pts;
            for (const auto& ce : lp.coedges()) {
                const auto& e = body.edges[ce.edge()];
                const std::size_t vi = static_cast<std::size_t>(
                    ce.reversed() ? e.vertex(1) : e.vertex(0));
                pts.push_back(body.vertices[vi].position());
            }
            if (pts.size() < 3) continue;
            const Vec3d o = pts[0];
            double A = 0.0, Mu = 0.0, Mv = 0.0;
            for (std::size_t i = 0; i < pts.size(); ++i) {
                const Vec3d& p = pts[i];
                const Vec3d& q = pts[(i + 1) % pts.size()];
                const Vec3d dp = p - o, dq = q - o;
                const double ui = dp.dot(e1), vi2 = dp.dot(e2);
                const double uj = dq.dot(e1), vj = dq.dot(e2);
                const double cr = ui * vj - uj * vi2;
                A += 0.5 * cr;
                Mu += (ui + uj) * cr / 6.0;
                Mv += (vi2 + vj) * cr / 6.0;
            }
            total += (o.dot(nn) * A + e1.dot(nn) * Mu
                      + e2.dot(nn) * Mv) / 3.0;
        }
    }
    return total;
}

bool radial_opposite(const b::BRepBody& body) {
    for (const auto& e : body.edges) {
        if (e.coedges().size() != 2) return false;
        if (e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            return false;
    }
    return true;
}

// THE MEMBER VERIFICATION — an admitted union's own contract: the
// structural gates, the merged top region against MY area oracle
// (the no-duplicated/no-missing witness: a duplicated region would
// exceed it, a missing one fall short), the boundary ownership
// (every top-fragment vertex on ∂A ∪ ∂B), and the volume.
void verify_union_member(const b::BRepBody& U, double z_top,
                         const std::vector<Vec2d>& a2d,
                         const std::vector<Vec2d>& b2d,
                         double v_oracle, double v_bound,
                         const char* what) {
    INFO(what << ": V=" << U.vertex_count()
              << " E=" << U.edge_count()
              << " F=" << U.face_count());
    CHECK(b::is_closed_orientable(U));
    CHECK(b::loops_are_closed(U));
    CHECK(radial_opposite(U));
    CHECK(b::euler_characteristic(U) == 2);
    const double area_oracle = my_union_area(a2d, b2d);
    double top_area = 0.0;
    std::size_t top_faces = 0;
    for (b::FaceId f = 0; f < U.face_count(); ++f) {
        const auto [n, area] = face_normal_area(U, f);
        if (n.z <= 0.9 * (2.0 * area) || area < 1e-12) continue;
        // the top quota: all vertices at z_top (within ε_L — the
        // welded band's own scale)
        bool at_top = true;
        for (const auto& lp :
             U.solid.shell().faces()[f].loops())
            for (const auto& ce : lp.coedges()) {
                const auto& e = U.edges[ce.edge()];
                for (const auto vi : {e.vertex(0), e.vertex(1)}) {
                    const double z =
                        U.vertices[static_cast<std::size_t>(vi)]
                            .position().z;
                    if (std::abs(z - z_top) > 4.0 * kEpsL)
                        at_top = false;
                }
            }
        if (!at_top) continue;
        ++top_faces;
        top_area += area;
        // THE OWNERSHIP: every vertex on ∂A ∪ ∂B.
        for (const auto& lp :
             U.solid.shell().faces()[f].loops())
            for (const auto& ce : lp.coedges()) {
                const auto& e = U.edges[ce.edge()];
                for (const auto vi : {e.vertex(0), e.vertex(1)}) {
                    const Vec3d p =
                        U.vertices[static_cast<std::size_t>(vi)]
                            .position();
                    const Vec2d p2{p.x, p.y};
                    CHECK((my_point_on_boundary(p2, a2d)
                           || my_point_on_boundary(p2, b2d)));
                }
            }
    }
    INFO(what << ": top_faces=" << top_faces
              << " top_area=" << top_area
              << " oracle=" << area_oracle);
    CHECK(top_faces >= 1);
    CHECK(std::abs(top_area - area_oracle) <= 1e-9);
    const double vol = poly_volume(U);
    INFO(what << ": volume " << vol << " oracle " << v_oracle);
    CHECK(std::abs(vol - v_oracle) <= v_bound);
}

// The AABB forms (the axis-aligned members' own volume oracle).
double aabb_union_volume(Vec3d ea, Vec3d cA, Vec3d eb, Vec3d cB) {
    const double lia[3] = {cA.x - ea.x, cA.y - ea.y, cA.z - ea.z};
    const double hia[3] = {cA.x + ea.x, cA.y + ea.y, cA.z + ea.z};
    const double lib[3] = {cB.x - eb.x, cB.y - eb.y, cB.z - eb.z};
    const double hib[3] = {cB.x + eb.x, cB.y + eb.y, cB.z + eb.z};
    double va = 1.0, vb = 1.0, vi = 1.0;
    for (int i = 0; i < 3; ++i) {
        va *= hia[i] - lia[i];
        vb *= hib[i] - lib[i];
        const double lo = std::max(lia[i], lib[i]);
        const double hi = std::min(hia[i], hib[i]);
        vi *= (hi > lo) ? (hi - lo) : 0.0;
    }
    return va + vb - vi;
}

}  // namespace

TEST_SUITE("K1 C2a — the plus crossing's contract") {
// The refusal bundles (the cohort-2 suite's own forms, verbatim in
// spirit: the result gate's runtime_error with the class token
// carried, and the source operands' digest immutability).
template <typename Fn>
std::string expect_output_refusal(Fn&& fn, const std::string& token,
                                  const char* what) {
    bool threw = false;
    std::string msg;
    try {
        fn();
    } catch (const std::runtime_error& e) {
        threw = true;
        msg = e.what();
    } catch (...) {
        MESSAGE(what << ": the WRONG exception type");
        CHECK(false);
    }
    INFO(what << ": " << msg);
    REQUIRE(threw);
    CHECK(msg.find(token) != std::string::npos);
    return msg;
}

template <typename Fn>
bool refused_leaves_source(const b::BRepBody& body, Fn&& fn) {
    const std::string before = b::chain_digest(body);
    try { fn(); } catch (...) {}
    return b::chain_digest(body) == before;
}

// The 2D rectangle (axis-aligned, centered) as the oracle's polygon.
std::vector<Vec2d> rect2d(double hx, double hy, double cx, double cy) {
    return {{cx - hx, cy - hy}, {cx + hx, cy - hy},
            {cx + hx, cy + hy}, {cx - hx, cy + hy}};
}

// The rotated rectangle's vertices (the oracle's own math).
std::vector<Vec2d> rot_rect2d(double hx, double hy, double cx,
                              double cy, double ang) {
    const double ca = std::cos(ang), sa = std::sin(ang);
    std::vector<Vec2d> out;
    for (const auto& [sx, sy] :
         std::vector<std::pair<double, double>>{
             {-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
        const double lx = sx * hx, ly = sy * hy;
        out.push_back({cx + lx * ca - ly * sa,
                       cy + lx * sa + ly * ca});
    }
    return out;
}

TEST_CASE("C2a the exact red, verbatim — the plus crossing's own "
          "contract, ADMITTED") {
    // The cohort-2 fuzz member (seed 20261117, member 4, δ = 0),
    // preserved verbatim: B strictly inside A in x, sticking out in
    // ±y; A the converse; the tops EXACTLY flush — the cleanest
    // coplanar crossing there is. BEFORE the correction this member
    // refused with overlay_region_irregular (the walk's own fragment
    // was a pentagon of 5.48 against the plus's 6.72); the contract
    // demands the merged plus, and this pin is the correction's own
    // red.
    const Vec3d eA{1.209195, 1.312911, 1.215093};
    const Vec3d eB{0.855096, 1.420279, 0.714009};
    const Vec3d cB{-0.023041630, 0.033909082, eA.z - eB.z};
    const auto a2d = rect2d(eA.x, eA.y, 0, 0);
    const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
    // THE PLUS PRECONDITION, derived: B inside A in x, A inside B in
    // y (the crossing's own signature — four interior crossings).
    REQUIRE(cB.x - eB.x > -eA.x);
    REQUIRE(cB.x + eB.x < eA.x);
    REQUIRE(cB.y - eB.y < -eA.y);
    REQUIRE(cB.y + eB.y > eA.y);
    auto A = make_box_at(eA, {0, 0, 0});
    auto B = make_box_at(eB, cB);
    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    verify_union_member(*U, eA.z, a2d, b2d,
                        aabb_union_volume(eA, {0, 0, 0}, eB, cB),
                        1e-9, "the exact red");
    // The operand reversal: the same contract the other way.
    auto U2 = b::bool_union(*B, *A, b::kBooleanHouseTolerance);
    REQUIRE(U2 != nullptr);
    CHECK(U2->vertex_count() == U->vertex_count());
    CHECK(U2->edge_count() == U->edge_count());
    CHECK(U2->face_count() == U->face_count());
    CHECK(b::euler_characteristic(*U2) == 2);
    CHECK(std::abs(poly_volume(*U2) - poly_volume(*U)) < 1e-9);
    // The rebuild: bit-identical (ADR-0014).
    auto U3 = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U3 != nullptr);
    CHECK(b::chain_digest(*U) == b::chain_digest(*U3));
}

TEST_CASE("C2a the δ band both sides — the plus inside, the step "
          "beyond") {
    const Vec3d eA{1.209195, 1.312911, 1.215093};
    const Vec3d eB{0.855096, 1.420279, 0.714009};
    const auto a2d = rect2d(eA.x, eA.y, 0, 0);
    // THE BAND'S OWN SCOPE (C2a's law): the crossing's correction
    // owns |δ| < ε_L and δ > ε_L; the |δ| = ε_L members are the
    // MIXED-CLASS SEAM'S knife edge (the walk keeps the exactly-ε_L
    // side strips while the ε_C window welds the tops — the D3
    // seat, RESERVED FOR K1 C2b): pinned here as today's honest
    // refusal, the pin flips with C2b's own landing.
    for (const double k : {-1.0, 1.0}) {
        const double d = k * kEpsL;
        const Vec3d cBk{-0.023041630, 0.033909082,
                        eA.z - eB.z + d};
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cBk);
        bool threw = false;
        std::string msg;
        try {
            auto U = b::bool_union(*A, *B,
                                   b::kBooleanHouseTolerance);
            (void)U;
        } catch (const std::runtime_error& e) {
            threw = true;
            msg = e.what();
        }
        INFO("δ=" << k << "·ε_L (the C2b knife): "
                  << (threw ? msg : "admitted"));
        if (threw) {
            CHECK(msg.find("result_edge_not_manifold")
                  != std::string::npos);
        } else {
            CHECK(b::euler_characteristic(
                      *b::bool_union(*A, *B,
                                     b::kBooleanHouseTolerance))
                  == 2);
        }
    }
    for (const double k : {-0.9, -0.5, 0.5, 0.9}) {
        const double d = k * kEpsL;
        const Vec3d cB{-0.023041630, 0.033909082,
                       eA.z - eB.z + d};
        const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cB);
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        INFO("δ=" << k << "·ε_L: "
                  << (U == nullptr ? "REFUSED" : "admitted"));
        REQUIRE(U != nullptr);
        // The welded plus: the volume within the band's own derived
        // slop of the honest stepped AABB form.
        verify_union_member(*U, eA.z, a2d, b2d,
                            aabb_union_volume(eA, {0, 0, 0}, eB, cB),
                            std::abs(d) * 6.0 + kEpsL * 60.0,
                            "the δ band in");
    }
    for (const double k : {1.5, 2.0}) {
        const double d = k * kEpsL;
        const Vec3d cB{-0.023041630, 0.033909082,
                       eA.z - eB.z + d};
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cB);
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        REQUIRE(U != nullptr);
        INFO("δ=" << k << "·ε_L (beyond): census V="
                  << U->vertex_count() << " E=" << U->edge_count()
                  << " F=" << U->face_count());
        // THE STEP IS REAL GEOMETRY: the two quotas' own areas —
        // A's top minus the overlap at z_top, B's top whole at
        // z_top + δ — and the exact stepped volume.
        CHECK(b::is_closed_orientable(*U));
        CHECK(radial_opposite(*U));
        CHECK(b::euler_characteristic(*U) == 2);
        const double v_step =
            aabb_union_volume(eA, {0, 0, 0}, eB, cB);
        CHECK(std::abs(poly_volume(*U) - v_step) <= 1e-9);
        CHECK(U->face_count() > 6);
    }
}

TEST_CASE("C2a the translation — the same pair at (5, −3, 2)") {
    const Vec3d eA{1.209195, 1.312911, 1.215093};
    const Vec3d eB{0.855096, 1.420279, 0.714009};
    const Vec3d t{5.0, -3.0, 2.0};
    const Vec3d cB{-0.023041630 + t.x, 0.033909082 + t.y,
                   eA.z - eB.z + t.z};
    const auto a2d = rect2d(eA.x, eA.y, t.x, t.y);
    const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
    auto A = make_box_at(eA, t);
    auto B = make_box_at(eB, cB);
    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    verify_union_member(*U, eA.z + t.z, a2d, b2d,
                        aabb_union_volume(eA, t, eB, cB), 1e-9,
                        "the translation");
}

TEST_CASE("C2a the rotations — the crossing pattern rotated and the "
          "mixed-angle diet") {
    // The z-matched construction (both z-spans equal): the volume
    // oracle's closed form V = V(A) + V(B) − |A∩B|·2h for ANY
    // in-plane rotation.
    const double hz = 0.8;
    const Vec3d eA{1.209195, 1.312911, hz};
    const Vec3d eB{0.855096, 1.420279, hz};
    const double rel_x = -0.023041630, rel_y = 0.033909082;
    for (const double deg : {30.0, 45.0}) {
        const double ang = deg * kPi / 180.0;
        const double ca = std::cos(ang), sa = std::sin(ang);
        const Vec3d cB{rel_x * ca - rel_y * sa,
                       rel_x * sa + rel_y * ca, 0.0};
        const auto a2d = rot_rect2d(eA.x, eA.y, 0, 0, ang);
        const auto b2d = rot_rect2d(eB.x, eB.y, cB.x, cB.y, ang);
        auto A = rot_box(eA, {0, 0, 0}, ang);
        auto B = rot_box(eB, cB, ang);
        REQUIRE(b::is_closed_orientable(*A));
        REQUIRE(b::is_closed_orientable(*B));
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        INFO("rotation " << deg << "°: "
                         << (U == nullptr ? "REFUSED" : "admitted"));
        REQUIRE(U != nullptr);
        const double va = 8.0 * eA.x * eA.y * hz;
        const double vb = 8.0 * eB.x * eB.y * hz;
        const double v_oracle =
            va + vb - my_intersection_area(a2d, b2d) * 2.0 * hz;
        verify_union_member(*U, hz, a2d, b2d, v_oracle, 1e-9,
                            "the rotation");
    }
    // THE MIXED ANGLE: A axis-aligned, B at 37° — the richer
    // crossing diet (B's rotated edges against A's axis-aligned
    // ones, up to eight crossings).
    {
        const double ang = 37.0 * kPi / 180.0;
        const Vec3d eB2{0.85, 0.71, hz};
        const Vec3d cB2{0.1, 0.05, 0.0};
        const auto a2d = rect2d(eA.x, eA.y, 0, 0);
        const auto b2d = rot_rect2d(eB2.x, eB2.y, cB2.x, cB2.y, ang);
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = rot_box(eB2, cB2, ang);
        REQUIRE(b::is_closed_orientable(*B));
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        INFO("mixed 37°: "
             << (U == nullptr ? "REFUSED" : "admitted"));
        REQUIRE(U != nullptr);
        const double va = 8.0 * eA.x * eA.y * hz;
        const double vb = 8.0 * eB2.x * eB2.y * hz;
        const double v_oracle =
            va + vb - my_intersection_area(a2d, b2d) * 2.0 * hz;
        verify_union_member(*U, hz, a2d, b2d, v_oracle, 1e-9,
                            "the mixed angle");
    }
}

TEST_CASE("C2a the scales — 0.1× and 10×, the same absolute ε_L") {
    for (const double s : {0.1, 10.0}) {
        const Vec3d eA{1.209195 * s, 1.312911 * s, 1.215093 * s};
        const Vec3d eB{0.855096 * s, 1.420279 * s, 0.714009 * s};
        const Vec3d cB{-0.023041630 * s, 0.033909082 * s,
                       eA.z - eB.z};
        const auto a2d = rect2d(eA.x, eA.y, 0, 0);
        const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cB);
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        INFO("scale " << s << ": "
                      << (U == nullptr ? "REFUSED" : "admitted"));
        REQUIRE(U != nullptr);
        verify_union_member(*U, eA.z, a2d, b2d,
                            aabb_union_volume(eA, {0, 0, 0}, eB, cB),
                            1e-9 * std::max(1.0, s * s * s),
                            "the scale");
    }
}

TEST_CASE("C2a the thin tongue — the near-coincident crossings, the "
          "resolution's own derived margin") {
    // B thin in x, tall in y: the two crossings on A's top edge sit
    // 2w apart — down to 2·10⁻⁵, twenty ε_L: the resolution's own
    // margin, derived (anything under a couple of ε_L would enter
    // the parametric windows' domain — a different seat).
    const double hz = 0.9;
    const Vec3d eA{1.2, 1.05, hz};
    for (const double w : {1e-3, 1e-4, 1e-5}) {
        const Vec3d eB{w, 1.2, hz};
        const Vec3d cB{0.1, 0.0, 0.0};
        const auto a2d = rect2d(eA.x, eA.y, 0, 0);
        const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cB);
        auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        INFO("tongue w=" << w << ": "
                         << (U == nullptr ? "REFUSED" : "admitted"));
        REQUIRE(U != nullptr);
        const double va = 8.0 * eA.x * eA.y * hz;
        const double vb = 8.0 * eB.x * eB.y * hz;
        const double v_oracle =
            va + vb - my_intersection_area(a2d, b2d) * 2.0 * hz;
        verify_union_member(*U, hz, a2d, b2d, v_oracle,
                            1e-9 + kEpsL * 60.0, "the tongue");
    }
}

TEST_CASE("C2a the coincident corner — the vertex-on-edge degenerate, "
          "the disposition observed and pinned") {
    // B rotated 30°, its (+,−) corner EXACTLY on A's top edge (the
    // corner between one edge going above and one below — both
    // crossing A's edge AT the same point), plus one interior
    // crossing: the coincident-crossing diet. The corner at
    // (0.2 + h·cosθ + k·sinθ, 1.05) with the center placed so the
    // corner's y lands exactly.
    const double hz = 0.9;
    const double ang = 30.0 * kPi / 180.0;
    const double hb = 0.5, kb = 0.35;
    const double dy_corner = hb * std::sin(ang) - kb * std::cos(ang);
    const Vec3d cB{0.2, 1.05 - dy_corner, 0.0};
    const auto a2d = rect2d(1.2, 1.05, 0, 0);
    const auto b2d = rot_rect2d(hb, kb, cB.x, cB.y, ang);
    auto A = make_box_at({1.2, 1.05, hz}, {0, 0, 0});
    auto B = rot_box({hb, kb, hz}, cB, ang);
    REQUIRE(b::is_closed_orientable(*B));
    // The corner precondition: the (+,−) corner exactly on y = 1.05.
    {
        const auto& corner = b2d[2];  // (1,1)-sign corner pre-rotation
        (void)corner;
        const double cy = cB.y + dy_corner;
        CHECK(std::abs(cy - 1.05) < 1e-12);
    }
    // THE OBSERVED DISPOSITION (pinned at the correction's green
    // run): the coincident pair at the corner does not close the
    // walk (the endpoint-crossing degenerate — both of B's adjacent
    // edges crossing A's edge AT the same point — is a different
    // class from the plus's interior crossings), the AREA INVARIANT
    // catches it, and the pair refuses with the honest
    // overlay_region_irregular — typed, named, the sources whole.
    // The vertex-on-edge diet stays a registered seat of its own.
    const std::string msg = expect_output_refusal([&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }, "overlay_region_irregular", "the coincident corner");
    (void)msg;
    CHECK(refused_leaves_source(*A, [&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }));
    CHECK(refused_leaves_source(*B, [&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }));
}

TEST_CASE("C2a the D1 reassessment — the area invariant's derived "
          "band, audited and pinned") {
    // THE AUDIT (2026-11-18): the second cohort's D1 widening charged
    // ONE half-tube (ε_L·8·scale) over ONE rim at the ENTRY scale —
    // underivable on three counts (the target's overlap error and the
    // candidate's own error are SEPARATE full-tube charges over rims
    // ⊆ ∂R ∪ ∂P; the entry scale goes stale when a partner extends
    // past the region's coordinates; the vertex/FP count rode
    // uncovered). THE REASSESSED BAND, every term derived:
    //   area_rel_tol·(A_R+A_P+A_C)  the three shoelace sums' floor
    // + 4·ε_L·(P_R+P_P)             the two ε_L-tubes (the overlap's
    //                                and the candidate's), Steiner
    // + 2π·ε_L²                     the tubes' caps
    // + n²·ε_mach·s_pair²           the FP summation charge
    // THE PINS: the arithmetic (the seam's pure form against this
    // file's own recomputation), the admit side (the exact red and
    // the D1 red's own sliver — the member that forced the widening),
    // the reject side (the 5.48-class miss and the coincident
    // corner's hull dent, both orders of magnitude above), and the
    // under-charge window (the old fixed band against the honest
    // budget on this very member).
    const double kRelArea = 1e-9;   // the caller's own declaration
    const double eps_mach =
        std::numeric_limits<double>::epsilon();

    // (a) THE ARITHMETIC — the seam's value equals this file's own
    // instantiation of the derivation, bit for bit (the same
    // expression, the same doubles: any drift in the production
    // formula breaks this pin visibly).
    {
        const Vec3d eA{1.209195, 1.312911, 1.215093};
        const Vec3d eB{0.855096, 1.420279, 0.714009};
        const Vec3d cB{-0.023041630, 0.033909082, eA.z - eB.z};
        const auto a2d = rect2d(eA.x, eA.y, 0, 0);
        const auto b2d = rect2d(eB.x, eB.y, cB.x, cB.y);
        const auto inter = my_convex_clip(a2d, b2d);
        const double aA = std::abs(my_shoelace(a2d));
        const double aB = std::abs(my_shoelace(b2d));
        const double aU = my_union_area(a2d, b2d);
        const double pR = my_perimeter(a2d), pP = my_perimeter(b2d);
        double s = 1.0;
        for (const auto& q : a2d)
            s = std::max({s, std::abs(q.x), std::abs(q.y)});
        for (const auto& q : b2d)
            s = std::max({s, std::abs(q.x), std::abs(q.y)});
        const double n_tot = double(a2d.size() + b2d.size()
                                    + inter.size() + 12) + 4.0;
        const double mine = kRelArea * (aA + aB + aU)
                          + 4.0 * kEpsL * (pR + pP)
                          + 2.0 * kPi * kEpsL * kEpsL
                          + n_tot * n_tot * eps_mach * s * s;
        const double seam =
            b::overlay_area_check_tol_for_tests(
                aA, aB, aU, pR, pP, a2d.size(), b2d.size(),
                inter.size(), 12, s, kRelArea,
                b::kBooleanHouseTolerance);
        INFO("band arithmetic: mine " << mine << " seam " << seam);
        CHECK(seam == mine);
        // (d) THE UNDER-CHARGE WINDOW: the old fixed band on the same
        // member (the entry scale = the region's own coordinates) —
        // the honest budget exceeds it FOUR-fold at least (observed
        // 7.3×): the audit's own finding, pinned.
        const double s_entry = std::max(1.0, eA.y);
        const double old_band = kRelArea * s_entry * s_entry * 8.0
                              + kEpsL * s_entry * 8.0;
        INFO("window: new " << seam << " old " << old_band
                            << " ratio " << seam / old_band);
        CHECK(seam > 4.0 * old_band);
        // (c-reject) THE 5.48-CLASS MISS: the wrong pentagon the
        // cohort observed against the honest 6.717501 — four orders
        // of magnitude above the band (observed 16151×): the
        // invariant still refuses it by construction.
        const double miss = aU - 5.48;
        INFO("the 5.48-class miss " << miss << " = "
             << miss / seam << "x the band");
        CHECK(miss > 1.0e4 * seam);
    }

    // (b-admit) THE D1 RED'S OWN SEAT: the member that forced the
    // widening (the ε_C band-edge, δ = +0.5·ε_L) — its SIDE-face pair
    // is where the invariant once read 4.000001: the partner extends
    // past the region's rim by δ, the honest discrepancy is the
    // sub-band sliver 1.5·δ, and the derived band must carry it with
    // a wide margin (the D1 disposition — the welded 8/12/6 body —
    // is pinned in the cohort-2 suite; this pin is the BUDGET's).
    {
        const double d = 0.5 * kEpsL;
        // A's side face in its own (x,z) plane: 2×2 at the origin;
        // B's side face: x ∈ [−0.5,1] (1.5 wide), z ∈ [δ, 1+δ]
        // (1 tall — B's own half-extent 0.5, the cohort's build).
        const auto face_A = rect2d(1.0, 1.0, 0.0, 0.0);
        const auto face_B = rect2d(0.75, 0.5, 0.25, d + 0.5);
        const double aA = std::abs(my_shoelace(face_A));
        const double aB = std::abs(my_shoelace(face_B));
        const double aI = my_intersection_area(face_A, face_B);
        // the walk's candidate: the sliver ε-collapsed — A's face.
        const double target = aA + aB - aI;
        const double discrep = std::abs(target - aA);
        const double band = b::overlay_area_check_tol_for_tests(
            aA, aB, aA, my_perimeter(face_A), my_perimeter(face_B),
            face_A.size(), face_B.size(), 8, face_A.size(),
            /*pair_scale=*/1.0, kRelArea, b::kBooleanHouseTolerance);
        INFO("D1 seat: sliver " << discrep << " band " << band
                                << " margin " << band / discrep);
        CHECK(std::abs(discrep - 1.5 * d) < 1e-12);  // the read's own
        CHECK(band > 50.0 * discrep);               // 80× observed
    }

    // (c-reject) THE COINCIDENT CORNER'S HULL DENT: the pinned
    // refusal's own margin — the hull candidate swallows a notch
    // orders of magnitude above the band (the walk cannot close the
    // endpoint degenerate, and the hull's dent is real geometry).
    {
        const double ang = 30.0 * kPi / 180.0;
        const double hb = 0.5, kb = 0.35;
        const double dy = hb * std::sin(ang) - kb * std::cos(ang);
        const auto a2d = rect2d(1.2, 1.05, 0, 0);
        const auto b2d = rot_rect2d(hb, kb, 0.2, 1.05 - dy, ang);
        const double aU = my_union_area(a2d, b2d);
        // the hull of the eight points — this file's own monotone
        // chain, zero production code.
        std::vector<Vec2d> all = a2d;
        all.insert(all.end(), b2d.begin(), b2d.end());
        std::sort(all.begin(), all.end(),
                  [](const Vec2d& u, const Vec2d& v) {
                      return u.x < v.x
                          || (u.x == v.x && u.y < v.y);
                  });
        const auto cross3 = [](const Vec2d& o, const Vec2d& p,
                               const Vec2d& q) {
            return (p.x - o.x) * (q.y - o.y)
                 - (p.y - o.y) * (q.x - o.x);
        };
        std::vector<Vec2d> hull;
        for (const auto& p : all) {
            while (hull.size() >= 2 && cross3(hull[hull.size() - 2],
                                              hull.back(), p) <= 0)
                hull.pop_back();
            hull.push_back(p);
        }
        const std::size_t lower = hull.size() + 1;
        for (auto it = all.rbegin(); it != all.rend(); ++it) {
            while (hull.size() >= lower
                   && cross3(hull[hull.size() - 2], hull.back(),
                             *it) <= 0)
                hull.pop_back();
            hull.push_back(*it);
        }
        hull.pop_back();
        const double aH = std::abs(my_shoelace(hull));
        const double band = b::overlay_area_check_tol_for_tests(
            std::abs(my_shoelace(a2d)), std::abs(my_shoelace(b2d)),
            aH, my_perimeter(a2d), my_perimeter(b2d),
            a2d.size(), b2d.size(), 8, hull.size(),
            /*pair_scale=*/1.25, kRelArea, b::kBooleanHouseTolerance);
        const double dent = std::abs(aH - aU);
        INFO("corner dent " << dent << " band " << band
                             << " margin " << dent / band);
        CHECK(dent > 1.0e3 * band);   // 6372× observed
    }

    // (e) THE ORDER NOTE — the operand-order permutation: both
    // orders answer the SAME census (28/46/20, χ = 2) with DIFFERENT
    // vertex enumerations (observed: the digests differ — an
    // enumeration fact, not a geometry fact, and NOT this contract's
    // pin). The order independence that IS the contract lives in the
    // walk's GEOMETRIC crossing order (the cure: the crossings sort
    // along the edge, any computation order lands the same chain),
    // exercised by the rotations and the tongue, and by each order's
    // own bit-identical rebuild (the digest test below).
    {
        const Vec3d eA{1.209195, 1.312911, 1.215093};
        const Vec3d eB{0.855096, 1.420279, 0.714009};
        const Vec3d cB{-0.023041630, 0.033909082, eA.z - eB.z};
        auto A = make_box_at(eA, {0, 0, 0});
        auto B = make_box_at(eB, cB);
        auto U1 = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        auto U2 = b::bool_union(*B, *A, b::kBooleanHouseTolerance);
        REQUIRE(U1 != nullptr);
        REQUIRE(U2 != nullptr);
        CHECK(U1->vertex_count() == U2->vertex_count());
        CHECK(U1->edge_count() == U2->edge_count());
        CHECK(U1->face_count() == U2->face_count());
        CHECK(b::euler_characteristic(*U1) == 2);
        CHECK(b::euler_characteristic(*U2) == 2);
        CHECK(std::abs(poly_volume(*U1) - poly_volume(*U2)) < 1e-9);
        // each order's rebuild: bit-identical (the digest law).
        auto U1b = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        auto U2b = b::bool_union(*B, *A, b::kBooleanHouseTolerance);
        CHECK(b::chain_digest(*U1) == b::chain_digest(*U1b));
        CHECK(b::chain_digest(*U2) == b::chain_digest(*U2b));
    }
}

TEST_CASE("C2a the repeated execution — the corpus digest pinned, "
          "bit-identical forever") {
    // The core members re-run twice: the census/area/volume digest
    // accumulated per member, the two passes bit-identical, the FNV
    // pinned.
    const auto run = [&](std::string& acc) {
        const Vec3d eA0{1.209195, 1.312911, 1.215093};
        const Vec3d eB0{0.855096, 1.420279, 0.714009};
        // 1. the exact red.
        {
            const Vec3d cB{-0.023041630, 0.033909082,
                           eA0.z - eB0.z};
            auto A = make_box_at(eA0, {0, 0, 0});
            auto B = make_box_at(eB0, cB);
            auto U = b::bool_union(*A, *B,
                                   b::kBooleanHouseTolerance);
            REQUIRE(U != nullptr);
            char line[160];
            std::snprintf(line, sizeof line,
                          "red %zu %zu %zu %.17g\n",
                          U->vertex_count(), U->edge_count(),
                          U->face_count(), poly_volume(*U));
            acc += line;
        }
        // 2. the rotations.
        for (const double deg : {30.0, 45.0}) {
            const double ang = deg * kPi / 180.0;
            const double hz = 0.8;
            const Vec3d eA{1.209195, 1.312911, hz};
            const Vec3d eB{0.855096, 1.420279, hz};
            const double ca = std::cos(ang), sa = std::sin(ang);
            const Vec3d cB{-0.023041630 * ca - 0.033909082 * sa,
                           -0.023041630 * sa + 0.033909082 * ca,
                           0.0};
            auto A = rot_box(eA, {0, 0, 0}, ang);
            auto B = rot_box(eB, cB, ang);
            auto U = b::bool_union(*A, *B,
                                   b::kBooleanHouseTolerance);
            REQUIRE(U != nullptr);
            char line[160];
            std::snprintf(line, sizeof line,
                          "rot%g %zu %zu %zu %.17g\n", deg,
                          U->vertex_count(), U->edge_count(),
                          U->face_count(), poly_volume(*U));
            acc += line;
        }
        // 3. the tongue and the scale.
        {
            const Vec3d eA{1.2, 1.05, 0.9};
            const Vec3d eB{1e-4, 1.2, 0.9};
            auto A = make_box_at(eA, {0, 0, 0});
            auto B = make_box_at(eB, {0.1, 0, 0});
            auto U = b::bool_union(*A, *B,
                                   b::kBooleanHouseTolerance);
            REQUIRE(U != nullptr);
            char line[160];
            std::snprintf(line, sizeof line,
                          "tng %zu %zu %zu %.17g\n",
                          U->vertex_count(), U->edge_count(),
                          U->face_count(), poly_volume(*U));
            acc += line;
        }
        {
            const Vec3d eA{1.209195 * 10, 1.312911 * 10,
                           1.215093 * 10};
            const Vec3d eB{0.855096 * 10, 1.420279 * 10,
                           0.714009 * 10};
            const Vec3d cB{-0.023041630 * 10, 0.033909082 * 10,
                           eA.z - eB.z};
            auto A = make_box_at(eA, {0, 0, 0});
            auto B = make_box_at(eB, cB);
            auto U = b::bool_union(*A, *B,
                                   b::kBooleanHouseTolerance);
            REQUIRE(U != nullptr);
            char line[160];
            std::snprintf(line, sizeof line,
                          "s10 %zu %zu %zu %.17g\n",
                          U->vertex_count(), U->edge_count(),
                          U->face_count(), poly_volume(*U));
            acc += line;
        }
    };
    std::string acc1, acc2;
    run(acc1);
    run(acc2);
    CHECK(acc1 == acc2);   // the re-run bit-identical
    std::uint64_t hv = 1469598103934665603ull;  // FNV-1a 64
    for (char c : acc1) {
        hv ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(c));
        hv *= 1099511628211ull;
    }
    std::printf("[k1-c2a] corpus digest %zu bytes hash %llx\n",
                acc1.size(), static_cast<unsigned long long>(hv));
    CHECK(hv == kPinnedCorpusHash);
}

}  // TEST_SUITE
