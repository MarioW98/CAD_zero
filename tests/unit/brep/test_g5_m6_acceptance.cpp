// tests/unit/brep/test_g5_m6_acceptance.cpp
//
// G5 m6 — THE FINAL ACCEPTANCE CORPUS (ADR-0043 D9's gate, the
// campaign's technical closure): the complete multi-edge result
// audited through the PRODUCTION APIs with equations that are THIS
// FILE'S OWN. The law is the brief's ("Do not assume a register entry
// is proof of implementation"; the oracle "shares no production
// construction or mass-property code"):
//
//   * THE CONSTRUCTION is production's (make_box → the four
//     fillet_edge verticals → fillet_ring_act) — the audited object
//     is exactly what the ship ships.
//   * THE ORACLE is mine: the V/E/F derivation from first principles
//     (the eight corners' three tangency points each = 24 vertices;
//     twelve flanks' two tangent lines + eight spheres' three arcs =
//     48 edges; six planes + twelve flanks + eight spheres = 26
//     faces; χ = 24 − 48 + 26 = 2), the Steiner volume
//     a³ + 6a²r + 3πar² + (4/3)πr³ at a = 2 − 2r (my own
//     decomposition, D7.2's road), the sphere centers, the frames,
//     the boundary midpoints.
//   * THE READS are the report's own authorities (BodySummary,
//     mass_properties, fillet_descent) — read, never consulted for
//     an expected value.
//
// THE AUDIT, per body (the brief's Phase 1 list, item by item):
// the twelve intended edges consumed exactly once (the tavola's
// Vanished edge rows, each original edge once, the causes recorded);
// the family census (8 spheres, 4+4+4 X/Y/Z flanks, 6 planes); the
// boundary agreement edge by edge (every edge's two co-edges'
// surfaces pass through the SAME 3D point at the p-curves'
// mid-spans — the sphere-to-cylinder coaxial arcs and the
// cylinder-to-plane tangent lines in one closed form); the p-curves
// coherent (the domain midpoints evaluate on their OWN face's
// surface to the shared points); every surface's orientation (the
// spheres' walks interior_left, the cylinders' normals outward at my
// samples); the loops closed (the query AND my own walk); the radial
// incidence valid (every edge exactly two co-edges, traversed in
// OPPOSITE directions); the shell closed, orientable, manifold
// (χ = 2, H = 0, the co-edge pairing); the provenance exact (the
// all-exact tally, zero linear tolerance — the closed forms' own
// signature); the persistent naming and genealogy (the eight corner
// names' plane triples, the twelve flank names, the tavola's
// resolution).
//
// THE SWEEP: the deep audit at r = 0.5 (the D9 guide case — the
// panel's own number 4 + 11π/12 = 6.879793265790644), the full
// audit at r ∈ {0.25, 0.5, 0.75} × {origin, (3, −2, 5)} — all eight
// corner orientations and all twelve edge placements ride the
// complete cube; the translated frame carries the naming whole.
// The digests bit-identical on the re-driven chains.
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace math = CAD_0::math;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-12;

struct SphereReader : b::SurfaceVisitor {
    const b::SphereSurface* sphere{nullptr};
    void visit(const b::SphereSurface& s) override { sphere = &s; }
};
struct XReader : b::SurfaceVisitor {
    const b::CylinderXSurface* x{nullptr};
    void visit(const b::CylinderXSurface& s) override { x = &s; }
};
struct YReader : b::SurfaceVisitor {
    const b::CylinderYSurface* y{nullptr};
    void visit(const b::CylinderYSurface& s) override { y = &s; }
};
struct ZReader : b::SurfaceVisitor {
    const b::CylinderZSurface* z{nullptr};
    void visit(const b::CylinderZSurface& s) override { z = &s; }
};
struct PlaneReader : b::SurfaceVisitor {
    const b::PlaneSurface* plane{nullptr};
    void visit(const b::PlaneSurface& s) override { plane = &s; }
};

// ---------------------------------------------------------------------------
// THE CHAIN — production's own road (the four verticals, then the
// ring-and-corners act).
// ---------------------------------------------------------------------------
struct ChainResult {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;  // 24/48/26
};

ChainResult build_cube(double r, math::Vec3d delta = {0, 0, 0}) {
    ChainResult out;
    out.box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    REQUIRE(out.box != nullptr);
    if (delta.length() > 0.0) b::translate_body(*out.box, delta);
    b::BRepBody* cur = out.box.get();
    std::unique_ptr<b::BRepBody> last;
    for (int k = 0; k < 4; ++k) {
        const ShapeId v{FeatureId{1},
                        SubshapeRef{}.push(
                            static_cast<std::uint32_t>(8 + k)),
                        {}};
        const b::FilletNaming ctx{
            FeatureId{static_cast<std::uint64_t>(2 + k)},
            &n::kBoxNaming};
        last = b::fillet_edge(*cur, v, r, ctx);
        REQUIRE(last != nullptr);
        cur = last.get();
    }
    const b::FilletNaming actx{FeatureId{6}, &n::kBoxNaming};
    out.result = b::fillet_ring_act(*cur, r, actx);
    REQUIRE(out.result != nullptr);
    return out;
}

// MY Steiner volume (D7.2's road, this file's own arithmetic) —
// the cube of half-extent e (the unit box the default).
double steiner(double r, double e = 1.0) {
    const double a = 2.0 * e - 2.0 * r;
    return a * a * a + 6.0 * a * a * r + 3.0 * kPi * a * r * r
           + (4.0 / 3.0) * kPi * r * r * r;
}

// The p-curve's domain midpoint (my own read of the payload).
math::Vec2d domain_mid(const b::PCurve& pc) {
    if (pc.kind == b::PCurve::Kind::Segment) {
        return {(pc.segment.a.x + pc.segment.b.x) * 0.5,
                (pc.segment.a.y + pc.segment.b.y) * 0.5};
    }
    if (pc.kind == b::PCurve::Kind::Arc) {
        const double mid = pc.arc.start_angle + 0.5 * pc.arc.sweep;
        return {pc.arc.center.x + pc.arc.radius * std::cos(mid),
                pc.arc.center.y + pc.arc.radius * std::sin(mid)};
    }
    return pc.circle.center;
}

// The face census: how many of each family.
struct Census {
    int spheres{0}, cx{0}, cy{0}, cz{0}, planes{0}, other{0};
    std::vector<const b::Face*> sphere_faces;
    std::vector<const b::Face*> cx_faces;
    std::vector<const b::Face*> cy_faces;
    std::vector<const b::Face*> cz_faces;
};

Census census_of(const b::BRepBody& body) {
    Census c;
    for (const auto& f : body.solid.shell().faces()) {
        SphereReader sr;
        XReader xr;
        YReader yr;
        ZReader zr;
        PlaneReader pr;
        if (f.surface() == nullptr) {
            ++c.other;
            continue;
        }
        f.surface()->accept(sr);
        f.surface()->accept(xr);
        f.surface()->accept(yr);
        f.surface()->accept(zr);
        f.surface()->accept(pr);
        if (sr.sphere != nullptr) {
            ++c.spheres;
            c.sphere_faces.push_back(&f);
        } else if (xr.x != nullptr) {
            ++c.cx;
            c.cx_faces.push_back(&f);
        } else if (yr.y != nullptr) {
            ++c.cy;
            c.cy_faces.push_back(&f);
        } else if (zr.z != nullptr) {
            ++c.cz;
            c.cz_faces.push_back(&f);
        } else if (pr.plane != nullptr) {
            ++c.planes;
        } else {
            ++c.other;
        }
    }
    return c;
}

// ---------------------------------------------------------------------------
// THE FULL AUDIT.
// ---------------------------------------------------------------------------
void audit_body(const b::BRepBody& body, const b::BRepBody& box,
                double r, math::Vec3d delta) {
    INFO("r=" << r << " delta=(" << delta.x << "," << delta.y
              << "," << delta.z << ")");

    // 1. THE COUNTS — MY derivation: 8 corners × 3 feet = 24
    //    vertices; 12 flanks × 2 tangent lines + 8 spheres × 3 arcs
    //    = 48 edges; 6 planes + 12 flanks + 8 spheres = 26 faces;
    //    χ = 24 − 48 + 26 = 2.
    REQUIRE(body.vertex_count() == 8 * 3);
    REQUIRE(body.edge_count() == 12 * 2 + 8 * 3);
    REQUIRE(body.face_count() == 6 + 12 + 8);
    REQUIRE(b::inner_loop_count(body) == 0);
    REQUIRE(b::euler_characteristic(body) == 2);
    REQUIRE(b::is_closed_orientable(body));
    REQUIRE(b::loops_are_closed(body));

    // 2. THE FAMILY CENSUS.
    const Census c = census_of(body);
    CHECK(c.spheres == 8);
    CHECK(c.cx == 4);
    CHECK(c.cy == 4);
    CHECK(c.cz == 4);
    CHECK(c.planes == 6);
    CHECK(c.other == 0);

    // 3. THE SPHERES — the eight centers at C = corner − r·Σsigns,
    //    one per sign triple (the octant census).
    std::set<std::string> seen_corners;
    for (const b::Face* f : c.sphere_faces) {
        SphereReader sr;
        f->surface()->accept(sr);
        const b::SphereSurface& s = *sr.sphere;
        CHECK(s.radius() == doctest::Approx(r).epsilon(kTol));
        const double e = 1.0 - r;
        const double sx = s.origin().x > delta.x ? 1.0 : -1.0;
        const double sy = s.origin().y > delta.y ? 1.0 : -1.0;
        const double sz = s.origin().z > delta.z ? 1.0 : -1.0;
        CHECK(std::abs(s.origin().x - delta.x - sx * e) < 1e-9);
        CHECK(std::abs(s.origin().y - delta.y - sy * e) < 1e-9);
        CHECK(std::abs(s.origin().z - delta.z - sz * e) < 1e-9);
        // THE NAMING: the corner's three planes in the name (the
        // set, not the order — the naming's own order is the act's).
        const std::string px = sx > 0 ? "right" : "left";
        const std::string py = sy > 0 ? "back" : "front";
        const std::string pz = sz > 0 ? "top" : "bottom";
        CHECK(f->name().find("corner[") != std::string::npos);
        CHECK(f->name().find(px) != std::string::npos);
        CHECK(f->name().find(py) != std::string::npos);
        CHECK(f->name().find(pz) != std::string::npos);
        seen_corners.insert(std::to_string(static_cast<int>(sx))
                            + std::to_string(static_cast<int>(sy))
                            + std::to_string(static_cast<int>(sz)));
    }
    CHECK(seen_corners.size() == 8);

    // 4. THE FLANKS — the axes through the centers' planes, the
    //    radii equal (four per placement), the names the flanks'.
    for (const b::Face* f : c.cz_faces) {
        ZReader zr;
        f->surface()->accept(zr);
        CHECK(zr.z->radius() == doctest::Approx(r).epsilon(kTol));
        CHECK(std::abs(std::abs(zr.z->axis_center().x - delta.x)
                       - (1.0 - r)) < 1e-9);
        CHECK(std::abs(std::abs(zr.z->axis_center().y - delta.y)
                       - (1.0 - r)) < 1e-9);
        CHECK(f->name().find("flank[") != std::string::npos);
    }
    for (const b::Face* f : c.cx_faces) {
        XReader xr;
        f->surface()->accept(xr);
        CHECK(xr.x->radius() == doctest::Approx(r).epsilon(kTol));
        CHECK(std::abs(std::abs(xr.x->axis_y() - delta.y)
                       - (1.0 - r)) < 1e-9);
        CHECK(std::abs(std::abs(xr.x->axis_z() - delta.z)
                       - (1.0 - r)) < 1e-9);
        CHECK(f->name().find("flank[") != std::string::npos);
    }
    for (const b::Face* f : c.cy_faces) {
        YReader yr;
        f->surface()->accept(yr);
        CHECK(yr.y->radius() == doctest::Approx(r).epsilon(kTol));
        CHECK(std::abs(std::abs(yr.y->axis_x() - delta.x)
                       - (1.0 - r)) < 1e-9);
        CHECK(std::abs(std::abs(yr.y->axis_z() - delta.z)
                       - (1.0 - r)) < 1e-9);
        CHECK(f->name().find("flank[") != std::string::npos);
    }

    // 5. THE RADIAL INCIDENCE — every edge exactly two co-edges,
    //    traversed in opposite directions (the orientable pairing).
    for (std::size_t ei = 0; ei < body.edge_count(); ++ei) {
        const auto& e = body.edges[ei];
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed()
              != e.coedges()[1]->reversed());
    }

    // 6. THE BOUNDARY AGREEMENT — every edge's two co-edges: the
    //    domain midpoints evaluate on their OWN surfaces to the SAME
    //    3D point (the coaxial arcs and the tangent lines exact).
    std::vector<std::vector<const b::CoEdge*>> face_coedges(
        body.solid.shell().faces().size());
    for (std::size_t fi = 0;
         fi < body.solid.shell().faces().size(); ++fi) {
        const auto& f = body.solid.shell().faces()[fi];
        for (const auto& lp : f.loops())
            for (const auto& ce : lp.coedges())
                face_coedges[fi].push_back(&ce);
    }
    for (std::size_t fi = 0;
         fi < body.solid.shell().faces().size(); ++fi) {
        const auto& f = body.solid.shell().faces()[fi];
        for (const b::CoEdge* ce : face_coedges[fi]) {
            REQUIRE(ce->p_curve() != nullptr);
            const math::Vec2d mid = domain_mid(*ce->p_curve());
            const math::Vec3d p =
                f.surface()->evaluate(mid.x, mid.y);
            const auto& e = body.edges[ce->edge()];
            for (const b::CoEdge* other : e.coedges()) {
                if (other == ce) continue;
                const b::Face* f2 = nullptr;
                for (std::size_t fj = 0;
                     fj < body.solid.shell().faces().size();
                     ++fj)
                    for (const b::CoEdge* c2 : face_coedges[fj])
                        if (c2 == other) f2 =
                            &body.solid.shell().faces()[fj];
                REQUIRE(f2 != nullptr);
                REQUIRE(other->p_curve() != nullptr);
                const math::Vec2d m2 =
                    domain_mid(*other->p_curve());
                const math::Vec3d q =
                    f2->surface()->evaluate(m2.x, m2.y);
                // The same 3D point, and ON both surfaces.
                CHECK((p - q).length() < 1e-9);
            }
        }
    }

    // 7. THE LOOPS — my own walk (the co-edges' vertex chain closes
    //    on every face), on top of the query's verdict.
    for (const auto& f : body.solid.shell().faces()) {
        for (const auto& lp : f.loops()) {
            REQUIRE(!lp.coedges().empty());
            for (std::size_t i = 0; i < lp.coedges().size(); ++i) {
                const auto& ce = lp.coedges()[i];
                const auto& nxt =
                    lp.coedges()[(i + 1) % lp.coedges().size()];
                const auto& e = body.edges[ce.edge()];
                const auto& e2 = body.edges[nxt.edge()];
                const std::size_t exit_v =
                    static_cast<std::size_t>(
                        ce.reversed() ? e.vertex(0)
                                      : e.vertex(1));
                const std::size_t entry_v =
                    static_cast<std::size_t>(
                        nxt.reversed() ? e2.vertex(1)
                                       : e2.vertex(0));
                CHECK(exit_v == entry_v);
            }
        }
    }

    // 8. THE ORIENTATION — the spheres' walks carry the material on
    //    the left (the p-curves' own flag, all three arcs Line2) and
    //    the cylinders' normals point outward (my radial check at
    //    the domain midpoints).
    for (const b::Face* f : c.sphere_faces) {
        for (const auto& lp : f->loops()) {
            REQUIRE(lp.coedges().size() == 3);
            for (const auto& ce : lp.coedges()) {
                REQUIRE(ce.p_curve()->kind
                        == b::PCurve::Kind::Segment);
                CHECK(ce.p_curve()->segment.interior_left);
            }
        }
    }
    for (std::size_t fi = 0;
         fi < body.solid.shell().faces().size(); ++fi) {
        const auto& f = body.solid.shell().faces()[fi];
        XReader xr;
        YReader yr;
        ZReader zr;
        f.surface()->accept(xr);
        f.surface()->accept(yr);
        f.surface()->accept(zr);
        if (xr.x == nullptr && yr.y == nullptr && zr.z == nullptr)
            continue;
        for (const b::CoEdge* ce : face_coedges[fi]) {
            const math::Vec2d mid = domain_mid(*ce->p_curve());
            const math::Vec3d p =
                f.surface()->evaluate(mid.x, mid.y);
            const math::Vec3d n =
                f.surface()->normal(mid.x, mid.y);
            math::Vec3d axis_point, axis_dir;
            if (xr.x != nullptr) {
                axis_point = {p.x, xr.x->axis_y(), xr.x->axis_z()};
                axis_dir = {1, 0, 0};
            } else if (yr.y != nullptr) {
                axis_point = {yr.y->axis_x(), p.y, yr.y->axis_z()};
                axis_dir = {0, 1, 0};
            } else {
                axis_point = {zr.z->axis_center().x,
                              zr.z->axis_center().y, p.z};
                axis_dir = {0, 0, 1};
            }
            const math::Vec3d foot =
                axis_point
                + axis_dir * ((p - axis_point).dot(axis_dir));
            const math::Vec3d radial = (p - foot) * (1.0 / r);
            CHECK(n.dot(radial) > 0.999);
        }
    }

    // 9. THE PROVENANCE — all exact, zero linear tolerance (the
    //    closed forms' own signature), via the report's own tally.
    const b::BodySummary sum = b::query_body(body);
    CHECK(sum.vertex_count == 24);
    CHECK(sum.edge_count == 48);
    CHECK(sum.face_count == 26);
    CHECK(sum.chi == 2);
    CHECK(sum.inner_loops == 0);
    CHECK(sum.orientable);
    CHECK(sum.loops_closed);
    CHECK(sum.genus_derived);
    CHECK(sum.genus == 0);
    CHECK(sum.vertices.exact == 24);
    CHECK(sum.vertices.approximated == 0);
    CHECK(sum.vertices.max_linear_tolerance == 0.0);
    CHECK(sum.edges.exact == 48);
    CHECK(sum.edges.approximated == 0);
    CHECK(sum.edges.max_linear_tolerance == 0.0);
    CHECK(sum.faces.exact == 26);
    CHECK(sum.faces.approximated == 0);
    CHECK(sum.faces.max_linear_tolerance == 0.0);

    // 10. THE VOLUME — MY Steiner form at 1e-12 (mass_properties
    //     the READ, the formula the ORACLE).
    const b::MassProperties mp =
        b::mass_properties(body, math::Tolerance{});
    CHECK(mp.volume == doctest::Approx(steiner(r)).epsilon(kTol));

    // 11. THE TWELVE EDGES CONSUMED EXACTLY ONCE — the whole-chain
    //     tavola (result against the original box): twelve Vanished
    //     edge rows, each original edge one time, causes recorded.
    {
        const auto tav = b::fillet_descent(body, box);
        std::map<std::string, int> vanished_edges;
        for (const auto& row : tav.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            if (row.entity != b::EntityKind::Edge) continue;
            ++vanished_edges[row.name];
            CHECK(!row.cause.empty());
        }
        REQUIRE(vanished_edges.size() == 12);
        for (int i = 0; i < 4; ++i) {
            CHECK(vanished_edges["box#1:vertical"
                                 + std::to_string(i)]
                  == 1);
        }
        for (int i = 0; i < 4; ++i) {
            CHECK(vanished_edges["box#1:top"
                                 + std::to_string(i)]
                  == 1);
            CHECK(vanished_edges["box#1:bottom"
                                 + std::to_string(i)]
                  == 1);
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// THE DEEP AUDIT — the D9 guide case (r = 0.5, the panel's number),
// the digests, the D9 figures verbatim.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m6 acceptance: the complete cube at r = 0.5 — the "
          "deep audit, the D9 figures") {
    const double r = 0.5;
    auto chain = build_cube(r);
    audit_body(*chain.result, *chain.box, r, {0, 0, 0});

    // THE D9 NUMBERS verbatim: 4 + 11π/12 = 6.879793265790644.
    CHECK(steiner(r)
          == doctest::Approx(4.0 + 11.0 * kPi / 12.0).epsilon(kTol));
    CHECK(steiner(r)
          == doctest::Approx(6.879793265790644).epsilon(1e-15));

    // THE DETERMINISM — the chain re-driven, the digest
    // bit-identical.
    auto chain2 = build_cube(r);
    CHECK(b::chain_digest(*chain.result)
          == b::chain_digest(*chain2.result));
}

// ---------------------------------------------------------------------------
// THE SWEEP — three radii × two placements (the origin and the
// translated frame), every one the full audit.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m6 acceptance: the sweep — r ∈ {0.25, 0.5, 0.75} × "
          "{origin, (3, −2, 5)}") {
    for (const double r : {0.25, 0.5, 0.75}) {
        {
            auto chain = build_cube(r);
            audit_body(*chain.result, *chain.box, r, {0, 0, 0});
        }
        {
            auto chain = build_cube(r, {3.0, -2.0, 5.0});
            audit_body(*chain.result, *chain.box, r,
                       {3.0, -2.0, 5.0});
        }
    }
}

// ---------------------------------------------------------------------------
// THE RESIZED CUBE (the payload-derived frame's own dividend: the
// act reads the box's center and half-extent from the six planes,
// never the world's — a cube of side 4 the same closed forms) and
// the NON-CUBE refused by name (the family's law, D1).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m6 acceptance: the resized cube and the non-cube's "
          "refusal — the payload's frame, the family's law") {
    // THE RESIZED CUBE: half-extent 2, r = 0.6 — the counts, the
    // census, the boundary agreement, MY Steiner at e = 2.
    {
        auto box = b::make_box({2.0, 2.0, 2.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        b::BRepBody* cur = box.get();
        std::vector<std::unique_ptr<b::BRepBody>> keep;
        for (int k = 0; k < 4; ++k) {
            const ShapeId v{FeatureId{1},
                            SubshapeRef{}.push(static_cast<
                                std::uint32_t>(8 + k)),
                            {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            keep.push_back(b::fillet_edge(*cur, v, 0.6, ctx));
            REQUIRE(keep.back() != nullptr);
            cur = keep.back().get();
        }
        const b::FilletNaming actx{FeatureId{6}, &n::kBoxNaming};
        auto result = b::fillet_ring_act(*cur, 0.6, actx);
        REQUIRE(result != nullptr);
        REQUIRE(result->vertex_count() == 24);
        REQUIRE(result->edge_count() == 48);
        REQUIRE(result->face_count() == 26);
        REQUIRE(b::euler_characteristic(*result) == 2);
        REQUIRE(b::is_closed_orientable(*result));
        const Census c = census_of(*result);
        CHECK(c.spheres == 8);
        CHECK(c.cx == 4);
        CHECK(c.cy == 4);
        CHECK(c.cz == 4);
        CHECK(c.planes == 6);
        // The boundary agreement (the shared-edge midpoint check,
        // the audit's own closed form).
        for (std::size_t ei = 0; ei < result->edge_count(); ++ei) {
            const auto& e = result->edges[ei];
            REQUIRE(e.coedges().size() == 2);
            math::Vec3d pts[2];
            int n = 0;
            for (const b::CoEdge* ce : e.coedges()) {
                for (std::size_t fi = 0;
                     fi < result->solid.shell().faces().size();
                     ++fi) {
                    const auto& f =
                        result->solid.shell().faces()[fi];
                    bool found = false;
                    for (const auto& lp : f.loops())
                        for (const auto& c2 : lp.coedges())
                            if (&c2 == ce) found = true;
                    if (!found) continue;
                    const auto mid =
                        domain_mid(*ce->p_curve());
                    pts[n++] =
                        f.surface()->evaluate(mid.x, mid.y);
                }
            }
            CHECK((pts[0] - pts[1]).length() < 1e-9);
        }
        const b::MassProperties mp = b::mass_properties(
            *result, math::Tolerance{});
        CHECK(mp.volume
              == doctest::Approx(steiner(0.6, 2.0)).epsilon(kTol));
    }
    // THE NON-CUBE refused by name (the new admission's own words).
    {
        auto box = b::make_box({1.5, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        b::BRepBody* cur = box.get();
        std::vector<std::unique_ptr<b::BRepBody>> keep;
        bool refused_early = false;
        for (int k = 0; k < 4 && !refused_early; ++k) {
            const ShapeId v{FeatureId{1},
                            SubshapeRef{}.push(static_cast<
                                std::uint32_t>(8 + k)),
                            {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            try {
                keep.push_back(
                    b::fillet_edge(*cur, v, 0.3, ctx));
                cur = keep.back().get();
            } catch (const std::exception&) {
                refused_early = true;
            }
        }
        if (!refused_early) {
            const b::FilletNaming actx{FeatureId{6},
                                       &n::kBoxNaming};
            bool refused = false;
            try {
                auto out = b::fillet_ring_act(*cur, 0.3, actx);
                (void)out;
            } catch (const std::exception& e) {
                refused = true;
                CHECK(std::string(e.what()).find("not a cube")
                      != std::string::npos);
            }
            CHECK(refused);
        }
    }
}
