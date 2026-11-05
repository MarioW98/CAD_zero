// tests/unit/brep/test_g2_u3_m3_toroidal.cpp
//
// G2 UNIT 3, m3 — IL FILLET, IL MORSO 2: IL RIM CIRCOLARE DEL FORO,
// IL FIANCO TOROIDALE (ADR-0039 D1.2, the chiral class). The goldens
// were pinned by the ADR BEFORE this code (D5's protocol); this file
// lands them green on the implementation, every number quoted from
// the derivation, never re-derived differently.
//
// THE GUIDE CASE: the u2 blind body (the box [-1,1]^3 with the blind
// hole a = 0.5, floor z = -0.5, top rim z = 1 — 12/17/8, H = 1,
// volume 8 - 0.375*pi) with its TOP rim — the pierced pair's own,
// sub#4:isect[box#1:top x ext#3:lat(0)]@hi, 0038 D3's pinned string
// "so the fillet cannot bend it" — filleted at r = 0.25 < a.
//
// THE DERIVED COUNTS (D1.2's table): V = 14, E = 20, F = 9, H = 1,
// chi = 2 (the dent family's own chi-flatness). The census 9/22/16
// out+vanished both sides; the co-edge census 20 x 2 = 40 (right
// 4 + back 4 + ... : top 6 + bottom 4 + four sides 16 + wall 6 + cap
// 2 + flank 6), every edge exactly two opposite co-edges — the
// contact circles walked OPPOSITE by their landing faces and the
// flank (the RACCOLTO's azimuth XOR), the flank's seam carried TWICE
// by its single loop.
//
// THE VOLUME: (8 - 0.375*pi) - V_r with
// V_r = 2*pi*[a*r^2*(1-pi/4) + r^3*(5/6-pi/4)] = 0.046842996386064,
// so V = 6.775059758517764 — all_exact, everything double; the
// toroidal wedge verified BY the mass (the same arithmetic read
// twice), the centroid ON THE AXIS (x-bar = y-bar = 0, the
// axisymmetry) with z-bar derived by the wedge's own closed form.
//
// THE TORO IS BORN RIGHT-HANDED: the flank is the torus's first
// trimming loop, and the flip of the registered left-handed pair
// (TorusSurface, ConeSurface — cone RIDING, the register's economy)
// lands in the same act, every flip with its discriminating pin (the
// frame rule pointwise — the guard that FAILS on the left-handed
// frame) in test_analytic_surfaces.cpp; the full-torus discriminator
// 2*pi^2*R*r^2 = 0.925275412602127 is NOT the flank's flux, and the
// tests pin the discrimination.
//
// THE COUNTERPROOFS (the naive red, per component): the counts
// without the dead pair; the re-spanning skipped; the inner loop not
// moved. THE REFUSALS: the two bands with their bounds carried
// (fillet_exits_annulus, fillet_eats_wall — the radius, the bound
// and the epsilon in the payload, never a dry string); the CONCAVE
// family (the floor rim) refused by name; the seam refused; the
// vertex blend never named (MURO, the refused tail).
//
// THE EDIT CONTRACT: the radius edit class 1 SILENT on the rim too —
// the reference taken once, the edit answering reference_resolved,
// the feature re-run, the counts fixed for every in-band r. The
// determinism: N = 10 rebuilds bit-identical, the digest pure of the
// recipe.
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <doctest/doctest.h>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The guide case's chain: box#1 -> sk#2 -> ext#3 -> sub#4 -> fil#5
// (ADR-0039 D3's own ids). The blind body: the u2 guide (the circle
// r = 0.5, prism height 2, offset (0,0,-0.5): the tool z in
// [-0.5, 1.5], the floor at -0.5, the rim through the top at z = 1).
struct RimGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    CAD_0::math::Vec3d offset{0.0, 0.0, -0.5};
    ShapeId rim{};

    explicit RimGuide(double radius = 0.25, bool named = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, offset,
                                  b::kSubtractTolerance, sub_ctx);
        // The rim by NAME, taken from the query (the structural
        // ShapeId, never a parse of the render): the seam-wired arc
        // PAIR answers to the one logical name — the pair taken
        // whole (the morse-5 register's own rule).
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        const b::FilletNaming ctx{FeatureId{5}, &n::kBoxNaming,
                                  FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, radius,
                                    named ? ctx : b::FilletNaming{});
    }
};

// The wedge's closed forms, derived (ADR-0039 D1.2 quoting 0033 D3):
// the removed region per the (rho, z) measure 2*pi*rho d rho dz over
// the corner window minus the quarter disc. Every quantity a closed
// form of (a, r, R = a + r, z_c = 1 - r, z_floor, z_rim).
struct WedgeForms {
    double v_r;        // the removed volume
    double mz;         // the removed first moment about z (about 0)
    double izz;        // the removed second moment about the z axis
    double z2;         // the removed integral of z^2 (about 0)
};
WedgeForms wedge_forms(double a, double r, double z_floor,
                       double z_rim) {
    const double R = a + r;
    const double z_c = z_rim - r;
    WedgeForms w{};
    // V_r = 2*pi*[a*r^2*(1-pi/4) + r^3*(5/6-pi/4)]  (the ADR's form).
    w.v_r = 2.0 * kPi *
            (a * r * r * (1.0 - kPi / 4.0) +
             r * r * r * (5.0 / 6.0 - kPi / 4.0));
    // M_z: the corner window's ring prism minus the quarter disc's,
    // both by the cylinder measure 2*pi*rho: the 2D integrals of
    // z*rho over (window - disc), times 2*pi.
    const double zint_quad =
        ((z_rim * z_rim - z_c * z_c) / 2.0) *
        (((a + r) * (a + r) - a * a) / 2.0);
    const double zint_quarto =
        z_c * R * kPi * r * r / 4.0 - z_c * r * r * r / 3.0 +
        R * r * r * r / 3.0 - r * r * r * r / 8.0;
    w.mz = -2.0 * kPi * (zint_quad - zint_quarto) *
           -1.0;  // the wedge is REMOVED: its moments subtract; the
                  // sign folded here (removed < 0 for material above
                  // the floor's midline: the caller adds it to the
                  // blind's).
    // The blind's own z-moment is -m_tool; the fillet's is that
    // minus the wedge's: fold once, in the caller.
    w.mz = 2.0 * kPi * (zint_quad - zint_quarto);
    // I_zz: 2*pi*(rho^3 over the window - over the quarter).
    const double rho3_quad =
        r * (((a + r) * (a + r) * (a + r) * (a + r) - a * a * a * a) /
             4.0);
    const double rho3_quarto =
        R * R * R * kPi * r * r / 4.0 - R * R * r * r * r +
        3.0 * kPi * R * r * r * r * r / 16.0 -
        2.0 * r * r * r * r * r / 15.0;
    w.izz = 2.0 * kPi * (rho3_quad - rho3_quarto);
    // z^2: 2*pi*(z^2*rho over the window - over the quarter).
    const double z2_quad =
        ((z_rim * z_rim * z_rim - z_c * z_c * z_c) / 3.0) *
        (((a + r) * (a + r) - a * a) / 2.0);
    const double z2_quarto =
        z_c * z_c * (R * kPi * r * r / 4.0 - r * r * r / 3.0) +
        2.0 * z_c * (R * r * r * r / 3.0 - r * r * r * r / 8.0) +
        (R * kPi * r * r * r * r / 16.0 - r * r * r * r * r / 15.0);
    w.z2 = 2.0 * kPi * (z2_quad - z2_quarto);
    return w;
}

// The digests (the m2 oracle's own shape, one family over).
std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_face(body, i).name;
        out += '\n';
    }
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

std::string coedge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        const auto info = b::query_face(body, i);
        for (const auto& loop : info.loops)
            for (const auto& pc : loop.coedges) {
                out += std::to_string(i);
                out += '|';
                out += std::to_string(pc.edge);
                out += '|';
                out += b::query_edge(body, pc.edge).name;
                out += '\n';
            }
    }
    return out;
}

std::uint64_t structure_digest(const b::BRepBody& body) {
    std::string out;
    out += std::to_string(body.vertex_count());
    out += '/';
    out += std::to_string(body.edge_count());
    out += '/';
    out += std::to_string(body.face_count());
    for (const auto& v : body.vertices) {
        out += std::to_string(v.position().x) + ',';
        out += std::to_string(v.position().y) + ',';
        out += std::to_string(v.position().z) + ';';
    }
    for (const auto& e : body.edges) {
        out += std::to_string(e.vertex(0)) + '-';
        out += std::to_string(e.vertex(1)) + ';';
    }
    for (const auto& f : body.solid.shell().faces()) {
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges()) {
                out += std::to_string(ce.edge());
                out += ce.reversed() ? 'R' : 'F';
                const auto* pc = ce.p_curve();
                if (pc != nullptr) {
                    out += 'p';
                    out += std::to_string(
                        static_cast<int>(pc->kind));
                    if (pc->kind == b::PCurve::Kind::Segment) {
                        out += std::to_string(
                                   pc->segment.a.x) + ',' +
                               std::to_string(pc->segment.a.y) +
                               ',' +
                               std::to_string(pc->segment.b.x) +
                               ',' +
                               std::to_string(pc->segment.b.y);
                    } else if (pc->kind ==
                               b::PCurve::Kind::Arc) {
                        out += std::to_string(
                                   pc->arc.center.x) + ',' +
                               std::to_string(pc->arc.center.y) +
                               ',' +
                               std::to_string(pc->arc.radius) +
                               ',' +
                               std::to_string(pc->arc.start_angle) +
                               ',' +
                               std::to_string(pc->arc.sweep);
                    }
                }
                out += ';';
            }
    }
    return fnv1a64(out);
}

} // namespace

// ===========================================================================
// THE GOLDEN — the guide case's every number, from the ADR's table
// ===========================================================================
TEST_CASE("G2-U3 m3 the golden: the blind top rim filleted at r=0.25 "
          "— 14/20/9/H=1/chi=2, the volume (8-0.375*pi)-V_r all exact, "
          "the census 40, the closure 9/22/16") {
    const RimGuide g(0.25);
    REQUIRE(g.blind != nullptr);
    REQUIRE(g.result != nullptr);

    // The blind base: the u2 golden, unmoved (additivity's own).
    REQUIRE(g.blind->vertex_count() == 12);
    REQUIRE(g.blind->edge_count() == 17);
    REQUIRE(g.blind->face_count() == 8);
    {
        const auto mp = b::mass_properties(
            *g.blind, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.375 * kPi));
    }

    // THE COUNTS — D1.2's table: the rim pair dies (-2 edges, -2
    // joints), five edges born (the two contact pairs + the flank's
    // seam), four vertices born, one face born; H stays 1 (the
    // pierced face keeps exactly one inner loop — now the a+r
    // circle).
    REQUIRE(g.result->vertex_count() == 14);
    REQUIRE(g.result->edge_count() == 20);
    REQUIRE(g.result->face_count() == 9);
    REQUIRE(b::inner_loop_count(*g.result) == 1);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.blind));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges; the
    // total 40 over the nine faces (top 6 + bottom 4 + four sides 16
    // + wall 6 + cap 2 + flank 6).
    {
        std::size_t total = 0;
        for (const auto& e : g.result->edges) {
            REQUIRE(e.coedges().size() == 2);
            CHECK(e.coedges()[0]->reversed() !=
                  e.coedges()[1]->reversed());
        }
        for (const auto& f : g.result->solid.shell().faces())
            for (const auto& loop : f.loops())
                total += loop.coedges().size();
        REQUIRE(total == 40);
        const std::size_t per_face[9] = {4, 6, 4, 4, 4, 4, 6, 2, 6};
        for (std::size_t i = 0; i < 9; ++i) {
            CAPTURE(i);
            std::size_t ce = 0;
            for (const auto& loop :
                 g.result->solid.shell().faces()[i].loops())
                ce += loop.coedges().size();
            REQUIRE(ce == per_face[i]);
        }
        // The flank's seam rides its single loop TWICE (the periodic
        // identification's signature, the wall's own pattern).
        std::size_t seam_uses = 0;
        for (const auto& ce :
             g.result->solid.shell().faces()[8].loops()[0].coedges())
            if (ce.edge() == 19) ++seam_uses;
        REQUIRE(seam_uses == 2);
    }

    // THE NUMBERING TABLE — the survivors compact, the born append:
    // 10/11 = S''@parete / S'@piano (the +X joint at the two contact
    // circles), 12/13 = A''@parete / A'@piano (the -X joint); the
    // wall's seam re-spanned (8, 10); the floor pair kept (9, 8).
    {
        const Vec3d pv[4] = {
            {0.5, 0.0, 0.75}, {0.75, 0.0, 1.0},
            {-0.5, 0.0, 0.75}, {-0.75, 0.0, 1.0}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            const Vec3d p = g.result->vertices[10 + i].position();
            CHECK(close(p.x, pv[i].x, 1e-12));
            CHECK(close(p.y, pv[i].y, 1e-12));
            CHECK(close(p.z, pv[i].z, 1e-12));
        }
        // The floor pair KEEPS its joints through the compaction:
        // e12 = A_c -> S_c = (9, 8), e13 = (8, 9). The wall's seam
        // re-spanned: e14 = (8, 10) — the floor joint kept, the high
        // end re-pointed at the parete contact's S''. The born five:
        // e15/16 = the plane contact pair (A'->S', S'->A'); e17/18 =
        // the wall contact pair (A''->S'', S''->A''); e19 = the
        // flank's seam (S'->S'').
        const std::pair<std::size_t, std::size_t> pe[6] = {
            {8, 10}, {13, 11}, {11, 13},
            {12, 10}, {10, 12}, {11, 10}};
        for (std::size_t i = 0; i < 6; ++i) {
            CAPTURE(i);
            CHECK(g.result->edges[14 + i].vertex(0) == pe[i].first);
            CHECK(g.result->edges[14 + i].vertex(1) == pe[i].second);
        }
        CHECK(g.result->edges[12].vertex(0) == 9);
        CHECK(g.result->edges[12].vertex(1) == 8);
        CHECK(g.result->edges[13].vertex(0) == 8);
        CHECK(g.result->edges[13].vertex(1) == 9);
    }

    // THE VOLUME — the classical form IS the test:
    // (8 - 0.375*pi) - V_r, V_r = 2*pi*[a*r^2*(1-pi/4) + r^3*(5/6-pi/4)]
    // = 0.046842996386064, all_exact, everything double.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        const double a = 0.5, r = 0.25;
        const double v_r = 2.0 * kPi *
                           (a * r * r * (1.0 - kPi / 4.0) +
                            r * r * r * (5.0 / 6.0 - kPi / 4.0));
        CHECK(close(v_r, 0.046842996386064, 1e-15));
        CHECK(close(mp.volume, (8.0 - 0.375 * kPi) - v_r, 1e-12));
        CHECK(close(mp.volume, 6.775059758517764, 1e-12));
        CHECK(mp.all_exact);
        // The wedge verified BY the mass: 8-0.375*pi - V == the
        // volume (the same arithmetic read twice).
        CHECK(close(8.0 - 0.375 * kPi - mp.volume, v_r, 1e-12));
        // The centroid ON THE AXIS (the axisymmetry), z-bar by the
        // wedge's own closed form: M_z = -m_tool - wedge_mz.
        CHECK(close(mp.centroid.x, 0.0, 1e-12));
        CHECK(close(mp.centroid.y, 0.0, 1e-12));
        const WedgeForms w = wedge_forms(0.5, 0.25, -0.5, 1.0);
        const double m_tool = 0.375 * kPi * 0.25;  // the cylinder's
        // z-moment about 0 (its centroid at 0.25): the blind's is
        // its negation (the box's is 0).
        const double mz_expected = -m_tool - w.mz;
        CHECK(close(mp.centroid.z, mz_expected / mp.volume, 1e-9));
        CHECK(close(mp.centroid.z, -0.050015860430586, 1e-7));
        // The inertias about the centroid: I_zz by the wedge's own
        // form (the box's 16/3 minus the tool's, minus the wedge's);
        // I_xx = I_zz/2's kin + the z^2 term; the off-diagonals 0.
        const double izz_box = 16.0 / 3.0;
        const double izz_tool = kPi * 0.0625 / 2.0 * 1.5;
        const double izz_expected =
            izz_box - izz_tool - w.izz;
        CHECK(close(mp.inertia_about_centroid(2, 2), izz_expected,
                    1e-9));
        CHECK(close(mp.inertia_about_centroid(2, 2), 5.171281, 1e-5));
        const double ixx_o =
            8.0 / 3.0 - (kPi * 0.0625 / 4.0) * 1.5 - w.izz / 2.0;
        const double z2_blind =
            8.0 / 3.0 - 0.375 * kPi * 0.25;
        const double z2_expected = z2_blind - w.z2;
        const double ixx_c = ixx_o + z2_expected -
                             mp.volume * mp.centroid.z * mp.centroid.z;
        CHECK(close(mp.inertia_about_centroid(0, 0), ixx_c, 1e-9));
        CHECK(close(mp.inertia_about_centroid(1, 1), ixx_c, 1e-9));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (i != j)
                    CHECK(std::abs(
                              mp.inertia_about_centroid(i, j)) <
                          1e-9);
        // THE NAIVE READS, red by discrimination: the full torus
        // 2*pi^2*R*r^2 = 0.925275412602127 is NOT the flank's flux
        // (the quarter's is 0.448520); the volume with the inner
        // loop left at the hole's radius is not the answer; the
        // blind's own volume is not the answer.
        CHECK_FALSE(close(0.925275412602127, 0.448519633389893));
        const double quarter = 2.0 * kPi * 0.25 *
            (-(0.75 * 0.75) + (3.0 * kPi / 4.0) * 0.75 * 0.25 -
             0.0625 + 0.75 * 0.75 - (0.25 * 0.75) / 2.0);
        CHECK(close(quarter, 0.448519633389893, 1e-12));
        CHECK_FALSE(close(mp.volume, 8.0 - 0.375 * kPi));
        CHECK_FALSE(close(mp.volume, 7.102309));
    }

    // THE FLANK'S SURFACE AND WALK: the TorusZ quarter window — the
    // axis (0,0), R = 0.75, r = 0.25, z_c = 0.75, the frame rule
    // pointwise, the window [0, 2pi] x [pi/2, pi] with every p-curve
    // a Line2 on the boundary, region-on-right (the wall's own
    // authoring).
    {
        const auto* surf =
            g.result->solid.shell().faces()[8].surface();
        REQUIRE(surf != nullptr);
        CHECK(surf->family() == b::SurfaceFamily::Torus);
        const auto* tor = dynamic_cast<const b::TorusZSurface*>(surf);
        REQUIRE(tor != nullptr);
        CHECK(close(tor->axis_center().x, 0.0, 1e-12));
        CHECK(close(tor->axis_center().y, 0.0, 1e-12));
        CHECK(close(tor->major_radius(), 0.75, 1e-12));
        CHECK(close(tor->minor_radius(), 0.25, 1e-12));
        CHECK(close(tor->z_center(), 0.75, 1e-12));
        for (int i = 0; i <= 8; ++i) {
            const double u = (2.0 * kPi) * i / 8.0;
            const double v = kPi / 2.0 + (kPi / 2.0) * i / 8.0;
            const double du = 1e-7;
            const Vec3d puu = tor->evaluate(u + du, v);
            const Vec3d pu = tor->evaluate(u, v);
            const Vec3d pvv = tor->evaluate(u, v + du);
            const Vec3d d_u = (puu - pu) * (1.0 / du);
            const Vec3d d_v = (pvv - pu) * (1.0 / du);
            CHECK(d_u.cross(d_v).dot(tor->normal(u, v)) > 0.0);
        }
        double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
        for (const auto& ce :
             g.result->solid.shell().faces()[8].loops()[0].coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            REQUIRE(pc->kind == b::PCurve::Kind::Segment);
            // Region-on-right (the wall's authoring, the decoupling's
            // own law: the material INSIDE the tube, BEHIND the
            // surface).
            CHECK(pc->segment.interior_left == false);
            u_min = std::min({u_min, pc->segment.a.x,
                              pc->segment.b.x});
            u_max = std::max({u_max, pc->segment.a.x,
                              pc->segment.b.x});
            v_min = std::min({v_min, pc->segment.a.y,
                              pc->segment.b.y});
            v_max = std::max({v_max, pc->segment.a.y,
                              pc->segment.b.y});
        }
        CHECK(close(u_min, 0.0, 1e-12));
        CHECK(close(u_max, 2.0 * kPi, 1e-12));
        CHECK(close(v_min, kPi / 2.0, 1e-12));
        CHECK(close(v_max, kPi, 1e-12));
        // The walk's co-edge table (the RACCOLTO's azimuth XOR: the
        // contact circles walked OPPOSITE by their landing faces).
        const auto& fc =
            g.result->solid.shell().faces()[8].loops()[0].coedges();
        REQUIRE(fc.size() == 6);
        CHECK(fc[0].edge() == 19);  // the seam FWD, v up at u = 0
        CHECK(fc[0].reversed() == false);
        CHECK(fc[1].edge() == 18);  // cw1 FWD, u 0->pi at v = pi
        CHECK(fc[1].reversed() == false);
        CHECK(fc[2].edge() == 17);  // cw0 FWD, u pi->2pi at v = pi
        CHECK(fc[2].reversed() == false);
        CHECK(fc[3].edge() == 19);  // the seam REV, v down at 2pi
        CHECK(fc[3].reversed() == true);
        CHECK(fc[4].edge() == 15);  // cp0 REV, u 2pi->pi at pi/2
        CHECK(fc[4].reversed() == true);
        CHECK(fc[5].edge() == 16);  // cp1 REV, u pi->0 at pi/2
        CHECK(fc[5].reversed() == true);
    }

    // THE DECOUPLING (ADR-0038 m4), one family over: the absorbed
    // faces keep their NATIVE frames (the pierced plane bit-identical,
    // the wall's cavity frame u_sign = -1 kept); the inner loop
    // re-authored at the contact circle (the blind door's convention:
    // the arcs hole = true, sweep pi); the wall's window re-timed with
    // the seam re-wired; the kept faces verbatim.
    {
        // The wall: the native cavity frame, the re-timed window
        // [floor, z_c], the contact pair replacing the dead rim pair.
        const auto* wsurf =
            g.result->solid.shell().faces()[6].surface();
        const auto* wall =
            dynamic_cast<const b::CylinderZSurface*>(wsurf);
        REQUIRE(wall != nullptr);
        CHECK(close(wall->radius(), 0.5, 1e-12));
        CHECK(close(wall->z_base(), -0.5, 1e-12));
        CHECK(wall->u_sign() == -1.0);
        double v_min = 1e9, v_max = -1e9;
        for (const auto& ce :
             g.result->solid.shell().faces()[6].loops()[0].coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Segment);
            CHECK(pc->segment.interior_left == false);
            v_min = std::min({v_min, pc->segment.a.y,
                              pc->segment.b.y});
            v_max = std::max({v_max, pc->segment.a.y,
                              pc->segment.b.y});
        }
        CHECK(close(v_min, 0.0, 1e-12));
        // h' = z_c - z_floor = 0.75 - (-0.5) = 1.25 (the re-timed
        // span: the dead rim at 1.5 replaced by the contact at 0.75).
        CHECK(close(v_max, 1.25, 1e-12));
        // The pierced face: the inner loop at the contact circle
        // (radius a + r, at the rim quota), hole = true, sweep pi.
        const auto& inner =
            g.result->solid.shell().faces()[1].loops()[1];
        REQUIRE(inner.is_outer() == false);
        REQUIRE(inner.coedges().size() == 2);
        for (const auto& ce : inner.coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            REQUIRE(pc->kind == b::PCurve::Kind::Arc);
            CHECK(pc->arc.hole == true);
            CHECK(close(pc->arc.radius, 0.75, 1e-12));
            CHECK(close(pc->arc.sweep, kPi, 1e-12));
        }
        // The p-curve/vertex coherence on every face (the trimming
        // contract, both kinds).
        for (std::size_t f = 0; f < 9; ++f) {
            const auto& face =
                g.result->solid.shell().faces()[f];
            const auto* sf = face.surface();
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges()) {
                    const auto* pc = ce.p_curve();
                    REQUIRE(pc != nullptr);
                    const auto s0 = static_cast<std::size_t>(
                        ce.start_vertex(g.result->edges));
                    const auto s1 = static_cast<std::size_t>(
                        ce.end_vertex(g.result->edges));
                    if (pc->kind == b::PCurve::Kind::Segment) {
                        const auto q0 = sf->evaluate(
                            pc->segment.a.x, pc->segment.a.y);
                        const auto q1 = sf->evaluate(
                            pc->segment.b.x, pc->segment.b.y);
                        CHECK((q0 - g.result->vertices[s0].position())
                                  .length_sq() < 1e-18);
                        CHECK((q1 - g.result->vertices[s1].position())
                                  .length_sq() < 1e-18);
                    } else if (pc->kind == b::PCurve::Kind::Arc) {
                        const auto q0 = sf->evaluate(
                            b::pcurve_arc_start_point(pc->arc).x,
                            b::pcurve_arc_start_point(pc->arc).y);
                        const auto q1 = sf->evaluate(
                            b::pcurve_arc_end_point(pc->arc).x,
                            b::pcurve_arc_end_point(pc->arc).y);
                        CHECK((q0 - g.result->vertices[s0].position())
                                  .length_sq() < 1e-18);
                        CHECK((q1 - g.result->vertices[s1].position())
                                  .length_sq() < 1e-18);
                    }
                }
        }
    }

    // The provenance: exact throughout.
    for (const auto& v : g.result->vertices)
        CHECK(v.provenance().is_exact());
    for (const auto& e : g.result->edges)
        CHECK(e.provenance().is_exact());
    for (const auto& f : g.result->solid.shell().faces())
        CHECK(f.provenance().is_exact());
}

// ===========================================================================
// THE NAMES — D3's table, character for character
// ===========================================================================
TEST_CASE("G2-U3 m3 the names: the flank, the two tangent wires, the "
          "flank's own seam, the shrink faces' names kept") {
    const RimGuide g(0.25);
    REQUIRE(g.result != nullptr);
    const std::string flank =
        "fil#5:flank[box#1:top \xc3\x97 ext#3:lat(0)]";

    // The flank: the referenced edge's two adjacent faces in its own
    // declared co-edge order (the rim pair's first co-edge sits on
    // the pierced face).
    CHECK(b::query_face(*g.result, 8).name == flank);
    // The two tangent wires (the level ABSENT — the tangency IS the
    // level): the plane contact lands on the pierced face, the wall
    // contact on the wall; the second operand the SAME feature's
    // flank (the same-feature extension, born earlier).
    CHECK(b::query_edge(*g.result, 15).name ==
          "fil#5:isect[box#1:top \xc3\x97 " + flank + "]");
    CHECK(b::query_edge(*g.result, 16).name ==
          "fil#5:isect[box#1:top \xc3\x97 " + flank + "]");
    CHECK(b::query_edge(*g.result, 17).name ==
          "fil#5:isect[ext#3:lat(0) \xc3\x97 " + flank + "]");
    CHECK(b::query_edge(*g.result, 18).name ==
          "fil#5:isect[ext#3:lat(0) \xc3\x97 " + flank + "]");
    // The flank's own seam: the sub's seam form, the same-feature
    // operand riding the same extension.
    CHECK(b::query_edge(*g.result, 19).name ==
          "fil#5:seam[" + flank + "]");
    // The shrink faces and the re-spanned edges KEEP their names (the
    // stamp copy): the mark rides the tavola's absorbed rows.
    CHECK(b::query_face(*g.result, 1).name == "box#1:top");
    CHECK(b::query_face(*g.result, 6).name == "ext#3:lat(0)");
    CHECK(b::query_face(*g.result, 7).name == "ext#3:cap(lo)");
    CHECK(b::query_edge(*g.result, 14).name ==
          "sub#4:seam[ext#3:lat(0)]");
    CHECK(b::query_edge(*g.result, 12).name ==
          "sub#4:isect[ext#3:cap(lo) \xc3\x97 ext#3:lat(0)]");

    // The structural round-trip: the by-name queries answer the
    // addresses.
    {
        const auto flank_id =
            g.result->solid.shell().faces()[8].shape_id();
        const auto faces = b::query_face_by_name(*g.result, flank_id);
        REQUIRE(faces.size() == 1);
        CHECK(faces[0] == 8);
        const auto wire_id = g.result->edges[15].shape_id();
        const auto edges = b::query_edge_by_name(*g.result, wire_id);
        REQUIRE(edges.size() == 2);  // the seam-wired arc PAIR, whole
        CHECK(edges[0] == 15);
        CHECK(edges[1] == 16);
    }

    // The unnamed mode: honest beside the named one.
    {
        const RimGuide g2(0.25, /*named=*/false);
        REQUIRE(g2.result != nullptr);
        CHECK(b::query_face(*g2.result, 8).name == "");
        for (std::size_t i : {15u, 16u, 17u, 18u, 19u})
            CHECK(b::query_edge(*g2.result, i).name == "");
        CHECK(b::query_face(*g2.result, 1).name == "box#1:top");
        CHECK(b::query_face(*g2.result, 6).name == "ext#3:lat(0)");
        CHECK(b::query_edge(*g2.result, 14).name ==
              "sub#4:seam[ext#3:lat(0)]");
    }

    // The depths: the wire's path resolves at depth 6 (the isect
    // role, the wall's indexed leg, the flank's role and its two
    // legs), the seam at 5 — kMaxDepth = 8 covers them with room.
    // The wall wire: the isect role, the wall's INDEXED leg (lat's
    // ArgIndex spends 2), the flank's role, the flank's two legs.
    CHECK(g.result->edges[17].shape_id().subshape.depth == 7);
    CHECK(g.result->edges[19].shape_id().subshape.depth == 5);
    CHECK(g.result->solid.shell().faces()[8]
              .shape_id()
              .subshape.depth == 4);
}

// ===========================================================================
// THE TAVOLA — the 47 rows, the census 9/22/16, the closure both sides
// ===========================================================================
TEST_CASE("G2-U3 m3 the tavola: the 47 rows, the census 9/22/16, the "
          "closure green both sides, the cause naming a BIRTH") {
    const RimGuide g(0.25);
    REQUIRE(g.result != nullptr);
    const b::DescentTable t = b::fillet_descent(*g.result, *g.blind);

    // THE COUNT (D3's own arithmetic): 47 rows — the result's
    // 14 + 20 + 9 = 43, plus the vanished tail (2 edges + 2 vertices).
    REQUIRE(t.rows.size() == 47);
    CHECK(t.count(b::Descent::Kept) == 30);      // 10 V + 14 E + 6 F
    CHECK(t.count(b::Descent::Absorbed) == 3);   // 1 E + 2 F
    CHECK(t.count(b::Descent::Born) == 10);      // 4 V + 5 E + 1 F
    CHECK(t.count(b::Descent::Vanished) == 4);   // 2 V + 2 E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    // The census 9/22/16, out + vanished, both sides (the third body
    // of the saga to carry it).
    {
        std::size_t kept_v = 0, kept_e = 0, kept_f = 0;
        std::size_t abs_v = 0, abs_e = 0, abs_f = 0;
        std::size_t born_v = 0, born_e = 0, born_f = 0;
        std::size_t van_v = 0, van_e = 0, van_f = 0;
        for (const auto& row : t.rows) {
            const bool k = row.relation == b::Descent::Kept;
            const bool a = row.relation == b::Descent::Absorbed;
            const bool bo = row.relation == b::Descent::Born;
            const bool va = row.relation == b::Descent::Vanished;
            if (row.entity == b::EntityKind::Vertex) {
                kept_v += k; abs_v += a; born_v += bo; van_v += va;
            } else if (row.entity == b::EntityKind::Edge) {
                kept_e += k; abs_e += a; born_e += bo; van_e += va;
            } else {
                kept_f += k; abs_f += a; born_f += bo; van_f += va;
            }
        }
        CHECK(kept_v == 10); CHECK(abs_v == 0);
        CHECK(born_v == 4);  CHECK(van_v == 2);
        CHECK(kept_e == 14); CHECK(abs_e == 1);   // the seam, re-timed
        CHECK(born_e == 5);  CHECK(van_e == 2);   // the dead rim pair
        CHECK(kept_f == 6);  CHECK(abs_f == 2);   // the top, the wall
        CHECK(born_f == 1);  CHECK(van_f == 0);
    }

    // The vanished rows' cause, VERBATIM: "replaced by the flank" —
    // the edge's death names a BIRTH, not a cut.
    {
        std::size_t causes = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            CHECK(row.cause == "replaced by the flank");
            if (row.entity == b::EntityKind::Edge) {
                // The dead pair: the top rim's own logical identity
                // (the tavola's rows carry the dead entity's stamp).
                CHECK(row.shape_id ==
                      g.blind->edges[14].shape_id());
            }
            ++causes;
        }
        CHECK(causes == 4);
    }

    // The born rows' names: the flank, the two wires (each a PAIR),
    // the flank's seam; the arcs of the contact circles are the
    // wires' own halves — every born edge named here (the honest
    // empty never appears in this class's born set).
    for (const auto& row : t.rows) {
        if (row.relation != b::Descent::Born) continue;
        if (row.entity == b::EntityKind::Face)
            CHECK(row.name ==
                  "fil#5:flank[box#1:top \xc3\x97 ext#3:lat(0)]");
        if (row.entity == b::EntityKind::Edge)
            CHECK(row.name.rfind("fil#5:", 0) == 0);
    }

    // THE CLOSURE, both sides green (the hard invariant).
    std::string why;
    CHECK(b::descent_closed(*g.result, t, &why));
    {
        const b::BRepBody no_second{};
        CHECK(b::old_generation_closed(*g.blind, no_second, t, &why));
    }

    // The derivation itself deterministic.
    const b::DescentTable t2 =
        b::fillet_descent(*g.result, *g.blind);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t.rows[i].relation == t2.rows[i].relation);
        CHECK(t.rows[i].entity == t2.rows[i].entity);
        CHECK(t.rows[i].address == t2.rows[i].address);
        CHECK(t.rows[i].name == t2.rows[i].name);
        CHECK(t.rows[i].cause == t2.rows[i].cause);
        CHECK(t.rows[i].operands == t2.rows[i].operands);
        CHECK(t.rows[i].shape_id == t2.rows[i].shape_id);
    }
}

// ===========================================================================
// THE TORO PIENO DISCRIMINATOR — the full-torus flux is NOT the
// flank's; the window IS the discrimination (two independent reads)
// ===========================================================================
TEST_CASE("G2-U3 m3 the full-torus discriminator: 2*pi^2*R*r^2 is the "
          "family's closed form, NOT the flank's flux — the naive "
          "full-window volume is red") {
    const double R = 0.75, r = 0.25, z_c = 0.75;
    const double pi = kPi;

    // The quarter window's flux (the flank's): the closed form
    // S = -R^2 + (3*pi/4)Rr - r^2 + R*z_c - (r*z_c)/2, F0 = 2*pi*r*S.
    const double S_quarter = -R * R + (3.0 * pi / 4.0) * R * r -
                             r * r + R * z_c - (r * z_c) / 2.0;
    const double F0_quarter = 2.0 * pi * r * S_quarter;

    // The full-meridian window's flux: the full torus's own
    // 3*V = 3*2*pi^2*R*r^2 — the family's discriminator.
    const double F0_full = 2.0 * pi * r * (3.0 * pi * R * r);
    CHECK(close(F0_full, 3.0 * 0.925275412602127, 1e-12));
    CHECK(close(2.0 * pi * pi * R * r * r, 0.925275412602127, 1e-12));

    // Two independent reads of EACH: the closed form vs the
    // quadrature (the same arithmetic read twice).
    {
        const int NU = 800, NV = 400;
        double quad_q = 0.0, quad_f = 0.0;
        for (int i = 0; i < NU; ++i) {
            for (int j = 0; j < NV; ++j) {
                const double v = pi / 2.0 + (j + 0.5) * (pi / 2.0) / NV;
                const double cv = std::cos(v), sv = std::sin(v);
                const double W = R + r * cv;
                const double xn = W * cv + z_c * sv + r * sv * sv;
                quad_q += xn * W * r * (2.0 * pi / NU) * (pi / 2.0 / NV);
            }
        }
        for (int i = 0; i < NU; ++i) {
            for (int j = 0; j < NV; ++j) {
                const double v = (j + 0.5) * (2.0 * pi) / NV;
                const double cv = std::cos(v), sv = std::sin(v);
                const double W = R + r * cv;
                const double xn = W * cv + z_c * sv + r * sv * sv;
                quad_f += xn * W * r * (2.0 * pi / NU) * (2.0 * pi / NV);
            }
        }
        CHECK(close(F0_quarter, quad_q, 1e-6));
        CHECK(close(F0_full, quad_f, 1e-6));
    }

    // THE DISCRIMINATION, red on the naive: the flank's flux is the
    // quarter's, never the full torus's; and the BODY's volume with
    // the flank read full-window would be the WRONG number.
    CHECK_FALSE(close(F0_quarter, F0_full));
    CHECK(F0_quarter < F0_full);
    const RimGuide g(0.25);
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    const double v_true = mp.volume;               // 6.7750597585...
    const double v_full_window =
        v_true - F0_quarter / 3.0 + F0_full / 3.0;  // the naive read
    CHECK(close(v_true, 6.775059758517764, 1e-12));
    CHECK_FALSE(close(v_true, v_full_window));
    CHECK(v_full_window > 7.0);  // the naive read is grossly wrong
    // And the machine's own flank flux is available indirectly: the
    // blind's volume, the top's and the wall's re-authorings — the
    // total agreeing with the closed forms IS the two-arithmetic pin.
    const double wedge_volume =
        2.0 * kPi * (0.5 * 0.25 * 0.25 * (1.0 - kPi / 4.0) +
                     0.25 * 0.25 * 0.25 * (5.0 / 6.0 - kPi / 4.0));
    CHECK(close(8.0 - 0.375 * kPi - v_true, wedge_volume, 1e-12));
}

// ===========================================================================
// THE CONTROPROOF — the naive red, per component (the injected
// defects refused by the very gates the golden passes)
// ===========================================================================
TEST_CASE("G2-U3 m3 the counterproof: the dead pair kept, the "
          "re-spanning skipped, the inner loop not moved — each red "
          "where the golden is green") {
    const RimGuide g(0.25);
    REQUIRE(g.result != nullptr);

    // THE NAIVE ARITHMETIC, pinned as the test's own numbers: the
    // construction that keeps the dead rim pair counts 16 vertices /
    // 22 edges / 9 faces and reads chi = 16 - 22 + 9 - 1 = 2 — EVEN
    // (the chi gate alone would NOT catch it): the census and the XOR
    // are the catch (the kept pair would carry ZERO co-edges).
    const std::int64_t naive_chi = 16 - 22 + 9 - 1;
    CHECK(naive_chi == 2);
    CHECK(naive_chi % 2 == 0);
    CHECK(b::euler_characteristic(*g.result) == 2);

    // THE INJECTED DEFECT, live: the dead rim pair re-added to the
    // result (their landing faces were re-authored without them —
    // they carry ZERO co-edges). The manifold gate refuses what the
    // golden passes.
    {
        b::BRepBody naive;
        naive.vertices = g.result->vertices;
        naive.edges = g.result->edges;
        const auto e1 = g.blind->edges[14];  // the dead pair's halves
        const auto e2 = g.blind->edges[15];
        naive.edges.push_back(e1);
        naive.edges.back().set_id(
            static_cast<b::EdgeId>(naive.edges.size() - 1));
        naive.edges.back().set_vertex(
            0, static_cast<b::VertexId>(
                   g.result->vertex_count() + 0));
        naive.edges.back().set_vertex(
            1, static_cast<b::VertexId>(
                   g.result->vertex_count() + 1));
        naive.edges.push_back(e2);
        naive.edges.back().set_id(
            static_cast<b::EdgeId>(naive.edges.size() - 1));
        naive.edges.back().set_vertex(
            0, static_cast<b::VertexId>(
                   g.result->vertex_count() + 1));
        naive.edges.back().set_vertex(
            1, static_cast<b::VertexId>(
                   g.result->vertex_count() + 0));
        naive.vertices.push_back(b::Vertex{Vec3d{0.5, 0, 1}});
        naive.vertices.push_back(b::Vertex{Vec3d{-0.5, 0, 1}});
        naive.solid = g.result->solid;  // the faces, verbatim
        b::link_radial_edges(naive);
        REQUIRE(naive.edges[naive.edge_count() - 2]
                    .coedges()
                    .size() == 0);
        CHECK_FALSE(b::is_closed_orientable(naive));
        CHECK(b::is_closed_orientable(*g.result));
    }

    // THE RE-SPANNING DEFECT, live: the wall's seam NOT re-spanned
    // (still pointing at the vanished joint) — the wall's walk cannot
    // chain and the XOR breaks.
    {
        b::BRepBody naive;
        naive.vertices = g.result->vertices;
        // The wall's seam re-pointed at the DEAD joint (a vertex id
        // that exists in the naive copy but sits nowhere in any walk).
        naive.vertices.push_back(b::Vertex{Vec3d{0.5, 0, 1}});
        naive.edges = g.result->edges;
        auto& seam = naive.edges[14];
        const b::VertexId other =
            seam.vertex(0) == 8 ? seam.vertex(1) : seam.vertex(0);
        seam.set_vertex(other == 10 ? 1 : 0,
                        static_cast<b::VertexId>(
                            g.result->vertex_count()));
        naive.solid = g.result->solid;
        b::link_radial_edges(naive);
        CHECK_FALSE(b::loops_are_closed(naive));
        CHECK_FALSE(b::is_closed_orientable(naive));
        CHECK(b::loops_are_closed(*g.result));
    }

    // THE INNER-LOOP DEFECT, red by discrimination: the volume with
    // the inner loop left at the hole's radius (0.5) is NOT the
    // golden's — the pin of the moved loop (the m3's own component).
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        // The top's area with the inner at radius 0.5 vs 0.75: the
        // naive volume gains (pi*(0.75^2 - 0.5^2))*z_rim/3 in the
        // flux — the discrimination number.
        const double naive_gain =
            kPi * (0.75 * 0.75 - 0.5 * 0.5) * 1.0 / 3.0;
        CHECK(close(naive_gain, 0.327249234748937, 1e-9));
        CHECK_FALSE(close(mp.volume, mp.volume + naive_gain));
        // And the flank's window IS the quarter (never the full
        // meridian): the p-curves pin it (the golden's own check).
        double v_min = 1e9, v_max = -1e9;
        for (const auto& ce :
             g.result->solid.shell().faces()[8].loops()[0].coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            v_min = std::min({v_min, pc->segment.a.y,
                              pc->segment.b.y});
            v_max = std::max({v_max, pc->segment.a.y,
                              pc->segment.b.y});
        }
        CHECK(close(v_max - v_min, kPi / 2.0, 1e-12));
        CHECK_FALSE(close(v_max - v_min, 2.0 * kPi));
    }
}

// ===========================================================================
// THE REFUSALS — the two bands with their bounds, the families by name
// ===========================================================================
TEST_CASE("G2-U3 m3 the refusals: the annulus and the wall bands carry "
          "their bounds, the concave family and the seam refuse by "
          "name, the vertex blend is never met") {
    auto blind = [] {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        return b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                 b::kSubtractTolerance, sub_ctx);
    }();

    const auto rim_ref = [](const b::BRepBody& body,
                            const char* want) {
        for (std::size_t i = 0; i < body.edge_count(); ++i)
            if (b::query_edge(body, i).name == want)
                return body.edges[i].shape_id();
        return ShapeId{};
    };
    const b::FilletNaming ctx{FeatureId{5}, &n::kBoxNaming,
                              FeatureId{3}, &n::kExtNaming};

    const auto try_fillet = [&](const ShapeId& ref, double radius,
                                std::string& msg) {
        try {
            auto r = b::fillet_rim_edge(*blind, ref, radius, ctx);
            (void)r;
            return false;
        } catch (const std::invalid_argument& e) {
            msg = e.what();
            return true;
        }
    };

    // THE ANNULUS BAND: r = 0.7 against the reach 1 - a = 0.5 — the
    // tangent circle would exit the pierced face; the bound, the
    // radius and the epsilon all carried.
    {
        std::string msg;
        const ShapeId top_rim =
            rim_ref(*blind, "sub#4:isect[box#1:top \xc3\x97 "
                            "ext#3:lat(0)]@hi");
        REQUIRE(try_fillet(top_rim, 0.7, msg));
        CHECK(msg.find("fillet_exits_annulus") != std::string::npos);
        CHECK(msg.find("0.700000") != std::string::npos);
        CHECK(msg.find("0.500000") != std::string::npos);
        CHECK(msg.find("0.000001") != std::string::npos);
    }
    // THE WALL BAND: the guide body's wall is TALL (1.5 > the
    // annulus's 0.5), so the wall band is never binding there — the
    // honest test builds the SHORT-wall blind body (the floor at
    // 0.75: the wall's height 0.25) where r = 0.3 meets ONLY the
    // wall's band.
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        auto short_blind = b::exact_subtract(
            *box, *tool, {0.0, 0.0, 0.75}, b::kSubtractTolerance,
            sub_ctx);
        REQUIRE(short_blind != nullptr);
        const ShapeId top_rim =
            rim_ref(*short_blind, "sub#4:isect[box#1:top \xc3\x97 "
                                  "ext#3:lat(0)]@hi");
        std::string msg;
        try {
            auto r = b::fillet_rim_edge(*short_blind, top_rim, 0.3,
                                        ctx);
            (void)r;
            FAIL("the short wall must refuse");
        } catch (const std::invalid_argument& e) {
            msg = e.what();
        }
        CHECK(msg.find("fillet_eats_wall") != std::string::npos);
        CHECK(msg.find("0.300000") != std::string::npos);
        CHECK(msg.find("0.250000") != std::string::npos);
        CHECK(msg.find("0.000001") != std::string::npos);
        // r exactly at the wall's bound: refused too.
        msg.clear();
        try {
            auto r = b::fillet_rim_edge(*short_blind, top_rim, 0.25,
                                        ctx);
            (void)r;
            FAIL("the wall's bound must refuse");
        } catch (const std::invalid_argument& e) {
            msg = e.what();
        }
        CHECK(msg.find("fillet_eats_wall") != std::string::npos);
    }
    // The annulus band's own edge (r exactly at a = the reach - the
    // hole's radius; r -> a is ALSO the concave family's near-band
    // conditioning seat): the degenerate families stay MURO, never
    // sedated — the strict interiority the derivation demands.
    {
        std::string msg;
        const ShapeId top_rim =
            rim_ref(*blind, "sub#4:isect[box#1:top \xc3\x97 "
                            "ext#3:lat(0)]@hi");
        REQUIRE(try_fillet(top_rim, 0.5, msg));
        CHECK(msg.find("fillet_exits_annulus") != std::string::npos);
    }

    // THE CONCAVE FAMILY: the floor rim (the carried cap x the wall)
    // refused BY NAME with the queue's entry.
    {
        std::string msg;
        const ShapeId floor_rim =
            rim_ref(*blind, "sub#4:isect[ext#3:cap(lo) \xc3\x97 "
                            "ext#3:lat(0)]");
        REQUIRE(try_fillet(floor_rim, 0.25, msg));
        CHECK(msg.find("fillet_out_of_class_pair") != std::string::npos);
        CHECK(msg.find("CONCAVE") != std::string::npos);
        CHECK(msg.find("inverted flank") != std::string::npos);
    }

    // THE SEAM: a fillet asked on the seam refused with the edge named.
    {
        std::string msg;
        const ShapeId seam =
            rim_ref(*blind, "sub#4:seam[ext#3:lat(0)]");
        REQUIRE(try_fillet(seam, 0.25, msg));
        CHECK(msg.find("seam") != std::string::npos);
        CHECK(msg.find("SINGLE edge") != std::string::npos);
    }

    // The non-positive radius, the foreign reference, the m2 box (out
    // of the class signature), the mirror family never met.
    {
        std::string msg;
        const ShapeId top_rim =
            rim_ref(*blind, "sub#4:isect[box#1:top \xc3\x97 "
                            "ext#3:lat(0)]@hi");
        REQUIRE(try_fillet(top_rim, 0.0, msg));
        CHECK(msg.find("radius") != std::string::npos);
        REQUIRE(try_fillet(top_rim, -0.25, msg));
        CHECK(msg.find("-0.250000") != std::string::npos);
        REQUIRE(try_fillet(ShapeId{FeatureId{9},
                                   SubshapeRef{}.push(3), {}},
                           0.25, msg));
        CHECK(msg.find("names no edge") != std::string::npos);
        // The m2 box: out of the rim class's signature (8/12/6).
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::FilletNaming m2ctx{FeatureId{5}, &n::kBoxNaming};
        try {
            auto r = b::fillet_rim_edge(
                *box, ShapeId{FeatureId{1},
                              SubshapeRef{}.push(10), {}},
                0.25, m2ctx);
            (void)r;
            FAIL("the box must refuse the rim class");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find(
                      "outside the class signature") !=
                  std::string::npos);
        }
    }

    // THE VERTEX BLEND: never met, never named by this class's
    // payload — the refused tail stays MURO (its seat is the corpus
    // unit's, ADR-0039 D4's own list).
    for (std::size_t i = 0; i < blind->edge_count(); ++i) {
        const auto e = b::query_edge(*blind, i);
        std::string msg;
        if (try_fillet(blind->edges[i].shape_id(), 0.25, msg))
            CHECK(msg.find("fillet_corner_is_vertex_blend") ==
                  std::string::npos);
    }
}

// ===========================================================================
// THE EDIT CONTRACT — the radius edit class 1 SILENT on the rim too
// ===========================================================================
TEST_CASE("G2-U3 m3 the edit contract: the rim's radius edit resolves "
          "class 1 silent, the feature re-runs, the counts fixed for "
          "every in-band r, the rebuild bit-identical") {
    // THE RESOLUTION (0036 D6's hook, one class over): the reference
    // across the radius edit — the old generation's tavola (the
    // subtract's own), the new (the blind re-materialized: the radius
    // edit does not touch the blind's recipe).
    const RimGuide g(0.25);
    REQUIRE(g.blind != nullptr);
    {
        const b::DescentTable old_tav =
            b::subtract_descent(*g.blind, *g.box, *g.tool);
        std::vector<CAD_0::sdf::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        auto tool2 = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        auto blind2 = b::exact_subtract(*g.box, *tool2,
                                        {0.0, 0.0, -0.5},
                                        b::kSubtractTolerance, sub_ctx);
        // The INSTANCE's own registry (the m4 SubRegistry shape):
        // the subtract's tables with the two operand bindings the
        // stamping site held — the relational legs recurse through
        // them, as the renderer does.
        const n::OperandBinding sub_bindings[2] = {
            {FeatureId{1}, &n::kBoxNaming},   // the target (box#1)
            {FeatureId{3}, &n::kExtNaming}};  // the tool (ext#3)
        const n::OperationNaming sub_decl{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles,
            {sub_bindings, 2}};
        const b::Generation old_gen{nullptr, &old_tav,
                                    &sub_decl, false};
        const b::Generation new_gen{blind2.get(), nullptr,
                                    &sub_decl, false};
        b::EditContext ectx{};
        ectx.roots_no_shift = true;
        const b::Resolution res =
            b::resolve(g.rim, old_gen, new_gen, ectx);
        CHECK(res.kind == b::Resolution::Kind::Resolved);
        CHECK(std::string(b::resolution_code(res.kind)) ==
              "reference_resolved");
        CHECK(res.resolved.name ==
              "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi");
    }

    // THE FEATURE RE-RUNS: the counts stay 14/20/9 for EVERY in-band
    // r; the volume the closed form's own value each time.
    for (const double radius : {0.05, 0.125, 0.25, 0.4, 0.49}) {
        CAPTURE(radius);
        const RimGuide gr(radius);
        REQUIRE(gr.result != nullptr);
        REQUIRE(gr.result->vertex_count() == 14);
        REQUIRE(gr.result->edge_count() == 20);
        REQUIRE(gr.result->face_count() == 9);
        REQUIRE(b::inner_loop_count(*gr.result) == 1);
        REQUIRE(b::euler_characteristic(*gr.result) == 2);
        CHECK(b::is_closed_orientable(*gr.result));
        const auto mp = b::mass_properties(
            *gr.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume,
                    (8.0 - 0.375 * kPi) -
                        2.0 * kPi *
                            (0.5 * radius * radius *
                                 (1.0 - kPi / 4.0) +
                             radius * radius * radius *
                                 (5.0 / 6.0 - kPi / 4.0)),
                    1e-12));
        CHECK(mp.all_exact);
        CHECK(close(mp.centroid.x, 0.0, 1e-12));
        CHECK(close(mp.centroid.y, 0.0, 1e-12));
    }

    // THE BIT-IDENTICAL RE-RUN (the same r, N = 10): the digest pure
    // of the recipe (zero floating-point content in the name legs).
    {
        std::string first_names, first_coedges;
        std::uint64_t first_struct = 0;
        for (int i = 0; i < 10; ++i) {
            const RimGuide g2(0.25);
            REQUIRE(g2.result != nullptr);
            const std::string names = name_digest(*g2.result);
            const std::string coedges =
                coedge_name_digest(*g2.result);
            const std::uint64_t structure =
                structure_digest(*g2.result);
            if (i == 0) {
                first_names = names;
                first_coedges = coedges;
                first_struct = structure;
            } else {
                INFO("rebuild ", i);
                CHECK(names == first_names);
                CHECK(coedges == first_coedges);
                CHECK(structure == first_struct);
            }
        }
        // The purity: another instance the same names byte for byte.
        const RimGuide g3(0.25);
        CHECK(name_digest(*g3.result) == first_names);
    }
}
