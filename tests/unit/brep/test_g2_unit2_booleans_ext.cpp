// tests/unit/brep/test_g2_unit2_booleans_ext.cpp
//
// G2 UNIT 2 — THE EXTENDED BOOLEANS: THE OPERATIONS CONSUME THE TABLE
// (ADR-0033 D5.2, over D1.2/D2 of the accepted ADR — the wall). Written
// FIRST per the house discipline: today they fail for the right reason
// (the operations, the gates and the trimmed-sphere mass family do not
// exist yet; the build itself names the absence).
//
// THE DERIVED TABLES, pinned BEFORE the goldens (the D7.5 analog the
// order demands — every row hand-checkable, the derivations in the
// header of booleans_ext.hpp):
//
//   THE TWO-HOLE GUIDE CASE (box a×b×c, TWO parallel through holes of
//   radius r, circles strictly interior and disjoint — the second cut
//   on a pierced target):
//
//     | Quantity | After cut 1 | After cut 2 | Source                    |
//     |----------|-------------|-------------|---------------------------|
//     | V        | 12          | 16          | +4 per cut (S,A × bottom, |
//     |          |             |             | top — the seam split)     |
//     | E        | 17          | 22          | +5 per cut (2 bottom arcs |
//     |          |             |             | + 2 top arcs + 1 seam)    |
//     | F        | 7           | 8           | +1 wall per cut           |
//     | H        | 2           | 4           | +1 inner loop per pierced |
//     |          |             |             | face per cut              |
//     | χ        | 0           | −2          | 16−22+8−4 = 2(1−2)        |
//     | genus    | 1           | 2           | each through hole +1      |
//     | volume   | abc−πr²c    | abc−2πr²c   | the classical             |
//
//   THE SPHERE×SPHERE PAIR (R₁, R₂, D in the open band; the radical
//   plane at h = (D²+R₁²−R₂²)/2D, ρ = √(R₁²−h²), cap(R,h) =
//   πh²(3R−h)/3 — the classical formulas ARE the tests):
//
//     | Body      | V | E | F | H | χ | genus | volume                     |
//     |-----------|---|---|---|---|---|-------|----------------------------|
//     | union     | 4 | 4 | 2 | 0 | 2 | 0     | V₁+V₂−cap(R₁,R₁−h)−        |
//     |           |   |   |   |   |   |       | cap(R₂,R₂−D+h)             |
//     | lens (∩)  | 4 | 4 | 2 | 0 | 2 | 0     | cap(R₁,R₁−h)+cap(R₂,R₂−D+h)|
//     | bitten (−)| 3 | 3 | 2 | 0 | 2 | 0     | cap(R₁,R₁+h)               |
//
//     (union/lens: P₁, P₂ the two poles, S, A the rim split at the
//     radical plane's own u axis; the seams P_i↔S, the rim split into
//     two π arcs. bitten: the far cap + the radical-plane disk, the
//     rim pair shared.)
//
//   THE SPHERE × CIRCLE-PRISM (coaxial, r < R, the tool's z-span
//   covering [−R, R]; w = √(R²−r²), cap height h_c = R−w, the core =
//   πr²·2w + 2·cap(R, h_c)):
//
//     | Body       | V | E  | F | H | χ | genus | volume            |
//     |------------|---|----|---|---|---|-------|-------------------|
//     | bead (−)   | 4 | 6  | 2 | 0 | 0 | 1     | (4/3)πR³ − core   |
//     | rivet (∪)  | 8 | 11 | 5 | 0 | 2 | 0     | (4/3)πR³+πr²L−core|
//     | core (∩)   | 6 | 7  | 3 | 0 | 2 | 0     | core              |
//
//     (bead: the equatorial ZONE + the wall band between the circles;
//     rivet: the zone + two wall bands + two end disks; core: the two
//     polar caps + the middle wall band.)
//
//   THE SPHERE × BOX-PROFILE (the walls of ONE axis family cutting,
//   the caps clear — the truncation semantics; d₀ the extreme cut
//   offset; every cap face needs its SEAM, hence the pole vertex):
//
//     | Body        | V  | E  | F | H | χ | volume                |
//     |-------------|----|----|---|---|---|-----------------------|
//     | far cap (−) | 3  | 3  | 2 | 0 | 2 | cap(R, R+d₀)          |
//     | near cap (∩)| 3  | 3  | 2 | 0 | 2 | cap(R, R−d₀)          |
//     | dome (∪)    | 11 | 15 | 7 | 1 | 2 | V_box+V_sphere−       |
//     |             |    |    |   |   |   | cap(R, R−d₀)          |
//     | slab (−,    | 6  | 6  | 4 | 0 | 4 | 2·cap(R, R−a)  (S=2)   |
//     |  straddling)|    |    |   |   |   |                       |
//
//     (dome: the box's bottom face GAINS the latitude circle as an
//     inner loop — the G1 pierced-face pattern on the tool's own
//     plane; slab: the straddling box keeps TWO far caps + two cut
//     disks.)
//
// THE DEGENERATES, each with its pinned disposition (the D7.4 analog):
//   * COINCIDENT spheres (equal centers, equal radii): the CO-RADIAL
//     DEDUP — union/intersect keep ONE surface (the operand verbatim,
//     the other absorbed); subtract refuses (the empty, the swallow
//     disposition);
//   * TANGENCIES (external D=R₁+R₂, internal D=|R₁−R₂|): the contact
//     is a POINT — the topology flips non-manifold — REFUSED with the
//     remedy (both ops);
//   * NESTED / SEPARATE: union keeps the operand (nested) / refuses
//     (separate: the disconnected union); intersect keeps the smaller
//     (nested) / refuses (separate: the empty); subtract keeps the
//     operand (both — the miss, the G1 tie-break);
//   * THE BEAD'S degenerates: the blind tool (a cap inside — the G1
//     disposition), r ≥ R (the empty), r = R (the tangent circle),
//     NON-COAXIAL (the Viviani quartic — the wall);
//   * THE TWO-HOLE special: the two circles TANGENT or CROSSING on
//     the pierced face → NON-ANNULAR → refused; the CROSSING
//     UNEQUAL-radii refusal NAMES the offset plane x =
//     (d²+r₁²−r₂²)/(2d) and its two typed lines — admitted by the
//     criterion, awaiting the consuming unit, NEVER "untypable"; the
//     NESTED circles (the new hole inside the old): the MISS, the
//     exact copy;
//   * THE FRAME BUDGET applied: the trimmed survivor carries ONE axis
//     family; the incompatible family of a later operation finds the
//     wall (the bead − an X-family box); the two-family gate refuses
//     at entry (the box whose x AND y walls both cut); the compatible
//     second cut on a budgeted face refuses naming the multi-cut
//     cascade;
//   * THE PROVENANCE GATE: an approximated operand refuses (the
//     honesty chain — never laundered).
//
// THE CONVENTION THIS UNIT DISCOVERS (pinned here as a regression
// guard, the derivation in booleans_ext.hpp's header): the sphere's
// re-frame azimuth for the cylinder pair is u_sign·(the wall's u=0
// meridian) — the wall's increasing-u direction at the seam — so the
// shared-rim walks oppose and the radial-edge XOR holds with both
// faces reading their regions material-on-left.
#include <doctest/doctest.h>

#include "CAD_0/brep/booleans_ext.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::brep::EdgeId;
using CAD_0::brep::FaceId;
using CAD_0::brep::VertexId;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3f;
namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;

constexpr double kPid = 3.14159265358979323846;

bool close(float a, float b, float tol = 1e-4f) { return std::abs(a - b) <= tol; }

// The classical spherical cap volume — the TEST, never the
// implementation (the machinery integrates the divergence theorem).
double cap_volume(double R, double h) {
    return kPid * h * h * (3.0 * R - h) / 3.0;
}

// ---- the operand builders ------------------------------------------------

std::vector<s::SketchChain> circle_chains(Vec2f center = {0.0f, 0.0f},
                                          float radius = 0.25f) {
    return {s::SketchChain{{s::sketch_circle(center, radius, false)}}};
}

// A Z-axis circle prism spanning [z_lo, z_hi] (make_prism extrudes
// from 0 up, so the offset carries the placement).
std::unique_ptr<BRepBody> circle_prism(float radius, float z_lo, float z_hi,
                                       Vec2f center = {0.0f, 0.0f}) {
    auto tool = b::make_prism(circle_chains(center, radius), z_hi - z_lo, +1);
    b::translate_body(*tool, Vec3f{0.0f, 0.0f, z_lo});
    return tool;
}

// ---- the structural contract (the G1-u4 helpers, reused) -----------------

void check_exact_provenance(const BRepBody& body) {
    for (const auto& v : body.vertices) {
        CHECK(v.provenance().is_exact());
        CHECK(v.provenance().linear_tolerance == 0.0);
    }
    for (const auto& e : body.edges) {
        CHECK(e.provenance().is_exact());
        CHECK(e.provenance().linear_tolerance == 0.0);
    }
    for (const auto& f : body.solid.shell().faces()) {
        CHECK(f.provenance().is_exact());
        CHECK(f.provenance().linear_tolerance == 0.0);
    }
}

void check_body_invariants(const BRepBody& body, std::size_t V, std::size_t E,
                           std::size_t F, std::int64_t H, std::int64_t chi) {
    REQUIRE(body.vertex_count() == V);
    REQUIRE(body.edge_count() == E);
    REQUIRE(body.face_count() == F);
    REQUIRE(b::inner_loop_count(body) == H);
    REQUIRE(b::euler_characteristic(body) == chi);
    CHECK(b::is_closed_orientable(body));
    CHECK(b::loops_are_closed(body));
    for (std::size_t i = 0; i < body.vertices.size(); ++i)
        CHECK(body.vertices[i].id() == static_cast<VertexId>(i));
    for (std::size_t i = 0; i < body.edges.size(); ++i)
        CHECK(body.edges[i].id() == static_cast<EdgeId>(i));
    for (std::size_t i = 0; i < body.face_count(); ++i)
        CHECK(body.solid.shell().faces()[i].id() ==
              static_cast<FaceId>(i));
    for (const auto& e : body.edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() != e.coedges()[1]->reversed());
    }
}

// Every exact face's every co-edge carries a typed p-curve (the
// trimming contract — the new families included).
void check_typed_pcurves(const BRepBody& body) {
    for (const auto& f : body.solid.shell().faces()) {
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges()) {
                const auto* pc = ce.p_curve();
                REQUIRE(pc != nullptr);
                const bool typed = pc->kind == s::SketchEntity::Kind::Segment ||
                                   pc->kind == s::SketchEntity::Kind::Arc ||
                                   pc->kind == s::SketchEntity::Kind::Circle;
                CHECK(typed);
            }
    }
}

// --- the digest (ADR-0014): every number as a hex float ------------------

void digest_vertex(std::string& out, const b::Vertex& v) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "V%llu:(%a,%a,%a)",
                  static_cast<unsigned long long>(v.id()),
                  static_cast<double>(v.position().x),
                  static_cast<double>(v.position().y),
                  static_cast<double>(v.position().z));
    out += buf;
}

void digest_pcurve(std::string& out, const s::SketchEntity& pc) {
    char buf[320];
    switch (pc.kind) {
    case s::SketchEntity::Kind::Segment:
        std::snprintf(buf, sizeof buf, "L(%a,%a;%a,%a;%d)",
                      static_cast<double>(pc.segment.a.x),
                      static_cast<double>(pc.segment.a.y),
                      static_cast<double>(pc.segment.b.x),
                      static_cast<double>(pc.segment.b.y),
                      pc.segment.interior_left ? 1 : 0);
        break;
    case s::SketchEntity::Kind::Circle:
        std::snprintf(buf, sizeof buf, "C(%a,%a;%a;%d)",
                      static_cast<double>(pc.circle.center.x),
                      static_cast<double>(pc.circle.center.y),
                      static_cast<double>(pc.circle.radius),
                      pc.circle.hole ? 1 : 0);
        break;
    case s::SketchEntity::Kind::Arc:
        std::snprintf(buf, sizeof buf, "A(%a,%a;%a;%d;%a;%a)",
                      static_cast<double>(pc.arc.center.x),
                      static_cast<double>(pc.arc.center.y),
                      static_cast<double>(pc.arc.radius),
                      pc.arc.hole ? 1 : 0,
                      static_cast<double>(pc.arc.start_angle),
                      static_cast<double>(pc.arc.sweep));
        break;
    }
    out += buf;
}

std::string body_digest(const BRepBody& body) {
    std::string out;
    for (const auto& v : body.vertices) digest_vertex(out, v);
    for (const auto& e : body.edges) {
        char buf[80];
        std::snprintf(buf, sizeof buf, "E%llu:%llu>%llu",
                      static_cast<unsigned long long>(e.id()),
                      static_cast<unsigned long long>(e.vertex(0)),
                      static_cast<unsigned long long>(e.vertex(1)));
        out += buf;
    }
    for (const auto& f : body.solid.shell().faces()) {
        out += "F";
        out += f.surface() ? f.surface()->type_name() : "(none)";
        if (const auto* sp =
                dynamic_cast<const b::SphereSurface*>(f.surface())) {
            char buf[320];
            std::snprintf(buf, sizeof buf, "(R=%a,C=(%a,%a,%a),n=(%a,%a,%a),"
                          "e0=(%a,%a,%a))",
                          static_cast<double>(sp->radius()),
                          static_cast<double>(sp->origin().x),
                          static_cast<double>(sp->origin().y),
                          static_cast<double>(sp->origin().z),
                          static_cast<double>(sp->pole().x),
                          static_cast<double>(sp->pole().y),
                          static_cast<double>(sp->pole().z),
                          static_cast<double>(sp->azimuth().x),
                          static_cast<double>(sp->azimuth().y),
                          static_cast<double>(sp->azimuth().z));
            out += buf;
        }
        for (const auto& loop : f.loops()) {
            out += loop.is_outer() ? "[O" : "[I";
            for (const auto& ce : loop.coedges()) {
                char buf[48];
                std::snprintf(buf, sizeof buf, "<%llu%s>",
                              static_cast<unsigned long long>(ce.edge()),
                              ce.reversed() ? "R" : "F");
                out += buf;
                if (ce.p_curve()) digest_pcurve(out, *ce.p_curve());
            }
            out += "]";
        }
    }
    return out;
}

// The refusal guard: the message must carry the needle (the golden
// error, lowercase-searchable — the u1 discipline).
bool refuses_with(const char* needle, const std::string& message) {
    return message.find(needle) != std::string::npos;
}

template <typename F>
std::string refusal_of(F&& f) {
    try {
        f();
    } catch (const std::invalid_argument& ex) {
        return ex.what();
    } catch (const std::logic_error& ex) {
        return ex.what();
    }
    return {};
}

// The family census of a body's faces.
std::vector<std::string> face_families(const BRepBody& body) {
    std::vector<std::string> out;
    for (const auto& f : body.solid.shell().faces())
        out.push_back(f.surface() ? f.surface()->type_name() : "(none)");
    return out;
}

std::size_t count_family(const BRepBody& body, const std::string& family) {
    std::size_t n = 0;
    for (const auto& fam : face_families(body))
        if (fam == family) ++n;
    return n;
}

} // namespace

// ---------------------------------------------------------------------------
// THE SPHERE×SPHERE PAIR
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the sphere×sphere union — the classical, exact") {
    const auto A = b::make_sphere(1.0f, Vec3f{0, 0, 0});
    const auto B = b::make_sphere(0.8f, Vec3f{0, 0, 0});
    b::translate_body(*B, Vec3f{1.2f, 0, 0});  // D = 1.2 in the open band

    auto U = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(U != nullptr);

    // The derived table: 4/4/2/0, χ=2, genus 0.
    check_body_invariants(*U, 4, 4, 2, 0, 2);
    check_exact_provenance(*U);
    check_typed_pcurves(*U);

    // Two spherical patch faces — the TRIMMED family (the full factory
    // sphere is 1 face; the union's caps are re-framed survivors).
    REQUIRE(count_family(*U, "sphere") == 2);
    const auto& f0 = U->solid.shell().faces()[0];
    const auto* sp0 = dynamic_cast<const b::SphereSurface*>(f0.surface());
    REQUIRE(sp0 != nullptr);
    // The re-frame: the poles lie on the center line (±X), the azimuth
    // is the radical plane's own u axis (the PlaneSurface rule).
    CHECK(std::abs(std::abs(sp0->pole().x) - 1.0f) < 1e-5f);
    CHECK(std::abs(sp0->pole().y) < 1e-5f);
    CHECK(std::abs(sp0->pole().z) < 1e-5f);

    // The rim: the radical-plane circle, h = 0.75, ρ = √0.4375, split
    // at S/A on the plane's u axis (±Z here). The four vertices: the
    // two poles on the center line, the two split points in the plane.
    const double rho = std::sqrt(0.4375);
    bool saw_neg = false, saw_pos = false;
    for (const auto& v : U->vertices) {
        const float x = v.position().x;
        if (close(x, -1.0f, 1e-4f)) saw_neg = true;         // P of the far sphere
        if (close(x, 1.2f + 0.8f, 1e-4f)) saw_pos = true;   // P of the near one
        if (close(x, 0.75f, 1e-4f)) {                       // on the rim plane
            const double r2 = static_cast<double>(v.position().y) *
                                  v.position().y +
                              static_cast<double>(v.position().z) *
                                  v.position().z;
            CHECK(close(static_cast<float>(r2),
                        static_cast<float>(rho * rho), 1e-3f));
        }
    }
    CHECK(saw_neg);
    CHECK(saw_pos);

    // The classical volume: V₁ + V₂ − the two caps the union absorbs.
    const double want = (4.0 / 3.0) * kPid * (1.0 + 0.512) -
                        cap_volume(1.0, 0.25) - cap_volume(0.8, 0.35);
    const auto mp = b::mass_properties(*U, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    // Determinism: the same operands, the same bytes (ADR-0014).
    auto U2 = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    CHECK(body_digest(*U) == body_digest(*U2));
}

TEST_CASE("g2-u2: the union's operand-order independence") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(0.8f);
    b::translate_body(*B, Vec3f{1.2f, 0, 0});

    auto U1 = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    auto U2 = b::exact_union(*B, *A, Vec3f{0, 0, 0}, 1e-6f);
    // The canonical identity precedes the keying: the pair (A, B) and
    // (B, A) assemble the SAME body — bit-identical, never a mirrored
    // numbering that would differ per input order.
    CHECK(body_digest(*U1) == body_digest(*U2));
}

TEST_CASE("g2-u2: the bitten sphere (subtract) — the cap + the disk") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(0.8f);
    b::translate_body(*B, Vec3f{1.2f, 0, 0});

    auto R = b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(R != nullptr);

    // The derived table: 3/3/2/0, χ=2 — the far cap (its pole
    // vertex + S, A; the seam + the two rim halves) + the disk.
    check_body_invariants(*R, 3, 3, 2, 0, 2);
    check_exact_provenance(*R);
    check_typed_pcurves(*R);

    // The families: one trimmed sphere cap, one radical-plane disk.
    REQUIRE(count_family(*R, "sphere") == 1);
    REQUIRE(count_family(*R, "plane") == 1);

    // The classical volume: the far cap of height R + h = 1.75.
    const double want = cap_volume(1.0, 1.75);
    const auto mp = b::mass_properties(*R, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    // The disk's plane is the radical plane: x = h = 0.75.
    for (const auto& f : R->solid.shell().faces()) {
        if (std::string(f.surface()->type_name()) != "plane") continue;
        const auto* pl = dynamic_cast<const b::PlaneSurface*>(f.surface());
        REQUIRE(pl != nullptr);
        CHECK(close(pl->origin().x, 0.75f, 1e-4f));
        CHECK(std::abs(pl->normal_vec().x) > 0.999f);
    }

    auto R2 = b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    CHECK(body_digest(*R) == body_digest(*R2));
}

TEST_CASE("g2-u2: the lens (intersect) — the two near caps") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(0.8f);
    b::translate_body(*B, Vec3f{1.2f, 0, 0});

    auto L = b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(L != nullptr);

    // The derived table: 4/4/2/0, χ=2 (the lens is a ball).
    check_body_invariants(*L, 4, 4, 2, 0, 2);
    check_exact_provenance(*L);
    check_typed_pcurves(*L);
    REQUIRE(count_family(*L, "sphere") == 2);

    // The classical lens volume: the two near caps.
    const double want = cap_volume(1.0, 0.25) + cap_volume(0.8, 0.35);
    const auto mp = b::mass_properties(*L, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    // The operand-order independence (the canonical identity).
    auto L2 = b::exact_intersect(*B, *A, Vec3f{0, 0, 0}, 1e-6f);
    CHECK(body_digest(*L) == body_digest(*L2));
}

// ---------------------------------------------------------------------------
// THE SPHERE-PAIR DEGENERATES
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the co-radial dedup (coincident spheres)") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(1.0f);

    // union: ONE surface kept, the other absorbed — the operand
    // verbatim (the co-radial dedup, the coplanar rule's analog).
    auto U = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    check_body_invariants(*U, 2, 1, 1, 0, 2);
    CHECK(body_digest(*U) == body_digest(*A));

    // intersect: the same surface (A ∩ A = A).
    auto I = b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    check_body_invariants(*I, 2, 1, 1, 0, 2);
    CHECK(body_digest(*I) == body_digest(*A));

    // subtract: the empty — refused, the swallow disposition.
    const std::string msg = refusal_of([&] {
        b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("co-radial", msg));
    CHECK(refuses_with("dedup", msg));
}

TEST_CASE("g2-u2: the tangencies — the point contact's pinned dispositions") {
    // EXTERNAL (D = R₁ + R₂): the union/intersect would be
    // non-manifold at the contact point — REFUSED; the subtract's
    // touch removes nothing (measure zero) — the EXACT COPY (the G1
    // tie-break, the deterministic disposition).
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.8f);
        b::translate_body(*B, Vec3f{1.8f, 0, 0});
        for (const auto& act :
             {std::string("union"), std::string("intersect")}) {
            const std::string msg = refusal_of([&] {
                if (act == "union")
                    b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
                else
                    b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
            });
            CHECK(refuses_with("external tangency", msg));
            CHECK(refuses_with("Remedy", msg));
        }
        auto S = b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        REQUIRE(S != nullptr);
        CHECK(body_digest(*S) == body_digest(*A));
    }
    // INTERNAL (D = |R₁ − R₂|, B inside A): the union/intersect read
    // the nested disposition (the container / the contained verbatim
    // — the touch adds or removes nothing); the subtract's cavity
    // would pinch at the contact point — REFUSED.
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.8f);
        b::translate_body(*B, Vec3f{0.2f, 0, 0});
        auto U = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*U) == body_digest(*A));
        auto I = b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*I) == body_digest(*B));
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("internal tangency", msg));
        CHECK(refuses_with("Remedy", msg));
    }
}

TEST_CASE("g2-u2: the nested and separate dispositions") {
    // NESTED: B strictly inside A.
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.3f);
        b::translate_body(*B, Vec3f{0.2f, 0, 0});

        auto U = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*U) == body_digest(*A));  // the operand

        auto I = b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*I) == body_digest(*B));  // the smaller

        auto S = b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*S) == body_digest(*A));  // the miss
    }
    // SEPARATE: D > R₁ + R₂.
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.8f);
        b::translate_body(*B, Vec3f{5.0f, 0, 0});

        // subtract: the miss — the exact copy (the G1 tie-break).
        auto S = b::exact_subtract(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*S) == body_digest(*A));

        // union: the disconnected union — refused.
        const std::string mu = refusal_of([&] {
            b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("disconnected", mu));

        // intersect: the empty — refused (the swallow disposition).
        const std::string mi = refusal_of([&] {
            b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("empty", mi));
    }
    // CONCENTRIC, unequal radii: the nested class on the center line.
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.5f);
        auto U = b::exact_union(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        CHECK(body_digest(*U) == body_digest(*A));
    }
}

// ---------------------------------------------------------------------------
// THE SPHERE × CIRCLE-PRISM (the coaxial family)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the bead — sphere minus the coaxial circle prism") {
    const auto A = b::make_sphere(1.0f);
    auto tool = circle_prism(0.4f, -2.0f, 2.0f);

    auto R = b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(R != nullptr);

    // The derived table: 4/6/2/0, χ=0, genus 1 (the through-hole).
    check_body_invariants(*R, 4, 6, 2, 0, 0);
    check_exact_provenance(*R);
    check_typed_pcurves(*R);

    // THE DISCRIMINATOR (the order's own): the bead has ONE
    // CylinderZSurface wall face — the faceted extraction would answer
    // planar strips — beside the trimmed spherical ZONE.
    REQUIRE(count_family(*R, "cylinder") == 1);
    REQUIRE(count_family(*R, "sphere") == 1);
    const auto* wall = dynamic_cast<const b::CylinderZSurface*>(
        R->solid.shell().faces()[1].surface());
    REQUIRE(wall != nullptr);
    CHECK(close(wall->radius(), 0.4f, 1e-5f));
    CHECK(wall->u_sign() < 0.0f);  // the cavity side (the G1 convention)

    // The zone: the equatorial band between the latitudes at
    // z = ±√(R²−r²) — the re-framed sphere (pole ±Z, the azimuth the
    // unit's XOR convention derives).
    const auto* zone = dynamic_cast<const b::SphereSurface*>(
        R->solid.shell().faces()[0].surface());
    REQUIRE(zone != nullptr);
    CHECK(std::abs(std::abs(zone->pole().z) - 1.0f) < 1e-5f);

    // The latitude split points: on the circles at z = ±w, split at
    // the ±X meridians.
    const double w = std::sqrt(1.0 - 0.16);
    std::size_t on_circle = 0;
    for (const auto& v : R->vertices) {
        const double r2 = static_cast<double>(v.position().x) *
                              v.position().x +
                          static_cast<double>(v.position().y) *
                              v.position().y;
        CHECK(close(static_cast<float>(r2), 0.16f, 1e-4f));
        CHECK(close(std::abs(v.position().z), static_cast<float>(w), 1e-4f));
        ++on_circle;
    }
    CHECK(on_circle == 4);

    // The classical volume: the sphere minus the core (the cylinder
    // πr²·2w plus the two caps of height R−w).
    const double core = kPid * 0.16 * 2.0 * w +
                        2.0 * cap_volume(1.0, 1.0 - w);
    const double want = (4.0 / 3.0) * kPid - core;
    const auto mp = b::mass_properties(*R, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    auto R2 = b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    CHECK(body_digest(*R) == body_digest(*R2));
}

TEST_CASE("g2-u2: the bead's degenerates — blind, r ≥ R, the Viviani wall") {
    const auto A = b::make_sphere(1.0f);

    // The BLIND tool: a cap strictly inside — the G1 disposition.
    {
        auto tool = circle_prism(0.4f, -0.5f, 2.5f);
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("blind", msg));
        CHECK(refuses_with("Remedy", msg));
    }
    // r > R: the tool swallows the sphere — the empty.
    {
        auto tool = circle_prism(1.2f, -2.0f, 2.0f);
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("empty", msg));
    }
    // r = R: the tangent circle — the equator degenerates the zone.
    {
        auto tool = circle_prism(1.0f, -2.0f, 2.0f);
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("tangent", msg));
    }
    // NON-COAXIAL: the Viviani quartic — the wall, named.
    {
        auto tool = circle_prism(0.4f, -2.0f, 2.0f, {0.3f, 0.0f});
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("Viviani", msg));
        CHECK(refuses_with("Remedy", msg));
    }
    // The tool not covering the sphere's span: the blind family.
    {
        auto tool = circle_prism(0.4f, -1.5f, 0.5f);
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("blind", msg));
    }
}

TEST_CASE("g2-u2: the rivet — sphere union the circle prism") {
    const auto A = b::make_sphere(1.0f);
    auto tool = circle_prism(0.4f, -2.0f, 2.0f);

    auto U = b::exact_union(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(U != nullptr);

    // The derived table: 8/11/5/0, χ=2 (the pill).
    check_body_invariants(*U, 8, 11, 5, 0, 2);
    check_exact_provenance(*U);
    check_typed_pcurves(*U);

    // The families: the zone (sphere), two wall bands (cylinder), two
    // end disks (plane).
    REQUIRE(count_family(*U, "sphere") == 1);
    REQUIRE(count_family(*U, "cylinder") == 2);
    REQUIRE(count_family(*U, "plane") == 2);

    // The classical volume: sphere + cylinder − the core.
    const double w = std::sqrt(1.0 - 0.16);
    const double core = kPid * 0.16 * 2.0 * w +
                        2.0 * cap_volume(1.0, 1.0 - w);
    const double want = (4.0 / 3.0) * kPid + kPid * 0.16 * 4.0 - core;
    const auto mp = b::mass_properties(*U, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));
}

TEST_CASE("g2-u2: the core — sphere intersect the circle prism") {
    const auto A = b::make_sphere(1.0f);
    auto tool = circle_prism(0.4f, -2.0f, 2.0f);

    auto I = b::exact_intersect(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(I != nullptr);

    // The derived table: 6/7/3/0, χ=2 (the barrel — a ball).
    check_body_invariants(*I, 6, 7, 3, 0, 2);
    check_exact_provenance(*I);
    check_typed_pcurves(*I);

    // The families: the two polar caps (sphere) + the middle band.
    REQUIRE(count_family(*I, "sphere") == 2);
    REQUIRE(count_family(*I, "cylinder") == 1);

    // The classical volume: the core itself.
    const double w = std::sqrt(1.0 - 0.16);
    const double want = kPid * 0.16 * 2.0 * w + 2.0 * cap_volume(1.0, 1.0 - w);
    const auto mp = b::mass_properties(*I, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    // The core's caps carry the poles: |z| = R on two vertices.
    std::size_t poles = 0;
    for (const auto& v : I->vertices)
        if (close(std::abs(v.position().z), 1.0f, 1e-4f) &&
            close(v.position().x, 0.0f, 1e-4f) &&
            close(v.position().y, 0.0f, 1e-4f))
            ++poles;
    CHECK(poles == 2);
}

// ---------------------------------------------------------------------------
// THE SPHERE × BOX-PROFILE (the truncation family)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the truncation — sphere minus the box above") {
    const auto A = b::make_sphere(1.0f);
    auto tool = b::make_box(Vec3f{1.5f, 1.5f, 1.5f});  // the D6 prism class

    // The box above: its bottom wall cuts at z = 0.7, the other walls
    // clear (1.5 > 1), the caps clear — ONE axis family (Z).
    auto R = b::exact_subtract(*A, *tool, Vec3f{0, 0, 2.2f}, 1e-6f);
    REQUIRE(R != nullptr);

    // The derived table: 3/3/2/0, χ=2 — the far cap + the cut disk.
    check_body_invariants(*R, 3, 3, 2, 0, 2);
    check_exact_provenance(*R);
    check_typed_pcurves(*R);
    REQUIRE(count_family(*R, "sphere") == 1);
    REQUIRE(count_family(*R, "plane") == 1);

    // The classical volume: the far cap of height R + d₀ = 1.7.
    const auto mp = b::mass_properties(*R, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume),
                static_cast<float>(cap_volume(1.0, 1.7)), 1e-4f));

    // The near piece: the intersect — the cap of height R − d₀ = 0.3.
    auto I = b::exact_intersect(*A, *tool, Vec3f{0, 0, 2.2f}, 1e-6f);
    check_body_invariants(*I, 3, 3, 2, 0, 2);
    const auto mp2 = b::mass_properties(*I, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(close(static_cast<float>(mp2.volume),
                static_cast<float>(cap_volume(1.0, 0.3)), 1e-4f));

    // The tangent limit: the bottom wall at z = R removes nothing —
    // the deterministic tie-break (the exact copy, the G1 rule).
    auto T = b::exact_subtract(*A, *tool, Vec3f{0, 0, 2.5f}, 1e-6f);
    REQUIRE(T != nullptr);
    CHECK(body_digest(*T) == body_digest(*A));

    // The box BELOW, missing entirely: the miss.
    auto M = b::exact_subtract(*A, *tool, Vec3f{0, 0, -4.0f}, 1e-6f);
    CHECK(body_digest(*M) == body_digest(*A));
}

TEST_CASE("g2-u2: the dome union — sphere union the box (the pierced tool face)") {
    const auto A = b::make_sphere(1.0f);
    auto tool = b::make_box(Vec3f{1.5f, 1.5f, 1.5f});

    auto U = b::exact_union(*A, *tool, Vec3f{0, 0, 2.2f}, 1e-6f);
    REQUIRE(U != nullptr);

    // The derived table: 11/15/7/1, χ=2 — the box's faces + the far
    // cap; the box's bottom face GAINS the circle as an inner loop.
    check_body_invariants(*U, 11, 15, 7, 1, 2);
    check_exact_provenance(*U);
    check_typed_pcurves(*U);
    REQUIRE(count_family(*U, "sphere") == 1);
    REQUIRE(count_family(*U, "plane") == 6);

    // The pierced tool face: the box's face 0 (the −Z cap) with TWO
    // loops — the outer rectangle verbatim + the latitude inner loop.
    const auto& bottom = U->solid.shell().faces()[1];
    REQUIRE(bottom.loops().size() == 2);
    CHECK(bottom.loops()[0].is_outer());
    CHECK(!bottom.loops()[1].is_outer());

    // The classical volume: box + sphere − the near cap.
    const double want = 27.0 + (4.0 / 3.0) * kPid - cap_volume(1.0, 0.3);
    const auto mp = b::mass_properties(*U, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));
}

TEST_CASE("g2-u2: the slab — sphere minus the straddling box") {
    const auto A = b::make_sphere(1.0f);
    // The straddling box: the x walls cut (±0.6 < R), the y walls and
    // the caps clear — ONE family (X), the slab semantics.
    auto tool = b::make_box(Vec3f{0.6f, 1.5f, 2.5f});

    auto R = b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(R != nullptr);

    // The derived table: 6/6/4/0, χ=4 — two far caps + two cut
    // disks: the straddling band SPLITS the sphere into two separate
    // cap solids (S = 2 shells, χ = 2+2 — the honest answer, the
    // subtract's own; a union would have refused the disconnection).
    check_body_invariants(*R, 6, 6, 4, 0, 4);
    check_exact_provenance(*R);
    check_typed_pcurves(*R);
    REQUIRE(count_family(*R, "sphere") == 2);
    REQUIRE(count_family(*R, "plane") == 2);

    // The classical volume: the two caps of height R − a.
    const auto mp = b::mass_properties(*R, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume),
                static_cast<float>(2.0 * cap_volume(1.0, 0.4)), 1e-4f));
}

TEST_CASE("g2-u2: the two-family gate — the frame budget at entry") {
    const auto A = b::make_sphere(1.0f);
    // The box whose x AND y walls both cut the sphere: TWO axis
    // families — the canonical budget example, refused at the gate.
    auto tool = b::make_box(Vec3f{0.6f, 0.6f, 2.5f});
    const std::string msg = refusal_of([&] {
        b::exact_subtract(*A, *tool, Vec3f{0, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("frame budget", msg));
    CHECK(refuses_with("one axis family", msg));
    CHECK(refuses_with("Remedy", msg));
}

// ---------------------------------------------------------------------------
// THE TWO-HOLE GUIDE CASE (prism × prism, the second cut)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the two-hole body — the guide case, the second cut") {
    auto target = b::make_box(Vec3f{1, 1, 1});  // abc = 8
    auto tool1 = circle_prism(0.25f, -1.5f, 1.5f, {-0.4f, 0.0f});
    auto tool2 = circle_prism(0.25f, -1.5f, 1.5f, {0.4f, 0.0f});

    // The first cut: the pinned G1 table (12/17/7/2, χ=0).
    auto first = b::exact_subtract(*target, *tool1, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(first != nullptr);
    check_body_invariants(*first, 12, 17, 7, 2, 0);

    // THE SECOND CUT on the pierced target — the unit's own act.
    auto second = b::exact_subtract(*first, *tool2, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(second != nullptr);

    // THE DERIVED TABLE: 16/22/8/4, χ=−2, genus 2.
    check_body_invariants(*second, 16, 22, 8, 4, -2);
    const auto g = b::genus_from_chi(b::euler_characteristic(*second));
    REQUIRE(g.has_value());
    CHECK(*g == 2);
    check_exact_provenance(*second);
    check_typed_pcurves(*second);

    // THE DISCRIMINATORS (the order's own): TWO distinct CylinderZ
    // walls (the faceted extraction answers planar strips), the
    // pierced faces carry TWO inner loops each.
    REQUIRE(count_family(*second, "cylinder") == 2);
    const auto& faces = second->solid.shell().faces();
    std::size_t pierced2 = 0;
    for (const auto& f : faces) {
        if (std::string(f.surface()->type_name()) != "plane") continue;
        if (f.loops().size() == 3) ++pierced2;
    }
    CHECK(pierced2 == 2);

    // The two walls' radii and centers — the tools' own numbers.
    for (const auto& f : faces) {
        const auto* cz =
            dynamic_cast<const b::CylinderZSurface*>(f.surface());
        if (cz == nullptr) continue;
        CHECK(close(cz->radius(), 0.25f, 1e-5f));
        const bool x1 = close(cz->axis_center().x, -0.4f, 1e-5f);
        const bool x2 = close(cz->axis_center().x, 0.4f, 1e-5f);
        CHECK((x1 || x2));
        CHECK(close(cz->axis_center().y, 0.0f, 1e-5f));
    }

    // The classical volume: abc − 2πr²c.
    const double want = 8.0 - 2.0 * kPid * 0.0625 * 2.0;
    const auto mp = b::mass_properties(*second, CAD_0::math::Tolerance(CAD_0::math::ToleranceKind::Linear, 1e-6));
    CHECK(mp.all_exact);
    CHECK(close(static_cast<float>(mp.volume), static_cast<float>(want),
                1e-4f));

    // Determinism: the whole chain, bit-identical.
    auto f1b = b::exact_subtract(*target, *tool1, Vec3f{0, 0, 0}, 1e-6f);
    auto f2b = b::exact_subtract(*f1b, *tool2, Vec3f{0, 0, 0}, 1e-6f);
    CHECK(body_digest(*second) == body_digest(*f2b));
}

TEST_CASE("g2-u2: the non-annular refusals and the nested miss") {
    auto target = b::make_box(Vec3f{1, 1, 1});
    auto tool1 = circle_prism(0.25f, -1.5f, 1.5f, {-0.4f, 0.0f});
    auto first = b::exact_subtract(*target, *tool1, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(first != nullptr);

    // TANGENT circles on the pierced plane: d = r₁ + r₂ — the pinch.
    {
        auto t = circle_prism(0.25f, -1.5f, 1.5f, {0.1f, 0.0f});
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*first, *t, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("tangent", msg));
        CHECK(refuses_with("Remedy", msg));
    }
    // CROSSING, EQUAL radii: the non-annular class, the mid-plane
    // entry named (the pair that WOULD be consumed).
    {
        auto t = circle_prism(0.25f, -1.5f, 1.5f, {0.0f, 0.0f});
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*first, *t, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("non-annular", msg));
        CHECK(refuses_with("mid-plane", msg));
    }
    // CROSSING, UNEQUAL radii: the offset-plane disposition NAMED —
    // typable, awaiting its consuming unit, never "untypable".
    {
        auto t = circle_prism(0.35f, -1.5f, 1.5f, {-0.1f, 0.0f});
        const std::string msg = refusal_of([&] {
            b::exact_subtract(*first, *t, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("non-annular", msg));
        CHECK(refuses_with("offset plane", msg));
        CHECK(refuses_with("two typed lines", msg));
        CHECK(msg.find("untypable") == std::string::npos);
    }
    // NESTED circles: the new hole inside the old — the tool removes
    // already-removed material: the MISS, the exact copy.
    {
        auto t = circle_prism(0.1f, -1.5f, 1.5f, {-0.4f, 0.0f});
        auto R = b::exact_subtract(*first, *t, Vec3f{0, 0, 0}, 1e-6f);
        REQUIRE(R != nullptr);
        CHECK(body_digest(*R) == body_digest(*first));
    }
}

// ---------------------------------------------------------------------------
// THE FRAME BUDGET APPLIED (the trimmed survivor's one family)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the budget wall on the trimmed survivor") {
    // The bead: its zone face is budgeted to the Z family (the stored
    // pole). A later operation asking the X family finds the wall.
    const auto A = b::make_sphere(1.0f);
    auto cyl = circle_prism(0.4f, -2.0f, 2.0f);
    auto bead = b::exact_subtract(*A, *cyl, Vec3f{0, 0, 0}, 1e-6f);
    REQUIRE(bead != nullptr);

    // The X-family box: its walls cut the zone under a NEW family.
    auto xbox = b::make_box(Vec3f{0.6f, 1.5f, 2.5f});
    const std::string msg = refusal_of([&] {
        b::exact_subtract(*bead, *xbox, Vec3f{0, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("frame budget", msg));
    CHECK(refuses_with("Remedy", msg));

    // The family-COMPATIBLE second cut (a coaxial Z cylinder of a
    // bigger radius): refused naming the multi-cut cascade — the
    // honest scope, the remedy real (re-cut the original operand).
    auto big = circle_prism(0.6f, -2.0f, 2.0f);
    const std::string msg2 = refusal_of([&] {
        b::exact_subtract(*bead, *big, Vec3f{0, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("multi-cut", msg2));
    CHECK(refuses_with("Remedy", msg2));
}

// ---------------------------------------------------------------------------
// THE PROVENANCE GATE
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the provenance gate — an approximated operand refuses") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(0.8f);

    // The record authored the way the provenance tests authored theirs
    // (the record is the thing under test, never laundered).
    auto bad = std::make_unique<BRepBody>(*B);
    b::translate_body(*bad, Vec3f{1.2f, 0, 0});
    bad->solid.shell_mut().faces_mut()[0].set_provenance(
        b::ProvenanceInfo::approximated(0.01));

    const std::string msg = refusal_of([&] {
        b::exact_union(*A, *bad, Vec3f{0, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("approximated", msg));
    CHECK(refuses_with("Remedy", msg));
}

// ---------------------------------------------------------------------------
// THE CLASS BOUNDARY (the mixtures and the landed opinions)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: the class boundary — the mixtures refuse honestly") {
    // A prism pair at the UNION: Phase D's landed opinion — pointed
    // at, never duplicated.
    const auto box1 = b::make_box(Vec3f{1, 1, 1});
    const auto box2 = b::make_box(Vec3f{1, 1, 1});
    const std::string msg = refusal_of([&] {
        b::exact_union(*box1, *box2, Vec3f{1.0f, 0, 0}, 1e-6f);
    });
    CHECK(refuses_with("Phase D", msg));
    CHECK(refuses_with("bool_union", msg));

    // A two-sphere-face target (the lens): the sphere class takes the
    // single-sphere body.
    {
        const auto A = b::make_sphere(1.0f);
        const auto B = b::make_sphere(0.8f);
        b::translate_body(*B, Vec3f{1.2f, 0, 0});
        auto lens = b::exact_intersect(*A, *B, Vec3f{0, 0, 0}, 1e-6f);
        REQUIRE(lens != nullptr);
        auto cyl = circle_prism(0.4f, -2.0f, 2.0f);
        const std::string m2 = refusal_of([&] {
            b::exact_subtract(*lens, *cyl, Vec3f{0, 0, 0}, 1e-6f);
        });
        CHECK(refuses_with("sphere", m2));
    }
}

// ---------------------------------------------------------------------------
// THE FULL-FUNCTION REGRESSION (every op, one digest sweep)
// ---------------------------------------------------------------------------

TEST_CASE("g2-u2: every op is a pure function of its operands") {
    const auto A = b::make_sphere(1.0f);
    const auto B = b::make_sphere(0.8f);
    b::translate_body(*B, Vec3f{1.2f, 0, 0});
    auto tool = circle_prism(0.4f, -2.0f, 2.0f);
    auto box = b::make_box(Vec3f{1.5f, 1.5f, 1.5f});

    struct Row {
        const char* name;
        std::unique_ptr<BRepBody> (*fn)(const BRepBody&, const BRepBody&,
                                        Vec3f, float);
        const BRepBody* a;
        const BRepBody* b;
        Vec3f off;
    };
    const Row rows[] = {
        {"union-sph", b::exact_union, A.get(), B.get(), Vec3f{0, 0, 0}},
        {"isect-sph", b::exact_intersect, A.get(), B.get(), Vec3f{0, 0, 0}},
        {"sub-sph", b::exact_subtract, A.get(), B.get(), Vec3f{0, 0, 0}},
        {"union-cyl", b::exact_union, A.get(), tool.get(), Vec3f{0, 0, 0}},
        {"isect-cyl", b::exact_intersect, A.get(), tool.get(),
         Vec3f{0, 0, 0}},
        {"sub-cyl", b::exact_subtract, A.get(), tool.get(), Vec3f{0, 0, 0}},
        {"sub-box", b::exact_subtract, A.get(), box.get(),
         Vec3f{0, 0, 2.2f}},
        {"union-box", b::exact_union, A.get(), box.get(), Vec3f{0, 0, 2.2f}},
    };
    for (const auto& row : rows) {
        auto r1 = row.fn(*row.a, *row.b, row.off, 1e-6f);
        auto r2 = row.fn(*row.a, *row.b, row.off, 1e-6f);
        REQUIRE(r1 != nullptr);
        INFO(row.name);
        CHECK(body_digest(*r1) == body_digest(*r2));
    }
}
