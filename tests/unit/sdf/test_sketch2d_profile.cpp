// tests/unit/sdf/test_sketch2d_profile.cpp
//
// ADR-0030 D3, MVP unit 1 — the composite: ProfileSDF (max-composition,
// bounds, winning-branch gradient) and SketchEntitySet (the complete
// construction certificate: samples + pairwise, the two instruments of
// Revision 2 — together sufficient, either alone not).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "CAD_0/sdf/sketch_profile.hpp"
#include "property/property_prng.hpp"

using namespace CAD_0::sdf;
using namespace CAD_0::math;
using CAD_0::test::property::Prng;

namespace {

constexpr float kPi = 3.14159265358979323846f;

// A CCW axis-aligned rectangle chain: interior on the left of every
// traversal (the natural authoring orientation).
SketchChain rect_chain(float x0, float y0, float x1, float y1) {
    // CCW: interior on the left of every traversal. Built as an
    // initializer list (the GCC 14 stringop-overflow false positive on
    // the emplace_back path of this aggregate is documented in the
    // toolchain contract's caveat 1 family).
    return SketchChain{{
        sketch_segment({x0, y0}, {x1, y0}, true), // bottom, E
        sketch_segment({x1, y0}, {x1, y1}, true), // right, N
        sketch_segment({x1, y1}, {x0, y1}, true), // top, W
        sketch_segment({x0, y1}, {x0, y0}, true), // left, S
    }};
}

// Independent oracles (test-side, built from the recipe data — never
// through the code under test).

float oracle_entity_value(const SketchEntity& e, Vec2f p) {
    switch (e.kind) {
    case SketchEntity::Kind::Segment:
        return Halfplane2D(e.segment.a, e.segment.b, e.segment.interior_left)
            .sample(p).value;
    case SketchEntity::Kind::Circle:
        return Disc2D(e.circle.center, e.circle.radius, e.circle.hole)
            .sample(p).value;
    case SketchEntity::Kind::Arc:
        return Disc2D(e.arc.center, e.arc.radius, e.arc.hole).sample(p).value;
    }
    return std::numeric_limits<float>::quiet_NaN();
}

float oracle_composed(const std::vector<SketchChain>& chains, Vec2f p) {
    float best = -std::numeric_limits<float>::infinity();
    for (const auto& c : chains)
        for (const auto& e : c.entities)
            best = std::max(best, oracle_entity_value(e, p));
    return best;
}

float point_segment_distance(Vec2f p, Vec2f a, Vec2f b) {
    const Vec2f d = b - a;
    const float dd = d.dot(d);
    const float t = dd > 0.0f ? std::clamp((p - a).dot(d) / dd, 0.0f, 1.0f) : 0.0f;
    return (p - (a + d * t)).length();
}

// Distance to the entity's CURVE (the drawn contour), span-restricted
// for arcs — the theorem's d(P, S).
float entity_curve_distance(const SketchEntity& e, Vec2f p) {
    switch (e.kind) {
    case SketchEntity::Kind::Segment:
        return point_segment_distance(p, e.segment.a, e.segment.b);
    case SketchEntity::Kind::Circle:
        return std::abs((p - e.circle.center).length() - e.circle.radius);
    case SketchEntity::Kind::Arc: {
        const float radial = std::abs((p - e.arc.center).length() - e.arc.radius);
        const float ang = std::atan2(p.y - e.arc.center.y, p.x - e.arc.center.x);
        float rel = std::fmod(ang - e.arc.start_angle, 2.0f * kPi);
        if (rel < 0.0f) rel += 2.0f * kPi;
        if (rel <= e.arc.sweep) return radial;
        return std::min((p - arc_start_point(e.arc)).length(),
                        (p - arc_end_point(e.arc)).length());
    }
    }
    return std::numeric_limits<float>::infinity();
}

float contour_distance(const std::vector<SketchChain>& chains, Vec2f p) {
    float best = std::numeric_limits<float>::infinity();
    for (const auto& c : chains)
        for (const auto& e : c.entities)
            best = std::min(best, entity_curve_distance(e, p));
    return best;
}

// The certificate's own checkpoints (the exact points the samples
// instrument evaluates): every entity's interior checkpoint plus every
// joint position of every multi-entity chain.
std::vector<Vec2f> certificate_checkpoints(const std::vector<SketchChain>& chains) {
    std::vector<Vec2f> pts;
    for (const auto& c : chains) {
        for (const auto& e : c.entities) {
            switch (e.kind) {
            case SketchEntity::Kind::Segment:
                pts.push_back((e.segment.a + e.segment.b) * 0.5f);
                break;
            case SketchEntity::Kind::Arc: {
                const float mid = e.arc.start_angle + e.arc.sweep * 0.5f;
                pts.push_back(e.arc.center + Vec2f{e.arc.radius * std::cos(mid),
                                                  e.arc.radius * std::sin(mid)});
                break;
            }
            case SketchEntity::Kind::Circle:
                pts.push_back(e.circle.center + Vec2f{e.circle.radius, 0.0f});
                break;
            }
        }
        if (c.entities.size() > 1) {
            for (std::size_t i = 0; i < c.entities.size(); ++i) {
                const auto& ei = c.entities[i];
                if (ei.kind == SketchEntity::Kind::Segment)
                    pts.push_back(ei.segment.b);
                else
                    pts.push_back(arc_end_point(ei.arc));
            }
        }
    }
    return pts;
}

// Every "(x, y)" point the message names — for the refusal contracts
// that must name the crossing point.
std::vector<Vec2f> parse_message_points(const std::string& msg) {
    std::vector<Vec2f> out;
    for (std::size_t i = 0; i < msg.size(); ++i) {
        if (msg[i] != '(') continue;
        const std::size_t close = msg.find(')', i);
        if (close == std::string::npos) break;
        float x = 0.0f, y = 0.0f;
        Vec2f v;
        if (std::sscanf(msg.c_str() + i, "(%f, %f)", &x, &y) == 2) {
            v = {x, y};
            out.push_back(v);
        }
        i = close;
    }
    return out;
}

} // namespace

// --- The composed field ---------------------------------------------------

TEST_CASE("ProfileSDF: square — the max-composition, pinned") {
    const std::vector<SketchChain> square{rect_chain(0.0f, 0.0f, 1.0f, 1.0f)};
    const ProfileSDF prof(square);

    CHECK(prof.entity_count() == 4);
    CHECK(prof.lipschitz() == 1.0f);

    // Interior: the incenter — every half-plane contributes -0.5.
    CHECK(prof.sample({0.5f, 0.5f}).value == doctest::Approx(-0.5f));
    // Mid-segment: the exact regime (f IS the distance there).
    CHECK(prof.sample({2.0f, 0.5f}).value == doctest::Approx(1.0f));
    CHECK(prof.sample({0.5f, -3.0f}).value == doctest::Approx(3.0f));
    // A joint: on the boundary.
    CHECK(prof.sample({1.0f, 1.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    // Winning-branch gradient: right edge wins at (2, 0.5).
    auto g = prof.sample({2.0f, 0.5f}).gradient;
    CHECK(g.x == doctest::Approx(1.0f));
    CHECK(g.y == doctest::Approx(0.0f));
    // Top edge wins at (0.5, 2).
    g = prof.sample({0.5f, 2.0f}).gradient;
    CHECK(g.x == doctest::Approx(0.0f));
    CHECK(g.y == doctest::Approx(1.0f));

    // Bounds: the intersection of the axis-aligned half-boxes = the square.
    const auto& b = prof.bounds();
    CHECK(b.min.x == doctest::Approx(0.0f));
    CHECK(b.min.y == doctest::Approx(0.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.max.y == doctest::Approx(1.0f));
}

TEST_CASE("ProfileSDF: the corner gap, pinned once (lie #2, live)") {
    // Right angle at (1,1), external bisector, perpendicular distance t
    // from both lines: f = t (the understated side), true d = t*sqrt(2)
    // — the 1/sin(theta/2) gap with theta = pi/2.
    const std::vector<SketchChain> square{rect_chain(0.0f, 0.0f, 1.0f, 1.0f)};
    const ProfileSDF prof(square);
    const float t = 0.5f;
    // Perpendicular distance t from both lines: the bisector point
    // (1+t, 1+t) sits t away from x=1 and from y=1.
    const Vec2f p{1.0f + t, 1.0f + t};
    CHECK(prof.sample(p).value == doctest::Approx(t).epsilon(1e-5f));
    // The true distance to the corner (the nearest boundary point is
    // the corner itself, distance t*sqrt(2) along the bisector).
    const float d_true = (p - Vec2f{1.0f, 1.0f}).length();
    CHECK(d_true == doctest::Approx(t * std::sqrt(2.0f)).epsilon(1e-5f));
    // The theorem holds: |f| <= d.
    CHECK(std::abs(prof.sample(p).value) <= d_true + 1e-5f);
}

TEST_CASE("ProfileSDF: rotated square — the orientation-blind diagonal "
           "falls back to the contour hull (declared closure)") {
    // CCW diamond: all four normals are diagonal, so every region box is
    // the universe — the ADR-literal intersection would refuse this
    // valid closed contour. The exact positive-spanning condition
    // certifies boundedness; the bounds fall back to the contour hull.
    SketchChain diamond{
        {sketch_segment({0.0f, 1.0f}, {-1.0f, 0.0f}, true),
         sketch_segment({-1.0f, 0.0f}, {0.0f, -1.0f}, true),
         sketch_segment({0.0f, -1.0f}, {1.0f, 0.0f}, true),
         sketch_segment({1.0f, 0.0f}, {0.0f, 1.0f}, true)}};
    const ProfileSDF prof({diamond});

    const auto& b = prof.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.min.y == doctest::Approx(-1.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.max.y == doctest::Approx(1.0f));
    // Incenter: every half-plane contributes -1/sqrt(2).
    CHECK(prof.sample({0.0f, 0.0f}).value ==
          doctest::Approx(-1.0f / std::sqrt(2.0f)).epsilon(1e-5f));
    // A vertex: on the boundary.
    CHECK(prof.sample({1.0f, 0.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
}

TEST_CASE("ProfileSDF: ring — the annulus field, exact on a stratified grid") {
    const std::vector<SketchChain> ring{
        SketchChain{{sketch_circle({0.0f, 0.0f}, 1.0f, /*hole=*/false)}},
        SketchChain{{sketch_circle({0.0f, 0.0f}, 0.4f, /*hole=*/true)}}};
    const ProfileSDF prof(ring);

    // Stratified sign check: bands x slices (the 0029 habit).
    // Inside the annulus: f < 0; in the hole or outside: f > 0.
    int inside_checked = 0, outside_checked = 0;
    for (int band = 0; band < 8; ++band) {
        const float rho = 0.05f + static_cast<float>(band) * 0.15f; // 0.05 .. 1.10
        for (int slice = 0; slice < 12; ++slice) {
            const float phi = static_cast<float>(slice) * (kPi / 6.0f);
            const Vec2f p{rho * std::cos(phi), rho * std::sin(phi)};
            const float f = prof.sample(p).value;
            if (rho > 0.4f && rho < 1.0f) {
                ++inside_checked;
                CHECK(f < 0.0f);
                // Exact: the annulus field is max(rho-1, 0.4-rho).
                const float exact = std::max(rho - 1.0f, 0.4f - rho);
                CHECK(f == doctest::Approx(exact).epsilon(1e-5f));
            } else {
                ++outside_checked;
                CHECK(f > 0.0f);
            }
        }
    }
    CHECK(inside_checked == 48);
    CHECK(outside_checked == 48);

    // Bounds: the outer disc's box (the hole contributes the universe).
    const auto& b = prof.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.max.y == doctest::Approx(1.0f));
    CHECK(b.bounded());
}

TEST_CASE("ProfileSDF: D-shape — segment + arc, the span box in the hull") {
    SketchChain d{
        {sketch_segment({-1.0f, 0.0f}, {1.0f, 0.0f}, true),
         sketch_arc({0.0f, 0.0f}, 1.0f, /*hole=*/false,
                    /*start_angle=*/0.0f, /*sweep=*/kPi)}};
    const ProfileSDF prof({d});

    // Field: apex on the zero-set, flat side exact, incenter tie.
    CHECK(prof.sample({0.0f, 1.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(prof.sample({0.0f, -0.5f}).value == doctest::Approx(0.5f));
    CHECK(prof.sample({0.0f, 0.5f}).value == doctest::Approx(-0.5f));
    CHECK(prof.sample({0.5f, 0.2f}).value ==
          doctest::Approx(std::max(-0.2f, std::sqrt(0.29f) - 1.0f)).epsilon(1e-5f));

    // Bounds: the arc's SPAN box (through (0,1)) meets the segment's
    // half-box {y > 0}: [−1,1] x [0,1].
    const auto& b = prof.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.min.y == doctest::Approx(0.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.max.y == doctest::Approx(1.0f));
}

// --- The certificate: conforming profiles construct ----------------------

TEST_CASE("Certificate: conforming profiles construct (square, ring, D)") {
    CHECK_NOTHROW(ProfileSDF({rect_chain(-2.0f, -1.0f, 3.0f, 2.0f)}));
    CHECK_NOTHROW(ProfileSDF({
        SketchChain{{sketch_circle({0.0f, 0.0f}, 1.0f, false)}},
        SketchChain{{sketch_circle({0.0f, 0.0f}, 0.4f, true)}}}));
    CHECK_NOTHROW(ProfileSDF({SketchChain{
        {sketch_segment({-1.0f, 0.0f}, {1.0f, 0.0f}, true),
         sketch_arc({0.0f, 0.0f}, 1.0f, false, 0.0f, kPi)}}}));
}

// --- The certificate: structural rejections -------------------------------

TEST_CASE("Certificate: an open chain is refused (the gap is named)") {
    std::vector<SketchChain> open{rect_chain(0.0f, 0.0f, 1.0f, 1.0f)};
    open[0].entities.pop_back(); // drop the left side
    try {
        ProfileSDF prof(open);
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("must share their joint") != std::string::npos);
        CHECK(msg.find("segment[chain 0 #2]") != std::string::npos); // top
        CHECK(msg.find("segment[chain 0 #0]") != std::string::npos); // bottom
    }
}

TEST_CASE("Certificate: a single non-circle entity is refused") {
    try {
        ProfileSDF prof({SketchChain{{sketch_segment({0.0f, 0.0f}, {1.0f, 1.0f}, true)}}});
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("single-entity chain must be a "
                                         "circle") != std::string::npos);
    }
}

TEST_CASE("Certificate: a circle inside a multi-entity chain is refused") {
    std::vector<SketchChain> bad{rect_chain(0.0f, 0.0f, 1.0f, 1.0f)};
    bad[0].entities[2] = sketch_circle({0.5f, 0.5f}, 0.1f, false);
    try {
        ProfileSDF prof(bad);
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find(
                  "cannot participate in a multi-entity chain") != std::string::npos);
    }
}

TEST_CASE("Certificate: a degenerate arc sweep is refused with the value") {
    for (const float bad_sweep : {0.0f, 2.0f * kPi, -0.5f,
                                  std::numeric_limits<float>::quiet_NaN()}) {
        std::vector<SketchChain> bad{SketchChain{
            {sketch_segment({-1.0f, 0.0f}, {1.0f, 0.0f}, true),
             sketch_arc({0.0f, 0.0f}, 1.0f, false, 0.0f, bad_sweep)}}};
        try {
            ProfileSDF prof(bad);
            (void)prof;
            FAIL("expected std::invalid_argument");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find("sweep") != std::string::npos);
            CHECK(std::string(e.what()).find("arc[chain 0 #1]") != std::string::npos);
        }
    }
}

TEST_CASE("Certificate: a flipped segment fails its own checkpoint "
           "(the samples instrument)") {
    // Square with the top segment's orientation flipped: the chain still
    // closes, the pairwise check sees no crossing — but the bottom
    // midpoint now evaluates OUTSIDE (the flipped half-plane excludes
    // it), and the sample check refuses, naming the entity.
    std::vector<SketchChain> bad{rect_chain(0.0f, 0.0f, 1.0f, 1.0f)};
    bad[0].entities[2] = sketch_segment({1.0f, 1.0f}, {0.0f, 1.0f}, false);
    try {
        ProfileSDF prof(bad);
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("must vanish") != std::string::npos);
        // The entity whose checkpoint caught the lie is named.
        bool names_entity = msg.find("segment[chain 0 #0]") != std::string::npos ||
                            msg.find("segment[chain 0 #1]") != std::string::npos ||
                            msg.find("segment[chain 0 #3]") != std::string::npos;
        CHECK(names_entity);
    }
}

TEST_CASE("Certificate: a notch deeper than the profile is wide is refused "
           "with the entity named (the loud error, not the silent sign flip)") {
    // A swallowing hole: the disc covers the square's midpoints — the
    // samples instrument catches it first (both instruments are loud).
    std::vector<SketchChain> bad{
        rect_chain(0.0f, 0.0f, 1.0f, 1.0f),
        SketchChain{{sketch_circle({0.5f, 0.5f}, 0.6f, /*hole=*/true)}}};
    try {
        ProfileSDF prof(bad);
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("must vanish") != std::string::npos);
        CHECK(msg.find("segment[chain 0 #0]") != std::string::npos); // bottom midpoint
    }
}

// --- Lie #5b: the review's catch, as an EXPLICIT regression ----------------

TEST_CASE("REGRESSION lie #5b: a hole circle crossing an outer segment "
           "mid-span is refused by the PAIRWISE check, both entities and "
           "the crossing point named") {
    // The counterexample's exact anatomy (ADR-0030 D3): the crossing
    // lands strictly between the segment's midpoint and its joint, so
    // EVERY sample passes and only the pairwise instrument can refuse.
    const std::vector<SketchChain> crossed{
        rect_chain(0.0f, 0.0f, 1.0f, 1.0f),
        SketchChain{{sketch_circle({0.7f, 0.15f}, 0.2f, /*hole=*/true)}}};

    // Anatomy, part 1 — verified independently: every checkpoint of the
    // certificate evaluates ON the composed zero-set (the samples
    // instrument alone cannot catch this shape).
    for (const Vec2f joint : {Vec2f{0, 0}, Vec2f{1, 0}, Vec2f{1, 1}, Vec2f{0, 1}})
        CHECK(std::abs(oracle_composed(crossed, joint)) <= kSampleTol);
    for (const Vec2f mid : {Vec2f{0.5f, 0.0f}, Vec2f{1.0f, 0.5f},
                            Vec2f{0.5f, 1.0f}, Vec2f{0.0f, 0.5f}})
        CHECK(std::abs(oracle_composed(crossed, mid)) <= kSampleTol);
    // The hole's own checkpoint (angle 0) is on its curve, inside.
    CHECK(std::abs(oracle_composed(crossed, Vec2f{0.9f, 0.15f})) <= kSampleTol);

    // Anatomy, part 2 — the expected crossings, computed independently:
    // (x-0.7)^2 + 0.15^2 = 0.2^2 on y = 0.
    const float dx = std::sqrt(0.2f * 0.2f - 0.15f * 0.15f);
    const Vec2f expected_a{0.7f - dx, 0.0f}; // the first root the certificate reports
    const Vec2f expected_b{0.7f + dx, 0.0f};
    REQUIRE((expected_a - Vec2f{0.5f, 0.0f}).length() > 1e-3f); // strictly past the midpoint
    REQUIRE((Vec2f{1.0f, 0.0f} - expected_b).length() > 1e-3f); // strictly before the joint

    try {
        ProfileSDF prof(crossed);
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        // The pairwise instrument (not the samples): the message says so.
        CHECK(msg.find("declared shared joints") != std::string::npos);
        // BOTH entities are named.
        CHECK(msg.find("segment[chain 0 #0]") != std::string::npos); // the bottom segment
        CHECK(msg.find("circle[chain 1 #0]") != std::string::npos);  // the hole
        // The crossing point is named. (One per pair is reported — the
        // certificate throws at the pair's FIRST illegal crossing; the
        // second root of this pair is not separately named.)
        const auto points = parse_message_points(msg);
        REQUIRE(!points.empty());
        bool found_a = false;
        for (const Vec2f& q : points)
            if ((q - expected_a).length() < 1e-3f) found_a = true;
        CHECK(found_a);
    }

    // Liveness of the same shape: the hole moved clear of the boundary
    // constructs — the refusal was about the crossing, not the hole.
    CHECK_NOTHROW(ProfileSDF({
        rect_chain(0.0f, 0.0f, 1.0f, 1.0f),
        SketchChain{{sketch_circle({0.5f, 0.5f}, 0.2f, true)}}}));
}

// --- The unbounded profile --------------------------------------------------

TEST_CASE("Certificate: an unbounded profile is refused — 'does not close'") {
    // A strip (two opposite half-planes, no outer disc) closed by two
    // complement arcs: structure closes, samples pass, pairwise passes
    // (the arcs' discs touch the segments only at the declared joints)
    // — and the region is the ADR's unbounded failure: refused.
    const float r = std::sqrt(1.25f);               // through (1,0) and (1,2)
    const float a_start = std::atan2(-1.0f, -0.5f); // angle of (1,0) from (1.5,1)
    const float sweep = std::atan2(1.0f, -0.5f) - a_start; // CCW to the angle of (1,2)
    REQUIRE(sweep > kPi); // the major arc, bulging right (outside the strip)
    REQUIRE(sweep < 2.0f * kPi);
    const float b_start = std::atan2(1.0f, 0.5f);   // angle of (-1,2) from (-1.5,1)

    SketchChain strip{
        {sketch_segment({-1.0f, 0.0f}, {1.0f, 0.0f}, true),
         sketch_arc({1.5f, 1.0f}, r, /*hole=*/true, a_start, sweep),
         sketch_segment({1.0f, 2.0f}, {-1.0f, 2.0f}, true),
         sketch_arc({-1.5f, 1.0f}, r, /*hole=*/true, b_start, sweep)}};
    try {
        ProfileSDF prof({strip});
        (void)prof;
        FAIL("expected std::invalid_argument");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("does not close") != std::string::npos);
    }
}

// --- Property families (seeded; failure regime IN range) --------------------

TEST_CASE("Property: |f| <= d(P, contour) — the theorem, live (seeded)") {
    // A 1-Lipschitz field vanishing on the contour cannot over-reach
    // its own zero-set: the composed profile is exactly such a field.
    Prng prng(0x5EED2D01u);
    int checked = 0;
    for (int member = 0; member < 12; ++member) {
        const float x0 = prng.frange(-2.0f, 0.0f), x1 = prng.frange(0.5f, 2.0f);
        const float y0 = prng.frange(-2.0f, 0.0f), y1 = prng.frange(0.5f, 2.0f);
        std::vector<SketchChain> chains{rect_chain(x0, y0, x1, y1)};
        if (prng.coin()) {
            const float rr = prng.frange(0.05f, 0.25f);
            chains.push_back(SketchChain{{sketch_circle(
                {prng.frange(x0 + rr + 0.05f, x1 - rr - 0.05f),
                 prng.frange(y0 + rr + 0.05f, y1 - rr - 0.05f)}, rr, true)}});
        }
        const ProfileSDF prof(chains);
        for (int pt = 0; pt < 64; ++pt) {
            const Vec2f p{prng.frange(x0 - 0.5f, x1 + 0.5f),
                          prng.frange(y0 - 0.5f, y1 + 0.5f)};
            const float f = prof.sample(p).value;
            const float d = contour_distance(chains, p);
            INFO("member ", member, " pt ", pt, " f=", f, " d=", d);
            CHECK(std::abs(f) <= d + 1e-5f);
            ++checked;
        }
    }
    CHECK(checked == 12 * 64);
}

TEST_CASE("Property: L = 1 — pairwise slopes never exceed the distance") {
    Prng prng(0x5EED2D02u);
    int checked = 0;
    for (int member = 0; member < 8; ++member) {
        const std::vector<SketchChain> chains{
            rect_chain(-1.5f, -0.8f, 1.2f, 1.0f)};
        const ProfileSDF prof(chains);
        for (int pt = 0; pt < 64; ++pt) {
            const Vec2f p{prng.frange(-2.0f, 2.0f), prng.frange(-1.8f, 2.0f)};
            const Vec2f q{prng.frange(-2.0f, 2.0f), prng.frange(-1.8f, 2.0f)};
            const float df = prof.sample(p).value - prof.sample(q).value;
            const float d = (p - q).length();
            if (d < 1e-3f) continue;
            INFO("member ", member, " pt ", pt, " |df|=", std::abs(df), " d=", d);
            CHECK(std::abs(df) <= d + 1e-5f);
            ++checked;
        }
    }
    CHECK(checked > 400); // liveness
}

TEST_CASE("Property: gradient vs FD, component-wise (seam scope documented)") {
    // Scope: the winning branch is smooth wherever it strictly wins and
    // the point is clear of every other entity's curve — points near
    // the max-seam (joints, span ends) are excluded by margin, counted,
    // and the liveness floor asserts the exclusions did not eat the
    // family (the box/intersect precedent of test_operators).
    Prng prng(0x5EED2D03u);
    const float h = 1e-3f;
    int compared = 0, skipped = 0;
    const std::vector<SketchChain> chains{rect_chain(-1.0f, -0.6f, 1.4f, 0.9f)};
    const ProfileSDF prof(chains);
    for (int pt = 0; pt < 256; ++pt) {
        const Vec2f p{prng.frange(-1.8f, 2.2f), prng.frange(-1.4f, 1.7f)};
        // Margin: the winning value vs the runner-up, and the clearance
        // to the nearest curve (the seam lives ON the curves).
        std::vector<float> vals;
        for (const auto& c : chains)
            for (const auto& e : c.entities)
                vals.push_back(oracle_entity_value(e, p));
        std::sort(vals.begin(), vals.end());
        const float margin = (vals.size() >= 2) ? (vals[vals.size() - 1] - vals[vals.size() - 2])
                                                : 1.0f;
        const float clearance = contour_distance(chains, p);
        if (margin < 1e-2f || clearance < 10.0f * h) {
            ++skipped;
            continue;
        }
        const auto s = prof.sample(p);
        const float fdx = (prof.sample({p.x + h, p.y}).value -
                           prof.sample({p.x - h, p.y}).value) / (2.0f * h);
        const float fdy = (prof.sample({p.x, p.y + h}).value -
                           prof.sample({p.x, p.y - h}).value) / (2.0f * h);
        // Budget: FD roundoff ~ eps*|f|/2h ~ 1e-4 at |f|~2, truncation
        // ~ h^2 ~ 1e-6; the failures this must catch (a wrong branch or
        // a missing sign) are O(1) — 2e-3 leaves two orders of margin.
        CHECK(std::abs(s.gradient.x - fdx) <= 2e-3f);
        CHECK(std::abs(s.gradient.y - fdy) <= 2e-3f);
        ++compared;
    }
    CHECK(compared > 64);  // liveness
    CHECK(skipped < 192);  // the exclusions did not eat the family
}

TEST_CASE("Property: the failure-regime family — crossings IN range, "
           "rejections with names, conforming liveness (revision 2's condition)") {
    // The house lesson (D3, exercised here): a property whose range
    // avoids the regime is vacuous. The generator's member range
    // INCLUDES hole circles that cross the outer segments mid-span
    // (lie #5b manufactured on purpose) AND holes that sit clear —
    // the former must be rejected at construction with both entities
    // and the point named, the latter must construct.
    Prng prng(0xB0D5EEDu);
    int crossing_members = 0, conforming_members = 0, pure_pairwise_members = 0;
    for (int member = 0; member < 40; ++member) {
        const std::vector<SketchChain> outer{rect_chain(-1.0f, -0.5f, 1.0f, 0.5f)};
        const Vec2f c{prng.frange(-1.2f, 1.2f), prng.frange(-0.7f, 0.7f)};
        const float r = prng.frange(0.1f, 0.55f);
        const std::vector<SketchChain> chains{
            outer[0], SketchChain{{sketch_circle(c, r, /*hole=*/true)}}};

        // Regime classification, INDEPENDENT of the kernel: does the
        // circle's curve cross any of the four segments?
        bool crosses = false;
        std::vector<Vec2f> expected;
        for (const auto& e : outer[0].entities) {
            const float dmin = point_segment_distance(c, e.segment.a, e.segment.b);
            const float dmax = std::max((c - e.segment.a).length(),
                                        (c - e.segment.b).length());
            if (dmin <= r && r <= dmax) {
                crosses = true;
                // Roots on this segment (independent quadratic).
                const Vec2f d = e.segment.b - e.segment.a;
                const Vec2f ac = e.segment.a - c;
                const float A = d.dot(d), B = 2.0f * d.dot(ac),
                            C = ac.dot(ac) - r * r;
                const float disc = B * B - 4.0f * A * C;
                if (disc >= 0.0f) {
                    const float sq = std::sqrt(disc);
                    for (const float t : {(-B - sq) / (2.0f * A), (-B + sq) / (2.0f * A)})
                        if (t >= 0.0f && t <= 1.0f)
                            expected.push_back(e.segment.a + d * t);
                }
            }
        }
        // Conforming: the hole sits strictly inside the rectangle.
        const bool inside = c.x + r < 1.0f && c.x - r > -1.0f &&
                            c.y + r < 0.5f && c.y - r > -0.5f;

        if (crosses) {
            ++crossing_members;
            // Which instrument SHOULD fire? Independent oracle over the
            // certificate's own checkpoints: if some checkpoint is
            // already violated, the samples fire (and name it); if all
            // checkpoints are clean, ONLY the pairwise can catch the
            // crossing — the pure lie-#5b anatomy. Both are loud; the
            // family asserts the right one per member.
            bool checkpoints_clean = true;
            for (const Vec2f& cp : certificate_checkpoints(chains))
                if (std::abs(oracle_composed(chains, cp)) > kSampleTol) {
                    checkpoints_clean = false;
                    break;
                }
            if (checkpoints_clean) ++pure_pairwise_members;
            try {
                ProfileSDF prof(chains);
                (void)prof;
                FAIL("expected std::invalid_argument for a crossing member");
            } catch (const std::invalid_argument& e) {
                const std::string msg = e.what();
                if (checkpoints_clean) {
                    // Pure lie #5b: only the pairwise instrument can refuse.
                    CHECK(msg.find("declared shared joints") != std::string::npos);
                    CHECK(msg.find("circle[chain 1 #0]") != std::string::npos);
                    CHECK(msg.find("segment[chain 0 #") != std::string::npos);
                    REQUIRE(!expected.empty());
                    const auto points = parse_message_points(msg);
                    bool found = false;
                    for (const Vec2f& p : points)
                        for (const Vec2f& x : expected)
                            if ((p - x).length() < 1e-3f) found = true;
                    CHECK(found); // the crossing point is named
                } else {
                    // The samples instrument caught the violated checkpoint.
                    CHECK(msg.find("must vanish") != std::string::npos);
                }
            }
        } else if (inside) {
            ++conforming_members;
            CHECK_NOTHROW(ProfileSDF{chains});
        }
    }
    // Liveness: the generator actually produced both regimes — and the
    // pure pairwise anatomy is IN its range (the house lesson).
    CHECK(crossing_members >= 5);
    CHECK(conforming_members >= 5);
    CHECK(pure_pairwise_members >= 1);
}
