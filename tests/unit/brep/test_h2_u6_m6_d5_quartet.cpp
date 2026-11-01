// tests/unit/brep/test_h2_u6_m6_d5_quartet.cpp
//
// H2 unit 6, morso 6 (FINALE) — IL QUARTETTO D5: L'EXIT CRITERION
// DELL'UNITÀ (ADR-0036 D5 + D6.5, the accepted act). The discriminating
// tests, in the order the ladder derived (1 with 4's first half; 2's
// two scenarios; 3 with 4's second half). THIS morso closes the unit:
// the naive demonstrated failing, the name demonstrated holding.
//
//   1. LA RIGENERAZIONE END-TO-END (D5.1) — the guide case WHOLE
//      (box → sketch → extrude → subtract): the name digest of the
//      complete chain pinned, the CO-EDGE leg landed (D5.1's own
//      words: "each face's record gains its name, each co-edge's
//      record gains its edge's name" — the leg the morsi 3-4 digest
//      pairs did not yet carry); N=10 rebuilds byte-identical on BOTH
//      compilers (the pinned FNV-1a 64 constants are the proof — zero
//      floating-point content, the same values under gcc and under
//      clang, the CI's two columns); the OLD oracles intact VERBATIM
//      (the m4 pins re-asserted character for character — a golden
//      that moves is a defect, not a re-pin: none moved).
//   2. I DUE EDIT TOPOLOGICI (D5.2) — the classes 3 and 4 SUL VERO,
//      both pure recipe edits, both in the exact class:
//      (a) the tool's circle re-authored as a CHAIN OF TWO ARCS — the
//          wall reference resolves reference_ambiguous NOMINATING
//          lat(0) E lat(1) and the remedy; the KIND and the COMPLETE
//          PAYLOAD pinned, not just the failure;
//      (b) the circle replaced by a SQUARE CHAIN — the rim reference
//          resolves reference_vanished NOMINATING the dead rim, the
//          cause (the authored entity was replaced) and the remedy.
//      IN BOTH: mai un crash (the resolution is a value), mai un
//      silenzio (the payload names everything) — and the tavola of
//      the NEW outcome is COERENTE: both closure sides green before
//      and after the edit.
//   3. L'EDIT CHE PRESERVA LA TOPOLOGIA (D5.3) — the class 1 SUL VERO,
//      and THE PAIR IS THE GOLDEN: the same scenario run twice — the
//      radius edit (within the class) → class 1, the feature re-runs
//      WITHOUT RESELECTION and the SILENCE itself is asserted (no log
//      line: there is nothing to say); the out-of-class edit (the
//      square chain) → class 4. The discrimination is the EDIT's
//      class: the name is a STRUCTURAL fact, not a geometric one.
//   4. LA CONTROPROVA NAIVE (D5.4) — the gesture that closes the unit.
//      The scenario, pinned: the target is the EXTRUDED SQUARE PRISM
//      (ext#2, chain of 4 — the same convex planar class as the box;
//      the subtract treats it identically: the same 12/17/7/2); the
//      fillet references the top rim
//      'sub#4:isect[ext#2:cap(hi) × ext#3:lat(0)]@hi', whose
//      materialized half-arc entities sit at the POSITIONAL ids
//      12-15 (E_A = 12). THE EDIT: the square sketch becomes PENTAGON
//      — the closing segment redirected, ONE entity appended (the
//      add-at-end family: no root moved, root 4 born) — and the
//      counts move EXACTLY as the pinned formula says (V: 8+4 →
//      10+4, E: 12+5 → 15+5, F: 6+1 → 7+1, χ = 0 in both;
//      subtract.hpp:84-88's general formula), asserted. THE TWO
//      CLAIMS, BOTH demonstrated:
//      1. ROSSO DELL'INGENUO: the positional ids of the referenced
//         rim ARE MOVED (the arcs now 15-18, the @hi pair 17/18) and
//         the entity that today answers to id 14 IS THE LAST VERTICAL
//         EDGE OF THE PENTAGON (the closing joint, asserted
//         geometrically: its endpoints and its position) — the
//         reference by address now points at A DIFFERENT AND WRONG
//         entity: the silent mis-reference CAPTURED, asserted, never
//         narrated. The naive block EXISTS in the test and FAILS
//         there — the discriminating test of the permanent contract.
//      2. VERDE DEL NOME: the reference by name re-derives class 1
//         (the appended moves nothing — the add-at-end covered) and
//         the feature re-runs on the new geometry without reselection.
//
// THE ORACLES (D5's golden declaration, honored to the letter): the
// name digest (faces, edges) re-asserts the m4 pins VERBATIM — no
// existing golden moved; the CO-EDGE leg is a NEW oracle, added
// beside the existing ones, bit-identical from its FIRST pinning
// (no prior name state to move). Determinism is ADR-0014's trivial
// case (zero floating-point content) — the pinned FNV-1a 64
// constants hold under gcc and under clang alike, which is the
// cross-compiler claim's own proof.
//
// THE HONEST BOUNDARIES, pinned as the design they are: the pentagon
// edit's honest shape (the m6 register, in the ADR's own act) — a
// closed chain cannot gain an entity by a bare append (the certificate
// refuses the degenerate closer); the canvas's real add-at-end flow is
// the closing segment REDIRECTED to the new vertex plus ONE segment
// appended — no root moved (0..3 keep their positions), root 4 born;
// the prism's own edges (the bottom/top curves, the verticals) read
// honestly EMPTY (outside D1's v1 extrude vocabulary) — and that very
// emptiness is the counterproof's sharpest edge: the entity that
// answers to the old address is not even nameable, while the name
// answers to the living rim.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

// ---------------------------------------------------------------------------
// The digest oracles — the m4 shapes VERBATIM (the old oracles intact)
// plus the D5.1 CO-EDGE leg (the quartet's own landing): per face, in
// address order, every co-edge of every loop carrying its edge's name.
// Pure integer arithmetic over the bytes, FNV-1a 64 — ADR-0014 honored
// by construction (the '×' renders multi-byte UTF-8: the hash walks
// BYTES, and .size() counts them — the literals below are the same
// bytes the walk hashed).
// ---------------------------------------------------------------------------

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
    return out;
}

std::string edge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

// THE CO-EDGE LEG (D5.1's own words, landed here — the quartet's own
// oracle): "each co-edge's record gains its edge's name". The rows
// walk the face's own loops in their own order — the wall's single
// loop carries the seam TWICE (the periodic identification, the XOR
// contract's own signature) and the pierced faces carry their inner
// loops' arcs under the born rim's ONE name (the seam-wired pair:
// both halves answer to it, ADR-0023).
std::string coedge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        const b::FaceInfo f = b::query_face(body, i);
        for (const auto& loop : f.loops) {
            for (const auto& ce : loop.coedges) {
                out += std::to_string(i);
                out += '|';
                out += b::query_edge(body, ce.edge).name;
                out += '\n';
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// The recipes — the m5 helpers verbatim (the same profiles the m5 pins
// ride on) plus the D5.4 scenario's own (the square/pentagon TARGET
// chains and the fixed circle tool)
// ---------------------------------------------------------------------------

// The guide case's tool recipe: the circle at the origin, r — the
// ADR-0032 D7.5 body; the radius is the EDIT KNOB of family 3.
std::vector<s::SketchChain> circle_chains(float radius = 0.25f) {
    return {s::SketchChain{
        {s::sketch_circle(Vec2f{0.0f, 0.0f}, radius, false)}}};
}

// THE TWO-ARC CHAIN (D5.2(a)'s own scenario, the m5 recipe verbatim):
// the circle re-authored as a chain of TWO ARCS — the lens.
std::vector<s::SketchChain> lens_chains() {
    const float r = 0.3f;
    const Vec2f c1{-0.12f, 0.0f}, c2{0.12f, 0.0f};
    const float y = std::sqrt(r * r - 0.12f * 0.12f);
    const Vec2f a{0.0f, y}, bb{0.0f, -y};
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;
    const float ang_b1 = std::atan2(bb.y - c1.y, bb.x - c1.x);
    const float ang_a1 = std::atan2(a.y - c1.y, a.x - c1.x);
    float span1 = ang_a1 - ang_b1;
    if (span1 < 0.0f) span1 += two_pi;
    const float ang_a2 = std::atan2(a.y - c2.y, a.x - c2.x);
    const float ang_b2 = std::atan2(bb.y - c2.y, bb.x - c2.x);
    float span2 = ang_b2 - ang_a2;
    if (span2 < 0.0f) span2 += two_pi;
    return {s::SketchChain{{
        s::sketch_arc(c1, r, false, ang_b1, span1),
        s::sketch_arc(c2, r, false, ang_a2, span2),
    }}};
}

// THE SQUARE CHAIN AS THE TOOL (D5.2(b)'s own scenario, the m5 recipe
// verbatim): the circle replaced by a square of four segments.
std::vector<s::SketchChain> square_tool_chains(float half = 0.2f) {
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-half, -half}, Vec2f{half, -half}, true),
        s::sketch_segment(Vec2f{half, -half}, Vec2f{half, half}, true),
        s::sketch_segment(Vec2f{half, half}, Vec2f{-half, half}, true),
        s::sketch_segment(Vec2f{-half, half}, Vec2f{-half, -half}, true),
    }}};
}

// ---------------------------------------------------------------------------
// The D1 chain (the ADR's own FeatureIds) — the m5 construction, plus
// the D5.4 scenario's chain: the TARGET is the extruded prism, the
// tool the fixed circle, the ids the ADR's own (ext#2 the target —
// the box's slot in the D1 chain, taken by the prism the same class;
// ext#3 the tool; sub#4 the subtract)
// ---------------------------------------------------------------------------

// The guide case (the m5 shape verbatim): box#1 -> sk#2 -> ext#3 ->
// sub#4, the tool recipe the EDIT KNOB.
struct GuideCase {
    std::unique_ptr<b::BRepBody> box;    // box#1
    std::vector<s::SketchChain> chains;  // the sk#2 recipe
    std::unique_ptr<b::BRepBody> tool;   // ext#3
    std::unique_ptr<b::BRepBody> result; // sub#4's result
};

GuideCase guide_case(std::vector<s::SketchChain> chains) {
    GuideCase g;
    g.box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    g.chains = std::move(chains);
    g.tool = b::make_prism(g.chains, 3.0f, +1, FeatureId{3});
    const b::SubtractNaming ctx{
        FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
    g.result = b::exact_subtract(
        *g.box, *g.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance, ctx);
    return g;
}

// THE D5.4 SCENARIO (the ADR's own): the target is the EXTRUDED
// SQUARE PRISM — ext#2, chain of 4, the same convex planar class as
// the box (the subtract treats it identically) — the tool the fixed
// circle ext#3 (the r = 0.25 through hole), the subtract sub#4. The
// target's sketch is the EDIT KNOB of family 4; the tool's is not
// touched by the edit.
float kD54Half = 0.5f;    // the square's half: the box's own 1x1
                           // cross-section, the guide case's proportion
Vec2f kD54P{-0.85f, 0.0f};  // the pentagon's new vertex

std::vector<s::SketchChain> square_target_chains() {
    const float h = kD54Half;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-h, -h}, Vec2f{h, -h}, true),
        s::sketch_segment(Vec2f{h, -h}, Vec2f{h, h}, true),
        s::sketch_segment(Vec2f{h, h}, Vec2f{-h, h}, true),
        s::sketch_segment(Vec2f{-h, h}, Vec2f{-h, -h}, true),
    }}};
}

// THE PENTAGON (the edit): the same first three segments (the roots
// 0..2 untouched, verbatim), the closing segment REDIRECTED to the new
// vertex P (a parameter edit on root 3), ONE segment appended (P back
// to the start — root 4 born). The add-at-end family: no root moved,
// nothing inserted, nothing deleted; the certificate certifies the
// convex CCW pentagon (the disc r = 0.25 strictly interior, the
// same class as before).
std::vector<s::SketchChain> pentagon_target_chains() {
    const float h = kD54Half;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-h, -h}, Vec2f{h, -h}, true),
        s::sketch_segment(Vec2f{h, -h}, Vec2f{h, h}, true),
        s::sketch_segment(Vec2f{h, h}, Vec2f{-h, h}, true),
        s::sketch_segment(Vec2f{-h, h}, kD54P, true),
        s::sketch_segment(kD54P, Vec2f{-h, -h}, true),
    }}};
}

struct D54Case {
    std::unique_ptr<b::BRepBody> target; // ext#2 (the prism)
    std::unique_ptr<b::BRepBody> tool;    // ext#3 (the circle)
    std::unique_ptr<b::BRepBody> result;  // sub#4's result
};

D54Case d54_case(std::vector<s::SketchChain> target_chains) {
    D54Case g;
    g.target = b::make_prism(target_chains, 1.0f, +1, FeatureId{2});
    g.tool = b::make_prism(circle_chains(0.25f), 3.0f, +1, FeatureId{3});
    const b::SubtractNaming ctx{
        FeatureId{4}, &n::kExtNaming, &n::kExtNaming};
    g.result = b::exact_subtract(
        *g.target, *g.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance, ctx);
    return g;
}

// The subtract's INSTANCE registries (the m4/m5 SubRegistry pattern):
// the guide case's (the box target) and the D5.4 scenario's (the
// prism target — BOTH operands the extrude's).
struct SubRegistry {
    n::OperandBinding operands[2]{};
    n::OperationNaming naming{};

    explicit SubRegistry(const GuideCase& g) {
        const auto& box_top =
            g.box->solid.shell().faces()[1];   // box#1:top's face
        const auto& wall =
            g.tool->solid.shell().faces()[0];  // ext#3:lat(0)'s face
        operands[0] = {box_top.shape_id().feature, &n::kBoxNaming};
        operands[1] = {wall.shape_id().feature, &n::kExtNaming};
        naming = n::OperationNaming{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles, operands};
    }

    explicit SubRegistry(const D54Case& g) {
        // The rim's grammar: the isect legs walk the TARGET's cap(hi)
        // (the prism's, the extrude's table) and the TOOL's lat(0).
        const auto& cap_hi =
            g.target->solid.shell().faces()[0];  // ext#2's own face
        const auto& wall =
            g.tool->solid.shell().faces()[0];    // ext#3:lat(0)'s face
        operands[0] = {cap_hi.shape_id().feature, &n::kExtNaming};
        operands[1] = {wall.shape_id().feature, &n::kExtNaming};
        naming = n::OperationNaming{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles, operands};
    }
};

// The generations the resolver reads (the m5 shape).
struct Generations {
    b::DescentTable old_tavola{};
    b::Generation old_gen{};
    b::Generation new_gen{};
};

Generations subtract_generations(const GuideCase& old_g,
                                 const GuideCase& new_g,
                                 const n::OperationNaming* grammar) {
    Generations out;
    out.old_tavola =
        b::subtract_descent(*old_g.result, *old_g.box, *old_g.tool);
    out.old_gen = b::Generation{old_g.result.get(), &out.old_tavola,
                                grammar, false};
    out.new_gen = b::Generation{new_g.result.get(), nullptr,
                                grammar, false};
    return out;
}

Generations d54_generations(const D54Case& old_g, const D54Case& new_g,
                            const n::OperationNaming* grammar) {
    Generations out;
    out.old_tavola =
        b::subtract_descent(*old_g.result, *old_g.target, *old_g.tool);
    out.old_gen = b::Generation{old_g.result.get(), &out.old_tavola,
                                grammar, false};
    out.new_gen = b::Generation{new_g.result.get(), nullptr,
                                grammar, false};
    return out;
}

// The downstream feature's STAND-IN (the m5 shape, one refinement the
// PAIR demands): holds the reference BY NAME, re-runs on the
// resolution. THE SILENCE IS ITS OWN PROTOCOL: the success is not
// logged (nothing to say); every other class writes the log. `ran`
// reads the LAST re-run (the pair's second leg must not inherit the
// first's success): a feature that re-ran once and now refuses is
// logged, never silently carried.
struct DownstreamFeature {
    ShapeId reference{};
    std::vector<std::string> log{};
    bool ran{false};

    bool re_run(const b::Generation& old_gen,
                const b::Generation& new_gen,
                const b::EditContext& edit_ctx) {
        ran = false;  // the LAST re-run's own verdict
        const b::Resolution r =
            b::resolve(reference, old_gen, new_gen, edit_ctx);
        if (r.kind == b::Resolution::Kind::Resolved) {
            ran = true;  // silent — no log line, the feature re-runs
            return true;
        }
        log.push_back(b::resolution_text(r));
        return false;
    }
};

// The new tavola's coherence, both sides (D5.2's own demand): the
// closure check green before and after the edit — never a crash, so
// a violation would be a DEFECT the check names, never a silence.
bool tavola_coherent(const GuideCase& g) {
    const b::DescentTable tav =
        b::subtract_descent(*g.result, *g.box, *g.tool);
    std::string why;
    if (!b::descent_closed(*g.result, tav, &why)) return false;
    return b::old_generation_closed(*g.box, *g.tool, tav, &why);
}

} // namespace

// ===========================================================================
// FAMILY 1 (D5.1) — LA RIGENERAZIONE END-TO-END: the whole chain's
// name digest pinned, the co-edge leg landed, N=10 both compilers
// ===========================================================================

TEST_CASE("H2-U6-M6 D5.1: the regeneration — the whole chain's name "
          "digest pinned, the co-edge leg landed, N=10 byte-identical") {
    const GuideCase g = guide_case(circle_chains(0.25f));

    // THE OLD ORACLES INTACT, VERBATIM — the m4 pins re-asserted
    // character for character (the box, the tool, the result): no
    // existing golden moved; a golden that moves is a defect, not a
    // re-pin — and none moved (this file re-asserts, it does not pin
    // anew).
    CHECK(name_digest(*g.box) ==
          "0|box#1:bottom\n"
          "1|box#1:top\n"
          "2|box#1:front\n"
          "3|box#1:back\n"
          "4|box#1:left\n"
          "5|box#1:right\n");
    CHECK(edge_name_digest(*g.box) ==
          "0|box#1:bottom0\n"
          "1|box#1:bottom1\n"
          "2|box#1:bottom2\n"
          "3|box#1:bottom3\n"
          "4|box#1:top0\n"
          "5|box#1:top1\n"
          "6|box#1:top2\n"
          "7|box#1:top3\n"
          "8|box#1:vertical0\n"
          "9|box#1:vertical1\n"
          "10|box#1:vertical2\n"
          "11|box#1:vertical3\n");
    CHECK(name_digest(*g.tool) ==
          "0|ext#3:lat(0)\n"
          "1|ext#3:cap(lo)\n"
          "2|ext#3:cap(hi)\n");
    CHECK(edge_name_digest(*g.tool) ==
          "0|\n"
          "1|\n"
          "2|\n"
          "3|\n"
          "4|ext#3:seam\n");
    CHECK(name_digest(*g.result) ==
          "0|box#1:bottom\n"
          "1|box#1:top\n"
          "2|box#1:front\n"
          "3|box#1:back\n"
          "4|box#1:left\n"
          "5|box#1:right\n"
          "6|ext#3:lat(0)\n");
    CHECK(edge_name_digest(*g.result) ==
          "0|box#1:bottom0\n"
          "1|box#1:bottom1\n"
          "2|box#1:bottom2\n"
          "3|box#1:bottom3\n"
          "4|box#1:top0\n"
          "5|box#1:top1\n"
          "6|box#1:top2\n"
          "7|box#1:top3\n"
          "8|box#1:vertical0\n"
          "9|box#1:vertical1\n"
          "10|box#1:vertical2\n"
          "11|box#1:vertical3\n"
          "12|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "13|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "14|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "15|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "16|sub#4:seam[ext#3:lat(0)]\n");

    // THE CO-EDGE LEG (D5.1's own words, landed HERE — the quartet's
    // own oracle, bit-identical from its FIRST pinning): every
    // co-edge's record carries its edge's name. The wall's single
    // loop walks the seam TWICE (the periodic identification); the
    // pierced faces' inner loops carry the born rim's ONE name on
    // BOTH halves (the seam-wired pair, ADR-0023); the tool's rim
    // halves and caps' closed curves read honestly EMPTY.
    CHECK(coedge_name_digest(*g.box) ==
          "0|box#1:bottom3\n"
          "0|box#1:bottom2\n"
          "0|box#1:bottom1\n"
          "0|box#1:bottom0\n"
          "1|box#1:top0\n"
          "1|box#1:top1\n"
          "1|box#1:top2\n"
          "1|box#1:top3\n"
          "2|box#1:bottom0\n"
          "2|box#1:vertical1\n"
          "2|box#1:top0\n"
          "2|box#1:vertical0\n"
          "3|box#1:bottom2\n"
          "3|box#1:vertical3\n"
          "3|box#1:top2\n"
          "3|box#1:vertical2\n"
          "4|box#1:bottom3\n"
          "4|box#1:vertical0\n"
          "4|box#1:top3\n"
          "4|box#1:vertical3\n"
          "5|box#1:bottom1\n"
          "5|box#1:vertical2\n"
          "5|box#1:top1\n"
          "5|box#1:vertical1\n");
    CHECK(coedge_name_digest(*g.tool) ==
          "0|ext#3:seam\n"
          "0|\n"
          "0|\n"
          "0|ext#3:seam\n"
          "0|\n"
          "0|\n"
          "1|\n"
          "1|\n"
          "2|\n"
          "2|\n");
    CHECK(coedge_name_digest(*g.result) ==
          "0|box#1:bottom3\n"
          "0|box#1:bottom2\n"
          "0|box#1:bottom1\n"
          "0|box#1:bottom0\n"
          "0|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "0|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "1|box#1:top0\n"
          "1|box#1:top1\n"
          "1|box#1:top2\n"
          "1|box#1:top3\n"
          "1|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "1|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "2|box#1:bottom0\n"
          "2|box#1:vertical1\n"
          "2|box#1:top0\n"
          "2|box#1:vertical0\n"
          "3|box#1:bottom2\n"
          "3|box#1:vertical3\n"
          "3|box#1:top2\n"
          "3|box#1:vertical2\n"
          "4|box#1:bottom3\n"
          "4|box#1:vertical0\n"
          "4|box#1:top3\n"
          "4|box#1:vertical3\n"
          "5|box#1:bottom1\n"
          "5|box#1:vertical2\n"
          "5|box#1:top1\n"
          "5|box#1:vertical1\n"
          "6|sub#4:seam[ext#3:lat(0)]\n"
          "6|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "6|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "6|sub#4:seam[ext#3:lat(0)]\n"
          "6|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "6|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n");

    // N=10 REBUILDS, byte-identical: same recipes, fresh
    // materialization every time — no state, no counter, no
    // memoization. The pinned FNV-1a 64 constants are the
    // cross-compiler proof (the CI's two columns run this same
    // assertion under gcc and under clang): the digests carry zero
    // floating-point content, so the constants are the SAME VALUES
    // everywhere — bit-identical from the FIRST pinning, no
    // migration, no re-pin (there is no prior name state to move).
    std::string box_f, box_e, box_c, tool_f, tool_e, tool_c,
        res_f, res_e, res_c;
    for (int i = 0; i < 10; ++i) {
        const GuideCase again = guide_case(circle_chains(0.25f));
        if (i == 0) {
            box_f = name_digest(*again.box);
            box_e = edge_name_digest(*again.box);
            box_c = coedge_name_digest(*again.box);
            tool_f = name_digest(*again.tool);
            tool_e = edge_name_digest(*again.tool);
            tool_c = coedge_name_digest(*again.tool);
            res_f = name_digest(*again.result);
            res_e = edge_name_digest(*again.result);
            res_c = coedge_name_digest(*again.result);
        } else {
            REQUIRE(name_digest(*again.box) == box_f);
            REQUIRE(edge_name_digest(*again.box) == box_e);
            REQUIRE(coedge_name_digest(*again.box) == box_c);
            REQUIRE(name_digest(*again.tool) == tool_f);
            REQUIRE(edge_name_digest(*again.tool) == tool_e);
            REQUIRE(coedge_name_digest(*again.tool) == tool_c);
            REQUIRE(name_digest(*again.result) == res_f);
            REQUIRE(edge_name_digest(*again.result) == res_e);
            REQUIRE(coedge_name_digest(*again.result) == res_c);
        }
    }
    // The pinned constants (the m4 columns re-asserted verbatim —
    // they did not move either) plus the co-edge leg's own, first
    // pinned HERE.
    CHECK(fnv1a64(res_f) == 0x5ff1eed6f2fba4c0ull);
    CHECK(fnv1a64(res_e) == 0x63cf590ec75cc1deull);
    CHECK(fnv1a64(box_f) == 0x4340ed4737f7ca41ull);
    CHECK(fnv1a64(tool_f) == 0x27c8a19f27563383ull);
    CHECK(fnv1a64(box_c) == 0x35986d75c428e819ull);
    CHECK(fnv1a64(tool_c) == 0xf1bf69ee52a8bae5ull);
    CHECK(fnv1a64(res_c) == 0x1d4dac19e0a4b46dull);

    // The unit-4 address plan, re-asserted verbatim on the same
    // recipe: the counts, χ, the wall ONE cylinder face — the names
    // arrived BESIDE the addresses, the numbering never moved.
    const b::BodySummary sum = b::query_body(*g.result);
    CHECK(sum.vertex_count == 12);
    CHECK(sum.edge_count == 17);
    CHECK(sum.face_count == 7);
    CHECK(sum.inner_loops == 2);
    CHECK(sum.chi == 0);
    CHECK(sum.genus_derived);
    CHECK(sum.genus == 1);
    CHECK(b::query_face(*g.result, 6).family == "cylinder");
}

// ===========================================================================
// FAMILY 2 (D5.2) — I DUE EDIT TOPOLOGICI: the classes 3 and 4 SUL
// VERO, the new tavole COERENTI (both closure sides green)
// ===========================================================================

TEST_CASE("H2-U6-M6 D5.2(a): the two-arc re-authoring — the wall "
          "reference AMBIGUOUS, lat(0) E lat(1) nominated, the tavola "
          "coherent") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(lens_chains());
    // The edit re-shaped the tool: ONE wall face became TWO (the
    // formula's own arithmetic: n = 1 -> 2, F = 6+n) — the m5 pins,
    // re-asserted here on the same recipe.
    REQUIRE(new_g.result->face_count() == 8);
    REQUIRE(new_g.result->edge_count() == 18);
    REQUIRE(b::query_body(*new_g.result).chi == 0);

    // THE NEW TAVOLA COERENTE (D5.2's own demand): both closure sides
    // green on the NEW outcome — the edit never crashes the bookkeeping
    // and never silences a hole in it.
    REQUIRE(tavola_coherent(new_g));
    REQUIRE(tavola_coherent(old_g));  // and before the edit, too

    // The wall's grammar is the EXTRUDE's — the ORIGINAL creator's
    // (the m5 family 2 pattern: the subtract's face table is honestly
    // empty, it births no faces).
    const Generations gens =
        subtract_generations(old_g, new_g, &n::kExtNaming);

    // THE PICK: the wall — the tool's ONE lateral, the absorbed face
    // that keeps its birth name ext#3:lat(0).
    const b::FaceInfo picked = b::query_face(*old_g.result, 6);
    REQUIRE(picked.name == "ext#3:lat(0)");
    const ShapeId reference = picked.id;

    // The edit context: EMPTY — the re-authoring is not a covered
    // flow and emits no map: the roots moved, the door is closed.
    const b::EditContext edit_ctx{};

    // THE RESOLUTION: mai un crash (a value), mai un silenzio (the
    // payload names everything). The KIND and the COMPLETE PAYLOAD:
    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);
    REQUIRE(r.kind == b::Resolution::Kind::Ambiguous);
    CHECK(b::resolution_code(r.kind) ==
          std::string("reference_ambiguous"));
    REQUIRE(r.ambiguous.candidates.size() == 2);
    REQUIRE(r.ambiguous.candidate_names.size() == 2);
    ShapeId want0{FeatureId{3}, SubshapeRef{}.push(0).push(0), {}};
    ShapeId want1{FeatureId{3}, SubshapeRef{}.push(0).push(1), {}};
    CHECK(r.ambiguous.candidates[0].subshape == want0.subshape);
    CHECK(r.ambiguous.candidates[1].subshape == want1.subshape);
    CHECK(r.ambiguous.candidate_names[0] == "ext#3:lat(0)");
    CHECK(r.ambiguous.candidate_names[1] == "ext#3:lat(1)");
    CHECK(r.ambiguous.entity_name == "ext#3:lat(0)");
    CHECK(r.ambiguous.remedy.find("reselect") != std::string::npos);

    // THE NAIVE BLOCK, in the negative: the positional id WOULD have
    // answered — the new body carries a wall stamped exactly [lat, 0]
    // (the FIRST ARC's — a different authored entity) — the roots rule
    // is the door that stops it (the m5 counterproof, re-asserted).
    const b::FaceInfo naive = b::query_face(*new_g.result, 6);
    REQUIRE(naive.name == "ext#3:lat(0)");
    CHECK(naive.id.subshape == reference.subshape);
    CHECK(naive.id.subshape != want1.subshape);

    // The report as text: the candidates in it, verbatim.
    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_ambiguous") == 0);
    CHECK(text.find("ext#3:lat(0)") != std::string::npos);
    CHECK(text.find("ext#3:lat(1)") != std::string::npos);
    CHECK(text.find("reselect") != std::string::npos);
}

TEST_CASE("H2-U6-M6 D5.2(b): the square chain — the rim reference "
          "VANISHED, the dead rim and the cause and the remedy named, "
          "the tavola coherent") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(square_tool_chains());
    // The square tool: n = 4 (the formula: V = 8+8, E = 12+12,
    // F = 6+4) — the m5 pins, re-asserted on the same recipe.
    REQUIRE(new_g.result->face_count() == 10);
    REQUIRE(new_g.result->edge_count() == 24);
    REQUIRE(b::query_body(*new_g.result).chi == 0);

    // THE NEW TAVOLA COERENTE: both closure sides green — the edit
    // killed the rim AND the bookkeeping says so, each dead entity in
    // exactly one vanished row.
    REQUIRE(tavola_coherent(new_g));

    const SubRegistry reg{old_g};
    const Generations gens =
        subtract_generations(old_g, new_g, &reg.naming);

    // THE PICK: the rim — the fillet's own string.
    const b::EdgeInfo picked = b::query_edge(*old_g.result, 14);
    REQUIRE(picked.name ==
            "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    const ShapeId reference = picked.id;

    const b::EditContext edit_ctx{};  // the replacement: no map

    // THE RESOLUTION: mai un crash, mai un silenzio — WHAT died (the
    // row carried whole), WHY (the authored entity was replaced), the
    // remedy. Never a mute "not found".
    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);
    REQUIRE(r.kind == b::Resolution::Kind::Vanished);
    CHECK(b::resolution_code(r.kind) ==
          std::string("reference_vanished"));
    CHECK(r.vanished.row.relation == b::Descent::Born);
    CHECK(r.vanished.row.entity == b::EntityKind::Edge);
    CHECK(r.vanished.row.address == 14);
    CHECK(r.vanished.row.name == picked.name);
    CHECK(r.vanished.why == "the authored entity was replaced");
    CHECK(r.vanished.remedy.find("reselect") != std::string::npos);
    CHECK(r.vanished.remedy.find("revert") != std::string::npos);

    // The report as text.
    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_vanished") == 0);
    CHECK(text.find(picked.name) != std::string::npos);
    CHECK(text.find("the authored entity was replaced") !=
          std::string::npos);
}

// ===========================================================================
// FAMILY 3 (D5.3) — L'EDIT CHE PRESERVA LA TOPOLOGIA, E LA COPPIA È IL
// GOLDEN: the same scenario run twice — within class → 1, out of
// class → 4
// ===========================================================================

TEST_CASE("H2-U6-M6 D5.3: the pair is the golden — the radius edit "
          "class 1 SILENT, the out-of-class edit class 4, the SAME "
          "reference") {
    // THE SAME SCENARIO, the same reference, the same downstream
    // feature: the guide case, the rim picked ONCE. The discrimination
    // is the EDIT's class — the name is a STRUCTURAL fact: within the
    // class it re-derives; out of class the entity died.
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const SubRegistry reg{old_g};

    // THE PICK (once): the rim — the fillet's own string, the pair's
    // own subject.
    const b::EdgeInfo picked = b::query_edge(*old_g.result, 14);
    REQUIRE(picked.name ==
            "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    DownstreamFeature fillet;
    fillet.reference = picked.id;

    // ---- THE FIRST RUN: the edit WITHIN the class (the radius grows,
    // the hole stays a hole) → class 1, SILENT ----
    {
        const GuideCase new_g = guide_case(circle_chains(0.30f));
        // The topology kept: the class held, the counts unchanged.
        REQUIRE(new_g.result->edge_count() == 17);
        REQUIRE(new_g.result->face_count() == 7);
        REQUIRE(tavola_coherent(new_g));
        const Generations gens =
            subtract_generations(old_g, new_g, &reg.naming);

        // The covered flow's declaration (the edit path emits it, the
        // resolver consumes it, never invents it).
        const b::EditContext edit_ctx{/*roots_no_shift=*/true, {}};

        // THE RESOLUTION: the rim re-derives IDENTICALLY (the name is
        // a fact about the GENEALOGY, not the geometry — the rim moved
        // in space and kept its name) and the feature RIPARTE SENZA
        // RISELEZIONE.
        CHECK(fillet.re_run(gens.old_gen, gens.new_gen, edit_ctx));
        REQUIRE(fillet.ran);

        // THE SILENCE, asserted: reference_resolved logs NOTHING —
        // there is nothing to say. The success is not an event.
        CHECK(fillet.log.empty());

        // The feature computed on the NEW geometry: the resolved name
        // answers the seam-wired PAIR on the new body, and the new
        // radius is live right there (the pierced face's inner loop,
        // the fillet's own input curve).
        const std::vector<b::EdgeId> ids =
            b::query_edge_by_name(*new_g.result, fillet.reference);
        REQUIRE(ids.size() == 2);
        CHECK(ids[0] == 14);
        CHECK(ids[1] == 15);
        const b::FaceInfo top = b::query_face(*new_g.result, 1);
        REQUIRE(top.inner_loop_count == 1);
        std::size_t arcs = 0;
        for (const auto& loop : top.loops) {
            for (const auto& ce : loop.coedges) {
                if (ce.kind == b::PCurveInfo::Kind::Arc) {
                    ++arcs;
                    CHECK(ce.radius ==
                          doctest::Approx(0.30).epsilon(1e-6));
                }
            }
        }
        CHECK(arcs >= 2);
    }

    // ---- THE SECOND RUN: the edit OUT of class (the circle replaced
    // by the square chain) → class 4, the SAME reference ----
    {
        const GuideCase new_g = guide_case(square_tool_chains());
        REQUIRE(tavola_coherent(new_g));
        const Generations gens =
            subtract_generations(old_g, new_g, &reg.naming);

        // The replacement: no map, no declaration — the honest class.
        const b::EditContext edit_ctx{};

        // THE RESOLUTION: the SAME reference, now VANISHED — the pair
        // is the golden, not the single: the name held under the
        // benign edit and DIED under the replacing one, and both
        // verdicts are the EDIT's own, never a guess.
        CHECK_FALSE(fillet.re_run(gens.old_gen, gens.new_gen, edit_ctx));
        CHECK_FALSE(fillet.ran);
        REQUIRE(fillet.log.size() == 1);
        // The log line (the feature's log — never silence): the dead
        // rim NAMED, the cause, the remedy.
        const std::string& text = fillet.log.front();
        CHECK(text.find("reference_vanished") == 0);
        CHECK(text.find(picked.name) != std::string::npos);
        CHECK(text.find("the authored entity was replaced") !=
              std::string::npos);
        CHECK(text.find("reselect") != std::string::npos);
    }
}

// ===========================================================================
// FAMILY 4 (D5.4) — LA CONTROPROVA NAIVE: red-on-address demonstrated,
// green-on-name demonstrated, the mis-reference CAPTURED
// ===========================================================================

TEST_CASE("H2-U6-M6 D5.4: the naive counterproof — the address moved "
          "and now points at the WRONG entity; the name held and the "
          "feature re-runs") {
    // ---- THE SCENARIO, pinned ----
    // The target: the EXTRUDED SQUARE PRISM (ext#2, chain of 4 — the
    // same convex planar class as the box; the subtract treats it
    // identically: the same counts as the guide case). The tool: the
    // circle ext#3 (r = 0.25, through). The subtract: sub#4.
    const D54Case sq = d54_case(square_target_chains());
    const b::BodySummary sq_sum = b::query_body(*sq.result);
    CHECK(sq_sum.vertex_count == 12);  // 8 + 4: the formula, held
    CHECK(sq_sum.edge_count == 17);    // 12 + 5
    CHECK(sq_sum.face_count == 7);     // 6 + 1
    CHECK(sq_sum.inner_loops == 2);
    CHECK(sq_sum.chi == 0);
    // The same counts as the guide case (the box's): the prism is the
    // box's own class — the D5.4 scenario's own premise, verified.
    const GuideCase guide = guide_case(circle_chains(0.25f));
    CHECK(b::query_body(*guide.result).edge_count == sq_sum.edge_count);

    // The fillet's reference: the TOP RIM. E_A = 12 (the square
    // prism's own edge count): the born half-arcs sit at the
    // positional ids 12-15 (12/13 the @lo pair, 14/15 the @hi pair),
    // the seam at 16.
    const b::EdgeInfo picked = b::query_edge(*sq.result, 14);
    REQUIRE(picked.name ==
            "sub#4:isect[ext#2:cap(hi) × ext#3:lat(0)]@hi");
    const ShapeId reference = picked.id;
    REQUIRE(reference.feature == FeatureId{4});
    REQUIRE(reference.subshape.depth == 4);
    CHECK(b::query_edge(*sq.result, 12).name ==
          "sub#4:isect[ext#2:cap(lo) × ext#3:lat(0)]@lo");
    CHECK(b::query_edge(*sq.result, 15).name == picked.name);
    CHECK(b::query_edge(*sq.result, 16).name ==
          "sub#4:seam[ext#3:lat(0)]");
    // The materialized halves of the ONE name: the pair answers.
    const std::vector<b::EdgeId> was =
        b::query_edge_by_name(*sq.result, reference);
    REQUIRE(was.size() == 2);
    CHECK(was[0] == 14);
    CHECK(was[1] == 15);
    // The pierced face is the target's own cap(hi) — D5.4's own name.
    CHECK(b::query_face(*sq.result, 5).name == "ext#2:cap(hi)");
    // The wall is the tool's own lateral (absorbed, the birth name
    // kept), and the pierced top face carries the inner loop.
    CHECK(b::query_face(*sq.result, 6).name == "ext#3:lat(0)");
    CHECK(b::query_face(*sq.result, 5).inner_loop_count == 1);

    // ---- THE EDIT: the square sketch becomes PENTAGON ----
    // The add-at-end family (the m6 register's honest shape): the
    // closing segment REDIRECTED to the new vertex, ONE segment
    // appended — no root moved (0..3 keep their positions), root 4
    // born. The covered flow: the edit path's no-shift declaration.
    const D54Case pent = d54_case(pentagon_target_chains());

    // THE COUNTS, asserted against the pinned formula
    // (subtract.hpp:84-88): a chain tool of n entities through a
    // target with (V_A, E_A, F_A) gives V = V_A+2n, E = E_A+3n,
    // F = F_A+n, H = 2 → χ = 0 — and the circle is the seam-wired
    // special case (V_A+4, E_A+5, F_A+1). The TARGET grew: the
    // square prism (8, 12, 6) became the pentagon prism (10, 15, 7):
    const b::BodySummary pent_target = b::query_body(*pent.target);
    CHECK(pent_target.vertex_count == 10);  // was 8
    CHECK(pent_target.edge_count == 15);    // was 12 — E_A moved
    CHECK(pent_target.face_count == 7);     // was 6
    // And the RESULT moved exactly as the formula says:
    // V: 8+4 → 10+4, E: 12+5 → 15+5, F: 6+1 → 7+1, χ = 0 in both.
    const b::BodySummary pent_sum = b::query_body(*pent.result);
    CHECK(pent_sum.vertex_count == 14);  // 10 + 4
    CHECK(pent_sum.edge_count == 20);    // 15 + 5
    CHECK(pent_sum.face_count == 8);     // 7 + 1
    CHECK(pent_sum.inner_loops == 2);
    CHECK(pent_sum.chi == 0);
    CHECK(pent_sum.genus_derived);
    CHECK(pent_sum.genus == 1);
    // The pentagon prism's laterals are named (the roots held): the
    // appended root 4 has its own lat(4) — the add-at-end's own
    // signature, nothing shifted.
    CHECK(b::query_face(*pent.result, 4).name == "ext#2:lat(4)");
    CHECK(b::query_face(*pent.result, 5).name == "ext#2:cap(lo)");
    CHECK(b::query_face(*pent.result, 6).name == "ext#2:cap(hi)");

    // ---- CLAIM 1: ROSSO DELL'INGENUO, demonstrated ----
    // The positional ids of the referenced rim ARE MOVED: the born
    // half-arcs now sit at 15-18 (15/16 the @lo pair, 17/18 the @hi
    // pair — was 14/15), the seam at 19 (was 16).
    CHECK(b::query_edge(*pent.result, 15).name ==
          "sub#4:isect[ext#2:cap(lo) × ext#3:lat(0)]@lo");
    CHECK(b::query_edge(*pent.result, 17).name == picked.name);
    CHECK(b::query_edge(*pent.result, 18).name == picked.name);
    CHECK(b::query_edge(*pent.result, 19).name ==
          "sub#4:seam[ext#3:lat(0)]");

    // THE NAIVE BLOCK — the negative that carries the proof: the
    // entity that TODAY answers to the id 14 is THE LAST VERTICAL
    // EDGE OF THE PENTAGON — the vertical at the closing joint (the
    // prism's own edge 14: the vertical at J_4, the joint where the
    // chain closes), asserted structurally (the endpoints), by name
    // (honestly EMPTY: the prism's verticals are outside D1's v1
    // extrude vocabulary) and geometrically (the position).
    const b::EdgeInfo naive = b::query_edge(*pent.result, 14);
    CHECK(naive.name.empty());
    CHECK(naive.id.subshape.depth == 0);
    CHECK(naive.v0 == 8);
    CHECK(naive.v1 == 9);
    const auto& pv0 = pent.result->vertices[8];
    const auto& pv1 = pent.result->vertices[9];
    CHECK(pv0.position().x == doctest::Approx(-0.5));
    CHECK(pv0.position().y == doctest::Approx(-0.5));
    CHECK(pv0.position().z == doctest::Approx(0.0));
    CHECK(pv1.position().z == doctest::Approx(1.0));
    // The mis-reference, CAPTURED: the reference by address (the id
    // the pick answered at pick time) now points at a DIFFERENT and
    // WRONG entity — not the rim (a vertical, unnamed, unnameable in
    // v1) — and the naive test block FAILS exactly there, where the
    // name holds: THIS is the discrimination, the proof that the
    // naming is new, not repainted.
    CHECK(!(naive.id.feature == reference.feature &&
            naive.id.subshape == reference.subshape));
    CHECK(naive.v0 != picked.v0);
    CHECK(naive.name != picked.name);
    // A fillet that referenced BY ADDRESS would now silently fillet
    // the pentagon's closing vertical instead of the rim: the entity
    // count says it all — the rim is an ARC PAIR born of the
    // subtract; the entity at 14 is a LINE, kept verbatim from the
    // target's own construction.
    CHECK(b::query_edge(*sq.result, 14).id.subshape.depth == 4);
    CHECK(naive.id.subshape.depth == 0);

    // ---- CLAIM 2: VERDE DEL NOME, demonstrated ----
    // The reference BY NAME re-derives class 1: the appended moves
    // nothing (the add-at-end covered flow — the roots 0..3 kept
    // their positions, the tool's chain untouched, the name's
    // genealogy intact) and the feature RIPARTE.
    const SubRegistry reg{sq};
    const Generations gens = d54_generations(sq, pent, &reg.naming);
    const b::EditContext edit_ctx{/*roots_no_shift=*/true, {}};

    DownstreamFeature fillet;
    fillet.reference = reference;
    CHECK(fillet.re_run(gens.old_gen, gens.new_gen, edit_ctx));
    REQUIRE(fillet.ran);
    CHECK(fillet.log.empty());  // the silence, asserted again

    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);
    REQUIRE(r.kind == b::Resolution::Kind::Resolved);
    CHECK(r.resolved.name == picked.name);
    CHECK(r.resolved.id.subshape == reference.subshape);

    // THE FEATURE RE-RUNS on the new geometry, without reselection:
    // the resolved name answers the seam-wired PAIR at the NEW
    // addresses (17/18 — the pair that moved), and the rim is live
    // right there (the pierced cap(hi)'s inner loop, radius 0.25).
    const std::vector<b::EdgeId> now =
        b::query_edge_by_name(*pent.result, r.resolved.id);
    REQUIRE(now.size() == 2);
    CHECK(now[0] == 17);
    CHECK(now[1] == 18);
    CHECK(now[0] != was[0]);  // the address MOVED — and the name held
    const b::FaceInfo cap_hi = b::query_face(*pent.result, 6);
    REQUIRE(cap_hi.inner_loop_count == 1);
    std::size_t arcs = 0;
    for (const auto& loop : cap_hi.loops) {
        for (const auto& ce : loop.coedges) {
            if (ce.kind == b::PCurveInfo::Kind::Arc) {
                ++arcs;
                CHECK(ce.radius ==
                      doctest::Approx(0.25).epsilon(1e-6));
            }
        }
    }
    CHECK(arcs >= 2);

    // THE PAIR, in one place: the name answers at 17/18 while the old
    // address answers the pentagon's closing vertical — the visible
    // pair IS the proof of the unit (the counterproof's own gesture).
    CHECK(b::query_edge(*pent.result, 17).name == picked.name);
    CHECK(b::query_edge(*pent.result, 14).name.empty());

    // The report as text.
    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_resolved") == 0);
    CHECK(text.find(picked.name) != std::string::npos);
}
