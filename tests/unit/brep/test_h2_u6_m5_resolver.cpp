// tests/unit/brep/test_h2_u6_m5_resolver.cpp
//
// H2 unit 6, morso 5 — IL RESOLVER (ADR-0036 D3 + D6.4, the accepted
// act): the four classes through mutation, the two query surfaces.
// The machine that answers "cosa succede al tuo riferimento dopo
// l'edit" — with classes NAMED, never silences, never coin flips.
// Every claim its own test, the naive version of each one failing
// where the naming is new:
//
//   1. CLASSE 1 — the radius grows within the class: the rim
//      re-derives identically (a STRUCTURAL fact, not a geometric
//      one) → reference_resolved, SILENT — the test asserts the
//      silence itself (no log line, the downstream feature re-runs
//      without reselection, the new geometry visible at the resolved
//      address, the seam-wired PAIR answering to the one name).
//   2. CLASSE 3 — the circle re-authored as a chain of two arcs:
//      the ONE wall face becomes lat(0) + lat(1) →
//      reference_ambiguous NOMINATING lat(0) E lat(1) — the KIND and
//      the PAYLOAD pinned (the candidates as ShapeIds, structural,
//      and their renders), never a coin flip. The naive positional
//      match would resolve the new lat(0) — the mis-reference — and
//      the roots rule is the door that stops it.
//   3. CLASSE 4 — the circle replaced by a square chain: the rim
//      CIRCLE has no descendant (the sketch's authored entity was
//      replaced; the new contour curves read honestly unnamed) →
//      reference_vanished NOMINATING the dead rim (the row carried
//      whole), the CAUSE ("the authored entity was replaced") and
//      the remedy.
//   4. CLASSE 4, THE TAVOLA'S OWN CAUSE — a reference to the tool's
//      cap (dead in the old generation's own operation, the vanished
//      row on the tavola) → reference_vanished with the ROW'S cause
//      verbatim: "cut away outside the target" — D3's own example,
//      the payload reading the tavola, not inventing.
//   5. CLASSE 2, LA MACHINERIE — the synthetic exerciser: the old
//      row → one living descendant with a different name, THE MAP
//      DECLARED (the edit path's own emission, synthetically stood
//      in) → reference_renamed with from/to and the map rendered.
//      THE REGISTER NOTE: v1 authored flows produce no instance —
//      registered boundary (ADR-0036 D3.2): the add-at-end edits
//      never shift a role, and the shifting edits arrive with the
//      real maps. A class with machinery and no v1 instance is a
//      registered boundary, not a promise — this test is the
//      machinery's own pin, not an instance's.
//   6. LA GUARDIA — the reference with NO ROW: the never-existed
//      path AND the injected defect (a row surgically removed from
//      the tavola — the closure violation demonstrated, not
//      asserted) → closure_defect, the machine calling it a DEFECT
//      ("a closure violation", "not a user error"), never a "not
//      found" wearing an error's clothes.
//   7. LA REGOLA DELLE RADICI — the edit that shifts roots WITHOUT a
//      map: the door is CLOSED. The same [seg, arc] -> [seg, seg,
//      seg, arc] edit that class 2's map follows honestly: without
//      the map the resolution is the HONEST class (ambiguous, all
//      four walls named — never the new lat(3) answering alone for
//      the old lat(1), never the positional lat(1)->lat(1) either),
//      and the benign radius edit itself needs the edit path's
//      declaration — the resolver never assumes the edit's class.
//   8. NESSUNA STRINGA PARSED, STRUCTURAL EVERYWHERE — the morse-2
//      compile probes extended to the resolver's own types (no
//      from_string, no parser, no string comparator); the payloads
//      compared as ShapeIds (the structure), the renders pinned only
//      where the render IS the claim (the error texts the user
//      reads).
//   9. LE DUE SUPERFICI — the query-by-name (the structural name →
//      the materializing ADDRESSES: the seam-wired pair answers
//      BOTH halves, the loud miss refuses) beside the untouched
//      positional query (the pick's address → the name at pick
//      time); the four CODES + the guard's token, one switch, the
//      family registered for the fillet (ADR-0036 D6).
//
// THE ORACLES: the structural comparisons (ShapeId's own operators)
// for every identity claim — the names are pinned only where the
// payload's own words are the pin (the class-3 candidates' renders,
// the class-4 cause, the codes); the resolution_text as the report
// the UI and the feature's log read. Determinism trivially honored
// (ADR-0014: zero floating-point content in the resolution's own
// data).
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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// THE NEGATIVE SURFACE (family 8) — the morse-2 probes, extended to
// the resolver's own types: each probe is a dependent
// requires-expression, so a from_string constructor or a name parser
// added to the headers tomorrow makes the static_assert FIRE.
// ---------------------------------------------------------------------------

template <typename T>
concept HasFromString =
    requires (const T& v) { v.from_string(std::string{}); };
static_assert(!HasFromString<CAD_0::brep::Resolution>,
              "ADR-0036 D1/D3: no constructor from a name string on "
              "the resolution — the reference is the pick's own "
              "structural answer, and the render is one direction");
static_assert(!HasFromString<CAD_0::brep::EditContext>,
              "ADR-0036 D4: the edit context is the edit path's own "
              "emission (the declaration, the map) — never a parsed "
              "string");
static_assert(!HasFromString<CAD_0::brep::Generation>,
              "ADR-0036 D3: the generation is the materialized state "
              "(the body, the tavola, the grammar) — never a parsed "
              "string");

// Free-function probes, lexically INSIDE the namespace so the
// unqualified lookup consults the resolver's own declarations.
namespace CAD_0::brep {
template <typename... A>
concept HasParseResolution =
    requires (A&&... a) { parse_resolution(std::forward<A>(a)...); };
static_assert(!HasParseResolution<std::string>,
              "ADR-0036 D3: no resolution parser exists in the public "
              "surface — the kernel never parses strings");
template <typename... A>
concept HasResolveFromString =
    requires (A&&... a) { resolve_from_string(std::forward<A>(a)...); };
static_assert(!HasResolveFromString<std::string>,
              "ADR-0036 D3: resolve() takes the structural ShapeId — "
              "there is no from-string leg, anywhere");
template <typename... A>
concept HasNameEquals =
    requires (A&&... a) { name_equals(std::forward<A>(a)...); };
static_assert(!HasNameEquals<std::string, std::string>,
              "ADR-0036 D1: no string comparator exists — equality is "
              "structural (ShapeId's own operators), the resolver "
              "included");
} // namespace CAD_0::brep

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

// ---------------------------------------------------------------------------
// The recipes (unit 4's own helpers, extended with morse 5's edit
// scenarios — the mutations the classes ride on)
// ---------------------------------------------------------------------------

// The guide case's tool recipe: the circle at the origin, r (the
// ADR-0032 D7.5 body — the radius is the EDIT KNOB of family 1).
std::vector<s::SketchChain> circle_chains(float radius = 0.25f) {
    return {s::SketchChain{
        {s::sketch_circle(Vec2f{0.0f, 0.0f}, radius, false)}}};
}

// THE TWO-ARC CHAIN (family 2, D5.2's own scenario): the circle
// re-authored as a chain of TWO ARCS — the lens (two circles
// crossing at both endpoints, the unit-3 test's own class: the
// same-circle split is refused by the certificate — two coincident
// circles cross at EVERY point — so the honest two-arc chain is the
// lens). The scenario's own point holds: the ONE wall face of the
// circle tool becomes lat(0) + lat(1) — the authored entity at
// root 0 REPLACED by two new ones, the split the ambiguity rides
// on.
std::vector<s::SketchChain> two_arc_chains() {
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

// THE SQUARE CHAIN (family 3, D5.2's own scenario): the circle
// replaced by a square of four segments — the authored entity gone,
// the new contour curves outside the born-edge vocabulary (honestly
// unnamed, the registered boundary).
std::vector<s::SketchChain> square_chains(float half = 0.2f) {
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-half, -half}, Vec2f{half, -half}, true),
        s::sketch_segment(Vec2f{half, -half}, Vec2f{half, half}, true),
        s::sketch_segment(Vec2f{half, half}, Vec2f{-half, half}, true),
        s::sketch_segment(Vec2f{-half, half}, Vec2f{-half, -half}, true),
    }}};
}

// THE "D" PROFILE (families 5 and 7): the diameter segment + the
// upper half-arc — the arc at ROOT 1, the entity the root map (or
// its absence) is about.
std::vector<s::SketchChain> d_chains() {
    const float pi = 3.14159265358979323846f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-1.0f, 0.0f}, Vec2f{1.0f, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, 1.0f, false, 0.0f, pi),
    }}};
}

// THE SPLIT "D" (families 5 and 7's edit): two segments inserted
// BEFORE the arc — the diameter split in three, the SAME arc now at
// ROOT 3 (the shift: 1 -> 3).
std::vector<s::SketchChain> d_split_chains() {
    const float pi = 3.14159265358979323846f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-1.0f, 0.0f}, Vec2f{-1.0f / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{-1.0f / 3.0f, 0.0f}, Vec2f{1.0f / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{1.0f / 3.0f, 0.0f}, Vec2f{1.0f, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, 1.0f, false, 0.0f, pi),
    }}};
}

// ---------------------------------------------------------------------------
// The D1 chain (the ADR's own FeatureIds: box#1 -> sk#2 -> ext#3 ->
// sub#4) — the m4 pattern, the tool recipe the EDIT KNOB
// ---------------------------------------------------------------------------

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

// The subtract's INSTANCE registry (the m4 SubRegistry pattern,
// verbatim): the operand bindings the stamping site held — the
// reference's grammar for the relational names.
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
};

// The generations the resolver reads: the old side (the tavola +
// the grammar) and the new side (the body + the grammar). The
// grammar is the REFERENCE's own — the subtract's instance registry
// for its born entities (the rims, the seam), the ORIGINAL creator's
// for an inherited one (the wall is the EXTRUDE's face; the caller
// holds the declaration, it made the reference).
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

// The downstream feature's STAND-IN (family 1's own shape): holds
// the reference BY NAME (the structural ShapeId, picked once),
// re-runs on the resolution. THE SILENCE IS ITS OWN PROTOCOL: the
// success is not logged (the feature re-runs silently, as it
// should); every other class writes the log — the dangling rule
// extended from feature granularity (scene.hpp:271-274) to ENTITY
// granularity, the resolution's classes its own words.
struct DownstreamFeature {
    ShapeId reference{};
    std::vector<std::string> log{};
    bool ran{false};

    bool re_run(const b::Generation& old_gen,
                const b::Generation& new_gen,
                const b::EditContext& edit_ctx) {
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

} // namespace

// ===========================================================================
// FAMILY 1 — CLASSE 1: the radius grows within the class →
// reference_resolved, SILENT (the silence itself asserted)
// ===========================================================================

TEST_CASE("H2-U6-M5 class 1: the radius edit — the rim re-derives "
          "identically, reference_resolved, SILENT") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(circle_chains(0.30f));
    REQUIRE(old_g.result->edge_count() == 17);   // the pinned counts
    REQUIRE(old_g.result->face_count() == 7);    // (12/17/7/2)
    REQUIRE(new_g.result->edge_count() == 17);   // the class held:
    REQUIRE(new_g.result->face_count() == 7);    // the topology kept

    const SubRegistry reg{old_g};
    const Generations gens =
        subtract_generations(old_g, new_g, &reg.naming);

    // THE PICK: the reference is the rim's own structural answer —
    // the shape_id the positional query carries beside the name (the
    // click's address -> the name, at pick time).
    const b::EdgeInfo picked = b::query_edge(*old_g.result, 14);
    REQUIRE(picked.name ==
            "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    const ShapeId reference = picked.id;
    REQUIRE(reference.feature == FeatureId{4});
    REQUIRE(reference.subshape.depth == 4);

    // THE EDIT CONTEXT the edit path emits for the covered flow (the
    // parameter edit: the roots kept their positions — the
    // declaration is the edit path's own contract, the resolver
    // consumes it, never invents it).
    const b::EditContext edit_ctx{/*roots_no_shift=*/true, {}};

    // THE RESOLUTION: class 1 — the name re-derived identically (a
    // STRUCTURAL fact: the rim moved in space, its genealogy path is
    // untouched by the radius).
    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);
    REQUIRE(r.kind == b::Resolution::Kind::Resolved);
    CHECK(b::resolution_code(r.kind) == std::string("reference_resolved"));
    // The payload: the re-derived identity — the SAME structure
    // (structural equality, never the render), and its render.
    CHECK(r.resolved.id.feature == reference.feature);
    CHECK(r.resolved.id.subshape == reference.subshape);
    CHECK(r.resolved.name == picked.name);

    // THE SILENCE, asserted: the downstream feature re-runs with NO
    // log line — the success is not an event (D3's own words: "the
    // feature re-runs, silently, as it should").
    DownstreamFeature fillet;
    fillet.reference = reference;
    CHECK(fillet.re_run(gens.old_gen, gens.new_gen, edit_ctx));
    CHECK(fillet.ran);
    CHECK(fillet.log.empty());

    // THE FEATURE RE-RUNS ON THE NEW GEOMETRY, without reselection:
    // the resolved name -> the materializing addresses (the by-name
    // query surface) — the seam-wired PAIR answers to the ONE name
    // (both halves, ADR-0023: the pair is the materialized form) —
    // and the new radius is visible right there (the pierced face's
    // inner loop, the fillet's own input curve).
    const std::vector<b::EdgeId> ids =
        b::query_edge_by_name(*new_g.result, r.resolved.id);
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
    CHECK(arcs >= 2);  // the rim halves — the NEW radius, live

    // The positional query stays intact — two planes, two doors
    // (D1): the address answers the address's own question.
    CHECK(b::query_edge(*new_g.result, 14).name == picked.name);

    // The report as text (the payload's own words, pinned where the
    // words ARE the claim).
    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_resolved") == 0);
    CHECK(text.find(picked.name) != std::string::npos);
}

// ===========================================================================
// FAMILY 2 — CLASSE 3: the two-arc chain → reference_ambiguous
// NOMINATING lat(0) E lat(1), the remedy with them
// ===========================================================================

TEST_CASE("H2-U6-M5 class 3: the circle re-authored as two arcs — "
          "the wall reference is AMBIGUOUS, lat(0) E lat(1) nominated") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(two_arc_chains());
    // The edit re-shaped the tool: ONE wall face became TWO (the
    // formula's own arithmetic: n = 1 -> 2, F = 6+n).
    REQUIRE(new_g.result->face_count() == 8);
    REQUIRE(new_g.result->edge_count() == 18);

    // The wall's grammar is the EXTRUDE's — the ORIGINAL creator's
    // (an inherited entity: the absorbed face keeps its birth name,
    // and the birth name's tables are the extrude's own walk roles;
    // the subtract's face table is honestly EMPTY — it births no
    // faces, the morse-4 register's own derivation). The SubRegistry
    // is not built here: the wall's reference never walks the
    // subtract's legs.
    const Generations gens =
        subtract_generations(old_g, new_g, &n::kExtNaming);

    // The picked reference: the wall (the tool's ONE lateral, the
    // absorbed face that keeps its birth name ext#3:lat(0)).
    const b::FaceInfo picked = b::query_face(*old_g.result, 6);
    REQUIRE(picked.name == "ext#3:lat(0)");
    const ShapeId reference = picked.id;

    // The edit context: EMPTY — the re-authoring is not a covered
    // flow (no no-shift declaration) and emits no map (v1 has none):
    // the roots moved, and the door is closed by contract.
    const b::EditContext edit_ctx{};

    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);

    // THE KIND and THE PAYLOAD, not just the failure: the reference
    // does NOT guess — every candidate nominated, by identity (the
    // structural ShapeIds) and by name (the renders).
    REQUIRE(r.kind == b::Resolution::Kind::Ambiguous);
    CHECK(b::resolution_code(r.kind) ==
          std::string("reference_ambiguous"));
    REQUIRE(r.ambiguous.candidates.size() == 2);
    REQUIRE(r.ambiguous.candidate_names.size() == 2);
    // The candidates AS STRUCTURE first (equality is structural,
    // ShapeId's own operators — the renders pinned after, where the
    // words are the claim): ext#3:lat(0) and ext#3:lat(1).
    ShapeId want0{FeatureId{3}, SubshapeRef{}.push(0).push(0), {}};
    ShapeId want1{FeatureId{3}, SubshapeRef{}.push(0).push(1), {}};
    CHECK(r.ambiguous.candidates[0].feature == want0.feature);
    CHECK(r.ambiguous.candidates[0].subshape == want0.subshape);
    CHECK(r.ambiguous.candidates[1].feature == want1.feature);
    CHECK(r.ambiguous.candidates[1].subshape == want1.subshape);
    CHECK(r.ambiguous.candidate_names[0] == "ext#3:lat(0)");
    CHECK(r.ambiguous.candidate_names[1] == "ext#3:lat(1)");
    // The entity named (the reference's own name, its row's) and the
    // remedy with the candidates.
    CHECK(r.ambiguous.entity_name == "ext#3:lat(0)");
    CHECK(r.ambiguous.remedy.find("reselect") != std::string::npos);

    // The naive positional match WOULD have resolved: the new body
    // carries a wall stamped exactly [lat, 0] (the first ARC's — a
    // different authored entity). The roots rule is the door that
    // stops it: no declaration, no map, no positional match — the
    // honest class, never the mis-reference.
    const b::FaceInfo naive =
        b::query_face(*new_g.result, 6);
    REQUIRE(naive.name == "ext#3:lat(0)");
    CHECK(naive.id.subshape == reference.subshape);

    // The report as text: the candidates in it, verbatim.
    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_ambiguous") == 0);
    CHECK(text.find("ext#3:lat(0)") != std::string::npos);
    CHECK(text.find("ext#3:lat(1)") != std::string::npos);
    CHECK(text.find("reselect") != std::string::npos);
}

// ===========================================================================
// FAMILY 3 — CLASSE 4: the square chain → reference_vanished, the
// dead rim named, the cause and the remedy with it
// ===========================================================================

TEST_CASE("H2-U6-M5 class 4: the circle replaced by a square chain — "
          "the rim reference is VANISHED, the cause and remedy named") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(square_chains(0.2f));
    // The square tool: n = 4 (the formula: V = 8+8, E = 12+12,
    // F = 6+4) — the walls exist and are NAMED; the born contour
    // curves read honestly EMPTY (the registered boundary: the
    // segment walls' curves are Phase D's SSI entry).
    REQUIRE(new_g.result->face_count() == 10);
    REQUIRE(new_g.result->edge_count() == 24);
    for (b::EdgeId i = 12; i < 24; ++i)
        CHECK(b::query_edge(*new_g.result, i).name.empty());

    const SubRegistry reg{old_g};
    const Generations gens =
        subtract_generations(old_g, new_g, &reg.naming);

    // The picked reference: the rim (the fillet's own string).
    const b::EdgeInfo picked = b::query_edge(*old_g.result, 14);
    REQUIRE(picked.name ==
            "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    const ShapeId reference = picked.id;

    const b::EditContext edit_ctx{};  // the replacement: no map

    const b::Resolution r =
        b::resolve(reference, gens.old_gen, gens.new_gen, edit_ctx);

    // THE KIND and THE PAYLOAD: what died (the row carried whole —
    // the subject's name and address on it), WHY (the authored
    // entity was replaced: the sketch's circle is gone, and the rim
    // died with it — the new contour curves are born of new
    // entities), and the remedy. Never a mute "not found".
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
// FAMILY 4 — CLASSE 4, the tavola's own cause: the reference to the
// tool's dead cap reads the vanished row's cause verbatim
// ===========================================================================

TEST_CASE("H2-U6-M5 class 4, the tavola's cause: the tool's cap — "
          "'cut away outside the target', the row's own words") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(circle_chains(0.30f));

    // The picked reference: the tool's bottom cap — an entity of
    // EXT#3's output, alive there, DEAD in sub#4's (cut away outside
    // the target). The tavola's vanished tail books it.
    const b::FaceInfo picked = b::query_face(*old_g.tool, 1);
    REQUIRE(picked.name == "ext#3:cap(lo)");
    const ShapeId reference = picked.id;
    REQUIRE(reference.feature == FeatureId{3});

    // The generations: sub#4's (the reference resolves against the
    // feature's output that consumed its home body); the grammar is
    // the EXTRUDE's — the cap's own creator, the caller holds it.
    b::DescentTable tavola =
        b::subtract_descent(*old_g.result, *old_g.box, *old_g.tool);
    const b::Generation old_gen{old_g.result.get(), &tavola,
                                &n::kExtNaming, false};
    const b::Generation new_gen{new_g.result.get(), nullptr,
                                &n::kExtNaming, false};

    const b::Resolution r =
        b::resolve(reference, old_gen, new_gen, b::EditContext{});

    // Class 4 through the TAVOLA'S OWN ROW: the guard finds the
    // vanished row (the dead entity's identity, stamped on it since
    // the morse-5 register), and the payload carries the row's cause
    // VERBATIM — D3's own example, the machine reading the record,
    // not inventing a diagnosis.
    REQUIRE(r.kind == b::Resolution::Kind::Vanished);
    CHECK(r.vanished.row.relation == b::Descent::Vanished);
    CHECK(r.vanished.row.name == "ext#3:cap(lo)");
    CHECK(r.vanished.why == "cut away outside the target");
    CHECK(r.vanished.remedy.find("survives the operation") !=
          std::string::npos);

    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_vanished") == 0);
    CHECK(text.find("cut away outside the target") !=
          std::string::npos);
}

// ===========================================================================
// FAMILY 5 — CLASSE 2, LA MACHINERIE: the synthetic exerciser, the
// map declared
// ===========================================================================

TEST_CASE("H2-U6-M5 class 2, the machinery: the map declared — one "
          "living descendant, a different name, RENAMED") {
    // THE REGISTER NOTE, in the act itself: v1 authored flows
    // produce no instance — registered boundary (ADR-0036 D3.2).
    // The add-at-end edits never shift a role; the shifting edits
    // (the middle insert/deletion family) arrive with the REAL maps,
    // and they bring this class its first consumer. This test
    // exercises the MACHINERY synthetically — the old row, the
    // declared map, the one descendant — not an authored instance:
    // a class with machinery and no v1 instance is a registered
    // boundary, not a promise.

    // The old generation: the "D" extrude (the diameter segment +
    // the arc, the arc at ROOT 1) — ext#3's own output.
    const auto old_chains = d_chains();
    const auto old_tool =
        b::make_prism(old_chains, 2.0f, +1, FeatureId{3});
    REQUIRE(old_tool->face_count() == 4);  // 2 laterals + 2 caps

    // The picked reference: the ARC's wall — lat(1).
    const b::FaceInfo picked = b::query_face(*old_tool, 1);
    REQUIRE(picked.name == "ext#3:lat(1)");
    const ShapeId reference = picked.id;

    // The new generation: the same "D", the diameter split in three
    // — the SAME arc now at ROOT 3 (the shift: 1 -> 3).
    const auto new_chains = d_split_chains();
    const auto new_tool =
        b::make_prism(new_chains, 2.0f, +1, FeatureId{3});
    REQUIRE(new_tool->face_count() == 6);  // 4 laterals + 2 caps
    REQUIRE(b::query_face(*new_tool, 3).name == "ext#3:lat(3)");

    b::DescentTable old_tavola = b::extrude_descent(
        *old_tool, old_chains, FeatureId{3}, FeatureId{2});
    const b::Generation old_gen{old_tool.get(), &old_tavola,
                                &n::kExtNaming, false};
    const b::Generation new_gen{new_tool.get(), nullptr,
                                &n::kExtNaming, false};

    // THE MAP, declared by the edit path (synthetically stood in):
    // root (chain 0, entity 1) -> root (chain 0, entity 3) — the
    // arc's own survival, the edit path's knowledge, emitted per
    // D4's amendment ("the edit path that shifts referenced roots
    // EMITS the root map or REFUSES with the remedy").
    b::EditContext edit_ctx;
    edit_ctx.root_map.push_back(
        {n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 1},
         n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 3}});

    const b::Resolution r =
        b::resolve(reference, old_gen, new_gen, edit_ctx);

    // THE KIND and THE PAYLOAD: from -> to, the map declared, the
    // feature's log writes it — never silence.
    REQUIRE(r.kind == b::Resolution::Kind::Renamed);
    CHECK(b::resolution_code(r.kind) ==
          std::string("reference_renamed"));
    CHECK(r.renamed.from.feature == reference.feature);
    CHECK(r.renamed.from.subshape == reference.subshape);
    ShapeId want_to{FeatureId{3}, SubshapeRef{}.push(0).push(3), {}};
    CHECK(r.renamed.to.feature == want_to.feature);
    CHECK(r.renamed.to.subshape == want_to.subshape);
    CHECK(r.renamed.from_name == "ext#3:lat(1)");
    CHECK(r.renamed.to_name == "ext#3:lat(3)");
    // The declared map, rendered — the log's own text.
    CHECK(r.renamed.map == "sk#2:1 -> sk#2:3");

    // The renamed reference ANSWERS on the new body (the by-name
    // surface, the feature's own next step).
    const std::vector<b::FaceId> ids =
        b::query_face_by_name(*new_tool, r.renamed.to);
    REQUIRE(ids.size() == 1);
    CHECK(ids[0] == 3);

    const std::string text = b::resolution_text(r);
    CHECK(text.find("reference_renamed") == 0);
    CHECK(text.find("ext#3:lat(1) -> ext#3:lat(3)") !=
          std::string::npos);
    CHECK(text.find("sk#2:1 -> sk#2:3") != std::string::npos);
    CHECK(text.find("never silence") != std::string::npos);
}

// ===========================================================================
// FAMILY 6 — LA GUARDIA: the reference with NO ROW is a closure
// defect, surfaced as the defect it is
// ===========================================================================

TEST_CASE("H2-U6-M5 the guard: the reference with no row — "
          "closure_defect, a defect named, never a 'not found'") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(circle_chains(0.30f));
    const SubRegistry reg{old_g};
    const Generations gens =
        subtract_generations(old_g, new_g, &reg.naming);

    // (a) The never-existed reference: a path no row carries.
    const ShapeId ghost{
        FeatureId{4}, SubshapeRef{}.push(1).push(1).push(7).push(7),
        {}};
    const b::Resolution r =
        b::resolve(ghost, gens.old_gen, gens.new_gen,
                   b::EditContext{});
    REQUIRE(r.kind == b::Resolution::Kind::ClosureDefect);
    CHECK(b::resolution_code(r.kind) ==
          std::string("closure_defect"));
    CHECK(r.closure_defect.orphan.feature == ghost.feature);
    CHECK(r.closure_defect.orphan.subshape == ghost.subshape);
    // The DEFECT's own words — not a user error, never silence.
    const std::string text = b::resolution_text(r);
    CHECK(text.find("closure_defect") == 0);
    CHECK(text.find("closure violation") != std::string::npos);
    CHECK(text.find("not a user error") != std::string::npos);

    // (b) THE INJECTED DEFECT: the rim's rows surgically removed
    // from the tavola — the closure violation demonstrated, not
    // asserted. The reference (the rim, picked below) HAD its rows
    // (TWO of them: the seam-wired pair shares the one identity,
    // one row per materialized half); the tavola lost them both;
    // the machine calls the defect.
    b::DescentTable torn = gens.old_tavola;
    const ShapeId rim = b::query_edge(*old_g.result, 14).id;
    std::size_t removed = 0;
    for (std::size_t i = 0; i < torn.rows.size();) {
        if (torn.rows[i].shape_id.feature == rim.feature &&
            torn.rows[i].shape_id.subshape == rim.subshape) {
            torn.rows.erase(torn.rows.begin() +
                            static_cast<std::ptrdiff_t>(i));
            ++removed;
        } else {
            ++i;
        }
    }
    REQUIRE(removed == 2);  // the pair's rows, both
    const b::Generation torn_gen{old_g.result.get(), &torn,
                                 &reg.naming, false};
    const b::Resolution r2 = b::resolve(
        rim, torn_gen, gens.new_gen, b::EditContext{});
    REQUIRE(r2.kind == b::Resolution::Kind::ClosureDefect);
    CHECK(r2.closure_defect.orphan.subshape == rim.subshape);
    CHECK(b::resolution_text(r2).find("closure violation") !=
          std::string::npos);
}

// ===========================================================================
// FAMILY 7 — LA REGOLA DELLE RADICI: the shift WITHOUT a map never
// produces a positional match — the door is closed
// ===========================================================================

TEST_CASE("H2-U6-M5 the roots rule: the shift without a map — the "
          "honest class, never the new lat answering for the old") {
    // The same edit as family 5 ([seg, arc] -> [seg, seg, seg, arc],
    // the arc's root 1 -> 3) — WITHOUT the map this time: the edit
    // path emitted nothing, and the naked positional match across a
    // generation is the mis-reference's own door, CLOSED by
    // contract.
    const auto old_chains = d_chains();
    const auto old_tool =
        b::make_prism(old_chains, 2.0f, +1, FeatureId{3});
    const auto new_chains = d_split_chains();
    const auto new_tool =
        b::make_prism(new_chains, 2.0f, +1, FeatureId{3});

    const b::FaceInfo picked = b::query_face(*old_tool, 1);
    REQUIRE(picked.name == "ext#3:lat(1)");
    const ShapeId reference = picked.id;

    b::DescentTable old_tavola = b::extrude_descent(
        *old_tool, old_chains, FeatureId{3}, FeatureId{2});
    const b::Generation old_gen{old_tool.get(), &old_tavola,
                                &n::kExtNaming, false};
    const b::Generation new_gen{new_tool.get(), nullptr,
                                &n::kExtNaming, false};

    // NO declaration, NO map: the honest classes only.
    const b::Resolution r =
        b::resolve(reference, old_gen, new_gen, b::EditContext{});

    // NEVER Resolved (the naive positional match would answer the
    // new lat(1) — the second segment's wall, a DIFFERENT authored
    // entity), NEVER Renamed (the smart list-diff would answer the
    // new lat(3) — the arc's wall, a GUESS): the reference does not
    // guess, and the honest class names ALL the candidates.
    REQUIRE(r.kind == b::Resolution::Kind::Ambiguous);
    CHECK(r.kind != b::Resolution::Kind::Resolved);
    CHECK(r.kind != b::Resolution::Kind::Renamed);
    REQUIRE(r.ambiguous.candidates.size() == 4);
    REQUIRE(r.ambiguous.candidate_names.size() == 4);
    for (std::uint32_t i = 0; i < 4; ++i) {
        CHECK(r.ambiguous.candidate_names[i] ==
              "ext#3:lat(" + std::to_string(i) + ")");
    }
    // The specific counterproofs, named: neither the positional
    // lat(1) nor the diffed lat(3) was silently followed.
    CHECK(r.resolved.id == ShapeId{});   // the payloads are inert
    CHECK(r.renamed.to == ShapeId{});    // outside their own class

    // THE DOOR, the other side: even the BENIGN edit (the radius —
    // family 1's own, where the name DOES re-derive) needs the edit
    // path's declaration. Without it, the resolver does not assume
    // the edit's class — the naked structural match on a
    // root-dependent name never happens.
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const GuideCase new_g = guide_case(circle_chains(0.30f));
    const SubRegistry reg{old_g};
    const Generations gens =
        subtract_generations(old_g, new_g, &reg.naming);
    const ShapeId rim = b::query_edge(*old_g.result, 14).id;
    const b::Resolution r2 =
        b::resolve(rim, gens.old_gen, gens.new_gen,
                   b::EditContext{});
    CHECK(r2.kind == b::Resolution::Kind::Ambiguous);
    REQUIRE(r2.ambiguous.candidates.size() == 1);
    CHECK(r2.ambiguous.candidate_names[0] ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    // ... and WITH the declaration (family 1's own pin): Resolved,
    // silent. The declaration is the difference — the edit path's
    // contract, consumed, never invented.
}

// ===========================================================================
// FAMILY 8 — LE DUE SUPERFICI + I CODICI: the by-name query beside
// the positional one, the four words + the guard's token
// ===========================================================================

TEST_CASE("H2-U6-M5 the two surfaces and the codes: the by-name "
          "queries, the seam-wired pair, the loud miss, the family") {
    const GuideCase g = guide_case(circle_chains(0.25f));

    // THE QUERY-BY-NAME (the name plane's own door): the structural
    // name -> the materializing addresses. The rim: the seam-wired
    // PAIR (one logical entity, both halves answering — ADR-0023).
    const ShapeId rim = b::query_edge(*g.result, 14).id;
    const ShapeId seam = b::query_edge(*g.result, 16).id;
    const ShapeId wall = b::query_face(*g.result, 6).id;
    const std::vector<b::EdgeId> rim_ids =
        b::query_edge_by_name(*g.result, rim);
    REQUIRE(rim_ids.size() == 2);
    CHECK(rim_ids[0] == 14);
    CHECK(rim_ids[1] == 15);
    const std::vector<b::EdgeId> seam_ids =
        b::query_edge_by_name(*g.result, seam);
    REQUIRE(seam_ids.size() == 1);
    CHECK(seam_ids[0] == 16);
    const std::vector<b::FaceId> wall_ids =
        b::query_face_by_name(*g.result, wall);
    REQUIRE(wall_ids.size() == 1);
    CHECK(wall_ids[0] == 6);

    // The PICK surface (the other door, intact): the address -> the
    // name at pick time — the positional query's own name field.
    CHECK(b::query_edge(*g.result, rim_ids[0]).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_face(*g.result, wall_ids[0]).name == "ext#3:lat(0)");

    // THE LOUD MISS (the P2 culture): a name this body does not
    // carry refuses with the diagnosis and the remedy — never an
    // empty answer.
    const ShapeId foreign{FeatureId{9}, rim.subshape, {}};
    CHECK_THROWS_AS(b::query_edge_by_name(*g.result, foreign),
                    std::invalid_argument);
    bool refused = false;
    try {
        (void)b::query_face_by_name(*g.result, foreign);
    } catch (const std::invalid_argument& e) {
        refused = true;
        CHECK(std::string(e.what()).find("remedy") !=
              std::string::npos);
    }
    CHECK(refused);

    // THE CODES — the family registered for the fillet (ADR-0036
    // D6: G2's error culture inherits these four words), the
    // guard's own token outside it.
    CHECK(std::string(b::resolution_code(
              b::Resolution::Kind::Resolved)) == "reference_resolved");
    CHECK(std::string(b::resolution_code(
              b::Resolution::Kind::Renamed)) == "reference_renamed");
    CHECK(std::string(b::resolution_code(
              b::Resolution::Kind::Ambiguous)) ==
          "reference_ambiguous");
    CHECK(std::string(b::resolution_code(
              b::Resolution::Kind::Vanished)) ==
          "reference_vanished");
    CHECK(std::string(b::resolution_code(
              b::Resolution::Kind::ClosureDefect)) == "closure_defect");
}

// ===========================================================================
// FAMILY 9 — THE INPUT CONTRACT, loud: the resolver refuses its own
// misuse, the P2 culture (never a guess, never a crash)
// ===========================================================================

TEST_CASE("H2-U6-M5 the input contract: the missing tavola, body or "
          "grammar refuse loudly, diagnosis + remedy") {
    const GuideCase old_g = guide_case(circle_chains(0.25f));
    const ShapeId rim = b::query_edge(*old_g.result, 14).id;
    b::DescentTable tavola =
        b::subtract_descent(*old_g.result, *old_g.box, *old_g.tool);
    const SubRegistry reg{old_g};

    const b::Generation no_tavola{old_g.result.get(), nullptr,
                                  &reg.naming, false};
    CHECK_THROWS_AS(
        b::resolve(rim, no_tavola, b::Generation{old_g.result.get(),
                                                 nullptr, &reg.naming,
                                                 false},
                   b::EditContext{}),
        std::invalid_argument);

    const b::Generation no_body{nullptr, &tavola, &reg.naming, false};
    CHECK_THROWS_AS(
        b::resolve(rim,
                   b::Generation{old_g.result.get(), &tavola,
                                 &reg.naming, false},
                   no_body, b::EditContext{}),
        std::invalid_argument);

    const b::Generation no_grammar{old_g.result.get(), &tavola,
                                   nullptr, false};
    CHECK_THROWS_AS(
        b::resolve(rim, no_grammar,
                   b::Generation{old_g.result.get(), nullptr,
                                 &reg.naming, false},
                   b::EditContext{}),
        std::invalid_argument);

    // A FOREIGN GRAMMAR (the box's tables for the subtract's rim)
    // refuses loudly too: the walk cannot see the relational legs
    // under a foreign registry — never a silent wrong answer.
    const b::Generation foreign{old_g.result.get(), &tavola,
                                &n::kBoxNaming, false};
    CHECK_THROWS_AS(
        b::resolve(rim, foreign,
                   b::Generation{old_g.result.get(), nullptr,
                                 &n::kBoxNaming, false},
                   b::EditContext{}),
        std::invalid_argument);
}
