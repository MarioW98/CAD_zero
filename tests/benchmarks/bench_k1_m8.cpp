// tests/benchmarks/bench_k1_m8.cpp
//
// K1 m8 — IL BENCHMARK SUITE (ADR-0041 D5 — the benchmark suite's
// own home: the performance twin of the census — capabilities get
// columns, costs get NUMBERS — and the register's fifth column of
// measured evidence, the campaign K1's closing row).
//
// THE TWO ROWS THIS SUITE STANDS:
//
//   RIGA ZERO — THE LAG PROFILE: the perceived latency of the edit,
//   end-to-end — from the GESTURE (the parameter edit) to the
//   reconstructed body READY (the chain re-run, the names resolved,
//   the ledger updated). The m6 typed corpus is the path: the five
//   members, measured ONE BY ONE. Row zero because it is the number
//   the user feels; every other row stands behind it.
//
//   THE CHAIN GATE (D1.3's chain, TIMED): every step of the chain-5
//   measured AFTER the step — the STEP's own time, never the
//   cumulative — so the gate can say WHICH step is slow, not only
//   that something is. The chain-5 is the guide chain: box#1 ->
//   sk#2 -> ext#3 -> sub#4 -> fil#5 (m5's own body, the counts and
//   the digests already pinned there, the m5 gate run beside as the
//   correctness witness — the same door, never a second opinion).
//
// THE METHODOLOGY, DECLARED (never implied):
//   * the inputs are FIXED IN THE CODE — the m6 corpus's own
//     parameters (r 0.25 -> 0.125, the D-chain's chord and arc, the
//     m6ter radii, the loaded chain's file authored here seed by
//     seed); no time source, no unseeded random, nothing re-ordered;
//   * warmup 3, iterations 21 — declared, pinned;
//   * the central number is the MEDIAN (never the minimum, never
//     the raw mean — a median cannot be bought with one lucky
//     sample nor dragged by one unlucky tail), p95 beside it
//     (nearest-rank);
//   * the measuring machine is declared in the REPORT and on the
//     first printed line of the run, never in a threshold: the CI
//     runner is not the maintainer's machine — the absolute numbers
//     are indicative, THE RATIOS ARE THE SUBSTANCE.
//
// THE THRESHOLDS, RELATIVE ONLY (the discipline's own law): the
// gate's red is a WITHIN-RUN A/B regression — pass B's median over
// pass A's, PER STEP, red beyond 3.0x (declared), the ratio's
// denominator floored at 1 us (declared, so a nanosecond step
// cannot counterfeit a regression) — never a second pinned on a
// machine that is not the one reading. The between-step ratios are
// the PROFILE the row carries (which step owns the time — the
// attribution), never a threshold: the work distribution is the
// algorithms' own, not a regression.
//
// THE DIGEST BEFORE THE TIMES (ADR-0014 via D5's determinism
// clause, VERBATIM): "Under the suite, every digest the tree
// already pins (the goldens' round-trips, the chain re-runs, the
// name/structure digests) is bit-identical — ADR-0014's clauses
// (`-ffp-contract=off`, the fixed chunk-then-reduce order, the
// quantizer) hold under measurement load, and no benchmark run may
// re-seed, re-order or re-thread its way to a better number. A
// performance number bought with a determinism drift is not a
// number." What is deterministic here — the inputs, the sequence
// of the operations, the DIGESTS of everything built (verified
// EVERY iteration of EVERY pass, before one time is believed: a
// benchmark that measures builds that differ among themselves
// measures nothing); what is NOT — the times, which the pinned
// verdict never carries. The boundary is the clause's own; the
// methodology declares it, never smudges it.
//
// NO NEW DEPENDENCIES: std::chrono and the repo's own structure.
// No external benchmark library — none linked, none proposed.
//
// THE COUNTERPROOF (a gate that cannot say no is a decorative
// number, not a gate):
//   * THE SLOWED STEP — sub#4 measured with a DECLARED dummy spin
//     injected into its timed region (pure arithmetic load, its
//     accumulator discarded through a volatile sink so the
//     optimizer cannot elide it, touching NOTHING the digests
//     read), calibrated at runtime to burn >= 4x the step's honest
//     median: the gate goes RED with the NAME of the slowed step,
//     the other four green beside it (the controls);
//   * THE DIGEST DRIFT — a pass whose box#1 digest differs by one
//     character: the gate REFUSES to judge the times (the digest
//     identity is the FIRST predicate — the times are not believed,
//     exactly the clause's own order).
//
// THE WITNESSES RECALLED (the two morsi that precede — their
// digests are the witnesses this suite recalls):
//   * the m6 survival ledger's pinned FNV-1a 64 (0x961ff463edcd8d63)
//     — VERIFIED here on the very corpus m6 pinned it on: the
//     determinism test's own THREE-member bundle (the silent
//     radius edit, the sketch root edit, the chained fillet
//     edit), built member per member as m6 built it, the same
//     ledger door called on it, the text's FNV required equal to
//     the m6 pin. The identity is checked, never assumed; the
//     five-member corpus (the corpus test's own — the load and
//     the recipe members beside) is checked on ITS pins, the
//     counts 9/10 with the three deaths;
//   * the m7 oracle suite's pinned FNV-1a 64 (0xfc55acb1e6b1ca14)
//     — CITED here as the corpus's truth's own witness: the oracle
//     declared the kernel's numbers true by roads sharing zero code
//     with it, and this suite measures a kernel already declared
//     true. The benchmark never re-derives the oracle — it is not
//     one (D1.5's independence respected in both directions).
//
// THE VERDICT: the DETERMINISTIC half of everything measured — the
// digests (as FNV-1a 64), the ledger's counts, the gate's
// booleans, the counterproofs' outcomes, the witnesses — rendered
// as one text, FNV-1a 64 PINNED. The times are printed in the
// report beside it and never pinned: they are the machine's own,
// the verdict is the suite's own. The bench exits 0 only when
// every check holds — the digests, the witnesses, the honest
// chain's A/B gate green, both counterproof reds, the verdict's
// FNV equal to the pin.
//
// The corpus constructions below are QUOTED from the m6 suite
// (tests/unit/brep/test_k1_m6_survival_ledger.cpp) and the m5
// baseline (test_k1_m5_chain_gate.cpp) — the benchmark measures
// the kernel through the same shapes the ledgers and the gates
// already hold: measuring a different corpus would be measuring
// nothing the register knows. The quotes are marked at each site.
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/survival_ledger.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef CAD_0_BENCH_BUILD_TYPE
#define CAD_0_BENCH_BUILD_TYPE "undeclared"
#endif

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
namespace sc = CAD_0::scene;
namespace io = CAD_0::io::native;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

// ---------------------------------------------------------------------------
// THE DECLARED CONSTANTS — the methodology, pinned where it runs.
// ---------------------------------------------------------------------------

constexpr int kWarmup = 3;        // declared warmup, discarded
constexpr int kIterations = 21;   // measured, odd -> the median is a sample
constexpr int kSabotageIterations = 7;    // the counterproof's own count
constexpr double kGateRatio = 3.0;       // the within-run A/B red line
constexpr double kGateFloorNs = 1e3;     // 1 us: the ratio's denominator floor
constexpr double kSabotageBudgetX = 4.0; // the spin burns >= 4x the honest median

// The witnesses (m6's and m7's own pins, recalled verbatim).
constexpr std::uint64_t kM6LedgerFnv = 0x961ff463edcd8d63ull;
constexpr std::uint64_t kM7OracleFnv = 0xfc55acb1e6b1ca14ull;

// The verdict's own pin — CALIBRATION NOTE (the m7 pattern): the
// constant below was read from this suite's own first clean run and
// pinned; any drift in any rendered value — digest, count, verdict,
// counterproof — moves it and the bench goes red.
constexpr std::uint64_t kM8VerdictFnv = 0x9b4cf2007b0bba70ull;

// ---------------------------------------------------------------------------
// The bench's own discipline — every failure collected, each named
// (the m5 gate's own philosophy: a check that hides the second
// failure behind the first is half a check). The hard stops are for
// the states the next line cannot survive (the m6 REQUIREs' own
// seats); everything else is collected and reported at the exit.
// ---------------------------------------------------------------------------

int g_failures = 0;

void require(bool cond, const std::string& what) {
    if (!cond) {
        std::printf("  [k1-m8-benchmark] FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

[[noreturn]] void fail_hard(const std::string& what) {
    std::printf("[k1-m8-benchmark] FATAL: %s\n", what.c_str());
    std::exit(1);
}

// FNV-1a 64 — the m5/m6/m7 suites' own instrument, quoted (the
// cross-run identity's single authority in this house).
std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string hex16(std::uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%016llx",
                  static_cast<unsigned long long>(v));
    return std::string(buf);
}

// ---------------------------------------------------------------------------
// THE TIMING HARNESS — std::chrono alone (no new dependencies): the
// steady clock (monotonic, never the wall), the median and the p95
// (nearest-rank), never the minimum, never the raw mean.
// ---------------------------------------------------------------------------

using Clock = std::chrono::steady_clock;

double elapsed_ns(Clock::time_point t0, Clock::time_point t1) {
    return std::chrono::duration<double, std::nano>(t1 - t0).count();
}

struct Stats {
    double median_ns{};
    double p95_ns{};
};

Stats stats_of(std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());
    const std::size_t count = samples.size();
    Stats st;
    st.median_ns = samples[count / 2];  // count odd (21): a real sample
    const std::size_t rank = static_cast<std::size_t>(
        std::ceil(0.95 * static_cast<double>(count)));
    st.p95_ns = samples[rank <= count ? rank - 1 : count - 1];
    return st;
}

double ms_of(double ns) { return ns * 1e-6; }

// ---------------------------------------------------------------------------
// THE CHAIN-5 — QUOTED from the m6 suite (its own Chain5, the m5
// baseline's body): box#1 -> sk#2 -> ext#3 -> sub#4 -> fil#5, every
// step's product kept, the rim and vertical2 found BY NAME.
// ---------------------------------------------------------------------------

struct Chain5 {
    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    ShapeId rim{};   // the subtract's born rim (the fillet's meal)
    ShapeId v2{};    // box#1:vertical2 — the KEPT factory edge

    explicit Chain5(double fillet_radius = 0.25) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name == "box#1:vertical2") {
                v2 = blind->edges[i].shape_id();
                break;
            }
        }
        const b::FilletNaming fil_ctx{FeatureId{5}, &n::kBoxNaming,
                                      FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, fillet_radius,
                                    fil_ctx);
    }
};

// The subtract's INSTANCE declaration — QUOTED from m6 (the rim's
// grammar: the creating operation's tables with the operands'
// bindings).
struct SubDecl {
    n::OperandBinding bindings[2]{};
    n::OperationNaming decl{};

    SubDecl() {
        bindings[0] = {FeatureId{1}, &n::kBoxNaming};
        bindings[1] = {FeatureId{3}, &n::kExtNaming};
        decl = n::OperationNaming{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles,
            {bindings, 2}};
    }
};

// THE D-CHAIN — QUOTED from m6 (member 2's own): box#1 -> sk#2
// [chord, arc] -> ext#3 -> sub#4, the tool authored CHORD-FIRST so
// the arc rides ROOT 1 (the middle insert moves it 1 -> 3).
struct DChain {
    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    ShapeId v2{};
    ShapeId wall{};  // ext#3:lat(1) — the ARC's wall, absorbed

    explicit DChain(std::vector<s::SketchChain> authored) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        chains = std::move(authored);
        tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -1.5},
                                  b::kSubtractTolerance, sub_ctx);
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            if (b::query_edge(*blind, i).name == "box#1:vertical2") {
                v2 = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->face_count(); ++i) {
            const auto f = b::query_face(*blind, i);
            if (f.name == "ext#3:lat(1)") {
                wall = f.id;
                break;
            }
        }
    }
};

std::vector<s::SketchChain> d_chords() {
    const float pi = 3.14159265358979323846f;
    const float r = 0.5f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-r, 0.0f}, Vec2f{r, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, r, false, 0.0f, pi),
    }}};
}

// THE EDIT — QUOTED from m6 (member 2's own): the middle insert,
// the chord split in three, the ARC VERBATIM at root 3.
std::vector<s::SketchChain> d_split() {
    const float pi = 3.14159265358979323846f;
    const float r = 0.5f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-r, 0.0f}, Vec2f{-r / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{-r / 3.0f, 0.0f}, Vec2f{r / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{r / 3.0f, 0.0f}, Vec2f{r, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, r, false, 0.0f, pi),
    }}};
}

// THE M6TER CHAIN — QUOTED from m6 (member 4's own): box -> fil#2
// (vertical2) -> fil#3 (vertical0), the radius edited.
struct M6ter {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;   // fil#2's result
    std::unique_ptr<b::BRepBody> result;  // fil#3's result
    ShapeId v2{};
    ShapeId v0{};

    M6ter(double r2 = 0.5, double r3 = 0.5) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        v2 = ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}};
        v0 = ShapeId{FeatureId{1}, SubshapeRef{}.push(8), {}};
        const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        first = b::fillet_edge(*box, v2, r2, ctx2);
        result = b::fillet_edge(*first, v0, r3, ctx3);
    }
};

// ---------------------------------------------------------------------------
// THE LOADED CHAIN'S WORLD — QUOTED from m6's LoadedChain (member
// 3's own): the minimal chain-5 scene boxed at the kernel's own
// FeatureIds, the reference DISCOVERED by the find (never typed by
// hand), the file SAVED (the pre-gesture world: the file on disk),
// then loaded and re-materialized ONCE for the corpus's own member
// (the m6 constructor's own walk, the digests REQUIRED equal — here
// collected). The TIMED path re-does the load per iteration.
// ---------------------------------------------------------------------------

struct LoadedChain {
    sc::Scene scene;
    std::unique_ptr<b::BRepBody> rerun_blind;
    std::unique_ptr<b::BRepBody> rerun_fil;
    std::string blind_digest;
    std::string fil_digest;
    std::string chains_digest;
    std::size_t rebuilt{0};
    std::string rim_string;
    std::filesystem::path file;

    LoadedChain() {
        {
            sc::SceneNode box_n;
            box_n.id = 1;
            box_n.name = "Box";
            box_n.builder = "box";
            box_n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                            {"extent_z", 1.0f}};
            box_n.body = s::make_box(CAD_0::math::Vec3f{1.0, 1.0, 1.0});
            scene.restore_node(std::move(box_n));
        }
        {
            std::vector<s::SketchChain> chains;
            chains.push_back({{s::sketch_circle(
                Vec2f{0.0f, 0.0f}, 0.5f, false)}});
            scene.add_sketch_node("Sketch", std::move(chains), {});
        }
        {
            sc::OperationRecipe r;
            r.type = sc::OperationType::Extrude;
            r.sketch = 2;
            r.distance = 2.0f;
            r.direction = 1;
            scene.add_operation_node("Extrude", r);
        }
        {
            sc::OperationRecipe r;
            r.type = sc::OperationType::Subtract;
            r.target = 1;
            r.tool = 3;
            r.offset_z = -0.5f;
            scene.add_operation_node("Subtract", r);
        }
        const sc::SceneNode* sub = scene.get_node(4);
        if (sub == nullptr || sub->hybrid == nullptr)
            fail_hard("the loaded world: node 4 missing at build");
        for (std::size_t i = 0; i < sub->hybrid->brep->edge_count();
             ++i) {
            const b::EdgeInfo e =
                b::query_edge(*sub->hybrid->brep, i);
            if (!e.name.empty() &&
                e.name.find("@hi") != std::string::npos &&
                e.name.find("isect") != std::string::npos) {
                rim_string = e.name;
                break;
            }
        }
        if (rim_string !=
            "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi")
            fail_hard("the loaded world: the rim string is not the "
                      "pinned baseline's");
        scene.add_fillet_node("Fillet",
                              b::FilletRecipe{rim_string, 0.25});

        const sc::SceneNode* sub2 = scene.get_node(4);
        const sc::SceneNode* fil2 = scene.get_node(5);
        if (sub2 == nullptr || sub2->hybrid == nullptr ||
            fil2 == nullptr || fil2->fillet_body == nullptr)
            fail_hard("the loaded world: the creation nodes missing");
        blind_digest = b::chain_digest(*sub2->hybrid->brep);
        fil_digest = b::chain_digest(*fil2->fillet_body);
        chains_digest =
            b::sketch_digest(scene.get_node(2)->sketch);

        file = std::filesystem::temp_directory_path() /
               "k1_m8_bench_load.CAD_0";
        io::save_file(file, scene, io::NativeCamera{});
        const io::ParsedDocument doc = io::load_file(file);

        sc::Scene re;
        io::install_document(re, io::ParsedDocument(doc));
        const auto rr = sc::rebuild_operations(re);
        rebuilt = rr.rebuilt;
        const sc::SceneNode* rsub = re.get_node(4);
        const sc::SceneNode* rfil = re.get_node(5);
        if (rsub == nullptr || rsub->hybrid == nullptr ||
            rfil == nullptr || rfil->fillet_body == nullptr)
            fail_hard("the loaded world: the re-run nodes missing");
        require(b::chain_digest(*rsub->hybrid->brep) == blind_digest,
                "the loaded world: the blind's re-run digest drift");
        require(b::chain_digest(*rfil->fillet_body) == fil_digest,
                "the loaded world: the fillet's re-run digest drift");
        rerun_blind =
            std::make_unique<b::BRepBody>(*rsub->hybrid->brep);
        b::link_radial_edges(*rerun_blind);
        rerun_fil =
            std::make_unique<b::BRepBody>(*rfil->fillet_body);
        b::link_radial_edges(*rerun_fil);
    }

    ~LoadedChain() {
        if (!file.empty()) {
            std::error_code ec{};
            std::filesystem::remove(file, ec);
        }
    }
};

// ---------------------------------------------------------------------------
// THE CORPUS WORLD — the m6 corpus built ONCE (quoted, member per
// member): the pre-gesture worlds (the old sides the timed paths
// consume), the new sides (the corpus ledger's own members), the
// tavole, the edit contexts, and the CORPUS LEDGER — the m6
// witnesses verified against their pins before any time is
// believed.
// ---------------------------------------------------------------------------

struct CorpusWorld;

// The m6 corpus's members 1, 2 and 4 — the THREE the m6
// determinism test pinned (its own bundle), verbatim: the corpus
// ledger and the m6 witness both consume them, one definition
// (the ledger's text is byte-sensitive: the witness's own
// requirement).
b::SurvivalMember m6_member1(const CorpusWorld& w);
b::SurvivalMember m6_member2(const CorpusWorld& w);
b::SurvivalMember m6_member4(const CorpusWorld& w);

struct CorpusWorld {
    // The pre-gesture worlds (the timed members' old sides).
    Chain5 original{0.25};
    DChain d_old{d_chords()};
    M6ter m6ter_original{0.5, 0.5};
    LoadedChain loaded;
    SubDecl sub_grammar{};

    // The corpus's own new sides (the m6 corpus members' bodies —
    // built once here; the timed paths rebuild them per iteration).
    Chain5 edited{0.125};
    DChain d_new{d_split()};
    M6ter m6ter_edited{0.25, 0.5};

    // The tavole — the old sides' (pre-gesture) and the corpus's.
    b::DescentTable sub_tav;
    b::DescentTable fil_tav;
    b::DescentTable d_sub_tav;
    b::DescentTable box_tav;
    b::DescentTable old_fil2_tav;
    b::DescentTable new_fil2_tav;
    b::DescentTable fil3_tav;
    b::DescentTable load_sub_tav;

    // The edit contexts (the m6 corpus's own declarations).
    b::EditContext no_shift;
    b::EditContext chord_map;

    // The corpus ledger — the m6 witness.
    b::SurvivalLedger corpus_ledger;
    std::array<std::string, 5> member_digests{};

    CorpusWorld() {
        if (original.blind == nullptr || original.result == nullptr ||
            edited.result == nullptr || d_old.blind == nullptr ||
            d_new.blind == nullptr ||
            m6ter_original.result == nullptr ||
            m6ter_edited.result == nullptr)
            fail_hard("the corpus world: a guide body is null");

        sub_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);
        fil_tav = b::fillet_descent(*edited.result, *edited.blind);
        d_sub_tav = b::subtract_descent(
            *d_old.blind, *d_old.box, *d_old.tool);
        box_tav = b::factory_descent(*m6ter_edited.box);
        old_fil2_tav = b::fillet_descent(
            *m6ter_original.first, *m6ter_original.box);
        new_fil2_tav = b::fillet_descent(
            *m6ter_edited.first, *m6ter_edited.box);
        fil3_tav = b::fillet_descent(
            *m6ter_edited.result, *m6ter_edited.first);
        load_sub_tav = b::subtract_descent(
            *loaded.scene.get_node(4)->hybrid->brep,
            *sc::operand_exact_brep(*loaded.scene.get_node(1), "box"),
            *sc::operand_exact_brep(*loaded.scene.get_node(3), "tool"));

        no_shift = b::EditContext{/*roots_no_shift=*/true, {}};
        chord_map.root_map.push_back(
            {n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 1},
             n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 3}});

        // THE FIVE MEMBERS — quoted from the m6 corpus, verbatim
        // (the ledger's text is byte-sensitive: the witness's own
        // requirement). Members 1, 2 and 4 ride the shared builders
        // (the determinism corpus's own three); members 3 and 5 are
        // the corpus test's own, inline here.
        std::vector<b::SurvivalMember> members;
        members.push_back(m6_member1(*this));
        members.push_back(m6_member2(*this));
        {  // Member 3: the LOAD BOOLEAN RE-RUN
            b::SurvivalMember m;
            m.kind = b::SurvivalEditKind::LoadBooleanRerun;
            m.name = "the load boolean re-run: the chain hydrated, "
                     "the boolean re-materialized from the recipe";
            m.digest = loaded.blind_digest;
            m.references = {
                {"box#1:vertical2", original.v2, &n::kBoxNaming, false},
                {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
                 original.rim, &sub_grammar.decl, false}};
            b::SurvivalStep edit_step;
            edit_step.kind = b::SurvivalStepKind::Edit;
            edit_step.name = "the load's re-materialization (pass 2, "
                             "the recipes the truth; the sketch payload "
                             "verbatim - no solve at load)";
            edit_step.refs = {
                {0, &load_sub_tav, loaded.rerun_blind.get(), no_shift},
                {1, &load_sub_tav, loaded.rerun_blind.get(), no_shift}};
            m.steps = {std::move(edit_step)};
            members.push_back(std::move(m));
        }

        {  // Member 4: the CHAINED FILLET EDIT
            members.push_back(m6_member4(*this));
        }

        {  // Member 5: the RECIPE RE-RUN
            const std::string rim_string =
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi";
            const ShapeId found = b::fillet_reference_find(
                *original.blind, rim_string);
            require(found == original.rim,
                    "the corpus member 5: the find's answer is not the "
                    "pinned rim");
            const b::FilletNaming fil_ctx{
                FeatureId{5}, &n::kBoxNaming, FeatureId{3},
                &n::kExtNaming};
            auto rerun = b::fillet_rim_edge(*original.blind, found,
                                            0.25, fil_ctx);
            if (rerun == nullptr)
                fail_hard("the corpus member 5: the re-run is null");
            require(b::chain_digest(*rerun) ==
                    b::chain_digest(*original.result),
                    "the corpus member 5: the re-run's digest drift");

            b::SurvivalMember m;
            m.kind = b::SurvivalEditKind::RecipeRerun;
            m.name = "the recipe re-run: the fields re-read, the find, "
                     "the DAG re-materialized";
            m.digest = b::chain_digest(*rerun);
            m.references = {
                {"box#1:vertical2", original.v2, &n::kBoxNaming, false},
                {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
                 original.rim, &sub_grammar.decl, false}};
            b::SurvivalStep edit_step;
            edit_step.kind = b::SurvivalStepKind::Edit;
            edit_step.name = "the recipe's fields re-read (the find on "
                             "the re-materialized operand, the radius "
                             "its own field)";
            edit_step.refs = {
                {0, &sub_tav, original.blind.get(), no_shift},
                {1, &sub_tav, original.blind.get(), no_shift}};
            m.steps = {std::move(edit_step)};
            members.push_back(std::move(m));
        }

        corpus_ledger = b::survival_ledger(members);
        for (std::size_t i = 0; i < members.size(); ++i)
            member_digests[i] = members[i].digest;
    }
};

// THE SHARED MEMBER BUILDERS — QUOTED verbatim from the m6
// determinism test's own bundle (the corpus that carries the
// pinned FNV): the members' strings byte-identical, the evidence
// the same world's own tavole and bodies.

b::SurvivalMember m6_member1(const CorpusWorld& w) {
    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::SilentRadiusEdit;
    m.name = "the silent radius edit: fil#5 at r=0.125";
    m.digest = b::chain_digest(*w.edited.result);
    m.references = {
        {"box#1:vertical2", w.original.v2, &n::kBoxNaming, false},
        {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
         w.original.rim, &w.sub_grammar.decl, false}};
    b::SurvivalStep es;
    es.kind = b::SurvivalStepKind::Edit;
    es.name = "sub#4 re-materialized (the edit touches fil#5 alone)";
    es.refs = {
        {0, &w.sub_tav, w.edited.blind.get(), w.no_shift},
        {1, &w.sub_tav, w.edited.blind.get(), w.no_shift}};
    b::SurvivalStep ms;
    ms.kind = b::SurvivalStepKind::Meal;
    ms.name = "fil#5 at r=0.125";
    ms.killer = "fil#5";
    ms.refs = {
        {0, &w.fil_tav, w.edited.result.get(), b::EditContext{}},
        {1, &w.fil_tav, w.edited.result.get(), b::EditContext{}}};
    m.steps = {std::move(es), std::move(ms)};
    return m;
}

b::SurvivalMember m6_member2(const CorpusWorld& w) {
    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::SketchRootEdit;
    m.name = "the sketch root edit: the chord split in "
             "three, the map declared";
    m.digest = b::chain_digest(*w.d_new.blind);
    m.references = {
        {"box#1:vertical2", w.d_old.v2, &n::kBoxNaming, false},
        {"ext#3:lat(1)", w.d_old.wall, &n::kExtNaming, false}};
    m.seats_declared = {
        "the rim rides the subtract's circle chains - the "
        "chain tool's exit edges are ADR-0038 D4's queued "
        "row (unnamed), and this member's rename seat is "
        "the arc's wall"};
    b::SurvivalStep es;
    es.kind = b::SurvivalStepKind::Edit;
    es.name = "sk#2 the middle insert (the chord split, "
              "the arc 1 -> 3, the map declared)";
    es.refs = {
        {0, &w.d_sub_tav, w.d_new.blind.get(), b::EditContext{}},
        {1, &w.d_sub_tav, w.d_new.blind.get(), w.chord_map}};
    m.steps = {std::move(es)};
    return m;
}

b::SurvivalMember m6_member4(const CorpusWorld& w) {
    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::ChainedFilletEdit;
    m.name = "the chained fillet edit: fil#2's radius 0.5 -> 0.25";
    m.digest = b::chain_digest(*w.m6ter_edited.result);
    m.references = {
        {"box#1:vertical2", w.m6ter_original.v2, &n::kBoxNaming,
         false},
        {"box#1:vertical0", w.m6ter_original.v0, &n::kBoxNaming,
         false}};
    m.seats_declared = {
        "the m6ter chain carries no subtract - the rim "
        "rides the subtract's chains (members 1, 3 and 5); "
        "this member's own seats are the baseline's "
        "vertical2 and the m5 kept-stamp vertical0"};
    b::SurvivalStep es;
    es.kind = b::SurvivalStepKind::Edit;
    es.name = "fil#2 re-materialized at r=0.25";
    es.refs = {
        {0, &w.box_tav, w.m6ter_edited.box.get(), b::EditContext{}},
        {1, &w.old_fil2_tav, w.m6ter_edited.first.get(),
         b::EditContext{}}};
    b::SurvivalStep m2;
    m2.kind = b::SurvivalStepKind::Meal;
    m2.name = "fil#2 at r=0.25 consumes vertical2";
    m2.killer = "fil#2";
    m2.refs = {{0, &w.new_fil2_tav, w.m6ter_edited.first.get(),
                b::EditContext{}}};
    b::SurvivalStep m3;
    m3.kind = b::SurvivalStepKind::Meal;
    m3.name = "fil#3 at r=0.5 consumes vertical0";
    m3.killer = "fil#3";
    m3.refs = {{1, &w.fil3_tav, w.m6ter_edited.result.get(),
                b::EditContext{}}};
    m.steps = {std::move(es), std::move(m2), std::move(m3)};
    return m;
}

// ---------------------------------------------------------------------------
// RIGA ZERO — the five members' timed paths. Each returns its own
// deterministic witness beside the elapsed time: the member's
// digest (the ledger's input) and the single-member ledger's
// counts. The OLD-side tavole and references ride the corpus world
// (they pre-date the gesture); everything the gesture causes — the
// chain re-run, the consumer tavole, the member's digest, the
// ledger's walk — is INSIDE the timed region. Declared, per member,
// in each path's own comment.
// ---------------------------------------------------------------------------

struct MemberOutcome {
    bool ok{false};
    std::string digest{};
    std::size_t resolved{};
    std::size_t renamed{};
    std::size_t ambiguous{};
    std::size_t vanished{};
    std::size_t deaths{};
};

struct MemberMeasurement {
    Stats stats{};
    std::string digest{};
    std::size_t resolved{};
    std::size_t renamed{};
    std::size_t ambiguous{};
    std::size_t vanished{};
    std::size_t deaths{};
};

MemberOutcome ledger_outcome(const b::SurvivalLedger& ledger,
                             const std::string& digest, bool ok) {
    MemberOutcome o;
    o.ok = ok;
    o.digest = digest;
    o.resolved = ledger.resolved;
    o.renamed = ledger.renamed;
    o.ambiguous = ledger.ambiguous;
    o.vanished = ledger.vanished;
    o.deaths = ledger.deaths.size();
    return o;
}

// THE MEMBER TIMED LOOP — warmup + iterations, EVERY iteration's
// witness checked against the first's (the digests bit-identical
// BEFORE the times are believed — the drift fails the run, never
// rides a number), the median + p95 over the measured iterations.
template <typename F>
MemberMeasurement measure_member(F&& work, const std::string& name) {
    MemberMeasurement out;
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(kIterations));
    bool have_first = false;
    for (int i = 0; i < kWarmup + kIterations; ++i) {
        const auto t0 = Clock::now();
        const MemberOutcome o = work();
        const auto t1 = Clock::now();
        require(o.ok, name + ": the path failed (iteration " +
                          std::to_string(i) + ")");
        if (!have_first) {
            out.digest = o.digest;
            out.resolved = o.resolved;
            out.renamed = o.renamed;
            out.ambiguous = o.ambiguous;
            out.vanished = o.vanished;
            out.deaths = o.deaths;
            have_first = true;
        } else {
            require(o.digest == out.digest,
                    name + ": the member's digest DRIFTED at iteration "
                    + std::to_string(i) +
                    " (ADR-0014: a performance number bought with a "
                    "determinism drift is not a number)");
            require(o.resolved == out.resolved &&
                        o.renamed == out.renamed &&
                        o.ambiguous == out.ambiguous &&
                        o.vanished == out.vanished &&
                        o.deaths == out.deaths,
                    name + ": the ledger's counts drifted at iteration "
                    + std::to_string(i));
        }
        if (i >= kWarmup)
            samples.push_back(elapsed_ns(t0, t1));
    }
    out.stats = stats_of(std::move(samples));
    return out;
}

// Member 1 — THE SILENT RADIUS EDIT: the gesture is fil#5's radius
// 0.25 -> 0.125; the timed path is the chain re-run at the new
// radius (the full box -> fillet construction, the DAG's own pass),
// the fillet's consumer tavola, the member's digest and the
// ledger's walk.
MemberOutcome run_member1(const CorpusWorld& w) {
    const Chain5 edited{0.125};
    if (edited.result == nullptr || edited.blind == nullptr)
        return {};
    const auto fil_tav_i =
        b::fillet_descent(*edited.result, *edited.blind);

    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::SilentRadiusEdit;
    m.name = "the silent radius edit: fil#5 at r=0.125";
    m.digest = b::chain_digest(*edited.result);
    m.references = {
        {"box#1:vertical2", w.original.v2, &n::kBoxNaming, false},
        {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
         w.original.rim, &w.sub_grammar.decl, false}};
    b::SurvivalStep edit_step;
    edit_step.kind = b::SurvivalStepKind::Edit;
    edit_step.name =
        "sub#4 re-materialized (the edit touches fil#5 alone)";
    edit_step.refs = {
        {0, &w.sub_tav, edited.blind.get(), w.no_shift},
        {1, &w.sub_tav, edited.blind.get(), w.no_shift}};
    b::SurvivalStep meal_step;
    meal_step.kind = b::SurvivalStepKind::Meal;
    meal_step.name = "fil#5 at r=0.125";
    meal_step.killer = "fil#5";
    meal_step.refs = {
        {0, &fil_tav_i, edited.result.get(), b::EditContext{}},
        {1, &fil_tav_i, edited.result.get(), b::EditContext{}}};
    m.steps = {std::move(edit_step), std::move(meal_step)};

    const auto ledger = b::survival_ledger({m});
    return ledger_outcome(ledger, m.digest, true);
}

// Member 2 — THE SKETCH ROOT EDIT: the gesture is the middle insert
// (the chord split in three, the arc moved 1 -> 3, the map
// declared); the timed path is the D-chain's re-run under the edit,
// the member's digest and the ledger's walk.
MemberOutcome run_member2(const CorpusWorld& w) {
    const DChain d_new{d_split()};
    if (d_new.blind == nullptr)
        return {};

    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::SketchRootEdit;
    m.name = "the sketch root edit: the chord split in three, "
             "the map declared";
    m.digest = b::chain_digest(*d_new.blind);
    m.references = {
        {"box#1:vertical2", w.d_old.v2, &n::kBoxNaming, false},
        {"ext#3:lat(1)", w.d_old.wall, &n::kExtNaming, false}};
    m.seats_declared = {
        "the rim rides the subtract's circle chains - the "
        "chain tool's exit edges are ADR-0038 D4's queued row "
        "(unnamed), and this member's rename seat is the arc's "
        "wall"};
    b::SurvivalStep edit_step;
    edit_step.kind = b::SurvivalStepKind::Edit;
    edit_step.name = "sk#2 the middle insert (the chord split, "
                     "the arc 1 -> 3, the map declared)";
    edit_step.refs = {
        {0, &w.d_sub_tav, d_new.blind.get(), b::EditContext{}},
        {1, &w.d_sub_tav, d_new.blind.get(), w.chord_map}};
    m.steps = {std::move(edit_step)};

    const auto ledger = b::survival_ledger({m});
    return ledger_outcome(ledger, m.digest, true);
}

// Member 3 — THE LOAD BOOLEAN RE-RUN: the gesture is the load (the
// file on disk, authored in the setup); the timed path is the
// load -> install -> rebuild_operations walk, the rebuilt body's
// extraction, the member's digest and the ledger's walk. The file
// I/O is inside the region — the honest end-to-end of opening the
// file, declared.
MemberOutcome run_member3(const CorpusWorld& w) {
    const io::ParsedDocument doc = io::load_file(w.loaded.file);
    sc::Scene re;
    io::install_document(re, io::ParsedDocument(doc));
    const auto rr = sc::rebuild_operations(re);
    const sc::SceneNode* rsub = re.get_node(4);
    if (rsub == nullptr || rsub->hybrid == nullptr ||
        rr.rebuilt == 0)
        return {};
    auto rerun_blind =
        std::make_unique<b::BRepBody>(*rsub->hybrid->brep);
    b::link_radial_edges(*rerun_blind);

    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::LoadBooleanRerun;
    m.name = "the load boolean re-run: the chain hydrated, the "
             "boolean re-materialized from the recipe";
    m.digest = b::chain_digest(*rerun_blind);
    m.references = {
        {"box#1:vertical2", w.original.v2, &n::kBoxNaming, false},
        {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
         w.original.rim, &w.sub_grammar.decl, false}};
    b::SurvivalStep edit_step;
    edit_step.kind = b::SurvivalStepKind::Edit;
    edit_step.name = "the load's re-materialization (pass 2, "
                     "the recipes the truth; the sketch payload "
                     "verbatim - no solve at load)";
    edit_step.refs = {
        {0, &w.load_sub_tav, rerun_blind.get(), w.no_shift},
        {1, &w.load_sub_tav, rerun_blind.get(), w.no_shift}};
    m.steps = {std::move(edit_step)};

    const auto ledger = b::survival_ledger({m});
    return ledger_outcome(ledger, m.digest, true);
}

// Member 4 — THE CHAINED FILLET EDIT: the gesture is fil#2's radius
// 0.5 -> 0.25; the timed path is the m6ter chain's re-run at the
// edited radius, the three tavole, the member's digest and the
// ledger's walk.
MemberOutcome run_member4(const CorpusWorld& w) {
    const M6ter m6ter_edited{0.25, 0.5};
    if (m6ter_edited.result == nullptr ||
        m6ter_edited.first == nullptr || m6ter_edited.box == nullptr)
        return {};
    const auto box_tav_i = b::factory_descent(*m6ter_edited.box);
    const auto new_fil2_tav_i = b::fillet_descent(
        *m6ter_edited.first, *m6ter_edited.box);
    const auto fil3_tav_i = b::fillet_descent(
        *m6ter_edited.result, *m6ter_edited.first);

    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::ChainedFilletEdit;
    m.name = "the chained fillet edit: fil#2's radius 0.5 -> 0.25";
    m.digest = b::chain_digest(*m6ter_edited.result);
    m.references = {
        {"box#1:vertical2", w.m6ter_original.v2, &n::kBoxNaming,
         false},
        {"box#1:vertical0", w.m6ter_original.v0, &n::kBoxNaming,
         false}};
    m.seats_declared = {
        "the m6ter chain carries no subtract - the rim rides "
        "the subtract's chains (members 1, 3 and 5); this "
        "member's own seats are the baseline's vertical2 and "
        "the m5 kept-stamp vertical0"};
    b::SurvivalStep edit_step;
    edit_step.kind = b::SurvivalStepKind::Edit;
    edit_step.name = "fil#2 re-materialized at r=0.25";
    edit_step.refs = {
        {0, &box_tav_i, m6ter_edited.box.get(), b::EditContext{}},
        {1, &w.old_fil2_tav, m6ter_edited.first.get(),
         b::EditContext{}}};
    b::SurvivalStep meal2;
    meal2.kind = b::SurvivalStepKind::Meal;
    meal2.name = "fil#2 at r=0.25 consumes vertical2";
    meal2.killer = "fil#2";
    meal2.refs = {{0, &new_fil2_tav_i, m6ter_edited.first.get(),
                   b::EditContext{}}};
    b::SurvivalStep meal3;
    meal3.kind = b::SurvivalStepKind::Meal;
    meal3.name = "fil#3 at r=0.5 consumes vertical0";
    meal3.killer = "fil#3";
    meal3.refs = {{1, &fil3_tav_i, m6ter_edited.result.get(),
                   b::EditContext{}}};
    m.steps = {std::move(edit_step), std::move(meal2),
               std::move(meal3)};

    const auto ledger = b::survival_ledger({m});
    return ledger_outcome(ledger, m.digest, true);
}

// Member 5 — THE RECIPE RE-RUN: the gesture is the recipe's fields
// re-read; the timed path is the find (by render against the
// re-materialized operand), the fillet's re-materialization, the
// member's digest and the ledger's walk.
MemberOutcome run_member5(const CorpusWorld& w) {
    const std::string rim_string =
        "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi";
    const ShapeId found =
        b::fillet_reference_find(*w.original.blind, rim_string);
    if (!(found == w.original.rim))
        return {};
    const b::FilletNaming fil_ctx{FeatureId{5}, &n::kBoxNaming,
                                  FeatureId{3}, &n::kExtNaming};
    auto rerun =
        b::fillet_rim_edge(*w.original.blind, found, 0.25, fil_ctx);
    if (rerun == nullptr)
        return {};
    if (b::chain_digest(*rerun) !=
        b::chain_digest(*w.original.result))
        return {};

    b::SurvivalMember m;
    m.kind = b::SurvivalEditKind::RecipeRerun;
    m.name = "the recipe re-run: the fields re-read, the find, "
             "the DAG re-materialized";
    m.digest = b::chain_digest(*rerun);
    m.references = {
        {"box#1:vertical2", w.original.v2, &n::kBoxNaming, false},
        {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
         w.original.rim, &w.sub_grammar.decl, false}};
    b::SurvivalStep edit_step;
    edit_step.kind = b::SurvivalStepKind::Edit;
    edit_step.name = "the recipe's fields re-read (the find on "
                     "the re-materialized operand, the radius "
                     "its own field)";
    edit_step.refs = {
        {0, &w.sub_tav, w.original.blind.get(), w.no_shift},
        {1, &w.sub_tav, w.original.blind.get(), w.no_shift}};
    m.steps = {std::move(edit_step)};

    const auto ledger = b::survival_ledger({m});
    return ledger_outcome(ledger, m.digest, true);
}

// ---------------------------------------------------------------------------
// THE CHAIN GATE — the five steps timed AFTER each step (the step's
// own time, never the cumulative), the m5 door run beside as the
// correctness witness (once per pass, untimed — the five
// predicates, the digests every iteration), the digests verified
// BEFORE the times are believed.
// ---------------------------------------------------------------------------

constexpr std::size_t kChainSteps = 5;
const std::array<const char*, kChainSteps> kStepNames{
    "box#1", "sk#2", "ext#3", "sub#4", "fil#5"};

struct StepTiming {
    std::string name;
    Stats stats{};
    std::string digest;
};

struct ChainPassResult {
    std::array<StepTiming, kChainSteps> steps{};
    bool correctness_green{false};
    std::string pass_name{};
};

// The m5 gate's own walk — QUOTED from the m5 baseline (the
// tavole, the held references, the five doors), the drift leg
// reading the FIRST iteration's digests (the re-run witness: the
// same recipe built 20+ times in the same process).
bool chain5_correctness_walk(
    const std::unique_ptr<b::BRepBody>& box,
    const std::vector<s::SketchChain>& chains,
    const std::unique_ptr<b::BRepBody>& tool,
    const std::unique_ptr<b::BRepBody>& blind,
    const std::unique_ptr<b::BRepBody>& result, const ShapeId& rim,
    const ShapeId& top,
    const std::array<std::string, kChainSteps>& first_digests) {
    const b::DescentTable box_tav = b::factory_descent(*box);
    const b::DescentTable ext_tav =
        b::extrude_descent(*tool, chains, FeatureId{3}, FeatureId{2});
    const b::DescentTable sub_tav =
        b::subtract_descent(*blind, *box, *tool);
    const b::DescentTable fil_tav =
        b::fillet_descent(*result, *blind);

    const auto box_v = b::chain_gate(b::ChainStep{
        box.get(), &box_tav, nullptr, nullptr, nullptr, {},
        first_digests[0]});
    const auto sketch_v =
        b::chain_gate_sketch(chains, first_digests[1]);
    const auto ext_v = b::chain_gate(b::ChainStep{
        tool.get(), &ext_tav, nullptr, nullptr, &chains, {},
        first_digests[2]});
    const auto sub_v = b::chain_gate(b::ChainStep{
        blind.get(), &sub_tav, box.get(), tool.get(), nullptr,
        {{top, b::Resolution::Kind::Resolved},
         {rim, b::Resolution::Kind::Resolved}},
        first_digests[3]});
    const auto fil_v = b::chain_gate(b::ChainStep{
        result.get(), &fil_tav, blind.get(), nullptr, nullptr,
        {{rim, b::Resolution::Kind::Vanished},
         {top, b::Resolution::Kind::Resolved}},
        first_digests[4]});

    bool green = box_v.pass && sketch_v.pass && ext_v.pass &&
                 sub_v.pass && fil_v.pass;
    green = green && box_v.failures.empty() &&
            sketch_v.failures.empty() && ext_v.failures.empty() &&
            sub_v.failures.empty() && fil_v.failures.empty();
    green = green && box_v.drift_zero && sketch_v.drift_zero &&
            ext_v.drift_zero && sub_v.drift_zero && fil_v.drift_zero;

    // The baseline's own counts (the m5 row's pins — the measured
    // chain IS the chain-5, or the benchmark measures nothing the
    // register knows).
    green = green && box_v.v == 8 && box_v.e == 12 && box_v.f == 6 &&
            box_v.chi == 2 && box_v.born == 26;
    green = green && ext_v.v == 4 && ext_v.e == 5 && ext_v.f == 3 &&
            ext_v.chi == 2 && ext_v.born == 12;
    green = green && sub_v.v == 12 && sub_v.e == 17 &&
            sub_v.f == 8 && sub_v.chi == 2 && sub_v.h == 1;
    green = green && fil_v.v == 14 && fil_v.e == 20 &&
            fil_v.f == 9 && fil_v.chi == 2 && fil_v.h == 1;
    return green;
}

ChainPassResult run_chain_pass(const std::string& pass_name) {
    ChainPassResult pass;
    pass.pass_name = pass_name;

    std::array<std::vector<double>, kChainSteps> samples{};
    std::array<std::string, kChainSteps> first_digests{};
    bool have_first = false;

    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    ShapeId rim{};
    ShapeId top{};

    for (int i = 0; i < kWarmup + kIterations; ++i) {
        // Step 1 — box#1 (the factory).
        auto t0 = Clock::now();
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto t1 = Clock::now();
        // Step 2 — sk#2 (the profile's authoring — the honest
        // ns-scale row: the authoring is data, the geometry is
        // where the time lives).
        chains.clear();
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        auto t2 = Clock::now();
        // Step 3 — ext#3 (the prism).
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        auto t3 = Clock::now();
        // Step 4 — sub#4 (the blind boolean).
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        auto t4 = Clock::now();
        // Step 5 — fil#5 (the reference find — the recipe door's
        // own walk — plus the construction).
        rim = ShapeId{};
        for (std::size_t k = 0; k < blind->edge_count(); ++k) {
            const auto e = b::query_edge(*blind, k);
            if (e.name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[k].shape_id();
                break;
            }
        }
        top = ShapeId{};
        for (std::size_t k = 0; k < blind->face_count(); ++k) {
            const auto f = b::query_face(*blind, k);
            if (f.name == "box#1:top") {
                top = blind->solid.shell().faces()[k].shape_id();
                break;
            }
        }
        const b::FilletNaming fil_ctx{FeatureId{5}, &n::kBoxNaming,
                                      FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, 0.25, fil_ctx);
        auto t5 = Clock::now();

        if (result == nullptr)
            fail_hard("the chain pass: a step's product is null");

        samples[0].push_back(elapsed_ns(t0, t1));
        samples[1].push_back(elapsed_ns(t1, t2));
        samples[2].push_back(elapsed_ns(t2, t3));
        samples[3].push_back(elapsed_ns(t3, t4));
        samples[4].push_back(elapsed_ns(t4, t5));

        // THE DIGESTS — untimed, every iteration, BEFORE the times
        // are believed (a benchmark that measures builds that
        // differ among themselves measures nothing).
        std::array<std::string, kChainSteps> digests{
            b::chain_digest(*box), b::sketch_digest(chains),
            b::chain_digest(*tool), b::chain_digest(*blind),
            b::chain_digest(*result)};
        if (!have_first) {
            first_digests = digests;
            have_first = true;
        } else {
            for (std::size_t k = 0; k < kChainSteps; ++k)
                require(digests[k] == first_digests[k],
                        std::string("the chain pass ") + pass_name +
                        ": the step " + kStepNames[k] +
                        "'s digest DRIFTED at iteration " +
                        std::to_string(i) +
                        " (ADR-0014: the times are not believed)");
        }
    }

    for (std::size_t k = 0; k < kChainSteps; ++k) {
        pass.steps[k].name = kStepNames[k];
        pass.steps[k].stats = stats_of(std::move(samples[k]));
        pass.steps[k].digest = first_digests[k];
    }

    // THE CORRECTNESS WALK — the m5 door on the last iteration's
    // chain, once per pass, untimed (the drift leg reads the first
    // iteration's digests: the re-run witness).
    pass.correctness_green = chain5_correctness_walk(
        box, chains, tool, blind, result, rim, top, first_digests);
    require(pass.correctness_green,
            "the chain pass " + pass_name +
            ": the m5 gate's correctness walk is RED (the measured "
            "chain is not the chain-5)");
    return pass;
}

// ---------------------------------------------------------------------------
// THE PERFORMANCE GATE — the bench's own verdict (a value, never a
// throw): the digest identity FIRST (the times believed only after
// it), then the per-step within-run A/B regression, the profile's
// shares beside. The thresholds are RELATIVE ONLY: the ratio 3.0
// over medians, the denominator floored at 1 us — never a second
// pinned on a machine that is not the one reading.
// ---------------------------------------------------------------------------

struct ChainBenchVerdict {
    bool pass{true};
    bool digest_identity{true};
    std::string digest_failure{};  // the drifted steps, named

    struct StepVerdict {
        std::string name{};
        bool judged{false};
        bool regressed{false};
        double ratio{1.0};
        double share{0.0};
    };
    std::vector<StepVerdict> steps{};

    // The named render — a gate that says only "failed" is a
    // decorative number, not a gate.
    std::string render() const {
        std::string t;
        if (!digest_identity) {
            t = "DIGEST DRIFT at " + digest_failure +
                " — the times are NOT believed (a performance "
                "number bought with a determinism drift is not a "
                "number)";
            return t;
        }
        for (const auto& s : steps) {
            if (!s.judged)
                continue;
            if (s.regressed)
                t += std::string("REGRESSED ") + s.name + ": pass B's "
                     "median is " + std::to_string(s.ratio) +
                     "x pass A's (beyond the declared " +
                     std::to_string(kGateRatio) + "x) — the step "
                     "that slowed; ";
        }
        if (t.empty())
            t = "green — every step judged, none regressed beyond " +
                std::to_string(kGateRatio) + "x within the run";
        return t;
    }
};

ChainBenchVerdict chain_bench_gate(const ChainPassResult& a,
                                   const ChainPassResult& b) {
    ChainBenchVerdict v;

    // (1) THE DIGEST IDENTITY — the first predicate: the two passes
    // must have measured THE SAME BUILDS, else the times are not
    // judged at all.
    for (std::size_t k = 0; k < kChainSteps; ++k) {
        if (a.steps[k].digest != b.steps[k].digest) {
            v.digest_identity = false;
            if (!v.digest_failure.empty())
                v.digest_failure += ", ";
            v.digest_failure += b.steps[k].name;
        }
    }
    if (!v.digest_identity) {
        v.pass = false;
        v.steps.clear();  // no time verdicts — the refusal
        return v;
    }

    // (2) THE PER-STEP REGRESSION — pass B's median over pass A's,
    // the denominator floored at 1 us (a nanosecond step cannot
    // counterfeit a regression), red beyond 3.0x.
    double total = 0.0;
    for (std::size_t k = 0; k < kChainSteps; ++k)
        total += b.steps[k].stats.median_ns;
    for (std::size_t k = 0; k < kChainSteps; ++k) {
        ChainBenchVerdict::StepVerdict s;
        s.name = b.steps[k].name;
        s.judged = true;
        const double denom = std::max(
            a.steps[k].stats.median_ns, kGateFloorNs);
        s.ratio = b.steps[k].stats.median_ns / denom;
        s.regressed = s.ratio > kGateRatio;
        s.share = total > 0.0
                      ? b.steps[k].stats.median_ns / total
                      : 0.0;
        if (s.regressed)
            v.pass = false;
        v.steps.push_back(s);
    }
    return v;
}

std::string render_step_digests(const ChainPassResult& p) {
    std::string t;
    for (std::size_t k = 0; k < kChainSteps; ++k)
        t += std::string(kStepNames[k]) + " " +
             hex16(fnv1a64(p.steps[k].digest)) + "\n";
    return t;
}

// ---------------------------------------------------------------------------
// THE COUNTERPROOF'S OWN INJECTION — a declared dummy spin: pure
// arithmetic load (a fixed-shape inner loop, the accumulator
// discarded through a volatile sink so the optimizer cannot elide
// it), touching NOTHING the digests read. Calibrated at runtime: it
// burns until it has spent >= 4x the step's honest median, so the
// red lands past the 3.0x gate on any machine, never by noise.
// ---------------------------------------------------------------------------

volatile double g_spin_sink = 0.0;

void spin_for(double budget_ns) {
    const auto deadline =
        Clock::now() + std::chrono::nanoseconds(
                           static_cast<std::int64_t>(budget_ns));
    double acc = 0.0;
    while (Clock::now() < deadline) {
        for (int i = 1; i <= 10000; ++i)
            acc += 1.0 / static_cast<double>(i);
    }
    g_spin_sink = acc;
}

}  // namespace

// ===========================================================================
// MAIN — the run's own order: the corpus and its witness first (a
// benchmark that has not verified WHAT it measures measures
// nothing), then riga zero, then the chain's two passes and the
// gate, then the counterproofs, then the verdict. The exit code is
// the bench's own answer: 0 only when every check holds.
// ===========================================================================
int main() {
    std::printf(
        "[k1-m8-benchmark] K1 m8 — the benchmark suite (ADR-0041 D5; "
        "the methodology declared in the source's own header)\n");
    std::printf("[k1-m8-benchmark] compiled with: %s\n", __VERSION__);
    std::printf("[k1-m8-benchmark] build type: %s\n",
                CAD_0_BENCH_BUILD_TYPE);
    std::printf(
        "[k1-m8-benchmark] methodology: warmup %d, iterations %d, "
        "median (never the min, never the raw mean), p95 "
        "nearest-rank; gate: within-run A/B per step, red beyond "
        "%.1fx, denominator floor 1us; the machine is declared in "
        "the report — the absolute numbers are indicative, the "
        "ratios are the substance\n",
        kWarmup, kIterations, kGateRatio);

    // ---- 1. THE CORPUS + THE M6 WITNESSES --------------------------------
    std::printf("[k1-m8-benchmark] the corpus world (m6 quoted) "
                "building...\n");
    const CorpusWorld corpus;

    // THE M6 WITNESS, FIRST — the determinism corpus's own pin: the
    // THREE members m6's determinism test bundled (the silent
    // radius edit, the sketch root edit, the chained fillet edit),
    // built here from the same world, the ledger's text required
    // to answer the SAME FNV (the identity checked, never assumed —
    // a benchmark that has not verified WHAT it measures measures
    // nothing).
    std::printf("[k1-m8-benchmark] the m6 witness: the determinism "
                "corpus (the three members m6 pinned)...\n");
    const std::vector<b::SurvivalMember> det_corpus = {
        m6_member1(corpus), m6_member2(corpus),
        m6_member4(corpus)};
    const auto det_ledger = b::survival_ledger(det_corpus);
    const std::uint64_t m6_fnv = fnv1a64(det_ledger.text());
    require(m6_fnv == kM6LedgerFnv,
            "the m6 witness: the determinism corpus's FNV " +
                hex16(m6_fnv) + " is NOT the m6 pin " +
                hex16(kM6LedgerFnv));
    std::printf(
        "[k1-m8-benchmark] the m6 witness VERIFIED: the determinism "
        "corpus's ledger answers the m6 pin, FNV %s (the corpus "
        "the benchmark measures is the corpus m6 measured)\n",
        hex16(m6_fnv).c_str());

    // The five-member corpus (the corpus test's own) on ITS pins:
    // the counts 9/10 with the register's three deaths.
    std::printf("[k1-m8-benchmark] the five-member corpus ledger "
                "(the m6 corpus test's own)...\n");
    require(corpus.corpus_ledger.total == 10,
            "the m6 corpus: the corpus total is not 10");
    require(corpus.corpus_ledger.resolved == 9,
            "the m6 corpus: the numerator is not 9 (class 1 alone)");
    require(corpus.corpus_ledger.renamed == 1,
            "the m6 corpus: the renamed count is not 1");
    require(corpus.corpus_ledger.ambiguous == 0,
            "the m6 corpus: the ambiguous count is not 0");
    require(corpus.corpus_ledger.vanished == 0,
            "the m6 corpus: the vanished count is not 0");
    require(corpus.corpus_ledger.deaths.size() == 3,
            "the m6 corpus: the deaths' register is not 3");
    require(corpus.corpus_ledger.fraction() ==
                std::pair<std::size_t, std::size_t>{9, 10},
            "the m6 corpus: the fraction is not 9/10");
    std::printf(
        "[k1-m8-benchmark] the five-member corpus VERIFIED: 9/10 "
        "with 3 deaths (the m6 corpus test's own pins)\n");

    // ---- 2. RIGA ZERO — the lag profile ------------------------------
    std::printf(
        "[k1-m8-benchmark] RIGA ZERO — the lag profile (the edit's "
        "end-to-end: the gesture -> the chain re-run -> the names "
        "resolved -> the ledger updated), the five members one by "
        "one:\n");
    const MemberMeasurement lag1 = measure_member(
        [&corpus] { return run_member1(corpus); },
        "member 1 (the silent radius edit)");
    const MemberMeasurement lag2 = measure_member(
        [&corpus] { return run_member2(corpus); },
        "member 2 (the sketch root edit)");
    const MemberMeasurement lag3 = measure_member(
        [&corpus] { return run_member3(corpus); },
        "member 3 (the load boolean re-run)");
    const MemberMeasurement lag4 = measure_member(
        [&corpus] { return run_member4(corpus); },
        "member 4 (the chained fillet edit)");
    const MemberMeasurement lag5 = measure_member(
        [&corpus] { return run_member5(corpus); },
        "member 5 (the recipe re-run)");

    const std::array<const MemberMeasurement*, 5> lag_rows{
        &lag1, &lag2, &lag3, &lag4, &lag5};
    const std::array<const char*, 5> lag_labels{
        "the silent radius edit: fil#5 at r=0.125",
        "the sketch root edit: the chord split in three, the map "
        "declared",
        "the load boolean re-run: the chain hydrated, the boolean "
        "re-materialized from the recipe",
        "the chained fillet edit: fil#2's radius 0.5 -> 0.25",
        "the recipe re-run: the fields re-read, the find, the DAG "
        "re-materialized"};
    // The expected per-member witnesses (the m6 corpus's own
    // answers — the benchmark measures the corpus m6 measured).
    const std::array<std::array<std::size_t, 5>, 5> lag_expected{{
        {{2, 0, 0, 0, 1}},  // m1: 2 resolved, 1 death at the meal
        {{1, 1, 0, 0, 0}},  // m2: 1 resolved, 1 renamed
        {{2, 0, 0, 0, 0}},  // m3: 2 resolved
        {{2, 0, 0, 0, 2}},  // m4: 2 resolved, 2 deaths at the links
        {{2, 0, 0, 0, 0}},  // m5: 2 resolved
    }};
    for (std::size_t k = 0; k < 5; ++k) {
        const MemberMeasurement& m = *lag_rows[k];
        require(m.digest == corpus.member_digests[k],
                std::string("riga zero, member ") +
                    std::to_string(k + 1) +
                    ": the digest is not the corpus member's own");
        require(m.resolved == lag_expected[k][0] &&
                    m.renamed == lag_expected[k][1] &&
                    m.ambiguous == lag_expected[k][2] &&
                    m.vanished == lag_expected[k][3] &&
                    m.deaths == lag_expected[k][4],
                std::string("riga zero, member ") +
                    std::to_string(k + 1) +
                    ": the ledger's counts are not the corpus's own");
        std::printf(
            "  %zu. %s\n      median %10.4f ms   p95 %10.4f ms   "
            "digest %s\n",
            k + 1, lag_labels[k], ms_of(m.stats.median_ns),
            ms_of(m.stats.p95_ns),
            hex16(fnv1a64(m.digest)).c_str());
    }

    // ---- 3. THE CHAIN GATE — two passes, the honest A/B --------------
    std::printf(
        "[k1-m8-benchmark] THE CHAIN GATE — the chain-5's five "
        "steps, the time AFTER each step (the step's own time, "
        "never the cumulative), two passes:\n");
    const ChainPassResult pass_a = run_chain_pass("A");
    const ChainPassResult pass_b = run_chain_pass("B");

    // The two passes' deterministic halves identical (the
    // in-process re-run witness — the m7 "built twice" pattern on
    // the real passes' own digests).
    require(render_step_digests(pass_a) == render_step_digests(pass_b),
            "the chain passes A and B: the deterministic halves "
            "differ (the times are not believed)");

    const ChainBenchVerdict honest =
        chain_bench_gate(pass_a, pass_b);
    require(honest.pass,
            "the honest chain's A/B gate: " + honest.render());
    double chain_total_b = 0.0;
    for (const auto& st : pass_b.steps)
        chain_total_b += st.stats.median_ns;
    for (std::size_t k = 0; k < kChainSteps; ++k) {
        std::printf(
            "  %-6s  A median %10.4f ms   B median %10.4f ms   "
            "ratio %7.3f   share %6.2f%%\n",
            kStepNames[k], ms_of(pass_a.steps[k].stats.median_ns),
            ms_of(pass_b.steps[k].stats.median_ns),
            honest.steps[k].ratio, 100.0 * honest.steps[k].share);
    }
    std::printf(
        "  the chain's median total (pass B): %.4f ms — the gate: "
        "%s\n",
        ms_of(chain_total_b), honest.render().c_str());

    // ---- 4. THE COUNTERPROOFS ----------------------------------------
    std::printf(
        "[k1-m8-benchmark] THE COUNTERPROOF — the slowed step (the "
        "declared dummy spin, >= 4x the honest median, touching "
        "nothing):\n");
    // The sabotaged pass: the four clean steps reuse pass B's own
    // medians (declared); sub#4 is re-measured with the spin
    // injected into its timed region.
    const double sub_budget =
        kSabotageBudgetX * pass_a.steps[3].stats.median_ns;
    std::vector<double> sabotaged_samples;
    {
        std::unique_ptr<b::BRepBody> sbox;
        std::vector<s::SketchChain> schains;
        std::unique_ptr<b::BRepBody> stool;
        std::unique_ptr<b::BRepBody> sblind;
        for (int i = 0; i < kWarmup + kSabotageIterations; ++i) {
            sbox = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
            schains.clear();
            schains.push_back(s::SketchChain{
                {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
            stool = b::make_prism(schains, 2.0f, +1, FeatureId{3});
            const b::SubtractNaming sub_ctx{
                FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
            const auto t0 = Clock::now();
            sblind = b::exact_subtract(*sbox, *stool,
                                       {0.0, 0.0, -0.5},
                                       b::kSubtractTolerance, sub_ctx);
            spin_for(sub_budget);
            const auto t1 = Clock::now();
            if (sblind == nullptr)
                fail_hard("the counterproof: the subtract is null");
            if (i >= kWarmup)
                sabotaged_samples.push_back(elapsed_ns(t0, t1));
        }
    }
    ChainPassResult sabotaged_pass = pass_b;  // the digests unchanged
    sabotaged_pass.steps[3].stats =
        stats_of(std::move(sabotaged_samples));
    const ChainBenchVerdict red =
        chain_bench_gate(pass_a, sabotaged_pass);
    require(!red.pass,
            "the counterproof (the slowed step): the gate did NOT "
            "go red — a gate that cannot say no is a decorative "
            "number, not a gate");
    require(red.digest_identity,
            "the counterproof: the digests must still be believed "
            "(the spin touched nothing)");
    require(red.steps.size() == kChainSteps,
            "the counterproof: every step must be judged");
    require(red.steps[3].judged && red.steps[3].regressed &&
                red.steps[3].name == std::string("sub#4"),
            "the counterproof: the red must NAME sub#4");
    bool controls_green = true;
    for (std::size_t k = 0; k < kChainSteps; ++k)
        if (k != 3)
            controls_green = controls_green && red.steps[k].judged &&
                             !red.steps[k].regressed;
    require(controls_green,
            "the counterproof: the clean steps must stay green "
            "beside the red (the controls)");
    std::printf(
        "  the gate RED on the named step: %s (sub#4's sabotaged "
        "median %.4f ms vs the honest %.4f ms, ratio %.2fx)\n",
        red.render().c_str(),
        ms_of(sabotaged_pass.steps[3].stats.median_ns),
        ms_of(pass_a.steps[3].stats.median_ns), red.steps[3].ratio);

    std::printf(
        "[k1-m8-benchmark] THE COUNTERPROOF — the digest drift (one "
        "character in box#1's digest: the times are not "
        "believed):\n");
    ChainPassResult drifted_pass = pass_b;
    drifted_pass.steps[0].digest.back() =
        drifted_pass.steps[0].digest.back() == '7' ? '8' : '7';
    const ChainBenchVerdict refused =
        chain_bench_gate(pass_a, drifted_pass);
    require(!refused.pass && !refused.digest_identity,
            "the counterproof (the digest drift): the gate did NOT "
            "refuse");
    require(refused.steps.empty(),
            "the counterproof (the digest drift): NO time verdicts "
            "may be issued — the times are not believed");
    require(refused.digest_failure.find("box#1") !=
                std::string::npos,
            "the counterproof (the digest drift): the refusal must "
            "name the drifted step");
    std::printf("  the gate REFUSED: %s\n", refused.render().c_str());

    // ---- 5. THE VERDICT — the deterministic half, FNV pinned ---------
    std::string verdict;
    verdict +=
        "k1-m8-benchmark verdict — the deterministic half (ADR-0014: "
        "the inputs, the sequence, the digests — never the times)\n";
    verdict += "methodology: warmup 3 iterations 21 median "
               "p95-nearest-rank gate-ratio 3.0 gate-floor 1us "
               "within-run A/B per step\n";
    for (std::size_t k = 0; k < kChainSteps; ++k)
        verdict += std::string("step-digest ") + kStepNames[k] + " " +
                   hex16(fnv1a64(pass_a.steps[k].digest)) + "\n";
    verdict += "gate: " +
               (honest.pass
                    ? std::string("the honest chain GREEN")
                    : std::string("RED — ") + honest.render()) +
               "\n";
    for (std::size_t k = 0; k < 5; ++k) {
        const MemberMeasurement& m = *lag_rows[k];
        verdict += "member " + std::to_string(k + 1) + " \"" +
                   lag_labels[k] + "\": resolved " +
                   std::to_string(m.resolved) + " renamed " +
                   std::to_string(m.renamed) + " ambiguous " +
                   std::to_string(m.ambiguous) + " vanished " +
                   std::to_string(m.vanished) + " deaths " +
                   std::to_string(m.deaths) + " digest " +
                   hex16(fnv1a64(m.digest)) + "\n";
    }
    verdict += "corpus-witness: the m6 determinism corpus (3 "
               "members: the silent radius edit, the sketch root "
               "edit, the chained fillet edit), fnv " +
               hex16(kM6LedgerFnv) + " VERIFIED\n";
    verdict += "corpus-witness: the five-member corpus 9/10 with 3 "
               "deaths (the m6 corpus test's own pins) VERIFIED\n";
    verdict += "oracle-witness: the m7 pin " + hex16(kM7OracleFnv) +
               " RECALLED (the corpus the oracle declared true)\n";
    verdict += "counterproof slowed-step: RED named sub#4, the "
               "controls green\n";
    verdict += "counterproof digest-drift: REFUSED named box#1, no "
               "time verdicts\n";

    const std::uint64_t verdict_fnv = fnv1a64(verdict);
    require(verdict_fnv == kM8VerdictFnv,
            "the verdict's FNV " + hex16(verdict_fnv) +
                " is NOT the pin " + hex16(kM8VerdictFnv) +
                " (a drift in any rendered value — digest, count, "
                "verdict, counterproof — moves it)");
    std::printf(
        "[k1-m8-benchmark] THE VERDICT (deterministic, %zu bytes): "
        "FNV-1a 64 %s %s\n",
        verdict.size(), hex16(verdict_fnv).c_str(),
        verdict_fnv == kM8VerdictFnv ? "== the pin" : "!= THE PIN");

    // ---- 6. THE EXIT --------------------------------------------------
    if (g_failures != 0) {
        std::printf(
            "[k1-m8-benchmark] FAILURES: %d — the numbers above are "
            "NOT to be believed\n",
            g_failures);
        return 1;
    }
    std::printf(
        "[k1-m8-benchmark] ALL GREEN — the digests believed, the "
        "witnesses verified, the gate green, both counterproofs "
        "red, the verdict pinned; the times are the machine's own "
        "(indicative), the ratios are the substance\n");
    return 0;
}
