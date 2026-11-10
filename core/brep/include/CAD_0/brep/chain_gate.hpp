// core/brep/include/CAD_0/brep/chain_gate.hpp
//
// K1 m5 — THE CHAIN GATE: the topological correctness after every
// step (ADR-0041 D1.3, the chain benchmark's own instrument — the
// gate the metric's definition demands: "executed with a GATE after
// EVERY step, the gate checking the invariants the house already
// derives per result, now checked as a standing contract on the
// intermediate states").
//
// THE FIVE PREDICATES (D1.3, verbatim in spirit — each with its
// NAMED message when it fails, never a bare false):
//
//   1. RADIAL — every edge exactly TWO OPPOSITE co-edges (the radial
//      structure, the census's own arithmetic — the count and the
//      doubled count);
//   2. LOOPS — every loop CLOSED head-tail (the walk returns; no
//      open chain survives a step);
//   3. CHI — χ = V−E+F−H EVEN, the body a closed orientable
//      composition (the parity is the gate, the value the golden's:
//      the sphere-topology body reads 2, the genus-1 additions 0);
//   4. CENSUS — the tavola's closure BOTH SIDES, read BY NAME: the
//      surviving rows' stamps answer on the result, the born rows'
//      stamps answer, the vanished rows' stamps answer NOWHERE (the
//      death is real) — and the references the chain HOLDS across
//      the step resolve with their ADR-0036 D3 class (MATCH found,
//      VANISHED gone with the tavola's own cause; the classes 2/3
//      are the registered v1 boundary, refused by name — never
//      guessed, never silently followed);
//   5. DRIFT — the step's digest bit-identical to the re-run's
//      (ADR-0014's determinism clause grown the N-step spine: the
//      flags, the accumulation order, the quantizer — the chain
//      re-run is the m4 N=10 round-trip with a spine).
//
// THE GATE IS A DOOR, NOT A SCATTERED ASSERT: ONE module, ONE
// consumer's seat — the tests and the report both call THIS door,
// never a private re-derivation. The verdicts are VALUES (booleans,
// named failure messages, the step's own numbers and digest), never
// throws: a gate that crashes on the corruption it exists to catch
// would be an assert wearing a costume. The loud INPUT contract is
// the P2 culture's own: a null body on a body step throws with the
// diagnosis and the remedy.
//
// WHAT THE GATE REUSES (never re-derives — one opinion per number):
// euler_characteristic / inner_loop_count / is_closed_orientable
// (the post-pass authorities, ADR-0032 D3), descent_closed /
// old_generation_closed / extrude_froms_resolve (the tavola's own
// closure checks, ADR-0036 D2), chain_is_closed (the sketch world's
// own closure predicate, ADR-0030 D2), query_all_faces /
// query_all_edges (the digest's payload, the H2 unit 6 pattern:
// "every face's full query answer as one comparable string, the
// names riding the records").
//
// THE DIGEST — chain_digest: the whole body as one comparable,
// multi-line string. The walk: the body's summary line (V E F H χ
// closed), one line per vertex (the position), one per edge (the
// endpoints, the name), one per face (the family, the loops, every
// co-edge's p-curve payload, the name) — the entities in the body's
// OWN order, so a re-numbering IS a drift the digest observes (the
// m6ter's own contract: "the survivors compact, the born append").
// The quantizer: every DOUBLE is written with 12 significant digits
// (ADR-0014's serialization rule for double-originated numbers; the
// native file's float32 shortest-round-trip is ANOTHER world's rule,
// never imported here — the digest reads the kernel's doubles). The
// digest is pure reading: no number is computed that the body's own
// authorities do not already carry.
//
// THE SKETCH STEP — chain_gate_sketch: the profile is the step's
// product, and the gate reads it in the profile's OWN world: the
// loop predicate consumes chain_is_closed (the one opinion the
// kernel and the canvas already share), the drift consumes
// sketch_digest (the chains' typed payload, quantized the same 12
// digits). The BODY predicates (radial, χ, census) are NOT-A-PLICABLE
// BY NAME — carried in the verdict's not_applicable list, never
// silently skipped: the sketch world has no radial structure, no
// χ, no tavola, and the gate says so instead of guessing.
//
// Consumers: the K1 m5 chain tests (the baseline chain-5 and the
// sequences behind it), the Python report surface (brep.chain_gate /
// brep.chain_digest — the same door, the report's own evidence: the
// body alone, the tavola-less steps honestly n/a), and — through
// them — the register's m5 row.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace CAD_0::brep {

// ---------------------------------------------------------------------------
// The five predicates — named, one token each (the failure message's
// own leading word, the verdict's row entry, the report's column).
// ---------------------------------------------------------------------------

enum class GateCheck : std::uint8_t {
    RadialTwoOpposite,  // (1) two opposite co-edges per edge
    LoopsClosed,        // (2) every loop closed head-tail
    ChiEvenOrientable,  // (3) χ even, closed orientable composition
    CensusByName,       // (4) the tavola's closure both sides, by name
    DriftZero           // (5) the digest bit-identical to the re-run
};

// The check's token — "radial", "loops", "chi", "census", "drift" —
// one switch, the single authority (the messages and the report both
// read it).
const char* gate_check_token(GateCheck c) noexcept;

// One failed check with its NAMED message — the message quotes the
// entity (kind, address, name), the numbers, and the remedy's
// direction; a bare "failed" is the silence this gate exists to
// prevent.
struct GateFailure {
    GateCheck check{};
    std::string message{};
};

// A reference the chain HOLDS across the step, with the class the
// honest step must answer (ADR-0036 D3's vocabulary — the two
// classes the authored flows produce; classes 2/3 are the registered
// v1 boundary and the gate REFUSES them by name, D3.2's own words:
// "machinery and no v1 instance is a registered boundary, not a
// promise").
struct HeldReference {
    geometry::ShapeId reference{};
    Resolution::Kind expected{Resolution::Kind::Resolved};
};

// ---------------------------------------------------------------------------
// The step's evidence — what the gate READS. The caller (the test,
// the report) holds the sequence's state and hands the gate the
// step's own facts; the gate never reaches outside them.
// ---------------------------------------------------------------------------

struct ChainStep {
    // The step's RESULT body — required (the body predicates' own
    // subject). Throws std::invalid_argument (diagnosis + remedy)
    // when null: a body step without a body is a caller's mismatch.
    const BRepBody* body{};

    // The step's tavola — the census predicate's own subject. Null
    // = the step declares no descent: the census is honestly n/a
    // (carried in the verdict, never guessed). The chain's honest
    // steps always carry one (the factories' all-born tavola is
    // derived by the caller — factory_descent — the derivation the
    // ADR-0036 D4 rule already pins).
    const DescentTable* tavola{};

    // The step's OPERANDS — the old generation's bodies:
    //   * a factory step (the box): both null — the old side is
    //     vacuously closed, nothing precedes the root;
    //   * the extrude: the `sketch` field carries the chains (the
    //     old side is the FROM side — extrude_froms_resolve);
    //   * the subtract: `previous` is the target A, `tool` the
    //     tool B (old_generation_closed's own two bodies);
    //   * the fillet: `previous` is the operand (the old side
    //     takes it as A and an EMPTY B — the one-operand pattern
    //     fillet_descent itself applies internally).
    const BRepBody* previous{};
    const BRepBody* tool{};
    const std::vector<sdf::SketchChain>* sketch{};

    // The references the chain holds across this step, each with its
    // expected D3 class — the census-by-name's own moving part (the
    // surviving, the dead — read against the tavola and the body).
    std::vector<HeldReference> held{};

    // The re-run's digest — the drift predicate's own subject.
    // Empty = the drift is honestly n/a this step (said, never
    // silent). The caller re-ran the chain from the recipes (the
    // DAG pass) and hands the digest the re-run produced.
    std::string rerun_digest{};
};

// ---------------------------------------------------------------------------
// The verdict — a value, never a throw. `pass` is the door's own
// answer; the per-check booleans carry the row's per-step verdict
// (the register's m5 line reads them): TRUE means the check ran and
// passed — or is honestly not-applicable, which the not_applicable
// list names — FALSE means the check ran and FAILED. `failures`
// carries every failed check's named message; `not_applicable`
// carries the checks this step's evidence does not reach (said,
// never hidden); the numbers and the digest are the step's own —
// the row's arithmetic.
// ---------------------------------------------------------------------------

struct ChainGateVerdict {
    bool pass{true};
    std::vector<GateFailure> failures{};
    std::vector<GateCheck> not_applicable{};

    bool radial_two_opposite{true};
    bool loops_closed{true};
    bool chi_even_orientable{true};
    bool census_by_name{true};
    bool drift_zero{true};

    // The step's numbers — the authorities' own answers, verbatim.
    std::size_t v{0};
    std::size_t e{0};
    std::size_t f{0};
    std::int64_t h{0};
    std::int64_t chi{0};

    // The census's own arithmetic (the tavola's counts by relation —
    // DescentTable::count's own answers): the row's born/kept/
    // absorbed/vanished numbers, zeroed when the census is n/a.
    std::size_t born{0};
    std::size_t kept{0};
    std::size_t absorbed{0};
    std::size_t split{0};
    std::size_t merged{0};
    std::size_t vanished{0};

    // The step's own digest — chain_digest(body): the drift's left
    // side and the row's identity pin.
    std::string digest{};
};

// chain_gate — THE DOOR. Reads the step's evidence, answers with the
// verdict. The five predicates run in their D1.3 order; a failed
// predicate does not stop the walk (a gate that hid the second
// corruption behind the first would be half a gate) — every failure
// is collected, each with its name.
//
// LOUD on the input contract, quiet in the answers: a null body
// throws std::invalid_argument (diagnosis + remedy, the P2
// culture); everything else — the corrupted bodies included — is
// ANSWERED, never thrown.
ChainGateVerdict chain_gate(const ChainStep& step);

// ---------------------------------------------------------------------------
// The sketch step's door — the profile's own world (see the header's
// sketch paragraph): loops + drift active, radial/χ/census n/a BY
// NAME. The same door's family, never a second opinion: the loop
// predicate IS chain_is_closed, the digest IS sketch_digest.
// ---------------------------------------------------------------------------

struct SketchGateVerdict {
    bool pass{true};
    std::vector<GateFailure> failures{};
    std::vector<GateCheck> not_applicable{};

    bool loops_closed{true};
    bool drift_zero{true};

    std::string digest{};  // sketch_digest(chains)
};

SketchGateVerdict chain_gate_sketch(
    const std::vector<sdf::SketchChain>& chains,
    const std::string& rerun_digest);

// ---------------------------------------------------------------------------
// The digests — the drift predicate's own instruments (see the
// header's digest paragraph for the format and the quantizer).
// ---------------------------------------------------------------------------

// The body's whole answer as one comparable string.
std::string chain_digest(const BRepBody& body);

// The profile's whole answer as one comparable string (the chains'
// typed payload, the same 12-digit quantizer).
std::string sketch_digest(
    const std::vector<sdf::SketchChain>& chains);

} // namespace CAD_0::brep
