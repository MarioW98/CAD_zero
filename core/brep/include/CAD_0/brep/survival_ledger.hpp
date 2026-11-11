// core/brep/include/CAD_0/brep/survival_ledger.hpp
//
// K1 m6 — THE SURVIVAL LEDGER: feature stability after parameter
// edits, the deaths named (ADR-0041 D1.4, the family protocol's
// fourth metric — the instrument the metric's definition demands:
// "The metric is the survival fraction — references that resolve
// class 1 over the total — AND the accounting of every death: each
// death carries its CLASS NAME and its payload, the four classes
// being ADR-0036 D3's, verbatim, never re-derived here").
//
// THE CONTRACT, in the metric's own words (D1.4, quoted):
//   "A death without its class name is a crash; a survival number
//    without the death ledger is a lie of omission."
// The ledger's discipline, derived from it and enforced AT THE DOOR:
//
//   1. A DEATH WITHOUT A NAME IS REFUSED. Every Vanished the walk
//      observes must carry its full name — WHAT died (the reference,
//      its render), WHO killed it (the step's own feature), WHY (the
//      cause VERBATIM — the tavola's vanished row's own cause, or the
//      resolver's diagnosis when the edit killed the entity), WHERE
//      (the step's own render). Any of them empty — the killer not
//      declared on the step, the tavola's cause wiped — and the
//      ledger REFUSES to register the member
//      (std::invalid_argument, diagnosis + remedy, the P2 culture):
//      never a partial ledger, never an anonymous death riding a
//      fraction.
//   2. THE FRACTION NEVER WITHOUT THE REGISTER. The verdict carries
//      the survival fraction (resolved over total, a PAIR — integers,
//      never a naked double) AND the deaths' register beside it, in
//      the same value: the consumer cannot read one without holding
//      the other. A fraction without its register is not
//      representable through this door.
//   3. THE SEATS ARE DECLARED, never silently skipped. A corpus
//      member whose chain does not carry a pinned reference (the
//      chained-fillets member carries no subtract — the rim rides the
//      subtract's chains; the D-tool's exit edges are ADR-0038 D4's
//      queued row, unnamed) declares the seat with its reason on the
//      member's evidence; the ledger reports it in the verdict
//      (seats_declared, verbatim) — the m5 gate's own
//      not-applicable-by-name discipline.
//
// THE CORPUS — the typed edit members the ledger is built to read
// (the mandate's five, each with its expected class):
//
//   1. SILENT RADIUS EDIT  — the fillet's own parameter, in-band:
//      the references resolve, the feature re-runs, the counts
//      stable — MATCH expected (the D5.3 pair, grown into a member).
//   2. SKETCH ROOT EDIT    — the root moves (the middle insert, the
//      map declared by the edit path — ADR-0036 D4's registered case
//      arriving with its real consumer): the dependent regenerates
//      under the new name — UNIQUE CONTINUER, the rename map declared,
//      never silent (class 2's FIRST authored instance: the v1
//      boundary the m5 gate refused by name, redeemed here).
//   3. LOAD BOOLEAN RE-RUN — the chain hydrated from the file (the
//      sketch payload carried VERBATIM — no solve at load), the
//      boolean re-materialized from the recipe: MATCH expected, the
//      digests bit-identical (ADR-0014's N-step clause through the
//      file's own round-trip).
//   4. CHAINED FILLET EDIT — the m6-correttiva-ter's chain (box ->
//      fil#2 -> fil#3), the radius edited, the m5 chain gate green
//      at every step of the re-materialized chain: the kept stamp
//      surviving the first link and dying the second — the deaths at
//      the links, each with the killer, the cause verbatim
//      ("replaced by the flank"), the step.
//   5. RECIPE RE-RUN       — the recipe's fields re-read (the
//      edge-reference string found by render against the
//      re-materialized operand — the m4 recipe door — the radius its
//      own typed field), the DAG re-materialized: MATCH expected.
//
// THE TWO STEP KINDS — the walk's own distinction, the metric's own:
// an EDIT step re-materializes the reference's chain under the edit
// (its resolution IS the class the member reports); a MEAL step is
// the consuming feature's own act (the fillet eating the rim, the
// flank replacing it — m5's census lesson): a death there is REAL and
// REGISTERED (the killer, the cause verbatim from the consumer's
// tavola, the step), but it is the FEATURE'S act, not the edit's —
// the class the member reports is the EDIT's answer, and the meal's
// death rides the register beside it. The discrimination is the
// caller's evidence (it built the chain, it knows which step is
// which): the ledger consumes the declaration, never invents it.
//
// THE WALK, per member, per reference: the references resolve step
// by step through THE RESOLVER — the landed authority, one opinion
// (zero re-derivation here: the guard, the vanished-row leg, the
// re-derivation with the roots rule, the descent with the candidates
// — all of it resolve()'s own). A RENAME follows the continuer (the
// reference's id becomes the renamed to — the feature's own next
// step, D3.2's "the reference follows to the new name"); a death
// stops the walk (the dead do not resolve); an ambiguity suspends it
// (the reference does not guess — the ledger does not either). The
// old side's tavola is per (reference, step): the CREATOR's rows for
// the edit steps, the CONSUMER's for the meal steps (the vanished
// row's own lodging — the death read where the tavola booked it).
//
// WHAT THE LEDGER REUSES (never re-derives — one opinion per number):
// resolve / resolution_code / resolution_text (the resolver, ADR-0036
// D3 + the morse-5 register) — the classes, the payloads, the
// causes; the DescentTable's rows (the vanished row's cause, the
// death's own verbatim). ZERO floating-point content anywhere in the
// ledger's own data (strings, integers, ShapeIds — ADR-0014 honored
// by construction; the members' digests are the CALLER's witnesses,
// carried verbatim, never computed here).
//
// THE GUARD — a reference that finds NO ROW AT ALL (resolve's
// ClosureDefect) is a closure violation in the evidence's tavola:
// the ledger REFUSES the member (std::invalid_argument, the
// resolution's own detail on the payload) — never a ledger built on
// a tavola that loses an entity (D2's floor, the resolver's own).
//
// Consumers: the K1 m6 corpus tests (tests/unit/brep/
// test_k1_m6_survival_ledger.cpp — the baseline, the five members,
// the counterproofs, the determinism), the Python report surface
// (brep.survival_ledger — the same door, the report's own evidence),
// and — through them — the register's m6 row.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::brep {

// ---------------------------------------------------------------------------
// The corpus member's kind — the mandate's five, typed (the expected
// class rides the token's own comment, the register's row reads it).
// ---------------------------------------------------------------------------

enum class SurvivalEditKind : std::uint8_t {
    SilentRadiusEdit,  // (1) the fillet's own parameter, in-band — MATCH
    SketchRootEdit,    // (2) the root moves, the map declared — RENAMED
    LoadBooleanRerun,  // (3) the chain hydrated, the boolean re-run — MATCH
    ChainedFilletEdit, // (4) the m6ter chain, the radius edited — the deaths
    RecipeRerun        // (5) the recipe's fields re-read — MATCH
};

// kind -> token, one switch, the single authority (the report's own
// column and the binding's own vocabulary).
const char* survival_edit_token(SurvivalEditKind k) noexcept;

// ---------------------------------------------------------------------------
// The evidence — what the caller (the test, the report) holds. The
// ledger never reaches outside it.
// ---------------------------------------------------------------------------

// One pinned reference: its render (the report's own string — the
// pinned baseline's), its structural id, and its GRAMMAR — the
// creating declaration the reference's paths were rendered under
// (ADR-0036 D3's per-creator rule: the subtract's for the rims, the
// extrude's for the walls, the factory's for the box's own).
struct SurvivalReference {
    std::string name{};
    geometry::ShapeId id{};
    const naming::OperationNaming* grammar{};
    bool multi_chain{false};
};

// The two step kinds — the walk's own distinction (see the header's
// own paragraph): the edit's class against the feature's meal.
enum class SurvivalStepKind : std::uint8_t {
    Edit,  // the re-materialization under the edit — THE CLASS's step
    Meal   // the consuming feature's act — the REGISTER's step
};

// One reference's evidence at one step: the OLD side's tavola (the
// creator's rows for an edit step, the CONSUMER's for a meal step —
// the vanished row's own lodging) and the NEW side's body, plus the
// edit context the step's path emitted (the no-shift declaration for
// the covered flows, the root map when the edit shifts and declares).
struct SurvivalStepRef {
    std::size_t reference{};           // the member's reference index
    const DescentTable* old_tavola{};  // the old side's rows (required)
    const BRepBody* new_body{};        // the new side's body (required)
    EditContext edit{};                // what the edit path emitted
};

// One step of the member's walk.
struct SurvivalStep {
    SurvivalStepKind kind{SurvivalStepKind::Edit};
    std::string name{};    // the step's render — the death's own WHERE
    std::string killer{};  // the step's creating feature — the death's WHO
    std::vector<SurvivalStepRef> refs{};
};

// The member's evidence: its kind (typed), its name (the report's
// own), the pinned references, the walk's steps, the seats it does
// not carry (declared, never silent), and the member's digest — the
// caller's determinism witness, carried verbatim (the ledger never
// computes one: the digest is the chain's own authority, not the
// ledger's).
struct SurvivalMember {
    SurvivalEditKind kind{SurvivalEditKind::SilentRadiusEdit};
    std::string name{};
    std::vector<SurvivalReference> references{};
    std::vector<SurvivalStep> steps{};
    std::vector<std::string> seats_declared{};
    std::string digest{};
};

// ---------------------------------------------------------------------------
// The verdict — a value, never a throw (the loud INPUT contract is
// the door's own: see survival_ledger() below).
// ---------------------------------------------------------------------------

// One (reference x member) verdict: the D3 class + the payload, the
// class's own words. The death's fields are MANDATORY when the class
// is Vanished — the discipline the door enforces (see the header's
// contract paragraph).
struct SurvivalEntry {
    std::string member{};     // the member's name
    std::string reference{};  // the pinned render — WHAT was tracked
    Resolution::Kind kind{Resolution::Kind::Resolved};
    std::string code{};       // resolution_code — the class's token

    // Class 2's payload: the rename, declared (the map rendered, the
    // feature's log writes it — never silence).
    std::string renamed_from{};
    std::string renamed_to{};
    std::string renamed_map{};

    // Class 3's payload: every candidate, named (the reference does
    // not guess — the ledger nominates them all).
    std::vector<std::string> candidates{};

    // Class 4's payload: the death, FULLY named — WHAT (died), WHO
    // (killer), WHY (cause, verbatim), WHERE (step) — and the remedy.
    // `dead` rides beside the class: a MEAL step's death does not
    // overwrite the edit's class (the feature's act is not the
    // edit's) — the death rides the REGISTER beside the class.
    bool dead{false};
    std::string died{};
    std::string killer{};
    std::string cause{};
    std::string remedy{};
    std::string step{};
};

// One member's ledger.
struct SurvivalMemberLedger {
    SurvivalEditKind kind{SurvivalEditKind::SilentRadiusEdit};
    std::string name{};
    std::vector<SurvivalEntry> entries{};
    std::vector<std::string> seats_declared{};
    std::string digest{};
    std::size_t resolved{0};
    std::size_t renamed{0};
    std::size_t ambiguous{0};
    std::size_t vanished{0};
};

// The corpus's ledger — the fraction AND the register, one value.
struct SurvivalLedger {
    std::vector<SurvivalMemberLedger> members{};
    std::size_t total{0};
    std::size_t resolved{0};
    std::size_t renamed{0};
    std::size_t ambiguous{0};
    std::size_t vanished{0};

    // THE DEATHS' REGISTER — every death the walks observed, each
    // fully named (the discipline's own guarantee: an anonymous death
    // never reached here — the door refused the member first).
    std::vector<SurvivalEntry> deaths{};

    // The survival fraction as a PAIR (resolved, total) — integers,
    // never a naked double (the register composes it; the report
    // renders it). THE NUMERATOR, pinned where the number is born
    // (ADR-0041 D1.4, verbatim: "references that resolve class 1
    // (`reference_resolved`, the feature re-running silently, ONE
    // undo entry) over the total"): class 1 ALONE counts — MATCH is
    // the numerator. UNIQUE CONTINUER (class 2, `reference_renamed`)
    // is a survival of the NAME, not a resolve: it rides the
    // DENOMINATOR and the register's counts beside the fraction,
    // never the numerator — never a fraction whose definition must
    // be guessed. ADR-0014 trivially honored: the counts are the
    // entries' own.
    std::pair<std::size_t, std::size_t> fraction() const noexcept {
        return {resolved, total};
    }

    // The register as text, one line per death — the report's own
    // render, readable riga per riga:
    //   "member | reference | killer | cause | step"
    std::string deaths_text() const;

    // The whole ledger as text (the report's own render): the
    // members' rows (the kind, the entries' codes) and the register.
    // Deterministic (strings and integers, zero floating-point
    // content) — the same corpus answers the same text, run after
    // run: the digest's own floor for the cross-run identity.
    std::string text() const;
};

// survival_ledger — THE DOOR. Walks every member's references
// through every step (the resolver's own answers, one opinion),
// records the classes and the deaths, computes the fraction over the
// entries, and returns the ledger — the fraction and the register in
// one value.
//
// LOUD on the input contract (the P2 culture — std::invalid_argument
// with the diagnosis and the remedy, the m5 gate's own split: loud
// on the evidence, quiet in the answers):
//   * a member without a name, a reference without a render or a
//     grammar, a step-ref whose tavola/body is missing (the
//     resolver's own refusal bubbles, its diagnosis verbatim);
//   * a step-ref for a reference already dead (the walk stopped at
//     its death — the caller's stale entry);
//   * a tracked reference with no EDIT step (the class needs its
//     own step — a meal alone does not answer the metric's question);
//   * A DEATH WITHOUT ITS NAME — the killer not declared on the
//     step, the cause wiped from the tavola, the step unnamed: the
//     ledger REFUSES to register the member ("a death without its
//     class name is a crash" — D1.4's own words, made a refusal);
//   * the GUARD: a reference that finds no row at all (the
//     resolver's closure_defect) — the evidence's tavola is
//     defective, the member is refused, the detail verbatim.
// The classes themselves — MATCH, RENAMED, AMBIGUOUS, VANISHED —
// are VALUES in the verdict, never throws: the honest answers of an
// honest walk.
SurvivalLedger survival_ledger(const std::vector<SurvivalMember>& members);

} // namespace CAD_0::brep
