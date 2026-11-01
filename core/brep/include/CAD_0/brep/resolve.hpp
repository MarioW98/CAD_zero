// core/brep/include/CAD_0/brep/resolve.hpp
//
// THE RESOLVER — ADR-0036 D3 + D6.4 (the accepted act, H2-u6 morso
// 5): the machine that answers "cosa succede al tuo riferimento dopo
// l'edit" — with classes NAMED, never silences, never coin flips.
// A downstream feature references an entity BY NAME (the structural
// ShapeId — D1's identity, never the address); an edit happens; the
// DAG re-materializes; the reference must resolve against the NEW
// generation. There are exactly FOUR outcomes, each with its pinned
// disposition, plus the guard under all four (D3's own derivation):
//
//   1. MATCH            reference_resolved   the name re-derives
//                                          identically; the feature
//                                          re-runs, SILENTLY.
//   2. UNIQUE CONTINUER reference_renamed   one living descendant,
//                                          a DIFFERENT name; the
//                                          map declared, the log
//                                          says so, never silent.
//   3. AMBIGUOUS        reference_ambiguous multiple descendants;
//                                          the reference does NOT
//                                          guess — the entity and
//                                          every candidate named,
//                                          the remedy with them.
//   4. VANISHED         reference_vanished  no living descendant;
//                                          what died, WHY, the
//                                          remedy — never a mute
//                                          "not found".
//   the guard           closure_defect      NO ROW AT ALL in the
//                                          old tavola — a closure
//                                          violation, a DEFECT
//                                          surfaced as such, not a
//                                          user error, not silence.
//
// THE CODES are the family registered for the fillet (ADR-0036 D6's
// hook: the G2 fillet's error culture inherits these four words —
// `reference_ambiguous`/`reference_vanished` its degenerates,
// `reference_resolved`/`reference_renamed` the silent and the logged
// successes; ADR-0033 D4's culture, one more province). The guard's
// own token stands OUTSIDE the family: the defect is the kernel's,
// not the reference's.
//
// THE ALGORITHM — three steps, PURE and DETERMINISTIC (B2's own
// order; zero floating-point content anywhere, ADR-0014 honored by
// construction):
//
//   0. THE GUARD, UNDER ALL FOUR: the reference's row in the OLD
//      tavola, found by ShapeId equality (the STRUCTURE the morse-5
//      register added to the rows, never the render). No row at all
//      → ClosureDefect: D2's closure property is the resolver's own
//      floor — a tavola that loses an entity is a defect the
//      resolution surfaces, never a "not found" wearing an error's
//      clothes.
//   1. THE VANISHED ROW: a reference whose row IS a vanished row
//      points at an entity the OLD generation's own operation
//      killed (the tool's caps — "cut away outside the target").
//      Class 4 immediately, the tavola's own cause on the payload.
//   2. RE-DERIVATION: the name re-derives from the NEW recipe (the
//      morsi 2-4 made it a pure function — the new body's stamps ARE
//      the re-derivation); a structural match → class 1. THE ROOTS
//      RULE (ADR-0036 D4's amendment, the review's own): the
//      re-derivation CONSUMES the edit context the edit path emits —
//      the no-shift declaration for the v1-covered flows
//      (add-at-end, the parameter edits), the root maps when the
//      edit shifts roots and declares them. NEVER the naked
//      positional match across a generation: that is the door of the
//      silent mis-reference, and it is CLOSED BY CONTRACT — a
//      reference whose genealogy depends on authored roots resolves
//      through the honest classes when the edit context covers it
//      not. (The map's remap: covered → the remapped name matches →
//      class 2 when the name differs, class 1 when it does not.)
//   3. THE DESCENT: the old row followed. The candidates are the new
//      generation's entities under the reference's ROLE SKELETON —
//      the path's role-table components (structural, compared), the
//      authored roots and the multi-chain groups thrown OPEN (the
//      indices that shift with the sketch's own layout):
//        k >= 1 → class 3, every candidate named (the identity AND
//                  the render), the remedy with them;
//        k == 0 → class 4, what died (the row), why (the authored
//                  entity was replaced / the role has no instance),
//                  the remedy.
//      The seam-wired pair (ADR-0023) is ONE candidate — the pair is
//      the name's materialized form, not two names; the candidates
//      are DISTINCT identities, the materialization count is the
//      query surface's own answer.
//
// NO IMPORTED TIE-BREAK (B2's own pin): ADR-0035 D3's bands stay
// where they belong — the in-band arithmetic; a reference is not a
// tolerance decision, and the classes 3 and 4 NEVER guess. No
// signature matching (ADR-0005's own correction, upheld), no silent
// re-picking (the scene's dangling protocol — scene.hpp:271-274,
// "a dangling reference is REPORTED, never silently re-picked" —
// EXTENDED here from feature granularity to ENTITY granularity:
// the resolution's classes are the protocol's own words at the
// subshape level, the granularity the fillet consumes).
//
// THE TWO QUERY SURFACES (ADR-0036 D6.4, B3):
//
//   * QUERY-BY-NAME — query_face_by_name / query_edge_by_name
//     (query.hpp): the structural name → the address(s) on the
//     materialized body, for the report and the vivo. The
//     counterpart of the positional query (query_face/query_edge),
//     which stays intact — D1's split: two planes, two doors.
//   * THE PICK — the click's address → the name AT PICK TIME: the
//     existing positional queries' own `name`/`id` fields (the pick
//     resolves the address to the identity it lands on); the
//     dangling protocol above is the pick's own contract across the
//     edit.
//
// HONEST BOUNDARIES, registered (D4's queue discipline, none a
// promise): vertex references land on the guard (the tavola's vertex
// rows carry no identities — vertex names are v1's queue); the root
// map's machinery ships with the SYNTHETIC test as its only
// exerciser — v1's authored flows produce no instance (the boundary
// registered at ADR-0036 D3.2: the add-at-end edits never shift a
// role, and the shifting edits arrive with the real maps); the map's
// remap covers the SINGLE-CHAIN roots (the sketch-chain ↔
// construction-group coincidence, naming.hpp's own note) — the
// multi-chain remap and the cross-chain maps are the registered
// future, and a map asked beyond the machinery refuses LOUDLY; the
// multi-chain flip (a chain appended, the group prefix appearing)
// resolves per the honest classes (the skeleton's groups are open)
// — the group re-derivation arrives with its first real case.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::brep {

// ---------------------------------------------------------------------------
// The edit context — WHAT THE EDIT PATH EMITS (ADR-0036 D4's
// amendment: "the edit path that shifts referenced roots EMITS the
// root map or REFUSES with the remedy"; the resolver CONSUMES the
// declaration, never invents it)
// ---------------------------------------------------------------------------

struct EditContext {
    // THE NO-SHIFT DECLARATION — the v1-covered flows (the canvas's
    // add-at-end, the parameter edits): the pre-existing roots keep
    // their positions in their chains, and the chains their order.
    // The edit path's own contract: the resolver trusts a structural
    // match on a root-dependent name ONLY under this declaration (or
    // a covering map) — the naked positional match across a
    // generation is the mis-reference door, closed by contract.
    bool roots_no_shift{false};

    // THE ROOT MAP — the edit that shifts roots and declares them
    // (the registered future: v1's authored flows emit none; the
    // machinery ships and the synthetic test exercises it). The
    // entries are (old root, new root) pairs in the ADR's own
    // vocabulary (naming::RootRef — the sketch chain entity); the
    // lookup reads kind + chain + entity (the map serves one sketch,
    // the roots' own feature).
    std::vector<std::pair<naming::RootRef, naming::RootRef>> root_map{};

    // The map's entry for a root, or nullptr when the map does not
    // carry it (an absent root is not a death sentence — the descent
    // decides, honestly, never a list-diff guess).
    const naming::RootRef* mapped_root(
        const naming::RootRef& old_root) const noexcept;

    // The map rendered, the log's own text ("sk#F:0 -> sk#F:1", the
    // entries in declared order) — the class-2 payload's declared
    // map, the feature's log writes it, never silence.
    std::string map_text() const;
};

// ---------------------------------------------------------------------------
// One generation of the referenced feature's output — what the
// resolution reads (the user's own signature: the old TAVOLA and
// the new GENERATION, each made whole with what its side consumes)
// ---------------------------------------------------------------------------

struct Generation {
    // The materialized output body — REQUIRED on the NEW side (the
    // stamps ARE the re-derivation); unread on the old side (the
    // tavola's rows carry the identities, the morse-5 register's own
    // landing).
    const BRepBody* body{};

    // The descent table — REQUIRED on the OLD side (the rows the
    // guard and the descent read); unread on the new side (the new
    // generation's descent is the caller's to derive when a consumer
    // wants it — the resolution never needs it).
    const DescentTable* tavola{};

    // THE REFERENCE'S GRAMMAR — the naming declaration the
    // reference's paths were rendered under: the CREATING
    // operation's tables for a feature's own entities (the
    // subtract's for the rims, the extrude's for the walls), and the
    // ORIGINAL creator's for an inherited entity (the absorbed wall
    // inside a subtract's result is the EXTRUDE's face — the caller
    // holds the declaration, it made the reference). The relational
    // legs recurse through the declaration's operand bindings, as
    // the renderer does.
    const naming::OperationNaming* naming{};

    // The instance's chain-count verdict for THIS side (the view
    // selector: the grouped view when the named body materialized
    // from more than one chain — the same flag the stamps were
    // rendered under; the sides may differ, the edit's own doing).
    bool multi_chain{false};
};

// ---------------------------------------------------------------------------
// The resolution — a CLOSED variant: exactly one alternative active,
// the kind's own (the PlaneCylinderSSI house pattern). Every payload
// carries the actionable data (ADR-0033 D4's culture): the names,
// the causes, the remedies — never a bare failure.
// ---------------------------------------------------------------------------

// Class 1 — MATCH: the name re-derived identically; the feature
// re-runs, silently (the silence is the CALLER's protocol: nothing
// is logged on this class — the test asserts it).
struct ResolutionResolved {
    geometry::ShapeId id{};      // the re-derived identity (== the reference)
    std::string name{};          // the canonical render — the same name
};

// Class 2 — UNIQUE CONTINUER: one living descendant, a DIFFERENT
// name; the map declared (the feature's log writes it, never
// silence). v1's authored flows produce no instance (ADR-0036
// D3.2's registered boundary); the machinery and the synthetic test
// ship.
struct ResolutionRenamed {
    geometry::ShapeId from{};    // the reference's identity
    geometry::ShapeId to{};      // the continuer's, under the new recipe
    std::string from_name{};
    std::string to_name{};
    std::string map{};           // the declared root map, rendered
};

// Class 3 — AMBIGUOUS: multiple descendants; the reference does NOT
// guess. The entity and EVERY candidate by name, the remedy with
// them. `candidates` and `candidate_names` are parallel (the same
// size, index-aligned) — the identities for the structural checks,
// the renders for the error the user reads.
struct ResolutionAmbiguous {
    std::vector<geometry::ShapeId> candidates{};
    std::vector<std::string> candidate_names{};
    std::string entity_name{};   // the reference, named (its row's own)
    std::string remedy{};
};

// Class 4 — VANISHED: no living descendant. WHAT died (the old
// tavola's row, carried whole — the subject's identity and name on
// it), WHY (the tavola's own cause when the row is a vanished row;
// the resolver's diagnosis otherwise), and the remedy. Never a mute
// "not found".
struct ResolutionVanished {
    DescentRow row{};            // the old tavola's row — what died
    std::string why{};           // the cause (CauseText)
    std::string remedy{};
};

// The guard — CLOSURE DEFECT: the reference found NO ROW AT ALL in
// the old tavola. Not a user error, not a silence: a closure
// violation (D2's own invariant — every entity of the generation in
// exactly one row), surfaced as the defect it is.
struct ResolutionClosureDefect {
    geometry::ShapeId orphan{};  // the reference that found no row
    std::string detail{};        // the diagnosis, the defect's own words
};

struct Resolution {
    enum class Kind : std::uint8_t {
        Resolved,       // reference_resolved   — class 1
        Renamed,        // reference_renamed    — class 2
        Ambiguous,      // reference_ambiguous  — class 3
        Vanished,       // reference_vanished   — class 4
        ClosureDefect   // closure_defect       — the guard
    };

    Kind kind{Kind::Resolved};
    ResolutionResolved resolved{};
    ResolutionRenamed renamed{};
    ResolutionAmbiguous ambiguous{};
    ResolutionVanished vanished{};
    ResolutionClosureDefect closure_defect{};
};

// kind -> the code token, one switch, the single authority: the
// four-word family registered for the fillet (ADR-0036 D6), the
// guard's own token outside it.
const char* resolution_code(Resolution::Kind k) noexcept;

// resolve — THE MACHINE. `reference` is the downstream feature's
// name (a structural ShapeId: the creating FeatureId + the path of
// role-table indices; never a string, never parsed — the round-trip
// is the pick's own answer). The old side: the tavola the reference
// was made against (+ the grammar); the new side: the re-materialized
// body (+ the grammar); the edit context: what the edit path emitted.
//
// LOUD on the input contract (the P2 culture): a missing tavola on
// the old side, a missing body on the new side, a missing grammar on
// either, a grammar the reference's path does not walk under, a root
// map the machinery cannot consume (a multi-chain/grouped reference
// — the registered boundary, named) — each throws
// std::invalid_argument with the diagnosis and the remedy. The
// resolution's OWN answers are values, never throws: the four
// classes and the guard are answers, not errors.
Resolution resolve(const geometry::ShapeId& reference,
                   const Generation& old_generation,
                   const Generation& new_generation,
                   const EditContext& edit_ctx);

// resolution_text — the report as text (the UI's error surface and
// the feature's log read this, verbatim): the code, the names, the
// cause, the remedy — one paragraph per class, the words pinned by
// the tests. Pure string building on the payload (zero
// floating-point content, ADR-0014 trivially honored).
std::string resolution_text(const Resolution& r);

} // namespace CAD_0::brep
