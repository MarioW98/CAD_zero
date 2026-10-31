// core/brep/include/CAD_0/brep/descent.hpp
//
// THE TAVOLA — ADR-0036 D2 (the accepted act, H2-u6 morso 2): every
// operation, beside the body, emits the descent table; and the
// factories' tavola is the trivial all-born case, DERIVED and never
// stored (D4: pure bookkeeping, recomputed from the body on every
// call — the recipe is the source of truth, the tavola is cache).
//
// THE SIX RELATIONS, named and closed (D2's table, derived from
// what the operations actually do — the vocabulary covers the
// genuine cases, and the closure property demands it be closed):
//
//   born      created by this operation
//   kept      survives 1:1, untouched — the operand's own records
//             all survive with it (the Layer-2 ownership bill)
//   absorbed  survives 1:1, MODIFIED — new loops, a re-framed
//             window, a trimmed patch; the operation's mark rides
//   split     one in -> k out
//   merged    k in -> one out
//   vanished  dies in this operation, WITH THE CAUSE recorded
//
// THE CLOSURE PROPERTY, hard and active from day one (the Layer-1
// spirit applied to the history): every entity of the result
// appears in EXACTLY ONE row — a tavola that loses an entity, or
// double-books one, is a defect the check names, never a silent
// gap. The full two-sided closure (the OLD generation's entities
// each in exactly one kept/absorbed/split/merged/vanished row)
// ARRIVED with morso 4 (the subtract's rows — old_generation_closed,
// below); the morsi 2-3 tavole carried the result side alone, and
// said so honestly.
//
// THE FROM (H2-u6 morso 3, the extrude's tavola — the first rows
// that HAVE one): a born row's `from` carries the authored ROOT
// the entity descends from — the sketch chain entity, or the
// sketch's face for the caps (naming.hpp's RootRef, D4 + the
// morse-3 register) — and the FROM side of the closure goes active
// with it: every root reference must point at an entity the
// upstream sketch REALLY has (a dangling root is the suspended
// reference the rule exists to forbid — extrude_froms_resolve,
// below). The body-address operands (the subtract's operands —
// morso 4's rows, the structured OperandRef) ride the `operands`
// field: the two channels serve the two operand classes, each named
// at its site — and the subtract's FROM-exists-upstream side is
// old_generation_closed's own (the refs' existence check).
//
// THE COST (D2's derivation, verified by the type): one row per
// output entity — O(|V|+|E|+|F|) of the result — integers and
// strings, ZERO floating-point content (ADR-0014 honored by
// construction; DescentRow's own fields are the proof).
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace CAD_0::brep {

// The six relations of D2, named and closed.
enum class Descent : std::uint8_t {
    Born,      // created by this operation
    Kept,      // survives 1:1, untouched
    Absorbed,  // survives 1:1, modified (the operation's mark rides)
    Split,     // one in -> k out
    Merged,    // k in -> one out
    Vanished   // dies in this operation (the cause recorded)
};

// relation -> token, one switch, the single authority.
const char* descent_token(Descent r) noexcept;

// The entity kinds a row can speak of. Vertex rows are part of the
// tavola from day one (D2's 47-row guide-case table counts them);
// vertex NAMES are out of v1 (ADR-0036 D4's queue — no consumer
// today), so a vertex row's name field reads empty, honestly.
enum class EntityKind : std::uint8_t {
    Vertex,
    Edge,
    Face
};

// One row of the tavola: the result entity (kind + address) and
// where it comes from. Integers and strings only — the cost
// derivation's own proof (zero floating-point content, ADR-0014 by
// construction).
//
// H2-u6 morso 4 (the subtract's rows — the body-address channel's
// first consumers): the `operands` field carries STRUCTURED refs —
// {which operand (0 = A the target, 1 = B the tool), the entity's
// kind, its address in that operand's OWN body} — the ambiguity a
// flat address cannot carry (A's face 2 and B's face 2 are different
// entities; the guide case books both), made explicit. The kept and
// the absorbed rows carry exactly one ref (the parent); the vanished
// rows exactly one (the dead entity, the row's own `address` the
// dead entity's address in its own body, the agreement checked); the
// born rows carry NONE — the old generation is booked by the
// kept/absorbed/split/merged/vanished rows only (D2's closure's own
// reading), and a born row's genealogy rides its NAME (D1: the
// relation is part of the role).
struct OperandRef {
    std::uint8_t operand{0};
    EntityKind kind{EntityKind::Face};
    std::uint64_t address{0};

    constexpr bool operator==(const OperandRef&) const noexcept = default;
};

struct DescentRow {
    Descent relation{Descent::Born};
    EntityKind entity{EntityKind::Face};

    // The row's subject's POSITIONAL id — for a result row (born /
    // kept / absorbed / split / merged) the address in the RESULT; for
    // a VANISHED row the dead entity's address in its OWN body (the
    // row has no result entity: the result-side closure skips the
    // vanished rows, the old-generation check books them).
    std::uint64_t address{0};

    // The operand entities this row books, as structured refs in the
    // operands' own bodies — empty for born, the parent for
    // kept/absorbed, the parents for split/merged, the dead entity
    // for vanished. The BODY-ADDRESS channel (the subtract's rows,
    // morse 4).
    std::vector<OperandRef> operands{};

    // The result entity's NAME (the canonical render) — faces and
    // edges of the named classes; empty where v1 has no names
    // (vertices, the not-yet-instrumented operations' born rows).
    std::string name{};

    // THE FROM (morso 3, the extrude's rows — the first that have
    // one): the AUTHORED ROOT the row descends from, structured
    // (naming::RootRef — the sketch chain entity, or the sketch's
    // face for the caps). Empty on the factories' rows (no
    // operands) and on the rows whose operands are body addresses
    // (those ride `operands`). The render — render_root — serves
    // the register, the report and the human; the check reads the
    // STRUCTURE, never the string.
    std::vector<naming::RootRef> from{};

    // Vanished rows: WHAT died and WHY — the tavola's own answer to
    // "which decision killed it" (D3's class-4 payload begins here).
    std::string cause{};
};

// The tavola itself: rows in the body's own deterministic order
// (vertices, then edges, then faces — the derivation's own walk).
struct DescentTable {
    std::vector<DescentRow> rows{};

    // How many rows carry this relation (the cost answer, queryable
    // — the report's own arithmetic, never a guess).
    std::size_t count(Descent relation) const noexcept;
};

// descent_closed — the CLOSURE CHECK, D2's hard invariant on the
// RESULT side: every vertex, edge and face of the body appears in
// EXACTLY ONE row of the tavola, and no row books an entity the body
// does not have (a ghost row is the same defect, read from the other
// side). Returns true when closed; otherwise returns false and, when
// `why` is non-null, writes the diagnosis naming the entity (kind +
// address) and the remedy (the tavola's own fix — a lost row is a
// bookkeeping defect at the site that knows the facts).
//
// H2-u6 morso 4: the VANISHED rows are SKIPPED on this side — they
// book the OLD generation (their subject is a dead operand entity,
// addressed in its own body), and the old generation's own check
// (old_generation_closed, below) books them there. A tavola with
// vanished rows would double-book the result's addresses otherwise
// (the vanished row's address is the DEAD entity's, in another body's
// numbering) — the skip is the two-sided closure's own rule, said
// here where it bites.
bool descent_closed(const BRepBody& body, const DescentTable& table,
                    std::string* why = nullptr);

// old_generation_closed — the OLD side of the closure (D2, morso 4:
// the two-sided closure arrives with the first operation that has
// operands): every entity of the OLD generation — A's and B's —
// appears in EXACTLY ONE row's `operands` (the kept/absorbed parent,
// the vanished dead, the split/merged parents; the born rows carry
// none by construction); every ref points at an entity the named
// operand body REALLY has (a dangling ref is the suspended reference
// the rule exists to forbid — the FROM-exists-upstream side); and a
// vanished row's own `address` agrees with its dead ref's (a row
// that disagrees with itself is a bookkeeping defect, loud). Returns
// true when closed; otherwise returns false and, when `why` is
// non-null, writes the diagnosis naming the entity (which body, kind
// + address) and the remedy.
bool old_generation_closed(const BRepBody& A, const BRepBody& B,
                            const DescentTable& table,
                            std::string* why = nullptr);

// factory_descent — the factories' tavola, DERIVED and never stored
// (ADR-0036 D4): one born row per produced entity — the vertices,
// the edges, the faces, in the body's own order, zero operands (a
// factory has none), the names the factory stamped beside the
// addresses. Recomputed deterministically on every call: the same
// body yields the same table, byte for byte (integers and strings,
// ADR-0014 trivially honored).
//
// In debug builds the closure check runs INSIDE (a tavola that
// loses an entity cannot pass silent): a violation throws
// std::logic_error carrying descent_closed's own diagnosis.
DescentTable factory_descent(const BRepBody& body);

// extrude_descent — THE EXTRUDE'S TAVOLA (ADR-0036 D2+D4, H2-u6
// morso 3): one born row per entity of the result, each carrying
// its FROM — the authored root (the sketch chain entity that owns
// the wall, the curve, the joint; the sketch's face for the caps).
// DERIVED, never stored: recomputed from the recipe with the SAME
// one-opinion functions the materialization itself consulted
// (extrude_detail's outer/hole judgment, the numbering tables' own
// layout), so the derivation and the construction cannot drift —
// and the drift-guard is ACTIVE: every derived name is verified
// against the body's own stamped name (a mismatch throws
// std::logic_error naming the entity — a tavola that lies about
// the body it describes is a defect, not a cache).
//
// `feature` is the extrude's own FeatureId (the body's names were
// stamped with it — the agreement check pins it); `sketch` is the
// SKETCH's FeatureId, the FROM references' own feature.
//
// In debug builds the result-side closure check runs INSIDE (the
// factory precedent); extrude_froms_resolve — the FROM side — is
// the caller's to call (and the tests' to pin).
DescentTable extrude_descent(
    const BRepBody& body, const std::vector<sdf::SketchChain>& chains,
    const geometry::FeatureId& feature,
    const geometry::FeatureId& sketch);

// extrude_froms_resolve — the FROM side of the closure (D2, active
// where the tavola first has FROMs): every root reference of every
// row points at an entity the upstream sketch REALLY has — the
// entity roots at a live (chain, position), the face root at a
// sketch that has chains at all. A dangling root is the suspended
// reference the rule exists to forbid: returns false and, when
// `why` is non-null, writes the diagnosis naming the dangling root
// (its canonical render) and the remedy. The check reads the
// STRUCTURE (never the render); the sketch's own chain count
// supplies the render's multi-chain verdict.
bool extrude_froms_resolve(const DescentTable& table,
                           const std::vector<sdf::SketchChain>& chains,
                           std::string* why = nullptr);

// subtract_descent — THE EXACT SUBTRACT'S TAVOLA (ADR-0036 D2, H2-u6
// morso 4): the six relations at work, the guide case's full table
// the golden (47 rows — D2's own count: 36 out + 11 vanished).
// DERIVED from the result's own topology and the operands' own
// records — the recording of the facts the assembly already holds,
// read back ("no new geometry, no new decisions: pure bookkeeping",
// D2's own rule):
//
//   * the KEPT rows — A's entities that survive 1:1 untouched (the
//     four sides, the twelve edges, the eight corners of the guide
//     case); their Layer-2 records are the operand's own, untouched
//     (the copy carried them verbatim);
//   * the ABSORBED rows — the pierced faces (A's own, their inner
//     loops born of the rim) and the walls (the tool's own laterals,
//     re-framed to the window — the row's operand the tool's lateral
//     face, found by the GENEALOGICAL correspondence tool_lateral_of:
//     an edge reference, never a geometric signature); the wall's
//     Layer-2 record is the operation's own (combined_provenance at
//     the creation site) — the bill the kept/absorbed split exists
//     to make checkable;
//   * the BORN rows — the result's new entities (the rim circles'
//     materialized halves and the seam, the split vertices); their
//     names read from the body's own stamps (the factory precedent —
//     the names were stamped by the same pass that built, the
//     tavola reports them, nothing to re-derive); no operands (the
//     born row's genealogy rides its name);
//   * the VANISHED rows — the tool's entities that die (the caps cut
//     away outside the target — the flush variant the same cause
//     covers; the rims, the seam and the split vertices re-authored
//     with the window), each with the CAUSE recorded and the dead
//     entity's own name; their `address` is the dead entity's, in
//     the tool's own body;
//   * the MISS (the tool disjoint from the target): every A entity
//     kept, every B entity vanished — the cause "the miss: nothing
//     removed (the exact copy stands)"; the laterals die with the
//     caps in a miss (nothing of B survives), said here where it
//     differs from the through;
//   * MERGED: NO instance in the exact path, and that is a VERIFIED
//     FACT, not a gap (the flush cap is a 1:1 + 1:0 — "the tool's
//     caps die (a flush cap absorbed — nothing is emitted for them,
//     which IS the dedup)", the assembly's own head comment,
//     subtract.hpp:1200 in this morse's file — the ADR's D2 cites
//     the same words at its verification's own line); the class
//     exists because the closure property demands the vocabulary
//     cover the genuine k -> 1.
//
// The rows' order: the RESULT's own walk (vertices, edges, faces —
// the derivation's own order) then the VANISHED tail in the dying
// operand's own order (vertices, edges, faces). In debug builds BOTH
// closure checks run INSIDE (the factory precedent): the result side
// and the old side, each violation throwing with its own diagnosis.
DescentTable subtract_descent(const BRepBody& result, const BRepBody& A,
                              const BRepBody& B);

} // namespace CAD_0::brep
