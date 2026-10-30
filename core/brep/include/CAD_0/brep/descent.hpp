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
// arrives with the operations' morsi, which have operands; this
// morse's check is the RESULT side, and it says so honestly.
//
// THE COST (D2's derivation, verified by the type): one row per
// output entity — O(|V|+|E|+|F|) of the result — integers and
// strings, ZERO floating-point content (ADR-0014 honored by
// construction; DescentRow's own fields are the proof).
#pragma once

#include "CAD_0/brep/topology.hpp"

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
struct DescentRow {
    Descent relation{Descent::Born};
    EntityKind entity{EntityKind::Face};

    // The result entity's POSITIONAL id (the address — the tavola
    // relates identities but it BOOKS them by address, the body's
    // own O(1) key; the name field carries the identity).
    std::uint64_t address{0};

    // The operand entities this row comes from, as addresses in the
    // operands' own bodies — empty for born, the parent for
    // kept/absorbed, the parents for split/merged, the dead entity
    // for vanished.
    std::vector<std::uint64_t> operands{};

    // The result entity's NAME (the canonical render) — faces and
    // edges of the named classes; empty where v1 has no names
    // (vertices, the not-yet-instrumented operations' born rows).
    std::string name{};

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
// RESULT side (this morse's scope, said honestly): every vertex,
// edge and face of the body appears in EXACTLY ONE row of the
// tavola, and no row books an entity the body does not have (a
// ghost row is the same defect, read from the other side). Returns
// true when closed; otherwise returns false and, when `why` is
// non-null, writes the diagnosis naming the entity (kind + address)
// and the remedy (the tavola's own fix — a lost row is a
// bookkeeping defect at the site that knows the facts).
bool descent_closed(const BRepBody& body, const DescentTable& table,
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

} // namespace CAD_0::brep
