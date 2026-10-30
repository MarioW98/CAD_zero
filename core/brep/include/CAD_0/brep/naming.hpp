// core/brep/include/CAD_0/brep/naming.hpp
//
// THE FACTORY ROOTS' NAMES — ADR-0036 D1 (the accepted act, H2-u6
// morso 2): the address is not the identity, and the identity is a
// STRUCTURE derived from the recipe, rendered to a string by ONE
// pure function. This header is that contract's code side:
//
//   * THE ABI IS ADR-0005's OWN — `ShapeId { FeatureId creator;
//     SubshapeRef path; Generation }` (geometry/shape_id.hpp), the
//     fields shipped in Phase A and never once populated. The name
//     IS the ShapeId filled with semantics (ADR-0036 D1: the
//     FeatureId is the creating DAG node; the path's components are
//     indices into the operation type's DECLARED ROLE TABLES — the
//     numbering tables the tests already pin, made normative; the
//     Generation keeps its feature-granularity role, ratified). NO
//     parallel type, NO ABI change — the amendment fills meaning.
//   * THE CANONICAL RENDER is ONE direction: structure -> string.
//     `render_name` is a pure function of (FeatureId, SubshapeRef,
//     role registry); equality, hash and comparison are STRUCTURAL
//     (ShapeId equality), and the string exists for the register,
//     the report and the human. THE KERNEL NEVER PARSES STRINGS:
//     there is NO constructor from a name in the public surface
//     (the negative is pinned by compile-time tests), and the only
//     reverse direction is the role tables' own frozen bijection
//     (index -> token), consulted as vocabulary, never as a parser.
//   * THE GRAMMAR v1, pinned formal (ADR-0036 D1's vocabulary, one
//     grammar for the SSI table and the boolean):
//
//       name      := feature ':' role
//       feature   := kind '#' <FeatureId decimale>
//       kind      := 'box' | 'sph' | 'cyl' | 'sk' | 'ext' | 'sub'
//       role      := token
//                  | token '(' <arg> ')'            /* indexed  */
//                  | token '[' operands ']' ('@' level)?
//       operands  := name (' × ' name)*   /* space-×-space, operands
//                                            in the DECLARED table
//                                            order, never reordered */
//       token     := [a-z][a-z0-9]*       /* lowercase, no escape */
//
//     The indexed form's <arg> is the path's own integer (the
//     extrude's lat(i) — the chain entity's position, the authored
//     root's own identity per D4) or the entry's own level word
//     (cap(lo)/cap(hi)). The renderer follows the grammar TO THE
//     LETTER: D1's names are the rendering goldens, character for
//     character — `sub#4:isect[box#1:top × ext#3:lat(0)]@hi` and
//     its siblings are pinned by the morse's tests, byte for byte.
//
//   * THE ROLE TABLES are the factories' construction tables made
//     NORMATIVE (below): the indices are those the goldens pin —
//     THE VALUES DO NOT MOVE. The act adds the VOCABULARY (index ->
//     token); it changes no number, and every existing golden,
//     count and positional query keeps working unchanged (the
//     address plane is untouched by construction).
//
// Purity (D1, the recipe doctrine's consequence): two identical
// calls of a factory produce identical names — no external input,
// no global counter, no state; a name that differs after a rebuild
// is a defect, not an event. Determinism (ADR-0014) is honored by
// construction: names are strings and integers, ZERO floating-point
// content anywhere in this header's data or render.
//
// H2-u6 morso 2 scope (ADR-0036 D6.1 — the roots): the three
// factories' tables are normative HERE; the extrude's and the
// subtract's tables arrive with their morsi (D6 rungs 2-3) — the
// renderer and the grammar are general from day one, pinned by the
// rendering goldens, so those morsi pin tables, not syntax.
#pragma once

#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace CAD_0::brep::naming {

// ---------------------------------------------------------------------------
// The grammar's vocabulary
// ---------------------------------------------------------------------------

// The feature kinds of the v1 grammar (ADR-0036 D1). The sketch is
// 'sk' — the authored roots' own feature (D4: the chain position IS
// the root name; the buck stops at authored data).
enum class FeatureKind : std::uint8_t {
    Box,       // "box" — the box factory
    Sphere,    // "sph" — the seam-wired sphere factory (ADR-0023 §1)
    Cylinder,  // "cyl" — the seam-wired cylinder factory (ADR-0023 §1)
    Sketch,    // "sk"  — the sketch document (the roots' own feature)
    Extrude,   // "ext" — the extrude operation
    Subtract   // "sub" — the exact subtract operation
};

// kind -> token, one switch, the single authority (the family_name
// precedent): a typo in a case label is a compile error.
const char* kind_token(FeatureKind k) noexcept;

// The role forms the grammar pins (see the grammar above). The form
// declares how many path components the role consumes and how they
// render — the renderer's consumption table:
//
//   Bare        token                          — 1 component
//   ArgIndex    token(<path integer>)          — 2 components
//   ArgWord     token(<the entry's own word>)  — 1 component
//   Relational  token[name × name](@level)?    — 1 + the operands'
//                 declared sub-paths (the genealogy, depth-bounded
//                 by SubshapeRef::kMaxDepth = 8)
enum class RoleForm : std::uint8_t {
    Bare,
    ArgIndex,
    ArgWord,
    Relational
};

// One operand slot of a relational role: which operand binding the
// sub-path belongs to (an index into the registry's operands), the
// operand's table the sub-path indexes (edges vs faces), and the
// FORM the operand's own role renders under — the declaration the
// renderer verifies against the operand's entry (a path that
// disagrees with the declared table is a defect, loud).
struct OperandSlot {
    std::uint8_t operand{0};
    bool edges{false};
    RoleForm form{RoleForm::Bare};
};

// One declared role — one row of a normative table.
struct RoleEntry {
    // [a-z][a-z0-9]* — the grammar's token, lowercase, no escape.
    const char* token{""};

    // The grammar alternative this role renders under.
    RoleForm form{RoleForm::Bare};

    // ArgWord's own argument (cap's "lo"/"hi") — nullptr otherwise.
    const char* word{nullptr};

    // The relational form's '@' level suffix ("lo"/"hi") — nullptr
    // when the role carries none. A LEVELLED relation is its own
    // table entry (the SSI entries' own discipline: the entry id
    // names the pair AND the level; the path spends no component
    // on it).
    const char* level{nullptr};

    // The relational form's operand slots, in DECLARED order — the
    // renderer never reorders (the grammar's own rule).
    std::span<const OperandSlot> slots{};
};

// The operand binding: the operand feature's id and its own naming
// declaration — the genealogy's render target. Names reference
// strictly older features (the DAG is acyclic; the registry is pure
// data, never state).
struct OperationNaming;

struct OperandBinding {
    geometry::FeatureId feature{};
    const OperationNaming* naming{};
};

// The registry ONE render consults: the creating operation's kind,
// the role table for the entity kind being named (faces or edges),
// and the operand bindings the relational roles read.
struct RoleRegistry {
    FeatureKind kind{FeatureKind::Box};
    std::span<const RoleEntry> roles{};
    std::span<const OperandBinding> operands{};
};

// The full naming declaration of one operation type: both role
// tables plus the operand bindings. The factories' declarations are
// the normative tables below; the operations' arrive with their
// morsi, declaring their own.
struct OperationNaming {
    FeatureKind kind{FeatureKind::Box};
    std::span<const RoleEntry> face_roles{};
    std::span<const RoleEntry> edge_roles{};
    std::span<const OperandBinding> operands{};

    constexpr RoleRegistry faces() const noexcept {
        return RoleRegistry{kind, face_roles, operands};
    }
    constexpr RoleRegistry edges() const noexcept {
        return RoleRegistry{kind, edge_roles, operands};
    }
};

// ---------------------------------------------------------------------------
// THE NORMATIVE FACTORY ROLE TABLES — ADR-0036 D1 (morso 2, the roots)
// ---------------------------------------------------------------------------
//
// NORMATIVE — ADR-0036 D1; the numbering is frozen by the goldens.
// These tables ARE the factories' construction tables (the comment
// blocks of factory.hpp) with the vocabulary added: the index is the
// position of the entity in the construction — the same number the
// existing goldens pin — and the token is the ADR's own word for it.
// THE VALUES DO NOT MOVE: an index that changed would move every
// golden that pins it, and a golden that moves is a defect (D5's
// declaration), not a re-pin.

// make_box — FACES (factory.hpp:126-131, the construction comment's
// own order and words: bottom/top/front/back/left/right, the
// outward-normal planes -Z/+Z/-Y/+Y/-X/+X).
inline constexpr RoleEntry kBoxFaceRoles[6] = {
    {"bottom", RoleForm::Bare, nullptr, nullptr, {}},
    {"top",    RoleForm::Bare, nullptr, nullptr, {}},
    {"front",  RoleForm::Bare, nullptr, nullptr, {}},
    {"back",   RoleForm::Bare, nullptr, nullptr, {}},
    {"left",   RoleForm::Bare, nullptr, nullptr, {}},
    {"right",  RoleForm::Bare, nullptr, nullptr, {}},
};

// make_box — EDGES (factory.hpp:117-120's construction comment: the
// bottom ring 0..3, the top ring 4..7, the verticals 8..11; the
// token is the ring's word with the ring position appended — the
// numbering comment's own vocabulary, token-legal).
inline constexpr RoleEntry kBoxEdgeRoles[12] = {
    {"bottom0",   RoleForm::Bare, nullptr, nullptr, {}},
    {"bottom1",   RoleForm::Bare, nullptr, nullptr, {}},
    {"bottom2",   RoleForm::Bare, nullptr, nullptr, {}},
    {"bottom3",   RoleForm::Bare, nullptr, nullptr, {}},
    {"top0",      RoleForm::Bare, nullptr, nullptr, {}},
    {"top1",      RoleForm::Bare, nullptr, nullptr, {}},
    {"top2",      RoleForm::Bare, nullptr, nullptr, {}},
    {"top3",      RoleForm::Bare, nullptr, nullptr, {}},
    {"vertical0", RoleForm::Bare, nullptr, nullptr, {}},
    {"vertical1", RoleForm::Bare, nullptr, nullptr, {}},
    {"vertical2", RoleForm::Bare, nullptr, nullptr, {}},
    {"vertical3", RoleForm::Bare, nullptr, nullptr, {}},
};

// make_sphere — the ADR-0023 §1 convention's own names, cited:
// F = {f_sphere} (the one spherical face, the seam-wired disk of
// the slit rectangle [0,2pi]x[0,pi]); E = {e_seam} (the meridian
// seam, north -> south). Vertex names are OUT of v1 (ADR-0036 D4's
// queue: no consumer today).
inline constexpr RoleEntry kSphereFaceRoles[1] = {
    {"sphere", RoleForm::Bare, nullptr, nullptr, {}},
};
inline constexpr RoleEntry kSphereEdgeRoles[1] = {
    {"seam", RoleForm::Bare, nullptr, nullptr, {}},
};

// make_cylinder — the ADR-0023 §1 convention's own names, cited:
// F = {f_lateral, f_top (+Y plane), f_bot (-Y plane)}; E = {e_top1
// (-Z half), e_top2 (+Z half), e_bot1 (+Z half), e_bot2 (-Z half),
// e_seam (the vertical at u=0)} — the four rim arcs are the
// CONSTRUCTION entities, each named beside the seam (no anonymous
// citizens); the LOGICAL one-name rim circle is the SUBTRACT's born
// role (morso 4's territory), not the factory's.
inline constexpr RoleEntry kCylinderFaceRoles[3] = {
    {"lateral", RoleForm::Bare, nullptr, nullptr, {}},
    {"top",     RoleForm::Bare, nullptr, nullptr, {}},
    {"bot",     RoleForm::Bare, nullptr, nullptr, {}},
};
inline constexpr RoleEntry kCylinderEdgeRoles[5] = {
    {"top1", RoleForm::Bare, nullptr, nullptr, {}},
    {"top2", RoleForm::Bare, nullptr, nullptr, {}},
    {"bot1", RoleForm::Bare, nullptr, nullptr, {}},
    {"bot2", RoleForm::Bare, nullptr, nullptr, {}},
    {"seam", RoleForm::Bare, nullptr, nullptr, {}},
};

// The factories' naming declarations — no operands, the pure
// bootstrap: every entity's name is the feature and the table index
// already in hand at the construction site.
inline constexpr OperationNaming kBoxNaming{
    FeatureKind::Box, kBoxFaceRoles, kBoxEdgeRoles, {}};
inline constexpr OperationNaming kSphereNaming{
    FeatureKind::Sphere, kSphereFaceRoles, kSphereEdgeRoles, {}};
inline constexpr OperationNaming kCylinderNaming{
    FeatureKind::Cylinder, kCylinderFaceRoles, kCylinderEdgeRoles, {}};

// ---------------------------------------------------------------------------
// THE CANONICAL RENDER — structure -> string, ONE direction
// ---------------------------------------------------------------------------

// render_name — the canonical render of a ShapeId's (FeatureId,
// SubshapeRef) pair under the operation's declared role table.
//
// PURE: a function of its three arguments and nothing else — no
// state, no memoization, no environment; the same triple renders
// the same bytes on every call, every platform (ADR-0014 honored
// trivially: strings and integers only). The factories call it in
// the pass that builds (the name is computed at materialization,
// zero extra passes); the tests call it to pin the goldens.
//
// LOUD on disagreement, the P2 culture: an empty or over-deep path,
// an index outside the declared table, a path that leaves
// components unconsumed, an operand slot the registry does not
// bind, an operand entry whose form disagrees with the slot's
// declaration — each throws std::invalid_argument naming the index,
// the table and the remedy. A silent wrong render would be a name
// that lies; this kernel's names do not lie.
std::string render_name(const geometry::FeatureId& feature,
                        const geometry::SubshapeRef& ref,
                        const RoleRegistry& registry);

// The role tables' own frozen bijection, the VOCABULARY reverse:
// the token's index in the declared table (the round-trip's name ->
// SubshapeRef leg — a table lookup, ONE word, never a string
// parser; the full name never crosses back). Returns the index, or
// throws std::invalid_argument naming the token and the table when
// the token is not the table's own (a foreign token is a caller
// bug, loud).
std::uint32_t role_index_of(std::span<const RoleEntry> roles,
                            const std::string& token);

} // namespace CAD_0::brep::naming
