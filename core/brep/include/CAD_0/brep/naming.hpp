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
//     grammar for the SSI table and the boolean; the group production
//     is the morse-3 register's first gesture, derived from D1's
//     multi-chain clause and recorded in the ADR's D1 register):
//
//       name      := feature ':' role
//       feature   := kind '#' <FeatureId decimale>
//       kind      := 'box' | 'sph' | 'cyl' | 'sk' | 'ext' | 'sub'
//       role      := group? token
//                  | group? token '(' <arg> ')'       /* indexed  */
//                  | group? token '[' operands ']' ('@' level)?
//       group     := 'c' <chain group> ':'   /* ONLY multi-chain, and
//                  only the CHAIN-OWNED roles: D1 — "the chain index
//                  prefixes the role", the numbering table's own
//                  multi-chain layout (the outer first — group 0 —
//                  then each hole in sketch order). The caps are the
//                  extruded region's, never a chain's: no group. */
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
//   * THE ROOT REFERENCE (ADR-0036 D4 + the morse-3 register): the
//     tavola's FROM names the AUTHORED ROOTS — the sketch's chain
//     entities, addressed by the sketch's own layout (the chain's
//     authored position, the entity's position in the chain: "the
//     chain position IS the root name", the buck stops at authored
//     data). Its render, ONE direction as everywhere:
//
//       root      := 'sk' '#' <FeatureId> ':' rootrole
//       rootrole  := <root position>                     /* the entity:
//                        its position in its chain, a bare number —
//                        there is no vocabulary to consult */
//                  | 'c' <chain> ':' <root position>     /* ONLY when
//                        the sketch itself is multi-chain — the same
//                        only-multi-chain elision as the roles; the
//                        chain is the SKETCH's own position, not the
//                        extrude's construction group */
//                  | 'face'                              /* the sketch's
//                        FACE — the extruded region, the caps' own
//                        parent: the region is the sketch's single
//                        output, the caps' genealogy names IT */
//
//     render_root is the same pure discipline as render_name: a
//     function of its arguments, loud on nothing, NEVER parsed.
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
//
// H2-u6 morso 3 scope (D6.2 — the extrude): the extrude's walk
// tables are normative HERE (kExtFaceRoles/kExtEdgeRoles), the
// group production and the root reference are pinned (the register
// above), and the subtract's tables remain morse 4's — the grammar
// now covers them all, the tables are what the morsi add.
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
//
// The GROUP component (the morse-3 register) rides in FRONT of the
// form's own consumption, and only under a grouped registry: +1
// component, rendered 'c<group>:' before the token.
enum class RoleForm : std::uint8_t {
    Bare,
    ArgIndex,
    ArgWord,
    Relational
};

// The role's ownership class (the morse-3 register's group rule,
// ADR-0036 D1's multi-chain clause): a CHAIN-OWNED role's name
// carries the chain group's prefix when the named body materialized
// from more than one chain ('c1:lat(0)', 'c1:seam') and spends the
// path's FIRST component on the group index; a FEATURE-OWNED role
// never does — the extrude's caps belong to the extruded REGION,
// not to any one chain, and their paths go straight to the role.
enum class RoleOwner : std::uint8_t {
    Feature,  // the operation's own output — no group, ever
    Chain     // a chain group's product — the group only multi-chain
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

    // The ownership class (the morse-3 register): chain-owned roles
    // consume the group component under a grouped registry; the
    // factories' roles are all feature-owned (the default).
    RoleOwner owner{RoleOwner::Feature};
};

// The operand binding: the operand feature's id and its own naming
// declaration — the genealogy's render target. Names reference
// strictly older features (the DAG is acyclic; the registry is pure
// data, never state).
struct OperationNaming;

struct OperandBinding {
    geometry::FeatureId feature{};
    const OperationNaming* naming{};

    // THE INSTANCE'S GROUP DECLARATION (the morse-3 register's item
    // 4, landed here — H2-u6 morso 4, the subtract's own tables): a
    // relational role's OPERAND view renders the FEATURE form today;
    // when the named body materialized from MORE THAN ONE CHAIN the
    // operand view must render the GROUP form for its chain-owned
    // roles ('ext#F:c0:lat(0)' inside `isect[...]`), the path's own
    // first component the group — the binding carries the verdict,
    // the path carries the index, as in every body's own stamps. The
    // subtract's own class never produces a multi-chain operand (the
    // class checks refuse the caps' inner loops), so the flag rides
    // FALSE on every in-class render; the machinery is the landing,
    // pinned at the naming layer by morse 4's tests.
    bool multi_chain{false};
};

// The registry ONE render consults: the creating operation's kind,
// the role table for the entity kind being named (faces or edges),
// the operand bindings the relational roles read — and, since the
// morse-3 register, the INSTANCE's group declaration: true when the
// named body materialized from more than one chain (the numbering
// table's own multi-chain layout), in which case every CHAIN-OWNED
// role's path carries the chain group as its FIRST component and
// renders 'c<group>:' before its token. Single-chain bodies render
// the feature form — the prefix exists ONLY multi-chain.
struct RoleRegistry {
    FeatureKind kind{FeatureKind::Box};
    std::span<const RoleEntry> roles{};
    std::span<const OperandBinding> operands{};
    bool multi_chain{false};
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

    // The FEATURE view (the default since morso 2): the path's first
    // component is the role index — the single-chain form, and the
    // feature-owned roles' form in every body.
    constexpr RoleRegistry faces() const noexcept {
        return RoleRegistry{kind, face_roles, operands};
    }
    constexpr RoleRegistry edges() const noexcept {
        return RoleRegistry{kind, edge_roles, operands};
    }

    // The GROUP views (the morse-3 register): the multi-chain
    // instance's own — the chain-owned roles' paths carry the chain
    // group as their FIRST component ('c0:lat(3)', 'c1:seam'). A
    // feature-owned role under a group view refuses LOUDLY (the
    // caps belong to the region: render them under faces()); a
    // chain-owned path under the feature view leaves its group
    // component unconsumed and refuses the same way — the two views
    // are disjoint by construction, and the misuses are loud.
    constexpr RoleRegistry grouped_faces() const noexcept {
        return RoleRegistry{kind, face_roles, operands,
                            /*multi_chain=*/true};
    }
    constexpr RoleRegistry grouped_edges() const noexcept {
        return RoleRegistry{kind, edge_roles, operands,
                            /*multi_chain=*/true};
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
// THE NORMATIVE EXTRUDE ROLE TABLES — ADR-0036 D1 (morso 3, the walk
// roles) + the morse-3 register
// ---------------------------------------------------------------------------
//
// NORMATIVE — ADR-0036 D1; the walk orders are frozen by the goldens
// (extrude.hpp:73-93's numbering tables, the unit-3 pins). These
// tables ARE those walk tables with the vocabulary added: the index
// is the ROW of the role table the path consults, the token the
// ADR's own word for the walk's product. THE VALUES DO NOT MOVE:
// the walk indices the goldens pin are untouched — the names ride
// BESIDE them, stamped in the pass that builds.

// make_prism — FACES (D1: "lat(i) for chain entity i's lateral,
// cap(lo)/cap(hi), the circle special case's single lat(0) wall"):
// the laterals are the CHAIN ENTITIES' own walls (chain-owned: the
// multi-chain group prefixes them, 'c0:lat(i)'); the caps are the
// extruded REGION's two planar lids (feature-owned: no group, in
// any body — the caps' FROM is the sketch's face, not a chain).
inline constexpr RoleEntry kExtFaceRoles[3] = {
    {"lat", RoleForm::ArgIndex, nullptr, nullptr, {},
     RoleOwner::Chain},
    {"cap", RoleForm::ArgWord, "lo", nullptr, {}, RoleOwner::Feature},
    {"cap", RoleForm::ArgWord, "hi", nullptr, {}, RoleOwner::Feature},
};

// make_prism — EDGES: the seam-wired vertical, ADR-0023 §1's own
// e_seam convention (the solid circle's wall and every hole's wall
// are seam-wired — "the pair seam-wired is the materialized form,
// the name is one"): born of the chain's entity, chain-owned with
// the rest ('c1:seam' multi-chain). The walk's OTHER edges — the
// contour's bottom/top curves and verticals, the solid circle's rim
// halves, the holes' closed rims — are OUTSIDE D1's v1 extrude
// vocabulary: they read honestly EMPTY, never a guess (the same
// queue discipline as the vertex names, D4; the extension is a
// table row away when a consumer arrives).
inline constexpr RoleEntry kExtEdgeRoles[1] = {
    {"seam", RoleForm::Bare, nullptr, nullptr, {}, RoleOwner::Chain},
};

// The extrude's naming declaration — the sketch operand's roots are
// named by the ROOT REFERENCE below (the tavola's FROM), not by an
// operand binding: the roots are authored data, and D4's buck stops
// there; the subtract's relational roles (the operand-binding
// machinery's first consumer) arrive with their own morso.
inline constexpr OperationNaming kExtNaming{
    FeatureKind::Extrude, kExtFaceRoles, kExtEdgeRoles, {}};

// The extrude's own role-table indices, named for the construction
// sites (D1: the path's components ARE these indices — the sites
// push them beside the walk indices they already hold).
inline constexpr std::uint32_t kExtLatRole = 0;    // faces: lat(i)
inline constexpr std::uint32_t kExtCapLoRole = 1;  // faces: cap(lo)
inline constexpr std::uint32_t kExtCapHiRole = 2;  // faces: cap(hi)
inline constexpr std::uint32_t kExtSeamRole = 0;   // edges: seam

// ---------------------------------------------------------------------------
// THE NORMATIVE SUBTRACT ROLE TABLES — ADR-0036 D1 (morso 4, the
// relational roles' first real consumer) + the morse-3 register's
// landing
// ---------------------------------------------------------------------------
//
// NORMATIVE — ADR-0036 D1; the born-edge layout is frozen by the
// exact path's numbering tables (subtract.hpp's NUMBERING block, the
// unit-4 pins) read through the ADR's own grammar:
//
//   the SSI curves   'sub#F:isect[<A-face-name> × <B-entity-name>]@lo|hi'
//                     — the operands PER NAME, in the DECLARED table
//                     order, never reordered (the grammar's own rule);
//                     a LEVELLED relation is its own table row (the
//                     SSI entries' discipline: the entry names the
//                     pair AND the level; the path spends no
//                     component on it).
//   the seam          'sub#F:seam[<B-role>]' — the wall's periodic
//                     identification (ADR-0023's own e_seam
//                     convention), the operand the B entity whose
//                     wall carries it.
//
// THE FACE TABLE IS EMPTY, honestly (D1's own derivation): the exact
// subtract births no faces — the wall is the tool's own lateral
// ABSORBED (re-framed to the window; it keeps its birth name, the
// inheritance the stamping copies), the pierced faces are the
// target's own. An empty span is the honest table; there is no dummy
// row to invent (the same discipline as the extrude's single-seam
// edge table).
//
// THE SLOTS' operand order is the SSI table's own: the TARGET's
// pierced face first (bare — the box's six or the prism target's
// cap(hi), D5.4's own name), the TOOL's wall second (indexed —
// lat(i), the chain entity's own lateral).
inline constexpr OperandSlot kSubIsectSlots[2] = {
    {0, false, RoleForm::Bare},      // the target's pierced face (bare)
    {1, false, RoleForm::ArgIndex},  // the tool's wall (indexed: lat(i))
};
inline constexpr OperandSlot kSubSeamSlots[1] = {
    {1, false, RoleForm::ArgIndex},  // the wall whose seam this is
};
inline constexpr RoleEntry kSubEdgeRoles[3] = {
    {"isect", RoleForm::Relational, nullptr, "lo", kSubIsectSlots},
    {"isect", RoleForm::Relational, nullptr, "hi", kSubIsectSlots},
    {"seam",  RoleForm::Relational, nullptr, nullptr, kSubSeamSlots},
};

// The subtract's TYPE-level declaration: the operand bindings are
// INSTANCE data (the operands' feature ids flow from their own
// stamped ShapeIds; their tables are the caller's naming context —
// subtract.hpp's SubtractNaming), so the static declaration binds
// NONE; the stamping site builds the pair in the pass that builds.
inline constexpr OperationNaming kSubNaming{
    FeatureKind::Subtract, {}, kSubEdgeRoles, {}};

// The subtract's own role-table indices, named for the stamping
// sites (the levelled entries' own rows: the rim at @lo, the rim at
// @hi, the seam).
inline constexpr std::uint32_t kSubIsectLoRole = 0;  // edges: isect@lo
inline constexpr std::uint32_t kSubIsectHiRole = 1;  // edges: isect@hi
inline constexpr std::uint32_t kSubSeamRole = 2;     // edges: seam

// ---------------------------------------------------------------------------
// THE ROOT REFERENCE — the authored roots the tavola's FROM names
// (ADR-0036 D4 + the morse-3 register)
// ---------------------------------------------------------------------------

// What a FROM points at (D4: the buck stops at authored data):
// a sketch CHAIN ENTITY — the root — or the sketch's FACE, the
// extruded region (the caps' own parent; the region is the sketch's
// single output, and the caps' genealogy names it, not the chains
// that bound it).
enum class RootKind : std::uint8_t {
    Entity,  // a sketch chain entity (the authored root)
    Face     // the sketch's face — the extruded region
};

// One authored-root reference: the sketch's own feature, the kind,
// and — for the entity roots — the SKETCH's own address (the
// chain's authored position, the entity's position in the chain:
// "the chain position IS the root name", and an add-at-end edit
// shifts NOTHING). The extrude's construction GROUP (the outer
// first) is a different index that happens to coincide when the
// outer is the sketch's chain 0 — the root always speaks the
// sketch's own layout, the authored data's own order.
struct RootRef {
    geometry::FeatureId feature{};
    RootKind kind{RootKind::Entity};
    std::uint32_t chain{0};
    std::uint32_t entity{0};

    // Structural equality (the house pattern, defaulted): two
    // references are the same root iff they point at the same
    // authored thing — never the render's string.
    constexpr bool operator==(const RootRef&) const noexcept = default;
};

// render_root — the root reference's canonical render, ONE direction
// (the grammar block above): 'sk#2:0' (the single-chain sketch's
// entity 0), 'sk#2:c1:0' (the multi-chain sketch's chain 1, entity
// 0 — the chain component ONLY when the sketch itself is
// multi-chain), 'sk#2:face' (the region). `sketch_multi_chain` is
// the SKETCH's own chain count's verdict (chains.size() > 1) — the
// caller holds the sketch; the reference itself is pure structure,
// never state. Strings and integers only (ADR-0014 trivially
// honored); never parsed, never compared as a string.
std::string render_root(const RootRef& root, bool sketch_multi_chain);

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
