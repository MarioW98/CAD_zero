// core/brep/src/resolve.cpp
//
// The resolver's implementation (ADR-0036 D3 + D6.4, H2-u6 morso 5):
// three pure steps over integers and strings — the guard, the
// re-derivation (the roots rule inside), the descent (the skeleton
// candidates). Zero floating-point content, zero state, zero
// parsing: the paths walk the SAME normative tables the renderer
// consults (naming.cpp's own consumption, mirrored and classified),
// and every comparison is structural (ShapeId's own operators).
#include "CAD_0/brep/resolve.hpp"

#include <sstream>
#include <stdexcept>

namespace CAD_0::brep {

// ---------------------------------------------------------------------------
// The codes — the family registered for the fillet (ADR-0036 D6)
// ---------------------------------------------------------------------------

const char* resolution_code(Resolution::Kind k) noexcept {
    switch (k) {
    case Resolution::Kind::Resolved: return "reference_resolved";
    case Resolution::Kind::Renamed:  return "reference_renamed";
    case Resolution::Kind::Ambiguous:
        return "reference_ambiguous";
    case Resolution::Kind::Vanished: return "reference_vanished";
    case Resolution::Kind::ClosureDefect: return "closure_defect";
    }
    return "reference_resolved";  // unreachable — all cases handled
}

// ---------------------------------------------------------------------------
// The edit context
// ---------------------------------------------------------------------------

const naming::RootRef* EditContext::mapped_root(
    const naming::RootRef& old_root) const noexcept {
    for (const auto& [from, to] : root_map) {
        if (from.kind == old_root.kind && from.chain == old_root.chain &&
            from.entity == old_root.entity) {
            return &to;
        }
    }
    return nullptr;
}

std::string EditContext::map_text() const {
    std::ostringstream os;
    bool first = true;
    for (const auto& [from, to] : root_map) {
        if (!first) os << ", ";
        first = false;
        os << naming::render_root(from, /*sketch_multi_chain=*/false)
           << " -> "
           << naming::render_root(to, /*sketch_multi_chain=*/false);
    }
    return os.str();
}

// ---------------------------------------------------------------------------
// The grammar walk — the renderer's mirror, classifying instead of
// rendering (the SAME consumption naming.cpp practices, the same
// loud refusals)
// ---------------------------------------------------------------------------

namespace {

// One walked path: the ROLE components' values (the skeleton — the
// structural spine, compared), the ROOT components (the authored
// positions, the roots rule's own), the GROUP components (the
// multi-chain layout's indices) with their positions (the remap's
// own map back into the path).
struct PathWalk {
    std::vector<std::uint32_t> skeleton{};
    std::vector<std::uint32_t> roots{};
    std::vector<std::uint32_t> groups{};
    std::vector<std::uint8_t> root_positions{};

    bool root_dependent() const noexcept {
        return !roots.empty() || !groups.empty();
    }
};

[[noreturn]] void refuse(const std::string& what) {
    throw std::invalid_argument("resolve: " + what);
}

void walk_role(const geometry::SubshapeRef& ref, std::size_t& pos,
               const naming::RoleRegistry& registry, PathWalk& out) {
    if (pos >= ref.depth) {
        refuse("the reference's path ends inside a role — the "
               "SubshapeRef has " +
               std::to_string(ref.depth) + " component(s) and the role "
               "at depth " + std::to_string(pos) + " needs one; remedy: "
               "the reference is the pick's own structural answer (the "
               "ShapeId the query surfaces carry) — a hand-built path "
               "must carry a component per role it crosses, exactly as "
               "the materialization built it (ADR-0036 D1).");
    }

    // THE GROUP component (the morse-3 register): under a GROUPED
    // registry the path's first component is the chain group — the
    // numbering table's own layout index, FREE in the skeleton (it
    // shifts with the sketch's chains, exactly as the roots do).
    if (registry.multi_chain) {
        out.groups.push_back(ref.path[pos]);
        ++pos;
        if (pos >= ref.depth) {
            refuse("the reference's path ends inside a group-prefixed "
                   "role — the group component was consumed and the "
                   "role index is missing; remedy: a chain-owned role's "
                   "path under the group view is [group, role index, "
                   "the role's own components] — the pick's structural "
                   "answer carries it whole.");
        }
    }

    const std::uint32_t idx = ref.path[pos];
    if (idx >= registry.roles.size()) {
        refuse("role index " + std::to_string(idx) +
               " is outside the declared table (" +
               std::to_string(registry.roles.size()) +
               " roles); remedy: the reference's grammar is the "
               "CREATING operation's own — pass the declaration the "
               "reference was rendered under (the feature's own "
               "tables for its born entities, the original creator's "
               "for an inherited one), never a foreign registry.");
    }
    const naming::RoleEntry& entry = registry.roles[idx];
    out.skeleton.push_back(idx);
    ++pos;

    switch (entry.form) {
    case naming::RoleForm::Bare:
        return;  // one component, structural

    case naming::RoleForm::ArgIndex: {
        if (pos >= ref.depth) {
            refuse("the indexed role '" + std::string(entry.token) +
                   "' demands its integer argument as the next path "
                   "component, and the path ends; remedy: an ArgIndex "
                   "role consumes two components — the role index and "
                   "the argument (the chain entity's own position, the "
                   "authored root per ADR-0036 D4).");
        }
        // THE ROOT — the authored position: the component the roots
        // rule exists for. FREE in the skeleton (an edit that shifts
        // the roots shifts it), the remap's own target.
        out.roots.push_back(ref.path[pos]);
        out.root_positions.push_back(static_cast<std::uint8_t>(pos));
        ++pos;
        return;
    }

    case naming::RoleForm::ArgWord:
        return;  // the entry's own word — no component spent

    case naming::RoleForm::Relational: {
        for (const naming::OperandSlot& slot : entry.slots) {
            if (slot.operand >= registry.operands.size()) {
                refuse("the relational role '" +
                       std::string(entry.token) + "' reads operand "
                       "binding " + std::to_string(slot.operand) +
                       " and the registry binds " +
                       std::to_string(registry.operands.size()) +
                       "; remedy: the reference's grammar is the "
                       "INSTANCE's own — the operand bindings the "
                       "stamping site held (the m4 SubRegistry shape), "
                       "complete or the walk cannot see the legs.");
            }
            const naming::OperandBinding& binding =
                registry.operands[slot.operand];
            if (binding.naming == nullptr) {
                refuse("the relational role '" +
                       std::string(entry.token) + "' names an unbound "
                       "operand; remedy: bind the operand's own naming "
                       "declaration (its tables and its feature id) "
                       "before resolving — the genealogy's legs walk "
                       "through it.");
            }
            const naming::RoleRegistry operand_view{
                binding.naming->kind,
                slot.edges ? binding.naming->edge_roles
                           : binding.naming->face_roles,
                binding.naming->operands,
                binding.multi_chain};
            walk_role(ref, pos, operand_view, out);
        }
        return;
    }
    }
}

PathWalk walk_path(const geometry::SubshapeRef& ref,
                   const naming::RoleRegistry& registry) {
    if (ref.depth == 0) {
        refuse("the reference's SubshapeRef is empty — an unnamed "
               "entity has no identity to resolve; remedy: the "
               "reference is the pick's own structural answer (the "
               "ShapeId beside the name, the query surfaces' own "
               "field) — an honestly unnamed entity (the v1 queues) "
               "cannot be referenced by name.");
    }
    PathWalk out;
    std::size_t pos = 0;
    walk_role(ref, pos, registry, out);
    if (pos != ref.depth) {
        refuse("the reference's path leaves " +
               std::to_string(ref.depth - pos) +
               " component(s) unconsumed under the declared grammar; "
               "remedy: the path is indices into the DECLARED tables "
               "(ADR-0036 D1) — pass the registry the reference was "
               "rendered under, never a foreign one.");
    }
    return out;
}

// ---------------------------------------------------------------------------
// The new body's stamps — the re-derivation's own material
// ---------------------------------------------------------------------------

const char* entity_token(EntityKind kind) noexcept {
    switch (kind) {
    case EntityKind::Vertex: return "vertex";
    case EntityKind::Edge:   return "edge";
    case EntityKind::Face:   return "face";
    }
    return "face";  // unreachable — all enum cases handled above
}

// One entity found by identity: the name rides with it (the payload
// renders what the user reads). The GENERATION field is not compared
// — D1's own derivation: the four classes supersede the bare
// staleness check (a stale reference deserves the WHICH, not a
// boolean).
struct FoundId {
    bool found{false};
    std::string name{};
};

FoundId find_by_id(const BRepBody& body, EntityKind kind,
                   const geometry::ShapeId& id) {
    FoundId out;
    if (kind == EntityKind::Edge) {
        for (const auto& e : body.edges) {
            if (e.shape_id().feature == id.feature &&
                e.shape_id().subshape == id.subshape) {
                out.found = true;
                out.name = e.name();
                return out;
            }
        }
    } else if (kind == EntityKind::Face) {
        for (const auto& f : body.solid.shell().faces()) {
            if (f.shape_id().feature == id.feature &&
                f.shape_id().subshape == id.subshape) {
                out.found = true;
                out.name = f.name();
                return out;
            }
        }
    }
    return out;
}

// The generation's registry view for one entity kind (the instance's
// chain-count verdict selects it — the same view the stamps were
// rendered under).
naming::RoleRegistry registry_of(const Generation& g, EntityKind kind) {
    const bool edges = kind == EntityKind::Edge;
    if (g.multi_chain) {
        return edges ? g.naming->grouped_edges()
                     : g.naming->grouped_faces();
    }
    return edges ? g.naming->edges() : g.naming->faces();
}

// The row's subject, named — the payload's own word for what died
// (the row's name when the class carries one, the kind + address
// otherwise, honestly).
std::string row_subject(const DescentRow& row) {
    if (!row.name.empty()) return row.name;
    return std::string("the unnamed ") + entity_token(row.entity) +
           " " + std::to_string(row.address);
}

} // namespace

// ---------------------------------------------------------------------------
// resolve — the three steps and the guard
// ---------------------------------------------------------------------------

Resolution resolve(const geometry::ShapeId& reference,
                   const Generation& old_generation,
                   const Generation& new_generation,
                   const EditContext& edit_ctx) {
    // THE INPUT CONTRACT, loud (the P2 culture): each side carries
    // what its own steps consume — the old side the tavola and the
    // grammar, the new side the body and the grammar.
    if (old_generation.tavola == nullptr) {
        refuse("the old generation carries no tavola — the guard and "
               "the descent read the rows (D2: the tavola lives beside "
               "the last materialized body); remedy: pass the descent "
               "table of the generation the reference was made against "
               "(the derivation functions emit it, pure and "
               "deterministic).");
    }
    if (old_generation.naming == nullptr ||
        new_generation.naming == nullptr) {
        refuse("a generation carries no naming declaration — the "
               "reference's grammar is the walk's own table (the "
               "creating operation's tables, the operand bindings the "
               "stamping site held); remedy: pass the declaration the "
               "reference was rendered under on BOTH sides.");
    }
    if (new_generation.body == nullptr) {
        refuse("the new generation carries no body — the re-derivation "
               "reads the new materialization's own stamps (the names "
               "are a pure function of the recipe, and the stamps ARE "
               "the re-derivation); remedy: pass the re-materialized "
               "output of the referenced feature.");
    }

    // STEP 0 — THE GUARD, under all four: the reference's row in the
    // old tavola, by ShapeId equality (the STRUCTURE the rows carry
    // since the morse-5 register — never the render). No row at all
    // is a closure violation, surfaced as the defect it is.
    const DescentRow* row = nullptr;
    for (const auto& r : old_generation.tavola->rows) {
        if (r.shape_id.feature == reference.feature &&
            r.shape_id.subshape == reference.subshape) {
            row = &r;
            break;
        }
    }
    if (row == nullptr) {
        Resolution out;
        out.kind = Resolution::Kind::ClosureDefect;
        out.closure_defect.orphan = reference;
        out.closure_defect.detail =
            "the reference has no row in the old generation's tavola "
            "— a closure violation (D2's own invariant: every entity "
            "of a generation appears in exactly one row), not a user "
            "error: either the tavola lost a row (a bookkeeping "
            "defect at the derivation that walked the generation) or "
            "the reference was never made against this generation "
            "(resolve it against its own); surfaced as a defect, "
            "never a silence.";
        return out;
    }

    const EntityKind kind = row->entity;
    const naming::RoleRegistry old_registry =
        registry_of(old_generation, kind);
    const naming::RoleRegistry new_registry =
        registry_of(new_generation, kind);

    // STEP 1 — THE VANISHED ROW: the reference points at an entity
    // the old generation's OWN operation killed (the tool's caps,
    // "cut away outside the target"; the tool's rims, "re-authored
    // with the window"). The tavola's row IS the record — class 4
    // with the tavola's own cause, the row carried whole.
    if (row->relation == Descent::Vanished) {
        Resolution out;
        out.kind = Resolution::Kind::Vanished;
        out.vanished.row = *row;
        out.vanished.why = row->cause;
        out.vanished.remedy =
            "the entity died in this feature's own operation (the "
            "tavola's vanished row carries the cause — the decision "
            "that killed it); remedy: reference an entity that "
            "survives the operation (the tavola's kept/absorbed rows "
            "are the living set), or revert the edit that feeds the "
            "consumed body.";
        return out;
    }

    // The reference's grammar walk — the renderer's mirror, loud on
    // disagreement (a foreign registry, a malformed path).
    const PathWalk walk = walk_path(reference.subshape, old_registry);

    // STEP 2 — RE-DERIVATION: the structural match against the new
    // body's stamps, under the ROOTS RULE (ADR-0036 D4's amendment).
    const FoundId full =
        find_by_id(*new_generation.body, kind, reference);

    if (!walk.root_dependent()) {
        // A PURE structural name (a factory role, a cap, a seam
        // without authored legs): the match needs no cover — the
        // name's components are the tables' own, stable by
        // construction.
        if (full.found) {
            Resolution out;
            out.kind = Resolution::Kind::Resolved;
            out.resolved.id = reference;
            out.resolved.name = full.name;
            return out;
        }
        // No match: the descent (the skeleton of a pure name IS the
        // full path — its candidates can only be none).
    } else if (edit_ctx.roots_no_shift) {
        // THE NO-SHIFT DECLARATION (the v1-covered flows: add-at-end,
        // the parameter edits): the edit path declares the
        // pre-existing roots kept their positions — the structural
        // match is trusted, the reference consumed the declaration.
        if (full.found) {
            Resolution out;
            out.kind = Resolution::Kind::Resolved;
            out.resolved.id = reference;
            out.resolved.name = full.name;
            return out;
        }
        // No match under the declaration: the descent (honest — the
        // entity died some other way; the classes decide).
    } else if (!edit_ctx.root_map.empty()) {
        // THE ROOT MAP (the registered future, exercised synthetic):
        // the remap of the reference's roots through the declared
        // map, then the structural match of the REMAPPED name. The
        // machinery covers the SINGLE-CHAIN roots (the group absent
        // — the sketch-chain/construction-group coincidence of
        // naming.hpp's own note); beyond it, LOUD, never a silent
        // dodge.
        if (!walk.groups.empty()) {
            refuse("the root map is emitted for a GROUPED (multi-chain "
                   "chain-owned) reference — outside the resolver's v1 "
                   "remap machinery (the registered boundary, ADR-0036 "
                   "D3.2 + D4's queue: the map's real cases arrive with "
                   "the middle-insert/deletion maps, and the multi-chain "
                   "correspondence is priced there); remedy: resolve "
                   "per the honest classes (no map passed), or extend "
                   "the machinery with the real case — never a silent "
                   "fall-through.");
        }
        bool covered = true;
        geometry::SubshapeRef remapped = reference.subshape;
        for (std::size_t i = 0; i < walk.roots.size(); ++i) {
            const naming::RootRef old_root{
                {}, naming::RootKind::Entity, 0, walk.roots[i]};
            const naming::RootRef* mapped = edit_ctx.mapped_root(old_root);
            if (mapped == nullptr) {
                // An absent root is not a death sentence — the map
                // does not cover it, the descent decides (never a
                // list-diff guess).
                covered = false;
                break;
            }
            if (mapped->kind != naming::RootKind::Entity ||
                mapped->chain != old_root.chain) {
                refuse("the root map moves a root ACROSS CHAINS (or "
                       "off the Entity kind) — outside the resolver's "
                       "v1 remap machinery (the registered boundary, "
                       "ADR-0036 D3.2: the single-chain roots are the "
                       "covered class); remedy: resolve per the honest "
                       "classes (no map passed), or extend the "
                       "machinery with the real case.");
            }
            remapped.path[walk.root_positions[i]] = mapped->entity;
        }
        if (covered) {
            const geometry::ShapeId remapped_id{reference.feature,
                                                remapped, {}};
            const FoundId mapped =
                find_by_id(*new_generation.body, kind, remapped_id);
            if (mapped.found) {
                if (remapped == reference.subshape) {
                    // The map is the identity on this reference's
                    // roots: the name did not move — class 1, silent.
                    Resolution out;
                    out.kind = Resolution::Kind::Resolved;
                    out.resolved.id = reference;
                    out.resolved.name = mapped.name;
                    return out;
                }
                // THE DECLARED RENAME: the reference follows to the
                // new name, the map declared on the payload — the
                // feature's log writes it, never silence.
                Resolution out;
                out.kind = Resolution::Kind::Renamed;
                out.renamed.from = reference;
                out.renamed.to = remapped_id;
                out.renamed.from_name = row->name;
                out.renamed.to_name = mapped.name;
                out.renamed.map = edit_ctx.map_text();
                return out;
            }
            // The remapped name does not match either: the descent.
        }
    }
    // else: THE DOOR CLOSED. No declaration, no map: the naked
    // positional match across a generation NEVER happens — the
    // reference's roots are uncovered, and the honest classes decide
    // below (B2's own contract: that door is the silent
    // mis-reference's own, closed by construction).

    // STEP 3 — THE DESCENT: the old row followed. The candidates:
    // the new generation's entities under the reference's ROLE
    // SKELETON — the role-table components compared (the structural
    // spine), the authored roots and the layout groups thrown OPEN
    // (the indices that shift with the sketch's own shape). Each
    // entity walks under the SAME grammar (the reference's), its own
    // registry view (the sides may differ in the chain-count
    // verdict — the edit's own doing).
    std::vector<geometry::ShapeId> candidates;
    std::vector<std::string> candidate_names;
    const auto collect = [&](const geometry::ShapeId& id,
                             const std::string& name) {
        for (const auto& c : candidates) {
            if (c.feature == id.feature && c.subshape == id.subshape)
                return;  // the seam-wired pair is ONE candidate —
                         // the pair is the name's materialized form,
                         // not two names (ADR-0023)
        }
        candidates.push_back(id);
        candidate_names.push_back(name);
    };
    if (kind == EntityKind::Edge) {
        for (const auto& e : new_generation.body->edges) {
            if (e.shape_id().subshape.depth == 0) continue;
            if (e.shape_id().feature != reference.feature) continue;
            const PathWalk w =
                walk_path(e.shape_id().subshape, new_registry);
            if (w.skeleton == walk.skeleton) {
                collect(e.shape_id(), e.name());
            }
        }
    } else if (kind == EntityKind::Face) {
        for (const auto& f : new_generation.body->solid.shell().faces()) {
            if (f.shape_id().subshape.depth == 0) continue;
            if (f.shape_id().feature != reference.feature) continue;
            const PathWalk w =
                walk_path(f.shape_id().subshape, new_registry);
            if (w.skeleton == walk.skeleton) {
                collect(f.shape_id(), f.name());
            }
        }
    }

    if (!candidates.empty()) {
        // CLASS 3 — AMBIGUOUS: the reference does not guess; the
        // entity and every candidate BY NAME, the remedy with them.
        Resolution out;
        out.kind = Resolution::Kind::Ambiguous;
        out.ambiguous.candidates = std::move(candidates);
        out.ambiguous.candidate_names = std::move(candidate_names);
        out.ambiguous.entity_name = row->name;
        out.ambiguous.remedy =
            "reselect the entity, or reference the set explicitly "
            "(the reference does not guess — a reference is not a "
            "tolerance decision, ADR-0036 D3).";
        return out;
    }

    // CLASS 4 — VANISHED: no living descendant under the reference's
    // role. What died (the row), why (the diagnosis), the remedy —
    // never a mute "not found".
    Resolution out;
    out.kind = Resolution::Kind::Vanished;
    out.vanished.row = *row;
    out.vanished.why =
        !walk.roots.empty()
            ? "the authored entity was replaced"
            : "the reference's role has no instance in the new "
              "generation";
    out.vanished.remedy =
        "reselect the surviving entity by its new name (the new "
        "generation's query surfaces carry it), or revert the edit.";
    return out;
}

// ---------------------------------------------------------------------------
// The report as text — the feature's log and the UI's error surface
// read this, verbatim
// ---------------------------------------------------------------------------

std::string resolution_text(const Resolution& r) {
    std::ostringstream os;
    switch (r.kind) {
    case Resolution::Kind::Resolved:
        os << "reference_resolved: " << r.resolved.name
           << " — the name re-derived identically from the new "
              "recipe; the feature re-runs, silently.";
        return os.str();

    case Resolution::Kind::Renamed:
        os << "reference_renamed: " << r.renamed.from_name << " -> "
           << r.renamed.to_name
           << " — the root map declared by the edit ("
           << r.renamed.map
           << "); the reference follows, and the feature's log says "
              "so, never silence.";
        return os.str();

    case Resolution::Kind::Ambiguous: {
        os << "reference_ambiguous: " << r.ambiguous.entity_name
           << " — the reference does not guess; the candidates: ";
        for (std::size_t i = 0; i < r.ambiguous.candidate_names.size();
             ++i) {
            if (i != 0) os << ", ";
            os << r.ambiguous.candidate_names[i];
        }
        os << "; remedy: " << r.ambiguous.remedy;
        return os.str();
    }

    case Resolution::Kind::Vanished:
        os << "reference_vanished: " << row_subject(r.vanished.row)
           << " — why: " << r.vanished.why
           << "; remedy: " << r.vanished.remedy;
        return os.str();

    case Resolution::Kind::ClosureDefect:
        os << "closure_defect: "
           << r.closure_defect.orphan.to_string() << " — "
           << r.closure_defect.detail;
        return os.str();
    }
    return std::string();  // unreachable — all cases handled above
}

} // namespace CAD_0::brep
