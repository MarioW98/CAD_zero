// core/brep/src/naming.cpp
//
// The canonical render's implementation (ADR-0036 D1, H2-u6 morso
// 2): one pure walk of the SubshapeRef under the declared tables,
// strings and integers only — zero floating-point content, zero
// state, zero parsing. The grammar is pinned in naming.hpp's
// header; every disagreement between the path and the declaration
// is LOUD (the P2 culture: diagnosis + remedy, never a silent
// wrong render — a name that lies is the one defect this entire
// contract exists to make impossible).
#include "CAD_0/brep/naming.hpp"

#include <sstream>
#include <stdexcept>

namespace CAD_0::brep::naming {

const char* kind_token(FeatureKind k) noexcept {
    switch (k) {
    case FeatureKind::Box:      return "box";
    case FeatureKind::Sphere:   return "sph";
    case FeatureKind::Cylinder: return "cyl";
    case FeatureKind::Sketch:   return "sk";
    case FeatureKind::Extrude:  return "ext";
    case FeatureKind::Subtract: return "sub";
    }
    return "box";  // unreachable — all enum cases handled above
}

namespace {

// The loud refusal, P2-shaped: what was asked, what the table
// declares, and the remedy.
[[noreturn]] void refuse(const std::string& what) {
    throw std::invalid_argument("render_name: " + what);
}

// Render the role whose leading component sits at path[pos], under
// the registry's table; pos advances past everything the role
// consumes. The feature part (kind#id) is the CALLER's: at the top
// level it is the name's own creator; inside a relational role it
// is the operand binding the slot names (the genealogy renders
// through the operand's own declaration).
void render_role(std::string& out,
                 const geometry::SubshapeRef& ref,
                 std::size_t& pos,
                 const RoleRegistry& registry) {
    if (pos >= ref.depth) {
        refuse("the path ends inside a role — the SubshapeRef has " +
               std::to_string(ref.depth) +
               " component(s) and the role at depth " +
               std::to_string(pos) + " needs one; remedy: the path is "
               "built by the materialization that owns the table — a "
               "hand-built path must carry a component per role it "
               "crosses (see ADR-0036 D1's depth derivation).");
    }

    // THE GROUP COMPONENT (the morse-3 register, ADR-0036 D1's
    // multi-chain clause): under a GROUPED registry — the
    // multi-chain body's own view — the path's first component is
    // the chain group, rendered 'c<group>:' before the token. Only
    // the CHAIN-OWNED roles carry it; a feature-owned role under
    // the group view is a caller's view bug and refuses loudly
    // below (the caps belong to the extruded region, never to a
    // chain).
    if (registry.multi_chain) {
        const std::uint32_t group = ref.path[pos];
        ++pos;
        out += 'c';
        out += std::to_string(group);
        out += ':';
        if (pos >= ref.depth) {
            refuse("the path ends inside a group-prefixed role — the "
                   "group component was consumed and the role index "
                   "is missing; remedy: a chain-owned role's path "
                   "under the group view is [group, role index, "
                   "the role's own components] — the feature-owned "
                   "roles (the caps) carry no group and render under "
                   "the plain view (faces()/edges()).");
        }
    }

    const std::uint32_t idx = ref.path[pos];
    if (idx >= registry.roles.size()) {
        refuse("role index " + std::to_string(idx) +
               " is outside the declared table (" +
               std::to_string(registry.roles.size()) +
               " roles); remedy: the first path component indexes the "
               "creating operation's role table for the entity kind "
               "being named — check the registry passed (faces() vs "
               "edges(), the plain vs the grouped view) and the "
               "table's own normative rows.");
    }
    const RoleEntry& entry = registry.roles[idx];
    if (registry.multi_chain && entry.owner != RoleOwner::Chain) {
        refuse("the role '" + std::string(entry.token) +
               "' is FEATURE-OWNED — the extrude's caps belong to the "
               "extruded region, not to any chain, and never carry "
               "the multi-chain group; remedy: render the "
               "feature-owned roles under the operation's plain view "
               "(faces()), and the group view (grouped_faces() / "
               "grouped_edges()) only for the chain-owned roles "
               "(lat, the seam).");
    }
    out += entry.token;
    ++pos;

    switch (entry.form) {
    case RoleForm::Bare:
        return;  // token — one component consumed

    case RoleForm::ArgIndex: {
        if (pos >= ref.depth) {
            refuse("the indexed role '" + std::string(entry.token) +
                   "' demands its integer argument as the next path "
                   "component, and the path ends; remedy: an ArgIndex "
                   "role consumes two components — the role index and "
                   "the argument (the chain entity's position).");
        }
        out += '(';
        out += std::to_string(ref.path[pos]);
        out += ')';
        ++pos;
        return;
    }

    case RoleForm::ArgWord:
        // The entry's own word (cap's lo/hi): the levelled entry is
        // its own table row — no path component spent on the word.
        if (entry.word == nullptr) {
            refuse("the role '" + std::string(entry.token) +
                   "' is declared ArgWord but carries no word; remedy: "
                   "the normative table's row is incomplete — a table "
                   "defect, fix the declaration, never the renderer.");
        }
        out += '(';
        out += entry.word;
        out += ')';
        return;

    case RoleForm::Relational: {
        out += '[';
        for (std::size_t s = 0; s < entry.slots.size(); ++s) {
            if (s != 0) out += " × ";  // space-×-space, the grammar
            const OperandSlot& slot = entry.slots[s];
            if (slot.operand >= registry.operands.size()) {
                refuse("the role '" + std::string(entry.token) +
                       "' slot " + std::to_string(s) +
                       " reads operand binding " +
                       std::to_string(slot.operand) +
                       " and the registry binds " +
                       std::to_string(registry.operands.size()) +
                       "; remedy: the creating operation's naming must "
                       "bind every slot's operand (the genealogy's "
                       "render targets — strictly older features).");
            }
            const OperandBinding& binding =
                registry.operands[slot.operand];
            if (binding.naming == nullptr) {
                refuse("the role '" + std::string(entry.token) +
                       "' slot " + std::to_string(s) +
                       "' names an unbound operand; remedy: bind the "
                       "operand's own naming declaration (its tables "
                       "and its feature id) before rendering.");
            }
            // The operand's own registry: the slot picks the table,
            // the binding carries the declaration — and, since the
            // morse-3 register's item 4 landed (morso 4), the
            // INSTANCE's group declaration: a multi-chain operand
            // renders the GROUP form for its chain-owned roles, the
            // path's own first component the group, exactly as the
            // body's own stamps practice (the two views stay disjoint
            // by construction — the caps still refuse under the group
            // view, the loudest place a view bug can land).
            const RoleRegistry operand_view{
                binding.naming->kind,
                slot.edges ? binding.naming->edge_roles
                           : binding.naming->face_roles,
                binding.naming->operands,
                binding.multi_chain};
            out += kind_token(operand_view.kind);
            out += '#';
            out += std::to_string(binding.feature.value);
            out += ':';
            render_role(out, ref, pos, operand_view);
        }
        out += ']';
        if (entry.level != nullptr) {
            out += '@';
            out += entry.level;
        }
        return;
    }
    }
}

} // namespace

std::string render_name(const geometry::FeatureId& feature,
                        const geometry::SubshapeRef& ref,
                        const RoleRegistry& registry) {
    if (ref.depth == 0) {
        refuse("the SubshapeRef is empty — an unnamed entity has no "
               "render; remedy: the materialization that owns the "
               "table populates the path (the factories bootstrap it "
               "from the construction index, ADR-0036 D1).");
    }
    if (ref.depth > geometry::SubshapeRef::kMaxDepth) {
        refuse("the SubshapeRef is deeper than the ABI's kMaxDepth "
               "(" + std::to_string(geometry::SubshapeRef::kMaxDepth) +
               "); remedy: the genealogy references strictly older "
               "features — a cycle in the recipe is a DAG defect, "
               "never a name.");
    }

    std::string out;
    out.reserve(24);
    out += kind_token(registry.kind);
    out += '#';
    out += std::to_string(feature.value);
    out += ':';

    std::size_t pos = 0;
    render_role(out, ref, pos, registry);

    if (pos != ref.depth) {
        refuse("the path leaves " +
               std::to_string(ref.depth - pos) +
               " component(s) unconsumed — the declared role forms "
               "consume the whole path or none of it disagrees; "
               "remedy: the path is indices into the DECLARED tables "
               "(ADR-0036 D1) — rebuild it from the materialization "
               "that owns the naming, never by hand.");
    }
    return out;
}

std::uint32_t role_index_of(std::span<const RoleEntry> roles,
                            const std::string& token) {
    for (std::size_t i = 0; i < roles.size(); ++i) {
        if (token == roles[i].token) {
            return static_cast<std::uint32_t>(i);
        }
    }
    std::ostringstream os;
    os << "role_index_of: the token '" << token << "' is not this "
          "table's own (" << roles.size() << " declared roles — the "
          "normative vocabulary is closed, ADR-0036 D1); remedy: the "
          "round-trip reads the SAME table the render wrote — pass "
          "the creating operation's declared table, and never a "
          "foreign token.";
    throw std::invalid_argument(os.str());
}

std::string render_root(const RootRef& root, bool sketch_multi_chain) {
    // The root reference's canonical render (the grammar block in
    // naming.hpp's header, the morse-3 register): the sketch's
    // feature, then the root's own form. PURE — a function of the
    // two arguments, strings and integers only, never parsed back.
    std::string out;
    out.reserve(16);
    out += kind_token(FeatureKind::Sketch);
    out += '#';
    out += std::to_string(root.feature.value);
    out += ':';
    switch (root.kind) {
    case RootKind::Face:
        // The sketch's FACE — the extruded region, the caps' parent.
        out += "face";
        return out;
    case RootKind::Entity:
        // The authored root: the entity's position in its chain,
        // the chain component prefixed only when the sketch itself
        // is multi-chain (the same only-multi-chain elision as the
        // roles' group — D4: the chain position IS the root name).
        if (sketch_multi_chain) {
            out += 'c';
            out += std::to_string(root.chain);
            out += ':';
        }
        out += std::to_string(root.entity);
        return out;
    }
    return out;  // unreachable — all enum cases handled above
}

} // namespace CAD_0::brep::naming
