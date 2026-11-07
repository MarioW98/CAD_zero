// core/brep/src/descent.cpp
//
// The tavola's derivation and closure check (ADR-0036 D2, H2-u6
// morso 2): pure integer bookkeeping over the body's own arrays —
// no geometry, no evaluation, no floating-point content anywhere
// (the cost derivation's own proof, in the code).
//
// H2-u6 morso 3: the extrude's own derivation joins — extrude_descent
// (the born-of genealogy, the FROM side) and extrude_froms_resolve
// (the dangling-root check). Still pure bookkeeping: the walk is
// RE-DERIVED from the recipe with the same one-opinion functions the
// materialization consulted, and the names the derivation renders are
// verified against the body's own — the drift-guard, always on.
//
// H2-u6 morso 4: the exact subtract's tavola joins — subtract_descent
// (the six relations at work, the two-sided closure's first table)
// and old_generation_closed (the old side's check). The names are
// READ from the bodies' own stamps (the factory precedent — the same
// pass that built wrote them; nothing to re-derive, nothing to
// drift); the genealogy rides the correspondence the stamping itself
// consulted (tool_lateral_of — one opinion, one lookup).
#include "CAD_0/brep/descent.hpp"

#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"

#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace CAD_0::brep {

const char* descent_token(Descent r) noexcept {
    switch (r) {
    case Descent::Born:      return "born";
    case Descent::Kept:      return "kept";
    case Descent::Absorbed:  return "absorbed";
    case Descent::Split:     return "split";
    case Descent::Merged:    return "merged";
    case Descent::Vanished:  return "vanished";
    }
    return "born";  // unreachable — all enum cases handled above
}

std::size_t DescentTable::count(Descent relation) const noexcept {
    std::size_t n = 0;
    for (const auto& row : rows) {
        if (row.relation == relation) ++n;
    }
    return n;
}

namespace {

// The body's own count per kind — the closure's expectation.
std::uint64_t entity_count(const BRepBody& body, EntityKind kind) {
    switch (kind) {
    case EntityKind::Vertex: return body.vertices.size();
    case EntityKind::Edge:   return body.edges.size();
    case EntityKind::Face:   return body.face_count();
    }
    return 0;  // unreachable — all enum cases handled above
}

const char* entity_token(EntityKind kind) noexcept {
    switch (kind) {
    case EntityKind::Vertex: return "vertex";
    case EntityKind::Edge:   return "edge";
    case EntityKind::Face:   return "face";
    }
    return "face";  // unreachable — all enum cases handled above
}

// Stamp the rows' IDENTITIES (H2-u6 morso 5, ADR-0036 D3): the
// result rows read the described body's own stamps at their
// addresses; the VANISHED rows read the DYING operand's — the row's
// address is the dead entity's own, in that body's numbering (the
// one place the dead identity lives). Vertices stay honestly
// default (v1's queue: the Vertex type carries no id field at all).
// One pass, pure bookkeeping — the same stamps the names were
// rendered from, so structure and render cannot drift.
void stamp_rows(DescentTable& table, const BRepBody& body,
                bool vanished_rows) {
    for (auto& row : table.rows) {
        if ((row.relation == Descent::Vanished) != vanished_rows)
            continue;
        if (row.entity == EntityKind::Edge) {
            if (row.address < body.edges.size()) {
                row.shape_id = body.edges[static_cast<std::size_t>(
                                          row.address)]
                                   .shape_id();
            }
        } else if (row.entity == EntityKind::Face) {
            const auto& faces = body.solid.shell().faces();
            if (row.address < faces.size()) {
                row.shape_id =
                    faces[static_cast<std::size_t>(row.address)]
                        .shape_id();
            }
        }
    }
}

} // namespace

bool descent_closed(const BRepBody& body, const DescentTable& table,
                    std::string* why) {
    // The booking: (kind, address) -> how many rows carry it. The
    // VANISHED rows are SKIPPED (morso 4): they book the OLD
    // generation — their subject is a dead operand entity, addressed
    // in its own body — and old_generation_closed books them there;
    // booking them here would double-book the result's addresses (a
    // vanished row's address is not a result address at all).
    std::map<std::pair<std::uint8_t, std::uint64_t>, std::size_t>
        booked;
    for (const auto& row : table.rows) {
        if (row.relation == Descent::Vanished) continue;
        ++booked[{static_cast<std::uint8_t>(row.entity), row.address}];
    }

    const EntityKind kinds[3] = {EntityKind::Vertex, EntityKind::Edge,
                                 EntityKind::Face};
    for (const auto kind : kinds) {
        const std::uint64_t n = entity_count(body, kind);
        for (std::uint64_t i = 0; i < n; ++i) {
            const auto it = booked.find(
                {static_cast<std::uint8_t>(kind), i});
            const std::size_t c = (it == booked.end()) ? 0 : it->second;
            if (c == 0) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "the tavola LOST " << entity_token(kind)
                       << " " << i << " of the result — no row books "
                          "it (the closure property demands exactly "
                          "one); remedy: the site that knows the "
                          "descent (the operation's own bookkeeping "
                          "pass) owes the row — a tavola that loses "
                          "an entity is a defect, never a gap.";
                    *why = os.str();
                }
                return false;
            }
            if (c > 1) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "the tavola DOUBLE-BOOKS " << entity_token(kind)
                       << " " << i << " of the result — " << c
                       << " rows carry it (the closure property "
                          "demands exactly one); remedy: every entity "
                          "of the result inherits or is born in "
                          "EXACTLY ONE row — find the pass that "
                          "emitted the extra row.";
                    *why = os.str();
                }
                return false;
            }
        }
    }

    // The ghost side: a row booking an entity the body does not
    // have is the same defect read from the other side.
    for (const auto& [key, _] : booked) {
        (void)_;
        const auto kind = static_cast<EntityKind>(key.first);
        if (key.second >= entity_count(body, kind)) {
            if (why != nullptr) {
                std::ostringstream os;
                os << "the tavola books " << entity_token(kind) << " "
                   << key.second
                   << " and the body does not have it (a ghost row); "
                      "remedy: the row's address is the result's own "
                      "positional id — a row for a missing entity is "
                      "a bookkeeping defect at the site that emitted "
                      "it.";
                *why = os.str();
            }
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// The old generation's closure (H2-u6 morso 4) — the two-sided
// property's second side, the first tavola with operands
// ---------------------------------------------------------------------------

namespace {

// Which operand a ref names, in words — the diagnosis' own.
const char* operand_token(std::uint8_t operand) noexcept {
    return operand == 0 ? "the target's" : "the tool's";
}

// The existence verdict for one ref: does the named operand body
// REALLY have this entity? A dangling ref is the suspended reference
// the closure exists to forbid (the FROM-exists-upstream side).
bool ref_is_live(const BRepBody& A, const BRepBody& B,
                 const OperandRef& ref) {
    const BRepBody& body = ref.operand == 0 ? A : B;
    return ref.address < entity_count(body, ref.kind);
}

} // namespace

bool old_generation_closed(const BRepBody& A, const BRepBody& B,
                            const DescentTable& table,
                            std::string* why) {
    // The booking: (operand, kind, address) -> how many rows' refs
    // carry it — every entity of the OLD generation in EXACTLY ONE
    // row (D2's closure, the old side).
    std::map<std::tuple<std::uint8_t, std::uint8_t, std::uint64_t>,
             std::size_t>
        booked;
    for (const auto& row : table.rows) {
        // The relation's own shape, verified where it is declared: a
        // born row books NONE (its genealogy rides its name, D1);
        // kept / absorbed / vanished book EXACTLY ONE (the parent,
        // or the dead entity); split / merged carry the parents (k
        // of them, k >= 2 — the shape is the parents' own).
        if (row.relation == Descent::Born) {
            if (!row.operands.empty()) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "a BORN row books " << row.operands.size()
                       << " operand ref(s) — a born row's genealogy "
                          "rides its NAME (D1: the relation is part of "
                          "the role; D2's closure books the old "
                          "generation by the surviving rows only); "
                          "remedy: the pass that emitted the row owed "
                          "no operands — find it and strip the refs.";
                    *why = os.str();
                }
                return false;
            }
            continue;
        }
        const bool parents =
            row.relation == Descent::Split || row.relation == Descent::Merged;
        if (!parents && row.operands.size() != 1) {
            if (why != nullptr) {
                std::ostringstream os;
                os << "a " << descent_token(row.relation) << " row books "
                   << row.operands.size()
                   << " operand ref(s) — a kept/absorbed/vanished row "
                      "books EXACTLY ONE (the parent, or the dead "
                      "entity); remedy: the pass that emitted the row "
                      "knows the one — find it and correct the refs.";
                *why = os.str();
            }
            return false;
        }
        for (const auto& ref : row.operands) {
            // The existence side: a dangling ref is the suspended
            // reference, never a silent gap.
            if (!ref_is_live(A, B, ref)) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "the tavola's FROM DANGLES: "
                       << operand_token(ref.operand) << " "
                       << entity_token(ref.kind) << " " << ref.address
                       << " does not exist in that operand body — a "
                          "suspended reference, exactly the defect the "
                          "closure exists to forbid (ADR-0036 D2); "
                          "remedy: the refs are written by the "
                          "derivation that walked the operands — a "
                          "dangling one is a derivation defect or a "
                          "caller's mismatched operand.";
                    *why = os.str();
                }
                return false;
            }
            // A vanished row's own subject agrees with its dead ref
            // (the row that disagrees with itself is a bookkeeping
            // defect, loud).
            if (row.relation == Descent::Vanished &&
                (row.entity != ref.kind || row.address != ref.address)) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "a VANISHED row books "
                       << operand_token(ref.operand) << " "
                       << entity_token(ref.kind) << " " << ref.address
                       << " but carries its own subject as "
                       << entity_token(row.entity) << " " << row.address
                       << " — the row disagrees with itself; remedy: a "
                          "vanished row's address IS the dead entity's "
                          "own, in its own body — find the pass that "
                          "emitted the row.";
                    *why = os.str();
                }
                return false;
            }
            ++booked[{ref.operand,
                      static_cast<std::uint8_t>(ref.kind),
                      ref.address}];
        }
    }

    // The coverage: every entity of A and of B, exactly once.
    for (const std::uint8_t operand : {std::uint8_t{0}, std::uint8_t{1}}) {
        const BRepBody& body = operand == 0 ? A : B;
        const EntityKind kinds[3] = {EntityKind::Vertex, EntityKind::Edge,
                                     EntityKind::Face};
        for (const auto kind : kinds) {
            const std::uint64_t n = entity_count(body, kind);
            for (std::uint64_t i = 0; i < n; ++i) {
                const auto it = booked.find(
                    {operand, static_cast<std::uint8_t>(kind), i});
                const std::size_t c =
                    (it == booked.end()) ? 0 : it->second;
                if (c == 0) {
                    if (why != nullptr) {
                        std::ostringstream os;
                        os << "the tavola LOST "
                           << operand_token(operand) << " "
                           << entity_token(kind) << " " << i
                           << " of the old generation — no row books it "
                              "(the closure property demands exactly "
                              "one); remedy: the site that knows the "
                              "descent owes the row — a tavola that "
                              "loses an entity is a defect, never a "
                              "gap.";
                        *why = os.str();
                    }
                    return false;
                }
                if (c > 1) {
                    if (why != nullptr) {
                        std::ostringstream os;
                        os << "the tavola DOUBLE-BOOKS "
                           << operand_token(operand) << " "
                           << entity_token(kind) << " " << i
                           << " of the old generation — " << c
                           << " rows carry it (the closure property "
                              "demands exactly one); remedy: every "
                              "entity of the old generation appears in "
                              "EXACTLY ONE row — find the pass that "
                              "emitted the extra row.";
                        *why = os.str();
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

DescentTable factory_descent(const BRepBody& body) {
    DescentTable table;
    table.rows.reserve(body.vertices.size() + body.edges.size() +
                       body.face_count());

    // The body's own deterministic order: vertices, edges, faces —
    // the same walk every derived pass practices. One born row per
    // entity, zero operands (a factory has none), the names the
    // factory stamped beside the addresses (empty for vertices:
    // v1 has no vertex names, honestly — D4's queue).
    for (std::size_t i = 0; i < body.vertices.size(); ++i) {
        table.rows.push_back(
            {Descent::Born, EntityKind::Vertex,
             static_cast<std::uint64_t>(i), {}, "", {}, ""});
    }
    for (std::size_t i = 0; i < body.edges.size(); ++i) {
        table.rows.push_back(
            {Descent::Born, EntityKind::Edge,
             static_cast<std::uint64_t>(i), {}, body.edges[i].name(),
             {}, ""});
    }
    for (const auto& face : body.solid.shell().faces()) {
        table.rows.push_back(
            {Descent::Born, EntityKind::Face, face.id(), {},
             face.name(), {}, ""});
    }

    // The rows' identities (morso 5): the body's own stamps, one
    // pass — the same stamps the names were rendered from.
    stamp_rows(table, body, /*vanished_rows=*/false);

#ifndef NDEBUG
    // The debug-build activation (D2's hard invariant, the Layer-1
    // spirit): a tavola that loses an entity cannot pass silent.
    std::string why;
    if (!descent_closed(body, table, &why)) {
        throw std::logic_error("factory_descent: the tavola broke its "
                               "own closure — " + why);
    }
#endif

    return table;
}

// ---------------------------------------------------------------------------
// The extrude's tavola (H2-u6 morso 3) — the born-of genealogy
// ---------------------------------------------------------------------------

namespace {

// One step of the construction sequence the materialization walked:
// the sketch's own chain index (the ROOT's address — D4's authored
// layout), the construction GROUP (the names' multi-chain prefix —
// the numbering table's own layout), and the chain's class.
struct WalkStep {
    std::size_t chain{0};
    std::uint32_t group{0};
    bool circle{false};   // a single-circle chain (solid outer or hole)
    std::size_t n{0};     // the contour's entity count (circles: 1)
    std::size_t v_base{0}, e_base{0}, f_base{0};
};

// The body's own names, indexed by address — the drift-guard's
// comparison side (the names the materialization stamped).
std::vector<std::string> body_edge_names(const BRepBody& body) {
    std::vector<std::string> out(body.edges.size());
    for (std::size_t i = 0; i < body.edges.size(); ++i)
        out[i] = body.edges[i].name();
    return out;
}

std::vector<std::string> body_face_names(const BRepBody& body) {
    std::vector<std::string> out(body.face_count());
    for (const auto& face : body.solid.shell().faces()) {
        if (face.id() < out.size()) out[face.id()] = face.name();
    }
    return out;
}

// The drift-guard's loud refusal: a derived name that disagrees with
// the body's own stamp is a tavola that lies — always on, never a
// debug luxury (the names are the tavola's own honesty).
[[noreturn]] void drift(const char* kind, std::uint64_t address,
                        const std::string& derived,
                        const std::string& stamped) {
    std::ostringstream os;
    os << "extrude_descent: the derived tavola disagrees with the "
          "body it describes — " << kind << " " << address
       << " derives as '" << derived << "' and the body's own stamp "
          "reads '" << stamped << "'; remedy: the tavola re-derives "
          "the walk from the SAME recipe the body materialized from "
          "(extrude_detail's own judgment, the numbering tables' own "
          "layout) and the extrude's own FeatureId — pass the recipe "
          "and the id the materialization used; a disagreement here "
          "is a construction/derivation drift, a kernel defect, "
          "never a cache miss.";
    throw std::logic_error(os.str());
}

} // namespace

DescentTable extrude_descent(
    const BRepBody& body, const std::vector<sdf::SketchChain>& chains,
    const geometry::FeatureId& feature,
    const geometry::FeatureId& sketch) {
    if (chains.empty())
        throw std::invalid_argument(
            "extrude_descent: the recipe has no chains — there is no "
            "walk to derive and no body to describe (the empty "
            "document is the materialization's own boundary, not the "
            "tavola's); remedy: pass the recipe the extrude "
            "materialized from.");

    // The outer/hole classification — the SAME one-opinion function
    // the materialization consulted (extrude_detail's own judgment:
    // the max enclosed area, the certificate's own mathematics).
    std::size_t outer = 0;
    if (chains.size() > 1) {
        double best = -1.0;
        for (std::size_t c = 0; c < chains.size(); ++c) {
            const double area = std::abs(
                extrude_detail::chain_signed_area(chains[c]));
            if (area > best) {
                best = area;
                outer = c;
            }
        }
    }
    const bool multi = chains.size() > 1;
    const auto is_circle_chain = [](const sdf::SketchChain& c) {
        return c.entities.size() == 1 &&
               c.entities[0].kind == sdf::SketchEntity::Kind::Circle;
    };

    // The construction sequence: the outer first (group 0), then
    // each hole in sketch order — the numbering table's own
    // multi-chain layout, the SAME sequence make_prism walked.
    std::vector<WalkStep> steps;
    steps.reserve(chains.size());
    {
        WalkStep first;
        first.chain = outer;
        first.group = 0;
        first.circle = is_circle_chain(chains[outer]);
        first.n = chains[outer].entities.size();
        steps.push_back(first);
    }
    std::uint32_t next_group = 1;
    for (std::size_t c = 0; c < chains.size(); ++c) {
        if (c == outer) continue;
        if (!is_circle_chain(chains[c]))
            throw std::invalid_argument(
                "extrude_descent: chain " + std::to_string(c) +
                " is a multi-entity hole — outside the expressible "
                "class the materialization itself enforces (holes "
                "are circles, one disc-complement entity each); a "
                "chain the materialization refused has no tavola. "
                "Remedy: pass the recipe the extrude "
                "materialized from.");
        WalkStep hole;
        hole.chain = c;
        hole.group = next_group++;
        hole.circle = true;
        hole.n = 1;
        steps.push_back(hole);
    }

    // The per-step layout bases — the numbering tables' own
    // arithmetic (contour: 2n vertices, 3n edges, n laterals; solid
    // circle: 4/5/1; hole circle: 2/3/1), accumulated in the walk's
    // own order, exactly as the materialization accumulated them.
    {
        std::size_t v = 0, e = 0, f = 0;
        for (WalkStep& st : steps) {
            st.v_base = v;
            st.e_base = e;
            st.f_base = f;
            if (!st.circle) {
                v += 2 * st.n;
                e += 3 * st.n;
                f += st.n;
            } else if (st.group == 0) {
                v += 4;
                e += 5;
                f += 1;
            } else {
                v += 2;
                e += 3;
                f += 1;
            }
        }
    }

    // The render views — the SAME ones the materialization stamped
    // with (single-chain: the feature form; multi-chain: the group
    // form for the chain-owned roles; the caps always the feature
    // view — the region's own).
    const naming::RoleRegistry lateral_view =
        multi ? naming::kExtNaming.grouped_faces()
              : naming::kExtNaming.faces();
    const naming::RoleRegistry seam_view =
        multi ? naming::kExtNaming.grouped_edges()
              : naming::kExtNaming.edges();

    // The lateral's name for step st, entity i (the walk role the
    // construction held in hand — re-derived, then verified below).
    const auto lateral_name = [&](const WalkStep& st, std::size_t i) {
        geometry::SubshapeRef ref{};
        if (multi) ref = ref.push(st.group);
        ref = ref.push(naming::kExtLatRole)
                  .push(static_cast<std::uint32_t>(i));
        return naming::render_name(feature, ref, lateral_view);
    };
    const auto seam_name = [&](const WalkStep& st) {
        geometry::SubshapeRef ref{};
        if (multi) ref = ref.push(st.group);
        ref = ref.push(naming::kExtSeamRole);
        return naming::render_name(feature, ref, seam_view);
    };

    // The rows — the body's own deterministic order: vertices, then
    // edges, then faces (the factory derivation's own walk). One
    // born row per entity, the FROM the authored root it descends
    // from: the chain entity that owns the wall, the curve, the
    // joint; the sketch's FACE for the caps (the extruded region).
    DescentTable table;
    table.rows.reserve(body.vertices.size() + body.edges.size() +
                       body.face_count());

    const auto entity_root = [&](std::size_t chain,
                                 std::uint32_t entity) {
        return naming::RootRef{sketch, naming::RootKind::Entity,
                               static_cast<std::uint32_t>(chain),
                               entity};
    };

    // Vertices: the contour's joints J_i (2i at z_lo, 2i+1 at z_hi,
    // J_i = entity i's end) born of entity i; the circles' seam and
    // antipode pairs born of the chain's one entity. No names (v1's
    // vertex queue, D4) — honestly empty.
    for (const WalkStep& st : steps) {
        if (!st.circle) {
            for (std::size_t i = 0; i < st.n; ++i) {
                for (const std::size_t offset : {std::size_t{0},
                                                 std::size_t{1}}) {
                    table.rows.push_back(
                        {Descent::Born, EntityKind::Vertex,
                         static_cast<std::uint64_t>(st.v_base + 2 * i +
                                                   offset),
                         {}, "", {entity_root(st.chain,
                                 static_cast<std::uint32_t>(i))}, ""});
                }
            }
        } else if (st.group == 0) {
            for (std::size_t k = 0; k < 4; ++k) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Vertex,
                     static_cast<std::uint64_t>(st.v_base + k), {}, "",
                     {entity_root(st.chain, 0)}, ""});
            }
        } else {
            for (std::size_t k = 0; k < 2; ++k) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Vertex,
                     static_cast<std::uint64_t>(st.v_base + k), {}, "",
                     {entity_root(st.chain, 0)}, ""});
            }
        }
    }

    // Edges: the contour's bottom/top curves (entity i's own, at
    // e_base+i and e_base+n+i) and the verticals at the joints
    // (e_base+2n+i, J_i = entity i's end) born of entity i; the
    // circles' rims and seam born of the chain's one entity. The
    // SEAM is the named one (kExtEdgeRoles' single row); the curves
    // and rims are outside v1's vocabulary — honestly empty. The
    // rows ride the body's own address order within the kind (the
    // factory derivation's own walk): bottoms, then tops, then the
    // verticals.
    for (const WalkStep& st : steps) {
        if (!st.circle) {
            for (std::size_t i = 0; i < st.n; ++i) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(st.e_base + i), {}, "",
                     {entity_root(st.chain,
                             static_cast<std::uint32_t>(i))}, ""});
            }
            for (std::size_t i = 0; i < st.n; ++i) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(st.e_base + st.n + i), {},
                     "",
                     {entity_root(st.chain,
                             static_cast<std::uint32_t>(i))}, ""});
            }
            for (std::size_t i = 0; i < st.n; ++i) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(st.e_base + 2 * st.n + i),
                     {}, "",
                     {entity_root(st.chain,
                             static_cast<std::uint32_t>(i))}, ""});
            }
        } else if (st.group == 0) {
            for (std::size_t k = 0; k < 4; ++k) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(st.e_base + k), {}, "",
                     {entity_root(st.chain, 0)}, ""});
            }
            table.rows.push_back(
                {Descent::Born, EntityKind::Edge,
                 static_cast<std::uint64_t>(st.e_base + 4), {},
                 seam_name(st), {entity_root(st.chain, 0)}, ""});
        } else {
            for (std::size_t k = 0; k < 2; ++k) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(st.e_base + k), {}, "",
                     {entity_root(st.chain, 0)}, ""});
            }
            table.rows.push_back(
                {Descent::Born, EntityKind::Edge,
                 static_cast<std::uint64_t>(st.e_base + 2), {},
                 seam_name(st), {entity_root(st.chain, 0)}, ""});
        }
    }

    // Faces: the laterals (lat(i) — chain-owned, the group prefix on
    // multi-chain bodies) born of their chain entities; the caps
    // (cap(lo)/cap(hi) — the region's own, never a chain's) born of
    // the sketch's FACE.
    for (const WalkStep& st : steps) {
        if (!st.circle) {
            for (std::size_t i = 0; i < st.n; ++i) {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Face,
                     static_cast<std::uint64_t>(st.f_base + i), {},
                     lateral_name(st, i),
                     {entity_root(st.chain,
                             static_cast<std::uint32_t>(i))}, ""});
            }
        } else {
            table.rows.push_back(
                {Descent::Born, EntityKind::Face,
                 static_cast<std::uint64_t>(st.f_base), {},
                 lateral_name(st, 0), {entity_root(st.chain, 0)}, ""});
        }
    }
    {
        // The caps' bases: after the chains' laterals (the walk's
        // own order — the -Z cap first, then the +Z cap).
        std::size_t f = 0;
        for (const WalkStep& st : steps)
            f += !st.circle ? st.n : 1;
        const naming::RootRef face_root{sketch, naming::RootKind::Face,
                                        0, 0};
        table.rows.push_back(
            {Descent::Born, EntityKind::Face,
             static_cast<std::uint64_t>(f), {},
             naming::render_name(
                 feature, geometry::SubshapeRef{}.push(
                                naming::kExtCapLoRole),
                 naming::kExtNaming.faces()),
             {face_root}, ""});
        table.rows.push_back(
            {Descent::Born, EntityKind::Face,
             static_cast<std::uint64_t>(f + 1), {},
             naming::render_name(
                 feature, geometry::SubshapeRef{}.push(
                                naming::kExtCapHiRole),
                 naming::kExtNaming.faces()),
             {face_root}, ""});
    }

    // THE DRIFT-GUARD, always on: every derived name (and every
    // derived emptiness) agrees with the body's own stamps — the
    // derivation and the construction walked the same recipe, or
    // the tavola lies and refuses to ship.
    //
    // The rows' identities (morso 5) follow the same discipline:
    // the body's own stamps at the derived addresses, one pass.
    stamp_rows(table, body, /*vanished_rows=*/false);
    {
        const std::vector<std::string> edge_names = body_edge_names(body);
        const std::vector<std::string> face_names = body_face_names(body);
        for (const auto& row : table.rows) {
            const std::string& stamped =
                row.entity == EntityKind::Edge
                    ? (row.address < edge_names.size()
                           ? edge_names[row.address]
                           : std::string())
                    : (row.entity == EntityKind::Face
                           ? (row.address < face_names.size()
                                  ? face_names[row.address]
                                  : std::string())
                           : std::string());
            if (row.name != stamped) {
                drift(row.entity == EntityKind::Edge ? "edge" : "face",
                      row.address, row.name, stamped);
            }
        }
    }

#ifndef NDEBUG
    // The debug-build activation (the factory precedent): the
    // result-side closure, inside.
    std::string why;
    if (!descent_closed(body, table, &why)) {
        throw std::logic_error("extrude_descent: the tavola broke its "
                               "own closure — " + why);
    }
#endif

    return table;
}

bool extrude_froms_resolve(const DescentTable& table,
                           const std::vector<sdf::SketchChain>& chains,
                           std::string* why) {
    const bool multi = chains.size() > 1;
    for (const auto& row : table.rows) {
        for (const auto& root : row.from) {
            bool live = false;
            const char* what = "the entity root";
            switch (root.kind) {
            case naming::RootKind::Entity:
                live = root.chain < chains.size() &&
                       root.entity <
                           chains[root.chain].entities.size();
                break;
            case naming::RootKind::Face:
                // The sketch's face exists iff the sketch has chains
                // at all (the region is the composition — an empty
                // document has no face to name).
                live = !chains.empty();
                what = "the face root (the extruded region)";
                break;
            }
            if (!live) {
                if (why != nullptr) {
                    std::ostringstream os;
                    os << "the tavola's FROM DANGLES: " << what << " "
                       << naming::render_root(root, multi);
                    if (root.kind == naming::RootKind::Entity)
                        os << " [chain " << root.chain << ", entity "
                           << root.entity << "]";
                    os << " does not exist in the upstream sketch ("
                       << chains.size() << " chain(s)";
                    for (std::size_t c = 0; c < chains.size(); ++c)
                        os << ", chain " << c << " has "
                           << chains[c].entities.size() << " entit"
                           << (chains[c].entities.size() == 1 ? "y"
                                                              : "ies");
                    os << ") — a suspended reference, exactly the "
                          "defect the closure exists to forbid "
                          "(ADR-0036 D2); remedy: the FROM is "
                          "written by the derivation that walked the "
                          "recipe — a dangling one is a derivation "
                          "defect or a caller's mismatched recipe; "
                          "pass the recipe the body materialized "
                          "from.";
                    *why = os.str();
                }
                return false;
            }
        }
    }
    return true;
}


// ---------------------------------------------------------------------------
// The exact subtract's tavola (H2-u6 morso 4) — the six relations at
// work, the guide case's 47-row table the golden
// ---------------------------------------------------------------------------

namespace {

// The causes, pinned once (the golden's own words — ADR-0036 D2's
// own table): the caps CUT AWAY outside the target (the flush
// variant the same cause covers, the coplanar dedup — subtract.hpp's
// assembly-head comment :1200, "nothing is emitted for them, which
// IS the dedup"); the rims, the seam and the split vertices
// RE-AUTHORED with the window; the miss removing nothing.
constexpr const char* kCutAwayCause =
    "cut away outside the target";
constexpr const char* kReAuthoredCause =
    "re-authored with the window";
constexpr const char* kMissCause =
    "the miss: nothing removed (the exact copy stands)";
// THE CROSSED's own (G2-u2 m3): A's corner column — the vertical and
// its two corners die because no re-authored face contains them.
constexpr const char* kColumnCause =
    "the corner column cut away by the notch";

// A deterministic key for a SubshapeRef (the stamp maps' own): the
// path's integers, one byte-separated string — structural content
// only, never the rendered name.
inline std::string render_path_key(
    const geometry::SubshapeRef& ref) {
    std::string key;
    for (std::uint8_t i = 0; i < ref.depth; ++i) {
        key += std::to_string(ref.path[i]);
        key += ',';
    }
    return key;
}

// The vanished rows' stamping when BOTH operands die entities (the
// crossed): the row's own operand ref names the body the dead
// entity lives in — operand 0 reads A, operand 1 reads B (the
// through/miss tables' B-only tail reads B, byte-identical to the
// single-body stamping it replaces).
void stamp_rows_crossed_vanished(DescentTable& table,
                                 const BRepBody& a,
                                 const BRepBody& b) {
    for (auto& row : table.rows) {
        if (row.relation != Descent::Vanished) continue;
        const BRepBody& src =
            (!row.operands.empty() &&
             row.operands[0].operand == 0)
                ? a
                : b;
        if (row.entity == EntityKind::Edge) {
            if (row.address < src.edges.size()) {
                row.shape_id =
                    src.edges[static_cast<std::size_t>(
                                  row.address)]
                        .shape_id();
            }
        } else if (row.entity == EntityKind::Face) {
            const auto& faces = src.solid.shell().faces();
            if (row.address < faces.size()) {
                row.shape_id =
                    faces[static_cast<std::size_t>(row.address)]
                        .shape_id();
            }
        }
    }
}

// B's +Z cap, the walk's own source face (the same rule
// read_tool_walk and cap_surface_of practice — the first +Z-ish
// plane face in shell order). The coedge order of its outer loop IS
// the walk's entity order; the derivation reads it on the
// UNTRANSLATED tool (the order is translation-invariant).
inline const Face* plus_z_cap(const BRepBody& body) {
    for (const auto& f : body.solid.shell().faces()) {
        if (subtract_detail::cap_surface_of(f) != nullptr) return &f;
    }
    return nullptr;
}

// Is this face one of the tool's caps? (a plane with |n.z| ~ 1 — the
// same placement signature every cap check reads; a cylinder is
// always a lateral.)
inline bool is_cap_face(const Face& f) {
    const auto* surf = f.surface();
    if (surf == nullptr || surf->family() != SurfaceFamily::Plane)
        return false;
    subtract_detail::TranslateReader r;
    surf->accept(r);
    return r.plane != nullptr && std::abs(r.plane->normal_vec().z) > 0.9;
}

// The tool's lateral for result-wall j: the walk entity j's own
// lateral face — the genealogical correspondence (an edge reference,
// never a geometric signature), the same lookup the stamping
// consulted (subtract_detail::tool_lateral_of).
inline const Face* wall_operand_of(const BRepBody& B,
                                   const Face* cap,
                                   std::size_t j) {
    if (cap == nullptr) return nullptr;
    if (j >= cap->loops()[0].coedges().size()) return nullptr;
    return subtract_detail::tool_lateral_of(
        B, cap->loops()[0].coedges()[j].edge());
}

// ---------------------------------------------------------------------------
// THE CROSSED TAVOLA (G2-u2 m3, ADR-0038 D3): the corner notch's 64
// rows — the two-sided closure's first table where BOTH operands die
// entities (A's corner column, the tool's outside). Derived by the
// STAMPS (the renumbering broke the positional rule the through/miss
// tables ride on: the result's ids compact over the dead column —
// ShapeId equality is the resolver's own discipline, structural,
// never the render); the vertex correspondence is the coordinate
// bit-match (v1 has no vertex identity — the survivors are verbatim
// copies, the born are new; the day the vertex names land, the
// correspondence reads the stamps instead).
//
// The joint vertical's genealogy reads its OWN stamp's meaning: the
// path [kExtJointRole, j] names the vertical at the chain's joint j
// — the walk table's own numbering (the tool's edge 2n + j, the
// extrude's frozen layout) — one opinion, the same tables the
// construction consulted.
// ---------------------------------------------------------------------------
DescentTable crossed_descent(const BRepBody& result, const BRepBody& A,
                             const BRepBody& B) {
    const std::size_t V_A = A.vertices.size();
    const std::size_t E_A = A.edges.size();
    const std::size_t F_A = A.face_count();

    // The stamp maps: the result's stamped entities -> the addresses.
    std::map<std::pair<std::uint64_t, std::string>, std::size_t>
        redge_of, rface_of;
    for (std::size_t i = 0; i < result.edges.size(); ++i) {
        const auto& sid = result.edges[i].shape_id();
        if (sid.subshape.depth == 0) continue;
        redge_of.emplace(
            std::pair{sid.feature.value,
                      render_path_key(sid.subshape)},
            i);
    }
    for (std::size_t i = 0; i < result.face_count(); ++i) {
        const auto& sid =
            result.solid.shell().faces()[i].shape_id();
        if (sid.subshape.depth == 0) continue;
        rface_of.emplace(
            std::pair{sid.feature.value,
                      render_path_key(sid.subshape)},
            i);
    }
    const auto a_edge_at = [&](std::size_t j) -> std::size_t {
        const auto& sid = A.edges[j].shape_id();
        const auto it = redge_of.find(
            {sid.feature.value, render_path_key(sid.subshape)});
        return it == redge_of.end() ? result.edges.size()
                                    : it->second;
    };
    const auto a_face_at = [&](std::size_t j) -> std::size_t {
        const auto& sid =
            A.solid.shell().faces()[j].shape_id();
        const auto it = rface_of.find(
            {sid.feature.value, render_path_key(sid.subshape)});
        return it == rface_of.end() ? result.face_count()
                                    : it->second;
    };

    // The vertex correspondence: the coordinate bit-match (the
    // survivors are verbatim copies — the positions are bit-identical
    // by construction; v1's honest rule, documented above).
    std::vector<std::size_t> rv_of(result.vertices.size(), SIZE_MAX);
    std::vector<std::size_t> av_of(V_A, SIZE_MAX);
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        const auto& p = result.vertices[i].position();
        for (std::size_t j = 0; j < V_A; ++j)
            if (av_of[j] == SIZE_MAX &&
                A.vertices[j].position().x == p.x &&
                A.vertices[j].position().y == p.y &&
                A.vertices[j].position().z == p.z) {
                rv_of[i] = j;
                av_of[j] = i;
                break;
            }
    }
    std::vector<std::size_t> bv_of(B.vertices.size(), SIZE_MAX);
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        const auto& p = result.vertices[i].position();
        for (std::size_t j = 0; j < B.vertices.size(); ++j)
            if (bv_of[j] == SIZE_MAX &&
                B.vertices[j].position().x == p.x &&
                B.vertices[j].position().y == p.y &&
                B.vertices[j].position().z == p.z) {
                bv_of[j] = i;
                break;
            }
    }

    // The joint vertical's own stamp: the path [kExtJointRole, j]
    // names the tool's vertical at joint j (the edge 2n + j).
    std::size_t n_chain = 0;
    if (const Face* cap = plus_z_cap(B))
        n_chain = cap->loops()[0].coedges().size();
    const auto joint_b_edge = [&](const geometry::ShapeId& sid) {
        if (sid.subshape.depth == 2 &&
            sid.subshape.path[0] == naming::kExtJointRole &&
            n_chain > 0)
            return 2 * n_chain + sid.subshape.path[1];
        return static_cast<std::uint64_t>(SIZE_MAX);
    };

    DescentTable table;
    table.rows.reserve(result.vertices.size() + result.edges.size() +
                       result.face_count() + V_A + E_A + F_A +
                       B.vertices.size() + B.edges.size() +
                       B.face_count());

    // ---- The result's vertices: the A-matched kept, the rest born.
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        if (rv_of[i] != SIZE_MAX) {
            table.rows.push_back(
                {Descent::Kept, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Vertex,
                   static_cast<std::uint64_t>(rv_of[i])}},
                 "", {}, ""});
        } else {
            table.rows.push_back(
                {Descent::Born, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i), {}, "", {}, ""});
        }
    }

    // ---- The result's edges: the A-matched kept (the span
    // unchanged: both endpoints A-matched, the order agreeing) or
    // absorbed (an endpoint born — the re-spanned); the joint
    // absorbed of the tool; the rest born.
    for (std::size_t i = 0; i < result.edges.size(); ++i) {
        const geometry::ShapeId& sid = result.edges[i].shape_id();
        bool booked = false;
        for (std::size_t j = 0; j < E_A && !booked; ++j) {
            if (a_edge_at(j) != i) continue;
            booked = true;
            const bool kept =
                rv_of[static_cast<std::size_t>(
                          result.edges[i].vertex(0))] ==
                    static_cast<std::size_t>(
                        A.edges[j].vertex(0)) &&
                rv_of[static_cast<std::size_t>(
                          result.edges[i].vertex(1))] ==
                    static_cast<std::size_t>(
                        A.edges[j].vertex(1));
            table.rows.push_back(
                {kept ? Descent::Kept : Descent::Absorbed,
                 EntityKind::Edge, static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Edge,
                   static_cast<std::uint64_t>(j)}},
                 result.edges[i].name(), {}, ""});
        }
        if (booked) continue;
        const std::uint64_t jb = joint_b_edge(sid);
        if (jb != static_cast<std::uint64_t>(SIZE_MAX)) {
            table.rows.push_back(
                {Descent::Absorbed, EntityKind::Edge,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Edge, jb}},
                 result.edges[i].name(), {}, ""});
            continue;
        }
        table.rows.push_back(
            {Descent::Born, EntityKind::Edge,
             static_cast<std::uint64_t>(i), {},
             result.edges[i].name(), {}, ""});
    }

    // ---- The result's faces: the A-matched kept (the loop's edge
    // sequence, mapped through the edge correspondence, equals the
    // operand's own) or absorbed; the B-matched (the walls)
    // absorbed.
    for (std::size_t i = 0; i < result.face_count(); ++i) {
        const Face& rf = result.solid.shell().faces()[i];
        bool booked = false;
        for (std::size_t j = 0; j < F_A && !booked; ++j) {
            if (a_face_at(j) != i) continue;
            booked = true;
            // The loop comparison: the result's loop's edges, each
            // mapped to A's own id (the born/unmapped break the
            // equality), against the operand face's sequence.
            bool same = rf.loops().size() ==
                        A.solid.shell().faces()[j].loops().size();
            if (same) {
                for (std::size_t li = 0;
                     li < rf.loops().size() && same; ++li) {
                    const auto& rc = rf.loops()[li].coedges();
                    const auto& ac =
                        A.solid.shell()
                            .faces()[j]
                            .loops()[li]
                            .coedges();
                    if (rc.size() != ac.size()) {
                        same = false;
                        break;
                    }
                    for (std::size_t ci = 0; ci < rc.size(); ++ci) {
                        const std::size_t re = rc[ci].edge();
                        // Map the result edge back to A's id.
                        std::size_t a_edge = SIZE_MAX;
                        for (std::size_t jj = 0; jj < E_A; ++jj)
                            if (a_edge_at(jj) == re) {
                                a_edge = jj;
                                break;
                            }
                        if (a_edge != ac[ci].edge()) {
                            same = false;
                            break;
                        }
                    }
                }
            }
            table.rows.push_back(
                {same ? Descent::Kept : Descent::Absorbed,
                 EntityKind::Face, static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Face,
                   static_cast<std::uint64_t>(j)}},
                 rf.name(), {}, ""});
        }
        if (booked) continue;
        // The B-matched (the walls): the stamp finds the tool's own
        // lateral.
        for (std::size_t j = 0; j < B.face_count() && !booked; ++j) {
            const auto& bsid =
                B.solid.shell().faces()[j].shape_id();
            const auto& rsid = rf.shape_id();
            if (bsid.subshape.depth > 0 && rsid == bsid) {
                booked = true;
                table.rows.push_back(
                    {Descent::Absorbed, EntityKind::Face,
                     static_cast<std::uint64_t>(i),
                     {{1, EntityKind::Face,
                       static_cast<std::uint64_t>(j)}},
                     rf.name(), {}, ""});
            }
        }
        if (booked) continue;
        // Unmatched (no A stamp, no B stamp): the row is still owed —
        // emitted with no booked operand, so the old-generation
        // closure below REFUSES LOUDLY (the m4 precedent).
        table.rows.push_back(
            {Descent::Absorbed, EntityKind::Face,
             static_cast<std::uint64_t>(i), {}, rf.name(), {}, ""});
    }

    // ---- The vanished tail: A's corner column first (A's own
    // order), then the tool's dead (B's own order) — each entity in
    // its own body's numbering, the cause recorded.
    for (std::size_t j = 0; j < V_A; ++j)
        if (av_of[j] == SIZE_MAX)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Vertex,
                 static_cast<std::uint64_t>(j),
                 {{0, EntityKind::Vertex,
                   static_cast<std::uint64_t>(j)}},
                 "", {}, kColumnCause});
    for (std::size_t j = 0; j < E_A; ++j)
        if (a_edge_at(j) == result.edges.size())
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Edge,
                 static_cast<std::uint64_t>(j),
                 {{0, EntityKind::Edge,
                   static_cast<std::uint64_t>(j)}},
                 A.edges[j].name(), {}, kColumnCause});
    for (std::size_t j = 0; j < F_A; ++j)
        if (a_face_at(j) == result.face_count())
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Face,
                 static_cast<std::uint64_t>(j),
                 {{0, EntityKind::Face,
                   static_cast<std::uint64_t>(j)}},
                 A.solid.shell().faces()[j].name(), {},
                 kColumnCause});

    // The tool's dead: the re-authored (the notch-chain entities'
    // curves and the joint's vertex pair) vs the cut-away. The
    // entities: the matched walls' stamps name them; the joint
    // vertical's stamp names the joint index.
    std::vector<bool> b_edge_alive(B.edges.size(), false);
    std::vector<bool> b_vert_alive(B.vertices.size(), false);
    std::vector<bool> b_face_alive(B.face_count(), false);
    std::vector<bool> b_edge_reauth(B.edges.size(), false);
    std::vector<bool> b_vert_reauth(B.vertices.size(), false);
    std::size_t joint_idx = SIZE_MAX;
    for (const auto& row : table.rows) {
        if (row.entity == EntityKind::Edge &&
            row.relation == Descent::Absorbed &&
            row.operands.size() == 1 &&
            row.operands[0].operand == 1) {
            const geometry::ShapeId& sid =
                result.edges[static_cast<std::size_t>(
                                 row.address)]
                    .shape_id();
            if (sid.subshape.depth == 2 &&
                sid.subshape.path[0] == naming::kExtJointRole)
                joint_idx = sid.subshape.path[1];
        }
    }
    std::vector<std::size_t> notch_ents;
    for (std::size_t j = 0; j < B.face_count(); ++j) {
        const auto& bsid =
            B.solid.shell().faces()[j].shape_id();
        if (bsid.subshape.depth == 0) continue;
        const auto it = rface_of.find(
            {bsid.feature.value, render_path_key(bsid.subshape)});
        if (it == rface_of.end()) continue;
        b_face_alive[j] = true;
        if (bsid.subshape.depth == 2 &&
            bsid.subshape.path[0] == naming::kExtLatRole)
            notch_ents.push_back(bsid.subshape.path[1]);
    }
    if (n_chain > 0) {
        for (const std::size_t ent : notch_ents) {
            if (ent < B.edges.size()) b_edge_reauth[ent] = true;
            if (n_chain + ent < B.edges.size())
                b_edge_reauth[n_chain + ent] = true;
        }
    }
    // The joint vertical itself is ALIVE (the absorbed row's
    // operand); its vertex pair re-timed (the re-authored cause).
    if (joint_idx != SIZE_MAX && n_chain > 0) {
        if (2 * n_chain + joint_idx < B.edges.size())
            b_edge_alive[2 * n_chain + joint_idx] = true;
        if (2 * joint_idx < B.vertices.size())
            b_vert_reauth[2 * joint_idx] = true;
        if (2 * joint_idx + 1 < B.vertices.size())
            b_vert_reauth[2 * joint_idx + 1] = true;
    }
    for (std::size_t j = 0; j < B.vertices.size(); ++j)
        if (bv_of[j] != SIZE_MAX) b_vert_alive[j] = true;
    for (const auto& row : table.rows) {
        if (row.entity != EntityKind::Edge) continue;
        if (row.relation == Descent::Absorbed &&
            row.operands.size() == 1 &&
            row.operands[0].operand == 1)
            b_edge_alive[row.operands[0].address] = true;
    }

    for (std::size_t j = 0; j < B.vertices.size(); ++j)
        if (!b_vert_alive[j])
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Vertex,
                 static_cast<std::uint64_t>(j),
                 {{1, EntityKind::Vertex,
                   static_cast<std::uint64_t>(j)}},
                 "", {},
                 b_vert_reauth[j] ? kReAuthoredCause
                                  : kCutAwayCause});
    for (std::size_t j = 0; j < B.edges.size(); ++j)
        if (!b_edge_alive[j])
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Edge,
                 static_cast<std::uint64_t>(j),
                 {{1, EntityKind::Edge,
                   static_cast<std::uint64_t>(j)}},
                 B.edges[j].name(), {},
                 b_edge_reauth[j] ? kReAuthoredCause
                                  : kCutAwayCause});
    for (std::size_t j = 0; j < B.face_count(); ++j)
        if (!b_face_alive[j])
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Face,
                 static_cast<std::uint64_t>(j),
                 {{1, EntityKind::Face,
                   static_cast<std::uint64_t>(j)}},
                 B.solid.shell().faces()[j].name(), {},
                 kCutAwayCause});

#ifndef NDEBUG
    {
        std::string why;
        if (!descent_closed(result, table, &why)) {
            throw std::logic_error(
                "subtract_descent: the tavola broke its own closure "
                "(the result side, the crossed) — " + why);
        }
        if (!old_generation_closed(A, B, table, &why)) {
            throw std::logic_error(
                "subtract_descent: the tavola broke its own closure "
                "(the old generation, the crossed) — " + why);
        }
    }
#endif

    // The rows' identities: the RESULT rows read the result's own
    // stamps; the VANISHED tail reads the DYING operand's own body —
    // the row's own operand ref names it (two dying operands: A's
    // column, the tool's dead).
    stamp_rows(table, result, /*vanished_rows=*/false);
    stamp_rows_crossed_vanished(table, A, B);
    return table;
}

} // namespace

DescentTable subtract_descent(const BRepBody& result, const BRepBody& A,
                              const BRepBody& B) {
    const std::size_t V_A = A.vertices.size();
    const std::size_t E_A = A.edges.size();
    const std::size_t F_A = A.face_count();

    // The outcome classification, read from the result's own counts
    // (the tavola describes what the result IS — no new geometry, no
    // new decisions): the through-hole appends at least one wall face
    // (the seam-wired circle's ONE, a chain's n) and pierces BOTH
    // faces of the pair (H = 2); the miss is the exact copy; THE
    // CROSSED (G2-u2 m3, ADR-0038 D3) appends the walls with the
    // pierced faces SIMPLY CONNECTED (H = 0 — the L-shaped outer
    // loops, no inner loops) and A LOSING entities (the corner
    // column died, the ids renumbered); THE BLIND (G2-u2 m4,
    // ADR-0038's m4 rule) appends the wall AND the carried cap (H =
    // 1 — exactly one pierced face, the cavity's floor the second
    // absorbed operand face). The discriminator is POSITIONAL on
    // purpose — never the stamps: a caller may derive the tavola
    // against a differently-stamped operand (the m5 click-test's
    // fresh bridge conversion, the resolver's own pattern), and the
    // through/miss tables' positional rule must keep answering for
    // them. The stamp-based correspondence lives INSIDE the crossed
    // branch, where the result's own renumbering forces it. (The m4
    // morse's H = 1 seat — the bill registered in the dispatch's
    // own words — is paid here.)
    const bool through = result.face_count() > F_A;
    const bool crossed = through &&
                         inner_loop_count(result) == 0;
    if (crossed) return crossed_descent(result, A, B);
    const bool blind = through && inner_loop_count(result) == 1;
    if (blind) {
        // ---- THE BLIND TAVOLA (the m4 door): the same positional
        // discipline as the through/miss tables — the floor's
        // correspondence reads the NORMAL SIGN (the carried cap
        // keeps its native frame: the floor's normal names the
        // tool's own cap, the near one; the far cap dies cut away).
        DescentTable table;
        table.rows.reserve(result.vertices.size() +
                           result.edges.size() +
                           result.face_count() + B.vertices.size() +
                           B.edges.size() + B.face_count());

        for (std::size_t i = 0; i < result.vertices.size(); ++i) {
            if (i < V_A) {
                table.rows.push_back(
                    {Descent::Kept, EntityKind::Vertex,
                     static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Vertex,
                       static_cast<std::uint64_t>(i)}},
                     "", {}, ""});
            } else {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Vertex,
                     static_cast<std::uint64_t>(i), {}, "", {}, ""});
            }
        }
        for (std::size_t i = 0; i < result.edges.size(); ++i) {
            if (i < E_A) {
                table.rows.push_back(
                    {Descent::Kept, EntityKind::Edge,
                     static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Edge,
                       static_cast<std::uint64_t>(i)}},
                     result.edges[i].name(), {}, ""});
            } else {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Edge,
                     static_cast<std::uint64_t>(i), {},
                     result.edges[i].name(), {}, ""});
            }
        }
        // The carried cap's normal sign, read once from the result's
        // own floor face (the LAST face the assembly appended): it
        // names the tool's near cap (the same sign) against the far
        // one (the opposite, cut away outside).
        double floor_nz = 0.0;
        {
            const auto& faces = result.solid.shell().faces();
            if (F_A + 1 < faces.size()) {
                const auto* surf = faces[F_A + 1].surface();
                if (surf != nullptr &&
                    surf->family() == SurfaceFamily::Plane) {
                    subtract_detail::TranslateReader r;
                    surf->accept(r);
                    if (r.plane != nullptr)
                        floor_nz = r.plane->normal_vec().z;
                }
            }
        }
        for (const auto& face : result.solid.shell().faces()) {
            const std::size_t i = face.id();
            if (i < F_A) {
                // A's own faces: the pierced one absorbed (its inner
                // loop born of the rim), the rest kept.
                const bool pierced = face.loops().size() > 1;
                table.rows.push_back(
                    {pierced ? Descent::Absorbed : Descent::Kept,
                     EntityKind::Face, static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Face,
                       static_cast<std::uint64_t>(i)}},
                     face.name(), {}, ""});
                continue;
            }
            const auto* surf = face.surface();
            const bool is_cyl =
                surf != nullptr &&
                surf->family() == SurfaceFamily::Cylinder;
            if (i == F_A && is_cyl) {
                // THE ABSORBED WALL: the tool's own lateral,
                // re-windowed to [z_floor, z_pierced] — the row's
                // operand the lateral face in the tool's own body
                // (the circle's single wall, the walk entity 0's
                // own — the same lookup the through table
                // practices).
                const Face* lateral =
                    wall_operand_of(B, plus_z_cap(B), 0);
                if (lateral != nullptr) {
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i),
                         {{1, EntityKind::Face,
                           static_cast<std::uint64_t>(
                               lateral->id())}},
                         face.name(), {}, ""});
                } else {
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i), {},
                         face.name(), {}, ""});
                }
                continue;
            }
            // THE ABSORBED INTERNAL CAP: the tool's near cap (the
            // carried face — the floor), found by the normal sign.
            {
                const Face* near_cap = nullptr;
                for (const auto& bf : B.solid.shell().faces()) {
                    if (!is_cap_face(bf)) continue;
                    subtract_detail::TranslateReader r;
                    bf.surface()->accept(r);
                    if (r.plane == nullptr) continue;
                    const double nz = r.plane->normal_vec().z;
                    if ((floor_nz < 0.0 && nz < 0.0) ||
                        (floor_nz > 0.0 && nz > 0.0)) {
                        near_cap = &bf;
                        break;
                    }
                }
                if (near_cap != nullptr) {
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i),
                         {{1, EntityKind::Face,
                           static_cast<std::uint64_t>(
                               near_cap->id())}},
                         face.name(), {}, ""});
                } else {
                    // No near cap found (a malformed tool the class
                    // checks missed): the row is still owed — the
                    // old-generation closure below REFUSES LOUDLY.
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i), {},
                         face.name(), {}, ""});
                }
            }
        }

        // ---- the VANISHED tail, the dying operand's own order: the
        // tool's split pair, rims and seam re-authored with the
        // re-based window; the FAR cap cut away outside (the near
        // cap and the lateral are absorbed — booked above).
        for (std::size_t i = 0; i < B.vertices.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Vertex,
                   static_cast<std::uint64_t>(i)}},
                 "", {}, kReAuthoredCause});
        for (std::size_t i = 0; i < B.edges.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Edge,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Edge,
                   static_cast<std::uint64_t>(i)}},
                 B.edges[i].name(), {}, kReAuthoredCause});
        for (const auto& face : B.solid.shell().faces()) {
            if (!is_cap_face(face)) continue;
            subtract_detail::TranslateReader r;
            face.surface()->accept(r);
            const double nz = r.plane->normal_vec().z;
            // The NEAR cap survived (absorbed — the floor); only the
            // far one died.
            if ((floor_nz < 0.0 && nz < 0.0) ||
                (floor_nz > 0.0 && nz > 0.0))
                continue;
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Face,
                 static_cast<std::uint64_t>(face.id()),
                 {{1, EntityKind::Face,
                   static_cast<std::uint64_t>(face.id())}},
                 face.name(), {}, kCutAwayCause});
        }

#ifndef NDEBUG
        {
            std::string why;
            if (!descent_closed(result, table, &why)) {
                throw std::logic_error(
                    "subtract_descent: the tavola broke its own "
                    "closure (the result side, the blind) — " + why);
            }
            if (!old_generation_closed(A, B, table, &why)) {
                throw std::logic_error(
                    "subtract_descent: the tavola broke its own "
                    "closure (the old generation, the blind) — " +
                    why);
            }
        }
#endif

        stamp_rows(table, result, /*vanished_rows=*/false);
        stamp_rows(table, B, /*vanished_rows=*/true);
        return table;
    }

    DescentTable table;
    table.rows.reserve(result.vertices.size() + result.edges.size() +
                       result.face_count() + B.vertices.size() +
                       B.edges.size() + B.face_count());

    // ---- the RESULT side (kept / absorbed / born), the body's own
    // deterministic order: vertices, then edges, then faces.
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        // A's vertices are all kept in the through AND the miss (the
        // copy carries them; none dies); the new split vertices are
        // born. v1 has no vertex names (D4's queue) — honestly empty.
        if (i < V_A) {
            table.rows.push_back(
                {Descent::Kept, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Vertex,
                   static_cast<std::uint64_t>(i)}},
                 "", {}, ""});
        } else {
            table.rows.push_back(
                {Descent::Born, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i), {}, "", {}, ""});
        }
    }
    for (std::size_t i = 0; i < result.edges.size(); ++i) {
        if (i < E_A) {
            table.rows.push_back(
                {Descent::Kept, EntityKind::Edge,
                 static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Edge,
                   static_cast<std::uint64_t>(i)}},
                 result.edges[i].name(), {}, ""});
        } else {
            // The born edges: the rim circles' materialized halves
            // and the seam (the through), unnamed where the stamping
            // found no vocabulary (the registered boundary) — the
            // names read from the body's own stamps (the factory
            // precedent: the same pass that built wrote them).
            table.rows.push_back(
                {Descent::Born, EntityKind::Edge,
                 static_cast<std::uint64_t>(i), {},
                 result.edges[i].name(), {}, ""});
        }
    }
    {
        // The walls' genealogical correspondence, derived once: the
        // walk's coedge order (the +Z cap's outer loop) read on the
        // untranslated tool — the result's wall j descends from the
        // walk entity j's own lateral.
        const Face* cap = through ? plus_z_cap(B) : nullptr;
        for (const auto& face : result.solid.shell().faces()) {
            const std::size_t i = face.id();
            if (i < F_A) {
                // A's own faces: the pierced pair absorbed (their
                // inner loops born of the rim), the rest kept — read
                // from the result's own loops (the copy plus the inner
                // loops the assembly added).
                const bool pierced = face.loops().size() > 1;
                table.rows.push_back(
                    {pierced ? Descent::Absorbed : Descent::Kept,
                     EntityKind::Face, static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Face,
                       static_cast<std::uint64_t>(i)}},
                     face.name(), {}, ""});
            } else {
                // The ABSORBED WALL: the tool's own lateral,
                // re-framed to the window — the row's operand the
                // lateral face in the tool's own body. (In the miss
                // there are no walls; this branch is through-only.)
                const std::size_t j = i - F_A;
                const Face* lateral = wall_operand_of(B, cap, j);
                if (lateral != nullptr) {
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i),
                         {{1, EntityKind::Face,
                           static_cast<std::uint64_t>(lateral->id())}},
                         face.name(), {}, ""});
                } else {
                    // No lateral found (a malformed tool the class
                    // checks missed): the row is still owed — emitted
                    // with no booked operand, so the old-generation
                    // closure below REFUSES LOUDLY (an absorbed row
                    // books exactly one parent; a correspondence that
                    // cannot find it is a defect the check names,
                    // never a silent gap).
                    table.rows.push_back(
                        {Descent::Absorbed, EntityKind::Face,
                         static_cast<std::uint64_t>(i), {},
                         face.name(), {}, ""});
                }
            }
        }
    }

    // ---- the VANISHED tail, the dying operand's own order
    // (vertices, edges, faces) -----------------------------------------
    if (!through) {
        // THE MISS: nothing of B survives — the laterals die with the
        // caps (the tool never met the target; the exact copy stands).
        for (std::size_t i = 0; i < B.vertices.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Vertex,
                   static_cast<std::uint64_t>(i)}},
                 "", {}, kMissCause});
        for (std::size_t i = 0; i < B.edges.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Edge,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Edge,
                   static_cast<std::uint64_t>(i)}},
                 B.edges[i].name(), {}, kMissCause});
        for (const auto& face : B.solid.shell().faces())
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Face,
                 static_cast<std::uint64_t>(face.id()),
                 {{1, EntityKind::Face,
                   static_cast<std::uint64_t>(face.id())}},
                 face.name(), {}, kMissCause});
    } else {
        // THE THROUGH: the caps cut away outside the target; the
        // rims, the seam and the split vertices re-authored with the
        // window (the wall's window re-timed to [z_bot, z_top] — its
        // own boundary curves and corners rebuilt at the target's
        // levels).
        for (std::size_t i = 0; i < B.vertices.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Vertex,
                   static_cast<std::uint64_t>(i)}},
                 "", {}, kReAuthoredCause});
        for (std::size_t i = 0; i < B.edges.size(); ++i)
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Edge,
                 static_cast<std::uint64_t>(i),
                 {{1, EntityKind::Edge,
                   static_cast<std::uint64_t>(i)}},
                 B.edges[i].name(), {}, kReAuthoredCause});
        for (const auto& face : B.solid.shell().faces()) {
            // The laterals ABSORBED (the walls above); the caps die.
            if (!is_cap_face(face)) continue;
            table.rows.push_back(
                {Descent::Vanished, EntityKind::Face,
                 static_cast<std::uint64_t>(face.id()),
                 {{1, EntityKind::Face,
                   static_cast<std::uint64_t>(face.id())}},
                 face.name(), {}, kCutAwayCause});
        }
    }

#ifndef NDEBUG
    // The debug-build activation (the factory precedent): BOTH sides
    // of the closure, inside — a tavola that loses or double-books an
    // entity, on either generation, cannot pass silent.
    {
        std::string why;
        if (!descent_closed(result, table, &why)) {
            throw std::logic_error(
                "subtract_descent: the tavola broke its own closure "
                "(the result side) — " + why);
        }
        if (!old_generation_closed(A, B, table, &why)) {
            throw std::logic_error(
                "subtract_descent: the tavola broke its own closure "
                "(the old generation) — " + why);
        }
    }
#endif

    // The rows' identities (morso 5): the RESULT rows read the
    // result's own stamps; the VANISHED tail reads the DYING
    // operand's — the row's address is the dead entity's own, in
    // B's numbering (the one place the dead identity lives; the
    // class-4 payload carries it whole).
    stamp_rows(table, result, /*vanished_rows=*/false);
    stamp_rows(table, B, /*vanished_rows=*/true);

    return table;
}

// ---------------------------------------------------------------------------
// fillet_descent — THE FILLET'S TAVOLA (ADR-0039 D3, G2-u3 m2): the
// box class's 35 rows, the closure both sides. The correspondence is
// STRUCTURAL, the same discipline the crossed corner practices (the
// result's own renumbering forces the stamps): the named kinds match
// by ShapeId (the survivors keep their stamps — the identity is
// where a thing was born, not where it currently sits), the vertices
// by exact position equality (the copies carry the doubles
// bit-identical; the born tangent points are strictly inside the
// faces, never on a surviving corner). The kept/absorbed split reads
// the records' own content: an edge whose stored pair moved is
// re-spanned (absorbed), a face whose walk's vertex sequence changed
// is re-authored (absorbed) — the operation's mark IS this table, the
// vanished tail's cause the feature's own act ("replaced by the
// flank" — the edge's death names a BIRTH, not a cut).
// ---------------------------------------------------------------------------
DescentTable fillet_descent(const BRepBody& result,
                            const BRepBody& operand) {
    DescentTable table;
    table.rows.reserve(result.vertices.size() +
                       result.edges.size() +
                       result.face_count() +
                       operand.vertices.size() +
                       operand.edges.size() +
                       operand.face_count());

    // The operand's vertex positions — the exact-match index (the
    // kept corners are bit-identical copies).
    const auto pos_eq = [](const math::Vec3d& a,
                           const math::Vec3d& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    };
    const auto operand_vertex_of = [&](const math::Vec3d& p) {
        for (std::size_t i = 0; i < operand.vertices.size(); ++i)
            if (pos_eq(p, operand.vertices[i].position()))
                return static_cast<std::int64_t>(i);
        return static_cast<std::int64_t>(-1);
    };
    // The operand's edges by stamp (the survivors keep their stamps).
    // A seam-wired arc PAIR answers to ONE logical name (the morse-5
    // register's rule): when several operand edges carry the same
    // stamp, the match disambiguates by the stored pair's POSITIONS
    // (the pair's two halves share the joints) and never lets two
    // result rows claim the same operand row (the closure's floor).
    const auto operand_edge_candidates = [&](const Edge& e) {
        std::vector<std::size_t> out;
        if (e.shape_id().subshape.depth == 0) return out;
        for (std::size_t i = 0; i < operand.edges.size(); ++i) {
            const auto& id = operand.edges[i].shape_id();
            if (id.subshape.depth > 0 &&
                id.feature == e.shape_id().feature &&
                id.subshape == e.shape_id().subshape)
                out.push_back(i);
        }
        return out;
    };
    std::vector<bool> edge_taken(operand.edges.size(), false);
    const auto operand_edge_of = [&](const Edge& e) {
        const auto cands = operand_edge_candidates(e);
        const math::Vec3d rp[2] = {
            result.vertices[static_cast<std::size_t>(e.vertex(0))]
                .position(),
            result.vertices[static_cast<std::size_t>(e.vertex(1))]
                .position()};
        if (cands.empty()) {
            // THE UNNAMED WORLD'S OWN CORRESPONDENCE (m6-correttiva-
            // ter, the chain's first case: the previous fillet's
            // quarter arcs survive 1:1 with NO stamps to match —
            // the stamped kinds' rule would book them born/vanished,
            // a lie about edges that never moved). The stored pair's
            // exact position equality against the operand's own
            // stamp-less edges — the VERTICES' own discipline, one
            // kind over. Guarded by the empty stamp on BOTH sides:
            // a stamped result edge with no candidate stays born
            // (the isect wires), and the stamped operand rows stay
            // outside the walk (the seam-wired pairs never enter —
            // their halves carry the shared stamp).
            if (e.shape_id().subshape.depth > 0)
                return static_cast<std::int64_t>(-1);
            for (std::size_t i = 0; i < operand.edges.size(); ++i) {
                if (edge_taken[i]) continue;
                const Edge& oe = operand.edges[i];
                if (oe.shape_id().subshape.depth > 0) continue;
                const math::Vec3d op[2] = {
                    operand.vertices[static_cast<std::size_t>(
                                          oe.vertex(0))]
                        .position(),
                    operand.vertices[static_cast<std::size_t>(
                                          oe.vertex(1))]
                        .position()};
                const bool same =
                    pos_eq(rp[0], op[0]) && pos_eq(rp[1], op[1]);
                const bool flipped =
                    pos_eq(rp[0], op[1]) && pos_eq(rp[1], op[0]);
                if (same || flipped) {
                    edge_taken[i] = true;
                    return static_cast<std::int64_t>(i);
                }
            }
            return static_cast<std::int64_t>(-1);
        }
        for (const std::size_t c : cands) {
            if (edge_taken[c]) continue;
            const Edge& oe = operand.edges[c];
            const math::Vec3d op[2] = {
                operand.vertices[static_cast<std::size_t>(oe.vertex(0))]
                    .position(),
                operand.vertices[static_cast<std::size_t>(oe.vertex(1))]
                    .position()};
            const bool same = pos_eq(rp[0], op[0]) && pos_eq(rp[1], op[1]);
            const bool flipped =
                pos_eq(rp[0], op[1]) && pos_eq(rp[1], op[0]);
            if (same || flipped) {
                edge_taken[c] = true;
                return static_cast<std::int64_t>(c);
            }
        }
        for (const std::size_t c : cands) {
            if (!edge_taken[c]) {
                edge_taken[c] = true;
                return static_cast<std::int64_t>(c);
            }
        }
        return static_cast<std::int64_t>(-1);
    };
    // The operand's face by stamp.
    const auto operand_face_of = [&](const Face& f) {
        if (f.shape_id().subshape.depth == 0)
            return static_cast<std::int64_t>(-1);
        const auto& faces = operand.solid.shell().faces();
        for (std::size_t i = 0; i < faces.size(); ++i) {
            const auto& id = faces[i].shape_id();
            if (id.subshape.depth > 0 &&
                id.feature == f.shape_id().feature &&
                id.subshape == f.shape_id().subshape)
                return static_cast<std::int64_t>(i);
        }
        return static_cast<std::int64_t>(-1);
    };
    // A face's walk as a vertex-position sequence over ALL its loops
    // (the m3 fix: the m2 read took loops()[0] alone — the box faces
    // carry one loop, so nothing moved; the pierced face carries an
    // inner loop TOO, and the m3 fillet moves the INNER one (the
    // contact circle) while the outer stays: the read must cover
    // every loop or the absorbed mark would be lost).
    const auto walk_positions =
        [&](const BRepBody& body, const Face& face) {
            std::vector<math::Vec3d> out;
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges())
                    out.push_back(
                        body.vertices[static_cast<std::size_t>(
                                          ce.start_vertex(body.edges))]
                            .position());
            return out;
        };
    const auto walks_equal =
        [&pos_eq](const std::vector<math::Vec3d>& a,
                  const std::vector<math::Vec3d>& b) {
        if (a.size() != b.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!pos_eq(a[i], b[i])) return false;
        return true;
    };
    // An edge's stored pair as a position multiset (the re-spanning
    // read: the pair moved iff the positions differ).
    const auto pair_moved = [&pos_eq](const BRepBody& ra,
                                      const Edge& re,
                                      const BRepBody& oa,
                                      const Edge& oe) {
        const math::Vec3d rp[2] = {
            ra.vertices[static_cast<std::size_t>(re.vertex(0))]
                .position(),
            ra.vertices[static_cast<std::size_t>(re.vertex(1))]
                .position()};
        const math::Vec3d op[2] = {
            oa.vertices[static_cast<std::size_t>(oe.vertex(0))]
                .position(),
            oa.vertices[static_cast<std::size_t>(oe.vertex(1))]
                .position()};
        const bool same_dir =
            pos_eq(rp[0], op[0]) && pos_eq(rp[1], op[1]);
        const bool flipped =
            pos_eq(rp[0], op[1]) && pos_eq(rp[1], op[0]);
        return !(same_dir || flipped);
    };

    // ---- The result's rows, the body's own walk (vertices, edges,
    // faces) — the bookkeeping the assembly already holds, read
    // back (no new geometry, no new decisions).
    for (std::size_t i = 0; i < result.vertices.size(); ++i) {
        const std::int64_t parent = operand_vertex_of(
            result.vertices[i].position());
        if (parent >= 0) {
            table.rows.push_back(
                {Descent::Kept, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Vertex,
                   static_cast<std::uint64_t>(parent)}},
                 "", {}, ""});
        } else {
            table.rows.push_back(
                {Descent::Born, EntityKind::Vertex,
                 static_cast<std::uint64_t>(i), {}, "", {}, ""});
        }
    }
    for (std::size_t i = 0; i < result.edges.size(); ++i) {
        const std::int64_t parent =
            operand_edge_of(result.edges[i]);
        if (parent >= 0) {
            const bool absorbed = pair_moved(
                result, result.edges[i], operand,
                operand.edges[static_cast<std::size_t>(parent)]);
            table.rows.push_back(
                {absorbed ? Descent::Absorbed : Descent::Kept,
                 EntityKind::Edge, static_cast<std::uint64_t>(i),
                 {{0, EntityKind::Edge,
                   static_cast<std::uint64_t>(parent)}},
                 result.edges[i].name(), {}, ""});
        } else {
            table.rows.push_back(
                {Descent::Born, EntityKind::Edge,
                 static_cast<std::uint64_t>(i), {},
                 result.edges[i].name(), {}, ""});
        }
    }
    {
        const auto& rfaces = result.solid.shell().faces();
        for (std::size_t i = 0; i < rfaces.size(); ++i) {
            const std::int64_t parent = operand_face_of(rfaces[i]);
            if (parent >= 0) {
                const bool absorbed = !walks_equal(
                    walk_positions(result, rfaces[i]),
                    walk_positions(
                        operand,
                        operand.solid.shell().faces()[
                            static_cast<std::size_t>(parent)]));
                table.rows.push_back(
                    {absorbed ? Descent::Absorbed : Descent::Kept,
                     EntityKind::Face, static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Face,
                       static_cast<std::uint64_t>(parent)}},
                     rfaces[i].name(), {}, ""});
            } else {
                table.rows.push_back(
                    {Descent::Born, EntityKind::Face,
                     static_cast<std::uint64_t>(i), {},
                     rfaces[i].name(), {}, ""});
            }
        }
    }

    // ---- The VANISHED tail: the operand's entities no result row
    // booked — the dead vertical and its two corner vertices, the
    // cause the feature's own act: "replaced by the flank".
    {
        std::vector<bool> booked_v(operand.vertices.size(), false);
        std::vector<bool> booked_e(operand.edges.size(), false);
        std::vector<bool> booked_f(operand.face_count(), false);
        for (const auto& row : table.rows) {
            if (row.operands.empty()) continue;
            const auto& ref = row.operands[0];
            if (ref.operand != 0) continue;
            if (ref.kind == EntityKind::Vertex)
                booked_v[static_cast<std::size_t>(ref.address)] = true;
            else if (ref.kind == EntityKind::Edge)
                booked_e[static_cast<std::size_t>(ref.address)] = true;
            else
                booked_f[static_cast<std::size_t>(ref.address)] = true;
        }
        const char* cause = "replaced by the flank";
        for (std::size_t i = 0; i < operand.vertices.size(); ++i)
            if (!booked_v[i])
                table.rows.push_back(
                    {Descent::Vanished, EntityKind::Vertex,
                     static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Vertex,
                       static_cast<std::uint64_t>(i)}},
                     "", {}, cause});
        for (std::size_t i = 0; i < operand.edges.size(); ++i)
            if (!booked_e[i])
                table.rows.push_back(
                    {Descent::Vanished, EntityKind::Edge,
                     static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Edge,
                       static_cast<std::uint64_t>(i)}},
                     operand.edges[i].name(), {}, cause});
        const auto& ofaces = operand.solid.shell().faces();
        for (std::size_t i = 0; i < ofaces.size(); ++i)
            if (!booked_f[i])
                table.rows.push_back(
                    {Descent::Vanished, EntityKind::Face,
                     static_cast<std::uint64_t>(i),
                     {{0, EntityKind::Face,
                       static_cast<std::uint64_t>(i)}},
                     ofaces[i].name(), {}, cause});
    }

#ifndef NDEBUG
    // The debug-build activation (the factory precedent): BOTH sides
    // of the closure, inside — a tavola that loses or double-books
    // an entity, on either generation, cannot pass silent. The old
    // side takes the operand as A and an EMPTY B (the fillet has one
    // operand; the empty body books nothing).
    {
        const BRepBody no_second{};
        std::string why;
        if (!descent_closed(result, table, &why)) {
            throw std::logic_error(
                "fillet_descent: the tavola broke its own closure "
                "(the result side) — " + why);
        }
        if (!old_generation_closed(operand, no_second, table,
                                   &why)) {
            throw std::logic_error(
                "fillet_descent: the tavola broke its own closure "
                "(the old generation) — " + why);
        }
    }
#endif

    // The rows' identities: the RESULT rows read the result's own
    // stamps; the VANISHED tail reads the dying operand's (the
    // subtract's own discipline, verbatim).
    stamp_rows(table, result, /*vanished_rows=*/false);
    stamp_rows(table, operand, /*vanished_rows=*/true);

    return table;
}

} // namespace CAD_0::brep
