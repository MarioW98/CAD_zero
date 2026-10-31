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

} // namespace

DescentTable subtract_descent(const BRepBody& result, const BRepBody& A,
                              const BRepBody& B) {
    const std::size_t V_A = A.vertices.size();
    const std::size_t E_A = A.edges.size();
    const std::size_t F_A = A.face_count();

    // The outcome classification, read from the result's own counts
    // (the tavola describes what the result IS — no new geometry, no
    // new decisions): the through-hole appends at least one wall face
    // (the seam-wired circle's ONE, a chain's n); the miss is the
    // exact copy.
    const bool through = result.face_count() > F_A;

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

    return table;
}

} // namespace CAD_0::brep
