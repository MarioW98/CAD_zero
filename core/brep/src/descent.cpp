// core/brep/src/descent.cpp
//
// The tavola's derivation and closure check (ADR-0036 D2, H2-u6
// morso 2): pure integer bookkeeping over the body's own arrays —
// no geometry, no evaluation, no floating-point content anywhere
// (the cost derivation's own proof, in the code).
#include "CAD_0/brep/descent.hpp"

#include <map>
#include <sstream>
#include <stdexcept>
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
    // The booking: (kind, address) -> how many rows carry it.
    std::map<std::pair<std::uint8_t, std::uint64_t>, std::size_t>
        booked;
    for (const auto& row : table.rows) {
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
             static_cast<std::uint64_t>(i), {}, "", ""});
    }
    for (std::size_t i = 0; i < body.edges.size(); ++i) {
        table.rows.push_back(
            {Descent::Born, EntityKind::Edge,
             static_cast<std::uint64_t>(i), {}, body.edges[i].name(),
             ""});
    }
    for (const auto& face : body.solid.shell().faces()) {
        table.rows.push_back(
            {Descent::Born, EntityKind::Face, face.id(), {},
             face.name(), ""});
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

} // namespace CAD_0::brep
