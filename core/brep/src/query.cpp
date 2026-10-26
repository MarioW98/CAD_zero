// core/brep/src/query.cpp
//
// The face query's implementation — ADR-0032 D8 (G1 unit 2): plain
// values out, one pass over the shell, no evaluation, no allocation
// beyond the records themselves. The p-curve payload is copied
// verbatim from the typed entities the factories wrote (D2.1: the
// sketch kernel's own numbers, untranslated).
#include "CAD_0/brep/query.hpp"

#include <sstream>
#include <stdexcept>

namespace CAD_0::brep {
namespace {

PCurveInfo describe_coedge(const CoEdge& ce) {
    PCurveInfo info;
    info.edge = ce.edge();
    info.reversed = ce.reversed();

    const PCurve* p = ce.p_curve();
    if (p == nullptr) return info;  // the faceted world — Kind::None

    switch (p->kind) {
    case PCurve::Kind::Segment:
        info.kind = PCurveInfo::Kind::Line;
        info.a = p->segment.a;
        info.b = p->segment.b;
        info.interior_left = p->segment.interior_left;
        break;
    case PCurve::Kind::Circle:
        info.kind = PCurveInfo::Kind::Circle;
        info.center = p->circle.center;
        info.radius = p->circle.radius;
        info.hole = p->circle.hole;
        break;
    case PCurve::Kind::Arc:
        info.kind = PCurveInfo::Kind::Arc;
        info.center = p->arc.center;
        info.radius = p->arc.radius;
        info.hole = p->arc.hole;
        info.start_angle = p->arc.start_angle;
        info.sweep = p->arc.sweep;
        break;
    }
    return info;
}

FaceInfo describe_face(const Face& face) {
    FaceInfo info;
    if (const SurfaceGeometry* surf = face.surface()) {
        info.has_surface = true;
        info.family = surf->type_name();
    }
    info.loop_count = face.loops().size();
    info.loops.reserve(face.loops().size());
    for (const auto& loop : face.loops()) {
        LoopInfo li;
        li.is_outer = loop.is_outer();
        if (li.is_outer) ++info.outer_loop_count;
        else ++info.inner_loop_count;
        li.coedges.reserve(loop.coedges().size());
        for (const auto& ce : loop.coedges())
            li.coedges.push_back(describe_coedge(ce));
        info.loops.push_back(std::move(li));
    }
    info.exact = face.provenance().is_exact();
    info.linear_tolerance = face.provenance().linear_tolerance;
    return info;
}

} // namespace

FaceInfo query_face(const BRepBody& body, FaceId face_id) {
    for (const auto& face : body.solid.shell().faces())
        if (face.id() == face_id) return describe_face(face);

    // Loud, with the remedy (the construction-time culture): a face
    // id that does not exist is a caller bug — name the range and
    // the enumeration entry point instead of guessing an answer.
    std::ostringstream os;
    os << "query_face: no face with id " << face_id << " in this body ("
       << body.face_count() << " face(s) in the shell) — remedy: ids come "
       << "from the body's own face numbering (the factories number by "
       << "shell order); enumerate with query_all_faces() or "
       << "CAD_0.brep.face_count() before querying.";
    throw std::invalid_argument(os.str());
}

std::vector<FaceInfo> query_all_faces(const BRepBody& body) {
    std::vector<FaceInfo> out;
    out.reserve(body.face_count());
    for (const auto& face : body.solid.shell().faces())
        out.push_back(describe_face(face));
    return out;
}

// ---------------------------------------------------------------------------
// The body query (G1 unit 6): a thin aggregation of the authorities —
// every number below is another pinned function's own answer.
// ---------------------------------------------------------------------------

namespace {

// One kind's tally: exact, approximated, and the max declared bound.
// The max is over ALL subshapes of the kind (an exact subshape
// contributes 0 — the pairing IS the contract), so the report's
// "max tolerance" is the honest worst case, never diluted.
ProvenanceTally tally(const std::vector<ProvenanceInfo>& records) {
    ProvenanceTally t;
    for (const ProvenanceInfo& p : records) {
        if (p.is_exact()) {
            ++t.exact;
        } else {
            ++t.approximated;
            if (p.linear_tolerance > t.max_linear_tolerance)
                t.max_linear_tolerance = p.linear_tolerance;
        }
    }
    return t;
}

} // namespace

BodySummary query_body(const BRepBody& body) {
    // The empty body refuses, loudly (the P2 pattern): there is no
    // shell to summarize, and χ = 0 / genus 1 painted on nothing
    // would be a lie with a straight face. The remedy names the
    // consumers that already refuse it — mass_properties for the same
    // reason, the bridge report on entry.
    if (body.vertices.empty() && body.edges.empty() &&
        body.face_count() == 0) {
        std::ostringstream os;
        os << "query_body: the empty body has no topology to report "
              "(no vertices, no edges, no faces) — remedy: pass a body "
              "with a shell (a factory product, an exact "
              "materialization, or a bridge extraction; "
              "mass_properties refuses the empty body for the same "
              "reason, and the bridge report refuses it on entry).";
        throw std::invalid_argument(os.str());
    }

    BodySummary s;
    s.vertex_count = body.vertex_count();
    s.edge_count = body.edge_count();
    s.face_count = body.face_count();
    s.inner_loops = inner_loop_count(body);
    s.chi = euler_characteristic(body);
    s.orientable = is_closed_orientable(body);
    s.loops_closed = loops_are_closed(body);

    // The genus, under the bridge report's own rule (one authority:
    // the same gate, the same function): derived only for a body the
    // structural contract already vouches for, with genus_from_chi
    // encoding the χ ≤ 2 / even gates itself.
    if (s.orientable) {
        if (const std::optional<std::int64_t> g = genus_from_chi(s.chi)) {
            s.genus_derived = true;
            s.genus = *g;
        }
    }

    // The provenance tallies, per kind (D8: "n exact / m
    // approximated, with the max tolerance" — never a guess, never
    // laundered).
    std::vector<ProvenanceInfo> records;
    records.reserve(body.vertices.size());
    for (const auto& v : body.vertices) records.push_back(v.provenance());
    s.vertices = tally(records);

    records.clear();
    records.reserve(body.edges.size());
    for (const auto& e : body.edges) records.push_back(e.provenance());
    s.edges = tally(records);

    records.clear();
    records.reserve(body.face_count());
    for (const auto& f : body.solid.shell().faces())
        records.push_back(f.provenance());
    s.faces = tally(records);

    return s;
}

} // namespace CAD_0::brep
