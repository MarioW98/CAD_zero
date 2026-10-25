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

} // namespace CAD_0::brep
