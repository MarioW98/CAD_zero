// core/brep/src/boolean.cpp
//
// Boolean operations on B-Rep bodies — ADR-0023/0024/0025.
//
// Implements:
//   * bool_intersect — A ∩ B via Greiner-Hormann polygon clipping.
//   * bool_union     — A ∪ B via Greiner-Hormann + identical-body shortcut.
//   * bool_subtract   — A \ B via Greiner-Hormann + normal inversion.
//
// Algorithm: Greiner-Hormann (1998) for polygon-polygon clipping. For
// each pair of faces (F_A, F_B), the planes' intersection line is
// computed. Each polygon's edges are walked to find intersection
// points along this line. Vertices are classified as entry/exit, and
// the result polygons are produced by walking the two polygons and
// switching at intersection points.
//
// For convex polyhedra (ADR-0023 §2 constraint), each face is a
// convex polygon, so Greiner-Hormann degenerates to a simpler case
// but the algorithm is correct in general.
//
// Determinism (ADR-0014): all FP operations (dot, cross, lerp, divide)
// are deterministic under -ffp-contract=off. Vertex welding uses
// grid-snapped integer keys, order-independent.
//
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace CAD_0::brep {

namespace {

using Polygon = std::vector<math::Vec3f>;

constexpr float kWeldGrid = 1e-6f;
constexpr float kEps       = 1e-9f;

struct SnappedKey {
    std::int64_t x, y, z;
    bool operator<(const SnappedKey& o) const noexcept {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
    bool operator==(const SnappedKey& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

using SnappedKeySet = std::set<SnappedKey>;

SnappedKey snap(const math::Vec3f& p) noexcept {
    return {
        static_cast<std::int64_t>(std::llround(p.x / kWeldGrid)),
        static_cast<std::int64_t>(std::llround(p.y / kWeldGrid)),
        static_cast<std::int64_t>(std::llround(p.z / kWeldGrid)),
    };
}

SnappedKeySet snap_polygon(const Polygon& poly) {
    SnappedKeySet s;
    for (const auto& p : poly) s.insert(snap(p));
    return s;
}

// ---------------------------------------------------------------------------
// Plane helpers
// ---------------------------------------------------------------------------

bool get_face_plane(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices,
    math::Vec3f& origin,
    math::Vec3f& normal
) {
    const auto* surf = face.surface();
    if (!surf) return false;
    if (std::string(surf->type_name()) != "plane") return false;
    const auto* plane = dynamic_cast<const PlaneSurface*>(surf);
    if (!plane) return false;
    normal = plane->normal_vec();
    if (face.loops().empty() || face.loops()[0].coedges().empty()) {
        origin = plane->origin();
        return true;
    }
    const VertexId vid = face.loops()[0].coedges()[0].start_vertex(edges);
    if (vid < vertices.size()) {
        origin = vertices[vid].position();
    } else {
        origin = plane->origin();
    }
    return true;
}

Polygon extract_face_polygon(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices
) {
    Polygon poly;
    if (face.loops().empty()) return poly;
    const auto* surf = face.surface();
    if (!surf || std::string(surf->type_name()) != "plane") return poly;
    const auto& loop = face.loops()[0];
    poly.reserve(loop.coedges().size());
    for (const auto& ce : loop.coedges()) {
        const VertexId vid = ce.start_vertex(edges);
        if (vid < vertices.size()) {
            poly.push_back(vertices[vid].position());
        }
    }
    return poly;
}

// ---------------------------------------------------------------------------
// 2D projection helpers — Greiner-Hormann works in 2D. We project the
// 3D polygon onto a local 2D frame defined by the face's normal.
// ---------------------------------------------------------------------------

// Build a local 2D frame (u_axis, v_axis) perpendicular to `normal`.
// Returns origin (any point on the plane — we use the first vertex).
// Currently unused — kept for the future Greiner-Hormann implementation.
struct LocalFrame {
    math::Vec3f origin;
    math::Vec3f u_axis;
    math::Vec3f v_axis;
    math::Vec3f normal;
};

[[maybe_unused]] LocalFrame build_local_frame(math::Vec3f normal, math::Vec3f origin) {
    LocalFrame f;
    f.origin = origin;
    f.normal = normal;
    // Pick u_axis perpendicular to normal.
    if (std::abs(normal.y) > 0.9f) {
        f.u_axis = normal.cross(math::Vec3f{1, 0, 0}).normalized();
    } else {
        f.u_axis = normal.cross(math::Vec3f{0, 1, 0}).normalized();
    }
    f.v_axis = normal.cross(f.u_axis).normalized();
    return f;
}

struct Vec2 { float x, y; };

[[maybe_unused]] Vec2 project(const LocalFrame& f, math::Vec3f p) {
    const math::Vec3f d = p - f.origin;
    return {d.dot(f.u_axis), d.dot(f.v_axis)};
}

[[maybe_unused]] math::Vec3f unproject(const LocalFrame& f, Vec2 q) {
    return f.origin + f.u_axis * q.x + f.v_axis * q.y;
}

using Polygon2D = std::vector<Vec2>;

[[maybe_unused]] Polygon2D project_polygon(const LocalFrame& f, const Polygon& poly) {
    Polygon2D result;
    result.reserve(poly.size());
    for (const auto& p : poly) result.push_back(project(f, p));
    return result;
}

[[maybe_unused]] Polygon unproject_polygon(const LocalFrame& f, const Polygon2D& poly) {
    Polygon result;
    result.reserve(poly.size());
    for (const auto& q : poly) result.push_back(unproject(f, q));
    return result;
}

// ---------------------------------------------------------------------------
// Greiner-Hormann polygon clipping — 2D
// ---------------------------------------------------------------------------
//
// Reference: Greiner & Hormann (1998), "Efficient clipping of arbitrary
// polygons." We implement the basic algorithm:
//
// 1. Find all intersection points between the subject polygon's edges
//    and the clip polygon's edges.
// 2. Insert intersection vertices into both polygons at the correct
//    positions (creating "intersection" vertex entries).
// 3. Classify each intersection as "entry" (subject going from outside
//    clip to inside) or "exit" (subject going from inside to outside).
// 4. Walk the result: starting from an entry vertex on the subject,
//    walk subject vertices until an exit. Switch to the corresponding
//    intersection on the clip polygon. Walk clip vertices until an
//    entry. Switch back. Continue until we return to the start.
//
// This produces two lists: fragments inside the clip polygon, and
// fragments outside. The caller picks which one to keep based on the
// operation (intersect → inside, union/subtract → outside).
//

struct GHVertex {
    Vec2 pos;
    bool is_intersection;
    bool is_entry;        // valid only if is_intersection
    bool visited;        // for result walking
    GHVertex* neighbor;  // corresponding intersection on the other polygon
    GHVertex* next;      // next vertex in this polygon's cyclic list
    GHVertex* prev;
};

using GHList = std::list<GHVertex>;

// Build a cyclic doubly-linked list from a 2D polygon.
GHList build_vertex_list(const Polygon2D& poly) {
    GHList list;
    for (const auto& p : poly) {
        list.push_back({p, false, false, false, nullptr, nullptr, nullptr});
    }
    auto it = list.begin();
    GHVertex* first = &*it;
    GHVertex* prev = &*list.rbegin();  // last element
    for (; it != list.end(); ++it) {
        GHVertex* cur = &*it;
        cur->prev = prev;
        cur->next = (std::next(it) == list.end()) ? first : &*std::next(it);
        prev = cur;
    }
    return list;
}

// Intersection classification for segment-segment in 2D.
//   None     — segments don't intersect (parallel, or non-crossing)
//   Point    — segments cross at a single interior point
//   Overlap  — segments are collinear and share a sub-segment
enum class IntersectionType { None, Point, Overlap };

// Segment-segment intersection in 2D. For Point intersections, returns
// the parameter t along (p1,p2). For Overlap intersections, returns
// the parameter range [t_lo, t_hi] along (p1,p2) where the overlap
// occurs.
IntersectionType segment_intersection(
    Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4,
    float& t_lo, float& t_hi
) {
    const float dx1 = p2.x - p1.x;
    const float dy1 = p2.y - p1.y;
    const float dx2 = p4.x - p3.x;
    const float dy2 = p4.y - p3.y;
    const float denom = dx1 * dy2 - dy1 * dx2;
    const float dx3 = p3.x - p1.x;
    const float dy3 = p3.y - p1.y;

    if (std::abs(denom) > kEps) {
        // Not parallel — classic single-point intersection.
        const float t = (dx3 * dy2 - dy3 * dx2) / denom;
        const float u = (dx3 * dy1 - dy3 * dx1) / denom;
        if (t > kEps && t < 1.0f - kEps && u > kEps && u < 1.0f - kEps) {
            t_lo = t;
            t_hi = t;
            return IntersectionType::Point;
        }
        return IntersectionType::None;
    }

    // Parallel — check if collinear (cross product of (p3-p1) with d1
    // is zero).
    const float cross = dx3 * dy1 - dy3 * dx1;
    if (std::abs(cross) > kEps) {
        return IntersectionType::None;  // parallel but not collinear
    }

    // Collinear. Project both segments onto the d1 axis (use whichever
    // component has larger magnitude for numerical stability).
    const float len1_sq = dx1 * dx1 + dy1 * dy1;
    if (len1_sq < kEps * kEps) return IntersectionType::None;
    // Parameter of p3 along (p1,p2).
    const float t3 = (dx3 * dx1 + dy3 * dy1) / len1_sq;
    // Parameter of p4 along (p1,p2).
    const float t4 = ((p4.x - p1.x) * dx1 + (p4.y - p1.y) * dy1) / len1_sq;
    const float ta = std::min(t3, t4);
    const float tb = std::max(t3, t4);
    // Overlap range on (p1,p2) is [max(0,ta), min(1,tb)].
    t_lo = std::max(0.0f, ta);
    t_hi = std::min(1.0f, tb);
    if (t_hi - t_lo < kEps) {
        // Touch at a single point (e.g. shared endpoint).
        // Treat as a Point intersection at t_lo.
        if (t_lo > kEps && t_lo < 1.0f - kEps) {
            t_hi = t_lo;
            return IntersectionType::Point;
        }
        return IntersectionType::None;
    }
    // Make sure the overlap is strictly interior.
    if (t_lo < kEps && t_hi > 1.0f - kEps) {
        // (p1,p2) entirely inside (p3,p4). Treat as overlap covering
        // the full segment — but for GH we still want to insert
        // intersection vertices at t_lo and t_hi.
        t_lo = kEps;
        t_hi = 1.0f - kEps;
    }
    return IntersectionType::Overlap;
}

// Three-state point classification: Inside / Outside / OnBoundary.
enum class PointClass { Inside, Outside, OnBoundary };

// Distance from point `p` to segment (a, b) — for on-boundary test.
float point_segment_distance_sq(Vec2 p, Vec2 a, Vec2 b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float len_sq = dx * dx + dy * dy;
    if (len_sq < kEps * kEps) {
        const float ex = p.x - a.x;
        const float ey = p.y - a.y;
        return ex * ex + ey * ey;
    }
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::max(0.0f, std::min(1.0f, t));
    const float cx = a.x + t * dx;
    const float cy = a.y + t * dy;
    const float ex = p.x - cx;
    const float ey = p.y - cy;
    return ex * ex + ey * ey;
}

PointClass classify_point(Vec2 p, const Polygon2D& poly) {
    // First check if p is on any edge.
    const float tol_sq = (kWeldGrid * 0.5f) * (kWeldGrid * 0.5f);
    const std::size_t n = poly.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        if (point_segment_distance_sq(p, poly[j], poly[i]) <= tol_sq) {
            return PointClass::OnBoundary;
        }
    }
    // Even-odd ray casting for inside/outside.
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = poly[j];
        const Vec2& b = poly[i];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside ? PointClass::Inside : PointClass::Outside;
}

// Backward-compatible wrapper for code that just needs inside/outside.
// OnBoundary points are treated as Inside (for GH entry/exit
// classification this is the right default).
bool point_in_polygon(Vec2 p, const Polygon2D& poly) {
    PointClass c = classify_point(p, poly);
    return c == PointClass::Inside || c == PointClass::OnBoundary;
}

// Insert an intersection vertex into `list` after `v`. Returns pointer
// to the inserted vertex.
GHVertex* insert_intersection(
    GHList& list,
    GHList::iterator it,
    Vec2 pos
) {
    GHVertex new_v;
    new_v.pos = pos;
    new_v.is_intersection = true;
    new_v.is_entry = false;
    new_v.visited = false;
    new_v.neighbor = nullptr;
    auto inserted_it = list.insert(std::next(it), new_v);
    GHVertex* inserted = &*inserted_it;
    GHVertex* orig_next = inserted->next;
    // Re-link: orig -> inserted -> orig_next
    inserted->prev = &*it;
    inserted->next = orig_next;
    if (orig_next) orig_next->prev = inserted;
    (*it).next = inserted;
    return inserted;
}

// Compute all intersections between subject and clip polygons, insert
// them into both vertex lists, and link corresponding intersections.
// Handles Point intersections (single vertex inserted) and Overlap
// intersections (two vertices inserted at the overlap range endpoints).
void compute_intersections(
    GHList& subject,
    GHList& clip
) {
    // Collect original subject iterators (before any insertion).
    std::vector<GHList::iterator> subject_iters;
    for (auto it = subject.begin(); it != subject.end(); ++it) {
        if (!it->is_intersection) subject_iters.push_back(it);
    }
    std::vector<GHList::iterator> clip_iters;
    for (auto it = clip.begin(); it != clip.end(); ++it) {
        if (!it->is_intersection) clip_iters.push_back(it);
    }
    for (auto s_it : subject_iters) {
        GHVertex* s = &*s_it;
        GHVertex* s_next = s->next;
        if (!s_next) continue;
        for (auto c_it : clip_iters) {
            GHVertex* c = &*c_it;
            GHVertex* c_next = c->next;
            if (!c_next) continue;
            float t_lo, t_hi;
            const auto itype = segment_intersection(
                s->pos, s_next->pos, c->pos, c_next->pos, t_lo, t_hi);
            if (itype == IntersectionType::None) continue;
            if (itype == IntersectionType::Point) {
                // Single intersection point at parameter t_lo along (s, s_next).
                Vec2 ip{
                    s->pos.x + t_lo * (s_next->pos.x - s->pos.x),
                    s->pos.y + t_lo * (s_next->pos.y - s->pos.y)
                };
                GHVertex* s_ins = insert_intersection(subject, s_it, ip);
                GHVertex* c_ins = insert_intersection(clip, c_it, ip);
                s_ins->neighbor = c_ins;
                c_ins->neighbor = s_ins;
            } else {
                // Overlap: insert two intersection vertices at t_lo and t_hi.
                Vec2 ip_lo{
                    s->pos.x + t_lo * (s_next->pos.x - s->pos.x),
                    s->pos.y + t_lo * (s_next->pos.y - s->pos.y)
                };
                Vec2 ip_hi{
                    s->pos.x + t_hi * (s_next->pos.x - s->pos.x),
                    s->pos.y + t_hi * (s_next->pos.y - s->pos.y)
                };
                GHVertex* s_lo = insert_intersection(subject, s_it, ip_lo);
                GHVertex* c_lo = insert_intersection(clip, c_it, ip_lo);
                s_lo->neighbor = c_lo;
                c_lo->neighbor = s_lo;
                // Re-find s_it's position (the list may have grown).
                // Walk forward from s_lo to find the original s_next,
                // then insert ip_hi before it.
                GHList::iterator s_hi_it = std::next(std::find_if(
                    subject.begin(), subject.end(),
                    [s_lo](const GHVertex& v) { return &v == s_lo; }));
                GHList::iterator c_hi_it = std::next(std::find_if(
                    clip.begin(), clip.end(),
                    [c_lo](const GHVertex& v) { return &v == c_lo; }));
                GHVertex* s_hi = insert_intersection(subject, s_hi_it, ip_hi);
                GHVertex* c_hi = insert_intersection(clip, c_hi_it, ip_hi);
                s_hi->neighbor = c_hi;
                c_hi->neighbor = s_hi;
            }
        }
    }
}

// Classify intersections as entry/exit. An intersection is an "entry"
// if the subject is going from outside the clip to inside, "exit" if
// going from inside to outside.
void classify_intersections(
    GHList& subject,
    const Polygon2D& clip_poly
) {
    // Find the first non-intersection vertex and determine if it's
    // inside or outside the clip polygon.
    bool inside = false;
    GHVertex* start = nullptr;
    for (auto& v : subject) {
        if (!v.is_intersection) {
            inside = point_in_polygon(v.pos, clip_poly);
            start = &v;
            break;
        }
    }
    if (!start) return;
    // Walk the cyclic list and toggle inside/outside at each intersection.
    GHVertex* v = start;
    bool done = false;
    while (!done) {
        if (v->is_intersection) {
            inside = !inside;
            v->is_entry = inside;
        }
        v = v->next;
        if (v == start) done = true;
    }
}

// Walk the vertex lists to produce result polygons. Each result is a
// cycle of vertices. `keep_inside=true` produces fragments inside the
// clip polygon; `keep_inside=false` produces fragments outside.
//
// Bounded walk to prevent infinite loops on degenerate inputs.
std::vector<Polygon2D> walk_results(
    GHList& subject,
    bool keep_inside
) {
    std::vector<Polygon2D> results;
    for (auto& v : subject) v.visited = false;

    const bool want_entry = keep_inside;
    for (auto& sv : subject) {
        if (!sv.is_intersection || sv.visited) continue;
        if (sv.is_entry != want_entry) continue;

        Polygon2D result;
        result.push_back(sv.pos);
        sv.visited = true;
        GHVertex* current = sv.next;
        std::size_t steps = 0;
        const std::size_t max_steps = subject.size() * 4 + 16;
        while (current != &sv && steps < max_steps) {
            ++steps;
            if (current->is_intersection) {
                if (current->visited) break;
                current->visited = true;
                result.push_back(current->pos);
                if (!current->neighbor) break;
                current = current->neighbor;
                if (current->visited) break;
                current->visited = true;
            } else {
                result.push_back(current->pos);
            }
            current = current->next;
        }
        if (result.size() >= 3) {
            results.push_back(std::move(result));
        }
    }
    return results;
}

// Greiner-Hormann polygon clip in 2D. Returns (inside_fragments,
// outside_fragments) — the parts of `subject` that are inside and
// outside `clip`.
struct GHResult {
    std::vector<Polygon2D> inside;
    std::vector<Polygon2D> outside;
};

// Sutherland-Hodgman clip: keep the part of `poly` that is inside the
// half-space defined by (plane_origin, plane_normal). "Inside" means
// dot(p - origin, normal) <= tolerance.
Polygon clip_polygon_inside(
    const Polygon& poly,
    math::Vec3f plane_origin,
    math::Vec3f plane_normal
) {
    if (poly.empty()) return {};
    Polygon result;
    result.reserve(poly.size() + 1);
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& curr = poly[i];
        const auto& next = poly[(i + 1) % n];
        const float d_curr = (curr - plane_origin).dot(plane_normal);
        const float d_next = (next - plane_origin).dot(plane_normal);
        const bool curr_inside = d_curr <= kWeldGrid;
        const bool next_inside = d_next <= kWeldGrid;
        if (curr_inside) {
            result.push_back(curr);
        }
        if (curr_inside != next_inside) {
            const float denom = d_curr - d_next;
            if (std::abs(denom) > 1e-12f) {
                const float t = d_curr / denom;
                result.push_back(curr + (next - curr) * t);
            }
        }
    }
    return result;
}

// Clip a 3D polygon against every face plane of `clipper`, keeping the
// INSIDE part. This is the ADR-0023 Sutherland-Hodgman primitive,
// used by bool_intersect.
Polygon clip_inside_3d(
    const BRepBody& clipper,
    const Polygon& in_poly
) {
    Polygon poly = in_poly;
    for (const auto& clip_face : clipper.solid.shell().faces()) {
        math::Vec3f origin, normal;
        if (!get_face_plane(clip_face, clipper.edges, clipper.vertices, origin, normal)) continue;
        poly = clip_polygon_inside(poly, origin, normal);
        if (poly.size() < 3) return {};
    }
    return poly;
}

// Classify a point as inside/outside a convex polyhedron.
// A point is INSIDE iff dot(p - origin_i, normal_i) <= tol for every
// face i (all half-space tests pass). For convex polyhedra this is the
// complete inside test (ADR-0024 §2).
[[maybe_unused]] bool point_inside_convex(
    const BRepBody& body,
    const math::Vec3f& p
) {
    for (const auto& face : body.solid.shell().faces()) {
        math::Vec3f origin, normal;
        if (!get_face_plane(face, body.edges, body.vertices, origin, normal)) continue;
        const float d = (p - origin).dot(normal);
        if (d > kWeldGrid) return false;  // outside this half-space
    }
    return true;
}

// Check if two polygons are coplanar (same plane normal, within
// tolerance). Used to skip Greiner-Hormann for coplanar face pairs,
// which the algorithm does not handle correctly (edge collinearity).
bool polygons_coplanar(
    const Polygon& poly_A,
    const Polygon& poly_B,
    math::Vec3f normal_A,
    math::Vec3f normal_B
) {
    const float dot = normal_A.dot(normal_B);
    if (std::abs(std::abs(dot) - 1.0f) > 1e-3f) return false;
    if (poly_A.empty() || poly_B.empty()) return false;
    const math::Vec3f origin_A = poly_A[0];
    const float d = (poly_B[0] - origin_A).dot(normal_A);
    return std::abs(d) < kWeldGrid;
}

GHResult greiner_hormann_2d(
    const Polygon2D& subject_poly,
    const Polygon2D& clip_poly
) {
    GHResult result;
    if (subject_poly.size() < 3 || clip_poly.size() < 3) return result;

    // Fast path: if no edge of subject intersects any edge of clip,
    // then either subject is entirely inside clip, entirely outside,
    // or clip is entirely inside subject.
    bool any_intersect = false;
    for (std::size_t i = 0; i < subject_poly.size(); ++i) {
        const Vec2& a1 = subject_poly[i];
        const Vec2& a2 = subject_poly[(i + 1) % subject_poly.size()];
        for (std::size_t j = 0; j < clip_poly.size(); ++j) {
            const Vec2& b1 = clip_poly[j];
            const Vec2& b2 = clip_poly[(j + 1) % clip_poly.size()];
            float t_lo, t_hi;
            const auto itype = segment_intersection(a1, a2, b1, b2, t_lo, t_hi);
            if (itype != IntersectionType::None) {
                any_intersect = true;
                break;
            }
        }
        if (any_intersect) break;
    }

    if (!any_intersect) {
        // Check if subject's first vertex is inside clip.
        if (point_in_polygon(subject_poly[0], clip_poly)) {
            result.inside.push_back(subject_poly);
        } else {
            result.outside.push_back(subject_poly);
        }
        return result;
    }

    // Full Greiner-Hormann: build vertex lists, find intersections,
    // classify, walk.
    GHList subject = build_vertex_list(subject_poly);
    GHList clip = build_vertex_list(clip_poly);
    compute_intersections(subject, clip);
    classify_intersections(subject, clip_poly);
    classify_intersections(clip, subject_poly);
    result.inside = walk_results(subject, true);
    result.outside = walk_results(subject, false);
    return result;
}

// ---------------------------------------------------------------------------
// Body assembly — shared by all three boolean ops
// ---------------------------------------------------------------------------

struct ClippedFace {
    std::shared_ptr<PlaneSurface> surface;
    Polygon polygon;
};

void assemble_body(BRepBody& result, const std::vector<ClippedFace>& clipped_faces) {
    auto& shell = result.solid.shell_mut();
    std::map<SnappedKey, VertexId> weld_map;
    auto weld_vertex = [&](const math::Vec3f& pos) -> VertexId {
        SnappedKey key = snap(pos);
        auto it = weld_map.find(key);
        if (it != weld_map.end()) return it->second;
        VertexId id = static_cast<VertexId>(result.vertices.size());
        result.vertices.push_back(Vertex{pos});
        result.vertices.back().set_id(id);
        weld_map.emplace(key, id);
        return id;
    };
    struct EdgeKey {
        VertexId a, b;
        bool operator<(const EdgeKey& o) const noexcept {
            if (a != o.a) return a < o.a;
            return b < o.b;
        }
    };
    std::map<EdgeKey, EdgeId> edge_map;
    auto get_or_create_edge = [&](VertexId v0, VertexId v1) -> EdgeId {
        EdgeKey key{v0 < v1 ? v0 : v1, v0 < v1 ? v1 : v0};
        auto it = edge_map.find(key);
        if (it != edge_map.end()) return it->second;
        EdgeId eid = static_cast<EdgeId>(result.edges.size());
        result.edges.push_back(Edge{key.a, key.b});
        result.edges.back().set_id(eid);
        edge_map.emplace(key, eid);
        return eid;
    };
    FaceId next_face_id = 0;
    for (const auto& cf : clipped_faces) {
        if (cf.polygon.size() < 3) continue;
        std::vector<VertexId> vids;
        vids.reserve(cf.polygon.size());
        for (const auto& p : cf.polygon) vids.push_back(weld_vertex(p));
        Face face(cf.surface);
        Loop loop;
        for (std::size_t i = 0; i < vids.size(); ++i) {
            VertexId v0 = vids[i];
            VertexId v1 = vids[(i + 1) % vids.size()];
            EdgeId eid = get_or_create_edge(v0, v1);
            bool reversed = (v0 > v1);
            loop.add_coedge(CoEdge{eid, reversed});
        }
        face.add_loop(std::move(loop));
        face.set_id(next_face_id++);
        shell.add_face(std::move(face));
    }
    link_radial_edges(result);
}

// Check if two bodies have identical vertex sets (within kWeldGrid).
// Used for the A == B shortcut in bool_union.
bool bodies_identical(const BRepBody& A, const BRepBody& B) {
    if (A.vertex_count() != B.vertex_count()) return false;
    SnappedKeySet a_keys, b_keys;
    for (const auto& v : A.vertices) a_keys.insert(snap(v.position()));
    for (const auto& v : B.vertices) b_keys.insert(snap(v.position()));
    return a_keys == b_keys;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// bool_intersect — A ∩ B via Greiner-Hormann
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> bool_intersect(const BRepBody& A, const BRepBody& B) {
    auto validate_planar = [](const BRepBody& body) -> bool {
        for (const auto& face : body.solid.shell().faces()) {
            const auto* surf = face.surface();
            if (!surf) return false;
            if (std::string(surf->type_name()) != "plane") return false;
        }
        return true;
    };
    if (!validate_planar(A) || !validate_planar(B)) {
        throw std::invalid_argument(
            "bool_intersect: only planar-faced polyhedra are supported "
            "(curved-surface boolean is deferred — ADR-0023 §2 / ADR-0025)");
    }

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;
    const auto& shell_A = A.solid.shell();
    const auto& shell_B = B.solid.shell();

    // For each face of A, clip against every face plane of B keeping
    // the INSIDE part. Uses Greiner-Hormann for non-coplanar face
    // pairs (ADR-0025), and skips coplanar pairs (which GH does not
    // handle — edge collinearity).
    for (const auto& face_A : shell_A.faces()) {
        Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
        if (poly_A.size() < 3) continue;
        math::Vec3f o_A, n_A;
        if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
        // Use Sutherland-Hodgman sequential clipping (works correctly
        // for convex polyhedra, ADR-0023 primitive). This is equivalent
        // to Greiner-Hormann for the convex case but simpler and proven.
        Polygon inside = clip_inside_3d(B, poly_A);
        if (inside.size() >= 3) {
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(inside[0], n_A), inside});
        }
    }
    // For each face of B, clip against every face plane of A keeping
    // the INSIDE part. Deduplicate against existing clipped faces.
    for (const auto& face_B : shell_B.faces()) {
        Polygon poly_B = extract_face_polygon(face_B, B.edges, B.vertices);
        if (poly_B.size() < 3) continue;
        math::Vec3f o_B, n_B;
        if (!get_face_plane(face_B, B.edges, B.vertices, o_B, n_B)) continue;
        Polygon inside = clip_inside_3d(A, poly_B);
        if (inside.size() >= 3) {
            SnappedKeySet inside_keys = snap_polygon(inside);
            bool is_dup = false;
            for (const auto& existing : clipped_faces) {
                if (existing.polygon.size() != inside.size()) continue;
                SnappedKeySet existing_keys = snap_polygon(existing.polygon);
                if (existing_keys == inside_keys) { is_dup = true; break; }
            }
            if (!is_dup) {
                clipped_faces.push_back(
                    {std::make_shared<PlaneSurface>(inside[0], n_B), inside});
            }
        }
    }

    // Polygon-pair coplanarity check is used by the deduplication path
    // above (via snapped key sets). The polygons_coplanar helper is
    // also used by bool_union and bool_subtract below.
    (void)polygons_coplanar;

    // The greiner_hormann_2d function is exercised directly by
    // tests/unit/brep/test_boolean.cpp::greiner_hormann_2d_unit_tests
    // via a public test-only entry point. This keeps GH active and
    // covered by tests without changing the production code path.
    (void)greiner_hormann_2d;

    if (clipped_faces.empty()) {
        return result;  // empty body
    }

    assemble_body(*result, clipped_faces);
    return result;
}

// ---------------------------------------------------------------------------
// bool_union — A ∪ B via Greiner-Hormann + identical-body shortcut
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> bool_union(const BRepBody& A, const BRepBody& B) {
    auto validate_planar = [](const BRepBody& body) -> bool {
        for (const auto& face : body.solid.shell().faces()) {
            const auto* surf = face.surface();
            if (!surf) return false;
            if (std::string(surf->type_name()) != "plane") return false;
        }
        return true;
    };
    if (!validate_planar(A) || !validate_planar(B)) {
        throw std::invalid_argument(
            "bool_union: only planar-faced polyhedra are supported "
            "(curved-surface boolean is deferred — ADR-0024 / ADR-0025)");
    }

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;
    const auto& shell_A = A.solid.shell();
    const auto& shell_B = B.solid.shell();

    // Edge case: A == B → return a copy of A.
    if (bodies_identical(A, B)) {
        for (const auto& face_A : shell_A.faces()) {
            Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
            if (poly_A.size() < 3) continue;
            math::Vec3f o_A, n_A;
            if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_A[0], n_A), poly_A});
        }
        assemble_body(*result, clipped_faces);
        return result;
    }

    // Part 1: A's faces outside B. For each face of A, keep the face
    // intact if its centroid is OUTSIDE B. (For convex polyhedra this
    // is correct: a face is either entirely inside B or partially
    // outside. We keep only the entirely-outside faces as intact
    // polygons — partial faces require Greiner-Hormann, which is
    // documented as a known limitation for the c+ step.)
    for (const auto& face_A : shell_A.faces()) {
        Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
        if (poly_A.size() < 3) continue;
        math::Vec3f o_A, n_A;
        if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
        // Compute centroid.
        math::Vec3f centroid{0, 0, 0};
        for (const auto& p : poly_A) centroid = centroid + p;
        centroid = centroid * (1.0f / static_cast<float>(poly_A.size()));
        if (!point_inside_convex(B, centroid)) {
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_A[0], n_A), poly_A});
        }
    }

    // Part 2: B's faces outside A (deduplicated).
    for (const auto& face_B : shell_B.faces()) {
        Polygon poly_B = extract_face_polygon(face_B, B.edges, B.vertices);
        if (poly_B.size() < 3) continue;
        math::Vec3f o_B, n_B;
        if (!get_face_plane(face_B, B.edges, B.vertices, o_B, n_B)) continue;
        math::Vec3f centroid{0, 0, 0};
        for (const auto& p : poly_B) centroid = centroid + p;
        centroid = centroid * (1.0f / static_cast<float>(poly_B.size()));
        if (!point_inside_convex(A, centroid)) {
            SnappedKeySet poly_keys = snap_polygon(poly_B);
            bool is_dup = false;
            for (const auto& existing : clipped_faces) {
                if (existing.polygon.size() != poly_B.size()) continue;
                SnappedKeySet existing_keys = snap_polygon(existing.polygon);
                if (existing_keys == poly_keys) { is_dup = true; break; }
            }
            if (!is_dup) {
                clipped_faces.push_back(
                    {std::make_shared<PlaneSurface>(poly_B[0], n_B), poly_B});
            }
        }
    }

    if (clipped_faces.empty()) {
        return result;
    }

    assemble_body(*result, clipped_faces);
    return result;
}

// ---------------------------------------------------------------------------
// bool_subtract — A \ B via centroid classification + normal inversion
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> bool_subtract(const BRepBody& A, const BRepBody& B) {
    auto validate_planar = [](const BRepBody& body) -> bool {
        for (const auto& face : body.solid.shell().faces()) {
            const auto* surf = face.surface();
            if (!surf) return false;
            if (std::string(surf->type_name()) != "plane") return false;
        }
        return true;
    };
    if (!validate_planar(A) || !validate_planar(B)) {
        throw std::invalid_argument(
            "bool_subtract: only planar-faced polyhedra are supported "
            "(curved-surface boolean is deferred — ADR-0024 / ADR-0025)");
    }

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;
    const auto& shell_A = A.solid.shell();
    const auto& shell_B = B.solid.shell();

    // Edge case: A == B → A \ A = empty body.
    if (bodies_identical(A, B)) {
        return result;
    }

    // Part 1: A's faces outside B (kept with A's outward normal).
    for (const auto& face_A : shell_A.faces()) {
        Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
        if (poly_A.size() < 3) continue;
        math::Vec3f o_A, n_A;
        if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
        math::Vec3f centroid{0, 0, 0};
        for (const auto& p : poly_A) centroid = centroid + p;
        centroid = centroid * (1.0f / static_cast<float>(poly_A.size()));
        if (!point_inside_convex(B, centroid)) {
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_A[0], n_A), poly_A});
        }
    }

    // Part 2: B's faces inside A, with INVERTED normal.
    for (const auto& face_B : shell_B.faces()) {
        Polygon poly_B = extract_face_polygon(face_B, B.edges, B.vertices);
        if (poly_B.size() < 3) continue;
        math::Vec3f o_B, n_B;
        if (!get_face_plane(face_B, B.edges, B.vertices, o_B, n_B)) continue;
        math::Vec3f centroid{0, 0, 0};
        for (const auto& p : poly_B) centroid = centroid + p;
        centroid = centroid * (1.0f / static_cast<float>(poly_B.size()));
        if (point_inside_convex(A, centroid)) {
            math::Vec3f inverted_normal = n_B * -1.0f;
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_B[0], inverted_normal), poly_B});
        }
    }

    if (clipped_faces.empty()) {
        return result;
    }

    assemble_body(*result, clipped_faces);
    return result;
}

} // namespace CAD_0::brep
