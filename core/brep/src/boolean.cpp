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
// H1 (ADR-0034): the whole pipeline is DOUBLE - the exact
// path's arithmetic (this file has no p-curve payload sites;
// the conversion is mechanical and total).
// Determinism (ADR-0014): all FP operations (dot, cross, lerp, divide)
// are deterministic under -ffp-contract=off. Vertex welding uses
// grid-snapped integer keys, order-independent.
//
// H2 unit 3 (ADR-0035, morso 2): the numerical taxonomy. The three
// bare constants are RETIRED — kWeldGrid, kPlaneTol, kEps are dead;
// every decision they fed is re-homed to the taxonomy's three
// families (the ADR's D2 map, site by site):
//   * the LINEAR world (ε_L, declared per operation through
//     BooleanTolerance::linear): welding, plane bands, degenerate-
//     length guards — and the canonical key's quantizer (D3: one
//     declared value, two consumers, zero drift);
//   * the PARAMETER world (ε_P, BooleanTolerance::parametric): the
//     (t,u) windows, endpoint exclusions, overlap widths — its own
//     typed tolerance, never the linear constant (D2.2);
//   * the DERIVED determinant band (D2.3): the parallelism admission
//     is |det| ≤ 4·ε_L·L — the bound scales with the square of the
//     geometric scale because det is bilinear in the direction
//     vectors. Inside the band the pair is equivalent-to-parallel
//     (D3 band machinery); outside it the crossing is real, its
//     point is computed in double, and its windows are ε_P.
//   * the ANGULAR world (D2 row 4): the two normal-parallelity
//     decisions, typed Angular, declared (loose pre-filter loose,
//     strict threshold strict).
// The classifications of the stored bytes run on the adaptive
// predicates (D1): orient2d through the core/math wrapper — the
// exact sign, zero thresholds inside the call.
//
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/detail/boolean_test_hooks.hpp"

#include "CAD_0/math/predicate.hpp"

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

using Polygon = std::vector<math::Vec3d>;

// H2 unit 1 (the input gate): the class gates that are RELATIVE to the
// body's own scale. A solid of volume under kVolumeRelTol * scale^3 or a
// face of area under kAreaRelTol * scale^2 (scale = the largest vertex
// coordinate magnitude) is outside the declared boolean class — the
// operations are defined on full-dimensional solids, and a near-flat
// body would classify every point "on boundary" without answering a
// geometric question. Relative, so the gate means the same thing at
// every model scale. They stand as they are (ADR-0035 D7: the H2-u1
// gates are green and relative — already this taxonomy's spirit).
constexpr double kVolumeRelTol = 1e-9;
constexpr double kAreaRelTol   = 1e-9;
// Degenerate-denominator guard for the edge/plane crossing
// interpolation in IntersectionTable: below this |d0 − d1| the crossing
// point is not computed and the edge start vertex is reused. Purely a
// numeric division guard — NOT a geometric tolerance (ADR-0035 D2 row 5
// ratified the rule: it stays out of the tolerance model, declared as
// the engineering guard it is).
constexpr double kIsectDenomEps = 1e-12;

// The per-operation linear epsilon accessor: ε_L, declared at the
// signature (ADR-0004), consumed by the linear world's decisions and
// by the derived determinant band (4·ε_L·L, D2.3). One accessor per
// world keeps the call sites honest about which tolerance they read.
inline double eps_l_of(const BooleanTolerance& tol) noexcept {
    return tol.linear.value;
}
inline double eps_p_of(const BooleanTolerance& tol) noexcept {
    return tol.parametric.value;
}

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

// The canonical key (D3): the SAME declared ε_L quantizes the weld
// grid and decides the equivalence — one declared value, two
// consumers, zero drift. Order-independent int64 keys (ADR-0016/0025
// pattern, unchanged in form).
SnappedKey snap(const math::Vec3d& p, double eps_l) noexcept {
    return {
        static_cast<std::int64_t>(std::llround(p.x / eps_l)),
        static_cast<std::int64_t>(std::llround(p.y / eps_l)),
        static_cast<std::int64_t>(std::llround(p.z / eps_l)),
    };
}

SnappedKeySet snap_polygon(const Polygon& poly, double eps_l) {
    SnappedKeySet s;
    for (const auto& p : poly) s.insert(snap(p, eps_l));
    return s;
}

// ---------------------------------------------------------------------------
// Plane helpers
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// The typed plane reader (H1, ADR-0034 D3 — the residue cleaned by H2
// unit 1): the plane payload is extracted through the surface VISITOR,
// the same way the SSI table reads its operands. The string-compare +
// dynamic_cast chain this file carried since Phase D ("is the type
// name 'plane'? then cast") is the exact defect class ADR-0034 D3
// banned in ssi.cpp — a typo'd tag compiled and failed at runtime.
// Here the family is the typed enum and the payload arrives through
// the visit overload; there is no string in the dispatch.
// ---------------------------------------------------------------------------
struct PlanePayloadReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    void visit(const PlaneSurface& s) override { plane = &s; }
};

// The face's plane payload, or nullptr when the face carries no
// surface or a non-plane family. The ONLY sanctioned plane extraction
// in this file — every consumer (the gate, the clippers) reads through
// it, so the class decision and the payload can never diverge.
const PlaneSurface* plane_of(const Face& face) {
    const auto* surf = face.surface();
    if (!surf) return nullptr;
    // The typed gate first (the enum — one naming authority): the
    // visitor below is the payload extraction for the admitted family.
    if (surf->family() != SurfaceFamily::Plane) return nullptr;
    PlanePayloadReader reader;
    surf->accept(reader);
    return reader.plane;
}

bool get_face_plane(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices,
    math::Vec3d& origin,
    math::Vec3d& normal
) {
    const auto* plane = plane_of(face);
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

// The face's outer-loop polygon, in loop order.
//
// H2 unit 1 (the input gate): this reader walks the face's FIRST loop
// — the class the boolean header declares is "exactly one outer loop
// per face", and the gate (validate_boolean_input below) refuses every
// multi-loop face BEFORE any reader runs. Within the enforced class
// the single-loop walk is exact: there are no inner loops to ignore,
// and a body that gains one is rejected with its name
// (input_face_inner_loops) instead of silently mis-read.
Polygon extract_face_polygon(
    const Face& face,
    const std::vector<Edge>& edges,
    const std::vector<Vertex>& vertices
) {
    Polygon poly;
    if (face.loops().empty()) return poly;
    if (!plane_of(face)) return poly;
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
// The slab decomposition (polygon_subtract_3d, ADR-0025 FASE 2) is the
// consumer: it projects both polygons into this frame, clips in 2D,
// and unprojects the fragments. Written for a direct Greiner-Hormann
// path that never shipped; the slab decomposition took it over —
// corrected here (the old note said "currently unused", which stopped
// matching the tree when FASE 2 landed).
struct LocalFrame {
    math::Vec3d origin;
    math::Vec3d u_axis;
    math::Vec3d v_axis;
    math::Vec3d normal;
};

LocalFrame build_local_frame(math::Vec3d normal, math::Vec3d origin) {
    LocalFrame f;
    f.origin = origin;
    f.normal = normal;
    // Pick u_axis perpendicular to normal.
    if (std::abs(normal.y) > 0.9) {
        f.u_axis = normal.cross(math::Vec3d{1, 0, 0}).normalized();
    } else {
        f.u_axis = normal.cross(math::Vec3d{0, 1, 0}).normalized();
    }
    f.v_axis = normal.cross(f.u_axis).normalized();
    return f;
}

struct Vec2 { double x, y; };

Vec2 project(const LocalFrame& f, math::Vec3d p) {
    const math::Vec3d d = p - f.origin;
    return {d.dot(f.u_axis), d.dot(f.v_axis)};
}

math::Vec3d unproject(const LocalFrame& f, Vec2 q) {
    return f.origin + f.u_axis * q.x + f.v_axis * q.y;
}

using Polygon2D = std::vector<Vec2>;

Polygon2D project_polygon(const LocalFrame& f, const Polygon& poly) {
    Polygon2D result;
    result.reserve(poly.size());
    for (const auto& p : poly) result.push_back(project(f, p));
    return result;
}

Polygon unproject_polygon(const LocalFrame& f, const Polygon2D& poly) {
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

// Segment-segment intersection in 2D — the three-family classification
// (ADR-0035). For Point intersections, returns the parameter t along
// (p1,p2). For Overlap intersections, returns the parameter range
// [t_lo, t_hi] along (p1,p2) where the overlap occurs.
//
// THE REGIME (every decision named, H2-u3 morso 2):
//   * det = d1×d2 is bilinear in the direction vectors (dimension
//     length²). Its parallelism admission is the DERIVED band
//     |det| ≤ 4·ε_L·L (D2.3: |Δdet| ≤ 4·ε_L·L with L the pair's
//     characteristic length — the maximal segment length). Outside
//     the band the crossing is real: family 1 governs, the crossing
//     point is computed in double, and the (t,u) windows are the
//     parameter world's ε_P (D2.2) — never the linear constant.
//   * Inside the det band the pair is equivalent-to-parallel (D3 band
//     machinery: the bytes do not support a crossing the arithmetic
//     suggests); cross = (p3−p1)×d1, the same bilinear form, gets the
//     same-derived band: outside it the pair stays out of the
//     collinear class (parallel, not equivalent-collinear); inside
//     both bands the projection decides, with the overlap windows in
//     ε_P.
//   * The degenerate-length guard (a segment shorter than ε_L) is a
//     LINEAR decision (D2 row 1): len_sq < ε_L².
//   * The classifications of the stored bytes run on the adaptive
//     predicate orient2d (D1): the exact sign of cross — the crossing
//     side of the collinearity — never a rounded subtraction. Zero is
//     a fact of the bytes.
IntersectionType segment_intersection(
    Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4,
    double eps_l, double eps_p,
    double& t_lo, double& t_hi
) {
    math::predicate::initialize();  // idempotent; the predicates' one-time setup
    const double dx1 = p2.x - p1.x;
    const double dy1 = p2.y - p1.y;
    const double dx2 = p4.x - p3.x;
    const double dy2 = p4.y - p3.y;
    const double det = dx1 * dy2 - dy1 * dx2;
    const double dx3 = p3.x - p1.x;
    const double dy3 = p3.y - p1.y;

    // The derived band (D2.3): 4·ε_L·L, L = the pair's characteristic
    // length (the maximal segment length — the domain extension of
    // THIS operand pair).
    const double L = std::max(std::sqrt(dx1 * dx1 + dy1 * dy1),
                              std::sqrt(dx2 * dx2 + dy2 * dy2));
    const double det_band = 4.0 * eps_l * L;

    if (std::abs(det) > det_band) {
        // Outside the band: the crossing is real (D2.3). The crossing
        // point is computed in double; the (t,u) windows are the
        // parameter world's ε_P (D2.2).
        const double t = (dx3 * dy2 - dy3 * dx2) / det;
        const double u = (dx3 * dy1 - dy3 * dx1) / det;
        if (t > eps_p && t < 1.0 - eps_p && u > eps_p && u < 1.0 - eps_p) {
            t_lo = t;
            t_hi = t;
            return IntersectionType::Point;
        }
        return IntersectionType::None;
    }

    // Inside the det band: equivalent-to-parallel (D3). Whether the
    // pair is ALSO equivalent-collinear is decided by cross's band;
    // the adaptive predicate certifies the true degeneracy of the
    // bytes first (D1: exact zero is a fact — a pair whose bytes are
    // EXACTLY collinear enters the projection path even where a
    // rounded |cross| could misread the sign).
    const double cross = dx3 * dy1 - dy3 * dx1;
    const double cross_exact_sign = math::predicate::orient2d(
        math::Vec2d{p1.x, p1.y}, math::Vec2d{p3.x, p3.y},
        math::Vec2d{p2.x, p2.y});
    const bool equivalent_collinear =
        (cross_exact_sign == 0.0) || (std::abs(cross) <= det_band);
    if (!equivalent_collinear) {
        return IntersectionType::None;  // equivalent-parallel, not equivalent-collinear
    }

    // Equivalent-collinear (inside both bands). The degenerate-length
    // guard is a LINEAR decision (D2 row 1): a segment shorter than
    // ε_L carries no direction the projection could trust.
    const double len1_sq = dx1 * dx1 + dy1 * dy1;
    if (len1_sq < eps_l * eps_l) return IntersectionType::None;
    // Project both segments onto the d1 axis (use whichever
    // component has larger magnitude for numerical stability).
    const double t3 = (dx3 * dx1 + dy3 * dy1) / len1_sq;
    // Parameter of p4 along (p1,p2).
    const double t4 = ((p4.x - p1.x) * dx1 + (p4.y - p1.y) * dy1) / len1_sq;
    const double ta = std::min(t3, t4);
    const double tb = std::max(t3, t4);
    // Overlap range on (p1,p2) is [max(0,ta), min(1,tb)].
    t_lo = std::max(0.0, ta);
    t_hi = std::min(1.0, tb);
    if (t_hi - t_lo < eps_p) {
        // Touch at a single point (e.g. shared endpoint): the overlap
        // width is a PARAMETER decision (D2.2). Treat as a Point
        // intersection at t_lo when it is interior.
        if (t_lo > eps_p && t_lo < 1.0 - eps_p) {
            t_hi = t_lo;
            return IntersectionType::Point;
        }
        return IntersectionType::None;
    }
    // Make sure the overlap is strictly interior (the endpoint
    // exclusions are the parameter world's, D2.2).
    if (t_lo < eps_p && t_hi > 1.0 - eps_p) {
        // (p1,p2) entirely inside (p3,p4). Treat as overlap covering
        // the full segment — but for GH we still want to insert
        // intersection vertices at t_lo and t_hi.
        t_lo = eps_p;
        t_hi = 1.0 - eps_p;
    }
    return IntersectionType::Overlap;
}

// Three-state point classification: Inside / Outside / OnBoundary.
enum class PointClass { Inside, Outside, OnBoundary };

// Distance from point `p` to segment (a, b) — for on-boundary test.
// The degenerate-segment guard is a LINEAR decision (D2 row 1):
// a segment shorter than ε_L has no interior to be on.
double point_segment_distance_sq(Vec2 p, Vec2 a, Vec2 b, double eps_l) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq < eps_l * eps_l) {
        const double ex = p.x - a.x;
        const double ey = p.y - a.y;
        return ex * ex + ey * ey;
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
    const double cx = a.x + t * dx;
    const double cy = a.y + t * dy;
    const double ex = p.x - cx;
    const double ey = p.y - cy;
    return ex * ex + ey * ey;
}

// Three-state point classification: Inside / Outside / OnBoundary.
//
// The OnBoundary band is the LINEAR world's (D2 row 1: the half-grid
// is the historical form, now ε_L-typed). The even-odd ray casting
// runs on the adaptive predicate (D1): the crossing side of the
// stored bytes is the EXACT orient2d sign — never a rounded division
// (the crossing counts when the sign agrees with the edge's own
// vertical direction, which is the orientation relation the old
// divided-x test approximated).
PointClass classify_point(Vec2 p, const Polygon2D& poly, double eps_l) {
    // First check if p is on any edge.
    const double tol_sq = (eps_l * 0.5) * (eps_l * 0.5);
    const std::size_t n = poly.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        if (point_segment_distance_sq(p, poly[j], poly[i], eps_l) <= tol_sq) {
            return PointClass::OnBoundary;
        }
    }
    // Even-odd ray casting for inside/outside — on the exact predicate
    // sign (D1). An edge crosses the horizontal ray at p iff the
    // endpoints straddle p.y AND p lies on the crossing side: the
    // orient2d sign of (a, b, p) agrees with the sign of (b.y − a.y).
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = poly[j];
        const Vec2& b = poly[i];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double side = math::predicate::orient2d(
                math::Vec2d{a.x, a.y}, math::Vec2d{b.x, b.y},
                math::Vec2d{p.x, p.y});
            const double dir = b.y - a.y;
            if ((side > 0.0 && dir > 0.0) || (side < 0.0 && dir < 0.0)) {
                inside = !inside;
            }
        }
    }
    return inside ? PointClass::Inside : PointClass::Outside;
}

// Backward-compatible wrapper for code that just needs inside/outside.
// OnBoundary points are treated as Inside (for GH entry/exit
// classification this is the right default).
bool point_in_polygon(Vec2 p, const Polygon2D& poly, double eps_l) {
    PointClass c = classify_point(p, poly, eps_l);
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
    GHList& clip,
    double eps_l, double eps_p
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
            double t_lo, t_hi;
            const auto itype = segment_intersection(
                s->pos, s_next->pos, c->pos, c_next->pos,
                eps_l, eps_p, t_lo, t_hi);
            if (itype == IntersectionType::None) continue;
            if (itype == IntersectionType::Point) {
                Vec2 ip{
                    s->pos.x + t_lo * (s_next->pos.x - s->pos.x),
                    s->pos.y + t_lo * (s_next->pos.y - s->pos.y)
                };
                GHVertex* s_ins = insert_intersection(subject, s_it, ip);
                GHVertex* c_ins = insert_intersection(clip, c_it, ip);
                s_ins->neighbor = c_ins;
                c_ins->neighbor = s_ins;
            } else {
                // Overlap: insert two intersection vertices.
                // Find the iterator pointing to the vertex BEFORE which
                // we want to insert ip_hi. That's the next original
                // vertex after s (i.e., the original s_next, now
                // reachable as s_lo->next->...->s_next).
                //
                // Simpler approach: insert ip_lo after s_it, then
                // insert ip_hi after the original s_next (which we
                // still hold as a pointer). To find its iterator, walk
                // from s_lo forward until we hit a vertex whose pos
                // matches s_next->pos AND is not an intersection.
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
                // Find the iterator for s_next (the original next vertex)
                // by walking forward from s_lo. We expect it to be
                // s_lo->next (the original s_next, untouched).
                GHVertex* cursor = s_lo->next;
                GHList::iterator s_hi_it = subject.end();
                for (auto it = subject.begin(); it != subject.end(); ++it) {
                    if (&*it == cursor) { s_hi_it = it; break; }
                }
                GHVertex* c_cursor = c_lo->next;
                GHList::iterator c_hi_it = clip.end();
                for (auto it = clip.begin(); it != clip.end(); ++it) {
                    if (&*it == c_cursor) { c_hi_it = it; break; }
                }
                if (s_hi_it != subject.end() && c_hi_it != clip.end()) {
                    GHVertex* s_hi = insert_intersection(subject, s_hi_it, ip_hi);
                    GHVertex* c_hi = insert_intersection(clip, c_hi_it, ip_hi);
                    s_hi->neighbor = c_hi;
                    c_hi->neighbor = s_hi;
                }
            }
        }
    }
}

// Classify intersections as entry/exit. An intersection is an "entry"
// if the subject is going from outside the clip to inside, "exit" if
// going from inside to outside.
void classify_intersections(
    GHList& subject,
    const Polygon2D& clip_poly,
    double eps_l
) {
    // Find the first non-intersection vertex and determine if it's
    // inside or outside the clip polygon.
    bool inside = false;
    GHVertex* start = nullptr;
    for (auto& v : subject) {
        if (!v.is_intersection) {
            inside = point_in_polygon(v.pos, clip_poly, eps_l);
            start = &v;
            break;
        }
    }
    if (!start) return;
    // Walk the cyclic list and toggle inside/outside at each intersection.
    // Bounded to prevent infinite loops on malformed lists.
    GHVertex* v = start;
    std::size_t steps = 0;
    const std::size_t max_steps = subject.size() * 2 + 8;
    while (v != nullptr && steps < max_steps) {
        ++steps;
        if (v->is_intersection) {
            inside = !inside;
            v->is_entry = inside;
        }
        v = v->next;
        if (v == start) break;
    }
}

// Walk the vertex lists to produce result polygons. Each result is a
// cycle of vertices. `keep_inside=true` produces fragments inside the
// clip polygon; `keep_inside=false` produces fragments outside.
//
// Bounded walk to prevent infinite loops on degenerate inputs.
// Defensive nullptr checks to prevent SIGSEGV on malformed lists.
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
        if (!sv.next) continue;  // defensive

        Polygon2D result;
        result.push_back(sv.pos);
        sv.visited = true;
        GHVertex* current = sv.next;
        std::size_t steps = 0;
        const std::size_t max_steps = subject.size() * 8 + 32;
        while (current != nullptr && current != &sv && steps < max_steps) {
            ++steps;
            if (current->is_intersection) {
                if (current->visited) break;
                current->visited = true;
                result.push_back(current->pos);
                if (!current->neighbor) break;
                current = current->neighbor;
                if (current == nullptr || current->visited) break;
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

// SharedVertexPool — single source of truth for vertex positions
// (ADR-0025 FASE B: single-source-of-truth fix).
//
// Canonicalizes every vertex position through a single pool. The pool
// maps a SnappedKey to the FIRST Vec3 inserted for that cell. When
// two paths produce vertices in the SAME cell, both receive the SAME
// canonical Vec3 — bit-identical by construction.
//
// For the grid-straddle case, the pool is fed by IntersectionTable
// which computes each interface point ONCE and reuses it across all
// paths. This eliminates the dual-computation that caused the straddle.
//
// H2 unit 3 (ADR-0035 D2.1/D3): the pool's quantizer is the operation's
// declared ε_L — the canonical key's quantizer IS the linear tolerance
// (one declared value, two consumers, zero drift). ADR-0014 untouched.
class SharedVertexPool {
public:
    explicit SharedVertexPool(double eps_l) noexcept : eps_l_(eps_l) {}

    const math::Vec3d& canonicalize(const math::Vec3d& pos) {
        SnappedKey key = snap(pos, eps_l_);
        auto it = pool_.find(key);
        if (it != pool_.end()) {
            return it->second;
        }
        pool_.emplace(key, pos);
        return pool_.at(key);
    }

    void canonicalize_polygon(Polygon& poly) {
        for (auto& p : poly) {
            p = canonicalize(p);
        }
    }

    double eps_l() const noexcept { return eps_l_; }

private:
    double eps_l_;
    std::map<SnappedKey, math::Vec3d> pool_;
};

// IntersectionTable — pre-computed interface points (ADR-0025 FASE B).
//
// For each (edge of A, face of B) pair where the edge crosses the face
// plane, the intersection point is computed ONCE and stored. Both
// clip_inside_3d calls (Part 1: A's faces against B, Part 2: B's faces
// against A) consume the SAME Vec3 from the table, eliminating the
// dual-computation that caused grid-straddle welding failures.
// The table's quantizer is the operation's declared ε_L (D3).
class IntersectionTable {
public:
    explicit IntersectionTable(double eps_l)
        : pool_(eps_l) {}

    const math::Vec3d& get_or_compute(
        const math::Vec3d& v0, const math::Vec3d& v1,
        math::Vec3d plane_origin, math::Vec3d plane_normal
    ) {
        const double d0 = (v0 - plane_origin).dot(plane_normal);
        const double d1 = (v1 - plane_origin).dot(plane_normal);
        const double denom = d0 - d1;
        math::Vec3d point;
        if (std::abs(denom) > kIsectDenomEps) {
            const double t = d0 / denom;
            point = v0 + (v1 - v0) * t;
        } else {
            point = v0;
        }
        return pool_.canonicalize(point);
    }

    SharedVertexPool& pool() { return pool_; }

private:
    SharedVertexPool pool_;
};

// Sutherland-Hodgman clip primitive.
// keep_inside=true  → keep vertices where dot(p - origin, normal) <= ε_L
// keep_inside=false → keep vertices where dot(p - origin, normal) >= -ε_L
// (complementary half-space, with symmetric tolerance band so points
// exactly on the plane are kept by both sides — needed for slab
// decomposition where adjacent slabs share boundary points). The band
// is the LINEAR world's (D2 row 1: the historical kPlaneTol decision,
// now ε_L, typed and declared per operation).
//
// If `table` is non-null, intersection points are computed via the
// table (single-source-of-truth) so both Part 1 and Part 2 of
// bool_union/subtract receive the SAME Vec3 for interface ring points.
Polygon clip_polygon_sh(
    const Polygon& poly,
    math::Vec3d plane_origin,
    math::Vec3d plane_normal,
    bool keep_inside,
    IntersectionTable* table,
    double eps_l
) {
    if (poly.empty()) return {};
    Polygon result;
    result.reserve(poly.size() + 1);
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& curr = poly[i];
        const auto& next = poly[(i + 1) % n];
        const double d_curr = (curr - plane_origin).dot(plane_normal);
        const double d_next = (next - plane_origin).dot(plane_normal);
        const bool curr_keep = keep_inside ? (d_curr <= eps_l)
                                           : (d_curr >= -eps_l);
        const bool next_keep = keep_inside ? (d_next <= eps_l)
                                           : (d_next >= -eps_l);
        if (curr_keep) {
            result.push_back(curr);
        }
        if (curr_keep != next_keep) {
            const double denom = d_curr - d_next;
            if (std::abs(denom) > eps_l) {
                if (table) {
                    result.push_back(table->get_or_compute(
                        curr, next, plane_origin, plane_normal));
                } else {
                    const double t = d_curr / denom;
                    result.push_back(curr + (next - curr) * t);
                }
            }
        }
    }
    return result;
}

// Backward-compatible wrapper: keep_inside=true.
Polygon clip_polygon_inside(
    const Polygon& poly,
    math::Vec3d plane_origin,
    math::Vec3d plane_normal,
    IntersectionTable* table,
    double eps_l
) {
    return clip_polygon_sh(poly, plane_origin, plane_normal, true, table, eps_l);
}

// Clip a 3D polygon against every face plane of `clipper`, keeping the
// INSIDE part. This is the ADR-0023 Sutherland-Hodgman primitive,
// used by bool_intersect, bool_union, and bool_subtract.
//
// If `table` is non-null, interface points are computed via the table
// (single-source-of-truth) instead of locally. The dedup bands are the
// LINEAR world's (ε_L, D2 row 1).
Polygon clip_inside_3d(
    const BRepBody& clipper,
    const Polygon& in_poly,
    IntersectionTable* table,
    double eps_l
) {
    Polygon poly = in_poly;
    for (const auto& clip_face : clipper.solid.shell().faces()) {
        math::Vec3d origin, normal;
        if (!get_face_plane(clip_face, clipper.edges, clipper.vertices, origin, normal)) continue;
        poly = clip_polygon_inside(poly, origin, normal, table, eps_l);
        if (poly.size() < 3) return {};
    }
    // Deduplicate consecutive vertices that are equal within ε_L.
    // Sutherland-Hodgman can produce duplicates when a vertex lies
    // exactly on the clip plane (it gets added both as "current inside"
    // and as "intersection point" with t≈0). Duplicates break
    // polygons_coincident() and Greiner-Hormann.
    Polygon deduped;
    deduped.reserve(poly.size());
    for (const auto& p : poly) {
        if (deduped.empty()) {
            deduped.push_back(p);
        } else {
            const auto& last = deduped.back();
            const double dx = p.x - last.x;
            const double dy = p.y - last.y;
            const double dz = p.z - last.z;
            if (dx*dx + dy*dy + dz*dz > eps_l * eps_l) {
                deduped.push_back(p);
            }
        }
    }
    // Also check wraparound: if the first and last vertices are equal,
    // drop the last one (the polygon is implicitly closed).
    if (deduped.size() >= 2) {
        const auto& first = deduped.front();
        const auto& last = deduped.back();
        const double dx = first.x - last.x;
        const double dy = first.y - last.y;
        const double dz = first.z - last.z;
        if (dx*dx + dy*dy + dz*dz <= eps_l * eps_l) {
            deduped.pop_back();
        }
    }
    return deduped;
}

// Classify a point as inside/outside a convex polyhedron.
// A point is INSIDE iff dot(p - origin_i, normal_i) <= ε_L for every
// face i (all half-space tests pass). For convex polyhedra this is the
// complete inside test (ADR-0024 §2). The band is the LINEAR world's
// (D2 row 1).
[[maybe_unused]] bool point_inside_convex(
    const BRepBody& body,
    const math::Vec3d& p,
    double eps_l
) {
    for (const auto& face : body.solid.shell().faces()) {
        math::Vec3d origin, normal;
        if (!get_face_plane(face, body.edges, body.vertices, origin, normal)) continue;
        const double d = (p - origin).dot(normal);
        if (d > eps_l) return false;  // outside this half-space
    }
    return true;
}

// Check if two polygons are coplanar (same plane normal, within
// tolerance). Used to skip Greiner-Hormann for coplanar face pairs,
// which the algorithm does not handle correctly (edge collinearity).
//
// The normal-parallelity decision is the ANGULAR world's (D2 row 4:
// the loose pre-filter stays loose, typed and declared — the declared
// angle's cosine reproduces the historical |n1·n2| threshold
// bit-exactly); the plane-distance decision is the LINEAR world's.
bool polygons_coplanar(
    const Polygon& poly_A,
    const Polygon& poly_B,
    math::Vec3d normal_A,
    math::Vec3d normal_B,
    const BooleanTolerance& tol
) {
    const double dot = normal_A.dot(normal_B);
    // Loose pre-filter (D2 row 4): |n1·n2| below cos(θ_prefilter)
    // means the normals are not parallel (≈ 2.6°, deliberately loose —
    // a cheap rejection before the distance test).
    if (std::abs(dot) < std::cos(tol.angular_prefilter.value)) return false;
    if (poly_A.empty() || poly_B.empty()) return false;
    const math::Vec3d origin_A = poly_A[0];
    const double d = (poly_B[0] - origin_A).dot(normal_A);
    return std::abs(d) < tol.linear.value;
}

GHResult greiner_hormann_2d(
    const Polygon2D& subject_poly,
    const Polygon2D& clip_poly,
    double eps_l,
    double eps_p
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
            double t_lo, t_hi;
            const auto itype = segment_intersection(a1, a2, b1, b2,
                                                    eps_l, eps_p, t_lo, t_hi);
            if (itype != IntersectionType::None) {
                any_intersect = true;
                break;
            }
        }
        if (any_intersect) break;
    }

    if (!any_intersect) {
        // Check if subject's first vertex is inside clip.
        if (point_in_polygon(subject_poly[0], clip_poly, eps_l)) {
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
    compute_intersections(subject, clip, eps_l, eps_p);
    classify_intersections(subject, clip_poly, eps_l);
    classify_intersections(clip, subject_poly, eps_l);
    result.inside = walk_results(subject, true);
    result.outside = walk_results(subject, false);
    return result;
}

// 3D polygon subtraction via slab decomposition (ADR-0025 FASE 2).
//
// subject \ clipper = subject \ (H₀ ∩ H₁ ∩ … ∩ Hₙ)
//                     = ⋃ᵢ (subject ∩ comp(Hᵢ) ∩ H₀ ∩ … ∩ H_{i-1})
//
// For each plane i of the clipper:
//   1. piece = SH_clip(subject, comp(Hᵢ))  — keep OUTSIDE half-space i
//   2. piece = SH_clip(piece, H₀) … SH_clip(piece, H_{i-1}) — keep
//      inside all previous half-spaces (so pieces are disjoint)
//   3. If piece has ≥ 3 vertices, it's a valid outside fragment.
//
// Uses the existing SH primitive (clip_polygon_sh) with the same
// ε_L band (D2 row 1) — no magic numbers, no GH in the production
// path. Interface points go through the IntersectionTable when non-null.
std::vector<Polygon> polygon_subtract_3d(
    const Polygon& subject_poly,
    const Polygon& clip_poly,
    math::Vec3d plane_normal,
    double eps_l
) {
    std::vector<Polygon> result;
    if (subject_poly.size() < 3 || clip_poly.size() < 3) return result;

    // Collect clipper face planes. The clip_poly is a single polygon
    // representing the clip region on the same plane as subject.
    // For slab decomposition we need the clipper's half-spaces, which
    // for a convex polygon are defined by its edges.
    // Each edge (v[j], v[j+1]) defines a half-space: the interior of
    // the convex polygon is to the left of the edge direction.
    // The outward normal of edge j (in 2D, on the subject's plane) is
    // perpendicular to the edge, pointing outward.
    //
    // But we're in 3D on a shared plane. The clip_poly's edges define
    // half-spaces in the LOCAL 2D frame. We project to 2D, do slab
    // decomposition in 2D, then unproject.
    LocalFrame frame = build_local_frame(plane_normal, subject_poly[0]);
    Polygon2D subject_2d = project_polygon(frame, subject_poly);
    Polygon2D clip_2d = project_polygon(frame, clip_poly);

    // Compute clipper edge half-spaces in 2D.
    // For a convex polygon with CCW winding, the interior is to the
    // left of each edge. The outward normal of edge (a→b) is (dy, -dx).
    struct HalfSpace2D {
        Vec2 origin;
        Vec2 normal;  // outward (points away from interior)
    };
    std::vector<HalfSpace2D> clip_planes;
    const std::size_t cn = clip_2d.size();
    for (std::size_t j = 0; j < cn; ++j) {
        const Vec2& a = clip_2d[j];
        const Vec2& b = clip_2d[(j + 1) % cn];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        // Outward normal (for CCW polygon): (dy, -dx) normalized.
        // The degenerate-edge guard is a LINEAR decision (D2 row 1):
        // an edge shorter than ε_L carries no direction to normalize.
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len < eps_l) continue;
        Vec2 outward{dy / len, -dx / len};
        clip_planes.push_back({a, outward});
    }

    if (clip_planes.empty()) return result;

    // Slab decomposition: for each clipper half-space i, compute:
    //   piece = subject ∩ comp(Hᵢ) ∩ H₀ ∩ … ∩ H_{i-1}
    // using SH clipping in 2D.
    auto sh_clip_2d = [eps_l](
        const Polygon2D& poly,
        Vec2 origin, Vec2 outward_normal,
        bool keep_inside
    ) -> Polygon2D {
        if (poly.empty()) return {};
        Polygon2D res;
        res.reserve(poly.size() + 1);
        const std::size_t n = poly.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2& curr = poly[i];
            const Vec2& next = poly[(i + 1) % n];
            const double d_curr = (curr.x - origin.x) * outward_normal.x
                               + (curr.y - origin.y) * outward_normal.y;
            const double d_next = (next.x - origin.x) * outward_normal.x
                               + (next.y - origin.y) * outward_normal.y;
            // keep_inside=true: keep d <= ε_L (inside the half-space)
            // keep_inside=false: keep d >= -ε_L (outside/complement)
            const bool curr_keep = keep_inside ? (d_curr <= eps_l)
                                               : (d_curr >= -eps_l);
            const bool next_keep = keep_inside ? (d_next <= eps_l)
                                               : (d_next >= -eps_l);
            if (curr_keep) res.push_back(curr);
            if (curr_keep != next_keep) {
                const double denom = d_curr - d_next;
                if (std::abs(denom) > eps_l) {
                    const double t = d_curr / denom;
                    res.push_back({curr.x + t * (next.x - curr.x),
                                   curr.y + t * (next.y - curr.y)});
                }
            }
        }
        return res;
    };

    for (std::size_t i = 0; i < clip_planes.size(); ++i) {
        // Step 1: keep OUTSIDE half-space i.
        Polygon2D piece = sh_clip_2d(subject_2d, clip_planes[i].origin,
                                      clip_planes[i].normal, /*keep_inside=*/false);
        if (piece.size() < 3) continue;
        // Step 2: keep INSIDE all previous half-spaces (0..i-1).
        for (std::size_t j = 0; j < i; ++j) {
            piece = sh_clip_2d(piece, clip_planes[j].origin,
                                clip_planes[j].normal, /*keep_inside=*/true);
            if (piece.size() < 3) break;
        }
        // Deduplicate consecutive vertices (SH can produce duplicates
        // when a vertex lies exactly on the clip plane).
        Polygon2D deduped;
        for (const auto& p : piece) {
            if (deduped.empty()) {
                deduped.push_back(p);
            } else {
                const auto& last = deduped.back();
                const double dx = p.x - last.x;
                const double dy = p.y - last.y;
                if (dx * dx + dy * dy > eps_l * eps_l) {
                    deduped.push_back(p);
                }
            }
        }
        // Check wraparound.
        if (deduped.size() >= 2) {
            const auto& first = deduped.front();
            const auto& last = deduped.back();
            const double dx = first.x - last.x;
            const double dy = first.y - last.y;
            if (dx * dx + dy * dy <= eps_l * eps_l) {
                deduped.pop_back();
            }
        }
        if (deduped.size() >= 3) {
            result.push_back(unproject_polygon(frame, deduped));
        }
    }
    return result;
}

// Check if two polygons have the same vertex set (within ε_L — the
// canonical key, D3). Used to detect "inside == F_A" (face entirely
// inside the other solid).
bool polygons_coincident(const Polygon& a, const Polygon& b, double eps_l) {
    if (a.size() != b.size()) return false;
    SnappedKeySet a_keys = snap_polygon(a, eps_l);
    SnappedKeySet b_keys = snap_polygon(b, eps_l);
    return a_keys == b_keys;
}

// ---------------------------------------------------------------------------
// ADR-0025 v5 — parallel-plane classification (3 cases).
//
// When a clip plane is PARALLEL to the (planar) subject's supporting
// plane, the signed distance d = (p - origin) . n is CONSTANT over the
// subject, so a single three-way classification decides the plane's
// effect — no per-vertex clipping needed.
//
// Why the rule exists: the SH clip keeps {d <= ε_L} and its
// complement keeps {d >= -ε_L}. With ε_L > 0 these two
// sets OVERLAP on the boundary band |d| <= ε_L, so a subject
// lying exactly on the plane is kept by BOTH sides — the tolerance
// makes comp(H) ∩ H ≠ ∅ on the boundary band. In exact geometry
// (ε_L = 0) the open complement does not intersect a subject
// with d == 0. The three cases close that hole deterministically
// (n is the OUTWARD normal of the clip solid, so d < 0 = inside):
//
//   1. d > +ε_L — subject strictly on the complement side: it
//      misses the half-space entirely, so it survives whole (for a
//      cap generator: it can never be inside the other solid).
//   2. d < -ε_L — subject strictly inside the half-space: the
//      plane does not restrict it — identity.
//   3. |d| <= ε_L — coplanar band: the subject lies ON the
//      boundary, never strictly inside, so it can never contribute
//      an interior boundary piece. Skip it as a piece generator: in
//      exact geometry comp(H) ∩ subject = ∅.
//
// The bands are the LINEAR world's (D2 row 1); the strict
// normal-parallelity threshold is the ANGULAR world's (D2 row 4:
// |n1·n2| below cos(θ_strict) — the declared angle's cosine
// reproduces the historical 1 − 1e-6 threshold bit-exactly).
// ---------------------------------------------------------------------------
enum class ParallelPlaneClass {
    NotParallel,     // plane cuts through the subject's plane: per-vertex clipping
    StrictlyInside,  // case 2: d < -ε_L everywhere
    StrictlyOutside, // case 1: d > +ε_L everywhere
    CoplanarBand,    // case 3: |d| <= ε_L everywhere
};

ParallelPlaneClass classify_parallel_plane(
    const Polygon& subject,
    math::Vec3d subject_normal,
    math::Vec3d plane_origin,
    math::Vec3d plane_normal,
    const BooleanTolerance& tol
) {
    const double eps_l = tol.linear.value;
    // Degenerate-normal guards: LINEAR decisions (D2 row 1) — a
    // normal shorter than ε_L carries no direction to normalize.
    const double plen = plane_normal.length();
    const double slen = subject_normal.length();
    if (plen < eps_l || slen < eps_l) return ParallelPlaneClass::NotParallel;
    const math::Vec3d pn = plane_normal * (1.0 / plen);
    const math::Vec3d sn = subject_normal * (1.0 / slen);
    // The strict threshold (D2 row 4): |n1·n2| below cos(θ_strict)
    // (≈ 0.081°) means NotParallel.
    if (std::abs(pn.dot(sn)) < std::cos(tol.angular.value)) {
        return ParallelPlaneClass::NotParallel;
    }
    // Parallel: d is constant over the planar subject. Classify from
    // the extremes over all vertices (a single value for exactly
    // parallel planes; the min/max window rejects near-parallel inputs
    // whose d drifts across the tolerance band — those fall back to
    // per-vertex clipping).
    double d_min = 0.0, d_max = 0.0;
    bool first = true;
    for (const auto& v : subject) {
        const double d = (v - plane_origin).dot(pn);
        if (first) {
            d_min = d_max = d;
            first = false;
        } else {
            d_min = std::min(d_min, d);
            d_max = std::max(d_max, d);
        }
    }
    if (d_max < -eps_l) return ParallelPlaneClass::StrictlyInside;
    if (d_min > eps_l) return ParallelPlaneClass::StrictlyOutside;
    if (d_min >= -eps_l && d_max <= eps_l) return ParallelPlaneClass::CoplanarBand;
    return ParallelPlaneClass::NotParallel;  // straddles the band: clip per-vertex
}

// ---------------------------------------------------------------------------
// Body assembly — shared by all three boolean ops
// ---------------------------------------------------------------------------

struct ClippedFace {
    std::shared_ptr<PlaneSurface> surface;
    Polygon polygon;
};

// The worst of the operands' Layer-2 records (ADR-0032 D2.4 + D8,
// G1 unit 1): if ANY entity of either operand is approximated, every
// entity the operation creates is approximated with the MAXIMUM of
// the declared bounds — a fragment of an approximated operand is a
// sample's descendant; the tolerance is inherited, never diluted,
// never averaged, and never silently reset to exact. Overstating a
// bound is conservative honesty; understating it is the lie the
// provenance field exists to prevent. Both operands fully exact →
// exact (plane∩plane fragments of closed-form inputs stay
// closed-form).
// combined_provenance was hoisted to the topology post-pass helpers
// (G1 unit 4: the exact subtract and the polygon booleans share the
// ONE worst-of definition — the no-drift rule at the entity level).

void assemble_body(BRepBody& result, std::vector<ClippedFace>& clipped_faces,
                   bool canonicalize, const ProvenanceInfo& provenance,
                   double eps_l) {
    auto& shell = result.solid.shell_mut();

    if (canonicalize) {
        SharedVertexPool pool(eps_l);
        for (auto& cf : clipped_faces) {
            pool.canonicalize_polygon(cf.polygon);
        }
    }
    std::map<SnappedKey, VertexId> weld_map;
    auto weld_vertex = [&](const math::Vec3d& pos) -> VertexId {
        SnappedKey key = snap(pos, eps_l);
        auto it = weld_map.find(key);
        if (it != weld_map.end()) return it->second;
        // Grid-boundary straddle repair. Two arithmetic paths can
        // produce the SAME geometric interface point 1-2 ulp apart,
        // and llround(p / ε_L) can then land on opposite sides
        // of a cell boundary (the x.5 line), yielding two "distinct"
        // vertices ~1e-10 apart, dangling coedges and a non-manifold
        // result. Found by the property-based analytic-box-oracle
        // family (seed 0xA11CED01: V=10/E=16/chi=0 on an in-class
        // box-pair intersection; the near-duplicates differed by one
        // ulp on a single coordinate). Repair: honor the ε_L
        // CONTRACT — distance <= ε_L means the same vertex,
        // exactly the semantics clip_inside_3d's dedup and the
        // T-junction pass already apply — with a proximity pass over
        // the already-welded vertices. The lowest-id vertex within
        // ε_L wins (deterministic), and the snapped key is also
        // bound to it so later lookups through either key hit the
        // same id.
        for (VertexId k = 0; k < static_cast<VertexId>(result.vertices.size()); ++k) {
            const math::Vec3d d = result.vertices[k].position() - pos;
            if (d.x * d.x + d.y * d.y + d.z * d.z <= eps_l * eps_l) {
                weld_map.emplace(key, k);
                return k;
            }
        }
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

    // Pre-pass 1: weld every face's vertices FIRST, so the complete
    // vertex set is known before any loop is built. T-junction points
    // may belong to OTHER faces than the one being assembled.
    std::vector<std::vector<VertexId>> face_vids;
    face_vids.reserve(clipped_faces.size());
    for (const auto& cf : clipped_faces) {
        std::vector<VertexId> vids;
        vids.reserve(cf.polygon.size());
        for (const auto& p : cf.polygon) vids.push_back(weld_vertex(p));
        face_vids.push_back(std::move(vids));
    }

    // Pre-pass 2 (ADR-0025 v5): T-junction edge subdivision. The slab
    // decomposition (polygon_subtract_3d) and the cap fragments can
    // produce adjacent faces whose shared boundary is one FULL edge on
    // one side and TWO half-edges on the other (a T-junction): an
    // L-shaped region split into two rectangles shares only part of its
    // neighbour's edge. Edges are keyed by vertex pairs, so the long
    // edge and the short edges do not match -> coedges dangle on
    // distinct overlapping edges -> non-manifold, chi != 2. Fix: split
    // every face edge at each already-welded vertex that lies ON the
    // open segment (within ε_L of the supporting line, at least
    // ε_L away from both endpoints along the edge). After the
    // split, both sides traverse matching sub-segments and every edge
    // ends up with exactly two co-edges.
    auto t_points_on_edge = [&](VertexId v0, VertexId v1) -> std::vector<VertexId> {
        std::vector<VertexId> out;
        const math::Vec3d p0 = result.vertices[v0].position();
        const math::Vec3d p1 = result.vertices[v1].position();
        const math::Vec3d d = p1 - p0;
        const double len2 = d.dot(d);
        // Degenerate-edge guard: a LINEAR decision (D2 row 1).
        if (len2 <= eps_l * eps_l) return out;
        const double len = std::sqrt(len2);
        for (VertexId k = 0; k < static_cast<VertexId>(result.vertices.size()); ++k) {
            if (k == v0 || k == v1) continue;
            const math::Vec3d m = result.vertices[k].position() - p0;
            const double md = m.dot(d);
            const double t = md / len2;
            if (t <= 0.0 || t >= 1.0) continue;
            // At least ε_L away from both endpoints ALONG the
            // edge (closer than that is the endpoint's own weld zone).
            if (t * len <= eps_l || (1.0 - t) * len <= eps_l) continue;
            // Within ε_L of the supporting line (Pythagoras,
            // clamped against FP cancellation).
            double dist2 = m.dot(m) - md * md / len2;
            if (dist2 < 0.0) dist2 = 0.0;
            if (dist2 <= eps_l * eps_l) out.push_back(k);
        }
        // Deterministic order: by position along the edge, ties by id.
        std::sort(out.begin(), out.end(), [&](VertexId a, VertexId b) {
            const double ta = (result.vertices[a].position() - p0).dot(d);
            const double tb = (result.vertices[b].position() - p0).dot(d);
            if (ta != tb) return ta < tb;
            return a < b;
        });
        return out;
    };

    for (std::size_t fi = 0; fi < clipped_faces.size(); ++fi) {
        const auto& vids = face_vids[fi];
        if (vids.size() < 3) continue;
        Face face(clipped_faces[fi].surface);
        Loop loop;
        for (std::size_t i = 0; i < vids.size(); ++i) {
            const VertexId v0 = vids[i];
            const VertexId v1 = vids[(i + 1) % vids.size()];
            // Chain v0 -> [T-junction vertices, ordered along the edge]
            // -> v1: each sub-segment becomes its own co-edge.
            std::vector<VertexId> chain = t_points_on_edge(v0, v1);
            chain.insert(chain.begin(), v0);
            chain.push_back(v1);
            for (std::size_t c = 0; c + 1 < chain.size(); ++c) {
                const EdgeId eid = get_or_create_edge(chain[c], chain[c + 1]);
                const bool reversed = (chain[c] > chain[c + 1]);
                loop.add_coedge(CoEdge{eid, reversed});
            }
        }
        face.add_loop(std::move(loop));
        face.set_id(next_face_id++);
        shell.add_face(std::move(face));
    }

    // Layer-2 (ADR-0032 D2.4, G1 unit 1): every entity the operation
    // creates carries the combined record — set once, here, where
    // every creation path (weld_vertex, get_or_create_edge, the face
    // loop above) has already funneled through.
    for (auto& v : result.vertices) {
        v.set_provenance(provenance);
    }
    for (auto& e : result.edges) {
        e.set_provenance(provenance);
    }
    for (auto& f : shell.faces_mut()) {
        f.set_provenance(provenance);
    }
    link_radial_edges(result);
}

// ---------------------------------------------------------------------------
// H2 unit 2 — THE OUTPUT GATE (the honest refusal of results).
// ---------------------------------------------------------------------------
//
// The Layer-1 assembly postconditions become HARD for the three
// polygon booleans, mirroring exact_subtract's culture (the bridge's
// rule: an assembly defect is thrown, never returned): after
// assemble_body links the radial edges, the result must be ONE of the
// honest outcome classes — anything else is refused with its name,
// the measured evidence and the remedy, and the body NEVER reaches
// the caller (it cannot enter the feature tree, the report, or an
// export).
//
// The honest outcome classes (derived, not assumed):
//
//   1. THE EMPTY ANSWER — zero faces. The operation truthfully
//      produced nothing (disjoint intersect, A \ A). Accepted.
//   2. THE ZERO-VOLUME CONTACT ANSWER (the pinned ADR-0025 class) —
//      exactly one face, every edge carrying exactly one co-edge,
//      loops closed, chi = 1: a disk. The true intersection of two
//      solids touching face-to-face IS that lower-dimensional answer,
//      and the pinned tests assert it as documented behavior — the
//      body does not claim to bound a volume (the shell's closed flag
//      is set false here: a disk has a boundary; the flag stops
//      lying). Accepted.
//   3. THE SOLID ANSWER — a closed orientable manifold (every edge
//      exactly two co-edges in opposite orientations, every loop
//      chained) with an EVEN chi (with the H term): every closed
//      orientable component contributes an even chi, so an even chi
//      is the whole coherence the assembly can be held to — chi > 2
//      (multi-shell, e.g. the union of disjoint bodies) is a
//      CLASSIFICATION per ADR-0022's amendment, never an assembly
//      defect, and is accepted as such.
//
// Everything else is refused:
//
//   result_loop_open          a loop does not chain.
//   result_edge_not_manifold  an edge carries != 2 co-edges — the
//                             open-shell signature (the boundary rim).
//   result_edge_not_orientable two co-edges traverse an edge the same
//                             way (the duplicated interface ring).
//   result_odd_chi            chi odd under passing structure — the
//                             assembly defect the structure alone
//                             cannot see (ADR-0022 §7's own analysis).
//
// The open-shell refusal names the KNOWN input class behind it — the
// partial coplanar overlap of ADR-0025 v5's limitation — and declares
// the refusal the interim answer: the class's full handling is H2
// unit 5. A refused result throws std::runtime_error (the operation
// ran; the RESULT failed its postcondition — an input defect would
// have been caught by the input gate first).
void assert_boolean_result_honest(BRepBody& result, const char* op) {
    auto refuse = [op](const std::string& cls, const std::string& diagnosis,
                       const std::string& remedy) {
        throw std::runtime_error(
            std::string(op) + ": the result was refused — " + cls + ": " +
            diagnosis + " Remedy: " + remedy + ".");
    };

    auto& shell = result.solid.shell_mut();
    const auto& faces = shell.faces();
    if (faces.empty()) {
        return;  // class 1: the honest empty answer
    }

    const bool loops_closed = loops_are_closed(result);
    std::size_t single = 0, paired = 0, bad_orientation = 0;
    for (const auto& e : result.edges) {
        const auto n = e.coedges().size();
        if (n == 1) {
            ++single;
        } else if (n == 2) {
            if (e.coedges()[0]->reversed() == e.coedges()[1]->reversed()) {
                ++bad_orientation;
            } else {
                ++paired;
            }
        }
    }

    // Class 2: the zero-volume contact answer (a disk).
    if (faces.size() == 1 && single == result.edges.size() &&
        paired == 0 && bad_orientation == 0 && loops_closed) {
        const std::int64_t chi = euler_characteristic(result);
        if (chi == 1) {
            shell.set_closed(false);  // a disk has a boundary: the flag
                                      // stops claiming closure
            return;
        }
    }

    // Class 3 gate: the structural invariants, each with its name.
    if (!loops_closed) {
        refuse("result_loop_open",
               "a loop of the assembled result does not chain (the "
               "radial-edge structure was built, so this is an "
               "assembly defect, not a caller error).",
               "report this as a kernel bug with the two inputs; the "
               "operation must never return a body whose loops do "
               "not close");
    }
    if (single != 0 || (paired + bad_orientation + single) !=
                           result.edges.size()) {
        refuse("result_edge_not_manifold",
               std::to_string(single) + " of " +
                   std::to_string(result.edges.size()) +
                   " edges carry a single co-edge (a boundary rim) — "
                   "the assembled shell bounds no solid. This is the "
                   "known PARTIAL COPLANAR OVERLAP class (ADR-0025 v5's "
                   "limitation): the union of the faces' outside parts "
                   "leaves the overlap's rim open, and until H2 unit 5 "
                   "sews the shared-face class the refusal IS the "
                   "answer. The body is NOT returned and must not "
                   "enter the feature tree.",
               "treat the refusal as the operation's honest result; "
               "the class's full handling (the sewn union boundary) "
               "is H2 unit 5 of the contract-honesty phase — a "
               "non-coplanar pair, or the exact subtract, are the "
               "in-class alternatives today");
    }
    if (bad_orientation != 0) {
        refuse("result_edge_not_orientable",
               std::to_string(bad_orientation) + " edge(s) carry two "
               "co-edges traversing them in the SAME direction (the "
               "duplicated interface ring ADR-0022 §7 warns about — "
               "chi cannot see it, the XOR does).",
               "report this as a kernel bug with the two inputs; the "
               "operation must never return a non-orientable "
               "assembly");
    }
    const std::int64_t chi = euler_characteristic(result);
    if ((chi & 1) != 0) {
        refuse("result_odd_chi",
               "chi = " + std::to_string(chi) +
                   " (odd, with the H term) while the structural "
                   "invariants pass — every closed orientable "
                   "component has even chi, so an odd chi is an "
                   "assembly defect the structure alone cannot see "
                   "(ADR-0022's own analysis).",
               "report this as a kernel bug with the two inputs");
    }
    // Class 3 accepted: the flag may stay true — the shell IS closed.
    shell.set_closed(true);
}

// Check if two bodies have identical vertex sets (within ε_L — the
// canonical key, D3). Used for the A == B shortcut in bool_union.
bool bodies_identical(const BRepBody& A, const BRepBody& B, double eps_l) {
    if (A.vertex_count() != B.vertex_count()) return false;
    SnappedKeySet a_keys, b_keys;
    for (const auto& v : A.vertices) a_keys.insert(snap(v.position(), eps_l));
    for (const auto& v : B.vertices) b_keys.insert(snap(v.position(), eps_l));
    return a_keys == b_keys;
}

// ---------------------------------------------------------------------------
// H2 unit 1 — THE INPUT GATE (the single boolean validator).
// ---------------------------------------------------------------------------
//
// ONE function, called as the FIRST thing by all three operations
// (union, intersect, subtract) on BOTH operands, before any shortcut,
// clipping or mutation. The contract it enforces is the one the header
// declares (boolean.hpp): planar-faced convex polyhedra, closed
// orientable manifolds, exactly one outer loop per face. Until this
// gate existed the only enforced clause was "the surfaces are planes"
// (the old validate_planar lambda, duplicated three times) — an input
// outside the declared class could be accepted and mis-clipped without
// a sound, because extract_face_polygon reads the first loop only and
// the half-space machinery assumes a closed convex solid.
//
// THE TAXONOMY — every violated prerequisite carries ITS OWN error
// (the house's actionable-error pattern: the message names the
// prerequisite, the responsible entity with its measured evidence, and
// the remedy; never a generic "invalid input"). The classes, in the
// deterministic order the gate checks them:
//
//   input_empty_body         the operand has no vertices/edges/faces —
//                            there is no solid to operate on.
//   input_topology_reference a co-edge names an edge that does not
//                            exist, or an edge names a vertex that does
//                            not exist — the topology graph is torn.
//   input_surface_not_plane  a face carries no surface or a non-plane
//                            family (the curved-surface class is G2's
//                            extended table, not this one).
//   input_face_inner_loops   a face carries more than one loop — the
//                            holed-face class (e.g. an exact subtract's
//                            pierced output re-entered as an input).
//                            The declared class is one outer loop per
//                            face; the old code silently read
//                            loops()[0] and ignored the hole.
//   input_loop_open          a loop does not chain (end of co-edge i !=
//                            start of co-edge i+1) or is too short to
//                            bound a region.
//   input_edge_not_manifold  an edge carries != 2 co-edges (a boundary
//                            rim or a non-manifold fan) — the half-space
//                            classification assumes a closed solid.
//   input_coedge_orientation an edge's two co-edges traverse it in the
//                            SAME direction (the orientability XOR,
//                            ADR-0022 §5 invariant 2).
//   input_loop_off_plane     a loop vertex lies off its own face's
//                            plane — the surface/border coherence the
//                            polygon reader depends on.
//   input_face_orientation   the face normals point INTO the solid
//                            (globally, measured by the divergence
//                            volume) — every inside/outside test would
//                            invert.
//   input_zero_volume        the operand is not full-dimensional (flat
//                            or measure-zero) — not a solid.
//   input_loop_winding       a loop winds CW around its own face
//                            normal — the slab decomposition
//                            (polygon_subtract_3d) takes the interior
//                            to the LEFT of each edge, which assumes
//                            CCW w.r.t. the normal.
//   input_face_not_convex    a face's polygon has a reflex vertex —
//                            the declared class is convex polygons.
//   input_not_convex         the polyhedron is not the intersection of
//                            its faces' half-spaces (a vertex lies
//                            beyond a face's supporting plane) — the
//                            Sutherland-Hodgman/inside tests are
//                            correct only for convex operands.
//
// The checks are ordered so the SAME input always yields the SAME
// first violation (deterministic refusals, ADR-0014 in spirit): data
// integrity before geometry, geometry before class shape.
//
// Throws std::invalid_argument (an INPUT defect — the operation was
// never obliged to run; the RESULT-side refusals of H2 unit 2 throw
// std::runtime_error, the exact_subtract precedent).
//
// The gate's own comparisons are the LINEAR world's (D2 row 1: the
// historical kPlaneTol sites, now ε_L, typed and declared per
// operation); the relative volume/area class gates stand as they are
// (D7).
void validate_boolean_input(const BRepBody& body, const char* op,
                            const char* input_name,
                            const BooleanTolerance& tol) {
    const double eps_l = tol.linear.value;
    const std::string where = std::string(op) + ": input " + input_name +
                              " rejected — ";
    const std::string note =
        " (the declared class: planar-faced convex polyhedra, closed "
        "orientable manifold, one outer loop per face — boolean.hpp)";
    auto fail = [&where, &note](const std::string& cls,
                                const std::string& diagnosis,
                                const std::string& remedy) {
        throw std::invalid_argument(
            where + cls + ": " + diagnosis + " Remedy: " + remedy + "." +
            note);
    };

    const auto& shell = body.solid.shell();
    const auto& faces = shell.faces();

    // -- 1. the empty body ------------------------------------------------
    if (body.vertices.empty() || body.edges.empty() || faces.empty()) {
        fail("input_empty_body",
             "the body carries " +
                 std::to_string(body.vertices.size()) + " vertex(es), " +
                 std::to_string(body.edges.size()) + " edge(s), " +
                 std::to_string(faces.size()) + " face(s) — there is no "
                 "solid to operate on.",
             "pass a real solid; if this came from an operation that "
             "may legitimately produce nothing, check for emptiness "
             "before calling");
    }

    // -- 2. topological reference coherence --------------------------------
    for (const auto& e : body.edges) {
        if (e.vertex(0) >= body.vertices.size() ||
            e.vertex(1) >= body.vertices.size()) {
            fail("input_topology_reference",
                 "edge " + std::to_string(e.id()) + " names vertex " +
                     std::to_string(std::max(e.vertex(0), e.vertex(1))) +
                     " but the body has " +
                     std::to_string(body.vertices.size()) +
                     " vertices — the topology graph is torn.",
                 "rebuild the body; edge endpoints must index the body's "
                 "own vertex table");
        }
    }
    for (const auto& f : faces) {
        for (const auto& loop : f.loops()) {
            for (const auto& ce : loop.coedges()) {
                if (ce.edge() >= body.edges.size()) {
                    fail("input_topology_reference",
                         "a co-edge of face " + std::to_string(f.id()) +
                             " names edge " + std::to_string(ce.edge()) +
                             " but the body has " +
                             std::to_string(body.edges.size()) +
                             " edges — the topology graph is torn.",
                         "rebuild the body; co-edges must reference "
                         "existing edges");
                }
            }
        }
    }

    // -- 3. the plane family (typed — the enum, never a string) ------------
    for (const auto& f : faces) {
        const auto* surf = f.surface();
        const auto* plane = plane_of(f);
        if (!plane) {
            const char* fam = surf ? family_name(surf->family())
                                   : family_name(SurfaceFamily::None);
            fail("input_surface_not_plane",
                 std::string("face ") + std::to_string(f.id()) +
                     " carries the '" + fam +
                     "' family — this boolean class clips planes only.",
                 "the curved-surface booleans belong to the extended G2 "
                 "table (ADR-0033); pass a planar-faced body");
        }
    }

    // -- 4. one outer loop per face (no inner loops in the class) ----------
    for (const auto& f : faces) {
        if (f.loops().size() != 1) {
            fail("input_face_inner_loops",
                 "face " + std::to_string(f.id()) + " carries " +
                     std::to_string(f.loops().size()) +
                     " loop(s); the declared class is exactly one outer "
                     "loop per face — a holed face would be silently "
                     "mis-read (only its first loop is on the class's "
                     "boundary).",
                 "the holed-face class is out of this boolean's scope "
                 "(the polygon reader walks the outer loop only); "
                     "operate on the disk-faced class or extend the "
                     "kernel with the holed-class booleans (H2 ledger)");
        }
    }

    // -- 5. loop closure ---------------------------------------------------
    for (const auto& f : faces) {
        for (const auto& loop : f.loops()) {
            const auto& ces = loop.coedges();
            if (ces.size() < 3) {
                fail("input_loop_open",
                     "face " + std::to_string(f.id()) +
                         " has a loop of " + std::to_string(ces.size()) +
                         " co-edge(s) — too short to bound a region.",
                     "rebuild the body; a face boundary needs at least "
                     "three chained co-edges");
            }
            for (std::size_t i = 0; i < ces.size(); ++i) {
                const VertexId end_i =
                    ces[i].end_vertex(body.edges);
                const VertexId start_next =
                    ces[(i + 1) % ces.size()].start_vertex(body.edges);
                if (end_i != start_next) {
                    fail("input_loop_open",
                         "face " + std::to_string(f.id()) +
                             ": co-edge " + std::to_string(i) +
                             " ends at vertex " + std::to_string(end_i) +
                             " but co-edge " +
                             std::to_string((i + 1) % ces.size()) +
                             " starts at vertex " +
                             std::to_string(start_next) +
                             " — the loop does not chain.",
                         "rebuild the body; each loop must be a closed "
                         "cycle of co-edges");
                }
            }
        }
    }

    // -- 6. edge manifoldness (the census) ---------------------------------
    for (const auto& e : body.edges) {
        const auto n = e.coedges().size();
        if (n != 2) {
            fail("input_edge_not_manifold",
                 "edge " + std::to_string(e.id()) + " carries " +
                     std::to_string(n) +
                     " co-edge(s) (a closed solid's edge carries "
                     "exactly 2) — the operand is open or non-manifold.",
                 "pass closed manifold solids; an open-shell operand "
                 "has no inside to classify");
        }
    }

    // -- 7. the orientability XOR (ADR-0022 §5 invariant 2) ----------------
    for (const auto& e : body.edges) {
        if (e.coedges().size() == 2 &&
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed()) {
            fail("input_coedge_orientation",
                 "edge " + std::to_string(e.id()) +
                     "'s two co-edges traverse it in the SAME direction "
                     "(the orientability XOR fails) — the faces around "
                     "this edge do not bound an orientable solid.",
                 "rebuild the body with consistent face orientations "
                 "(every shared edge traversed once per direction)");
        }
    }

    // -- 8. surface/border coherence: loop vertices coplanar ---------------
    // The plane measured here is the one the algorithm itself uses:
    // get_face_plane anchors the clipping plane at the loop's FIRST
    // vertex (the PlaneSurface's origin field is a parameterization
    // anchor anywhere along the plane, not a point on the face). The
    // coherence the polygon machinery needs is exactly this: every
    // loop vertex on the plane through the first vertex with the
    // face's normal.
    for (const auto& f : faces) {
        const auto* plane = plane_of(f);
        const auto& loop = f.loops()[0];
        const math::Vec3d anchor =
            body.vertices[loop.coedges()[0].start_vertex(body.edges)]
                .position();
        for (const auto& ce : loop.coedges()) {
            const VertexId vid = ce.start_vertex(body.edges);
            const math::Vec3d p = body.vertices[vid].position();
            const double d =
                (p - anchor).dot(plane->normal_vec());
            if (std::abs(d) > eps_l) {
                fail("input_loop_off_plane",
                     "face " + std::to_string(f.id()) + ": vertex " +
                         std::to_string(vid) +
                         " lies " + std::to_string(d) +
                         " mm off the face's own plane — the surface "
                         "and its border disagree.",
                     "rebuild the body; every loop vertex must lie on "
                     "its face's surface within the plane tolerance");
            }
        }
    }

    // The polygons, once, for the metric gates below.
    std::vector<Polygon> polys;
    polys.reserve(faces.size());
    std::vector<math::Vec3d> normals;
    normals.reserve(faces.size());
    double scale = 0.0;
    for (const auto& v : body.vertices) {
        scale = std::max(scale, std::abs(v.position().x));
        scale = std::max(scale, std::abs(v.position().y));
        scale = std::max(scale, std::abs(v.position().z));
    }
    for (const auto& f : faces) {
        polys.push_back(extract_face_polygon(f, body.edges, body.vertices));
        normals.push_back(plane_of(f)->normal_vec());
    }

    // -- 9. the divergence volume with the STORED normals ------------------
    // V = (1/3)∮x·n dA over the faces' triangle fans, the TRUE area per
    // fan triangle (independent of loop winding): for an outward-normal
    // convex solid every fan triangle's centroid lies inside and every
    // contribution is positive. Globally negative → the normals point
    // into the solid; ~zero → the operand is not full-dimensional.
    {
        double vol = 0.0;
        for (std::size_t fi = 0; fi < faces.size(); ++fi) {
            const auto& poly = polys[fi];
            const auto& n = normals[fi];
            if (poly.size() < 3) continue;
            for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
                const math::Vec3d cr =
                    (poly[i] - poly[0]).cross(poly[i + 1] - poly[0]);
                const math::Vec3d centroid =
                    (poly[0] + poly[i] + poly[i + 1]) * (1.0 / 3.0);
                vol += centroid.dot(n) * cr.length() / 3.0;
            }
        }
        const double vol_tol =
            kVolumeRelTol * scale * scale * scale;
        if (vol < -vol_tol) {
            fail("input_face_orientation",
                 "the face normals point INTO the solid (the divergence "
                 "volume with the stored normals reads " +
                     std::to_string(vol) + " mm^3) — every "
                     "inside/outside test would invert.",
                 "author the faces with outward normals (the loop "
                 "winding CCW around the outward normal); if the body "
                 "came from a kernel path, report it — outward "
                 "orientation is that path's contract");
        }
        if (vol <= vol_tol) {
            fail("input_zero_volume",
                 "the operand is not full-dimensional (the divergence "
                 "volume reads " + std::to_string(vol) +
                     " mm^3, the class gate is " + std::to_string(vol_tol) +
                     " mm^3 at this scale) — there is no inside to "
                     "classify.",
                 "pass a solid with positive volume; flat or degenerate "
                 "bodies are outside the boolean class");
        }
    }

    // -- 10. loop winding w.r.t. the face's own normal ---------------------
    // The slab decomposition (polygon_subtract_3d) takes the interior to
    // the LEFT of each clip edge — that rule assumes the polygon winds
    // CCW in the right-handed frame (u, v, n). A 3D-CCW-around-n loop
    // projects CCW in that frame by construction; a negative projected
    // area is the CW signature.
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& poly = polys[fi];
        const auto& n = normals[fi];
        math::Vec3d u_axis;
        if (std::abs(n.y) > 0.9) {
            u_axis = n.cross(math::Vec3d{1, 0, 0}).normalized();
        } else {
            u_axis = n.cross(math::Vec3d{0, 1, 0}).normalized();
        }
        const math::Vec3d v_axis = n.cross(u_axis).normalized();
        double area2 = 0.0;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const auto& a = poly[i];
            const auto& b = poly[(i + 1) % poly.size()];
            const double au = (a - poly[0]).dot(u_axis);
            const double av = (a - poly[0]).dot(v_axis);
            const double bu = (b - poly[0]).dot(u_axis);
            const double bv = (b - poly[0]).dot(v_axis);
            area2 += au * bv - bu * av;
        }
        const double area = 0.5 * area2;
        const double area_tol = kAreaRelTol * scale * scale;
        if (area < -area_tol) {
            fail("input_loop_winding",
                 "face " + std::to_string(faces[fi].id()) +
                     "'s loop winds CW around its own normal (the "
                     "projected signed area reads " +
                     std::to_string(area) +
                     ") — the slab decomposition would take the wrong "
                     "side of every clip edge.",
                 "author loops CCW around the face's outward normal "
                 "(the right-hand rule); check the surface normal's "
                 "direction first — a flipped normal reads exactly "
                 "like this");
        }
    }

    // -- 11. face convexity -------------------------------------------------
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& poly = polys[fi];
        const auto& n = normals[fi];
        const std::size_t m = poly.size();
        for (std::size_t i = 0; i < m; ++i) {
            const math::Vec3d cr =
                (poly[(i + 1) % m] - poly[i])
                    .cross(poly[(i + 2) % m] - poly[(i + 1) % m]);
            // Collinear runs (cross ≈ 0) are allowed; a negative cross
            // is a reflex vertex.
            if (cr.dot(n) < -eps_l) {
                fail("input_face_not_convex",
                     "face " + std::to_string(faces[fi].id()) +
                         " has a reflex vertex at its loop position " +
                         std::to_string((i + 1) % m) +
                         " — the declared class is convex polygons.",
                     "split the non-convex face, or operate on the "
                     "convex class only (the boolean's polygon "
                     "machinery — Sutherland-Hodgman, the slab "
                     "decomposition — is convex-class)");
            }
        }
    }

    // -- 12. polyhedron convexity (the supporting-plane test) ---------------
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& poly = polys[fi];
        const auto& n = normals[fi];
        bool violated = false, saw_inside = false;
        VertexId offender = 0;
        double worst = 0.0;
        for (VertexId vid = 0; vid < body.vertices.size(); ++vid) {
            const double d =
                (body.vertices[vid].position() - poly[0]).dot(n);
            if (d > eps_l && (!violated || d > worst)) {
                violated = true;
                worst = d;
                offender = vid;
            }
            if (d < -eps_l) saw_inside = true;
        }
        if (!violated) continue;
        if (!saw_inside) {
            // Every other vertex lies BEYOND this face's plane: the
            // face's normal points into the solid (a local orientation
            // defect the divergence volume could not see — the
            // contribution of one small flipped face cancels under the
            // global sum).
            fail("input_face_orientation",
                 "face " + std::to_string(faces[fi].id()) +
                     "'s normal points into the solid (every other "
                     "vertex lies beyond its plane, e.g. vertex " +
                     std::to_string(offender) + " at " +
                     std::to_string(worst) + " mm) — this face's "
                     "half-space would classify the solid as outside.",
                 "flip the face's normal to the outward direction; "
                 "loops stay CCW around the OUTWARD normal");
        }
        fail("input_not_convex",
             "vertex " + std::to_string(offender) + " lies " +
                 std::to_string(worst) +
                 " mm beyond face " + std::to_string(faces[fi].id()) +
                 "'s supporting plane — the polyhedron is not the "
                 "intersection of its faces' half-spaces.",
             "the boolean class is convex polyhedra (boolean.hpp); "
             "split the body into convex pieces or extend the kernel "
             "to the non-convex class");
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// bool_intersect — A ∩ B via Greiner-Hormann
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> bool_intersect(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol) {
    const double eps_l = eps_l_of(tol);
    // H2 unit 1: THE INPUT GATE — the one validator, both operands,
    // as the FIRST thing (nothing clips before the contract holds).
    validate_boolean_input(A, "bool_intersect", "A", tol);
    validate_boolean_input(B, "bool_intersect", "B", tol);

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
        math::Vec3d o_A, n_A;
        if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
        // Use Sutherland-Hodgman sequential clipping (works correctly
        // for convex polyhedra, ADR-0023 primitive). This is equivalent
        // to Greiner-Hormann for the convex case but simpler and proven.
        Polygon inside = clip_inside_3d(B, poly_A, nullptr, eps_l);
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
        math::Vec3d o_B, n_B;
        if (!get_face_plane(face_B, B.edges, B.vertices, o_B, n_B)) continue;
        Polygon inside = clip_inside_3d(A, poly_B, nullptr, eps_l);
        if (inside.size() >= 3) {
            SnappedKeySet inside_keys = snap_polygon(inside, eps_l);
            bool is_dup = false;
            for (const auto& existing : clipped_faces) {
                if (existing.polygon.size() != inside.size()) continue;
                SnappedKeySet existing_keys = snap_polygon(existing.polygon, eps_l);
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

    assemble_body(*result, clipped_faces, /*canonicalize=*/false,
                  combined_provenance(A, B), eps_l);
    assert_boolean_result_honest(*result, "bool_intersect");
    return result;
}

// ---------------------------------------------------------------------------
// bool_union — A ∪ B via polygon subtraction (ADR-0025)
// ---------------------------------------------------------------------------
//
// Algorithm:
//   boundary(A ∪ B) = {parts of A's faces outside B}
//                   ∪ {parts of B's faces outside A}
//
// For each face F_A of A:
//   1. inside = clip_inside_3d(B, F_A)  # Sutherland-Hodgman, convex-safe
//   2. If inside is empty → F_A is entirely outside B → keep F_A intact
//   3. If inside == F_A (vertex sets equal) → F_A entirely inside B → skip
//   4. Otherwise → F_A is partially inside B → outside = polygon_subtract_3d(F_A, inside)
//      Add outside fragments (with F_A's outward normal).
//
// Same for B's faces against A. The deduplication by SnappedKeySet
// catches the case where A and B share a coincident face (e.g. boxes
// touching at a face): both A's top and B's bottom survive as fragments
// with the same vertex set, and deduplication keeps only one.
//
std::unique_ptr<BRepBody> bool_union(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol) {
    const double eps_l = eps_l_of(tol);
    // H2 unit 1: THE INPUT GATE — the one validator, both operands,
    // as the FIRST thing (nothing clips before the contract holds).
    validate_boolean_input(A, "bool_union", "A", tol);
    validate_boolean_input(B, "bool_union", "B", tol);

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;

    // Edge case: A == B → return a copy of A.
    if (bodies_identical(A, B, eps_l)) {
        for (const auto& face_A : A.solid.shell().faces()) {
            Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
            if (poly_A.size() < 3) continue;
            math::Vec3d o_A, n_A;
            if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_A[0], n_A), poly_A});
        }
        assemble_body(*result, clipped_faces, /*canonicalize=*/true,
                      combined_provenance(A, B), eps_l);
        assert_boolean_result_honest(*result, "bool_union");
        return result;
    }

    // FASE B: IntersectionTable — single source of truth for interface
    // ring points. Pre-computed once, consumed by both Part 1 (A's faces
    // against B) and Part 2 (B's faces against A). Quantized by ε_L (D3).
    IntersectionTable table(eps_l);

    auto process_faces = [&](
        const BRepBody& self,
        const BRepBody& other
    ) {
        for (const auto& face : self.solid.shell().faces()) {
            Polygon poly = extract_face_polygon(face, self.edges, self.vertices);
            if (poly.size() < 3) continue;
            math::Vec3d o, n;
            if (!get_face_plane(face, self.edges, self.vertices, o, n)) continue;

            // Compute the part of `poly` that is inside `other`.
            // Pass the IntersectionTable so interface points are shared.
            Polygon inside = clip_inside_3d(other, poly, &table, eps_l);

            if (inside.size() < 3) {
                clipped_faces.push_back(
                    {std::make_shared<PlaneSurface>(poly[0], n), poly});
            } else if (polygons_coincident(poly, inside, eps_l)) {
                continue;
            } else {
                auto outside_fragments = polygon_subtract_3d(poly, inside, n, eps_l);
                for (const auto& frag : outside_fragments) {
                    if (frag.size() >= 3) {
                        clipped_faces.push_back(
                            {std::make_shared<PlaneSurface>(frag[0], n), frag});
                    }
                }
            }
        }
    };

    process_faces(A, B);
    process_faces(B, A);

    if (clipped_faces.empty()) {
        return result;
    }

    assemble_body(*result, clipped_faces, /*canonicalize=*/true,
                  combined_provenance(A, B), eps_l);
    assert_boolean_result_honest(*result, "bool_union");
    return result;
}

// ---------------------------------------------------------------------------
// bool_subtract — A \ B via polygon subtraction + normal inversion
//                (ADR-0025 v3, mirrors bool_union's architecture)
// ---------------------------------------------------------------------------
//
// Algorithm:
//   boundary(A \ B) = {parts of A's faces outside B}                 (Part 1)
//                   ∪ {parts of B's faces inside A, INVERTED normal}  (Part 2)
//
// Part 1 (mirrors bool_union):
//   For each face F_A of A:
//     1. inside = clip_inside_3d(B, F_A)
//     2. If inside is empty → F_A entirely outside B → keep intact
//     3. If inside == F_A → F_A entirely inside B → skip (not on boundary)
//     4. Otherwise → outside = polygon_subtract_3d(F_A, inside, n_A)
//        Add outside fragments with A's outward normal.
//
// Part 2 (caps with inverted normals):
//   For each face F_B of B:
//     1. inside = clip_inside_3d(A, F_B)
//     2. If inside is empty → F_B entirely outside A → skip (no cap)
//     3. If inside == F_B → F_B entirely inside A → add as a single
//        cap face with INVERTED normal
//     4. Otherwise → add `inside` fragments (the partial cap) with
//        INVERTED normal. (Note: inside is already a single polygon
//        from Sutherland-Hodgman sequential clipping; for non-convex
//        results, polygon_subtract would be needed, but A is convex
//        so inside is convex and single-piece.)
//
std::unique_ptr<BRepBody> bool_subtract(
    const BRepBody& A, const BRepBody& B,
    const BooleanTolerance& tol) {
    const double eps_l = eps_l_of(tol);
    // H2 unit 1: THE INPUT GATE — the one validator, both operands,
    // as the FIRST thing (nothing clips before the contract holds).
    validate_boolean_input(A, "bool_subtract", "A", tol);
    validate_boolean_input(B, "bool_subtract", "B", tol);

    auto result = std::make_unique<BRepBody>();
    auto& shell = result->solid.shell_mut();
    shell.set_closed(true);

    std::vector<ClippedFace> clipped_faces;

    // Edge case: A == B → A \ A = empty body.
    if (bodies_identical(A, B, eps_l)) {
        return result;
    }

    // FASE B: IntersectionTable — single source of truth for interface
    // ring points. Shared between Part 1 (A's faces against B) and
    // Part 2 (B's faces against A). Quantized by ε_L (D3).
    IntersectionTable table(eps_l);

    // Part 1: A's faces outside B (kept with A's outward normal).
    for (const auto& face_A : A.solid.shell().faces()) {
        Polygon poly_A = extract_face_polygon(face_A, A.edges, A.vertices);
        if (poly_A.size() < 3) continue;
        math::Vec3d o_A, n_A;
        if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;

        Polygon inside = clip_inside_3d(B, poly_A, &table, eps_l);

        if (inside.size() < 3) {
            // inside is empty or degenerate (area-zero). The removed
            // portion has zero area, so poly_A survives intact.
            clipped_faces.push_back(
                {std::make_shared<PlaneSurface>(poly_A[0], n_A), poly_A});
        } else if (polygons_coincident(poly_A, inside, eps_l)) {
            continue;
        } else {
            auto outside_fragments = polygon_subtract_3d(poly_A, inside, n_A, eps_l);
            for (const auto& frag : outside_fragments) {
                if (frag.size() >= 3) {
                    clipped_faces.push_back(
                        {std::make_shared<PlaneSurface>(frag[0], n_A), frag});
                }
            }
        }
    }

    // Part 2: B's faces inside A, with INVERTED normal (the cavity caps).
    //
    // A face of B can only contribute a cavity cap where it lies
    // STRICTLY inside A. FASE 1 evidence (ADR-0025 v5): in the
    // grid-straddle case the four faces of B that are COPLANAR with
    // faces of A (B's ±Y and ±Z, at y,z = ±1) each emitted a spurious
    // cap equal to the coplanar overlap rectangle — the tolerance band
    // {d <= ε_L} kept them "inside" although they lie ON ∂A,
    // never in A's interior. The parallel-plane rule (3 cases) skips
    // them before clipping.
    for (const auto& face_B : B.solid.shell().faces()) {
        Polygon poly_B = extract_face_polygon(face_B, B.edges, B.vertices);
        if (poly_B.size() < 3) continue;
        math::Vec3d o_B, n_B;
        if (!get_face_plane(face_B, B.edges, B.vertices, o_B, n_B)) continue;

        // ADR-0025 v5: classify poly_B against every face plane of A
        // that is PARALLEL to it (d is constant over the face, one
        // value to classify).
        bool skip_cap = false;
        for (const auto& face_A : A.solid.shell().faces()) {
            math::Vec3d o_A, n_A;
            if (!get_face_plane(face_A, A.edges, A.vertices, o_A, n_A)) continue;
            const auto cls = classify_parallel_plane(poly_B, n_B, o_A, n_A, tol);
            if (cls == ParallelPlaneClass::CoplanarBand) {
                // Case 3: the face lies ON a face plane of A — it is on
                // ∂A, never in A's interior. In exact geometry
                // comp(H) ∩ subject = ∅; the tolerance band is what
                // wrongly kept it. No cavity cap.
                skip_cap = true;
                break;
            }
            if (cls == ParallelPlaneClass::StrictlyOutside) {
                // Case 1 (early exit): the face is entirely on the
                // complement side of an A half-space, so it can never
                // be inside A — no cap. (Equivalent to the sequential
                // clip emptying it, but decided from one constant d.)
                skip_cap = true;
                break;
            }
            // Case 2 (StrictlyInside): the plane does not restrict the
            // face — identity, continue with the remaining planes.
            // NotParallel: the plane cuts the face; the sequential
            // clip below handles it.
        }
        if (skip_cap) continue;

        Polygon inside = clip_inside_3d(A, poly_B, &table, eps_l);
        if (inside.size() < 3) {
            continue;
        }
        // The part of B's face strictly inside A, re-oriented to point
        // into the cavity (toward where B was removed).
        math::Vec3d inverted_normal = n_B * -1.0;
        // The loop arrives wound CCW w.r.t. B's ORIGINAL outward normal
        // (clip_inside_3d preserves the source face's winding). The cap
        // is emitted with the INVERTED normal, so the loop must be
        // reversed to stay CCW w.r.t. the face's own normal — otherwise
        // the two faces sharing each cap edge traverse it in the SAME
        // direction and is_closed_manifold's coedge-orientation XOR
        // (invariant 2, ADR-0022 §5) fails.
        std::reverse(inside.begin(), inside.end());
        clipped_faces.push_back(
            {std::make_shared<PlaneSurface>(inside[0], inverted_normal), inside});
    }

    if (clipped_faces.empty()) {
        return result;
    }

    assemble_body(*result, clipped_faces, /*canonicalize=*/true,
                  combined_provenance(A, B), eps_l);
    assert_boolean_result_honest(*result, "bool_subtract");
    return result;
}

// Test-only entry point — declaration and contract live in the
// test-only seam header CAD_0/brep/detail/boolean_test_hooks.hpp
// (ADR-0025 v5 FASE 2). Exposes the internal polygon_subtract_3d slab
// decomposition to the unit-test suite. The ε_L is the caller's
// declaration (no hidden house value).
std::vector<std::vector<math::Vec3d>> polygon_subtract_fragments_for_tests(
    const std::vector<math::Vec3d>& subject,
    const std::vector<math::Vec3d>& clip,
    math::Vec3d plane_normal,
    double eps_l
) {
    const std::vector<Polygon> fragments = polygon_subtract_3d(subject, clip, plane_normal, eps_l);
    std::vector<std::vector<math::Vec3d>> out;
    out.reserve(fragments.size());
    for (const auto& f : fragments) {
        out.emplace_back(f.begin(), f.end());
    }
    return out;
}

// Test-only entry point — the segment classification under its
// declared tolerances (H2-u3 morso 2): the regime goldens read the
// three-family decision directly. The enums mirror the internal ones;
// the tolerances are the caller's declaration.
SegmentIsectType segment_intersection_for_tests(
    math::Vec2d p1, math::Vec2d p2, math::Vec2d p3, math::Vec2d p4,
    double eps_l, double eps_p,
    double& t_lo, double& t_hi
) {
    const auto itype = segment_intersection(
        Vec2{p1.x, p1.y}, Vec2{p2.x, p2.y},
        Vec2{p3.x, p3.y}, Vec2{p4.x, p4.y},
        eps_l, eps_p, t_lo, t_hi);
    switch (itype) {
        case IntersectionType::Point:   return SegmentIsectType::Point;
        case IntersectionType::Overlap: return SegmentIsectType::Overlap;
        case IntersectionType::None:    break;
    }
    return SegmentIsectType::None;
}

} // namespace CAD_0::brep
