// core/brep/src/ssi.cpp
//
// The analytic SSI table's implementation — ADR-0033 D1.1/D5.1 (G2
// unit 1). One derivation per entry, in the order the ADR's table
// pins them; every disposition the tests pin is computed here and
// nowhere else. Pure functions throughout (ADR-0014): the same
// operands answer the same bytes.
#include "CAD_0/brep/ssi.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace CAD_0::brep {
namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717958647692f;

// --- small helpers --------------------------------------------------------

float norm_two_pi(float a) {
    float r = std::fmod(a, kTwoPi);
    if (r < 0.0f) r += kTwoPi;
    return r;
}

std::string num(float v) { return std::to_string(v); }

// The full-period latitude p-curve: ONE Line2 spanning the domain's
// period at v = const (the wall's full-window precedent; the closed
// periodic loop as the domain's horizontal line).
PCurve latitude_pc(float v) {
    return sdf::sketch_segment(math::Vec2f{0.0f, v},
                               math::Vec2f{kTwoPi, v},
                               /*interior_left=*/true);
}

// The vertical line p-curve at u = const over the face's authored
// v-window (the boolean will trim within it — the answer spans what
// the face itself spans).
PCurve vline_pc(float u, float v0, float v1) {
    return sdf::sketch_segment(math::Vec2f{u, v0}, math::Vec2f{u, v1},
                               /*interior_left=*/true);
}

// --- the operand records --------------------------------------------------

struct PlaneRec {
    math::Vec3f origin{}, normal{}, u_axis{}, v_axis{};
};

struct SphereRec {
    math::Vec3f center{}, pole{}, azimuth{};
    float radius{0.0f};
};

enum class CylKind : std::uint8_t { AxisY, AxisZ };

struct CylRec {
    CylKind kind{CylKind::AxisY};
    math::Vec3f axis_point{}, axis_dir{}, azimuth0{};
    float radius{0.0f};
    float u_sign{1.0f};  // AxisZ only: u = s·azimuth

    // The u of a unit world direction (⊥ the axis) — the inverse of
    // the surface's own parametrization, per placement.
    float u_of(const math::Vec3f& d) const {
        float u = 0.0f;
        if (kind == CylKind::AxisZ) {
            const float phi = std::atan2(d.y, d.x);
            u = u_sign > 0.0f ? phi : -phi;
        } else {
            u = std::atan2(-d.z, d.x);
        }
        return norm_two_pi(u);
    }

    // The height coordinate of a world point (the domain's v).
    float v_of(const math::Vec3f& p) const {
        return (p - axis_point).dot(axis_dir);
    }
};

struct SurfRec {
    std::string family{"?"};
    const Face* face{nullptr};
    bool approximated{false};
    double declared_tol{0.0};

    PlaneRec plane{};
    SphereRec sphere{};
    CylRec cyl{};
};

// The face with this id, or the loud caller-bug refusal (the
// query_face culture: diagnosis + remedy).
const Face& face_by_id(const BRepBody& body, FaceId id, const char* who) {
    for (const auto& f : body.solid.shell().faces())
        if (f.id() == id) return f;
    std::ostringstream os;
    os << "ssi_query: no face with id " << id << " in operand " << who
       << " (" << body.face_count() << " face(s) in the shell) — remedy: "
          "ids come from the body's own face numbering (the factories "
          "number by shell order); enumerate with CAD_0.brep.face_count()"
          " / face_query before consulting the table.";
    throw std::invalid_argument(os.str());
}

SurfRec read_face(const BRepBody& body, FaceId id, const char* who) {
    const Face& f = face_by_id(body, id, who);
    SurfRec r;
    r.face = &f;
    r.approximated = !f.provenance().is_exact();
    r.declared_tol = f.provenance().linear_tolerance;

    const SurfaceGeometry* s = f.surface();
    if (s == nullptr) {
        r.family = "(no surface)";
        return r;
    }
    r.family = s->type_name();

    if (r.family == "plane") {
        const auto* pl = dynamic_cast<const PlaneSurface*>(s);
        if (pl) {
            r.plane.origin = pl->origin();
            r.plane.normal = pl->normal_vec().normalized();
            r.plane.u_axis = pl->u_axis();
            r.plane.v_axis = pl->v_axis();
        }
    } else if (r.family == "sphere") {
        const auto* sp = dynamic_cast<const SphereSurface*>(s);
        if (sp) {
            r.sphere.center = sp->origin();
            r.sphere.radius = sp->radius();
            r.sphere.pole = sp->pole().normalized();
            r.sphere.azimuth = sp->azimuth().normalized();
        }
    } else if (r.family == "cylinder") {
        if (const auto* cz = dynamic_cast<const CylinderZSurface*>(s)) {
            r.cyl.kind = CylKind::AxisZ;
            r.cyl.axis_point = {cz->axis_center().x, cz->axis_center().y,
                                cz->z_base()};
            r.cyl.axis_dir = {0.0f, 0.0f, 1.0f};
            r.cyl.radius = cz->radius();
            r.cyl.u_sign = cz->u_sign() > 0.0f ? 1.0f : -1.0f;
        } else if (const auto* cy =
                       dynamic_cast<const CylinderSurface*>(s)) {
            r.cyl.kind = CylKind::AxisY;
            r.cyl.axis_point = cy->origin();
            r.cyl.axis_dir = {0.0f, 1.0f, 0.0f};
            r.cyl.radius = cy->radius();
        }
        // The u = 0 meridian direction, from the surface's own
        // parametrization (robust to any future placement): the
        // unit direction from the axis to evaluate(0, 0).
        if (r.cyl.radius > 0.0f) {
            r.cyl.azimuth0 =
                (s->evaluate(0.0f, 0.0f) - r.cyl.axis_point).normalized();
        }
    }
    // cone / torus / anything else: the family name is all the record
    // needs — the dispatch refuses them with their registered words.
    return r;
}

// The frame budget, DERIVED from the face's own record: a face whose
// loops carry typed cuts beyond the seam structure (distinct edge ids
// beyond the seam's single one) has its frame budgeted. The seam-only
// factory sphere reads 1; a boolean survivor (unit 2's world) and the
// test's authored record read more. The record is the thing under
// test — this walk honors it, it does not launder it.
std::size_t distinct_loop_edges(const Face& f) {
    std::set<EdgeId> ids;
    for (const auto& loop : f.loops())
        for (const auto& ce : loop.coedges()) ids.insert(ce.edge());
    return ids.size();
}

bool face_is_budgeted(const Face& f) { return distinct_loop_edges(f) > 1; }

// Does the sphere's stored pole already type the entry's canonical
// axis (parallel, either sign)? The cosine band reads the declared
// tolerance — the same value every classification in this file reads.
bool pole_serves(const SphereRec& s, const math::Vec3f& axis, float tol) {
    return std::abs(s.pole.dot(axis.normalized())) >= 1.0f - tol;
}

// The face's authored v-window (the line p-curves' span), from its
// own segment p-curves. False when not derivable — the caller refuses
// loudly (never a guessed span).
bool face_v_window(const Face& f, float& v0, float& v1) {
    bool any = false;
    for (const auto& loop : f.loops())
        for (const auto& ce : loop.coedges()) {
            const PCurve* p = ce.p_curve();
            if (p == nullptr || p->kind != PCurve::Kind::Segment) continue;
            for (const math::Vec2f pt : {p->segment.a, p->segment.b}) {
                if (!any) {
                    v0 = v1 = pt.y;
                    any = true;
                } else {
                    v0 = std::min(v0, pt.y);
                    v1 = std::max(v1, pt.y);
                }
            }
        }
    return any;
}

// The out-of-table record builder — the wall speaks with one voice.
SsiResult wall(SsiResult r, std::string message) {
    r.kind = SsiResult::Kind::OutOfTable;
    r.curves.clear();
    r.a.pcurves.clear();
    r.b.pcurves.clear();
    r.message = std::move(message);
    return r;
}

// --- the sphere operand's domain answer -----------------------------------
//
// The latitude of the plane {x : (x − c)·p̂ = k} on the sphere, in the
// frame whose (unit) pole is `pole`: v = arccos(k / R). The same
// closed form serves the stored pole (no re-frame: k measured on it)
// and the entry's canonical axis (the re-framed domain).
float latitude_v(float k, float R) {
    return static_cast<float>(
        std::acos(std::clamp(k / R, -1.0f, 1.0f)));
}

} // namespace

// ---------------------------------------------------------------------------
// ssi_query — the dispatch and the four entries
// ---------------------------------------------------------------------------
SsiResult ssi_query(const BRepBody& body_a, FaceId face_a,
                    const BRepBody& body_b, FaceId face_b, float tol) {
    const SurfRec ra = read_face(body_a, face_a, "A");
    const SurfRec rb = read_face(body_b, face_b, "B");

    SsiResult out;
    out.family_a = ra.family;
    out.family_b = rb.family;

    // The provenance gate, first: the exact table consults EXACT
    // faces. An approximated operand is the bridge's faceted world —
    // refused with the remedy, never laundered into exact math.
    if (ra.approximated || rb.approximated) {
        const SurfRec& bad = ra.approximated ? ra : rb;
        return wall(std::move(out),
                    "ssi_query: the exact table consults exact faces — "
                    "this operand's face is approximated (tolerance " +
                        num(static_cast<float>(bad.declared_tol)) +
                        " mm, the bridge's faceted world; the family is " +
                        bad.family + "), and answering closed-form "
                        "intersection data on a sampled surface would "
                        "present approximated geometry as exact — the "
                        "lie the Layer-2 provenance exists to prevent. "
                        "Remedy: query authored/factory faces (exact/0), "
                        "or take the faceted path (the bridge's own "
                        "world, ADR-0026) for that operand.");
    }

    const bool a_plane = ra.family == "plane";
    const bool b_plane = rb.family == "plane";
    const bool a_sph = ra.family == "sphere";
    const bool b_sph = rb.family == "sphere";
    const bool a_cyl = ra.family == "cylinder";
    const bool b_cyl = rb.family == "cylinder";

    // ---- ENTRY 4: plane × sphere (either order) ---------------------------
    if ((a_plane && b_sph) || (a_sph && b_plane)) {
        const SurfRec& pl = a_plane ? ra : rb;
        const SurfRec& sp = a_plane ? rb : ra;
        SsiDomainAnswer& plane_dom = a_plane ? out.a : out.b;
        SsiDomainAnswer& sph_dom = a_plane ? out.b : out.a;

        const math::Vec3f n = pl.plane.normal;
        const float R = sp.sphere.radius;
        const float d = (sp.sphere.center - pl.plane.origin).dot(n);

        if (std::abs(d) > R + tol) {
            out.kind = SsiResult::Kind::Empty;
            out.message = "separate (|d| = " + num(std::abs(d)) +
                          " > R = " + num(R) + ": the plane misses the "
                          "sphere)";
            return out;
        }
        if (std::abs(std::abs(d) - R) <= tol) {
            out.kind = SsiResult::Kind::Point;
            out.point = sp.sphere.center - n * (d >= 0.0f ? R : -R);
            out.message = "the tangent point (|d| = R: the plane touches "
                          "the sphere at the foot of the perpendicular)";
            return out;
        }

        out.kind = SsiResult::Kind::Circle;
        // The derivation: the foot of the perpendicular is the circle's
        // center; the radius the classical √(R²−d²); the frame is the
        // plane's own (u, v).
        const math::Vec3f foot = sp.sphere.center - n * d;
        const float rr = std::sqrt(std::max(0.0f, R * R - d * d));
        SsiCurve c;
        c.kind = SsiCurve::Kind::Circle;
        c.center = foot;
        c.radius = rr;
        c.u_ref = pl.plane.u_axis;
        c.v_ref = pl.plane.v_axis;
        out.curves.push_back(c);

        // The plane-domain p-curve: the Circle2 at the foot's domain
        // coordinates (the same numbers the sketcher would read).
        const math::Vec3f rel = foot - pl.plane.origin;
        plane_dom.pcurves.push_back(sdf::sketch_circle(
            math::Vec2f{rel.dot(pl.plane.u_axis), rel.dot(pl.plane.v_axis)},
            rr, /*hole=*/false));

        // The sphere-domain p-curve: the latitude. The stored frame
        // serves when its pole is parallel to the plane's normal; else
        // the entry DERIVES the re-frame (pole = the normal, azimuth =
        // the plane's u axis) — budget-gated.
        if (pole_serves(sp.sphere, n, tol)) {
            const float k =
                (pl.plane.origin - sp.sphere.center).dot(sp.sphere.pole);
            sph_dom.pcurves.push_back(latitude_pc(latitude_v(k, R)));
        } else {
            if (sp.face != nullptr && face_is_budgeted(*sp.face)) {
                return wall(
                    std::move(out),
                    "ssi_query: plane×sphere refused — the frame budget: "
                    "the latitude is typed only in a frame whose pole is "
                    "the plane's normal, but this sphere face carries "
                    "typed cuts beyond the seam (" +
                        std::to_string(distinct_loop_edges(*sp.face)) +
                        " distinct edges): one frame = one axis family "
                        "per face (ADR-0033 D1.1), and the stored "
                        "family's cuts would go untyped under the "
                        "requested axis — out-of-table, the accepted "
                        "boundary (option (i)). Remedy: reorient the "
                        "operands (cut ⊥ the budgeted axis), re-frame "
                        "before the cuts land, accept the faceted path, "
                        "or await the boolean unit's cut-aware re-frame "
                        "(G2 unit 2).");
            }
            sph_dom.reframe_needed = true;
            sph_dom.frame_pole = n;
            sph_dom.frame_azimuth = pl.plane.u_axis;
            sph_dom.pcurves.push_back(latitude_pc(latitude_v(-d, R)));
        }
        return out;
    }

    // ---- ENTRY 5: sphere × sphere -----------------------------------------
    if (a_sph && b_sph) {
        const SphereRec& s1 = ra.sphere;
        const SphereRec& s2 = rb.sphere;
        const float R1 = s1.radius, R2 = s2.radius;
        const math::Vec3f w = s2.center - s1.center;
        const float D = w.length();

        if (D <= tol) {
            if (std::abs(R1 - R2) <= tol) {
                out.kind = SsiResult::Kind::Coincident;
                out.message =
                    "the co-radial degenerate: equal spheres, same center "
                    "— the SAME surface, the co-radial dedup rule's "
                    "trigger (the coplanar rule's analog, G1's subtract); "
                    "the consuming operation dedups (G2 unit 2), the "
                    "table reports the degenerate.";
            } else {
                out.kind = SsiResult::Kind::Empty;
                out.message = "concentric, different radii (nested: one "
                              "sphere entirely inside the other)";
            }
            return out;
        }
        if (D > R1 + R2 + tol) {
            out.kind = SsiResult::Kind::Empty;
            out.message = "separate (D = " + num(D) + " > R1+R2 = " +
                          num(R1 + R2) + ")";
            return out;
        }
        const bool ext_tan = std::abs(D - (R1 + R2)) <= tol;
        const bool int_tan =
            std::abs(D - std::abs(R1 - R2)) <= tol && R1 != R2;
        if (ext_tan || int_tan) {
            out.kind = SsiResult::Kind::Point;
            const math::Vec3f u = w * (1.0f / D);
            out.point = s1.center + u * R1;
            out.message = std::string(int_tan ? "internal" : "external") +
                          " tangency (D = " +
                          (int_tan ? "|R1-R2|" : "R1+R2") +
                          ": the spheres touch at one point on the "
                          "center line)";
            return out;
        }
        if (D < std::abs(R1 - R2) - tol) {
            out.kind = SsiResult::Kind::Empty;
            out.message = "nested (D = " + num(D) + " < |R1-R2| = " +
                          num(std::abs(R1 - R2)) +
                          ": one sphere entirely inside the other)";
            return out;
        }

        out.kind = SsiResult::Kind::Circle;
        // The derivation: the RADICAL PLANE — subtract the equations,
        // 2(c₂−c₁)·x = ‖c₂‖²−‖c₁‖²+R₁²−R₂², i.e. the plane ⊥ the
        // center line at h = (D²+R₁²−R₂²)/2D from c₁; the circle's
        // radius ρ = √(R₁²−h²).
        const math::Vec3f u = w * (1.0f / D);
        const float h = (D * D + R1 * R1 - R2 * R2) / (2.0f * D);
        const float rho = std::sqrt(std::max(0.0f, R1 * R1 - h * h));
        const math::Vec3f cc = s1.center + u * h;

        // The derived radical plane's frame — the ONE plane-frame rule
        // in the house (the PlaneSurface constructor's own).
        const PlaneSurface radical(cc, u);

        SsiCurve c;
        c.kind = SsiCurve::Kind::Circle;
        c.center = cc;
        c.radius = rho;
        c.u_ref = radical.u_axis();
        c.v_ref = radical.v_axis();
        out.curves.push_back(c);

        // Each sphere's latitude, re-framed to the center line with
        // the radical plane's own azimuth (one derived frame, shared).
        const SphereRec* recs[2] = {&s1, &s2};
        const SurfRec* srecs[2] = {&ra, &rb};
        SsiDomainAnswer* doms[2] = {&out.a, &out.b};
        for (int i = 0; i < 2; ++i) {
            const SphereRec& s = *recs[i];
            const SurfRec& srec = *srecs[i];
            SsiDomainAnswer& dom = *doms[i];
            if (pole_serves(s, u, tol)) {
                const float k = (cc - s.center).dot(s.pole);
                dom.pcurves.push_back(latitude_pc(latitude_v(k, s.radius)));
            } else {
                if (srec.face != nullptr && face_is_budgeted(*srec.face)) {
                    return wall(
                        std::move(out),
                        "ssi_query: sphere×sphere refused — the frame "
                        "budget: the radical-plane latitude is typed only "
                        "in a frame whose pole is the center line, but "
                        "this sphere face carries typed cuts beyond the "
                        "seam (" +
                            std::to_string(distinct_loop_edges(*srec.face)) +
                            " distinct edges): one frame = one axis "
                            "family per face (ADR-0033 D1.1), and the "
                            "stored family's cuts would go untyped under "
                            "the requested axis — out-of-table, the "
                            "accepted boundary (option (i)). Remedy: "
                            "reorient the operands (cut ⊥ the budgeted "
                            "axis), re-frame before the cuts land, accept "
                            "the faceted path, or await the boolean "
                            "unit's cut-aware re-frame (G2 unit 2).");
                }
                dom.reframe_needed = true;
                dom.frame_pole = u;
                dom.frame_azimuth = radical.u_axis();
                const float k = (cc - s.center).dot(u);
                dom.pcurves.push_back(
                    latitude_pc(latitude_v(k, s.radius)));
            }
        }
        return out;
    }

    // ---- ENTRY 6: cylinder × sphere coaxial (either order) ----------------
    if ((a_cyl && b_sph) || (a_sph && b_cyl)) {
        const SurfRec& cy = a_cyl ? ra : rb;
        const SurfRec& sp = a_cyl ? rb : ra;
        SsiDomainAnswer& cyl_dom = a_cyl ? out.a : out.b;
        SsiDomainAnswer& sph_dom = a_cyl ? out.b : out.a;

        const math::Vec3f a = cy.cyl.axis_dir;  // unit
        const float r = cy.cyl.radius;
        const float R = sp.sphere.radius;
        const math::Vec3f e0 = cy.cyl.azimuth0;
        const math::Vec3f e1 = a.cross(e0);  // the circle frame's v

        // The coaxiality gate: the sphere's center on the axis (the
        // linear band reads the declared tolerance).
        const math::Vec3f rel = sp.sphere.center - cy.cyl.axis_point;
        const math::Vec3f perp = rel - a * rel.dot(a);
        if (perp.length() > tol) {
            return wall(
                std::move(out),
                "ssi_query: cylinder×sphere non-coaxial (the center sits " +
                    num(perp.length()) +
                    " off the cylinder's axis): the intersection is the "
                    "Viviani-class quartic (the sphere and the cylinder "
                    "touching the diameter — the classical closed quartic "
                    "with the figure-eight planar projection), and the "
                    "p-curves are spherical sinusoids in BOTH domains — "
                    "typed in neither, the both-domain criterion's own "
                    "exhibit (ADR-0033 D2). Remedy: reorient the "
                    "operands (coaxial), accept the faceted path (the "
                    "bridge's own world), or await G3's splines.");
        }

        if (R < r - tol) {
            out.kind = SsiResult::Kind::Empty;
            out.message = "contained (R = " + num(R) + " < r = " + num(r) +
                          ": the sphere never reaches the cylindrical "
                          "wall)";
            return out;
        }
        if (std::abs(R - r) <= tol) {
            out.kind = SsiResult::Kind::Circle;
            SsiCurve c;
            c.kind = SsiCurve::Kind::Circle;
            c.center = sp.sphere.center;
            c.radius = r;
            c.u_ref = e0;
            c.v_ref = e1;
            out.curves.push_back(c);
            out.message = "the tangent circle (R = r: the sphere touches "
                          "the wall along its equator)";
            cyl_dom.pcurves.push_back(
                latitude_pc(cy.cyl.v_of(sp.sphere.center)));
            if (pole_serves(sp.sphere, a, tol)) {
                const float k = 0.0f;  // the equatorial plane through c
                sph_dom.pcurves.push_back(
                    latitude_pc(latitude_v(k, R)));
            } else {
                if (sp.face != nullptr && face_is_budgeted(*sp.face)) {
                    return wall(std::move(out),
                                "ssi_query: cylinder×sphere refused — the "
                                "frame budget (the sphere face carries "
                                "typed cuts beyond the seam; one frame = "
                                "one axis family per face, ADR-0033 "
                                "D1.1). Remedy: reorient, re-frame before "
                                "the cuts land, the faceted path, or "
                                "await G2 unit 2's cut-aware re-frame.");
                }
                sph_dom.reframe_needed = true;
                sph_dom.frame_pole = a;
                sph_dom.frame_azimuth = e0;
                sph_dom.pcurves.push_back(latitude_pc(latitude_v(0.0f, R)));
            }
            return out;
        }

        out.kind = SsiResult::Kind::CirclePair;
        // The derivation: the sphere meets the wall where the axial
        // offset w satisfies r² + w² = R² — the TWO circles at
        // c ± √(R²−r²)·â, radius r (the ADR's own z = ±√(R²−r²) in
        // the canonical frame). Order: +â first.
        const float wv = std::sqrt(std::max(0.0f, R * R - r * r));
        const math::Vec3f cps[2] = {sp.sphere.center + a * wv,
                                    sp.sphere.center - a * wv};
        for (int i = 0; i < 2; ++i) {
            SsiCurve c;
            c.kind = SsiCurve::Kind::Circle;
            c.center = cps[i];
            c.radius = r;
            c.u_ref = e0;
            c.v_ref = e1;
            out.curves.push_back(c);
            cyl_dom.pcurves.push_back(latitude_pc(cy.cyl.v_of(cps[i])));
        }
        if (pole_serves(sp.sphere, a, tol)) {
            for (int i = 0; i < 2; ++i) {
                const float k = (cps[i] - sp.sphere.center).dot(sp.sphere.pole);
                sph_dom.pcurves.push_back(
                    latitude_pc(latitude_v(k, R)));
            }
        } else {
            if (sp.face != nullptr && face_is_budgeted(*sp.face)) {
                return wall(std::move(out),
                            "ssi_query: cylinder×sphere refused — the "
                            "frame budget (the sphere face carries typed "
                            "cuts beyond the seam; one frame = one axis "
                            "family per face, ADR-0033 D1.1). Remedy: "
                            "reorient, re-frame before the cuts land, the "
                            "faceted path, or await G2 unit 2's cut-aware "
                            "re-frame.");
            }
            sph_dom.reframe_needed = true;
            sph_dom.frame_pole = a;
            sph_dom.frame_azimuth = e0;
            for (int i = 0; i < 2; ++i) {
                const float k = (cps[i] - sp.sphere.center).dot(a);
                sph_dom.pcurves.push_back(
                    latitude_pc(latitude_v(k, R)));
            }
        }
        return out;
    }

    // ---- ENTRY 7: cylinder × cylinder, parallel, EQUAL radii --------------
    if (a_cyl && b_cyl) {
        const CylRec& c1 = ra.cyl;
        const CylRec& c2 = rb.cyl;
        const math::Vec3f a1 = c1.axis_dir;

        // The parallelism gate (the cosine band reads the declared
        // tolerance — the kParallelDotMin discipline).
        if (std::abs(a1.dot(c2.axis_dir)) < 1.0f - tol) {
            return wall(
                std::move(out),
                "ssi_query: cylinder×cylinder non-parallel: the general "
                "pair intersects in an irreducible quartic space curve "
                "(Bézout on the quadric pair; Levin's pencil method the "
                "classical reduction). The named special case cuts both "
                "ways: the Steinmetz disposition (equal radii, "
                "perpendicular axes) degenerates to two ellipses in "
                "SPACE — but on each cylinder's domain the p-curve is "
                "z = r·sin θ, the sinusoid again, typed in NEITHER "
                "domain. The space-curve degeneration does not rescue "
                "the domain curve: the boundary is the domain (ADR-0033 "
                "D2's own proof of strictness). Remedy: reorient the "
                "operands (parallel axes), accept the faceted path, or "
                "await G3's splines.");
        }

        if (std::abs(c1.radius - c2.radius) > tol) {
            return wall(
                std::move(out),
                "ssi_query: cylinder×cylinder parallel with unequal "
                "radii (r1 = " +
                    num(c1.radius) + ", r2 = " + num(c2.radius) +
                    "): outside the registered entry (equal radii, "
                    "ADR-0033 D1.1 row 7). The acceptance's correction, "
                    "on the record: the transverse case |r1-r2| < d < "
                    "r1+r2 is two lines in the offset plane x = "
                    "(d²+r1²−r2²)/(2d) (u = const on both domains — "
                    "typed, admitted by the both-domain criterion; the "
                    "drafted 'empty or tangency line' note missed the "
                    "interior), awaiting the unit that consumes it; the "
                    "tangency boundary (d = r1+r2, |r1-r2|) and the "
                    "empty exterior stand as drafted. Remedy: equal "
                    "radii, reorient, the faceted path, or await the "
                    "entry's extension with its consuming unit.");
        }

        const float r = c1.radius;
        // The perpendicular offset between the axes (either axis
        // serves: parallel).
        const math::Vec3f delta = c2.axis_point - c1.axis_point;
        math::Vec3f dperp = delta - a1 * delta.dot(a1);
        const float d = dperp.length();

        if (d <= tol) {
            out.kind = SsiResult::Kind::Coincident;
            out.message =
                "the co-radial degenerate: coaxial cylinders of equal "
                "radii — the SAME surface, the co-radial dedup rule's "
                "trigger (the coplanar rule's analog, G1's subtract); "
                "the consuming operation dedups (G2 unit 2), the table "
                "reports the degenerate.";
            return out;
        }
        if (std::abs(d - 2.0f * r) <= tol) {
            out.kind = SsiResult::Kind::TangentLine;
            out.message = "the tangent double line (d = 2r: the walls "
                          "touch along the common ruling)";
            const math::Vec3f dh = dperp * (1.0f / d);
            const math::Vec3f p = c1.axis_point + dh * r;
            SsiCurve c;
            c.kind = SsiCurve::Kind::Line;
            c.point = p;
            c.dir = a1;
            out.curves.push_back(c);
            float w1a = 0, w1b = 0, w2a = 0, w2b = 0;
            if (!face_v_window(*ra.face, w1a, w1b) ||
                !face_v_window(*rb.face, w2a, w2b)) {
                return wall(std::move(out),
                            "ssi_query: the line pair's p-curves span the "
                            "faces' authored v-windows, and a cylinder "
                            "face's window is not derivable here (no "
                            "typed segment p-curves on the face) — a "
                            "guessed span would misstate the trim. "
                            "Remedy: report the body (its authoring "
                            "path skipped the window).");
            }
            out.a.pcurves.push_back(vline_pc(c1.u_of(dh), w1a, w1b));
            out.b.pcurves.push_back(vline_pc(c2.u_of(dh * -1.0f), w2a, w2b));
            return out;
        }
        if (d > 2.0f * r + tol) {
            out.kind = SsiResult::Kind::Empty;
            out.message = "separate (d = " + num(d) + " > 2r = " +
                          num(2.0f * r) + ")";
            return out;
        }

        out.kind = SsiResult::Kind::LinePair;
        // The derivation: subtract the equations — the intersection
        // lies in the MID-PLANE between the axes, at ±√(r²−d²/4)
        // along the right-handed vertical δ̂⊥×â (the ADR's y = ±√(r²−
        // d²/4) in its canonical frame). Order: the + vertical first.
        const math::Vec3f dh = dperp * (1.0f / d);
        const math::Vec3f vert = dh.cross(a1);
        const float half = std::sqrt(std::max(0.0f, r * r - d * d * 0.25f));
        const math::Vec3f mid = c1.axis_point + dh * (d * 0.5f);
        const math::Vec3f pts[2] = {mid + vert * half, mid - vert * half};
        for (int i = 0; i < 2; ++i) {
            SsiCurve c;
            c.kind = SsiCurve::Kind::Line;
            c.point = pts[i];
            c.dir = a1;
            out.curves.push_back(c);
        }
        float w1a = 0, w1b = 0, w2a = 0, w2b = 0;
        if (!face_v_window(*ra.face, w1a, w1b) ||
            !face_v_window(*rb.face, w2a, w2b)) {
            return wall(std::move(out),
                        "ssi_query: the line pair's p-curves span the "
                        "faces' authored v-windows, and a cylinder "
                        "face's window is not derivable here (no typed "
                        "segment p-curves on the face) — a guessed span "
                        "would misstate the trim. Remedy: report the "
                        "body (its authoring path skipped the window).");
        }
        for (int i = 0; i < 2; ++i) {
            out.a.pcurves.push_back(
                vline_pc(c1.u_of((pts[i] - c1.axis_point).normalized()),
                         w1a, w1b));
            out.b.pcurves.push_back(
                vline_pc(c2.u_of((pts[i] - c2.axis_point).normalized()),
                         w2a, w2b));
        }
        return out;
    }

    // ---- The wall: the landed, the registered, the unknown ----------------
    const std::string pair =
        out.family_a + "×" + out.family_b;

    const bool has_cone = ra.family == "cone" || rb.family == "cone";
    const bool has_torus = ra.family == "torus" || rb.family == "torus";
    const bool has_plane = a_plane || b_plane;
    const bool has_cyl = a_cyl || b_cyl;

    if (has_cone) {
        return wall(
            std::move(out),
            "ssi_query: " + pair + " — the cone mathematics is REGISTERED "
            "for the record (ADR-0033 D1.1 row 8: the ⊥ circle at "
            "constant slant, the through-apex ruling pair, the oblique "
            "conics refused — the secant family, the sinusoid's sibling) "
            "and stays unreachable: no writer exists, and none is "
            "designed in G2 (a tapered extrude is a feature decision of "
            "its own). Remedy: none needed — no authored path reaches a "
            "cone face today; the pair is out of the table by scope, "
            "not by secrecy.");
    }
    if (has_torus) {
        return wall(
            std::move(out),
            "ssi_query: " + pair + " — the toroidal entries (ADR-0033 "
            "D1.1 rows 9–10: plane×torus ⊥ axis, cylinder×torus coaxial) "
            "are registered for the fillet's unit (D3/D5.3), where the "
            "rolling-ball construction consumes them by construction "
            "(the tangency circles are built, then tabled) — declared "
            "by the register, not discovered by the user. Remedy: await "
            "the fillet unit; no authored body carries a torus face "
            "before it.");
    }
    if (a_plane && b_plane) {
        return wall(
            std::move(out),
            "ssi_query: plane×plane — landed in the POLYGON booleans "
            "(Phase D, ADR-0024/0025: the convex-polyhedra class, the "
            "slab decomposition's own intersection); the SSI table does "
            "not duplicate a landed opinion. Remedy: consult the "
            "boolean operations (bool_union / bool_subtract / "
            "bool_intersect).");
    }
    if (has_plane && has_cyl) {
        return wall(
            std::move(out),
            "ssi_query: plane×cylinder — landed in the EXACT SUBTRACT's "
            "own SSI entry (G1 unit 4, ADR-0032 D5: the ⊥ circle with "
            "its Line2 on the cylinder domain, the ∥ line pair with the "
            "tangent and empty cases, the oblique refused naming the "
            "angle — the sinusoid lesson the table's criterion "
            "generalizes). The table's general dispatch for this pair "
            "arrives with the booleans (G2 unit 2). Remedy: consult the "
            "subtract, or await unit 2.");
    }
    if (ra.family == "?" || rb.family == "?" ||
        ra.family == "(no surface)" || rb.family == "(no surface)") {
        return wall(std::move(out),
                    "ssi_query: " + pair + " — a face without a surface "
                    "has no geometry to intersect. Remedy: report the "
                    "body (its authoring path produced a surfaceless "
                    "face).");
    }
    return wall(
        std::move(out),
        "ssi_query: " + pair + " is outside the analytic table's "
        "dispatch (the table knows plane, sphere and the two cylinder "
        "placements; the symmetric images are the same entries). "
        "Remedy: report the body — a new family would be a kernel "
        "extension, not a silent skip.");
}

// ---------------------------------------------------------------------------
// reframe_sphere_face — the re-frame rule's standalone act
// ---------------------------------------------------------------------------
ReframeRecord reframe_sphere_face(BRepBody& body, FaceId face,
                                  const math::Vec3f& pole,
                                  const math::Vec3f& azimuth, float tol) {
    (void)tol;
    // The face, or the loud caller bug (the query_face culture).
    Face* f = nullptr;
    std::size_t index = 0;
    for (std::size_t i = 0; i < body.solid.shell().faces().size(); ++i) {
        if (body.solid.shell().faces()[i].id() == face) {
            f = &body.solid.shell_mut().faces_mut()[i];
            index = i;
            break;
        }
    }
    if (f == nullptr) {
        std::ostringstream os;
        os << "reframe_sphere_face: no face with id " << face
           << " in this body (" << body.face_count()
           << " face(s) in the shell) — remedy: ids come from the body's "
              "own face numbering; enumerate with face_query before "
              "re-framing.";
        throw std::invalid_argument(os.str());
    }

    // The class gate: the sphere — the unit's revolution class.
    const auto* sp = dynamic_cast<const SphereSurface*>(f->surface());
    if (sp == nullptr) {
        const char* fam =
            f->surface() ? f->surface()->type_name() : "(no surface)";
        throw std::invalid_argument(
            std::string("reframe_sphere_face: the face is a '") + fam +
            "' — the re-frame table covers the SPHERE (the unit's "
            "revolution class; the cylinder's frame is fixed by its own "
            "axis, a placement is translate_body's job; the cone and "
            "torus have no writer). Remedy: pass a sphere face.");
    }

    // THE FRAME BUDGET: the standalone act serves the SEAM-ONLY face.
    // A face with typed cuts refuses — the incompatible family is the
    // wall, and the compatible one is the boolean unit's own step
    // (unit 2 re-authors the cuts beside the frame); the refusal says
    // so, it does not guess.
    const std::size_t edges = distinct_loop_edges(*f);
    if (edges > 1) {
        throw std::invalid_argument(
            "reframe_sphere_face: the frame budget — the face carries "
            "typed cuts beyond the seam (" +
            std::to_string(edges) +
            " distinct edges), so its frame is budgeted to one axis "
            "family (ADR-0033 D1.1: one frame = one axis family per "
            "face). A standalone re-frame would leave the cuts untyped "
            "under the new axis — out-of-table, the accepted boundary "
            "(option (i)); the family-COMPATIBLE re-frame of a cut face "
            "is the boolean unit's own step (G2 unit 2 re-authors the "
            "cuts beside the frame). Remedy: re-frame before the cuts "
            "land (the seam-only face), or await unit 2's cut-aware "
            "re-frame.");
    }
    if (f->loops().empty() || f->loops()[0].coedges().empty()) {
        throw std::invalid_argument(
            "reframe_sphere_face: the face carries no seam structure to "
            "re-wire — the standalone re-frame serves the factories' "
            "seam-wired sphere (the authored shape, ADR-0023's slit "
            "rectangle); a seamless face is not in the authored class. "
            "Remedy: report the body (its authoring path skipped the "
            "seam wire).");
    }

    // The derived frame (the constructor validates the pair loudly).
    const SphereSurface next(sp->radius(), sp->origin(), pole, azimuth);

    // The seam edge — the single edge the (seam-only) loops carry.
    const EdgeId seam = f->loops()[0].coedges()[0].edge();

    // The rebuilt face: the same mathematical object (C, R), the NEW
    // frame, the seam pair re-wired in the SAME slit-rectangle form,
    // the id / provenance / orientation preserved.
    Face nf(std::make_shared<SphereSurface>(next));
    {
        Loop loop;
        CoEdge seam_fwd{seam, false};  // north → south, the RIGHT edge
        seam_fwd.set_p_curve(sdf::sketch_segment(
            math::Vec2f{kTwoPi, 0.0f}, math::Vec2f{kTwoPi, kPi},
            /*interior_left=*/true));
        CoEdge seam_rev{seam, true};  // south → north, the LEFT edge
        seam_rev.set_p_curve(sdf::sketch_segment(
            math::Vec2f{0.0f, kPi}, math::Vec2f{0.0f, 0.0f},
            /*interior_left=*/true));
        loop.add_coedge(seam_fwd);
        loop.add_coedge(seam_rev);
        nf.add_loop(std::move(loop));
    }
    nf.set_id(f->id());
    nf.set_provenance(f->provenance());
    nf.set_forward(f->forward());

    // The poles move with the frame (the seam is derived state): the
    // seam edge's v0 → the new north, v1 → the new south.
    const Edge& se = body.edges[seam];
    const math::Vec3f north = sp->origin() + next.pole() * sp->radius();
    const math::Vec3f south = sp->origin() - next.pole() * sp->radius();
    for (auto& v : body.vertices) {
        if (v.id() == se.vertex(0)) v.set_position(north);
        else if (v.id() == se.vertex(1)) v.set_position(south);
    }

    body.solid.shell_mut().faces_mut()[index] = std::move(nf);
    link_radial_edges(body);

    return ReframeRecord{next.pole(), next.azimuth()};
}

} // namespace CAD_0::brep
