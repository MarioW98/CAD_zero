// core/brep/src/ssi.cpp
//
// The analytic SSI table's implementation — ADR-0033 D1.1/D5.1 (G2
// unit 1). One derivation per entry, in the order the ADR's table
// pins them; every disposition the tests pin is computed here and
// nowhere else. Pure functions throughout (ADR-0014): the same
// operands answer the same bytes.
//
// H1 (ADR-0034) — THE ARITHMETIC AND THE DISPATCH, as completed by
//   H2 unit 3 morso 3 (ADR-0035 D4):
//   * every derivation computes in DOUBLE (the exact path's domain)
//     and the p-curve payloads are STORED in double — the BREP
//     parametric domain's own vocabulary (pcurve.hpp, one opinion
//     per world). The double→float demotions that lived at the
//     authoring sites since ADR-0034 D1's boundary row are REMOVED,
//     not converted; the sketch world is entered only at the
//     extrude caps' frontier (float→double, exact, one direction);
//   * the dispatch is TYPED (D3): the operand records are a
//     std::variant of typed payloads built through the surface
//     VISITOR, and the entry selection is a std::visit overload set
//     — the landed pairs are overloads on the REC TYPES, the wall is
//     the generic fallback. No string compare, no dynamic_cast chain:
//     a typo'd family name does not compile.
//
// HYGIENE, verified at morso 3 (the order's pending item): this
// file carries ZERO dynamic_cast — the chain it carried before
// c39ad8b (ADR-0034's acceptance) died with the typed dispatch, all
// of it, and no residue came back (grep is the witness; the exact
// path as a whole — ssi, boolean, factories, extrude, subtract —
// is dynamic_cast-free). The only dynamic_cast chains left in the
// tree are core/io's SDF serializer (native.cpp — the serialization
// world, outside the exact path's jurisdiction and outside this
// morso's scope) and the tests' own type assertions.
#include "CAD_0/brep/ssi.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

namespace CAD_0::brep {
namespace {

// The periodic domain's constants come from the vocabulary itself
// (pcurve.hpp: kPi/kTwoPi, DOUBLE — the BREP parametric domain's own
// world since morso 3, ADR-0035 D4). No local copy: one declaration
// per constant, the same bytes the factories author.

// --- small helpers --------------------------------------------------------

double norm_two_pi(double a) {
    double r = std::fmod(a, kTwoPi);
    if (r < 0.0) r += kTwoPi;
    return r;
}

std::string num(double v) { return std::to_string(v); }

// The full-period latitude p-curve: ONE Line2 spanning the domain's
// period at v = const (the wall's full-window precedent; the closed
// periodic loop as the domain's horizontal line). DOUBLE END TO END
// (morso 3, ADR-0035 D4): the latitude the derivation computes IS
// the latitude stored — the quantizer that demoted it to the float
// payload died with the float payload itself.
PCurve latitude_pc(double v) {
    return pcurve_segment(math::Vec2d{0.0, v},
                          math::Vec2d{kTwoPi, v},
                          /*interior_left=*/true);
}

// The vertical line p-curve at u = const over the face's authored
// v-window (the boolean will trim within it — the answer spans what
// the face itself spans). Payload domain: double, the BREP domain's
// own world.
PCurve vline_pc(double u, double v0, double v1) {
    return pcurve_segment(math::Vec2d{u, v0},
                          math::Vec2d{u, v1},
                          /*interior_left=*/true);
}

// --- the operand records (typed payloads, ADR-0034 D3) ----------------------

struct PlaneRec {
    math::Vec3d origin{}, normal{}, u_axis{}, v_axis{};
};

struct SphereRec {
    math::Vec3d center{}, pole{}, azimuth{};
    double radius{0.0};
};

enum class CylKind : std::uint8_t { AxisY, AxisZ };

struct CylRec {
    CylKind kind{CylKind::AxisY};
    math::Vec3d axis_point{}, axis_dir{}, azimuth0{};
    double radius{0.0};
    double u_sign{1.0};  // AxisZ only: u = s·azimuth

    // The u of a unit world direction (⊥ the axis) — the inverse of
    // the surface's own parametrization, per placement.
    double u_of(const math::Vec3d& d) const {
        double u = 0.0;
        if (kind == CylKind::AxisZ) {
            const double phi = std::atan2(d.y, d.x);
            u = u_sign > 0.0 ? phi : -phi;
        } else {
            u = std::atan2(-d.z, d.x);
        }
        return norm_two_pi(u);
    }

    // The height coordinate of a world point (the domain's v).
    double v_of(const math::Vec3d& p) const {
        return (p - axis_point).dot(axis_dir);
    }
};

// The families the table refuses without payload mathematics — the
// family NAME is all the record needs (the registered refusals).
struct ConeRec {};
struct TorusRec {};
struct NoSurfaceRec {};
struct UnknownFamilyRec {};

// The typed payload of one operand's surface.
using SurfaceRec = std::variant<std::monostate, PlaneRec, SphereRec,
                                CylRec, ConeRec, TorusRec, NoSurfaceRec,
                                UnknownFamilyRec>;

// The operand record: the family (the enum — one naming authority)
// with its typed payload, plus the provenance fields the gates read.
struct SurfRecord {
    SurfaceFamily family{SurfaceFamily::Unknown};
    bool approximated{false};
    double declared_tol{0.0};
    const Face* face{nullptr};
    SurfaceRec rec{std::monostate{}};
};

// THE PAYLOAD READER — the surface visitor (no string compare, no
// dynamic_cast chain; the typed overloads are the whole extraction).
struct SsiSurfaceReader : SurfaceVisitor {
    SurfRecord r;

    void from(const SurfaceGeometry& s) {
        r.family = s.family();
        s.accept(*this);
    }

    void visit(const PlaneSurface& s) override {
        PlaneRec p;
        p.origin = s.origin();
        p.normal = s.normal_vec().normalized();
        p.u_axis = s.u_axis();
        p.v_axis = s.v_axis();
        r.rec = p;
    }

    void visit(const SphereSurface& s) override {
        SphereRec sp;
        sp.center = s.origin();
        sp.radius = s.radius();
        sp.pole = s.pole().normalized();
        sp.azimuth = s.azimuth().normalized();
        r.rec = sp;
    }

    void visit(const CylinderZSurface& s) override {
        CylRec c;
        c.kind = CylKind::AxisZ;
        c.axis_point = {s.axis_center().x, s.axis_center().y, s.z_base()};
        c.axis_dir = {0.0, 0.0, 1.0};
        c.radius = s.radius();
        c.u_sign = s.u_sign() > 0.0 ? 1.0 : -1.0;
        fill_azimuth0(c, s);
        r.rec = c;
    }

    void visit(const CylinderSurface& s) override {
        CylRec c;
        c.kind = CylKind::AxisY;
        c.axis_point = s.origin();
        c.axis_dir = {0.0, 1.0, 0.0};
        c.radius = s.radius();
        c.u_sign = 1.0;
        fill_azimuth0(c, s);
        r.rec = c;
    }

    void visit(const ConeSurface&) override { r.rec = ConeRec{}; }
    void visit(const TorusSurface&) override { r.rec = TorusRec{}; }

protected:
    void unhandled() override {
        // A subclass the vocabulary does not know: the honest wall.
        r.rec = UnknownFamilyRec{};
    }

private:
    // The u = 0 meridian direction, from the surface's own
    // parametrization (robust to any future placement): the unit
    // direction from the axis to evaluate(0, 0).
    void fill_azimuth0(CylRec& c, const SurfaceGeometry& s) {
        if (c.radius > 0.0) {
            c.azimuth0 = (s.evaluate(0.0, 0.0) - c.axis_point).normalized();
        }
    }
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

SurfRecord read_face(const BRepBody& body, FaceId id, const char* who) {
    const Face& f = face_by_id(body, id, who);
    SurfRecord r;
    r.face = &f;
    r.approximated = !f.provenance().is_exact();
    r.declared_tol = f.provenance().linear_tolerance;

    const SurfaceGeometry* s = f.surface();
    if (s == nullptr) {
        r.family = SurfaceFamily::None;
        r.rec = NoSurfaceRec{};
        return r;
    }
    // The typed payload comes from the VISITOR; the face pointer and
    // the Layer-2 fields stay on the SAME record (the merge the
    // entries' gates and windows read).
    SsiSurfaceReader reader;
    reader.from(*s);
    reader.r.face = r.face;
    reader.r.approximated = r.approximated;
    reader.r.declared_tol = r.declared_tol;
    return reader.r;
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
bool pole_serves(const SphereRec& s, const math::Vec3d& axis, double tol) {
    return std::abs(s.pole.dot(axis.normalized())) >= 1.0 - tol;
}

// The face's authored v-window (the line p-curves' span), from its
// own segment p-curves. False when not derivable — the caller refuses
// loudly (never a guessed span). PAYLOAD DOMAIN: the window is the
// face's own authored double bytes, read verbatim (morso 3: the
// window reads the same world the payloads store).
bool face_v_window(const Face& f, double& v0, double& v1) {
    bool any = false;
    for (const auto& loop : f.loops())
        for (const auto& ce : loop.coedges()) {
            const PCurve* p = ce.p_curve();
            if (p == nullptr || p->kind != PCurve::Kind::Segment) continue;
            for (const math::Vec2d pt : {p->segment.a, p->segment.b}) {
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
double latitude_v(double k, double R) {
    return std::acos(std::clamp(k / R, -1.0, 1.0));
}

// The budget refusal's words, shared by the entries (one voice).
SsiResult refuse_budget(SsiResult& out, const SurfRecord& sp_rec,
                        const char* entry) {
    return wall(
        out,
        std::string("ssi_query: ") + entry +
            " refused — the frame budget: the latitude is typed only in "
            "a frame whose pole is the operation's canonical axis, but "
            "this sphere face carries typed cuts beyond the seam (" +
            std::to_string(distinct_loop_edges(*sp_rec.face)) +
            " distinct edges): one frame = one axis family per face "
            "(ADR-0033 D1.1), and the stored family's cuts would go "
            "untyped under the requested axis — out-of-table, the "
            "accepted boundary (option (i)). Remedy: reorient the "
            "operands (cut ⊥ the budgeted axis), re-frame before the "
            "cuts land, accept the faceted path, or await the boolean "
            "unit's cut-aware re-frame (G2 unit 2).");
}

// THE WALL'S MESSAGE SELECTOR — the refusal is DERIVED from the typed
// family enums (H1, ADR-0034 D3): one fallback, the named owners
// selected by the enum, in the order the original ladder read them
// (cone first, torus, then the landed pairs' owners are the explicit
// overloads above, then the surfaceless, then the generic wall).
SsiResult refusal_for(const SurfRecord& ra, const SurfRecord& rb,
                      SsiResult& out) {
    const bool has_cone = ra.family == SurfaceFamily::Cone ||
                          rb.family == SurfaceFamily::Cone;
    const bool has_torus = ra.family == SurfaceFamily::Torus ||
                           rb.family == SurfaceFamily::Torus;
    const bool has_plane = ra.family == SurfaceFamily::Plane ||
                           rb.family == SurfaceFamily::Plane;
    const bool has_cyl = ra.family == SurfaceFamily::Cylinder ||
                         rb.family == SurfaceFamily::Cylinder;
    const bool has_none = ra.family == SurfaceFamily::None ||
                          rb.family == SurfaceFamily::None;
    const std::string pair = family_name(ra.family) +
                             std::string("×") + family_name(rb.family);

    if (has_cone) {
        return wall(out,
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
        return wall(out,
            "ssi_query: " + pair + " — the toroidal entries (ADR-0033 "
            "D1.1 rows 9–10: plane×torus ⊥ axis, cylinder×torus coaxial) "
            "are registered for the fillet's unit (D3/D5.3), where the "
            "rolling-ball construction consumes them by construction "
            "(the tangency circles are built, then tabled) — declared "
            "by the register, not discovered by the user. Remedy: await "
            "the fillet unit; no authored body carries a torus face "
            "before it.");
    }
    if (has_plane && has_cyl) {
        return wall(out,
            "ssi_query: plane×cylinder — landed in the EXACT SUBTRACT's "
            "own SSI entry (G1 unit 4, ADR-0032 D5: the ⊥ circle with "
            "its Line2 on the cylinder domain, the ∥ line pair with the "
            "tangent and empty cases, the oblique refused naming the "
            "angle — the sinusoid lesson the table's criterion "
            "generalizes). The table's general dispatch for this pair "
            "arrives with the booleans (G2 unit 2). Remedy: consult the "
            "subtract, or await unit 2.");
    }
    if (has_plane) {
        return wall(out,
            "ssi_query: plane×plane — landed in the POLYGON booleans "
            "(Phase D, ADR-0024/0025: the convex-polyhedra class, the "
            "slab decomposition's own intersection); the SSI table does "
            "not duplicate a landed opinion. Remedy: consult the "
            "boolean operations (bool_union / bool_subtract / "
            "bool_intersect).");
    }
    if (has_none) {
        return wall(out,
            "ssi_query: " + pair + " — a face without a surface "
            "has no geometry to intersect. Remedy: report the body "
            "(its authoring path produced a surfaceless face).");
    }
    return wall(out,
        "ssi_query: " + pair + " is outside the analytic table's "
        "dispatch (the table knows plane, sphere and the two cylinder "
        "placements; the symmetric images are the same entries). "
        "Remedy: report the body — a new family would be a kernel "
        "extension, not a silent skip.");
}

// The lambda-visitor idiom: the overload set for std::visit below.
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// --- the four entries, one derivation each (ADR-0033 D1.1) ----------------

// ENTRY 4: plane × sphere (d < R): the circle √(R²−d²) at the foot of
// the perpendicular; Circle2 on the plane's own domain; the latitude
// Line2 on the sphere's domain re-framed to the plane's normal (or
// typed in the stored frame when the pole already serves).
SsiResult entry_plane_sphere(const PlaneRec& pl, const SphereRec& sp,
                             const SurfRecord& sp_rec, SsiResult& out,
                             double tol, SsiDomainAnswer& plane_dom,
                             SsiDomainAnswer& sph_dom) {
    const math::Vec3d n = pl.normal;
    const double R = sp.radius;
    const double d = (sp.center - pl.origin).dot(n);

    if (std::abs(d) > R + tol) {
        out.kind = SsiResult::Kind::Empty;
        out.message = "separate (|d| = " + num(std::abs(d)) +
                      " > R = " + num(R) + ": the plane misses the "
                      "sphere)";
        return out;
    }
    if (std::abs(std::abs(d) - R) <= tol) {
        out.kind = SsiResult::Kind::Point;
        out.point = sp.center - n * (d >= 0.0 ? R : -R);
        out.message = "the tangent point (|d| = R: the plane touches "
                      "the sphere at the foot of the perpendicular)";
        return out;
    }

    out.kind = SsiResult::Kind::Circle;
    // The derivation: the foot of the perpendicular is the circle's
    // center; the radius the classical √(R²−d²); the frame is the
    // plane's own (u, v).
    const math::Vec3d foot = sp.center - n * d;
    const double rr = std::sqrt(std::max(0.0, R * R - d * d));
    SsiCurve c;
    c.kind = SsiCurve::Kind::Circle;
    c.center = foot;
    c.radius = rr;
    c.u_ref = pl.u_axis;
    c.v_ref = pl.v_axis;
    out.curves.push_back(c);

    // The plane-domain p-curve: the Circle2 at the foot's domain
    // coordinates (the same numbers the sketcher would read).
    // DOUBLE END TO END (morso 3): the projection in double, the
    // payload in the domain's own double — the demotion died.
    const math::Vec3d rel = foot - pl.origin;
    const math::Vec2d cu{rel.dot(pl.u_axis), rel.dot(pl.v_axis)};
    plane_dom.pcurves.push_back(
        pcurve_circle(cu, rr, /*hole=*/false));

    // The sphere-domain p-curve: the latitude. The stored frame
    // serves when its pole is parallel to the plane's normal; else
    // the entry DERIVES the re-frame (pole = the normal, azimuth =
    // the plane's u axis) — budget-gated.
    if (pole_serves(sp, n, tol)) {
        const double k = (pl.origin - sp.center).dot(sp.pole);
        sph_dom.pcurves.push_back(latitude_pc(latitude_v(k, R)));
    } else {
        if (sp_rec.face != nullptr && face_is_budgeted(*sp_rec.face)) {
            return refuse_budget(
                out, sp_rec, "plane×sphere");
        }
        sph_dom.reframe_needed = true;
        sph_dom.frame_pole = n;
        sph_dom.frame_azimuth = pl.u_axis;
        sph_dom.pcurves.push_back(latitude_pc(latitude_v(-d, R)));
    }
    return out;
}

// ENTRY 5: sphere × sphere (non-nested): the circle in the RADICAL
// PLANE (subtract the equations: 2(c₂−c₁)·x = ‖c₂‖²−‖c₁‖²+R₁²−R₂² —
// a plane ⊥ the center line, then entry 4's mathematics); each sphere
// re-framed to the center line, azimuth = the derived radical plane's
// own u axis (the PlaneSurface rule — one frame derivation for every
// plane in the house).
SsiResult entry_sphere_sphere(const SphereRec& s1, const SphereRec& s2,
                              const SurfRecord& ra, const SurfRecord& rb,
                              SsiResult& out, double tol) {
    const double R1 = s1.radius, R2 = s2.radius;
    const math::Vec3d w = s2.center - s1.center;
    const double D = w.length();

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
        const math::Vec3d u = w * (1.0 / D);
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
    const math::Vec3d u = w * (1.0 / D);
    const double h = (D * D + R1 * R1 - R2 * R2) / (2.0 * D);
    const double rho = std::sqrt(std::max(0.0, R1 * R1 - h * h));
    const math::Vec3d cc = s1.center + u * h;

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
    const SurfRecord* srecs[2] = {&ra, &rb};
    SsiDomainAnswer* doms[2] = {&out.a, &out.b};
    for (int i = 0; i < 2; ++i) {
        const SphereRec& s = *recs[i];
        const SurfRecord& srec = *srecs[i];
        SsiDomainAnswer& dom = *doms[i];
        if (pole_serves(s, u, tol)) {
            const double k = (cc - s.center).dot(s.pole);
            dom.pcurves.push_back(latitude_pc(latitude_v(k, s.radius)));
        } else {
            if (srec.face != nullptr && face_is_budgeted(*srec.face)) {
                return refuse_budget(
                out, srec, "sphere×sphere");
            }
            dom.reframe_needed = true;
            dom.frame_pole = u;
            dom.frame_azimuth = radical.u_axis();
            const double k = (cc - s.center).dot(u);
            dom.pcurves.push_back(
                latitude_pc(latitude_v(k, s.radius)));
        }
    }
    return out;
}

// ENTRY 6: cylinder × sphere coaxial (r < R): the TWO circles at
// axial offsets ±√(R²−r²), the cylinder's own radius; Line2 v=const
// on the cylinder domain, latitudes on the sphere re-framed to the
// axis (or typed in the stored frame when the pole already serves —
// the no-reframe path).
SsiResult entry_cyl_sphere(const CylRec& cy, const SphereRec& sp,
                           const SurfRecord& sp_rec, SsiResult& out,
                           double tol, SsiDomainAnswer& cyl_dom,
                           SsiDomainAnswer& sph_dom) {
    const math::Vec3d a = cy.axis_dir;  // unit
    const double r = cy.radius;
    const double R = sp.radius;
    const math::Vec3d e0 = cy.azimuth0;
    const math::Vec3d e1 = a.cross(e0);  // the circle frame's v

    // The coaxiality gate: the sphere's center on the axis (the
    // linear band reads the declared tolerance).
    const math::Vec3d rel = sp.center - cy.axis_point;
    const math::Vec3d perp = rel - a * rel.dot(a);
    if (perp.length() > tol) {
        return wall(
            out,
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
        c.center = sp.center;
        c.radius = r;
        c.u_ref = e0;
        c.v_ref = e1;
        out.curves.push_back(c);
        out.message = "the tangent circle (R = r: the sphere touches "
                      "the wall along its equator)";
        cyl_dom.pcurves.push_back(latitude_pc(cy.v_of(sp.center)));
        if (pole_serves(sp, a, tol)) {
            const double k = 0.0;  // the equatorial plane through c
            sph_dom.pcurves.push_back(
                latitude_pc(latitude_v(k, R)));
        } else {
            if (sp_rec.face != nullptr && face_is_budgeted(*sp_rec.face)) {
                return refuse_budget(
                out, sp_rec,
                                     "cylinder×sphere");
            }
            sph_dom.reframe_needed = true;
            sph_dom.frame_pole = a;
            sph_dom.frame_azimuth = e0;
            sph_dom.pcurves.push_back(latitude_pc(latitude_v(0.0, R)));
        }
        return out;
    }

    out.kind = SsiResult::Kind::CirclePair;
    // The derivation: the sphere meets the wall where the axial
    // offset w satisfies r² + w² = R² — the TWO circles at
    // c ± √(R²−r²)·â, radius r (the ADR's own z = ±√(R²−r²) in
    // the canonical frame). Order: +â first.
    const double wv = std::sqrt(std::max(0.0, R * R - r * r));
    const math::Vec3d cps[2] = {sp.center + a * wv,
                                sp.center - a * wv};
    for (int i = 0; i < 2; ++i) {
        SsiCurve c;
        c.kind = SsiCurve::Kind::Circle;
        c.center = cps[i];
        c.radius = r;
        c.u_ref = e0;
        c.v_ref = e1;
        out.curves.push_back(c);
        cyl_dom.pcurves.push_back(latitude_pc(cy.v_of(cps[i])));
    }
    if (pole_serves(sp, a, tol)) {
        for (int i = 0; i < 2; ++i) {
            const double k = (cps[i] - sp.center).dot(sp.pole);
            sph_dom.pcurves.push_back(
                latitude_pc(latitude_v(k, R)));
        }
    } else {
        if (sp_rec.face != nullptr && face_is_budgeted(*sp_rec.face)) {
            return refuse_budget(
                out, sp_rec,
                                 "cylinder×sphere");
        }
        sph_dom.reframe_needed = true;
        sph_dom.frame_pole = a;
        sph_dom.frame_azimuth = e0;
        for (int i = 0; i < 2; ++i) {
            const double k = (cps[i] - sp.center).dot(a);
            sph_dom.pcurves.push_back(
                latitude_pc(latitude_v(k, R)));
        }
    }
    return out;
}

// ENTRY 7: cylinder × cylinder, parallel, EQUAL radii, offset d < 2r:
// the two lines in the mid-plane, at ±√(r²−d²/4) along the
// right-handed vertical δ̂⊥×â (the ADR's y = ±√(r²−d²/4) in its
// canonical frame); Line2 u=const on each domain, spanning the face's
// OWN authored v-window (derived from its loops — the window the
// boolean will trim within).
SsiResult entry_cyl_cyl(const CylRec& c1, const CylRec& c2,
                        const SurfRecord& ra, const SurfRecord& rb,
                        SsiResult& out, double tol) {
    const math::Vec3d a1 = c1.axis_dir;

    // The parallelism gate (the cosine band reads the declared
    // tolerance — the kParallelDotMin discipline).
    if (std::abs(a1.dot(c2.axis_dir)) < 1.0 - tol) {
        return wall(
            out,
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
            out,
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

    const double r = c1.radius;
    // The perpendicular offset between the axes (either axis
    // serves: parallel).
    const math::Vec3d delta = c2.axis_point - c1.axis_point;
    math::Vec3d dperp = delta - a1 * delta.dot(a1);
    const double d = dperp.length();

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
    if (std::abs(d - 2.0 * r) <= tol) {
        out.kind = SsiResult::Kind::TangentLine;
        out.message = "the tangent double line (d = 2r: the walls "
                      "touch along the common ruling)";
        const math::Vec3d dh = dperp * (1.0 / d);
        const math::Vec3d p = c1.axis_point + dh * r;
        SsiCurve c;
        c.kind = SsiCurve::Kind::Line;
        c.point = p;
        c.dir = a1;
        out.curves.push_back(c);
        // The p-curve span: the faces' authored v-windows (the
        // domain's own double bytes, read verbatim — morso 3).
        double w1a = 0, w1b = 0, w2a = 0, w2b = 0;
        if (!face_v_window(*ra.face, w1a, w1b) ||
            !face_v_window(*rb.face, w2a, w2b)) {
            return wall(out,
                        "ssi_query: the line pair's p-curves span the "
                        "faces' authored v-windows, and a cylinder "
                        "face's window is not derivable here (no "
                        "typed segment p-curves on the face) — a "
                        "guessed span would misstate the trim. "
                        "Remedy: report the body (its authoring "
                        "path skipped the window).");
        }
        out.a.pcurves.push_back(
            vline_pc(c1.u_of(dh), w1a, w1b));
        out.b.pcurves.push_back(
            vline_pc(c2.u_of(dh * -1.0), w2a, w2b));
        return out;
    }
    if (d > 2.0 * r + tol) {
        out.kind = SsiResult::Kind::Empty;
        out.message = "separate (d = " + num(d) + " > 2r = " +
                      num(2.0 * r) + ")";
        return out;
    }

    out.kind = SsiResult::Kind::LinePair;
    // The derivation: subtract the equations — the intersection
    // lies in the MID-PLANE between the axes, at ±√(r²−d²/4)
    // along the right-handed vertical δ̂⊥×â (the ADR's y = ±√(r²−
    // d²/4) in its canonical frame). Order: the + vertical first.
    const math::Vec3d dh = dperp * (1.0 / d);
    const math::Vec3d vert = dh.cross(a1);
    const double half = std::sqrt(std::max(0.0, r * r - d * d * 0.25));
    const math::Vec3d mid = c1.axis_point + dh * (d * 0.5);
    const math::Vec3d pts[2] = {mid + vert * half, mid - vert * half};
    for (int i = 0; i < 2; ++i) {
        SsiCurve c;
        c.kind = SsiCurve::Kind::Line;
        c.point = pts[i];
        c.dir = a1;
        out.curves.push_back(c);
    }
    double w1a = 0, w1b = 0, w2a = 0, w2b = 0;
    if (!face_v_window(*ra.face, w1a, w1b) ||
        !face_v_window(*rb.face, w2a, w2b)) {
        return wall(out,
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

} // namespace

// ---------------------------------------------------------------------------
// ssi_query — the typed dispatch and the four entries
// ---------------------------------------------------------------------------
SsiResult ssi_query(const BRepBody& body_a, FaceId face_a,
                    const BRepBody& body_b, FaceId face_b, double tol) {
    const SurfRecord ra = read_face(body_a, face_a, "A");
    const SurfRecord rb = read_face(body_b, face_b, "B");

    SsiResult out;
    out.family_a = family_name(ra.family);
    out.family_b = family_name(rb.family);

    // The provenance gate, first: the exact table consults EXACT
    // faces. An approximated operand is the bridge's faceted world —
    // refused with the remedy, never laundered into exact math.
    if (ra.approximated || rb.approximated) {
        const SurfRecord& bad = ra.approximated ? ra : rb;
        return wall(out,
                    "ssi_query: the exact table consults exact faces — "
                    "this operand's face is approximated (tolerance " +
                        num(bad.declared_tol) +
                        " mm, the bridge's faceted world; the family is " +
                        family_name(bad.family) + "), and answering closed-form "
                        "intersection data on a sampled surface would "
                        "present approximated geometry as exact — the "
                        "lie the Layer-2 provenance exists to prevent. "
                        "Remedy: query authored/factory faces (exact/0), "
                        "or take the faceted path (the bridge's own "
                        "world, ADR-0026) for that operand.");
    }

    // THE TYPED ENTRY DISPATCH (H1, ADR-0034 D3): the landed pairs
    // are overloads on the REC TYPES — the admission criterion made a
    // compile-time structure; the wall is the generic fallback. A
    // typo'd family name does not compile.
    return std::visit(overloaded{
        // ---- ENTRY 4: plane × sphere (either order) -------------------
        [&](const PlaneRec& pl, const SphereRec& sp) {
            return entry_plane_sphere(pl, sp, rb, out, tol,
                                      out.a, out.b);
        },
        [&](const SphereRec& sp, const PlaneRec& pl) {
            return entry_plane_sphere(pl, sp, ra, out, tol,
                                      out.b, out.a);
        },

        // ---- ENTRY 5: sphere × sphere ---------------------------------
        [&](const SphereRec& s1, const SphereRec& s2) {
            return entry_sphere_sphere(s1, s2, ra, rb, out, tol);
        },

        // ---- ENTRY 6: cylinder × sphere coaxial (either order) --------
        [&](const CylRec& cy, const SphereRec& sp) {
            return entry_cyl_sphere(cy, sp, rb, out, tol,
                                    out.a, out.b);
        },
        [&](const SphereRec& sp, const CylRec& cy) {
            return entry_cyl_sphere(cy, sp, ra, out, tol,
                                    out.b, out.a);
        },

        // ---- ENTRY 7: cylinder × cylinder, parallel, EQUAL radii ------
        [&](const CylRec& c1, const CylRec& c2) {
            return entry_cyl_cyl(c1, c2, ra, rb, out, tol);
        },

        // ---- THE WALL: the single generic fallback (the refusal
        // message is DERIVED from the typed family enums inside
        // refusal_for — one fallback, no ambiguity, a typo cannot
        // compile). -----------------------------------------------
        [&](const auto&, const auto&) {
            return refusal_for(ra, rb, out);
        },
    }, ra.rec, rb.rec);
}

// ---------------------------------------------------------------------------
// reframe_sphere_face — the re-frame rule's standalone act
// ---------------------------------------------------------------------------
ReframeRecord reframe_sphere_face(BRepBody& body, FaceId face,
                                  const math::Vec3d& pole,
                                  const math::Vec3d& azimuth, double tol) {
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

    // The class gate: the sphere — the unit's revolution class. The
    // TYPED payload read (H1, ADR-0034 D3): the visitor, no
    // dynamic_cast chain.
    const SphereSurface* sphere = nullptr;
    {
        struct SphereReader : SurfaceVisitor {
            const SphereSurface* found{nullptr};
            void visit(const SphereSurface& s) override { found = &s; }
        } reader;
        f->surface()->accept(reader);
        sphere = reader.found;
    }
    if (sphere == nullptr) {
        const char* fam =
            f->surface() ? family_name(f->surface()->family())
                          : family_name(SurfaceFamily::None);
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
    const SphereSurface next(sphere->radius(), sphere->origin(), pole,
                             azimuth);

    // The seam edge — the single edge the (seam-only) loops carry.
    const EdgeId seam = f->loops()[0].coedges()[0].edge();

    // The rebuilt face: the same mathematical object (C, R), the NEW
    // frame, the seam pair re-wired in the SAME slit-rectangle form,
    // the id / provenance / orientation preserved. The seam payloads
    // are the BREP domain's own double constants (the same bytes the
    // factories author — morso 3: the frontier reversed, the float
    // constants retired with it).
    Face nf(std::make_shared<SphereSurface>(next));
    {
        Loop loop;
        CoEdge seam_fwd{seam, false};  // north → south, the RIGHT edge
        seam_fwd.set_p_curve(pcurve_segment(
            math::Vec2d{kTwoPi, 0.0}, math::Vec2d{kTwoPi, kPi},
            /*interior_left=*/true));
        CoEdge seam_rev{seam, true};  // south → north, the LEFT edge
        seam_rev.set_p_curve(pcurve_segment(
            math::Vec2d{0.0, kPi}, math::Vec2d{0.0, 0.0},
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
    const math::Vec3d north = sphere->origin() + next.pole() * sphere->radius();
    const math::Vec3d south = sphere->origin() - next.pole() * sphere->radius();
    for (auto& v : body.vertices) {
        if (v.id() == se.vertex(0)) v.set_position(north);
        else if (v.id() == se.vertex(1)) v.set_position(south);
    }

    body.solid.shell_mut().faces_mut()[index] = std::move(nf);
    link_radial_edges(body);

    return ReframeRecord{next.pole(), next.azimuth()};
}

} // namespace CAD_0::brep
