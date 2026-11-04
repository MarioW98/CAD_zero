// core/brep/src/mass_properties.cpp
//
// The exact mass properties — the machinery (ADR-0032 D4, G1 unit 5).
// The method, the per-family closed forms and the derivations are
// documented in mass_properties.hpp; this file carries the arithmetic.
//
// Determinism (ADR-0014): +, −, ×, ÷ only — the trigonometric
// integrals run through derived reduction recurrences (no tables, no
// sqrt anywhere in this file), the segment kernels are polynomial
// closed forms, the disc and sphere constants are π-multiples. All
// accumulation follows the shell/loop/co-edge orders.
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/pcurve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::brep {

namespace {

// ---------------------------------------------------------------------------
// The trigonometric integral — ∫_a^b cos^i φ sin^j φ dφ by reduction
// ---------------------------------------------------------------------------
//
// The recurrences (integration by parts, each verified against the
// classical half-angle forms by the unit's golden tests):
//
//   T(0,0) = Δφ
//   T(0,j) = [−cosφ·sin^{j−1}φ]_a^b / j + ((j−1)/j)·T(0,j−2)
//   T(i,0) = [ sinφ·cos^{i−1}φ]_a^b / i + ((i−1)/i)·T(i−2,0)
//   T(1,j) = [sin^{j+1}φ]_a^b / (j+1)
//   T(i,j) = ([sin^{j+1}φ·cos^{i−1}φ]_a^b + (i−1)·T(i−2,j+2)) / (j+1)
//
double trig_T(int i, int j, double a, double b) {
    if (i == 0 && j == 0) return b - a;
    const auto pow_of = [](double f, int e) {
        double r = 1.0;
        for (int k = 0; k < e; ++k) r *= f;
        return r;
    };
    const double sa = std::sin(a), ca = std::cos(a);
    const double sb = std::sin(b), cb = std::cos(b);
    if (i == 0) {
        const double boundary = (-cb * pow_of(sb, j - 1)) -
                                (-ca * pow_of(sa, j - 1));
        double rec = 0.0;
        if (j >= 2) rec = ((j - 1.0) / j) * trig_T(0, j - 2, a, b);
        return boundary / j + rec;
    }
    if (j == 0) {
        const double boundary = (sb * pow_of(cb, i - 1)) -
                                (sa * pow_of(ca, i - 1));
        double rec = 0.0;
        if (i >= 2) rec = ((i - 1.0) / i) * trig_T(i - 2, 0, a, b);
        return boundary / i + rec;
    }
    if (i == 1) {
        return (pow_of(sb, j + 1) - pow_of(sa, j + 1)) / (j + 1.0);
    }
    const double boundary = (pow_of(sb, j + 1) * pow_of(cb, i - 1)) -
                            (pow_of(sa, j + 1) * pow_of(ca, i - 1));
    return (boundary + (i - 1.0) * trig_T(i - 2, j + 2, a, b)) / (j + 1.0);
}

// ---------------------------------------------------------------------------
// Forms — polynomials in (v, cosφ, sinφ), the kernels' atoms
// ---------------------------------------------------------------------------
//
// A Term is coef · v^kv · cos^ic φ · sin^js φ. The needed degrees:
// kv ≤ 2 and ic+js ≤ 3 for the cylinder kernels, ic+js ≤ 4 for the
// arc kernels (u³ dv) — the recurrences cover all of them.
struct Term {
    double coef;
    int kv, ic, js;
};

struct Form {
    std::vector<Term> terms;

    void add(double coef, int kv, int ic, int js) {
        if (coef == 0.0) return;
        for (auto& t : terms)
            if (t.kv == kv && t.ic == ic && t.js == js) {
                t.coef += coef;
                return;
            }
        terms.push_back({coef, kv, ic, js});
    }

    void scale(double k) {
        for (auto& t : terms) t.coef *= k;
    }

    // The pure φ integral (the arc kernels: no v dependence).
    double integrate_phi(double a, double b) const {
        double sum = 0.0;
        for (const Term& t : terms)
            sum += t.coef * trig_T(t.ic, t.js, a, b);
        return sum;
    }

    // The rectangle integral (the cylinder kernels): the v-powers and
    // the trig powers separate.
    double integrate_rect(double phi0, double phi1, double v0,
                          double v1) const {
        double sum = 0.0;
        for (const Term& t : terms) {
            double vint;
            switch (t.kv) {
            case 0: vint = v1 - v0; break;
            case 1: vint = (v1 * v1 - v0 * v0) / 2.0; break;
            default: vint = (v1 * v1 * v1 - v0 * v0 * v0) / 3.0; break;
            }
            sum += t.coef * vint * trig_T(t.ic, t.js, phi0, phi1);
        }
        return sum;
    }
};

Form operator*(const Form& a, const Form& b) {
    Form r;
    r.terms.reserve(a.terms.size() * b.terms.size());
    for (const Term& x : a.terms)
        for (const Term& y : b.terms)
            r.add(x.coef * y.coef, x.kv + y.kv, x.ic + y.ic, x.js + y.js);
    return r;
}

// The linear form c0 + cv·v + cc·cosφ + cs·sinφ.
Form linear_form(double c0, double cv, double cc, double cs) {
    Form f;
    f.add(c0, 0, 0, 0);
    f.add(cv, 1, 0, 0);
    f.add(cc, 0, 1, 0);
    f.add(cs, 0, 0, 1);
    return f;
}

// ---------------------------------------------------------------------------
// The 2D region moments — Green's kernels over the loops
// ---------------------------------------------------------------------------
//
// Green's theorem, region on the LEFT of the walk:
//     A   = ∮ u dv            Mu  = ∮ (u²/2) dv
//     Mv  = −∮ (v²/2) du      Quu = ∮ (u³/3) dv
//     Qvv = −∮ (v³/3) du      Quv = ∮ (u²v/2) dv
// A walk with the region on its RIGHT computes the negated moments;
// the caller folds the winding sign σ = sign(A) in, so ANY authored
// direction reads correctly (the caps' mirror/rotation frames, the
// pierced faces' flipped walks: one geometric rule — the bits are
// never consulted, the winding and the positional role carry the
// region semantics).
struct Moments2 {
    double A{0.0}, Mu{0.0}, Mv{0.0};
    double Quu{0.0}, Quv{0.0}, Qvv{0.0};

    void accumulate(const Moments2& o, double sign) {
        A += sign * o.A;
        Mu += sign * o.Mu;
        Mv += sign * o.Mv;
        Quu += sign * o.Quu;
        Quv += sign * o.Quv;
        Qvv += sign * o.Qvv;
    }
};

// One LINE segment (au,av)→(bu,bv): the polynomial kernels, from the
// parametrization u = au + p·t, v = av + q·t, t ∈ [0,1].
void add_segment(Moments2& m, double au, double av, double bu, double bv) {
    const double p = bu - au, q = bv - av;
    m.A += q * (au + p / 2.0);
    m.Mu += (q / 2.0) * (au * au + au * p + p * p / 3.0);
    m.Mv += -(p / 2.0) * (av * av + av * q + q * q / 3.0);
    m.Quu += (q / 3.0) * (au * au * au + 1.5 * au * au * p +
                          au * p * p + p * p * p / 4.0);
    m.Qvv += -(p / 3.0) * (av * av * av + 1.5 * av * av * q +
                           av * q * q + q * q * q / 4.0);
    m.Quv += (q / 2.0) * (au * au * av +
                          (au * au * q + 2.0 * au * p * av) / 2.0 +
                          (2.0 * au * p * q + p * p * av) / 3.0 +
                          p * p * q / 4.0);
}

// One ARC (centre (cu,cv), radius r, CCW from α over sweep s — every
// Arc2 p-curve traverses its CCW authored sweep, the vocabulary's
// only direction): the kernels as φ-forms with dv = r cosφ dφ and
// du = −r sinφ dφ.
void add_arc(Moments2& m, double cu, double cv, double r, double alpha,
             double sweep) {
    const double beta = alpha + sweep;
    const Form u = linear_form(cu, 0.0, r, 0.0);
    const Form v = linear_form(cv, 0.0, 0.0, r);
    const Form dvk = linear_form(0.0, 0.0, r, 0.0);    // r cosφ dφ
    const Form duk = linear_form(0.0, 0.0, 0.0, -r);   // −r sinφ dφ

    m.A += (u * dvk).integrate_phi(alpha, beta);

    Form u2 = u * u;
    u2.scale(0.5);
    m.Mu += (u2 * dvk).integrate_phi(alpha, beta);

    Form v2 = v * v;
    v2.scale(0.5);
    m.Mv += -((v2 * duk).integrate_phi(alpha, beta));

    Form u3 = (u * u) * u;
    u3.scale(1.0 / 3.0);
    m.Quu += (u3 * dvk).integrate_phi(alpha, beta);

    Form v3 = (v * v) * v;
    v3.scale(1.0 / 3.0);
    m.Qvv += -((v3 * duk).integrate_phi(alpha, beta));

    Form u2v = (u * u) * v;
    u2v.scale(0.5);
    m.Quv += (u2v * dvk).integrate_phi(alpha, beta);
}

// The DISC's closed forms (a lone Circle2 loop — the hole rim,
// direction-free): the uniform disc's moments.
Moments2 disc_moments(double cu, double cv, double r) {
    Moments2 m;
    const double A = M_PI * r * r;
    m.A = A;
    m.Mu = A * cu;
    m.Mv = A * cv;
    m.Quu = A * (cu * cu + r * r / 4.0);
    m.Quv = A * cu * cv;
    m.Qvv = A * (cv * cv + r * r / 4.0);
    return m;
}

// ---------------------------------------------------------------------------
// The per-face divergence kernels
// ---------------------------------------------------------------------------
//
// Every face contributes:
//     F0  = ∮ x·n dA
//     Fi  = ∮ x_i² n_i dA              (no sum on i)
//     Fij = ∮ x_i x_j (x·n) dA         (symmetric)
// and the body aggregates V = F0/3, M_i = Fi/2, S_ij = Fij/5.
struct Contrib {
    double F0{0.0};
    double Fi[3]{0.0, 0.0, 0.0};
    double Fij[3][3]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    void accumulate(const Contrib& o) {
        F0 += o.F0;
        for (int i = 0; i < 3; ++i) {
            Fi[i] += o.Fi[i];
            for (int j = 0; j < 3; ++j) Fij[i][j] += o.Fij[i][j];
        }
    }
};

// Expand the planar region moments through the plane's own frame:
// x = origin + u·U + v·V; n constant; x·n = origin·n constant.
Contrib expand_planar(const PlaneSurface& pl, const Moments2& reg) {
    const auto o = pl.origin();
    const auto n = pl.normal_vec();
    const auto U = pl.u_axis();
    const auto V = pl.v_axis();
    const double d = double(o.x) * n.x + double(o.y) * n.y +
                     double(o.z) * n.z;

    Contrib c;
    c.F0 = d * reg.A;

    // ∮x_i² dA and ∮x_i x_j dA over the planar region.
    double ix2[3];
    double ixix[3][3];
    for (int i = 0; i < 3; ++i) {
        const double oi = i == 0 ? o.x : i == 1 ? o.y : o.z;
        const double Ui = i == 0 ? U.x : i == 1 ? U.y : U.z;
        const double Vi = i == 0 ? V.x : i == 1 ? V.y : V.z;
        const double mui = Ui * reg.Mu + Vi * reg.Mv;
        ix2[i] = oi * oi * reg.A + 2.0 * oi * mui +
                 Ui * Ui * reg.Quu + 2.0 * Ui * Vi * reg.Quv +
                 Vi * Vi * reg.Qvv;
        c.Fi[i] = (i == 0 ? n.x : i == 1 ? n.y : n.z) * ix2[i];
    }
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
            const double oi = i == 0 ? o.x : i == 1 ? o.y : o.z;
            const double oj = j == 0 ? o.x : j == 1 ? o.y : o.z;
            const double Ui = i == 0 ? U.x : i == 1 ? U.y : U.z;
            const double Uj = j == 0 ? U.x : j == 1 ? U.y : U.z;
            const double Vi = i == 0 ? V.x : i == 1 ? V.y : V.z;
            const double Vj = j == 0 ? V.x : j == 1 ? V.y : V.z;
            ixix[i][j] = oi * oj * reg.A +
                         oi * (Uj * reg.Mu + Vj * reg.Mv) +
                         oj * (Ui * reg.Mu + Vi * reg.Mv) +
                         Ui * Uj * reg.Quu +
                         (Ui * Vj + Uj * Vi) * reg.Quv +
                         Vi * Vj * reg.Qvv;
            c.Fij[i][j] = c.Fij[j][i] = d * ixix[i][j];
        }
    return c;
}

// The exact planar face: the loops' Green sums with the winding sign
// σ and the positional semantics (first loop = outer, subsequent =
// holes — ADR-0021; outer-minus-inner is the region arithmetic).
//
// THE REGION SIDE (G2-u2 m4, ADR-0038's decoupling rule — the frame
// serves the geometry, the walk serves the topology): the OUTER
// loop's region bits declare which side of the walk the face's
// material lies on. A loop authored per the sketch discipline
// declares region-on-LEFT (interior_left = true / hole = false —
// every factory face, the extrude's caps, the re-authored outer
// loops) and the outward the divergence reads is the frame normal;
// a CARRIED face whose material side flips (the blind hole's
// internal cap — the kernel's first) declares region-on-RIGHT on
// its own native frame, and the outward is the frame normal's
// OPPOSITE: the whole contribution negates. The bits, not the walk
// direction, carry the side — the winding σ still normalizes the
// moments (any authored direction reads its magnitude correctly),
// exactly as before; only the SIGN's master moves, from the frame
// alone to the frame AND the declared side. An outer loop whose
// entities declare BOTH sides has no region at all — refused loud.
Contrib planar_contrib(const PlaneSurface& pl, const Face& f, double tol,
                       std::size_t face_index) {
    Moments2 reg;
    int outer_side = 0;  // +1: region on the LEFT; -1: on the RIGHT
    for (std::size_t li = 0; li < f.loops().size(); ++li) {
        const auto& loop = f.loops()[li];

        // The lone Circle2 loop: the disc's closed forms (the hole
        // rim, direction-free — the vocabulary's honest answer).
        if (loop.coedges().size() == 1) {
            const auto* pc = loop.coedges()[0].p_curve();
            if (pc != nullptr && pc->kind == PCurve::Kind::Circle) {
                if (li == 0)
                    throw std::invalid_argument(
                        "mass_properties: face " + std::to_string(face_index) +
                        " carries a Circle2 as its FIRST loop - a disc face "
                        "whose outer boundary is a single circle has no "
                        "producer in the exact world (the solid disc's caps "
                        "are authored as pi-sweep Arc2 pairs, ADR-0032 D6). "
                        "Remedy: report the body's construction.");
                reg.accumulate(disc_moments(pc->circle.center.x,
                                             pc->circle.center.y,
                                             pc->circle.radius),
                               -1.0);  // the positional hole sign
                continue;
            }
        }

        // The line/arc walk: the as-traversed Green sums...
        Moments2 L;
        for (const auto& ce : loop.coedges()) {
            const auto* pc = ce.p_curve();
            if (pc == nullptr)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(face_index) +
                    " is exact but a co-edge carries no p-curve - the exact "
                    "world's invariant (every exact face carries its "
                    "trimming) is violated. Remedy: report the body's "
                    "construction (a relabeled faceted body is a lied "
                    "record, and the machinery does not launder it).");
            if (pc->kind == PCurve::Kind::Segment) {
                add_segment(L, pc->segment.a.x, pc->segment.a.y,
                            pc->segment.b.x, pc->segment.b.y);
                if (li == 0) {
                    const int side =
                        pc->segment.interior_left ? 1 : -1;
                    if (outer_side == 0) outer_side = side;
                    else if (outer_side != side)
                        throw std::invalid_argument(
                            "mass_properties: face " +
                            std::to_string(face_index) +
                            "'s outer loop declares its region on BOTH "
                            "sides (a segment with interior_left = " +
                            std::to_string(!pc->segment.interior_left) +
                            " beside one with " +
                            std::to_string(outer_side > 0) +
                            ") - a boundary whose entities disagree has "
                            "no region to integrate. Remedy: report the "
                            "body's construction.");
                }
            } else if (pc->kind == PCurve::Kind::Arc) {
                add_arc(L, pc->arc.center.x, pc->arc.center.y,
                        pc->arc.radius, pc->arc.start_angle, pc->arc.sweep);
                if (li == 0) {
                    const int side = pc->arc.hole ? -1 : 1;
                    if (outer_side == 0) outer_side = side;
                    else if (outer_side != side)
                        throw std::invalid_argument(
                            "mass_properties: face " +
                            std::to_string(face_index) +
                            "'s outer loop declares its region on BOTH "
                            "sides (an arc with hole = " +
                            std::to_string(pc->arc.hole) +
                            " beside an entity with region-on-" +
                            std::string(outer_side > 0 ? "left" : "right") +
                            ") - a boundary whose entities disagree has "
                            "no region to integrate. Remedy: report the "
                            "body's construction.");
                }
            } else {
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(face_index) +
                    " loop " + std::to_string(li) + " mixes a Circle2 with "
                    "other co-edges - a closed curve stands alone in its "
                    "loop (no producer in the exact world). Remedy: report "
                    "the body's construction.");
            }
        }
        if (loop.coedges().empty() ||
            std::abs(L.A) <= tol * tol)
            throw std::invalid_argument(
                "mass_properties: face " + std::to_string(face_index) +
                " loop " + std::to_string(li) + " is degenerate (its "
                "enclosed domain area reads " + std::to_string(L.A) +
                ") - the Green kernels need a non-degenerate boundary. "
                "Remedy: report the body's construction.");
        // ...with the winding sign: the enclosed region's moments are
        // σ·L; the outer loop contributes +, the holes −.
        const double sigma = L.A > 0.0 ? 1.0 : -1.0;
        reg.accumulate(L, (li == 0 ? 1.0 : -1.0) * sigma);
    }
    Contrib c = expand_planar(pl, reg);
    if (outer_side < 0) {
        // THE DECOUPLING'S READ (the m4 rule): the outer loop declares
        // the region on the RIGHT of its walk — the material side is
        // the frame normal's OPPOSITE, and the divergence term of the
        // WHOLE face negates (F0, Fi, Fij — one outward, one face).
        c.F0 = -c.F0;
        for (int i = 0; i < 3; ++i) c.Fi[i] = -c.Fi[i];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) c.Fij[i][j] = -c.Fij[i][j];
    }
    return c;
}

// ---------------------------------------------------------------------------
// The toroidal patch family (G2-u3 m3, ADR-0039 D1.2): the (u, v)
// rectangle integrals on T(u, v) = C + (R + r cos v)(cos u·e1 + sin u·e2)
// + r sin v·a-hat, with n = cos v·radial + sin v·a-hat and
// dA = (R + r cos v)·r du dv — the same divergence
// kernels the cylindrical patches already pay, one family over. The
// integrand is a product of powers of trig functions in u AND in v:
// the forms below carry FOUR indices (uc, us, vc, vs) and the
// definite integrals come from ONE exact recurrence (integration by
// parts, descending a by 2; the a = 1 base closed by the sin
// substitution), so every power the products reach is closed —
// never a quadrature, never a table typed by hand.
// ---------------------------------------------------------------------------

// The exact definite integral of cos^a sin^b over [x0, x1], by the
// parts recurrence: I(a, 0) = [cos^(a-1) sin]/a| + ((a-1)/a) I(a-2, 0);
// I(0, b) = -[sin^(b-1) cos]/b| + ((b-1)/b) I(0, b-2); the mixed case
// descends a by 2 while b rises by 2, and the a = 1 base closes by
// the sin substitution: I(1, b) = [sin^(b+1)]/(b+1)|.
double integ_cs(int a, int b, double x0, double x1) {
    const double s0 = std::sin(x0), s1 = std::sin(x1);
    const double c0 = std::cos(x0), c1 = std::cos(x1);
    if (a == 0 && b == 0) return x1 - x0;
    if (b == 0) {
        if (a == 1) return s1 - s0;
        double am1 = 1.0;
        for (int i = 0; i < a - 1; ++i) am1 *= c1;
        double am0 = 1.0;
        for (int i = 0; i < a - 1; ++i) am0 *= c0;
        return (am1 * s1 - am0 * s0) / a +
               ((a - 1.0) / a) * integ_cs(a - 2, 0, x0, x1);
    }
    if (a == 0) {
        if (b == 1) return -c1 + c0;
        double bm1 = 1.0;
        for (int i = 0; i < b - 1; ++i) bm1 *= s1;
        double bm0 = 1.0;
        for (int i = 0; i < b - 1; ++i) bm0 *= s0;
        return -(bm1 * c1 - bm0 * c0) / b +
               ((b - 1.0) / b) * integ_cs(0, b - 2, x0, x1);
    }
    if (a == 1)
        return (std::pow(s1, b + 1) - std::pow(s0, b + 1)) / (b + 1);
    double am1 = 1.0;
    for (int i = 0; i < a - 1; ++i) am1 *= c1;
    double am0 = 1.0;
    for (int i = 0; i < a - 1; ++i) am0 *= c0;
    const double s1b = std::pow(s1, b + 1);
    const double s0b = std::pow(s0, b + 1);
    return (am1 * s1b - am0 * s0b) / (b + 1) +
           ((a - 1.0) / (b + 1)) * integ_cs(a - 2, b + 2, x0, x1);
}

// A term of the two-variable trig forms: coef * cos^uc sin^us (u)
// * cos^vc sin^vs (v).
struct BTerm {
    double coef;
    int uc, us, vc, vs;
};

struct BForm {
    std::vector<BTerm> terms;

    void add(double coef, int uc, int us, int vc, int vs) {
        if (coef == 0.0) return;
        for (auto& t : terms)
            if (t.uc == uc && t.us == us && t.vc == vc && t.vs == vs) {
                t.coef += coef;
                return;
            }
        terms.push_back({coef, uc, us, vc, vs});
    }
};

inline BForm operator*(const BForm& a, const BForm& b) {
    BForm r;
    r.terms.reserve(a.terms.size() * b.terms.size());
    for (const BTerm& x : a.terms)
        for (const BTerm& y : b.terms)
            r.add(x.coef * y.coef, x.uc + y.uc, x.us + y.us,
                  x.vc + y.vc, x.vs + y.vs);
    return r;
}

// The definite rectangle integral: every term separates into its u
// factor and its v factor, each closed by the recurrence above.
double b_integrate(const BForm& f, double u0, double u1,
                   double v0, double v1) {
    double sum = 0.0;
    for (const BTerm& t : f.terms)
        sum += t.coef * integ_cs(t.uc, t.us, u0, u1) *
               integ_cs(t.vc, t.vs, v0, v1);
    return sum;
}

// The torus frame: the tube's center circle C around the axis a-hat,
// the azimuth frame (e1, e2) — the SAME mathematics for both
// placements (the fillet's Z-axis TorusZSurface and the analytic
// family's Y-axis TorusSurface, the mass machinery unifying like it
// already does for the cylinders).
struct TorusFrame {
    double C[3], a[3], e1[3], e2[3];
    double R, r;
};

Contrib torus_contrib(const TorusFrame& fr, double u0, double u1,
                       double v0, double v1) {
    // x_i = C_i + (R + r cos v)(cos u e1_i + sin u e2_i) + r sin v a_i
    BForm x[3], n[3], W;
    for (int i = 0; i < 3; ++i) {
        x[i].add(fr.C[i], 0, 0, 0, 0);
        x[i].add(fr.R * fr.e1[i], 1, 0, 0, 0);
        x[i].add(fr.R * fr.e2[i], 0, 1, 0, 0);
        x[i].add(fr.r * fr.e1[i], 1, 0, 1, 0);
        x[i].add(fr.r * fr.e2[i], 0, 1, 1, 0);
        x[i].add(fr.r * fr.a[i], 0, 0, 0, 1);
        n[i].add(fr.e1[i], 1, 0, 1, 0);
        n[i].add(fr.e2[i], 0, 1, 1, 0);
        n[i].add(fr.a[i], 0, 0, 0, 1);
    }
    W.add(fr.R, 0, 0, 0, 0);
    W.add(fr.r, 0, 0, 1, 0);

    // x·n, as a form (the sum over i of x_i * n_i).
    BForm xn;
    for (int i = 0; i < 3; ++i) {
        const BForm p = x[i] * n[i];
        for (const BTerm& t : p.terms) xn.add(t.coef, t.uc, t.us, t.vc, t.vs);
    }

    Contrib c;
    // F0 = r * Int[ (x·n) W ],  Fi = r * Int[ x_i^2 n_i W ],
    // Fij = r * Int[ x_i x_j (x·n) W ] — the measure
    // (R + r cos v) r du dv is split: the (R + r cos v) rides W, the r
    // rides the prefactor, du dv is the rectangle.
    {
        const BForm g = xn * W;
        c.F0 = fr.r * b_integrate(g, u0, u1, v0, v1);
    }
    for (int i = 0; i < 3; ++i) {
        const BForm xi2 = x[i] * x[i];
        const BForm g = xi2 * n[i] * W;
        c.Fi[i] = fr.r * b_integrate(g, u0, u1, v0, v1);
    }
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
            const BForm xixj = x[i] * x[j];
            const BForm g = xixj * xn * W;
            const double v =
                fr.r * b_integrate(g, u0, u1, v0, v1);
            c.Fij[i][j] = c.Fij[j][i] = v;
        }
    return c;
}

// The unified cylinder frame: x = P0 + v·a + r·radial(φ), with
// radial(φ) = cosφ·e1 + sinφ·e2, n = s·radial, dA = r dφ dv, and the
// parameter relation φ = s·u (du = s·dφ — the one substitution
// factor, folded into the final r·s).
struct CylFrame {
    double P0[3], a[3], e1[3], e2[3];
    double r, s;
};

Contrib cylinder_contrib(const CylFrame& fr, double u0, double u1,
                         double v0, double v1) {
    // The φ-bounds (the substitution φ = s·u; trig_T handles the
    // reversed order's sign).
    const double phi0 = fr.s * u0, phi1 = fr.s * u1;

    // x_i as a linear form in (v, cosφ, sinφ).
    Form xi[3];
    for (int i = 0; i < 3; ++i)
        xi[i] = linear_form(fr.P0[i], fr.a[i], fr.r * fr.e1[i],
                            fr.r * fr.e2[i]);
    // x·n = s·(P0·radial + r).
    Form xn;
    {
        const double pe1 = fr.P0[0] * fr.e1[0] + fr.P0[1] * fr.e1[1] +
                           fr.P0[2] * fr.e1[2];
        const double pe2 = fr.P0[0] * fr.e2[0] + fr.P0[1] * fr.e2[1] +
                           fr.P0[2] * fr.e2[2];
        xn.add(fr.s * pe1, 0, 1, 0);
        xn.add(fr.s * pe2, 0, 0, 1);
        xn.add(fr.s * fr.r, 0, 0, 0);
    }
    // n_i = s·radial_i.
    Form ni[3];
    for (int i = 0; i < 3; ++i) {
        ni[i].add(fr.s * fr.e1[i], 0, 1, 0);
        ni[i].add(fr.s * fr.e2[i], 0, 0, 1);
    }

    Contrib c;
    c.F0 = fr.r * fr.s * xn.integrate_rect(phi0, phi1, v0, v1);
    for (int i = 0; i < 3; ++i)
        c.Fi[i] = fr.r * fr.s *
                  ((xi[i] * xi[i]) * ni[i]).integrate_rect(phi0, phi1, v0, v1);
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
            c.Fij[i][j] = c.Fij[j][i] =
                fr.r * fr.s *
                ((xi[i] * xi[j]) * xn).integrate_rect(phi0, phi1, v0, v1);
        }
    return c;
}

// The full sphere's closed forms: x·n = R, so F0 = 3V, Fi = 2V·C_i,
// Fij = 5V(C_iC_j + δ_ij R²/5).
Contrib sphere_contrib(double cx, double cy, double cz, double R) {
    const double V = (4.0 / 3.0) * M_PI * R * R * R;
    const double C[3] = {cx, cy, cz};
    Contrib c;
    c.F0 = 4.0 * M_PI * R * R * R;
    for (int i = 0; i < 3; ++i) c.Fi[i] = 2.0 * V * C[i];
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j)
            c.Fij[i][j] = c.Fij[j][i] =
                5.0 * V * C[i] * C[j] + (i == j ? V * R * R : 0.0);
    return c;
}

// The polyhedral face (the Mirtich reduction): the vertex polygon
// projected into the face's own plane, the same Green machinery with
// synthesized segments — the honest value of the faceted body.
Contrib polyhedron_contrib(const PlaneSurface& pl, const Face& f,
                           const std::vector<Edge>& edges,
                           const std::vector<Vertex>& vertices, double tol,
                           std::size_t face_index) {
    const auto project = [&pl](const math::Vec3d& P) {
        const math::Vec3d d = P - pl.origin();
        return std::pair<double, double>(
            d.x * pl.u_axis().x + d.y * pl.u_axis().y +
                d.z * pl.u_axis().z,
            d.x * pl.v_axis().x + d.y * pl.v_axis().y +
                d.z * pl.v_axis().z);
    };

    Moments2 reg;
    for (std::size_t li = 0; li < f.loops().size(); ++li) {
        const auto& loop = f.loops()[li];
        if (loop.coedges().size() < 3)
            throw std::invalid_argument(
                "mass_properties: face " + std::to_string(face_index) +
                " loop " + std::to_string(li) + " carries fewer than 3 "
                "co-edges - a degenerate faceted polygon. Remedy: check "
                "the body's extraction (the bridge's error contract "
                "rejects degenerate triangles).");
        Moments2 L;
        std::pair<double, double> first{0.0, 0.0}, prev{0.0, 0.0};
        for (std::size_t k = 0; k <= loop.coedges().size(); ++k) {
            const auto& ce = loop.coedges()[k % loop.coedges().size()];
            const VertexId vid = ce.start_vertex(edges);
            if (vid >= vertices.size())
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(face_index) +
                    " references a vertex id out of range - a malformed "
                    "body. Remedy: report the body's construction.");
            const auto q = project(vertices[vid].position());
            if (k == 0) {
                first = q;
            } else {
                add_segment(L, prev.first, prev.second, q.first, q.second);
            }
            prev = q;
        }
        if (std::abs(L.A) <= tol * tol)
            throw std::invalid_argument(
                "mass_properties: face " + std::to_string(face_index) +
                " loop " + std::to_string(li) + " projects to a degenerate "
                "polygon (area " + std::to_string(L.A) + "). Remedy: check "
                "the body's extraction.");
        const double sigma = L.A > 0.0 ? 1.0 : -1.0;
        reg.accumulate(L, (li == 0 ? 1.0 : -1.0) * sigma);
    }
    return expand_planar(pl, reg);
}

} // namespace

// ---------------------------------------------------------------------------
// The surface payload reader - the typed dispatch (H1, ADR-0034 D3):
// the visitor extracts the concrete surface's parameters; the family
// switch decides the closed form. No string compare, no dynamic_cast
// chain.
// ---------------------------------------------------------------------------
namespace {

struct MassSurfaceReader : SurfaceVisitor {
    const PlaneSurface* plane{nullptr};
    const SphereSurface* sphere{nullptr};
    const TorusZSurface* torus_z{nullptr};
    const TorusSurface* torus_y{nullptr};
    bool cyl_found{false};
    CylFrame cyl{};

    void visit(const PlaneSurface& p) override { plane = &p; }
    void visit(const SphereSurface& s) override { sphere = &s; }
    // The two torus placements (G2-u3 m3): the fillet's Z-axis
    // writer and the analytic family's Y-axis class — one
    // frame, the cylinders' own unification.
    void visit(const TorusZSurface& z) override { torus_z = &z; }
    void visit(const TorusSurface& y) override { torus_y = &y; }
    void visit(const CylinderZSurface& z) override {
        cyl_found = true;
        cyl.P0[0] = z.axis_center().x;
        cyl.P0[1] = z.axis_center().y;
        cyl.P0[2] = z.z_base();
        cyl.a[0] = 0.0; cyl.a[1] = 0.0; cyl.a[2] = 1.0;
        cyl.e1[0] = 1.0; cyl.e1[1] = 0.0; cyl.e1[2] = 0.0;
        cyl.e2[0] = 0.0; cyl.e2[1] = 1.0; cyl.e2[2] = 0.0;
        cyl.r = z.radius();
        cyl.s = z.u_sign();
    }
    void visit(const CylinderSurface& y) override {
        cyl_found = true;
        cyl.P0[0] = y.origin().x;
        cyl.P0[1] = y.origin().y;
        cyl.P0[2] = y.origin().z;
        cyl.a[0] = 0.0; cyl.a[1] = 1.0; cyl.a[2] = 0.0;
        cyl.e1[0] = 1.0; cyl.e1[1] = 0.0; cyl.e1[2] = 0.0;
        cyl.e2[0] = 0.0; cyl.e2[1] = 0.0; cyl.e2[2] = -1.0;
        cyl.r = y.radius();
        cyl.s = 1.0;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// mass_properties — the divergence theorem over the body's faces
// ---------------------------------------------------------------------------
MassProperties mass_properties(const BRepBody& body, math::Tolerance tol) {
    const auto& faces = body.solid.shell().faces();

    if (faces.empty())
        throw std::invalid_argument(
            "mass_properties: the body has no faces to integrate - the mass "
            "properties of the empty solid are undefined. Remedy: check the "
            "operation (an A-minus-A subtract produces the empty body); "
            "query the operation's operands instead.");

    if (!is_closed_orientable(body))
        throw std::invalid_argument(
            "mass_properties: the body's shell fails the closed-orientable "
            "contract - the divergence theorem needs a CLOSED surface (an "
            "open or non-orientable shell has no volume to integrate). "
            "Remedy: check the body's construction; a boolean fragment that "
            "failed assembly is not a solid.");

    // The path decision (the honesty contract): ANY approximated face
    // sends the WHOLE body down the polyhedral path — exact and
    // approximated faces are never mixed in one sum.
    bool any_approximated = false;
    for (const auto& f : faces)
        if (!f.provenance().is_exact()) {
            any_approximated = true;
            break;
        }

    Contrib total;
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& f = faces[fi];
        const auto* surf = f.surface();
        // H1 (ADR-0034 D3): the typed dispatch - the family ENUM (a
        // typo cannot compile) and the payload VISITOR below; the
        // string survives only in the MESSAGES, derived from the enum.
        const SurfaceFamily family =
            surf ? surf->family() : SurfaceFamily::None;

        if (any_approximated) {
            // The polyhedral path: every face reduces to its vertex
            // polygon in its own plane.
            if (family != SurfaceFamily::Plane)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    " is approximated and curved (family '" +
                    std::string(family_name(family)) +
                    "') - the polyhedral path reduces PLANAR faces (the "
                    "bridge's faceted output); a curved approximated face "
                    "has no producer. Remedy: report the body's "
                    "construction.");
            MassSurfaceReader reader;
            surf->accept(reader);
            total.accumulate(polyhedron_contrib(*reader.plane, f, body.edges,
                                                body.vertices, tol.value, fi));
            continue;
        }

        // The exact path, per family (the typed dispatch).
        MassSurfaceReader reader;
        if (surf != nullptr) surf->accept(reader);
        if (family == SurfaceFamily::Plane) {
            total.accumulate(planar_contrib(*reader.plane, f, tol.value, fi));
        } else if (family == SurfaceFamily::Cylinder) {
            // The unified frame for both placements (the extrude's
            // Z-axis CylinderZSurface and the factory's Y-axis
            // CylinderSurface — G1 does not unify the surfaces, the
            // MASS machinery unifies the mathematics).
            if (!reader.cyl_found)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    " carries an unknown 'cylinder' subtype - the mass "
                    "machinery covers CylinderZSurface and "
                    "CylinderSurface. Remedy: report the body's "
                    "construction.");
            const CylFrame fr = reader.cyl;

            // The window: the bounding box of the wall's Line2
            // endpoints (the authored rectangle — the seam-wired walks
            // SPLIT edges, never move the window), verified on the
            // boundary (the patch class IS the rectangle).
            double u0 = std::numeric_limits<double>::infinity();
            double u1 = -std::numeric_limits<double>::infinity();
            double v0 = std::numeric_limits<double>::infinity();
            double v1 = -std::numeric_limits<double>::infinity();
            std::vector<std::pair<double, double>> endpoints;
            for (const auto& loop : f.loops())
                for (const auto& ce : loop.coedges()) {
                    const auto* pc = ce.p_curve();
                    if (pc == nullptr)
                        throw std::invalid_argument(
                            "mass_properties: face " + std::to_string(fi) +
                            " is exact but a co-edge carries no p-curve - "
                            "the exact world's invariant is violated. "
                            "Remedy: report the body's construction.");
                    if (pc->kind != PCurve::Kind::Segment)
                        throw std::invalid_argument(
                            "mass_properties: face " + std::to_string(fi) +
                            "'s cylindrical loop carries a non-Line2 "
                            "p-curve - outside the rectangle-window class "
                            "(the G1 patches are theta-interval x "
                            "z-interval rectangles, ADR-0032 D4). Remedy: "
                            "report the body's construction.");
                    endpoints.push_back({pc->segment.a.x, pc->segment.a.y});
                    endpoints.push_back({pc->segment.b.x, pc->segment.b.y});
                }
            for (const auto& e : endpoints) {
                u0 = std::min(u0, e.first);
                u1 = std::max(u1, e.first);
                v0 = std::min(v0, e.second);
                v1 = std::max(v1, e.second);
            }
            if (u1 - u0 <= tol.value || v1 - v0 <= tol.value)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    "'s cylindrical window is degenerate ([" +
                    std::to_string(u0) + ", " + std::to_string(u1) + "] x [" +
                    std::to_string(v0) + ", " + std::to_string(v1) +
                    "]) - a zero-area patch. Remedy: report the body's "
                    "construction.");
            if (u1 - u0 > 2.0 * M_PI + tol.value)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    "'s cylindrical window spans " +
                    std::to_string(u1 - u0) +
                    " > 2pi - the patch would double-cover the cylinder. "
                    "Remedy: report the body's construction.");
            for (const auto& e : endpoints) {
                const bool on_u =
                    std::abs(e.first - u0) <= tol.value ||
                    std::abs(e.first - u1) <= tol.value;
                const bool on_v =
                    std::abs(e.second - v0) <= tol.value ||
                    std::abs(e.second - v1) <= tol.value;
                if (!(on_u || on_v))
                    throw std::invalid_argument(
                        "mass_properties: face " + std::to_string(fi) +
                        "'s cylindrical loop leaves the window boundary "
                        "(the point (" + std::to_string(e.first) + ", " +
                        std::to_string(e.second) +
                        ") is strictly inside) - a non-rectangular patch "
                        "is outside the G1 class. Remedy: report the "
                        "body's construction.");
            }
            total.accumulate(cylinder_contrib(fr, u0, u1, v0, v1));
        } else if (family == SurfaceFamily::Sphere) {
            // The FULL sphere's closed form — the only sphere the
            // exact world produces (trimmed spheres are G2's, D1).
            if (faces.size() != 1)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    " is a sphere among " + std::to_string(faces.size()) +
                    " faces - only the FULL sphere (the factory's "
                    "seam-wired single-face body) is in the G1 class; a "
                    "trimmed spherical region arises from sphere "
                    "booleans (G2's table). Remedy: check the body's "
                    "construction.");
            total.accumulate(sphere_contrib(reader.sphere->origin().x,
                                            reader.sphere->origin().y,
                                            reader.sphere->origin().z,
                                            reader.sphere->radius()));
        } else if (family == SurfaceFamily::Torus) {
            // THE TOROIDAL PATCH (G2-u3 m3, ADR-0039 D1.2): the
            // (u, v) rectangle integrals — the family that landed
            // with its first consumer (the fillet's flank). The frame
            // unifies both placements (the cylinders' own discipline).
            TorusFrame fr{};
            if (reader.torus_z != nullptr) {
                fr.C[0] = reader.torus_z->axis_center().x;
                fr.C[1] = reader.torus_z->axis_center().y;
                fr.C[2] = reader.torus_z->z_center();
                fr.a[0] = 0.0; fr.a[1] = 0.0; fr.a[2] = 1.0;
                fr.e1[0] = 1.0; fr.e1[1] = 0.0; fr.e1[2] = 0.0;
                fr.e2[0] = 0.0; fr.e2[1] = 1.0; fr.e2[2] = 0.0;
                fr.R = reader.torus_z->major_radius();
                fr.r = reader.torus_z->minor_radius();
            } else if (reader.torus_y != nullptr) {
                fr.C[0] = reader.torus_y->origin().x;
                fr.C[1] = reader.torus_y->origin().y;
                fr.C[2] = reader.torus_y->origin().z;
                fr.a[0] = 0.0; fr.a[1] = 1.0; fr.a[2] = 0.0;
                fr.e1[0] = 1.0; fr.e1[1] = 0.0; fr.e1[2] = 0.0;
                // e2 = (0, 0, -1): the m3 flip's own frame —
                // u = -azimuth around the Y axis, right-handed.
                fr.e2[0] = 0.0; fr.e2[1] = 0.0; fr.e2[2] = -1.0;
                fr.R = reader.torus_y->major_radius();
                fr.r = reader.torus_y->minor_radius();
            } else {
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    " carries an unknown 'torus' subtype - the mass "
                    "machinery covers TorusZSurface and TorusSurface. "
                    "Remedy: report the body's construction.");
            }

            // The window: the bounding box of the loop's Line2
            // endpoints (the cylinder patch's own discipline —
            // the seam-wired walks SPLIT edges, never move the window),
            // verified on the boundary (the patch class IS the
            // rectangle).
            double u0 = std::numeric_limits<double>::infinity();
            double u1 = -std::numeric_limits<double>::infinity();
            double v0 = std::numeric_limits<double>::infinity();
            double v1 = -std::numeric_limits<double>::infinity();
            std::vector<std::pair<double, double>> endpoints;
            for (const auto& loop : f.loops())
                for (const auto& ce : loop.coedges()) {
                    const auto* pc = ce.p_curve();
                    if (pc == nullptr)
                        throw std::invalid_argument(
                            "mass_properties: face " + std::to_string(fi) +
                            " is exact but a co-edge carries no p-curve - "
                            "the exact world's invariant is violated. "
                            "Remedy: report the body's construction.");
                    if (pc->kind != PCurve::Kind::Segment)
                        throw std::invalid_argument(
                            "mass_properties: face " + std::to_string(fi) +
                            "'s toroidal loop carries a non-Line2 "
                            "p-curve - outside the rectangle-window class "
                            "(the toroidal patches are u-interval x "
                            "v-interval rectangles, ADR-0039 D1.2). "
                            "Remedy: report the body's construction.");
                    endpoints.push_back({pc->segment.a.x, pc->segment.a.y});
                    endpoints.push_back({pc->segment.b.x, pc->segment.b.y});
                }
            for (const auto& e : endpoints) {
                u0 = std::min(u0, e.first);
                u1 = std::max(u1, e.first);
                v0 = std::min(v0, e.second);
                v1 = std::max(v1, e.second);
            }
            if (u1 - u0 <= tol.value || v1 - v0 <= tol.value)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    "'s toroidal window is degenerate ([" +
                    std::to_string(u0) + ", " + std::to_string(u1) +
                    "] x [" + std::to_string(v0) + ", " +
                    std::to_string(v1) +
                    "]) - a zero-area patch. Remedy: report the body's "
                    "construction.");
            if (u1 - u0 > 2.0 * M_PI + tol.value)
                throw std::invalid_argument(
                    "mass_properties: face " + std::to_string(fi) +
                    "'s toroidal window spans " + std::to_string(u1 - u0) +
                    " > 2pi - the patch would double-cover the torus. "
                    "Remedy: report the body's construction.");
            for (const auto& e : endpoints) {
                const bool on_u =
                    std::abs(e.first - u0) <= tol.value ||
                    std::abs(e.first - u1) <= tol.value;
                const bool on_v =
                    std::abs(e.second - v0) <= tol.value ||
                    std::abs(e.second - v1) <= tol.value;
                if (!(on_u || on_v))
                    throw std::invalid_argument(
                        "mass_properties: face " + std::to_string(fi) +
                        "'s toroidal loop leaves the window boundary "
                        "(the point (" + std::to_string(e.first) + ", " +
                        std::to_string(e.second) +
                        ") is strictly inside) - a non-rectangular patch "
                        "is outside the family. Remedy: report the "
                        "body's construction.");
            }
            total.accumulate(torus_contrib(fr, u0, u1, v0, v1));
        } else {
            throw std::invalid_argument(
                "mass_properties: face " + std::to_string(fi) + " carries a "
                "'" + std::string(family_name(family)) +
                "' surface - outside the supported families (plane with "
                "line/arc loops, cylindrical patch, full sphere, toroidal "
                "patch - landed G2-u3 m3 with the fillet's flank; the cone "
                "closed form arrives with its own first writer, registered, "
                "none designed today). Remedy: check the body's faces, or "
                "accept the faceted path (the SDF materialization's bridge "
                "extract answers the polyhedral value).");
        }
    }

    // ---- the aggregate ------------------------------------------------
    MassProperties mp;
    mp.all_exact = !any_approximated;

    const double V = total.F0 / 3.0;
    if (V <= tol.value)
        throw std::invalid_argument(
            std::string("mass_properties: the volume integral reads ") +
            std::to_string(V) + " - " +
            (V < -tol.value
                 ? "NEGATIVE: the body's face normals point inward (an "
                   "assembly defect; the divergence theorem reads the "
                   "oriented volume)"
                 : "zero within the declared tolerance (a degenerate "
                   "body)") +
            ". Remedy: check the body's construction; the centroid and "
            "the inertia are undefined without a volume.");
    mp.volume = V;

    double M[3];
    for (int i = 0; i < 3; ++i) M[i] = total.Fi[i] / 2.0;
    double S[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) S[i][j] = total.Fij[i][j] / 5.0;

    const double c[3] = {M[0] / V, M[1] / V, M[2] / V};
    mp.centroid = math::Vec3d{c[0], c[1], c[2]};

    // I(origin) = δ·tr(S) − S; then the parallel axis to the centroid.
    const double tr = S[0][0] + S[1][1] + S[2][2];
    const double c2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const double I0 = (i == j ? tr : 0.0) - S[i][j];
            const double shift = V * ((i == j ? c2 : 0.0) - c[i] * c[j]);
            mp.inertia_about_centroid(i, j) = I0 - shift;
        }
    return mp;
}

} // namespace CAD_0::brep
