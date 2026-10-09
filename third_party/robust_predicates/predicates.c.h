/*
 * third_party/robust_predicates/predicates.c.h
 *
 * Stub for Shewchuk's robust geometric predicates.
 *
 * This file provides a working, *non-adaptive* fallback that produces
 * correct sign for non-degenerate inputs. It is intended to allow the
 * rest of cadforge to compile and run during Phase A.
 *
 * For production use (Phase D onwards), replace with the full Shewchuk
 * implementation by downloading robust.c from:
 *   http://www.cs.cmu.edu/~quake/robust.html
 * and renaming it to predicates.c.h.
 *
 * Original author: Jonathan Richard Shewchuk
 * License: Public Domain
 */

#ifndef ROBUST_PREDICATES_C_H
#define ROBUST_PREDICATES_C_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Floating-point error bounds (computed once at init).
 * In the full Shewchuk implementation these are derived from
 * the floating-point characteristics of the host CPU.
 * ============================================================ */

static double splitter;       /* = 2^ceiling(p/2) + 1 */
static double epsilon;       /* = 2^-(p-1), unit round-off */
static double resulterrbond;
static double ccwerrboundA, ccwerrboundB, ccwerrboundC;
static double o3derrboundA,  o3derrboundB,  o3derrboundC;
static double iccerrboundA, iccerrboundB, iccerrboundC;
static double isperrboundA, isperrboundB, isperrboundC;

static void exactinit(void) {
    double half = 0.5;
    /* IEEE 754 double has 53 bits of precision.
     * epsilon = 2^-(p-1) computed via ldexp to avoid shift overflow. */
    epsilon = half * (1 << 30);  /* start from a safe shift */
    epsilon = 1.0;
    while (half * epsilon + 1.0 != 1.0) epsilon *= half;
    splitter = 1.0 + epsilon;
    resulterrbond = (3.0 + 8.0 * epsilon) * epsilon;
    ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
    ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
    ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon;
    o3derrboundA = (7.0 + 56.0 * epsilon) * epsilon;
    o3derrboundB = (3.0 + 28.0 * epsilon) * epsilon;
    o3derrboundC = (26.0 + 288.0 * epsilon) * epsilon;
    iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
    iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
    iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon;
    isperrboundA = (16.0 + 224.0 * epsilon) * epsilon;
    isperrboundB = (5.0 + 72.0 * epsilon) * epsilon;
    isperrboundC = (71.0 + 1408.0 * epsilon) * epsilon;
    (void)resulterrbond;
    (void)isperrboundA; (void)isperrboundB; (void)isperrboundC;
}

/* ============================================================
 * 2D orientation — non-adaptive fallback
 * Returns a positive value if pa, pb, pc are counter-clockwise,
 * negative if clockwise, 0 if collinear.
 *
 * For correct sign on near-degenerate inputs, replace with the
 * full Shewchuk adaptive version.
 * ============================================================ */
static double orient2dadapt(const double *pa, const double *pb,
                             const double *pc, double detsum) {
    (void)detsum;
    const double acx = pa[0] - pc[0];
    const double bcy = pb[1] - pc[1];
    const double acy = pa[1] - pc[1];
    const double bcx = pb[0] - pc[0];
    return acx * bcy - acy * bcx;
}

static double orient2d(const double *pa, const double *pb, const double *pc) {
    const double detleft  = (pa[0] - pc[0]) * (pb[1] - pc[1]);
    const double detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
    const double detsum = (detleft > 0.0 ? detleft : -detleft)
                        + (detright > 0.0 ? detright : -detright);
    return orient2dadapt(pa, pb, pc, detsum);
}

/* ============================================================
 * 3D orientation
 * Returns positive if pd is below the plane through pa,pb,pc
 * (right-handed coordinate system), negative if above, 0 if coplanar.
 * ============================================================ */
static double orient3dadapt(const double *pa, const double *pb,
                             const double *pc, const double *pd,
                             double apermanent) {
    (void)apermanent;
    const double adx = pa[0] - pd[0];
    const double bdx = pb[0] - pd[0];
    const double cdx = pc[0] - pd[0];
    const double ady = pa[1] - pd[1];
    const double bdy = pb[1] - pd[1];
    const double cdy = pc[1] - pd[1];
    const double adz = pa[2] - pd[2];
    const double bdz = pb[2] - pd[2];
    const double cdz = pc[2] - pd[2];
    const double detleft  = adx * (bdy * cdz - bdz * cdy);
    const double detright = bdx * (ady * cdz - adz * cdy);
    const double detmid   = cdx * (ady * bdz - adz * bdy);
    return detleft + detright + detmid;
}

static double orient3d(const double *pa, const double *pb,
                       const double *pc, const double *pd) {
    const double adx = pa[0] - pd[0];
    const double bdx = pb[0] - pd[0];
    const double cdx = pc[0] - pd[0];
    const double ady = pa[1] - pd[1];
    const double bdy = pb[1] - pd[1];
    const double cdy = pc[1] - pd[1];
    const double adz = pa[2] - pd[2];
    const double bdz = pb[2] - pd[2];
    const double cdz = pc[2] - pd[2];
    const double detleft  = adx * (bdy * cdz - bdz * cdy);
    const double detright = bdx * (ady * cdz - adz * cdy);
    const double detmid   = cdx * (ady * bdz - adz * bdy);
    const double permanent = (detleft > 0.0 ? detleft : -detleft)
                           + (detright > 0.0 ? detright : -detright)
                           + (detmid > 0.0 ? detmid : -detmid);
    return orient3dadapt(pa, pb, pc, pd, permanent);
}

/* ============================================================
 * Incircle test — non-adaptive fallback
 * Returns positive if pd lies inside the circle through pa,pb,pc,
 * negative if outside, 0 if cocircular.
 * ============================================================ */
static double incircleadapt(const double *pa, const double *pb,
                             const double *pc, const double *pd,
                             double permanent) {
    (void)permanent;
    const double adx = pa[0] - pd[0];
    const double ady = pa[1] - pd[1];
    const double bdx = pb[0] - pd[0];
    const double bdy = pb[1] - pd[1];
    const double cdx = pc[0] - pd[0];
    const double cdy = pc[1] - pd[1];
    const double alift = adx * adx + ady * ady;
    const double blift = bdx * bdx + bdy * bdy;
    const double clift = cdx * cdx + cdy * cdy;
    const double det = alift * (bdx * cdy - bdy * cdx)
                    + blift * (cdx * ady - cdy * adx)
                    + clift * (adx * bdy - ady * bdx);
    return det;
}

static double incircle(const double *pa, const double *pb,
                       const double *pc, const double *pd) {
    const double adx = pa[0] - pd[0];
    const double ady = pa[1] - pd[1];
    const double bdx = pb[0] - pd[0];
    const double bdy = pb[1] - pd[1];
    const double cdx = pc[0] - pd[0];
    const double cdy = pc[1] - pd[1];
    const double alift = adx * adx + ady * ady;
    const double blift = bdx * bdx + bdy * bdy;
    const double clift = cdx * cdx + cdy * cdy;
    const double permanent = (alift + blift + clift) * 8.0 * epsilon + 1.0;
    return incircleadapt(pa, pb, pc, pd, permanent);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ROBUST_PREDICATES_C_H */
