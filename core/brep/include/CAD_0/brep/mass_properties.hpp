// core/brep/include/CAD_0/brep/mass_properties.hpp
//
// THE EXACT MASS PROPERTIES — ADR-0032 D4 (G1 unit 5): the B-Rep's
// first quantitative interrogation. Volume, centroid, inertia tensor
// — computed by the DIVERGENCE THEOREM over the exact faces, never
// estimated from a mesh.
//
// THE METHOD, FROM FIRST PRINCIPLES. For a closed body B with
// outward unit normal n on ∂B, every volume moment is a surface
// integral:
//
//     V   = (1/3) ∮_∂B x·n  dA                    (volume)
//     M_i = (1/2) ∮_∂B x_i² n_i dA                (first moments:
//            ∇·(½ x_i² e_i) = x_i)
//     S_ij = (1/5) ∮_∂B x_i x_j (x·n) dA          (second moments)
//
// The 1/5 is DERIVED, not looked up: the primitive-function ansatz
// ∇·((1/5) x_i x_j x) = x_i x_j — the monomial x_i x_j x_k carries
// five contraction terms (two from the indices i,j; three from the
// position vector's k), each contributing 1 to the divergence of
// (c · x_i x_j x). The closed-form tests are the check.
//
// The centroid is M/V; the inertia tensor about the origin is
// I_ij = δ_ij·tr(S) − S_ij (density 1); to the centroid via the
// classical parallel-axis subtraction V(|c|²δ_ij − c_i c_j).
//
// THE CLOSED FORMS PER G1 FACE FAMILY (each derived; the machinery
// below carries the derivations inline):
//
//   * PLANE FACE (region bounded by Line2/Arc2 loops, in the face's
//     own (u,v) domain): n is constant and x·n = origin·n is constant
//     over the plane, so every kernel reduces to the planar region's
//     moments {A, ∮u, ∮v, ∮u², ∮uv, ∮v²} — computed from the loops by
//     GREEN'S THEOREM (the polygon part by the shoelace pattern, each
//     arc by its trigonometric line integrals, a lone Circle2 inner
//     loop by the disc's closed forms). The region semantics are the
//     kernel's own: the FIRST loop is the outer boundary, subsequent
//     loops are holes (ADR-0021), and each loop's winding sign σ =
//     sign(∮u dv as traversed) — so outer-minus-inner is positional
//     arithmetic, whatever direction each walk runs (the mirror and
//     rotation frames of the caps, the flipped walks of the pierced
//     faces: all the same mathematics, read geometrically).
//
//   * CYLINDRICAL PATCH (a (θ, z) rectangle window, radius r): the
//     unified frame {P0 (axis base point), a (axis), e1, e2 (radial
//     basis), s (the normal sign — the frame rule's u_sign)} writes
//     x = P0 + v·a + r·radial(φ), n = s·radial(φ), dA = r dφ dv, and
//     every kernel is a product of powers of (cos φ, sin φ) and v
//     over the rectangle — integrated term by term through the
//     reduction recurrences of ∫cos^i sin^j (derived below, no
//     tables). The window is read from the wall's Line2 loops (the
//     bounding box of the authored rectangle — the seam-wired walks
//     split edges, never move the window).
//
//   * SPHERE (full, seam-wired — the only sphere the exact world
//     produces): x·n = R, so the closed forms V = (4/3)πR³,
//     M = V·C, S_ij = V(C_iC_j + δ_ij R²/5). Trimmed spheres arise
//     only from sphere booleans — outside G1's subtract class (D1) —
//     honestly declared; the field stays G2's.
//
// THE POLYHEDRAL PATH (the honesty contract): if ANY face's
// provenance is approximated (a faceted body from the bridge), the
// RESULT is the polyhedral divergence value — EVERY face reduced to
// its vertex polygon in its own plane (Mirtich 1996's reduction,
// subsumed by the same Green machinery with synthesized segments),
// and all_exact is false. Exact and approximated faces are NEVER
// mixed in one sum: the flag is the honesty, the value is the
// polyhedron's.
//
// THE ERROR CONTRACT (the ADR-0026 P2 pattern — visible, never a
// wrong number): out-of-class faces (cone, torus, a curved
// approximated face, an exact face without p-curves), the empty
// body, the non-closed shell, and the degenerate volume each refuse
// with std::invalid_argument naming the diagnosis and the remedy.
//
// DETERMINISM (ADR-0014): the arithmetic is +, −, ×, ÷ only (the
// recurrences and the disc/sphere constants; no sqrt anywhere in
// this machinery), the iteration orders are the shell/loop/co-edge
// orders, and the doubles are deterministic under -ffp-contract=off.
// The API's storage (double volume; float centroid/inertia, the
// pinned struct) casts once, deterministically.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/math/vec.hpp"

namespace CAD_0::brep {

// The answer, as plain values (the ADR-0032 D4 pinned shape).
// Density 1 throughout: consumers scale.
struct MassProperties {
    double volume{0.0};
    math::Vec3f centroid{};
    math::Mat3 inertia_about_centroid{};  // row-major access via (r, c)
    // Provenance of every contributing face was 'exact'.
    bool all_exact{true};
};

// mass_properties — the divergence theorem over the body's faces.
//
// `tol` is the operation's declared Layer-3 decision tolerance
// (ADR-0004: explicit per operation, no global epsilon). It bands
// the degenerate-volume refusal, the degenerate-loop winding check
// (tol² — the domain areas), and the cylindrical window's
// on-boundary verification.
//
// Throws std::invalid_argument (diagnosis + remedy, the P2 pattern)
// for: the empty body; a shell that fails the closed-orientable
// contract (the divergence theorem needs a closed surface); an
// out-of-class face family in the exact path (cone, torus, an exact
// face without p-curves, a non-rectangular cylindrical window, a
// curved face in the polyhedral path); a degenerate loop; and a
// volume that reads zero within the tolerance (the centroid is
// undefined).
MassProperties mass_properties(const BRepBody& body, math::Tolerance tol);

} // namespace CAD_0::brep
