// core/brep/src/tessellate.cpp
//
// THE DISPLAY BOUNDARY (H1, ADR-0034): the tessellation is the
// float pipeline's own domain (the SDF mesh world); the exact
// surfaces are double. The conversion is ONE DIRECTION (double ->
// float), explicit at each site below, and the parameter windows are
// dispatched on the TYPED family enum (no string compare).
//
// THE TRIM (G2-u3 m6-correttiva-bis, the click-test's second "non
// passed" — the six crossed planes): the face's OWN window and the
// boundary's own shape. The uncured road sampled the Plane family
// over the hardcoded [-10,10]^2 parametric window and the closed
// families over their full sweeps — it never read the face's loops:
// on the box guide case the display mesh carried 16,272 of 16,807
// vertices (96.8%) OUTSIDE the filleted body, the planes reaching
// +-10 in a [-1,1]^3 world and the flank sampled over the full 360
// degrees for its quarter. The cure reads what the exact kernel
// already authored — the p-curves, every co-edge's use of its edge
// typed in the owning face's domain (ADR-0032 D2.2) — in two moves:
//
//   * THE WINDOW: the parametric box of the boundary's p-curves —
//     the face's own extent, never the family's hardcoded default;
//   * THE REGION: the even-odd test over the loops sampled as
//     polylines — inside the outer loop, outside the inner ones,
//     the seam jumps (the closed surfaces' meridian pairs) closed
//     with straight segments: in the unrolled plane the sphere's
//     seam pair closes as the window rectangle.
//
// The region BITS (interior_left / hole) are deliberately NOT read
// here: they are the mass machinery's vocabulary, a different
// consumer with a different question. On the blind hole's internal
// cap the subtract INVERTS the bit to declare the material side
// flipped for the VOLUME (subtract D1.1) — and the same hole=true
// that means "outside the circle" on the pierced top face's rim
// means the disc on the cap. The bits answer "where the material
// lies"; the SHAPES answer "where the patch is" — and the display
// tessellates the patch.
//
// THE DECLARED LIMITS (the honest boundary of this cure):
//   * the per-face grids stay UNWELDED — the bridge's watertight
//     contract still refuses them by name (H2-u4); a watertight
//     tessellator remains the registered follow-up;
//   * the trim is decided at the CELL CENTER: a curved boundary
//     shows as the display grid's own stairstep (the class MC's
//     road already declares), a strip thinner than a cell may
//     vanish — the display resolution's own honesty, never a
//     promise of exactness;
//   * the faceted world (bridge-extracted faces, plane-per-triangle,
//     ADR-0026) carries NO p-curves: those faces keep the family
//     windows below — the declared fallback, never a guess.
#include "CAD_0/brep/tessellate.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace CAD_0::brep {

namespace {

// The p-curve vocabulary's own constants (pcurve.hpp) — kPi, kTwoPi —
// are the domain's one opinion; no second definition here.

// Sample a p-curve as a 2D polyline — the SHAPE alone. Lines are
// their two endpoints; circles and arcs are sampled CCW at a density
// whose chord error sits far below a display cell (64 points per
// full turn: on r = 0.75 the sagitta is ~4e-3 against the 2/48 cell
// at the workbench's resolution).
void sample_pcurve(const PCurve& c, std::vector<math::Vec2d>& out) {
    switch (c.kind) {
    case PCurve::Kind::Segment:
        out.push_back(c.segment.a);
        out.push_back(c.segment.b);
        break;
    case PCurve::Kind::Circle: {
        const int k = 64;
        for (int i = 0; i < k; ++i) {
            const double t = kTwoPi * double(i) / double(k);
            out.push_back(c.circle.center +
                          math::Vec2d{c.circle.radius * std::cos(t),
                                      c.circle.radius * std::sin(t)});
        }
        break;
    }
    case PCurve::Kind::Arc: {
        const double sweep = c.arc.sweep;
        int k = int(std::ceil(64.0 * sweep / kTwoPi));
        if (k < 4) k = 4;
        for (int i = 0; i <= k; ++i) {
            const double t = c.arc.start_angle +
                             sweep * double(i) / double(k);
            out.push_back(c.arc.center +
                          math::Vec2d{c.arc.radius * std::cos(t),
                                      c.arc.radius * std::sin(t)});
        }
        break;
    }
    }
}

// The face's loops as closed polylines. False — the declared
// fallback — when the face does not speak the exact vocabulary:
// a loop with no co-edges, or any co-edge without its p-curve (the
// faceted world). The p-curves chain head-to-tail (the vocabulary's
// DIRECTION convention); where a closed surface's seam pair jumps
// across the period the gap closes with a straight segment — the
// unrolled plane's own identification.
bool face_polygons(const Face& face,
                   std::vector<std::vector<math::Vec2d>>& loops) {
    for (const Loop& loop : face.loops()) {
        if (loop.coedges().empty()) return false;
        std::vector<math::Vec2d> poly;
        for (const CoEdge& ce : loop.coedges()) {
            const PCurve* pc = ce.p_curve();
            if (pc == nullptr) return false;
            std::vector<math::Vec2d> pts;
            sample_pcurve(*pc, pts);
            if (poly.empty()) {
                poly = pts;
                continue;
            }
            if (!(pts.front().x == poly.back().x &&
                  pts.front().y == poly.back().y))
                poly.push_back(pts.front());  // the seam jump, straight
            for (std::size_t k = 1; k < pts.size(); ++k)
                poly.push_back(pts[k]);
        }
        if (poly.size() < 3) return false;
        if (!(poly.front().x == poly.back().x &&
              poly.front().y == poly.back().y))
            poly.push_back(poly.front());    // the ring's closing
        loops.push_back(std::move(poly));
    }
    return !loops.empty();
}

// The even-odd region test: a point is in the patch iff it lies
// inside an odd number of the loops' polygons — inside the outer
// loop, outside the inner ones, whatever the walks' directions (the
// ray cast is direction-free, the reason this reads shapes and not
// region bits).
bool in_patch(const std::vector<std::vector<math::Vec2d>>& loops,
              math::Vec2d p) {
    bool inside = false;
    for (const std::vector<math::Vec2d>& poly : loops) {
        const std::size_t n = poly.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const math::Vec2d& a = poly[i];
            const math::Vec2d& b = poly[j];
            if ((a.y > p.y) != (b.y > p.y)) {
                const double t = (p.y - b.y) / (a.y - b.y);
                if (p.x < b.x + t * (a.x - b.x)) inside = !inside;
            }
        }
    }
    return inside;
}

} // namespace

sdf::TriangleMesh tessellate_face(const Face& face,
                                  const TessellationOptions& opts) {
    sdf::TriangleMesh mesh;
    const SurfaceGeometry* surf = face.surface();
    if (!surf) return mesh;

    const std::uint32_t N = opts.resolution;

    // The family windows — the declared fallback (the faceted world,
    // a face whose boundary box degenerates): full parametric range
    // for closed surfaces, the legacy fixed range for open ones.
    double u_min = 0.0, u_max = kTwoPi;
    double v_min = 0.0, v_max = 1.0;
    switch (surf->family()) {
    case SurfaceFamily::Plane:
        u_min = -10.0; u_max = 10.0;
        v_min = -10.0; v_max = 10.0;
        break;
    case SurfaceFamily::Cylinder:
        v_min = -1.0; v_max = 1.0;  // half-height
        break;
    case SurfaceFamily::Sphere:
        v_min = 0.0; v_max = kPi;
        break;
    case SurfaceFamily::Cone:
        v_min = 0.0; v_max = 1.0;
        break;
    case SurfaceFamily::Torus:
        u_min = 0.0; u_max = kTwoPi;
        v_min = 0.0; v_max = kTwoPi;
        break;
    default:
        break;
    }

    // THE WINDOW AND THE REGION (the trim): the boundary's own box
    // overrides the family defaults whenever the face speaks the
    // exact vocabulary and the box is non-degenerate (a closed loop
    // spans both axes; the guard is the honest fallback, never a
    // guess).
    std::vector<std::vector<math::Vec2d>> loops;
    bool trimmed = false;
    if (face_polygons(face, loops)) {
        double bu0 = loops[0][0].x, bu1 = bu0;
        double bv0 = loops[0][0].y, bv1 = bv0;
        for (const std::vector<math::Vec2d>& poly : loops)
            for (const math::Vec2d& p : poly) {
                if (p.x < bu0) bu0 = p.x;
                if (p.x > bu1) bu1 = p.x;
                if (p.y < bv0) bv0 = p.y;
                if (p.y > bv1) bv1 = p.y;
            }
        if (bu1 - bu0 > 0.0 && bv1 - bv0 > 0.0) {
            u_min = bu0; u_max = bu1;
            v_min = bv0; v_max = bv1;
            trimmed = true;
        }
    }

    // Sample grid over the (own or fallback) window.
    const std::uint32_t Nu = N + 1;
    const std::uint32_t Nv = N + 1;
    const double du = (u_max - u_min) / double(N);
    const double dv = (v_max - v_min) / double(N);

    // Generate vertices (the double->float boundary conversion, one
    // direction, declared here — the mesh's own domain). The trim
    // emits a cell only when its CENTER lies in the patch; the
    // vertices ride a compact grid->mesh map (the dropped cells'
    // corners never enter the buffers).
    const auto to_f3 = [](const math::Vec3d& p) {
        return math::Vec3f{static_cast<float>(p.x),
                           static_cast<float>(p.y),
                           static_cast<float>(p.z)};
    };
    std::vector<std::int32_t> grid(static_cast<std::size_t>(Nu) * Nv,
                                   -1);
    const auto vertex_at =
        [&](std::uint32_t i, std::uint32_t j) -> std::uint32_t {
            const std::size_t g =
                static_cast<std::size_t>(j) * Nu + i;
            if (grid[g] < 0) {
                grid[g] = static_cast<std::int32_t>(
                    mesh.positions.size());
                const double u = u_min + double(i) * du;
                const double v = v_min + double(j) * dv;
                mesh.positions.push_back(to_f3(surf->evaluate(u, v)));
                if (opts.compute_normals)
                    mesh.normals.push_back(to_f3(surf->normal(u, v)));
            }
            return static_cast<std::uint32_t>(grid[g]);
        };

    // Generate triangles (two per grid cell, the winding unchanged)
    // — the cell's center deciding the trim.
    for (std::uint32_t j = 0; j < N; ++j) {
        for (std::uint32_t i = 0; i < N; ++i) {
            if (trimmed) {
                const math::Vec2d c{u_min + (double(i) + 0.5) * du,
                                    v_min + (double(j) + 0.5) * dv};
                if (!in_patch(loops, c)) continue;
            }
            const std::uint32_t v00 = vertex_at(i, j);
            const std::uint32_t v10 = vertex_at(i + 1, j);
            const std::uint32_t v01 = vertex_at(i, j + 1);
            const std::uint32_t v11 = vertex_at(i + 1, j + 1);

            mesh.indices.push_back(v00);
            mesh.indices.push_back(v10);
            mesh.indices.push_back(v11);

            mesh.indices.push_back(v00);
            mesh.indices.push_back(v11);
            mesh.indices.push_back(v01);
        }
    }

    return mesh;
}

sdf::TriangleMesh tessellate_body(const BRepBody& body,
                                  const TessellationOptions& opts) {
    sdf::TriangleMesh mesh;

    for (const auto& face : body.solid.shell().faces()) {
        sdf::TriangleMesh face_mesh = tessellate_face(face, opts);
        // Append face_mesh to mesh
        const std::uint32_t offset =
            static_cast<std::uint32_t>(mesh.positions.size());
        for (const auto& p : face_mesh.positions) mesh.positions.push_back(p);
        if (opts.compute_normals && !face_mesh.normals.empty()) {
            for (const auto& n : face_mesh.normals) mesh.normals.push_back(n);
        }
        for (const std::uint32_t idx : face_mesh.indices)
            mesh.indices.push_back(idx + offset);
    }

    return mesh;
}

} // namespace CAD_0::brep
