// bindings/python/src/sdf_bindings.cpp
//
// Exposes the native SDF kernel to Python. The public API mirrors the
// C++ helpers in core/sdf/primitives.hpp and operators.hpp, plus the
// batched evaluation API in core/sdf/evaluate.hpp.
//
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/evaluate.hpp"

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/quat.hpp"

#include <memory>

namespace nb = nanobind;
using namespace CAD_0::sdf;
using namespace CAD_0::math;

void bind_sdf(nb::module_& m) {
    // SDFBody — opaque handle to an SDF tree root.
    nb::class_<SDFBody>(m, "SDFBody")
        .def(nb::init<>())
        .def("value",     &SDFBody::value)
        .def("sample",    [](const SDFBody& b, const Vec3f& p) { return b.sample(p); })
        .def("lipschitz",  &SDFBody::lipschitz)
        .def("bounds",     &SDFBody::bounds)
        .def("describe",   &SDFBody::describe)
        .def("__repr__",   [](const SDFBody& b) {
            return "<SDFBody " + b.describe() + ">";
        });

    // Primitives
    m.def("sphere",   &make_sphere,   nb::arg("radius"));
    m.def("box",      [](Vec3f e)     { return make_box(e); }, nb::arg("extent"));
    m.def("cylinder", &make_cylinder, nb::arg("radius"), nb::arg("height"));
    m.def("torus",    &make_torus,    nb::arg("R"), nb::arg("r"));
    m.def("cone",     &make_cone,     nb::arg("radius"), nb::arg("height"));
    m.def("capsule",  &make_capsule,  nb::arg("a"), nb::arg("b"), nb::arg("radius"));
    m.def("plane",    &make_plane);

    // Operators
    m.def("union",          &sdf_union,          nb::arg("a"), nb::arg("b"));
    m.def("intersect",      &sdf_intersect,      nb::arg("a"), nb::arg("b"));
    m.def("subtract",       &sdf_subtract,       nb::arg("a"), nb::arg("b"));
    m.def("smooth_union",   &sdf_smooth_union,   nb::arg("a"), nb::arg("b"), nb::arg("k"));

    // Transforms
    m.def("translate", &sdf_translate, nb::arg("body"), nb::arg("offset"));
    m.def("rotate",    &sdf_rotate,    nb::arg("body"), nb::arg("q"));
    m.def("scale",     &sdf_scale,     nb::arg("body"), nb::arg("s"));
    m.def("twist",     &sdf_twist,     nb::arg("body"), nb::arg("k"));

    // Batched evaluation
    nb::class_<EvalResult>(m, "EvalResult")
        .def_ro("values",    &EvalResult::values)
        .def_ro("gradients", &EvalResult::gradients);

    // Accept either a list of Vec3f or a numpy (N,3) float32 array.
    m.def("evaluate", [](const SDFBody& body, nb::ndarray<float, nb::ndim<2>> points) {
        if (points.shape(1) != 3) {
            throw std::invalid_argument("points must have shape (N, 3)");
        }
        const auto n = points.shape(0);
        std::vector<Vec3f> pts(n);
        for (std::size_t i = 0; i < n; ++i) {
            pts[i] = Vec3f{points(i, 0), points(i, 1), points(i, 2)};
        }
        EvalResult out;
        CpuEvalBackend backend;
        backend.evaluate(body, std::span<const Vec3f>(pts.data(), n), out);
        return out;
    }, nb::arg("body"), nb::arg("points").noconvert());

    // EvalBackend (for documentation / introspection)
    nb::class_<CpuEvalBackend>(m, "CpuEvalBackend")
        .def(nb::init<unsigned>(), nb::arg("num_threads") = 0)
        .def("evaluate",
             [](CpuEvalBackend& b, const SDFBody& body,
                nb::ndarray<float, nb::ndim<2>> points) {
            if (points.shape(1) != 3)
                throw std::invalid_argument("points must have shape (N, 3)");
            const auto n = points.shape(0);
            std::vector<Vec3f> pts(n);
            for (std::size_t i = 0; i < n; ++i) {
                pts[i] = Vec3f{points(i, 0), points(i, 1), points(i, 2)};
            }
            EvalResult out;
            b.evaluate(body, std::span<const Vec3f>(pts.data(), n), out);
            return out;
        });

    // ---- Display-pipeline normals from exact field gradients --------------
    //
    // Graphics sprint: the viewport's shading buffer. For every mesh
    // vertex p: n = normalize(body.sample(p).gradient) — the kernel's
    // EXACT analytic gradient (never finite differences). On a CSG min
    // the gradient is piecewise: smooth across curved regions, and it
    // picks ONE operand's direction at a live edge (never an average
    // across the crease) — the CAD look. This is strictly better than
    // the MC mesh normals, which interpolate the gradients of the two
    // GRID CORNERS flanking a crossing: near a crease those corners can
    // sit in different CSG regions and the lerp smears the edge dark.
    //
    // Degenerate vertices (|grad| ~ 0, e.g. a sphere's center) keep the
    // supplied mesh normal so the buffer NEVER contains NaN (a NaN
    // length fails the > threshold comparison and takes the same
    // fallback). Without mesh_normals, degenerate rows are (0,0,0) —
    // still never NaN.
    //
    // DISPLAY PIPELINE ONLY: the MC mesh itself (positions, indices,
    // mesh normals) is NOT touched — the exporters, the bridge and the
    // golden files consume mesh.normals and must stay byte-identical.
    m.def("display_normals",
        [](const SDFBody& body,
           nb::ndarray<float, nb::ndim<2>> positions,
           nb::object mesh_normals) {
            if (positions.shape(1) != 3)
                throw std::invalid_argument("positions must have shape (N, 3)");
            const auto n = positions.shape(0);

            // Optional (N,3) float32 fallback normals. Accepted states:
            // None / an EMPTY array ("no mesh normals") or an exact
            // (N,3) C-contiguous match — anything else is a caller bug
            // and fails loudly (never a silent misread of strided or
            // mistyped memory).
            const float* fb = nullptr;
            if (!mesh_normals.is_none()) {
                if (!nb::isinstance<nb::ndarray<nb::numpy, float, nb::ndim<2>>>(mesh_normals))
                    throw std::invalid_argument(
                        "mesh_normals must be a float32 numpy array of "
                        "shape (N, 3), or None");
                auto arr = nb::cast<nb::ndarray<nb::numpy, float, nb::ndim<2>>>(mesh_normals);
                if (arr.shape(0) == 0) {
                    // Empty: "no mesh normals available" — degenerate
                    // vertices will get (0,0,0) (still never NaN).
                } else if (arr.shape(0) == n && arr.shape(1) == 3
                           && arr.stride(0) == 3 && arr.stride(1) == 1) {
                    fb = arr.data();
                } else if (arr.shape(0) != n || arr.shape(1) != 3) {
                    throw std::invalid_argument(
                        "mesh_normals must have shape (N, 3) matching "
                        "positions (or be None/an empty array)");
                } else {
                    throw std::invalid_argument(
                        "mesh_normals must be C-contiguous");
                }
            }

            // Same guard constant as the kernel's own normal paths
            // (interp_gradient_abs, CylinderSDF's axis guard): below it a
            // direction is not defined.
            constexpr float kDegenerateGradient = 1e-9f;

            auto* out = new std::vector<float>();
            out->resize(std::size_t(n) * 3);
            for (std::size_t i = 0; i < n; ++i) {
                const Vec3f p{positions(i, 0), positions(i, 1), positions(i, 2)};
                const Vec3f g = body.sample(p).gradient;
                const float gl = g.length();
                Vec3f nv;
                if (gl > kDegenerateGradient) {
                    nv = g * (1.0f / gl);
                } else if (fb != nullptr) {
                    nv = Vec3f{fb[i * 3 + 0], fb[i * 3 + 1], fb[i * 3 + 2]};
                } else {
                    nv = Vec3f{0.0f, 0.0f, 0.0f};
                }
                (*out)[i * 3 + 0] = nv.x;
                (*out)[i * 3 + 1] = nv.y;
                (*out)[i * 3 + 2] = nv.z;
            }

            nb::capsule owner(out, [](void* p) noexcept {
                delete static_cast<std::vector<float>*>(p);
            });
            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                out->data(), {std::size_t(n), std::size_t(3)}, owner);
        },
        nb::arg("body"), nb::arg("positions"), nb::arg("mesh_normals").none() = nb::none(),
        "Display-only normal buffer from the EXACT SDF gradient sampled at "
        "every vertex position: n = normalize(body.sample(p).gradient). "
        "Vertices with a degenerate gradient (~0) keep the mesh normal "
        "(or (0,0,0) when none is supplied) — the buffer never contains "
        "NaN. The mesh itself is untouched: exports, the bridge and the "
        "goldens consume mesh.normals and stay byte-identical.");
}
