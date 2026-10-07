// core/math/src/predicate.cpp
//
// Wraps Shewchuk's robust predicates C implementation.
// The C source is vendored under third_party/robust_predicates/.
//
#include "CAD_0/math/predicate.hpp"

// We compile the C implementation as part of the C++ translation unit
// with `extern "C"` linkage. The implementation is public domain.
// The third_party/robust_predicates directory is added to the include
// path by core/math/CMakeLists.txt — we include the header directly.
extern "C" {
#include "predicates.c.h"
}

namespace CAD_0::math::predicate {

void initialize() noexcept {
    static bool initialized = false;
    if (!initialized) {
        exactinit();
        initialized = true;
    }
}

double orient2d(const Vec<2, double>& a, const Vec<2, double>& b, const Vec<2, double>& c) noexcept {
    return ::orient2d(const_cast<double*>(&a.x),
                      const_cast<double*>(&b.x),
                      const_cast<double*>(&c.x));
}

double orient2d(const Vec<2, float>& a, const Vec<2, float>& b, const Vec<2, float>& c) noexcept {
    double pa[2] = {a.x, a.y};
    double pb[2] = {b.x, b.y};
    double pc[2] = {c.x, c.y};
    return ::orient2d(pa, pb, pc);
}

double orient3d(const Vec<3, double>& a, const Vec<3, double>& b,
                const Vec<3, double>& c, const Vec<3, double>& d) noexcept {
    return ::orient3d(const_cast<double*>(&a.x),
                      const_cast<double*>(&b.x),
                      const_cast<double*>(&c.x),
                      const_cast<double*>(&d.x));
}

double orient3d(const Vec<3, float>& a, const Vec<3, float>& b,
                const Vec<3, float>& c, const Vec<3, float>& d) noexcept {
    double pa[3] = {a.x, a.y, a.z};
    double pb[3] = {b.x, b.y, b.z};
    double pc[3] = {c.x, c.y, c.z};
    double pd[3] = {d.x, d.y, d.z};
    return ::orient3d(pa, pb, pc, pd);
}

double incircle2d(const Vec<2, double>& a, const Vec<2, double>& b,
                  const Vec<2, double>& c, const Vec<2, double>& d) noexcept {
    return ::incircle(const_cast<double*>(&a.x),
                      const_cast<double*>(&b.x),
                      const_cast<double*>(&c.x),
                      const_cast<double*>(&d.x));
}

} // namespace CAD_0::math::predicate
