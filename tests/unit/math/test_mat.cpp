// tests/unit/math/test_mat.cpp
#include <doctest/doctest.h>

#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/quat.hpp"

using namespace CAD_0::math;

TEST_CASE("Mat4f: identity") {
    Mat4f m = Mat4f::identity();
    CHECK(m.transform_point({1, 2, 3}) == Vec3f{1, 2, 3});
    CHECK(m.transform_dir({1, 0, 0}) == Vec3f{1, 0, 0});
}

TEST_CASE("Mat4f: translation") {
    Mat4f t = Mat4f::translation({1, 2, 3});
    CHECK(t.transform_point({0, 0, 0}) == Vec3f{1, 2, 3});
    CHECK(t.transform_point({1, 1, 1}) == Vec3f{2, 3, 4});
    // Directions are not affected by translation.
    CHECK(t.transform_dir({1, 0, 0}) == Vec3f{1, 0, 0});
}

TEST_CASE("Mat4f: scaling") {
    Mat4f s = Mat4f::scaling({2, 3, 4});
    CHECK(s.transform_point({1, 1, 1}) == Vec3f{2, 3, 4});
}

TEST_CASE("Mat4f: rotation_z by 90 degrees rotates x to y") {
    const float pi_half = 3.14159265358979323846f / 2.0f;
    Mat4f r = Mat4f::rotation_z(pi_half);
    Vec3f out = r.transform_point({1, 0, 0});
    CHECK(out.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(out.y == doctest::Approx(1.0f).epsilon(1e-5f));
    CHECK(out.z == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("Mat4f: inverse_orthonormal for rigid transform") {
    // M = T * R (column-major: cols[3] = translation, cols[0..2] = rotation)
    // For a column-major rigid transform, M(p) = R(p) + t.
    // The inverse is M^-1(q) = R^T(q - t) = R^T * q - R^T * t.
    //
    // So M * M^-1 should be identity when applied as M^-1 * M (or vice versa,
    // the operator* is matrix composition, and inverse_orthonormal returns
    // M^-1 such that M^-1 * M = I and M * M^-1 = I).
    Mat4f t = Mat4f::translation({1, 2, 3}) * Mat4f::rotation_z(0.5f);
    Mat4f inv = t.inverse_orthonormal();
    // Test on a known point: M^-1(M(p)) should equal p.
    Vec3f p{5, 7, 11};
    Vec3f mp = t.transform_point(p);
    Vec3f recovered = inv.transform_point(mp);
    CHECK(recovered.x == doctest::Approx(p.x).epsilon(1e-5f));
    CHECK(recovered.y == doctest::Approx(p.y).epsilon(1e-5f));
    CHECK(recovered.z == doctest::Approx(p.z).epsilon(1e-5f));
}

TEST_CASE("Mat4f: inverse of scaling") {
    Mat4f s = Mat4f::scaling({2, 4, 8});
    Mat4f inv = s.inverse();
    CHECK(inv.transform_point({2, 4, 8}) == Vec3f{1, 1, 1});
}

TEST_CASE("Quatf: rotation by axis-angle matches matrix") {
    Quatf q = Quatf::from_axis_angle({0, 1, 0}, 0.5f);
    Vec3f v{1, 0, 0};
    Vec3f vq = q.rotate(v);
    Vec3f vm = q.to_matrix().transform_point(v);
    CHECK(vq.x == doctest::Approx(vm.x).epsilon(1e-5f));
    CHECK(vq.y == doctest::Approx(vm.y).epsilon(1e-5f));
    CHECK(vq.z == doctest::Approx(vm.z).epsilon(1e-5f));
}

TEST_CASE("Quatf: identity quaternion is identity transform") {
    Quatf q = Quatf::identity();
    Vec3f v{1, 2, 3};
    CHECK(q.rotate(v) == v);
}
