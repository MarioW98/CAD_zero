// core/sdf/src/mc33_process.hpp
//
// MC33 (Chernyaev 1995) cube processing — the dispatch logic that
// replaces the simple kTriangleTable lookup with the full MC33
// case dispatch + asymptotic decider + interior test.
//
// This is included by mesh_extract.cpp and uses the tables from
// mc33_tables.hpp.
//
#pragma once

#include "mc33_tables.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace CAD_0::sdf::mc33 {

// test_face and kCornerLookup are already defined in mc33_tables.hpp.
// We just need test_interior and process_cube here.

// Interior test — determines whether the interior of the cube is
// connected (case where the surface passes through the center).
inline bool test_interior(std::int8_t s, const float cube[8],
                           int mc_case, int config, int subconfig) {
    float t, At = 0, Bt = 0, Ct = 0, Dt = 0, a, b;
    int test = 0;
    int edge = -1;

    switch (mc_case) {
    case 4:
    case 10:
        a = (cube[4] - cube[0]) * (cube[6] - cube[2]) -
            (cube[7] - cube[3]) * (cube[5] - cube[1]);
        b = cube[2] * (cube[4] - cube[0]) + cube[0] * (cube[6] - cube[2]) -
            cube[1] * (cube[7] - cube[3]) - cube[3] * (cube[5] - cube[1]);
        if (std::abs(a) < std::numeric_limits<float>::epsilon())
            return s > 0;
        t = -b / (2.0f * a);
        if (t < 0 || t > 1) return s > 0;
        At = cube[0] + (cube[4] - cube[0]) * t;
        Bt = cube[3] + (cube[7] - cube[3]) * t;
        Ct = cube[2] + (cube[6] - cube[2]) * t;
        Dt = cube[1] + (cube[5] - cube[1]) * t;
        break;

    case 6:
        edge = kTest6[config][2];
        break;
    case 7:
        edge = kTest7[config][4];
        break;
    case 12:
        edge = kTest12[config][3];
        break;
    case 13:
        edge = kTiling13_5_1[config][subconfig][0];
        break;

    default:
        return s > 0;
    }

    if (mc_case == 6 || mc_case == 7 || mc_case == 12 || mc_case == 13) {
        switch (edge) {
        case 0:
            t = cube[0] / (cube[0] - cube[1]);
            Bt = cube[3] + (cube[2] - cube[3]) * t;
            Ct = cube[7] + (cube[6] - cube[7]) * t;
            Dt = cube[4] + (cube[5] - cube[4]) * t;
            break;
        case 1:
            t = cube[1] / (cube[1] - cube[2]);
            Bt = cube[0] + (cube[3] - cube[0]) * t;
            Ct = cube[4] + (cube[7] - cube[4]) * t;
            Dt = cube[5] + (cube[6] - cube[5]) * t;
            break;
        case 2:
            t = cube[2] / (cube[2] - cube[3]);
            Bt = cube[1] + (cube[0] - cube[1]) * t;
            Ct = cube[5] + (cube[4] - cube[5]) * t;
            Dt = cube[6] + (cube[7] - cube[6]) * t;
            break;
        case 3:
            t = cube[3] / (cube[3] - cube[0]);
            Bt = cube[2] + (cube[1] - cube[2]) * t;
            Ct = cube[6] + (cube[5] - cube[6]) * t;
            Dt = cube[7] + (cube[4] - cube[7]) * t;
            break;
        case 4:
            t = cube[4] / (cube[4] - cube[5]);
            Bt = cube[7] + (cube[6] - cube[7]) * t;
            Ct = cube[3] + (cube[2] - cube[3]) * t;
            Dt = cube[0] + (cube[1] - cube[0]) * t;
            break;
        case 5:
            t = cube[5] / (cube[5] - cube[6]);
            Bt = cube[4] + (cube[7] - cube[4]) * t;
            Ct = cube[0] + (cube[3] - cube[0]) * t;
            Dt = cube[1] + (cube[2] - cube[1]) * t;
            break;
        case 6:
            t = cube[6] / (cube[6] - cube[7]);
            Bt = cube[5] + (cube[4] - cube[5]) * t;
            Ct = cube[1] + (cube[0] - cube[1]) * t;
            Dt = cube[2] + (cube[3] - cube[2]) * t;
            break;
        case 7:
            t = cube[7] / (cube[7] - cube[4]);
            Bt = cube[6] + (cube[5] - cube[6]) * t;
            Ct = cube[2] + (cube[1] - cube[2]) * t;
            Dt = cube[3] + (cube[0] - cube[3]) * t;
            break;
        case 8:
            t = cube[0] / (cube[0] - cube[4]);
            Bt = cube[3] + (cube[7] - cube[3]) * t;
            Ct = cube[2] + (cube[6] - cube[2]) * t;
            Dt = cube[1] + (cube[5] - cube[1]) * t;
            break;
        case 9:
            t = cube[1] / (cube[1] - cube[5]);
            Bt = cube[0] + (cube[4] - cube[0]) * t;
            Ct = cube[3] + (cube[7] - cube[3]) * t;
            Dt = cube[2] + (cube[6] - cube[2]) * t;
            break;
        case 10:
            t = cube[2] / (cube[2] - cube[6]);
            Bt = cube[1] + (cube[5] - cube[1]) * t;
            Ct = cube[0] + (cube[4] - cube[0]) * t;
            Dt = cube[3] + (cube[7] - cube[3]) * t;
            break;
        case 11:
            t = cube[3] / (cube[3] - cube[7]);
            Bt = cube[2] + (cube[6] - cube[2]) * t;
            Ct = cube[1] + (cube[5] - cube[1]) * t;
            Dt = cube[0] + (cube[4] - cube[0]) * t;
            break;
        default:
            return s > 0;
        }
    }

    if (At >= 0) test++;
    if (Bt >= 0) test += 2;
    if (Ct >= 0) test += 4;
    if (Dt >= 0) test += 8;

    switch (test) {
    case 0: return s > 0;
    case 1: return s > 0;
    case 2: return s > 0;
    case 3: return s > 0;
    case 4: return s > 0;
    case 5:
        if (At * Ct - Bt * Dt < std::numeric_limits<float>::epsilon())
            return s > 0;
        break;
    case 6: return s > 0;
    case 7: return s < 0;
    case 8: return s > 0;
    case 9: return s > 0;
    case 10:
        if (At * Ct - Bt * Dt >= std::numeric_limits<float>::epsilon())
            return s > 0;
        break;
    case 11: return s < 0;
    case 12: return s > 0;
    case 13: return s < 0;
    case 14: return s < 0;
    case 15: return s < 0;
    }
    return s < 0;
}

// Result of process_cube: a list of triangles (each = 3 edge indices,
// where 0..11 are standard edges and 12 means "cube-center vertex v12").
struct ProcessResult {
    static constexpr int MAX_TRIS = 12;
    int edges[MAX_TRIS * 3];  // up to 12 triangles × 3 edges
    int num_tris = 0;
    bool needs_v12 = false;   // true if any triangle uses edge index 12
};

// Process a cube using MC33.
// cube[8] = SDF values at the 8 corners (corner 0 at (cx,cy,cz), etc.).
// cube_index = 8-bit sign pattern.
// Returns the list of triangles as edge indices.
inline ProcessResult process_cube(const float cube[8], std::uint32_t cube_index) {
    ProcessResult result;

    const int mc_case = kCases[cube_index][0];
    const int config = kCases[cube_index][1];
    int subconfig = 0;

    auto add_tris = [&](const std::int8_t* tri, int n) {
        for (int i = 0; i < n * 3 && result.num_tris < ProcessResult::MAX_TRIS; ++i) {
            result.edges[result.num_tris * 3 + i % 3] = tri[i];
            if (tri[i] == 12) result.needs_v12 = true;
            if (i % 3 == 2) result.num_tris++;
        }
    };

    auto add_tris_v12 = [&](const std::int8_t* tri, int n) {
        result.needs_v12 = true;
        for (int i = 0; i < n * 3 && result.num_tris < ProcessResult::MAX_TRIS; ++i) {
            result.edges[result.num_tris * 3 + i % 3] = tri[i];
            if (i % 3 == 2) result.num_tris++;
        }
    };

    switch (mc_case) {
    case 0: break;
    case 1: add_tris(kTiling1[config], 1); break;
    case 2: add_tris(kTiling2[config], 2); break;
    case 3:
        if (test_face(kTest3[config], cube))
            add_tris(kTiling3_2[config], 4);
        else
            add_tris(kTiling3_1[config], 2);
        break;
    case 4:
        if (test_interior(kTest4[config], cube, mc_case, config, subconfig))
            add_tris(kTiling4_1[config], 2);
        else
            add_tris(kTiling4_2[config], 6);
        break;
    case 5: add_tris(kTiling5[config], 3); break;
    case 6:
        if (test_face(kTest6[config][0], cube))
            add_tris(kTiling6_2[config], 5);
        else {
            if (test_interior(kTest6[config][1], cube, mc_case, config, subconfig))
                add_tris(kTiling6_1_1[config], 3);
            else
                add_tris_v12(kTiling6_1_2[config], 9);
        }
        break;
    case 7: {
        if (test_face(kTest7[config][0], cube)) subconfig += 1;
        if (test_face(kTest7[config][1], cube)) subconfig += 2;
        if (test_face(kTest7[config][2], cube)) subconfig += 4;
        switch (subconfig) {
        case 0: add_tris(kTiling7_1[config], 3); break;
        case 1: add_tris(kTiling7_2[config][0], 5); break;
        case 2: add_tris(kTiling7_2[config][1], 5); break;
        case 3: add_tris_v12(kTiling7_3[config][0], 9); break;
        case 4: add_tris(kTiling7_2[config][2], 5); break;
        case 5: add_tris_v12(kTiling7_3[config][1], 9); break;
        case 6: add_tris_v12(kTiling7_3[config][2], 9); break;
        case 7:
            if (test_interior(kTest7[config][3], cube, mc_case, config, subconfig))
                add_tris_v12(kTiling7_4_2[config], 9);
            else
                add_tris(kTiling7_4_1[config], 5);
            break;
        }
        break;
    }
    case 8: add_tris(kTiling8[config], 2); break;
    case 9: add_tris(kTiling9[config], 4); break;
    case 10:
        if (test_face(kTest10[config][0], cube)) {
            if (test_face(kTest10[config][1], cube))
                add_tris(kTiling10_1_1_[config], 4);
            else
                add_tris_v12(kTiling10_2[config], 8);
        } else {
            if (test_face(kTest10[config][1], cube)) {
                add_tris_v12(kTiling10_2_[config], 8);
            } else {
                if (test_interior(kTest10[config][2], cube, mc_case, config, subconfig))
                    add_tris(kTiling10_1_1[config], 4);
                else
                    add_tris(kTiling10_1_2[config], 8);
            }
        }
        break;
    case 11: add_tris(kTiling11[config], 4); break;
    case 12:
        if (test_face(kTest12[config][0], cube)) {
            if (test_face(kTest12[config][1], cube))
                add_tris(kTiling12_1_1_[config], 4);
            else
                add_tris_v12(kTiling12_2[config], 8);
        } else {
            if (test_face(kTest12[config][1], cube)) {
                add_tris_v12(kTiling12_2_[config], 8);
            } else {
                if (test_interior(kTest12[config][2], cube, mc_case, config, subconfig))
                    add_tris(kTiling12_1_1[config], 4);
                else
                    add_tris(kTiling12_1_2[config], 8);
            }
        }
        break;
    case 13: {
        if (test_face(kTest13[config][0], cube)) subconfig += 1;
        if (test_face(kTest13[config][1], cube)) subconfig += 2;
        if (test_face(kTest13[config][2], cube)) subconfig += 4;
        if (test_face(kTest13[config][3], cube)) subconfig += 8;
        if (test_face(kTest13[config][4], cube)) subconfig += 16;
        if (test_face(kTest13[config][5], cube)) subconfig += 32;
        int sc = kSubconfig13[subconfig];
        switch (sc) {
        case 0: add_tris(kTiling13_1[config], 4); break;
        case 1: case 2: case 3: case 4: case 5: case 6:
            add_tris(kTiling13_2[config][sc-1], 6); break;
        case 7: case 8: case 9: case 10: case 11: case 12:
        case 13: case 14: case 15: case 16: case 17: case 18:
            add_tris_v12(kTiling13_3[config][sc-7], 10); break;
        case 19: case 20: case 21: case 22:
            add_tris_v12(kTiling13_4[config][sc-19], 12); break;
        case 23: case 24: case 25: case 26: {
            int sub = sc - 23;
            if (test_interior(kTest13[config][6], cube, mc_case, config, sub))
                add_tris(kTiling13_5_1[config][sub], 6);
            else
                add_tris_v12(kTiling13_5_2[config][sub], 10);
            break;
        }
        case 27: case 28: case 29: case 30: case 31: case 32:
        case 33: case 34: case 35: case 36: case 37: case 38:
            add_tris_v12(kTiling13_3_[config][sc-27], 10); break;
        case 39: case 40: case 41: case 42: case 43: case 44:
            add_tris(kTiling13_2_[config][sc-39], 6); break;
        case 45: add_tris(kTiling13_1_[config], 4); break;
        default: break;
        }
        break;
    }
    case 14: add_tris(kTiling14[config], 4); break;
    default: break;
    }

    return result;
}

} // namespace CAD_0::sdf::mc33
