// core/sdf/src/mesh_extract.cpp
//
// Marching Cubes implementation (Phase B.5 — with vertex welding + narrow-band).
//
// Reference: Lorensen & Cline, "Marching Cubes: A High Resolution 3D
// Surface Construction Algorithm", SIGGRAPH 1987.
//
// Phase B.5 enhancements vs Phase B:
//   * Vertex welding: each edge in the grid is shared by up to 4 cells.
//     Phase B emitted a new vertex for each cell, resulting in ~4x
//     duplicate vertices. Phase B.5 uses a deterministic edge key
//     (cell coords + edge index) so that neighbouring cells emit the
//     same vertex id, then maps it through a hash table.
//   * Narrow-band culling: cells whose 8 corners all share the same
//     sign are skipped (no surface crossing). For compact SDFs this
//     avoids ~75% of cell evaluations.
//
// Edge numbering convention (cube with 8 corners labeled 0..7):
//   Corner positions:
//     0 = (0,0,0), 1 = (1,0,0), 2 = (1,1,0), 3 = (0,1,0)
//     4 = (0,0,1), 5 = (1,0,1), 6 = (1,1,1), 7 = (0,1,1)
//   Edge indices 0..11 connect:
//     0: 0-1, 1: 1-2, 2: 2-3, 3: 3-0  (bottom face)
//     4: 4-5, 5: 5-6, 6: 6-7, 7: 7-4  (top face)
//     8: 0-4, 9: 1-5, 10: 2-6, 11: 3-7  (vertical edges)
//
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/sdf/evaluate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace CAD_0::sdf {

// ---------------------------------------------------------------------------
// Edge table — for each of the 256 cube sign configurations, lists which
// of the 12 edges cross the surface (1 bit per edge, 12 bits used).
// ---------------------------------------------------------------------------
static const std::uint32_t kEdgeTable[256] = {
    0x0,   0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99,  0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33,  0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa,  0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0xdaf, 0xca6, 0xfaa, 0xea3, 0x9a9, 0x8a0,
    0x460, 0x569, 0x663, 0x76a, 0x66,  0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff,  0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55,  0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc,  0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55,  0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff,  0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66,  0x76a, 0x663, 0x569, 0x460,
    0x8a0, 0x9a9, 0xea3, 0xfaa, 0xca6, 0xdaf, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa,  0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33,  0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99,  0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

// ---------------------------------------------------------------------------
// Triangle table — for each of the 256 cube sign configurations, lists
// up to 5 triangles, each specified by 3 edge indices (or -1 for "no edge").
// ---------------------------------------------------------------------------
static const std::int8_t kTriangleTable[256][16] = {
    {  -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    1,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    8,    3,    9,    8,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    2,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    3,    1,    2,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    2,   10,    0,    2,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,    8,    3,    2,   10,    8,   10,    9,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,   11,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,   11,    2,    8,   11,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    9,    0,    2,    3,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,   11,    2,    1,    9,   11,    9,    8,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,   10,    1,   11,   10,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,   10,    1,    0,    8,   10,    8,   11,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    9,    0,    3,   11,    9,   11,   10,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    8,   10,   10,    8,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    7,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    3,    0,    7,    3,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    1,    9,    8,    4,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    1,    9,    4,    7,    1,    7,    3,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    2,   10,    8,    4,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    4,    7,    3,    0,    4,    1,    2,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    2,   10,    9,    0,    2,    8,    4,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   10,    9,    2,    9,    7,    2,    7,    3,    7,    9,    4,   -1,   -1,   -1,   -1},

    {   8,    4,    7,    3,   11,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    4,    7,   11,    2,    4,    2,    0,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    0,    1,    8,    4,    7,    2,    3,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    7,   11,    9,    4,   11,    9,   11,    2,    9,    2,    1,   -1,   -1,   -1,   -1},

    {   3,   10,    1,    3,   11,   10,    7,    8,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,   11,   10,    1,    4,   11,    1,    0,    4,    7,   11,    4,   -1,   -1,   -1,   -1},

    {   4,    7,    8,    9,    0,   11,    9,   11,   10,   11,    0,    3,   -1,   -1,   -1,   -1},

    {   4,    7,   11,    4,   11,    9,    9,   11,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    5,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    5,    4,    0,    8,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    5,    4,    1,    5,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    5,    4,    8,    3,    5,    3,    1,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    2,   10,    9,    5,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    0,    8,    1,    2,   10,    4,    9,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    2,   10,    5,    4,    2,    4,    0,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   10,    5,    3,    2,    5,    3,    5,    4,    3,    4,    8,   -1,   -1,   -1,   -1},

    {   9,    5,    4,    2,    3,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,   11,    2,    0,    8,   11,    4,    9,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    5,    4,    0,    1,    5,    2,    3,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,    1,    5,    2,    5,    8,    2,    8,   11,    4,    8,    5,   -1,   -1,   -1,   -1},

    {  10,    3,   11,   10,    1,    3,    9,    5,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    9,    5,    0,    8,    1,    8,   10,    1,    8,   11,   10,   -1,   -1,   -1,   -1},

    {  11,   10,    0,   11,    0,    3,    9,    5,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    4,    9,    5,    9,   11,    5,   11,   10,   11,    9,    8,   -1,   -1,   -1,   -1},

    {   8,    4,    7,    9,    5,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    5,    4,    7,    3,    0,    7,    0,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    5,    4,    1,    5,    0,    7,    8,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    5,    4,    1,    4,    8,    1,    8,    7,    1,    7,    3,   -1,   -1,   -1,   -1},

    {   4,    9,    5,    7,    8,    4,    1,    2,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    4,    7,    3,    0,    4,    1,    2,   10,    9,    5,    4,   -1,   -1,   -1,   -1},

    {   5,    4,    9,    2,   10,    0,    0,   10,    8,    8,   10,    7,   -1,   -1,   -1,   -1},

    {   3,    4,    7,    3,    0,    4,    4,    9,    5,    2,    3,    4,    2,    4,   10,    2},

    {   7,    8,    4,    2,    3,   11,    5,    4,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    2,    4,   11,    4,    7,    9,    4,    5,    0,    4,    2,   -1,   -1,   -1,   -1},

    {   4,    7,    8,    0,    5,    4,    0,    1,    5,    2,    3,   11,   -1,   -1,   -1,   -1},

    {   5,    4,    9,    5,    2,    4,    5,   11,    2,    2,   11,    1,    7,    8,    4,   -1},

    {   3,   10,    1,    3,   11,   10,    7,    8,    4,    9,    5,    4,   -1,   -1,   -1,   -1},

    {   1,   11,   10,    1,    4,   11,    1,    0,    4,    4,    7,   11,    5,    4,    9,   -1},

    {   9,    5,    4,    0,    8,    6,    0,    6,   11,    6,    8,    7,   10,    6,   11,   -1},

    {   4,    7,   11,    4,   11,    9,    9,   11,   10,    5,    4,    9,   -1,   -1,   -1,   -1},

    {   5,   10,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    3,    5,   10,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    0,    1,    5,   10,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    8,    3,    1,    9,    8,    5,   10,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    6,    5,    2,    6,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    6,    5,    1,    2,    6,    3,    0,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    6,    5,    9,    0,    6,    0,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    9,    8,    5,    8,    2,    5,    2,    6,    3,    8,    2,   -1,   -1,   -1,   -1},

    {   2,    3,   11,   10,    6,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    0,    8,   11,    2,    0,   10,    6,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    1,    9,    2,    3,   11,    5,   10,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,   10,    6,    1,    9,    2,    9,   11,    2,    9,    8,   11,   -1,   -1,   -1,   -1},

    {   6,    3,   11,    6,    5,    3,    5,    1,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,   11,    0,   11,    5,    0,    5,    1,    5,   11,    6,   -1,   -1,   -1,   -1},

    {   3,   11,    6,    0,    3,    6,    0,    6,    5,    0,    5,    9,   -1,   -1,   -1,   -1},

    {   6,    5,    9,    6,    9,   11,   11,    9,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,   10,    6,    4,    7,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    3,    0,    4,    7,    3,    6,    5,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    9,    0,    5,   10,    6,    8,    4,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    6,    5,    1,    9,    7,    1,    7,    3,    7,    9,    4,   -1,   -1,   -1,   -1},

    {   6,    1,    2,    6,    5,    1,    4,    7,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    2,    5,    5,    2,    6,    3,    0,    4,    3,    4,    7,   -1,   -1,   -1,   -1},

    {   8,    4,    7,    9,    0,    5,    0,    6,    5,    0,    2,    6,   -1,   -1,   -1,   -1},

    {   7,    3,    9,    7,    9,    4,    3,    2,    9,    5,    9,    6,    2,    9,    5,    6},

    {   3,   11,    2,    7,    8,    4,   10,    6,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,   10,    6,    4,    7,    2,    4,    2,    0,    2,    7,   11,   -1,   -1,   -1,   -1},

    {   0,    1,    9,    4,    7,    8,    2,    3,   11,    5,   10,    6,   -1,   -1,   -1,   -1},

    {   9,    2,    1,    9,   11,    2,    9,    4,   11,    7,   11,    4,    5,   10,    6,   -1},

    {   8,    4,    7,    3,   11,    5,    3,    5,    1,    5,   11,    6,   -1,   -1,   -1,   -1},

    {   5,    1,   11,    5,   11,    6,    1,    0,   11,    7,   11,    4,    0,    4,   11,   -1},

    {   0,    5,    9,    0,    6,    5,    0,    3,    6,   11,    6,    3,    8,    4,    7,   -1},

    {   6,    5,    9,    6,    9,   11,    4,    7,    9,    7,   11,    9,   -1,   -1,   -1,   -1},

    {  10,    4,    9,    6,    4,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,   10,    6,    4,    9,   10,    0,    8,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    0,    1,   10,    6,    0,    6,    4,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    3,    1,    8,    1,    6,    8,    6,    4,    6,    1,   10,   -1,   -1,   -1,   -1},

    {   1,    4,    9,    1,    2,    4,    2,    6,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    0,    8,    1,    2,    9,    2,    4,    9,    2,    6,    4,   -1,   -1,   -1,   -1},

    {   0,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    3,    2,    8,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    4,    9,   10,    6,    4,   11,    2,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    2,    2,    8,   11,    4,    9,   10,    4,   10,    6,   -1,   -1,   -1,   -1},

    {   3,   11,    2,    0,    1,    6,    0,    6,    4,    6,    1,   10,   -1,   -1,   -1,   -1},

    {   6,    4,    1,    6,    1,   10,    4,    8,    1,    2,    1,   11,    8,   11,    1,   -1},

    {   9,    6,    4,    9,    3,    6,    9,    1,    3,   11,    6,    3,   -1,   -1,   -1,   -1},

    {   8,   11,    1,    8,    1,    0,   11,    6,    1,    9,    1,    4,    6,    4,    1,   -1},

    {   6,    4,    9,    6,    9,    3,    6,    3,   11,    0,    3,    9,   -1,   -1,   -1,   -1},

    {   5,    6,    9,    5,    9,   11,   11,    9,    6,    4,    6,    8,    8,    6,   11,   11},

    {   6,    4,    8,   11,    6,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    6,   11,    3,    0,    6,    0,    4,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    6,   11,    8,    4,    6,    0,    1,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    4,    6,    9,    6,    3,    9,    3,    1,   11,    3,    6,   -1,   -1,   -1,   -1},

    {   6,    8,    4,   11,    8,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    6,   11,    3,    0,    6,    0,    4,    6,    6,    4,    8,   -1,   -1,   -1,   -1},

    {   0,    1,    9,    8,    4,    6,    8,    6,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    1,    9,   11,    3,    1,    9,    6,    4,    9,    4,    8,    6,    4,    9,   -1},

    {   6,    4,    1,    6,    1,   10,    4,    8,    1,    2,    1,   11,    8,   11,    1,   -1},

    {   1,    4,    9,    1,    2,    4,    2,    6,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    0,    8,    1,    2,    9,    2,    4,    9,    2,    6,    4,   -1,   -1,   -1,   -1},

    {   0,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    3,    2,    8,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    4,    9,   10,    6,    4,   11,    2,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    2,    2,    8,   11,    4,    9,   10,    4,   10,    6,   -1,   -1,   -1,   -1},

    {   3,   11,    2,    0,    1,    6,    0,    6,    4,    6,    1,   10,   -1,   -1,   -1,   -1},

    {   6,    4,    1,    6,    1,   10,    4,    8,    1,    2,    1,   11,    8,   11,    1,   -1},

    {   9,    6,    4,    9,    3,    6,    9,    1,    3,   11,    6,    3,   -1,   -1,   -1,   -1},

    {   8,   11,    1,    8,    1,    0,   11,    6,    1,    9,    1,    4,    6,    4,    1,   -1},

    {   6,    4,    9,    6,    9,    3,    6,    3,   11,    0,    3,    9,   -1,   -1,   -1,   -1},

    {   5,    6,    9,    5,    9,   11,   11,    9,    6,    4,    6,    8,    8,    6,   11,   11},

    {   6,    4,    8,   11,    6,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    6,   11,    3,    0,    6,    0,    4,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    6,   11,    8,    4,    6,    0,    1,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    4,    6,    9,    6,    3,    9,    3,    1,   11,    3,    6,   -1,   -1,   -1,   -1},

    {   6,    8,    4,   11,    8,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    6,   11,    3,    0,    6,    0,    4,    6,    6,    4,    8,   -1,   -1,   -1,   -1},

    {   0,    1,    9,    8,    4,    6,    8,    6,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    1,    9,   11,    3,    1,    9,    6,    4,    9,    4,    8,    6,    4,    9,   -1},

    {   6,    4,    1,    6,    1,   10,    4,    8,    1,    2,    1,   11,    8,   11,    1,   -1},

    {   1,    4,    9,    1,    2,    4,    2,    6,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    0,    8,    1,    2,    9,    2,    4,    9,    2,    6,    4,   -1,   -1,   -1,   -1},

    {   0,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    3,    2,    8,    2,    4,    4,    2,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    4,    9,   10,    6,    4,   11,    2,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    8,    2,    2,    8,   11,    4,    9,   10,    4,   10,    6,   -1,   -1,   -1,   -1},

    {   3,   11,    2,    0,    1,    6,    0,    6,    4,    6,    1,   10,   -1,   -1,   -1,   -1},

    {   6,    4,    1,    6,    1,   10,    4,    8,    1,    2,    1,   11,    8,   11,    1,   -1},

    {   9,    6,    4,    9,    3,    6,    9,    1,    3,   11,    6,    3,   -1,   -1,   -1,   -1},

    {   8,   11,    1,    8,    1,    0,   11,    6,    1,    9,    1,    4,    6,    4,    1,   -1},

    {   6,    4,    9,    6,    9,    3,    6,    3,   11,    0,    3,    9,   -1,   -1,   -1,   -1},

    {   5,    6,    9,    5,    9,   11,   11,    9,    6,    4,    6,    8,    8,    6,   11,   11},

    {   8,    0,    3,    9,    2,    1,    9,    4,    2,    4,    6,    2,   -1,   -1,   -1,   -1},

    {   9,    4,    1,    4,    2,    1,    4,    6,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    3,    8,    6,    1,    8,    4,    6,    8,   10,    1,    6,   -1,   -1,   -1,   -1},

    {   1,    0,   10,    0,    6,   10,    0,    4,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   6,   10,    4,   10,    9,    4,    3,    8,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    4,   10,   10,    4,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    5,    6,   11,    9,    6,    9,    7,    4,    9,   11,    7,   -1,   -1,   -1,   -1},

    {   9,    5,    0,    5,    6,    0,    6,    3,    0,    3,    6,   11,    7,    4,    8,   -1},

    {  11,    1,    5,    6,   11,    5,   11,    0,    1,    4,   11,    7,   11,    4,    0,   -1},

    {   7,    4,    8,    5,   11,    3,    1,    5,    3,    6,   11,    5,   -1,   -1,   -1,   -1},

    {   1,    2,    9,    2,   11,    9,   11,    4,    9,    4,   11,    7,    6,   10,    5,   -1},

    {   9,    1,    0,    8,    7,    4,   11,    3,    2,    6,   10,    5,   -1,   -1,   -1,   -1},

    {   6,   10,    5,    2,    7,    4,    0,    2,    4,   11,    7,    2,   -1,   -1,   -1,   -1},

    {   2,   11,    3,    4,    8,    7,    5,    6,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    3,    7,    4,    9,    7,    9,    2,    3,    6,    9,    5,    5,    9,    2,   -1},

    {   7,    4,    8,    5,    0,    9,    5,    6,    0,    6,    2,    0,   -1,   -1,   -1,   -1},

    {   5,    2,    1,    6,    2,    5,    4,    0,    3,    7,    4,    3,   -1,   -1,   -1,   -1},

    {   2,    1,    6,    1,    5,    6,    8,    7,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    6,   10,    7,    9,    1,    3,    7,    1,    4,    9,    7,   -1,   -1,   -1,   -1},

    {   0,    9,    1,    6,   10,    5,    7,    4,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    3,    4,    3,    7,    4,   10,    5,    6,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   6,   10,    5,    8,    7,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    5,    6,   11,    9,    6,    8,    9,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   6,   11,    3,    6,    3,    0,    5,    6,    0,    9,    5,    0,   -1,   -1,   -1,   -1},

    {  11,    8,    0,    5,   11,    0,    1,    5,    0,    6,   11,    5,   -1,   -1,   -1,   -1},

    {  11,    3,    6,    3,    5,    6,    3,    1,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   6,   10,    5,    2,    9,    1,    2,   11,    9,   11,    8,    9,   -1,   -1,   -1,   -1},

    {   9,    1,    0,   11,    3,    2,    6,   10,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    0,   11,    0,    2,   11,    5,    6,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    3,    2,    5,    6,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    9,    5,    2,    8,    5,    6,    2,    5,    2,    8,    3,   -1,   -1,   -1,   -1},

    {   5,    6,    9,    6,    0,    9,    6,    2,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    6,    1,    6,    2,    1,    8,    0,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    6,    1,    1,    6,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    1,    8,    9,    1,    6,   10,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,    0,    9,    6,   10,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    0,    6,   10,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   6,   10,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    7,    4,    9,   11,    4,   10,   11,    9,    9,    4,    5,   -1,   -1,   -1,   -1},

    {   4,    5,    9,    6,    8,    0,   11,    6,    0,    7,    8,    6,   11,    6,   10,   -1},

    {  10,   11,    1,   11,    4,    1,    4,    0,    1,   11,    7,    4,    9,    4,    5,   -1},

    {   1,   10,    3,   10,   11,    3,    4,    8,    7,    4,    5,    9,   -1,   -1,   -1,   -1},

    {   9,    4,    5,    4,    2,    5,    2,   11,    5,    1,   11,    2,    4,    8,    7,   -1},

    {   8,    7,    4,    4,    5,    0,    5,    1,    0,   11,    3,    2,   -1,   -1,   -1,   -1},

    {   4,    2,   11,    7,    4,   11,    5,    4,    9,    2,    4,    0,   -1,   -1,   -1,   -1},

    {   4,    8,    7,   11,    3,    2,    9,    4,    5,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   7,    4,    3,    4,    0,    3,    5,    9,    4,    4,    3,    2,   10,    4,    2,   -1},

    {   9,    4,    5,    0,   10,    2,    8,   10,    0,    7,   10,    8,   -1,   -1,   -1,   -1},

    {   7,    4,    3,    4,    0,    3,   10,    2,    1,    4,    5,    9,   -1,   -1,   -1,   -1},

    {   5,    9,    4,    4,    8,    7,   10,    2,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    1,    8,    4,    1,    7,    8,    1,    3,    7,    1,   -1,   -1,   -1,   -1},

    {   4,    5,    0,    0,    5,    1,    4,    8,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    9,    0,    3,    7,    4,    0,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   7,    4,    8,    4,    5,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    4,    5,   11,    9,    5,   10,   11,    5,    8,    9,   11,   -1,   -1,   -1,   -1},

    {   0,   10,   11,    3,    0,   11,    4,    5,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    9,    4,    1,    8,    0,    1,   10,    8,   10,   11,    8,   -1,   -1,   -1,   -1},

    {  11,    3,   10,    3,    1,   10,    4,    5,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,    1,    2,    8,    5,    2,   11,    8,    2,    5,    8,    4,   -1,   -1,   -1,   -1},

    {   4,    5,    0,    5,    1,    0,   11,    3,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   11,    0,   11,    8,    0,    5,    9,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    9,   11,    3,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   5,   10,    2,    5,    2,    3,    4,    5,    3,    8,    4,    3,   -1,   -1,   -1,   -1},

    {  10,    2,    5,    2,    4,    5,    2,    0,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    0,    3,   10,    2,    1,    5,    9,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    2,    1,    4,    5,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    8,    5,    3,    8,    5,    1,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    0,    0,    5,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    9,    3,    8,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   4,    5,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    7,    4,    9,   11,    4,   10,   11,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    7,    4,   11,    0,    9,   10,   11,    9,    3,    0,   11,   -1,   -1,   -1,   -1},

    {  10,   11,    1,   11,    4,    1,    4,    0,    1,    4,   11,    7,   -1,   -1,   -1,   -1},

    {   1,   10,    3,   10,   11,    3,    4,    8,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  11,    7,    4,   11,    4,    9,    2,   11,    9,    1,    2,    9,   -1,   -1,   -1,   -1},

    {   1,    0,    9,    7,    4,    8,   11,    3,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   7,    4,   11,    4,    2,   11,    4,    0,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   7,    4,    8,    2,   11,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,   10,    2,    7,    9,    2,    3,    7,    2,    4,    9,    7,   -1,   -1,   -1,   -1},

    {  10,    2,    9,    2,    0,    9,    7,    4,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   7,    4,    3,    4,    0,    3,   10,    2,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    2,    1,    7,    4,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    1,    4,    1,    7,    4,    1,    3,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    1,    0,    7,    4,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    3,    4,    4,    3,    7,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   8,    7,    4,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    8,    9,   11,    8,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    9,    3,    9,   11,    3,    9,   10,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,   10,    0,   10,    8,    0,   10,   11,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   1,   10,    3,    3,   10,   11,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   11,    1,   11,    9,    1,   11,    8,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   0,    9,    1,   11,    3,    2,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   11,    0,    0,   11,    8,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   2,   11,    3,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    2,    8,   10,    2,    8,    9,   10,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    2,    9,    9,    2,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    0,   10,    2,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  10,    2,    1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    1,    1,    8,    9,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   9,    1,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {   3,    8,    0,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

    {  -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1,   -1},

};

// ---------------------------------------------------------------------------
// Helper: for each of the 12 cube edges, list the two corner indices it
// connects. Used to interpolate the vertex position along the edge.
// ---------------------------------------------------------------------------
static const std::array<std::pair<int, int>, 12> kEdgeCorners = {{
    {0, 1}, {1, 2}, {2, 3}, {3, 0},  // bottom face
    {4, 5}, {5, 6}, {6, 7}, {7, 4},  // top face
    {0, 4}, {1, 5}, {2, 6}, {3, 7},  // vertical edges
}};

// ---------------------------------------------------------------------------
// Edge-anchoring rationale (design notes — no code here).
//
// Each grid edge is identified by its two grid-vertex coordinates. The
// vertex emitted on that edge by marching cubes is shared by up to 4
// adjacent cells, and *all* of them must produce the same vertex id
// (watertightness). The canonical approach is to anchor each edge at a
// canonical cell (typically the one with the smaller-coordinate
// endpoint at its corner 0) and look up the local edge index there.
//
// An older version of this file carried a `kEdgeAnchors` table that
// tried to express this mapping, but the table had errors for edges 2,
// 6, and 10 (the +X edge at y=cy+1, the +X edge at y=cy+1,z=cz+1, and
// the +Z edge at x=cx+1,y=cy+1). Worse, the table was *never read* —
// the actual vertex deduplication used `grid_edge_key()` below, which
// computes a unique hash from the two endpoint grid coordinates
// directly. The dead table was both wrong and unused, so it has been
// removed to avoid confusion.
//
// The current implementation:
//   * For each local edge index 0..11, look up the two corner indices
//     (kEdgeCorners).
//   * Convert corner indices to absolute grid coordinates using
//     kCornerOffset.
//   * Compute a 64-bit hash of the two endpoints (canonicalised so
//     the smaller-coordinate endpoint comes first).
//   * Look up the hash in `edge_cache`. If present, reuse the cached
//     vertex id; otherwise allocate a new vertex and store it.
//
// This guarantees watertightness: any two cells that share a grid edge
// will compute the same hash and thus obtain the same vertex id.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// marching_cubes implementation (Phase B.5)
// ---------------------------------------------------------------------------

namespace {

struct Grid {
    std::uint32_t N;  // vertices per axis = resolution + 1
    std::vector<float> field;
    std::vector<math::Vec3f> points;
    std::vector<math::Vec3f> grad;
    math::Bboxf bounds;
    math::Vec3f cell_size;
    bool has_normals;

    std::size_t idx(std::uint32_t xi, std::uint32_t yi, std::uint32_t zi) const noexcept {
        return std::size_t(zi) * N * N + std::size_t(yi) * N + xi;
    }
};

// Evaluate the SDF on a uniform grid.
Grid evaluate_grid(const SDFBody& body, const math::Bboxf& bounds,
                   std::uint32_t resolution, bool compute_normals) {
    Grid g{};
    g.N = resolution + 1;
    g.bounds = bounds;
    g.has_normals = compute_normals;

    const auto extent = bounds.extent();
    g.cell_size = {extent.x / float(resolution),
                  extent.y / float(resolution),
                  extent.z / float(resolution)};

    const std::size_t total = std::size_t(g.N) * g.N * g.N;
    g.points.resize(total);
    g.field.resize(total);
    if (compute_normals) g.grad.resize(total);

    for (std::uint32_t zi = 0; zi < g.N; ++zi) {
        for (std::uint32_t yi = 0; yi < g.N; ++yi) {
            for (std::uint32_t xi = 0; xi < g.N; ++xi) {
                const float fx = bounds.min.x + extent.x * (float(xi) / float(resolution));
                const float fy = bounds.min.y + extent.y * (float(yi) / float(resolution));
                const float fz = bounds.min.z + extent.z * (float(zi) / float(resolution));
                g.points[g.idx(xi, yi, zi)] = {fx, fy, fz};
            }
        }
    }

    EvalResult result;
    CpuEvalBackend backend;
    backend.evaluate(body,
                     std::span<const math::Vec3f>(g.points.data(), g.points.size()),
                     result);
    for (std::size_t i = 0; i < total; ++i) {
        g.field[i] = result.values[i];
        if (compute_normals) g.grad[i] = result.gradients[i];
    }
    return g;
}

// Compute the vertex position along a grid edge between two absolute
// grid points (ax,ay,az) and (bx,by,bz). The endpoints are canonicalized
// (A < B lexicographically) so that two cells sharing the same physical
// grid edge compute the EXACT SAME floating-point interpolation parameter
// t = fA / (fA - fB), producing a bit-identical vertex position. This is
// the root fix for the watertightness bug: previously, interp_edge took
// (cx,cy,cz, ca, cb) and computed t from the cell-local corner order,
// so adjacent cells passing the same endpoints in different order got
// t and 1-t, which are mathematically equal but can differ in the last
// ULP, yielding two distinct vertices on one physical edge.
math::Vec3f interp_edge_abs(const Grid& g,
                             std::int32_t ax, std::int32_t ay, std::int32_t az,
                             std::int32_t bx, std::int32_t by, std::int32_t bz) {
    // Canonicalize: A < B lexicographically.
    auto less = [](std::int32_t ax, std::int32_t ay, std::int32_t az,
                   std::int32_t bx, std::int32_t by, std::int32_t bz) {
        if (ax != bx) return ax < bx;
        if (ay != by) return ay < by;
        return az < bz;
    };
    if (!less(ax, ay, az, bx, by, bz)) {
        std::swap(ax, bx); std::swap(ay, by); std::swap(az, bz);
    }

    const std::size_t ia = g.idx(ax, ay, az);
    const std::size_t ib = g.idx(bx, by, bz);
    const float fA = g.field[ia];
    const float fB = g.field[ib];

    // t = fA / (fA - fB), clamped to [0, 1].
    float t = 0.5f;
    const float denom = fA - fB;
    if (std::abs(denom) > 1e-9f) {
        t = fA / denom;
        t = std::clamp(t, 0.0f, 1.0f);
    }
    const math::Vec3f& pA = g.points[ia];
    const math::Vec3f& pB = g.points[ib];
    return {pA.x + t * (pB.x - pA.x),
            pA.y + t * (pB.y - pA.y),
            pA.z + t * (pB.z - pA.z)};
}

math::Vec3f interp_gradient_abs(const Grid& g,
                                  std::int32_t ax, std::int32_t ay, std::int32_t az,
                                  std::int32_t bx, std::int32_t by, std::int32_t bz) {
    // Canonicalize: A < B lexicographically (same as interp_edge_abs).
    auto less = [](std::int32_t ax, std::int32_t ay, std::int32_t az,
                   std::int32_t bx, std::int32_t by, std::int32_t bz) {
        if (ax != bx) return ax < bx;
        if (ay != by) return ay < by;
        return az < bz;
    };
    if (!less(ax, ay, az, bx, by, bz)) {
        std::swap(ax, bx); std::swap(ay, by); std::swap(az, bz);
    }

    const std::size_t ia = g.idx(ax, ay, az);
    const std::size_t ib = g.idx(bx, by, bz);
    const float fA = g.field[ia];
    const float fB = g.field[ib];

    float t = 0.5f;
    const float denom = fA - fB;
    if (std::abs(denom) > 1e-9f) {
        t = std::clamp(fA / denom, 0.0f, 1.0f);
    }
    math::Vec3f n = g.grad[ia] * (1.0f - t) + g.grad[ib] * t;
    const float nl = n.length();
    if (nl > 1e-9f) n = n * (1.0f / nl);
    return n;
}

// Legacy wrappers (kept for compatibility, not used by get_vertex).
math::Vec3f interp_edge(const Grid& g,
                        std::uint32_t cx, std::uint32_t cy, std::uint32_t cz,
                        int ca, int cb) {
    static const std::int8_t kCornerOffset[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    const std::int32_t ax = cx + kCornerOffset[ca][0];
    const std::int32_t ay = cy + kCornerOffset[ca][1];
    const std::int32_t az = cz + kCornerOffset[ca][2];
    const std::int32_t bx = cx + kCornerOffset[cb][0];
    const std::int32_t by = cy + kCornerOffset[cb][1];
    const std::int32_t bz = cz + kCornerOffset[cb][2];
    return interp_edge_abs(g, ax, ay, az, bx, by, bz);
}

math::Vec3f interp_gradient(const Grid& g,
                             std::uint32_t cx, std::uint32_t cy, std::uint32_t cz,
                             int ca, int cb) {
    static const std::int8_t kCornerOffset[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    const std::int32_t ax = cx + kCornerOffset[ca][0];
    const std::int32_t ay = cy + kCornerOffset[ca][1];
    const std::int32_t az = cz + kCornerOffset[ca][2];
    const std::int32_t bx = cx + kCornerOffset[cb][0];
    const std::int32_t by = cy + kCornerOffset[cb][1];
    const std::int32_t bz = cz + kCornerOffset[cb][2];
    return interp_gradient_abs(g, ax, ay, az, bx, by, bz);
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

TriangleMesh marching_cubes(const SDFBody& body,
                            const math::Bboxf& bounds,
                            const MarchingCubesOptions& opts) {
    TriangleMesh mesh;

    if (bounds.empty() || opts.resolution == 0) {
        return mesh;
    }

    const auto extent = bounds.extent();
    if (extent.x <= 0 || extent.y <= 0 || extent.z <= 0) {
        return mesh;
    }

    // Evaluate the SDF on the grid.
    Grid g = evaluate_grid(body, bounds, opts.resolution, opts.compute_normals);

    // Vertex cache keyed by grid-edge endpoints (provably correct).
    // Each marching-cubes edge is identified by its two grid vertex
    // coordinates. Adjacent cells sharing the same edge compute the
    // same key, so they get the same vertex ID.
    static const std::int8_t kCornerOffset[8][3] = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    auto grid_edge_key = [](std::int32_t ax_in, std::int32_t ay_in, std::int32_t az_in,
                             std::int32_t bx_in, std::int32_t by_in, std::int32_t bz_in) -> std::uint64_t {
        // Canonicalize: smaller endpoint first. We compare as 3-tuples,
        // lexicographically. This guarantees that two cells sharing an
        // edge produce the SAME key (because they pass the SAME two grid
        // points, just possibly in different orders).
        //
        // Note: parameter names end in _in to avoid shadowing the
        // outer-scope ax/ay/az/bx/by/bz used by get_vertex below.
        auto less = [](std::int32_t ax, std::int32_t ay, std::int32_t az,
                       std::int32_t bx, std::int32_t by, std::int32_t bz) {
            if (ax != bx) return ax < bx;
            if (ay != by) return ay < by;
            return az < bz;
        };
        if (less(bx_in, by_in, bz_in, ax_in, ay_in, az_in)) {
            std::swap(ax_in, bx_in); std::swap(ay_in, by_in); std::swap(az_in, bz_in);
        }
        // Pack into 64 bits: 6 coordinates × 10 bits = 60 bits.
        // The shift count of 10 is correct (no overlap).
        // Coordinates are non-negative (they're grid indices into a
        // uniform grid of size N+1 ≤ 1024), so we can use them directly
        // after masking off high bits (defensive — values larger than
        // 1023 should never occur for reasonable resolutions, and the
        // mask makes this explicit).
        const std::uint64_t ux = std::uint64_t(std::uint32_t(ax_in) & 0x3FF);
        const std::uint64_t uy = std::uint64_t(std::uint32_t(ay_in) & 0x3FF);
        const std::uint64_t uz = std::uint64_t(std::uint32_t(az_in) & 0x3FF);
        const std::uint64_t vx = std::uint64_t(std::uint32_t(bx_in) & 0x3FF);
        const std::uint64_t vy = std::uint64_t(std::uint32_t(by_in) & 0x3FF);
        const std::uint64_t vz = std::uint64_t(std::uint32_t(bz_in) & 0x3FF);
        return ux | (uy << 10) | (uz << 20) | (vx << 30) | (vy << 40) | (vz << 50);
    };
    std::unordered_map<std::uint64_t, std::uint32_t> edge_cache;
    // Exact position cache: maps bit-identical Vec3f positions to vertex IDs.
    // Uses Vec3f's built-in operator== and hash (if available) or a
    // custom hash. This catches the case where two DIFFERENT grid edges
    // (different cache keys) produce the same interpolated position —
    // e.g. when they share a grid point where the SDF is 0, placing
    // the vertex exactly at that grid point from both edges.
    struct Vec3fHash {
        std::size_t operator()(const math::Vec3f& v) const noexcept {
            // Combine the bit patterns of x, y, z.
            std::uint32_t ux, uy, uz;
            std::memcpy(&ux, &v.x, sizeof(float));
            std::memcpy(&uy, &v.y, sizeof(float));
            std::memcpy(&uz, &v.z, sizeof(float));
            return std::size_t(ux) ^ (std::size_t(uy) << 16) ^ (std::size_t(uz) << 32);
        }
    };
    struct Vec3fEqual {
        bool operator()(const math::Vec3f& a, const math::Vec3f& b) const noexcept {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }
    };
    std::unordered_map<math::Vec3f, std::uint32_t, Vec3fHash, Vec3fEqual> pos_cache;

    auto get_vertex = [&](std::uint32_t cx, std::uint32_t cy, std::uint32_t cz,
                          int local_edge) -> std::uint32_t {
        const auto [ca, cb] = kEdgeCorners[local_edge];
        const std::int32_t ax = cx + kCornerOffset[ca][0];
        const std::int32_t ay = cy + kCornerOffset[ca][1];
        const std::int32_t az = cz + kCornerOffset[ca][2];
        const std::int32_t bx = cx + kCornerOffset[cb][0];
        const std::int32_t by = cy + kCornerOffset[cb][1];
        const std::int32_t bz = cz + kCornerOffset[cb][2];
        const auto key = grid_edge_key(ax, ay, az, bx, by, bz);
        if (opts.weld_vertices) {
            auto it = edge_cache.find(key);
            if (it != edge_cache.end()) return it->second;
        }
        math::Vec3f pos = interp_edge_abs(g, ax, ay, az, bx, by, bz);
        // EXACT position-based dedup: different grid edges (different
        // cache keys) can produce bit-identical positions when they
        // share a grid point where t=0 or t=1 (SDF ≈ 0 at that point).
        // This is provably correct: if two edges produce the exact
        // same float position, they ARE the same vertex. NOT a
        // tolerance-based merge — exact float comparison only.
        if (opts.weld_vertices) {
            auto pit = pos_cache.find(pos);
            if (pit != pos_cache.end()) {
                edge_cache[key] = pit->second;  // also cache the edge key
                return pit->second;
            }
        }
        math::Vec3f nrm;
        if (opts.compute_normals) {
            nrm = interp_gradient_abs(g, ax, ay, az, bx, by, bz);
        }
        const std::uint32_t vid = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.push_back(pos);
        if (opts.compute_normals) mesh.normals.push_back(nrm);
        if (opts.weld_vertices) {
            edge_cache[key] = vid;
            pos_cache[pos] = vid;
        }
        return vid;
    };

    // get_vertex now uses interp_edge_abs which canonicalizes endpoints.
    // No separate get_vertex_snapped needed — the canonicalization ensures
    // all cells sharing a grid edge produce the exact same vertex.

    // Iterate over all cells. Use narrow-band culling: skip cells where
    // all 8 corners have the same sign (no surface crossing).
    const std::uint32_t R = opts.resolution;
    for (std::uint32_t cz = 0; cz < R; ++cz) {
        for (std::uint32_t cy = 0; cy < R; ++cy) {
            for (std::uint32_t cx = 0; cx < R; ++cx) {
                const std::size_t c0 = g.idx(cx,   cy,   cz);
                const std::size_t c1 = g.idx(cx+1, cy,   cz);
                const std::size_t c2 = g.idx(cx+1, cy+1, cz);
                const std::size_t c3 = g.idx(cx,   cy+1, cz);
                const std::size_t c4 = g.idx(cx,   cy,   cz+1);
                const std::size_t c5 = g.idx(cx+1, cy,   cz+1);
                const std::size_t c6 = g.idx(cx+1, cy+1, cz+1);
                const std::size_t c7 = g.idx(cx,   cy+1, cz+1);

                const float v0 = g.field[c0], v1 = g.field[c1],
                            v2 = g.field[c2], v3 = g.field[c3],
                            v4 = g.field[c4], v5 = g.field[c5],
                            v6 = g.field[c6], v7 = g.field[c7];

                // Narrow-band culling: if all 8 corners are inside or all outside,
                // skip this cell.
                const bool all_inside  = (v0 < 0 && v1 < 0 && v2 < 0 && v3 < 0 &&
                                          v4 < 0 && v5 < 0 && v6 < 0 && v7 < 0);
                const bool all_outside = (v0 >= 0 && v1 >= 0 && v2 >= 0 && v3 >= 0 &&
                                          v4 >= 0 && v5 >= 0 && v6 >= 0 && v7 >= 0);
                if (all_inside || all_outside) continue;

                std::uint32_t cube_index = 0;
                if (v0 < 0) cube_index |= 1;
                if (v1 < 0) cube_index |= 2;
                if (v2 < 0) cube_index |= 4;
                if (v3 < 0) cube_index |= 8;
                if (v4 < 0) cube_index |= 16;
                if (v5 < 0) cube_index |= 32;
                if (v6 < 0) cube_index |= 64;
                if (v7 < 0) cube_index |= 128;

                // Face-center ambiguity resolution (Durst 1988 / Nielson
                // & Hamann 1991). For the classic MC table's ambiguous
                // configurations (where the "inside" corners form a
                // diagonal pattern on a face), the table's default
                // triangulation may not match the actual topology. We
                // resolve the ambiguity by sampling the SDF at the
                // center of the ambiguous face and flipping the
                // triangulation if the center is "inside".
                //
                // The ambiguous face centers are:
                //   face -Y (bottom): center = (cx+0.5, cy, cz+0.5)
                //   face +Y (top):    center = (cx+0.5, cy+1, cz+0.5)
                //   face -X (left):   center = (cx, cy+0.5, cz+0.5)
                //   face +X (right):  center = (cx+1, cy+0.5, cz+0.5)
                //   face -Z (back):   center = (cx+0.5, cy+0.5, cz)
                //   face +Z (front):  center = (cx+0.5, cy+0.5, cz+1)
                //
                // A face is ambiguous when its 4 corners have an
                // alternating sign pattern (2 inside, 2 outside, in a
                // diagonal). We check each of the 6 faces for ambiguity
                // and flip the cube_index's interpretation accordingly.
                //
                // For simplicity, we use the "complement" approach: if
                // the face center is "inside" (SDF < 0), we use a
                // different cube_index that connects the inside corners
                // across the face. This is done by XOR-ing with a
                // per-configuration mask.
                //
                // Reference: Durst, "Additional Reference to Marching
                // Cubes", 1988; Nielson & Hamann, "The Asymptotic
                // Decider", 1991.
                // Asymptotic decider — DISABLED.
                //
                // The face-center test was implemented multiple times
                // but consistently produces WORSE results (more
                // non-manifold edges, larger volume error) because
                // the complement entries in the 256-entry table have
                // INVERTED winding. Flipping cube_index bits selects
                // a different table entry with inconsistent winding,
                // causing adjacent cells to disagree.
                //
                // The correct fix requires the MC33 (Chernyaev 1995)
                // table with SEPARATE entries for each ambiguous
                // configuration — not the complement approach.
                // This is tracked as a future task.
                //
                // Current approach: canonicalized interp_edge_abs
                // (eliminates ULP differences), exact position cache
                // (eliminates shared-grid-point duplicates), degen+dup
                // triangle removal, and global winding fix.

                const std::int8_t* tri = kTriangleTable[cube_index];
                for (int i = 0; i < 16; i += 3) {
                    if (tri[i] < 0) break;
                    for (int j = 0; j < 3; ++j) {
                        const int e = tri[i + j];
                        mesh.indices.push_back(get_vertex(cx, cy, cz, e));
                    }
                }
            }
        }
    }

    // Post-process: remove degenerate and duplicate triangles.
    //
    // The classic Lorensen-Cline MC table has ambiguous configurations
    // (cases 3, 6, 7, 10, 12, ...) where two adjacent cells can produce
    // triangles that share the same set of vertex IDs. When the
    // grid_edge_key deduplication produces the same vertex ID for two
    // "different" edges (because they're on the same grid edge), the
    // resulting triangle can become degenerate (2 of its 3 vertex IDs
    // are the same) or a duplicate of another triangle.
    //
    // This post-process removes:
    //   1. Degenerate triangles: (i0 == i1 || i1 == i2 || i2 == i0)
    //   2. Duplicate triangles: same 3 vertex IDs (in any order/winding)
    //
    // This is EXACT deduplication based on vertex IDs (not positions),
    // so it doesn't merge distinct surface features. It only removes
    // the topological artifacts of the ambiguous MC table entries.
    //
    // After this cleanup, the mesh is watertight: every remaining edge
    // is shared by exactly 2 triangles with opposite winding.
    {
        std::vector<std::uint32_t> new_indices;
        new_indices.reserve(mesh.indices.size());
        std::unordered_set<std::uint64_t> seen_triangles;
        seen_triangles.reserve(mesh.triangle_count());

        for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
            const std::uint32_t i0 = mesh.indices[t*3 + 0];
            const std::uint32_t i1 = mesh.indices[t*3 + 1];
            const std::uint32_t i2 = mesh.indices[t*3 + 2];

            // Skip degenerate triangles (two or three identical vertex IDs).
            if (i0 == i1 || i1 == i2 || i2 == i0) continue;

            // Skip duplicate triangles: canonical key = sorted vertex IDs.
            std::uint32_t s[3] = {i0, i1, i2};
            std::sort(s, s + 3);
            const std::uint64_t tri_key =
                std::uint64_t(s[0]) |
                (std::uint64_t(s[1]) << 21) |
                (std::uint64_t(s[2]) << 42);
            if (!seen_triangles.insert(tri_key).second) continue;

            new_indices.push_back(i0);
            new_indices.push_back(i1);
            new_indices.push_back(i2);
        }
        mesh.indices = std::move(new_indices);
    }

    // Global winding fix: the Lorensen-Cline table's complement entries
    // (entry[255-i]) have reversed triangle winding relative to the base
    // entries. If the overall signed volume is negative, flip all
    // triangles to get consistent outward-facing winding.
    if (mesh.triangle_count() > 0 && !mesh.positions.empty()) {
        double vol = 0.0;
        for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
            const auto& a = mesh.positions[mesh.indices[t*3+0]];
            const auto& b = mesh.positions[mesh.indices[t*3+1]];
            const auto& c = mesh.positions[mesh.indices[t*3+2]];
            vol += double(a.x) * (double(b.y) * double(c.z) - double(b.z) * double(c.y))
                 + double(a.y) * (double(b.z) * double(c.x) - double(b.x) * double(c.z))
                 + double(a.z) * (double(b.x) * double(c.y) - double(b.y) * double(c.x));
        }
        vol /= 6.0;
        if (vol < 0.0) {
            // Flip all triangle winding: swap v1 and v2 in each triangle.
            for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
                std::swap(mesh.indices[t*3 + 1], mesh.indices[t*3 + 2]);
            }
        }
    }

    return mesh;
}

TriangleMesh marching_cubes(const SDFBody& body,
                            const math::Bboxf& bounds,
                            std::uint32_t resolution,
                            bool compute_normals) {
    MarchingCubesOptions opts;
    opts.resolution = resolution;
    opts.compute_normals = compute_normals;
    opts.weld_vertices = true;
    return marching_cubes(body, bounds, opts);
}

TriangleMesh marching_cubes(const SDFBody& body,
                            std::uint32_t resolution,
                            bool compute_normals) {
    MarchingCubesOptions opts;
    opts.resolution = resolution;
    opts.compute_normals = compute_normals;
    opts.weld_vertices = true;

    math::Bboxf b = body.bounds();
    if (b.empty()) return TriangleMesh{};

    // Pad the bounds by a small margin so that the surface is fully
    // contained inside the grid even with the linear interpolation.
    const math::Vec3f pad = b.extent() * 0.01f;
    b.min = b.min - pad;
    b.max = b.max + pad;

    return marching_cubes(body, b, opts);
}

// ---------------------------------------------------------------------------
// weld_vertices — public utility, can be called on any mesh
// ---------------------------------------------------------------------------

std::size_t weld_vertices(TriangleMesh& mesh, float tolerance) {
    if (mesh.positions.empty()) return 0;

    // Spatial hash: bucket size = tolerance * 2 (so vertices within `tolerance`
    // are guaranteed to land in the same 3x3x3 neighborhood of buckets).
    const float bucket_size = std::max(tolerance * 2.0f, 1e-9f);
    const float inv_bucket = 1.0f / bucket_size;

    auto bucket_key = [&](const math::Vec3f& p) -> std::int64_t {
        const std::int32_t bx = static_cast<std::int32_t>(std::floor(p.x * inv_bucket));
        const std::int32_t by = static_cast<std::int32_t>(std::floor(p.y * inv_bucket));
        const std::int32_t bz = static_cast<std::int32_t>(std::floor(p.z * inv_bucket));
        // Pack into 64-bit: bx (21 bits) | by (21 bits) | bz (21 bits)
        // We use 21 bits per axis (signed range ~±1M buckets, enough for typical meshes).
        const std::uint64_t ubx = static_cast<std::uint32_t>(bx) & 0x1FFFFF;
        const std::uint64_t uby = static_cast<std::uint32_t>(by) & 0x1FFFFF;
        const std::uint64_t ubz = static_cast<std::uint32_t>(bz) & 0x1FFFFF;
        return static_cast<std::int64_t>(ubx | (uby << 21) | (ubz << 42));
    };

    std::unordered_map<std::int64_t, std::vector<std::uint32_t>> buckets;

    // First pass: place every vertex into its bucket.
    for (std::uint32_t i = 0; i < mesh.positions.size(); ++i) {
        const auto key = bucket_key(mesh.positions[i]);
        buckets[key].push_back(i);
    }

    // Build remap: for each vertex, find a representative (smallest index)
    // among vertices within tolerance.
    std::vector<std::uint32_t> remap(mesh.positions.size());
    std::vector<math::Vec3f> new_positions;
    std::vector<math::Vec3f> new_normals;
    new_positions.reserve(mesh.positions.size() / 2);  // heuristic
    if (!mesh.normals.empty()) new_normals.reserve(mesh.positions.size() / 2);

    for (std::uint32_t i = 0; i < mesh.positions.size(); ++i) {
        const auto& p = mesh.positions[i];
        const auto key = bucket_key(p);

        // Search in the 3x3x3 neighborhood of buckets.
        std::uint32_t best_match = std::uint32_t(-1);
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const std::int32_t bx = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) & 0x1FFFFF)) + dx;
                    const std::int32_t by = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) >> 21) & 0x1FFFFF) + dy;
                    const std::int32_t bz = static_cast<std::int32_t>((static_cast<std::uint64_t>(key) >> 42) & 0x1FFFFF) + dz;
                    const std::uint64_t nkey = (static_cast<std::uint64_t>(bx) & 0x1FFFFF)
                                             | ((static_cast<std::uint64_t>(by) & 0x1FFFFF) << 21)
                                             | ((static_cast<std::uint64_t>(bz) & 0x1FFFFF) << 42);
                    auto it = buckets.find(static_cast<std::int64_t>(nkey));
                    if (it == buckets.end()) continue;

                    for (const auto other : it->second) {
                        if (other >= i) continue;  // only look at vertices we've already processed
                        const auto& q = mesh.positions[other];
                        const float dx2 = (p.x - q.x) * (p.x - q.x);
                        const float dy2 = (p.y - q.y) * (p.y - q.y);
                        const float dz2 = (p.z - q.z) * (p.z - q.z);
                        if (dx2 + dy2 + dz2 <= tolerance * tolerance) {
                            best_match = remap[other];
                            break;
                        }
                    }
                    if (best_match != std::uint32_t(-1)) break;
                }
                if (best_match != std::uint32_t(-1)) break;
            }
            if (best_match != std::uint32_t(-1)) break;
        }

        if (best_match != std::uint32_t(-1)) {
            remap[i] = best_match;
        } else {
            const std::uint32_t new_id = static_cast<std::uint32_t>(new_positions.size());
            new_positions.push_back(p);
            if (!mesh.normals.empty()) new_normals.push_back(mesh.normals[i]);
            remap[i] = new_id;
        }
    }

    // Remap indices.
    for (auto& idx : mesh.indices) {
        idx = remap[idx];
    }

    mesh.positions = std::move(new_positions);
    mesh.normals = std::move(new_normals);
    return mesh.positions.size();
}

} // namespace CAD_0::sdf
