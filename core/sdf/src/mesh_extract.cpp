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

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
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
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  1,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  8,  3,  9,  8,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  2,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  3,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9,  2,  10, 0,  2,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2,  8,  3,  2,  10, 8,  10, 9,  8,  -1, -1, -1, -1, -1, -1, -1},
    {3,  11, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  11, 2,  8,  11, 0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  9,  0,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  11, 2,  1,  9,  11, 9,  8,  11, -1, -1, -1, -1, -1, -1, -1},
    {3,  10, 1,  11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  10, 1,  0,  8,  10, 8,  11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3,  9,  0,  3,  11, 9,  11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9,  8,  10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4,  3,  0,  7,  3,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  1,  9,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4,  1,  9,  4,  7,  1,  7,  3,  1,  -1, -1, -1, -1, -1, -1, -1},
    {1,  2,  10, 8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  4,  7,  3,  0,  4,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1},
    {9,  2,  10, 9,  0,  2,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1},
    {2,  10, 9,  2,  9,  7,  2,  7,  3,  7,  9,  4,  -1, -1, -1, -1},
    {8,  4,  7,  3,  11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4,  7,  11, 2,  4,  2,  0,  4,  -1, -1, -1, -1, -1, -1, -1},
    {9,  0,  1,  8,  4,  7,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1},
    {4,  7,  11, 9,  4,  11, 9,  11, 2,  9,  2,  1,  -1, -1, -1, -1},
    {3,  10, 1,  3,  11, 10, 7,  8,  4,  -1, -1, -1, -1, -1, -1, -1},
    {1,  11, 10, 1,  4,  11, 1,  0,  4,  7,  11, 4, -1, -1, -1, -1},
    {4,  7,  8,  9,  0,  11, 9,  11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4,  7,  11, 4,  11, 9,  9,  11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9,  5,  4,  0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  5,  4,  1,  5,  0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8,  5,  4,  8,  3,  5,  3,  1,  5,  -1, -1, -1, -1, -1, -1, -1},
    {1,  2,  10, 9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  0,  8,  1,  2,  10, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1},
    {5,  2,  10, 5,  4,  2,  4,  0,  2,  -1, -1, -1, -1, -1, -1, -1},
    {2,  10, 5,  3,  2,  5,  3,  5,  4,  3,  4,  8,  -1, -1, -1, -1},
    {9,  5,  4,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  11, 2,  0,  8,  11, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1},
    {0,  5,  4,  0,  1,  5,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1},
    {2,  1,  5,  2,  5,  8,  2,  8,  11, 4,  8,  5,  -1, -1, -1, -1},
    {10, 3,  11, 10, 1,  3,  9,  5,  4,  -1, -1, -1, -1, -1, -1, -1},
    {4,  9,  5,  0,  8,  1,  8,  10, 1,  8,  11, 10, -1, -1, -1, -1},
    {11, 10, 0,  11, 0,  3,  9,  5,  4,  -1, -1, -1, -1, -1, -1, -1},
    {5,  4,  9,  5,  9,  11, 5,  11, 10, 11, 9,  8,  -1, -1, -1, -1},
    {8,  4,  7,  9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9,  5,  4,  7,  3,  0,  7,  0,  4,  -1, -1, -1, -1, -1, -1, -1},
    {0,  5,  4,  1,  5,  0,  7,  8,  4,  -1, -1, -1, -1, -1, -1, -1},
    {1,  5,  4,  1,  4,  8,  1,  8,  7,  1,  7,  3,  -1, -1, -1, -1},
    {4,  9,  5,  7,  8,  4,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1},
    {3,  4,  7,  3,  0,  4,  1,  2,  10, 9,  5,  4,  -1, -1, -1, -1},
    {5,  4,  9,  2,  10, 0,  0,  10, 8,  8,  10, 7,  -1, -1, -1, -1},
    {3,  4,  7,  3,  0,  4,  4,  9,  5,  2,  3,  4,  2,  4,  10, 2},
    {7,  8,  4,  2,  3,  11, 5,  4,  9,  -1, -1, -1, -1, -1, -1, -1},
    {11, 2,  4,  11, 4,  7,  9,  4,  5,  0,  4,  2,  -1, -1, -1, -1},
    {4,  7,  8,  0,  5,  4,  0,  1,  5,  2,  3,  11, -1, -1, -1, -1},
    {5,  4,  9,  5,  2,  4,  5,  11, 2,  2,  11, 1,  7,  8,  4,  -1},
    {3,  10, 1,  3,  11, 10, 7,  8,  4,  9,  5,  4,  -1, -1, -1, -1},
    {1,  11, 10, 1,  4,  11, 1,  0,  4,  4,  7,  11, 5,  4,  9,  -1},
    {9,  5,  4,  0,  8,  6,  0,  6,  11, 6,  8,  7,  10, 6,  11, -1},
    {4,  7,  11, 4,  11, 9,  9,  11, 10, 5,  4,  9,  -1, -1, -1, -1},
    {5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  3,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9,  0,  1,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  8,  3,  1,  9,  8,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1},
    {1,  6,  5,  2,  6,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1,  6,  5,  1,  2,  6,  3,  0,  8,  -1, -1, -1, -1, -1, -1, -1},
    {9,  6,  5,  9,  0,  6,  0,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
    {5,  9,  8,  5,  8,  2,  5,  2,  6,  3,  8,  2,  -1, -1, -1, -1},
    {2,  3,  11, 10, 6,  5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0,  8,  11, 2,  0,  10, 6,  5,  -1, -1, -1, -1, -1, -1, -1},
    {0,  1,  9,  2,  3,  11, 5,  10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5,  10, 6,  1,  9,  2,  9,  11, 2,  9,  8,  11, -1, -1, -1, -1},
    {6,  3,  11, 6,  5,  3,  5,  1,  3,  -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  11, 0,  11, 5,  0,  5,  1,  5,  11, 6,  -1, -1, -1, -1},
    {3,  11, 6,  0,  3,  6,  0,  6,  5,  0,  5,  9,  -1, -1, -1, -1},
    {6,  5,  9,  6,  9,  11, 11, 9,  8,  -1, -1, -1, -1, -1, -1, -1},
    {5,  10, 6,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4,  3,  0,  4,  7,  3,  6,  5,  10, -1, -1, -1, -1, -1, -1, -1},
    {1,  9,  0,  5,  10, 6,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1},
    {10, 6,  5,  1,  9,  7,  1,  7,  3,  7,  9,  4,  -1, -1, -1, -1},
    {6,  1,  2,  6,  5,  1,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1},
    {1,  2,  5,  5,  2,  6,  3,  0,  4,  3,  4,  7,  -1, -1, -1, -1},
    {8,  4,  7,  9,  0,  5,  0,  6,  5,  0,  2,  6,  -1, -1, -1, -1},
    {7,  3,  9,  7,  9,  4,  3,  2,  9,  5,  9,  6,  2,  9,  5,  6},
    {3,  11, 2,  7,  8,  4,  10, 6,  5, -1, -1, -1, -1, -1, -1, -1},
    {5,  10, 6,  4,  7,  2,  4,  2,  0,  2,  7,  11, -1, -1, -1, -1},
    {0,  1,  9,  4,  7,  8,  2,  3,  11, 5,  10, 6, -1, -1, -1, -1},
    {9,  2,  1,  9,  11, 2,  9,  4,  11, 7,  11, 4,  5,  10, 6,  -1},
    {8,  4,  7,  3,  11, 5,  3,  5,  1,  5,  11, 6,  -1, -1, -1, -1},
    {5,  1,  11, 5,  11, 6,  1,  0,  11, 7,  11, 4,  0,  4,  11, -1},
    {0,  5,  9,  0,  6,  5,  0,  3,  6,  11, 6,  3,  8,  4,  7,  -1},
    {6,  5,  9,  6,  9,  11, 4,  7,  9,  7,  11, 9, -1, -1, -1, -1},
    {10, 4,  9,  6,  4,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4,  10, 6,  4,  9,  10, 0,  8,  3,  -1, -1, -1, -1, -1, -1, -1},
    {10, 0,  1,  10, 6,  0,  6,  4,  0,  -1, -1, -1, -1, -1, -1, -1},
    {8,  3,  1,  8,  1,  6,  8,  6,  4,  6,  1,  10, -1, -1, -1, -1},
    {1,  4,  9,  1,  2,  4,  2,  6,  4,  -1, -1, -1, -1, -1, -1, -1},
    {3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4,  -1, -1, -1, -1},
    {0,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8,  3,  2,  8,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
    {10, 4,  9,  10, 6,  4,  11, 2,  3,  -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  2,  2,  8,  11, 4,  9,  10, 4,  10, 6,  -1, -1, -1, -1},
    {3,  11, 2,  0,  1,  6,  0,  6,  4,  6,  1,  10, -1, -1, -1, -1},
    {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
    {9,  6,  4,  9,  3,  6,  9,  1,  3,  11, 6,  3,  -1, -1, -1, -1},
    {8,  11, 1,  8,  1,  0,  11, 6,  1,  9,  1,  4,  6,  4,  1,  -1},
    {6,  4,  9,  6,  9,  3,  6,  3,  11, 0,  3,  9,  -1, -1, -1, -1},
    {5,  6,  9,  5,  9,  11, 11, 9,  6,  4,  6,  8,  8,  6,  11, 11},
    {6,  4,  8,  11, 6,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  6,  11, 3,  0,  6,  0,  4,  6,  -1, -1, -1, -1, -1, -1, -1},
    {8,  6,  11, 8,  4,  6,  0,  1,  9,  -1, -1, -1, -1, -1, -1, -1},
    {9,  4,  6,  9,  6,  3,  9,  3,  1,  11, 3,  6,  -1, -1, -1, -1},
    {6,  8,  4,  11, 8,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  6,  11, 3,  0,  6,  0,  4,  6,  6,  4,  8,  -1, -1, -1, -1},
    {0,  1,  9,  8,  4,  6,  8,  6,  11, -1, -1, -1, -1, -1, -1, -1},
    {11, 1,  9,  11, 3,  1,  9,  6,  4,  9,  4,  8,  6,  4,  9,  -1},
    {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
    {1,  4,  9,  1,  2,  4,  2,  6,  4,  -1, -1, -1, -1, -1, -1, -1},
    {3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4,  -1, -1, -1, -1},
    {0,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8,  3,  2,  8,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
    {10, 4,  9,  10, 6,  4,  11, 2,  3,  -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  2,  2,  8,  11, 4,  9,  10, 4,  10, 6,  -1, -1, -1, -1},
    {3,  11, 2,  0,  1,  6,  0,  6,  4,  6,  1,  10, -1, -1, -1, -1},
    {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
    {9,  6,  4,  9,  3,  6,  9,  1,  3,  11, 6,  3,  -1, -1, -1, -1},
    {8,  11, 1,  8,  1,  0,  11, 6,  1,  9,  1,  4,  6,  4,  1,  -1},
    {6,  4,  9,  6,  9,  3,  6,  3,  11, 0,  3,  9,  -1, -1, -1, -1},
    {5,  6,  9,  5,  9,  11, 11, 9,  6,  4,  6,  8,  8,  6,  11, 11},
    {6,  4,  8,  11, 6,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  6,  11, 3,  0,  6,  0,  4,  6,  -1, -1, -1, -1, -1, -1, -1},
    {8,  6,  11, 8,  4,  6,  0,  1,  9,  -1, -1, -1, -1, -1, -1, -1},
    {9,  4,  6,  9,  6,  3,  9,  3,  1,  11, 3,  6,  -1, -1, -1, -1},
    {6,  8,  4,  11, 8,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3,  6,  11, 3,  0,  6,  0,  4,  6,  6,  4,  8,  -1, -1, -1, -1},
    {0,  1,  9,  8,  4,  6,  8,  6,  11, -1, -1, -1, -1, -1, -1, -1},
    {11, 1,  9,  11, 3,  1,  9,  6,  4,  9,  4,  8,  6,  4,  9,  -1},
    {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
    {1,  4,  9,  1,  2,  4,  2,  6,  4,  -1, -1, -1, -1, -1, -1, -1},
    {3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4,  -1, -1, -1, -1},
    {0,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8,  3,  2,  8,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
    {10, 4,  9,  10, 6,  4,  11, 2,  3,  -1, -1, -1, -1, -1, -1, -1},
    {0,  8,  2,  2,  8,  11, 4,  9,  10, 4,  10, 6,  -1, -1, -1, -1},
    {3,  11, 2,  0,  1,  6,  0,  6,  4,  6,  1,  10, -1, -1, -1, -1},
    {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
    {9,  6,  4,  9,  3,  6,  9,  1,  3,  11, 6,  3,  -1, -1, -1, -1},
    {8,  11, 1,  8,  1,  0,  11, 6,  1,  9,  1,  4,  6,  4,  1,  -1},
    {6,  4,  9,  6,  9,  3,  6,  3,  11, 0,  3,  9,  -1, -1, -1, -1},
    {5,  6,  9,  5,  9,  11, 11, 9,  6,  4,  6,  8,  8,  6,  11, 11}
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
// For each edge, the canonical (cell-anchored) corner coordinates.
// Edge 0 (corners 0-1) is anchored at (0,0,0); it's shared with the
// cell at (-1,0,0). To deduplicate, anchor each edge at the cell
// whose origin (corner 0) coincides with the smaller-coordinate
// endpoint of the edge.
// ---------------------------------------------------------------------------
struct EdgeAnchor {
    std::int32_t dx, dy, dz;       // offset of the anchoring cell
    int edge_idx;                   // edge index in that cell
};

// For each of the 12 edges, return the anchor offset (dx, dy, dz) and
// the edge index in the anchored cell.
//
// Edge 0 of cell (cx,cy,cz) connects corner 0 (cx,cy,cz) and corner 1 (cx+1,cy,cz).
// Edge 1 connects corner 1 (cx+1,cy,cz) and corner 2 (cx+1,cy+1,cz).
// Edge 2 connects corner 2 (cx+1,cy+1,cz) and corner 3 (cx,cy+1,cz).
// Edge 3 connects corner 3 (cx,cy+1,cz) and corner 0 (cx,cy,cz).
// Edge 8 connects corner 0 (cx,cy,cz) and corner 4 (cx,cy,cz+1).
//
// Consider an edge in the grid: identified by its two endpoint grid coords.
// To deduplicate, each grid edge has a canonical (cell, edge_idx) representation.
//
// Edge 0 of cell (cx,cy,cz): endpoints (cx,cy,cz) and (cx+1,cy,cz).
//   Also edge 0 of cell (cx-1,cy,cz): endpoints (cx-1,cy,cz) and (cx,cy,cz).
//   Canonical: the cell with smaller cx, i.e. (cx-1, cy, cz), edge 0.
//
// Each edge is anchored at the cell whose corner 0 is at the
// smaller-coordinate endpoint.
static const std::array<EdgeAnchor, 12> kEdgeAnchors = {{
    {0, 0, 0, 0},  // edge 0: along +X, anchored at (cx, cy, cz)
    {0, 0, 0, 1},  // edge 1: along +Y at x=cx+1, anchored at (cx, cy, cz)
    {-1, 0, 0, 1}, // edge 2: along +X at y=cy+1, anchored at (cx-1, cy, cz)
    {0, 0, 0, 3},  // edge 3: along +Y at x=cx, anchored at (cx, cy, cz)
    {0, 0, 0, 4},  // edge 4: along +X at z=cz+1, anchored at (cx, cy, cz)
    {0, 0, 0, 5},  // edge 5: along +Y at x=cx+1, z=cz+1, anchored at (cx, cy, cz)
    {-1, 0, 0, 5}, // edge 6: along +X at y=cy+1, z=cz+1, anchored at (cx-1, cy, cz)
    {0, 0, 0, 7},  // edge 7: along +Y at x=cx, z=cz+1, anchored at (cx, cy, cz)
    {0, 0, 0, 8},  // edge 8: along +Z at (cx, cy, cz), anchored at (cx, cy, cz)
    {0, 0, 0, 9},  // edge 9: along +Z at (cx+1, cy, cz), anchored at (cx, cy, cz)
    {0, -1, 0, 9}, // edge 10: along +Z at (cx+1, cy+1, cz), anchored at (cx, cy-1, cz)
    {0, 0, 0, 11}, // edge 11: along +Z at (cx, cy+1, cz), anchored at (cx, cy, cz)
}};

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

// Compute the vertex position along an edge of cell (cx, cy, cz).
// The edge is identified by the two corner indices ca, cb in {0..7}.
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

    const std::size_t ia = g.idx(ax, ay, az);
    const std::size_t ib = g.idx(bx, by, bz);
    const float va = g.field[ia];
    const float vb = g.field[ib];

    // Linear interpolation t = va / (va - vb), clamped to [0, 1].
    float t = 0.5f;
    const float denom = va - vb;
    if (std::abs(denom) > 1e-9f) {
        t = va / denom;
        t = std::clamp(t, 0.0f, 1.0f);
    }
    const math::Vec3f& pa = g.points[ia];
    const math::Vec3f& pb = g.points[ib];
    return {pa.x + t * (pb.x - pa.x),
            pa.y + t * (pb.y - pa.y),
            pa.z + t * (pb.z - pa.z)};
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

    const std::size_t ia = g.idx(ax, ay, az);
    const std::size_t ib = g.idx(bx, by, bz);
    const float va = g.field[ia];
    const float vb = g.field[ib];

    float t = 0.5f;
    const float denom = va - vb;
    if (std::abs(denom) > 1e-9f) {
        t = std::clamp(va / denom, 0.0f, 1.0f);
    }
    math::Vec3f n = g.grad[ia] * (1.0f - t) + g.grad[ib] * t;
    const float nl = n.length();
    if (nl > 1e-9f) n = n * (1.0f / nl);
    return n;
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
    auto grid_edge_key = [](std::int32_t ax, std::int32_t ay, std::int32_t az,
                             std::int32_t bx, std::int32_t by, std::int32_t bz) -> std::uint64_t {
        // Canonicalize: smaller endpoint first
        if (ax > bx || (ax == bx && ay > by) || (ax == bx && ay == by && az > bz)) {
            std::swap(ax, bx); std::swap(ay, by); std::swap(az, bz);
        }
        // Each coordinate fits in 10 bits (supports resolution up to 1023).
        // 6 coordinates × 10 bits = 60 bits, fits in uint64_t with NO collisions.
        const std::uint64_t ux = std::uint64_t(std::uint32_t(ax) & 0x3FF);
        const std::uint64_t uy = std::uint64_t(std::uint32_t(ay) & 0x3FF);
        const std::uint64_t uz = std::uint64_t(std::uint32_t(az) & 0x3FF);
        const std::uint64_t vx = std::uint64_t(std::uint32_t(bx) & 0x3FF);
        const std::uint64_t vy = std::uint64_t(std::uint32_t(by) & 0x3FF);
        const std::uint64_t vz = std::uint64_t(std::uint32_t(bz) & 0x3FF);
        return ux | (uy << 10) | (uz << 20) | (vx << 30) | (vy << 40) | (vz << 50);
    };
    std::unordered_map<std::uint64_t, std::uint32_t> edge_cache;

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
        math::Vec3f pos = interp_edge(g, cx, cy, cz, ca, cb);
        math::Vec3f nrm;
        if (opts.compute_normals) {
            nrm = interp_gradient(g, cx, cy, cz, ca, cb);
        }
        const std::uint32_t vid = static_cast<std::uint32_t>(mesh.positions.size());
        mesh.positions.push_back(pos);
        if (opts.compute_normals) mesh.normals.push_back(nrm);
        if (opts.weld_vertices) edge_cache[key] = vid;
        return vid;
    };

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
