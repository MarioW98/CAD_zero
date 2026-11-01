"""Example: build a lattice ball SDF, extract a mesh, export to STL/OBJ/3MF.

Mirrors the C++ example `examples/cpp/sphere_to_stl.cpp`. Requires the
CAD_0 Python bindings (`pip install CAD_0` from the CAD_0 root). The
module-level `body` makes it a CLI input
(`CAD_0 view examples/sdf_lattice_ball.py`).

Usage:
    python examples/sdf_lattice_ball.py
"""

from __future__ import annotations

import time
from pathlib import Path

import CAD_0

# A lattice ball: sphere minus 3 orthogonal cylinders.
_ball = CAD_0.sdf.sphere(radius=2.0)

_cyl_y = CAD_0.sdf.cylinder(radius=0.4, height=10.0)
# from_axis_angle takes a Vec3f axis, not a tuple (the bindings do
# not auto-convert) — the original tuple call raised TypeError and
# the example had never actually run (found by the Consegna B m2
# venv-virgin proof).
_cyl_z = CAD_0.sdf.rotate(
    CAD_0.sdf.cylinder(radius=0.4, height=10.0),
    CAD_0.math.Quatf.from_axis_angle(
        CAD_0.math.Vec3f(1.0, 0.0, 0.0), 3.14159 / 2.0),
)
_cyl_x = CAD_0.sdf.rotate(
    CAD_0.sdf.cylinder(radius=0.4, height=10.0),
    CAD_0.math.Quatf.from_axis_angle(
        CAD_0.math.Vec3f(0.0, 1.0, 0.0), 3.14159 / 2.0),
)

_holes = CAD_0.sdf.union(
    CAD_0.sdf.union(_cyl_y, _cyl_z),
    _cyl_x,
)
body = CAD_0.sdf.subtract(_ball, _holes)


def main() -> None:
    print(f"Body: {body.describe()}")
    print(f"Lipschitz: {body.lipschitz()}")
    print()

    # Extract mesh (Phase B.5: vertex welding enabled by default).
    print("Marching cubes (resolution=64, weld=True)...")
    t_start = time.perf_counter()
    mesh = CAD_0.sdf.marching_cubes(body, resolution=64, compute_normals=True)
    elapsed = (time.perf_counter() - t_start) * 1000
    print(f"  vertices:  {mesh.vertex_count()}")
    print(f"  triangles: {mesh.triangle_count()}")
    print(f"  elapsed:   {elapsed:.1f} ms")
    print()

    # Export to multiple formats, into the current directory (no
    # hardcoded paths: the example must run anywhere, not just in the
    # machine that wrote it).
    for ext in ("stl", "obj", "3mf"):
        path = Path(f"lattice_ball_py.{ext}")
        t_start = time.perf_counter()
        CAD_0.io.export(path, mesh, name="lattice_ball")
        elapsed = (time.perf_counter() - t_start) * 1000
        size = path.stat().st_size
        print(f"  {path.name:30s}  {size:>10} bytes  {elapsed:.1f} ms")


if __name__ == "__main__":
    main()
