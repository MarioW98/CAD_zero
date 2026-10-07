"""Example: build a lattice ball SDF, extract a mesh, export to STL/OBJ/3MF.

Mirrors the C++ example `examples/cpp/sphere_to_stl.cpp`. Requires the
cadforge Python bindings (`pip install -e .` from the cadforge root).

Usage:
    python examples/sdf_lattice_ball.py
"""

from __future__ import annotations

import time
from pathlib import Path

import cadforge


def main() -> None:
    # Build a lattice ball: sphere minus 3 orthogonal cylinders.
    ball = cadforge.sdf.sphere(radius=2.0)

    cyl_y = cadforge.sdf.cylinder(radius=0.4, height=10.0)
    cyl_z = cadforge.sdf.rotate(
        cadforge.sdf.cylinder(radius=0.4, height=10.0),
        cadforge.math.Quatf.from_axis_angle((1.0, 0.0, 0.0), 3.14159 / 2.0),
    )
    cyl_x = cadforge.sdf.rotate(
        cadforge.sdf.cylinder(radius=0.4, height=10.0),
        cadforge.math.Quatf.from_axis_angle((0.0, 1.0, 0.0), 3.14159 / 2.0),
    )

    holes = cadforge.sdf.union(
        cadforge.sdf.union(cyl_y, cyl_z),
        cyl_x,
    )
    body = cadforge.sdf.subtract(ball, holes)
    print(f"Body: {body.describe()}")
    print(f"Lipschitz: {body.lipschitz()}")
    print()

    # Extract mesh (Phase B.5: vertex welding enabled by default).
    print("Marching cubes (resolution=64, weld=True)...")
    t_start = time.perf_counter()
    mesh = cadforge.sdf.marching_cubes(body, resolution=64, compute_normals=True)
    elapsed = (time.perf_counter() - t_start) * 1000
    print(f"  vertices:  {mesh.vertex_count()}")
    print(f"  triangles: {mesh.triangle_count()}")
    print(f"  elapsed:   {elapsed:.1f} ms")
    print()

    # Export to multiple formats.
    out_dir = Path("/home/z/my-project/download")
    out_dir.mkdir(parents=True, exist_ok=True)

    for ext in ("stl", "obj", "3mf"):
        path = out_dir / f"lattice_ball_py.{ext}"
        t_start = time.perf_counter()
        cadforge.io.export(path, mesh, name="lattice_ball")
        elapsed = (time.perf_counter() - t_start) * 1000
        size = path.stat().st_size
        print(f"  {path.name:30s}  {size:>10} bytes  {elapsed:.1f} ms")


if __name__ == "__main__":
    main()
