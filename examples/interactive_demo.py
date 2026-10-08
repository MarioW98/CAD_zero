"""Interactive demo: build SDF, extract mesh, view in 3D.

This example demonstrates the complete Python workflow:
  1. Build an SDF body using CSG operations
  2. Extract a triangle mesh via marching cubes
  3. Visualize the mesh in a 3D matplotlib window
  4. Show a 2D cross-section of the SDF field

Usage:
    cd /path/to/CAD_0/build/python
    python3 /path/to/examples/interactive_demo.py
"""

from __future__ import annotations

import sys
import os

# Ensure the compiled extension is on the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'python'))

import numpy as np
import CAD_0


def build_lattice_ball():
    """Build a sphere with 3 orthogonal cylindrical holes (lattice ball)."""
    ball = CAD_0.sdf.sphere(radius=2.0)

    cyl_y = CAD_0.sdf.cylinder(radius=0.4, height=10.0)
    cyl_z = CAD_0.sdf.rotate(
        CAD_0.sdf.cylinder(radius=0.4, height=10.0),
        CAD_0.math.Quatf.from_axis_angle((1.0, 0.0, 0.0), 3.14159 / 2.0),
    )
    cyl_x = CAD_0.sdf.rotate(
        CAD_0.sdf.cylinder(radius=0.4, height=10.0),
        CAD_0.math.Quatf.from_axis_angle((0.0, 1.0, 0.0), 3.14159 / 2.0),
    )

    holes = CAD_0.sdf.union(
        CAD_0.sdf.union(cyl_y, cyl_z),
        cyl_x,
    )
    body = CAD_0.sdf.subtract(ball, holes)
    return body


def main():
    print("=" * 60)
    print("CAD_0 Interactive Demo")
    print("=" * 60)
    print()

    # Step 1: Build the SDF
    print("Step 1: Building lattice ball SDF...")
    body = build_lattice_ball()
    print(f"  Body: {body.describe()}")
    print(f"  Bounds: {body.bounds().min} -> {body.bounds().max}")
    print(f"  Lipschitz: {body.lipschitz()}")
    print()

    # Step 2: Evaluate on a few points
    print("Step 2: Evaluating SDF at sample points...")
    sample_pts = np.array([
        [0.0, 0.0, 0.0],   # center (inside sphere, inside holes → outside result)
        [1.5, 0.0, 0.0],   # inside sphere, outside holes → inside result
        [3.0, 0.0, 0.0],   # outside sphere → outside result
    ], dtype=np.float32)
    result = CAD_0.sdf.evaluate(body, sample_pts)
    for i, (p, v) in enumerate(zip(sample_pts, result.values)):
        status = "inside" if v < 0 else "outside"
        print(f"  pt{i} = {p.tolist()}  sdf = {v:.4f}  ({status})")
    print()

    # Step 3: Extract mesh
    print("Step 3: Extracting mesh (marching cubes, res=48)...")
    mesh = CAD_0.sdf.marching_cubes(body, resolution=48, compute_normals=True)
    print(f"  Vertices:  {mesh.vertex_count()}")
    print(f"  Triangles: {mesh.triangle_count()}")
    print()

    # Step 4: Export
    print("Step 4: Exporting mesh...")
    out_dir = "/home/z/my-project/download"
    os.makedirs(out_dir, exist_ok=True)
    for ext in ("stl", "obj", "3mf"):
        path = os.path.join(out_dir, f"interactive_demo.{ext}")
        CAD_0.io.export(path, mesh, name="lattice_ball")
        size = os.path.getsize(path)
        print(f"  {ext.upper()}: {path} ({size:,} bytes)")
    print()

    # Step 5: 3D View
    print("Step 5: Opening 3D viewer...")
    print("  (close the window to continue)")
    try:
        fig = CAD_0.viewer.view_mesh(
            mesh,
            title="CAD_0 — Lattice Ball (3D Mesh)",
            color='cyan',
            show_normals=False,
            linewidth=0.0,
        )
    except Exception as e:
        print(f"  3D viewer error: {e}")
    print()

    # Step 6: 2D Cross-section
    print("Step 6: Opening 2D cross-section viewer (z=0)...")
    print("  (close the window to finish)")
    try:
        fig2 = CAD_0.viewer.view_sdf_slices(
            body,
            resolution=64,
            plane='z',
            slice_at=0.0,
            title="CAD_0 — Lattice Ball (Cross-Section)",
        )
    except Exception as e:
        print(f"  2D viewer error: {e}")
    print()

    print("=" * 60)
    print("Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
