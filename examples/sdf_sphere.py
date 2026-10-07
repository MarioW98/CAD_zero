"""Example: build a sphere SDF and evaluate it on a grid.

Usage (after `pip install cadforge`):
    python examples/sdf_sphere.py
"""

from __future__ import annotations

import numpy as np

import cadforge


def main() -> None:
    # Build a unit sphere centered at the origin.
    sph = cadforge.sdf.sphere(radius=1.0)
    print(f"Body: {sph.describe()}")
    print(f"Bounds: min={sph.bounds().min}, max={sph.bounds().max}")
    print(f"Lipschitz constant: {sph.lipschitz()}")

    # Evaluate at three points: center, surface, outside.
    pts = np.array([
        [0.0, 0.0, 0.0],   # inside  → -1
        [1.0, 0.0, 0.0],   # surface → 0
        [2.0, 0.0, 0.0],   # outside → +1
    ], dtype=np.float32)

    out = cadforge.sdf.evaluate(sph, pts)
    print(f"\nEvaluated {pts.shape[0]} points:")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
