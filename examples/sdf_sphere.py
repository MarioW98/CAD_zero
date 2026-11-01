"""Example: build a sphere SDF and evaluate it on a grid.

The module IS the model: the module-level `body` is what the CLI
consumes (`CAD_0 info / eval-sdf / view / slice examples/sdf_sphere.py`).

Usage (after `pip install CAD_0`):
    python examples/sdf_sphere.py
"""

from __future__ import annotations

import numpy as np

import CAD_0

# A unit sphere centered at the origin (an SDF is an expression
# tree — building it costs nothing until you evaluate it).
body = CAD_0.sdf.sphere(radius=1.0)


def main() -> None:
    print(f"Body: {body.describe()}")
    print(f"Bounds: min={body.bounds().min}, max={body.bounds().max}")
    print(f"Lipschitz constant: {body.lipschitz()}")

    # Evaluate at three points: center, surface, outside.
    pts = np.array([
        [0.0, 0.0, 0.0],   # inside  → -1
        [1.0, 0.0, 0.0],   # surface → 0
        [2.0, 0.0, 0.0],   # outside → +1
    ], dtype=np.float32)

    out = CAD_0.sdf.evaluate(body, pts)
    print(f"\nEvaluated {pts.shape[0]} points:")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
