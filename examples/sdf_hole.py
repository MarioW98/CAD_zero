"""Example: hybrid model — sphere minus cylinder (a hole through a ball).

Shows the native SDF CSG: subtract (boolean difference) and translate.
"""

from __future__ import annotations

import numpy as np

import cadforge


def main() -> None:
    # A sphere of radius 2
    ball = cadforge.sdf.sphere(radius=2.0)
    # A cylinder of radius 0.5 along Y, height 10
    cyl = cadforge.sdf.cylinder(radius=0.5, height=10.0)
    # Subtract: ball - cyl
    body = cadforge.sdf.subtract(ball, cyl)
    print(f"Body: {body.describe()}")
    print(f"Lipschitz: {body.lipschitz()}")

    # Sample along the Y axis: at y=0 (center of ball, inside cylinder)
    # we should be OUTSIDE the result (cylinder carved out the material).
    pts = np.array([
        [0.0, 0.0, 0.0],  # inside cylinder — should be outside the carved result
        [1.5, 0.0, 0.0],  # outside cylinder, inside sphere — should be inside
        [3.0, 0.0, 0.0],  # outside sphere — should be outside
    ], dtype=np.float32)

    out = cadforge.sdf.evaluate(body, pts)
    print("\nSampled values:")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
