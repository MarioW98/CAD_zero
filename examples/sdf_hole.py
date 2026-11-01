"""Example: hybrid model — sphere minus cylinder (a hole through a ball).

Shows the native SDF CSG: subtract (boolean difference). The
module-level `body` makes it a CLI input (`CAD_0 info examples/sdf_hole.py`).
"""

from __future__ import annotations

import numpy as np

import CAD_0

# A sphere of radius 2 with a cylinder of radius 0.5 (along Y, height
# 10) subtracted: ball - cyl.
_ball = CAD_0.sdf.sphere(radius=2.0)
_cyl = CAD_0.sdf.cylinder(radius=0.5, height=10.0)
body = CAD_0.sdf.subtract(_ball, _cyl)


def main() -> None:
    print(f"Body: {body.describe()}")
    print(f"Lipschitz: {body.lipschitz()}")

    # Sample along the Y axis: at y=0 (center of ball, inside cylinder)
    # we should be OUTSIDE the result (cylinder carved out the material).
    pts = np.array([
        [0.0, 0.0, 0.0],  # inside cylinder — should be outside the carved result
        [1.5, 0.0, 0.0],  # outside cylinder, inside sphere — should be inside
        [3.0, 0.0, 0.0],  # outside sphere — should be outside
    ], dtype=np.float32)

    out = CAD_0.sdf.evaluate(body, pts)
    print("\nSampled values:")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
