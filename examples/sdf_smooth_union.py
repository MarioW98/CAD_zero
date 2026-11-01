"""Example: smooth union of two spheres (organic blend).

Demonstrates smooth_min (polynomial smin) — useful for lattice / TPMS
design, where sharp boolean edges are undesirable. The module-level
`body` makes it a CLI input (`CAD_0 info examples/sdf_smooth_union.py`).
"""

from __future__ import annotations

import numpy as np

import CAD_0

_a = CAD_0.sdf.sphere(radius=1.0)
# translate takes a Vec3f offset, not a tuple (the bindings do not
# auto-convert — the original tuple call raised TypeError; the example
# had never actually run until the Consegna B m2 venv-virgin proof).
_b = CAD_0.sdf.translate(CAD_0.sdf.sphere(radius=1.0),
                         CAD_0.math.Vec3f(2.0, 0.0, 0.0))
body = CAD_0.sdf.smooth_union(_a, _b, k=0.6)


def main() -> None:
    print(f"Body: {body.describe()}")

    # Sample along the X axis between the two spheres.
    pts = np.array([
        [0.0, 0.0, 0.0],  # center of sphere A → -1
        [1.0, 0.0, 0.0],  # midpoint — smooth_min makes this < hard_min
        [2.0, 0.0, 0.0],  # center of sphere B → -1
    ], dtype=np.float32)

    out = CAD_0.sdf.evaluate(body, pts)
    print("\nSmooth union values (k=0.6):")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
