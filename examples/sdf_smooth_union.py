"""Example: smooth union of two spheres (organic blend).

Demonstrates smooth_min (polynomial smin) — useful for lattice / TPMS
design, where sharp boolean edges are undesirable.
"""

from __future__ import annotations

import numpy as np

import cadforge


def main() -> None:
    a = cadforge.sdf.sphere(radius=1.0)
    b = cadforge.sdf.translate(cadforge.sdf.sphere(radius=1.0), (2.0, 0.0, 0.0))

    smooth = cadforge.sdf.smooth_union(a, b, k=0.6)
    print(f"Body: {smooth.describe()}")

    # Sample along the X axis between the two spheres.
    pts = np.array([
        [0.0, 0.0, 0.0],  # center of sphere A → -1
        [1.0, 0.0, 0.0],  # midpoint — smooth_min makes this < hard_min
        [2.0, 0.0, 0.0],  # center of sphere B → -1
    ], dtype=np.float32)

    out = cadforge.sdf.evaluate(smooth, pts)
    print("\nSmooth union values (k=0.6):")
    for i, v in enumerate(out.values):
        print(f"  pt={pts[i].tolist()}  sdf={v:.6f}")


if __name__ == "__main__":
    main()
