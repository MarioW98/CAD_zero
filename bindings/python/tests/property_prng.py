"""Seeded, deterministic PRNG for the property-based Python tests.

Mini-traccia PARTE 1 infrastructure (test-only, zero new dependencies):
the seed is part of the test — never the clock, never entropy — and the
failure diagnosis carries seed + iteration + parameters + the violated
check, so a failure is a reproducible configuration.

random.Random's Mersenne-Twister core and the methods used here
(random / uniform via frange / randrange / getrandbits) are stable
across CPython versions; only these are used (no gauss, no sample).
"""

from __future__ import annotations

import random
from typing import Any, Dict


class SeededPrng:
    """Deterministic seeded PRNG with the family's helpers."""

    def __init__(self, seed: int) -> None:
        self.seed = seed
        self._rng = random.Random(seed)

    def unit(self) -> float:
        """Uniform in [0, 1)."""
        return self._rng.random()

    def frange(self, lo: float, hi: float) -> float:
        """Uniform float in [lo, hi)."""
        return lo + (hi - lo) * self._rng.random()

    def int_below(self, n: int) -> int:
        """Uniform integer in [0, n)."""
        return self._rng.randrange(n)

    def coin(self) -> bool:
        return bool(self._rng.getrandbits(1))


def case(seed: int, iteration: int, params: Dict[str, Any], check: str) -> str:
    """The failure diagnosis format: rerun with this seed to reproduce."""
    return (
        f"seed={seed} iter={iteration} params={params} "
        f"violated check: {check} - rerun with this seed to reproduce"
    )
