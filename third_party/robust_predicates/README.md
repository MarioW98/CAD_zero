# Robust Geometric Predicates — Shewchuk, authentic and vendored

`predicates.c.h` is **the authentic `robust.c` by Jonathan Richard
Shewchuk** (revision of May 18, 1996) — adaptive precision floating-point
arithmetic and fast robust geometric predicates:

- `orient2d`, `orient3d`, `incircle`, `insphere` — the adaptive,
  robust versions (exact sign, always);
- the `*fast`/`*exact`/`*slow` companions of each — Shewchuk's own
  non-robust / exact-arithmetic / slow-reference variants;
- the expansion arithmetic machinery (`grow_expansion`, `scale_expansion`,
  `fast_expansion_sum`, ...) the adaptive filters are built on.

## Provenance

- **Source**: http://www.cs.cmu.edu/~quake/robust.c — the canonical URL no
  longer resolves (404 at vendoring time, 2026-10-29). The text was taken
  from the Insight Software Consortium's faithful mirror
  (`ITKRobustPredicate`, file `src/predicates.c`) — same revision, one
  mirror-local addition (an `#include "predicates.h"`), removed here.
- **License**: Public Domain — the author's own declaration, in the file
  header. Compatible with the ADR-0003 license whitelist (public domain:
  no usage limits). See also [NOTICE](../../NOTICE).
- **Local modifications**: exhaustive and explicit, listed in the
  provenance header at the top of the file:
  1. every K&R-style function definition converted to an ANSI prototype
     (C++ has no old-style definitions) — mechanical, all 36 functions
     listed in the header;
  2. the mirror's `#include "predicates.h"` removed;
  3. the include guard and `extern "C"` wrapper added (the house
     consumption pattern: `core/math/src/predicate.cpp` includes this
     file inside one `extern "C"` block, one definition TU).

**Nothing else was touched.** The tracked file is the truth — never
overwrite it by hand and never `curl` over it: this README's predecessor
instructed exactly that (a supply-chain hazard, since removed), and the
file it told you to fetch was the same public-domain text that is now
committed here, byte-accounted for by the provenance header.

## The tests that prove the vendoring

`tests/unit/math/test_predicate_adaptive.cpp` pins the classic
near-degenerate families — nearly-collinear points at ulp scale,
nearly-coplanar `orient3d`, nearly-cocircular `incircle` — each with the
exact vectors that build it, the adaptive predicate's pinned sign, and
the **negative block**: the same inputs through a mathematically-correct
naive double determinant, CHECKED to fail (wrong sign or a false zero)
where the adaptive passes. The discrimination is the proof the tree
carries the real thing: a non-adaptive impostor cannot pass it.

History, for the record (ADR-0034): until H1 this directory carried a
hand-written non-adaptive *stub* that imitated Shewchuk's structure, and
this README (then instructing a `curl`-overwrite) called it vendored.
The stub's `orient3d` also carried a sign error in its middle term
(`bdx·(ady·cdz − adz·cdy)` instead of Shewchuk's
`bdx·(cdy·adz − cdz·ady)`) — the unit test that pinned its output pinned
the bug. Both are gone.
