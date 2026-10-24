# ADR-0003: Third-party policy — no CAD kernels

## Status
Accepted (2025-10-07). **Amended 2026-10-25** — the license
whitelist pinned by the user at ADR-0031 acceptance (see
§ Amendment at the end of this document).

## Context
The proposal forbids third-party CAD kernels and allows
computational geometry libraries (CGAL, OpenVDB,
Shewchuk predicates) on a case-by-case basis.

## Decision

### Forbidden
Any kernel that ships a full B-Rep / NURBS / boolean stack:
- Any proprietary or open-source third-party CAD kernel
- Any SaaS-licensed geometry engine

These would replace, not augment, the native kernel. Their inclusion
would compromise the dual-representation design (ADR-0002).

### Allowed (with ADR or runtime note)
| Library | License | Use | Phase |
|---------|---------|-----|-------|
| Shewchuk robust predicates | Public Domain | `orient2d/3d`, `incircle` | Phase A — vendored under `third_party/robust_predicates/` |
| oneTBB | Apache-2.0 | Thread pool, task graph, parallel_for | Phase A — see ADR-0009 |
| xsimd | BSD-3-Clause | SIMD abstraction for SDF evaluation | Phase A — see ADR-0008 |
| fmt | MIT | Logging + formatting | Phase A |
| spdlog | MIT | Async logging | Phase A |
| doctest | MIT | Unit testing | Phase A |
| nanobind | BSD-3-Clause | Python bindings | Phase A — see ADR-0011 |

### Allowed with restrictions
- **CGAL**: only the headers under `boost`-compatible licenses. The
  `GPLv3+` parts are incompatible with our Apache-2.0 license and
  must NOT be linked. ADR-0013 will document the per-module license
  map before any CGAL header is included.
- **OpenVDB**: MPL-2.0, file-level copyleft. Compatible with Apache-2.0
  but requires care when distributing binary wheels.

## Consequences
- The vendored Shewchuk predicates ship under public domain.
- CGAL is the most permissive dependency we will ever allow, and only
  for header-only utilities. If a CGAL module requires linking against
  a GPLv3+ shared object, the integration is rejected.
- All third-party additions require a companion ADR before merging.

## License of CAD_0 itself
Apache-2.0 (see `LICENSE`). Compatible with all the "Allowed" libraries
above.

## Amendment (2026-10-25) — the license whitelist, pinned by the user

At the acceptance of ADR-0031 (the exact B-Rep kernel fork — decided:
road (a), all in house, forever), the user pinned this policy's
license constraint:

> terze parti SOLO con licenze senza limiti d'uso (pubblico
> dominio, MIT, BSD, Apache-2.0, zlib, ISCL...). GPL / LGPL /
> con-eccezioni: FUORI.

In policy terms: **third-party components are allowed only under
licenses without usage limits** — public domain, MIT, BSD,
Apache-2.0, zlib, ISC, and the permissive family. **GPL, LGPL,
and licenses with exceptions are out**: no linkage, no vendoring,
now and in any future phase. This closes, as policy, the road
ADR-0031 D1(b) priced: an OCCT (LGPL-2.1 + exception) adoption
would violate the whitelist regardless of any future cost-benefit
reassessment.

Mechanical consequences on the tables above:

- Every currently allowed row already satisfies the constraint
  (Shewchuk public domain; oneTBB Apache-2.0; xsimd and nanobind
  BSD-3-Clause; fmt, spdlog, doctest MIT). Nothing in the tree
  changes.
- **CGAL** — was "allowed with restrictions" (Boost-licensed
  headers only). Under the whitelist its mixed licensing
  (GPL-3+ packages alongside the permissive ones) keeps it out
  entirely. No CGAL integration was ever merged; nothing changes
  in the tree. ADR-0013's CGAL row is superseded by this
  amendment.
- **OpenVDB** — was "allowed with restrictions". MPL-2.0 is
  file-level copyleft: a usage limit (modifications to MPL files
  must be released). Under the whitelist it moves from
  "allowed with restrictions" to **out**. It was never merged
  either; nothing changes in the tree. ADR-0013's OpenVDB row
  is superseded by this amendment.

Boundary note, flagged and NOT decided here: the UI runtime
PySide6 (LGPL-3.0, the `[ui]` extra) lives under ADR-0006 /
ADR-0013 / NOTICE jurisdiction — dynamically imported,
replaceable, the C++ kernel links no Qt — and is not part of
this policy's dependency table. Whether the whitelist
principle's scope is the kernel and its dependencies (PySide6's
documented LGPL compliance regime continues) or extends to the
UI shell itself (a future migration decision) is a maintainer
decision this amendment does not take; it is recorded here so
the question is visible rather than silently scoped away.
