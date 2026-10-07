# ADR-0003: Third-party policy — no CAD kernels

## Status
Accepted (2025-10-07)

## Context
The proposal forbids third-party CAD kernels (OpenCascade, Parasolid,
ACIS) and allows computational geometry libraries (CGAL, OpenVDB,
Shewchuk predicates) on a case-by-case basis.

## Decision

### Forbidden
Any kernel that ships a full B-Rep / NURBS / boolean stack:
- OpenCascade (OCCT)
- Parasolid
- ACIS / Spatial
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

## License of cadforge itself
Apache-2.0 (see `LICENSE`). Compatible with all the "Allowed" libraries
above.
