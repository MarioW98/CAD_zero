# ADR-0013: License compatibility with dependencies

## Status
Accepted (2025-10-07)

## Context
CAD_0 ships under Apache-2.0. Not all open-source licenses are
compatible. The two main risk areas are CGAL (mixed licenses) and
OpenVDB (file-level copyleft).

## Decision

### License compatibility table

| Dependency | License | Compatible? | Notes |
|-----------|---------|-------------|-------|
| fmt | MIT | Yes | No restrictions |
| spdlog | MIT | Yes | No restrictions |
| doctest | MIT | Yes | No restrictions |
| oneTBB | Apache-2.0 | Yes | Same license as CAD_0 |
| xsimd | BSD-3-Clause | Yes | BSD-3 is Apache-2.0-compatible |
| nanobind | BSD-3-Clause | Yes | BSD-3 is Apache-2.0-compatible |
| Shewchuk predicates | Public Domain | Yes | No restrictions |
| CGAL (header-only, Boost-licensed parts) | Boost-1.0 / GPL-3+ | **Mixed** | Only `LGPL`/`GPL` headers may be excluded; some are dual-licensed. See `third_party/cgal/USAGE.md` (to be created in Phase D). |
| OpenVDB | MPL-2.0 | Yes | File-level copyleft; modifications to OpenVDB source must be released. Use as a *library* (no source modification) is unrestricted. |

### Forbidden combinations
- Linking against any GPL-3+-only library — would force CAD_0 to
  become GPL-3+. Specifically:
  - CGAL's `Boolean_set_operations` package is GPLv3+. We do not link
    it. Our boolean operations are implemented natively.
  - CGAL's `Nef_2`/`Nef_3` packages are GPLv3+. Same.
- Static linking against LGPL-2.1+ libraries without releasing the
  object files — would force CAD_0 to ship object files for every
  released version. Acceptable but operationally heavy; avoid by
  using dynamic linking or by re-implementing the needed utility.

### Distribution license notice
The `LICENSE` file ships with every wheel. The `NOTICE` file
(to be created) lists all bundled dependencies and their licenses.

## Consequences
- CGAL is **deferred** until Phase D. Before including any CGAL header,
  we audit the package's license and document it in `third_party/cgal/USAGE.md`.
- OpenVDB may be added in Phase B (SDF grids) without restrictions.
- The NOTICE file is required for the first public release.
