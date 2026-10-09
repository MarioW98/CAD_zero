# ADR-0019: Project rename cadforge → CAD_0

## Status
Accepted (2025-10-07)

## Context
The project was originally named "cadforge" during the initial
prototyping phases (Phase 0 through Phase C). As the project moves
toward a more formal structure, the name has been changed to "CAD_0"
to reflect a fresh identifier.

## Decision
**All references to "cadforge" are renamed to "CAD_0".**

### Naming convention
| Context | Convention | Example |
|---|---|---|
| Project directory | `CAD_0/` | `/home/z/my-project/CAD_0/` |
| C++ namespace | `CAD_0::` | `CAD_0::math::Vec3f` |
| C++ include path | `CAD_0/...` | `#include "CAD_0/math/vec.hpp"` |
| CMake target | `CAD_0_<module>` | `CAD_0_math`, `CAD_0_sdf` |
| CMake macro prefix | `CAD_0_` | `CAD_0_BUILD_PYTHON`, `CAD_0_USE_TBB` |
| Python package | `CAD_0` | `import CAD_0` |
| Python compiled extension | `_CAD_0` | `from CAD_0 import _CAD_0` |
| Generated config header | `CAD_0/config.h` | (from `cmake/CAD_0_config.h.in`) |

### Files affected
- 133 source files modified by an automated rename script
- 11 directories renamed from `cadforge/` to `CAD_0/` (under `core/*/include/`, `app/src/`, `bindings/python/`)
- 1 generated config file renamed: `cmake/cadforge_config.h.in` → `cmake/CAD_0_config.h.in`
- 1 CMake module name updated: `_cadforge` → `_CAD_0` (nanobind extension)
- All Python `__init__.py` files updated to reference `_CAD_0` instead of `_cadforge`

### Trade-offs
- **Python module name with uppercase + underscore**: violates PEP 8 (which recommends lowercase), but is fully functional. The choice is consistent with the project name and avoids ambiguity.
- **C++ namespace with uppercase + underscore**: legal C++, no issues with compilers.
- **CMake target names**: mixed case is allowed; the underscore in `CAD_0_math` could collide with the prefix `CAD_0_` but the target names are sufficiently distinct.

## Consequences
- The build system, tests, examples, and Python modules all use the new name.
- 165/165 tests still pass after the rename.
- The C++ example `lattice_ball.stl` still produces identical output.
- The Python package can be imported as `import CAD_0` (once nanobind is configured).

## Migration notes for users
- If you have a previous checkout named `cadforge/`, rename it to `CAD_0/`.
- All `#include "cadforge/..."` lines become `#include "CAD_0/..."`.
- All `cadforge::` namespace references become `CAD_0::`.
- The Python package changes from `import cadforge` to `import CAD_0`.

## Future work
- The name "CAD_0" may change again in the future; this ADR documents the
  pattern for any future rename.
- A `CHANGELOG.md` entry should be added tracking this rename.
