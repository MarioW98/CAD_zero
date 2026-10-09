# Robust Geometric Predicates (Shewchuk 1997)

Public-domain adaptive precision arithmetic for `orient2d`, `orient3d`,
`incircle`, `insphere`. Used by CAD_0 for robust B-Rep boolean
operations and triangulation.

- **Original source**: http://www.cs.cmu.edu/~quake/robust.html
- **License**: Public Domain (cited in source)
- **Wrapped by**: `core/math/src/predicate.cpp`

DO NOT MODIFY. Upgrades are tracked in `CHANGELOG.md`.

In Phase A we ship a stub `predicates.c.h` so the build is functional
without the full Shewchuk source. To install the real predicates:

```bash
cd third_party/robust_predicates
curl -O https://www.cs.cmu.edu/~quake/robust.c
mv robust.c predicates.c.h
```
