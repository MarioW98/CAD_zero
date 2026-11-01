# The dependency register

Every dependency CAD_0 builds on or ships with, its exact version,
and the decision that keeps it there. This is the register; the
**source of truth for the CPM pins is `CMakeLists.txt`** — this
file describes and explains, it does not define. The license
inventory lives in [NOTICE](../NOTICE); the per-component flags in
[docs/audits/2026-10-16-third-party-license-audit.md](audits/2026-10-16-third-party-license-audit.md).

Consegna B m2 (2026-11-01) derived and applied the decision
recorded here; ADR-0014's determinism promise starts at the
dependency chain — a build six months from now must produce the
same bytes as a build today.

## The decision: pin + cache, vendor only what is small and stable

The house criterion (the H1 lesson, applied in both directions):

- **Small and stable → vendor it in `third_party/`**, source in the
  tree, build never touches the network. The precedent: Shewchuk's
  robust predicates — small (one translation unit), frozen by
  history (revision of May 18, 1996, public domain), vendored by H1
  with a provenance header and discrimination tests
  (ADR-0034).
- **Big and alive → pin + cache**: the exact tag in `CMakeLists.txt`
  (`GIT_TAG`), the sources cached (`CPM_SOURCE_CACHE`), the build
  touching the network only to fill or refresh that cache — and
  always on pinned versions.

All six CPM dependencies are big and alive; none is a vendor
candidate. The decision is priced, not felt:

| dependency | pin (`GIT_TAG`) | fetched when | license | why not vendored |
|---|---|---|---|---|
| fmt | `10.2.1` | always | MIT-style | a large, actively developed library — vendoring forks it under our maintenance forever |
| spdlog | `v1.14.1` | always | MIT | same class: big, alive, upstream fixes security and platform bugs |
| xsimd | `12.1.1` | always | BSD-3-Clause | header-only but broad and on a moving train (new ISA support) — a fork would rot |
| doctest | `v2.4.12` | `CAD_0_BUILD_TESTING` (ON by default, OFF in wheels) | MIT | test-only; v2.4.12 is the CMake-4.x-compat pin — following upstream is the point of not forking |
| nanobind | `v2.2.0` | `CAD_0_BUILD_PYTHON` | BSD-3-Clause | it is a whole build system, alive; pinning rides its fixes, forking inherits its bugs |
| oneTBB | `v2022.0.0` | `CAD_0_USE_TBB=ON` — opt-in, OFF on every shipped surface (ADR-0009 amendment) | Apache-2.0 | big; and it ships nowhere — vendoring would carry dead weight into every clone |

Vendored in-tree (no CPM, no network ever):

| source | version | house | license |
|---|---|---|---|
| CPM.cmake | 0.40.2 | `cmake/CPM.cmake` | MIT |
| Shewchuk robust predicates (`robust.c`) | rev. May 18, 1996 | `third_party/robust_predicates/` | Public Domain |

## The pip axes

| axis | constraint | floating? | why |
|---|---|---|---|
| build (`[build-system]`) | `scikit-build-core>=0.10.0`, `nanobind>=2.2.0` | within the floor — the honest note: the PyPI side is a range, not a pin | build tooling resolves at build time against PyPI; the *geometric* reproducibility lives in the CMake/CPM pins, the nanobind **CMake module** used by the build is the CPM pin `v2.2.0` above |
| runtime (base) | `numpy>=1.24`, `matplotlib>=3.5` | floor-bounded | the installed library tracks the user's environment; the wheel declares what it needs, not what it freezes |
| extras | `ui`: PySide6, PyOpenGL · `cli`: typer, rich · `dev`: pytest, pytest-xdist, mypy, ruff, black | floor-bounded | optional faces, optional weight — the `[cli]` entry point degrades with a message when the extra is absent |

Two different nanobinds appear above, on purpose: the PyPI package
(build requirement, version-ranged) and the CPM source (the CMake
modules that actually build the extension, pinned `v2.2.0`). A
fully-offline pip build from an sdist is therefore NOT part of the
promise — the offline guarantee below is about the **CMake/CPM
dependency chain**; the pip axis needs PyPI reachable (or a local
wheelhouse) at build time. Said, not hidden.

## The cache policy

- **CI** (`.github/workflows/ci-linux.yml`, Consegna B m2): every
  `build-and-test` leg exports
  `CPM_SOURCE_CACHE="$HOME/.cache/CPM"` for the Configure step and
  restores/saves it with `actions/cache`, keyed on
  `hashFiles('CMakeLists.txt')` — the pins live in that file, so a
  pin bump invalidates the cache by construction; `restore-keys`
  let a bumped build reuse everything the bump did not move. The
  first configure after a pin change fetches only the delta.
- **Local**: `export CPM_SOURCE_CACHE="$HOME/.cache/CPM"` once per
  shell (or per preset invocation) and every configure afterwards
  hits the cache instead of the network.

## The offline build — demonstrated, not promised

CPM.cmake itself is vendored (`cmake/CPM.cmake`), so with a
populated cache nothing in configure or build needs the network:

```bash
# 1. populate the cache once (online)
CPM_SOURCE_CACHE=/tmp/cpm-cache cmake --preset linux-gcc-release

# 2. prove the offline configure + build (network black-holed)
env http_proxy=http://127.0.0.1:1 https_proxy=http://127.0.0.1:1 \
    CPM_SOURCE_CACHE=/tmp/cpm-cache \
    cmake -B build-offline -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_COMPILER=g++-14 \
      -DCAD_0_BUILD_PYTHON=ON -DCAD_0_BUILD_TESTING=ON \
      -DPython3_EXECUTABLE="$(which python3)"
env http_proxy=http://127.0.0.1:1 https_proxy=http://127.0.0.1:1 \
    CPM_SOURCE_CACHE=/tmp/cpm-cache cmake --build build-offline
```

The proof of 2026-11-01 (Consegna B m2): a fresh configure + full
build with the proxies pointed at a closed port completed on cache
hits alone — every CPM line read `using cached package`, the build
never touched the network. The five always-fetched dependencies
(fmt, spdlog, doctest, xsimd, nanobind) were all served from
`/tmp/cpm-cache`.

## The six-months question

What changes in six months if the pins hold: nothing. The residual
risks, priced:

- **A tag moves upstream** (rare, but a tag is mutable): the cache
  would keep serving the old bytes until a pin bump or a cache miss
  exposes the new ones. The mitigation is visibility — the pins are
  few, listed here, and every bump is a reviewed commit.
- **A repo disappears or rewrites history**: the cache keeps the
  build working locally; CI would go red at the next cold cache.
  The escalation path is vendoring — priced above, refused for now,
  revisitable per dependency the day one goes dark.

That is the whole trade: pin + cache buys reproducibility with
~zero maintenance; vendoring buys network-independence at the price
of forking six living projects. The house takes the first, keeps
the second loaded for what is small and stable.
