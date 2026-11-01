# Contributing to CAD_0

Thank you for your interest in contributing to CAD_0! This document
describes how to set up your environment and the conventions we follow.

## Build prerequisites

- C++20 compiler — the CI-tested columns are **GCC 14** and **Clang 19**
  (the `linux-*` presets pin the versioned binaries `g++-14` /
  `clang++-19`, so a preset build IS the toolchain the CI judges).
  Older compilers (GCC ≥ 11, Clang ≥ 14) still build through a plain
  `cmake -B build` invocation, but are not CI-tested anymore.
- Windows / MSVC: **registered, not CI-tested** — see
  [docs/audits/2026-11-01-ci-runner-asan-and-attestations.md](docs/audits/2026-11-01-ci-runner-asan-and-attestations.md)
  (the registered unit and its price list).
- CMake ≥ 3.22
- Python ≥ 3.10 (for bindings and tests; the CI byte-compiles the tree
  under a real 3.10 interpreter — the syntax floor is a gate, not a hope)
- Ninja (recommended, not required on Windows)

## Quick setup

```bash
git clone https://github.com/MarioW98/CAD_zero.git
cd CAD_zero

# Configure (Linux)
cmake --preset linux-gcc-release

# Build
cmake --build --preset linux-gcc-release

# Test
ctest --preset linux-gcc-release
```

## Coding standards

- **C++**: C++20, no extensions. `.clang-format` is authoritative.
  There is no format gate in CI — formatting is held by review, and
  the gap is said, not hidden.
- **Python**: ruff + black, configured in `pyproject.toml` (installed
  by the `dev` extra). mypy ships in the `dev` extra too, but no lint
  or type gate runs in CI yet — when one lands, this sentence changes
  in the same commit.
- **Headers**: every public header lives under
  `core/<module>/include/CAD_0/<module>/`. Internal headers (no
  public API) live under `core/<module>/src/`.
- **Includes**: relative includes use `#include "CAD_0/..."` (project
  root). Third-party includes use `#include <...>`.

## ADRs (Architecture Decision Records)

Any non-trivial design decision requires an ADR under `docs/adr/`.
Format: `NNNN-short-slug.md` (4-digit zero-padded, kebab-case).

## Testing

- Unit tests live under `tests/unit/` and use `doctest`, compiled
  into the single `CAD_0_tests` executable.
- Property-based families live under `tests/property/`: seeded
  generators shared by the per-area test TUs.
- Golden files (byte-identical `.CAD_0` documents, one per format
  version) live under `tests/golden/` and are consumed by the
  round-trip tests.
- The Python bindings suite is a ctest member (`CAD_0_python_bindings`,
  via `scripts/run_python_bindings_suite.py`): in the sanitizer build
  it runs with the ASan runtime preloaded — the most exposed surface
  under the strongest instrumentation.

A PR is not mergeable until `ctest` passes on the Linux CI matrix (the
GCC and Clang columns of `.github/workflows/ci-linux.yml`). Windows is
not CI-tested: the port is registered as its own unit — see
[docs/audits/2026-11-01-ci-runner-asan-and-attestations.md](docs/audits/2026-11-01-ci-runner-asan-and-attestations.md).
No sentence in this repo may claim a platform the CI does not testify.

## The two version axes

This project carries **two independent version numbers**, and every
source that mentions a version speaks exactly one of them:

| axis | today | spoken by |
|---|---|---|
| **Software** (the code shipping as a unit) | 0.1.0 | `project(VERSION ...)` in `CMakeLists.txt`, `version` in `pyproject.toml`, `CAD_0.__version__`, **git tags** (`v0.1.0-alpha`), [CHANGELOG.md](CHANGELOG.md) in the root |
| **Format** (the `.CAD_0` document format) | v0.4.0 | `kNativeVersion` in `core/io/include/CAD_0/io/native.hpp`, the [ADR-0007](docs/adr/0007-native-format-versioning.md) series, the migrator chain, the `native_v0_x_0.CAD_0` goldens |

The rules:

- **Git tags speak the software; `v0.x` bumps speak the format.** The
  coincidence of both starting at 0.1.0 is historical, not a coupling.
- The two axes move independently, each with its own changelog: the
  software's is [CHANGELOG.md](CHANGELOG.md) + the git history; the
  format's is the ADR-0007 series (every bump has its ADR and its
  migrator).
- No source may present one axis's number as the other's.

No bump of either axis happens casually: the format's v0.5.0 is already
reserved for the fillet unit (ADR-0036 D6).

## Where things live — the register criterion

The repo is the authority on itself, so every artifact has exactly one
house. The criterion: **the register lives in `docs/`, unless a
universal convention pins it to the root**.

- **`docs/` is the register**: `docs/adr/` (decisions),
  `docs/audits/` (dated verification records), `docs/status/` (the
  living status register and the cronaca), `docs/history/` (archived
  chronicles), `docs/architecture/`, and the operational manuals
  (`docs/istruzioni.md` — build and packaging; `docs/CODESPACES.md`).
- **The root keeps only what a universal convention puts there**:
  `README.md` (the vetrina), `LICENSE`, `NOTICE`, `CHANGELOG.md`, this
  `CONTRIBUTING.md`, `pyproject.toml`, `CMakeLists.txt`,
  `CMakePresets.json`, and the tool dotfiles (`.clang-format`,
  `.editorconfig`, `.gitignore`). A root file needs a universal
  convention to justify itself — nothing else grows there.
- **`scripts/` is tooling, never the register**: checkers, launchers,
  the build helper, the suite runner. The generator is tooling; the
  artifact it generates is register — two houses, never confused
  (`scripts/record_demo.py` vs `CHANGELOG.md`).
- **`third_party/` is vendored source**: pinned in the tree with a
  provenance header, never fetched at build time.
- **Build-time dependencies are pinned and registered**: every pin
  and the pin-vs-vendor decision lives in
  [docs/dependencies.md](docs/dependencies.md); `CMakeLists.txt` is
  the source of truth the manifest describes.

Release notes were the case that decided the criterion: they lived in
`scripts/` (the tooling house) until Consegna B m2 (2026-11-01) moved
them to the root `CHANGELOG.md` — the one register convention that is
universal enough (tools and users look for it there) to live at the
root. `FIXES_BUG3_4_5.md` went the other way: root-born delivery
cover note, archived in `docs/history/` with its content intact.

## Commit messages

We follow the Conventional Commits spec:

```
feat(sdf): add torus primitive with sharp feature preservation
fix(geometry): correct Shape::as_hybrid for moved-from state
docs(adr): add ADR-0015 for voxel memory layout
```

### Subject lint (enforced)

The subject line is enforced. CI judges exactly the commits you push
(`subject-lint` job in `.github/workflows/ci-linux.yml`); a
non-conforming subject turns the run red *after* the push, when the
commit is already immutable on `main`. To catch it at the source, install
the local hook (once per clone) — it runs the same versioned checker at
commit time, so a bad subject is rejected before the commit exists:

```bash
sh scripts/install_git_hooks.sh
```

A subject is conforming when it is not a bare tag or file path, does not
start with an empty marker (`fix`, `various`, `check`, ... — Italian
equivalents included: `correzione`, `sistemazione`, `modifica`,
`aggiornamento`) or a progress label (`intro`, `bozza`, `riepilogo`,
`Unità N` — the phase is not the content), and is at least 15
characters.

Since H2 (Parte C), the subject also must NOT contain a delivery
counter (`Commit 1/4` — the position in a delivery is not a content),
a git SHA (git identifies; the subject describes — `0x`-prefixed hex
literals excluded, they are data, not references), a prompt-title
banner (runs of `=`/`═` — the subject is one line, not a pasted
manifest), or SHOUTING (more than half the letters uppercase). And it
must carry the conventional type prefix — `feat(brep): ...`,
`fix(tests): ...`, `docs: ...` — with the scope REQUIRED for the
code-touching types (`feat`, `fix`, `test`, `refactor`, `perf`) and
optional for the registry types (`docs`, `chore`, `ci`, `build`,
`style`, `revert`). The machine refuses a non-conforming subject
BEFORE the commit exists (the hook) and judges the pushed range in CI.

**History note (annotated, not rewritten).** The deliveries that
landed with non-conforming subjects — `Commit 1/4 — c851bdc · docs
(PARTE A) ...`, `g2 - dimenticanza file`, and the pre-lint era's
markers — stay in history exactly as they happened: the house does
not rewrite pushed history. The extended lint applies to NEW commits
from this one forward; `check_commit_subject.py --all` remains a
report (not a gate) for exactly this reason.

The derivation of every rule lives in the checker's
docstring; preview a subject without touching history with
`python3 scripts/check_commit_subject.py --explain "..."`.
Merge commits are skipped both here and in CI (their subject is
generated by git, not chosen by the author).

## Pre-delivery local verification (mandatory)

CI is the last line of defense, not the first. Every delivery — code,
docs, workflow, goldens, no exception for "it's only a comment" — passes
the complete local verification *before* pushing:

1. **The full suite.** C++ and Python tests on the surface CI judges,
   with the compilers the CI judges (the pinned presets: gcc 14 and
   clang 19): the sanitizer build (`linux-gcc-debug` carries
   ASan/UBSan, the `-Werror` contract of the kernel targets AND the
   Python bindings suite under the preloaded sanitizer — the
   `CAD_0_python_bindings` ctest member), the release build, the clang
   build, and the golden files untouched:

   ```bash
   cmake --preset linux-gcc-debug && cmake --build --preset linux-gcc-debug \
     && ctest --preset linux-gcc-debug
   cmake --preset linux-gcc-release && cmake --build --preset linux-gcc-release \
     && ctest --preset linux-gcc-release
   cmake --preset linux-clang && cmake --build --preset linux-clang \
     && ctest --preset linux-clang
   git status --porcelain tests/golden/   # empty output = goldens intact
   ```

   The bindings suite runs inside each `ctest` above (its launcher
   preloads the ASan runtime in the instrumented build); the GL-bound
   UI pair runs under Xvfb as the workflow does. The Python syntax
   floor (3.10) is checked by `python3 scripts/check_fstring_backslashes.py`
   locally and by the CI's `python-3-10-floor` job (byte-compilation
   under a real 3.10 interpreter).

2. **The subject checker, on the exact subject of the commit you are
   about to deliver** — not a draft of it, not the branch name, the
   subject as it will be pushed:

   ```bash
   python3 scripts/check_commit_subject.py --explain "<subject>"
   ```

   A delivery whose subject does not pass its own checker is not a
   delivery. This is the self-demonstration rule: the commit that fixes
   a red run cannot carry the same defect that made the run red.

   The checker ships with its own regression test, driven through the
   same `--explain` surface — run it whenever the checker itself
   changes:

   ```bash
   python3 scripts/test_check_commit_subject.py
   ```

3. **The hook, installed once per clone.** The two checks above are
   what you run; the hook is what runs even when you forget:

   ```bash
   sh scripts/install_git_hooks.sh
   ```

   From then on a non-conforming subject is rejected at `git commit`
   time — it never exists, so it can never reach CI. Squashing and
   rebasing create new commits with hand-written subjects: the hook
   judges those too, using the versioned checker from the tree being
   committed.

Runs #91, #92 and #93 were all red for the same single reason: a commit
subject that a five-second local check would have rejected ("Unità 2
sketch" — 14 characters; "unità 2 fix e unità 3" — a progress label;
"correzione" — an Italian empty marker that only the length rule
caught, and only by chance). The code in all three runs was green:
build, tests, bindings, UI smoke on all three compilers. The only
defect was the subject itself, and the push made it immutable. Had
this section existed and been followed, none of those runs would have
happened.

## Branches

- `main` — stable, always builds, always passes tests.
- `feature/<name>` — short-lived feature branches.
- `experimental/<name>` — long-lived research branches; no merge
  guarantees.

## Licensing

By contributing, you agree that your contributions are licensed under
the Apache-2.0 license. If your contribution includes third-party code,
declare it in the PR description and ensure compatibility with
ADR-0013.
