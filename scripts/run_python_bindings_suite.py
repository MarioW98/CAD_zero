#!/usr/bin/env python3
"""The Python bindings suite as a ctest member — the ASan gap closed.

Consegna B m1 (2026-11-01): the H1 correction report had declared the
gap — the Python bindings (nanobind, the memory bridges: the most
exposed surface of the library) were the least protected, their tests
running WITHOUT AddressSanitizer while the same C++ paths were
instrumented through the doctest suite. This launcher is the closure:
the bindings suite runs inside ctest, in every job of the CI matrix,
and when the extension module is ASan-instrumented the matching
runtime is PRELOADED so the interposed allocator is installed before
the interpreter starts — the same instrumented build the C++ suite
always consumed, now consumed by the Python layer too.

Ground truth, not configuration: whether the module is instrumented
is read from the module itself (its DT_NEEDED on libasan, via ldd).
A divergence between what the build configuration promises and what
the module says FAILS this run loudly: the gap is closed by running
under the sanitizer, never by skipping (a target pulled out of the
suite to make a run green is the gap reopening, wider).

Leak detection is OFF (ASAN_OPTIONS=detect_leaks=0): at interpreter
shutdown CPython keeps its arenas and PySide6 keeps its singletons —
LeakSanitizer would drown the report in noise that is not ours. The
defect classes the gap was declared for — use-after-free, heap
overflow, the bridge lifetimes — stay armed.

The first run of this target FOUND something (2026-11-01, recorded in
docs/audits/2026-11-01-ci-runner-asan-and-attestations.md): under the
preloaded sanitizer, the first C++ exception thrown by the bridge
died with

    AddressSanitizer: CHECK failed: asan_interceptors.cpp:458
        "((__interception::real___cxa_throw)) != (0)"

Python is a C host: it does not load libstdc++, so when the module is
dlopen'd and its first std::invalid_argument crosses the bridge,
ASan's __cxa_throw interceptor has no real symbol to call. The
correction is here, in the launcher: the C++ runtime is preloaded
TOO (read from the module's own DT_NEEDED via ldd — ground truth,
not a guess), so libstdc++ is in the initial link map and the
interceptor binds. The exception's message contract survived the
sanitizer unchanged (test_empty_input_original_exception_message is
the regression: it asserts the ORIGINAL diagnosis text).

The two GL-bound UI tests (test_ui_navigation.py, test_ui_startup.py)
are excluded HERE and run in their own workflow step under Xvfb with
a real GL context (see .github/workflows/ci-linux.yml): they need a
display, not a sanitizer — this is the same split CI always had, not
a coverage loss.

Usage (wired from tests/CMakeLists.txt; not meant to be typed by hand):

    run_python_bindings_suite.py \
        --python-dir <build>/python \
        --tests-dir  <source>/bindings/python/tests \
        [--asan-runtime    <absolute path to libasan.so>] \
        [--asan-compiler   <compiler asked to resolve libasan.so>]

Exit code: 0 only if both the standalone regression suite and pytest
pass; ctest turns anything else red.
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys


def _fail(message: str) -> "NoReturn":  # type: ignore[valid-type]
    print(f"run_python_bindings_suite: FAIL — {message}", file=sys.stderr)
    raise SystemExit(2)


def _find_module(python_dir: str) -> str:
    """Locate the built extension module (fail if the build is missing)."""
    matches = sorted(glob.glob(os.path.join(python_dir, "CAD_0", "_CAD_0*.so")))
    if not matches:
        _fail(
            f"no _CAD_0*.so under {python_dir}/CAD_0 — the bindings module "
            "is not built; this target consumes the built module, it does "
            "not skip when the build is absent"
        )
    return matches[0]


def _module_needs_asan(module: str) -> bool:
    """DT_NEEDED on libasan, read from the module itself (ldd)."""
    return "libasan" in _ldd_output(module)


def _ldd_output(module: str) -> str:
    if shutil.which("ldd") is None:
        _fail("ldd not available — the instrumentation check cannot run")
    proc = subprocess.run(
        ["ldd", module], capture_output=True, text=True, check=False
    )
    if proc.returncode != 0:
        _fail(f"ldd failed on {module}: {proc.stderr.strip()}")
    return proc.stdout


def _ldd_resolve(ldd_out: str, soname: str) -> str:
    """The path ldd resolved for a DT_NEEDED soname ("" if absent)."""
    for line in ldd_out.splitlines():
        parts = [p.strip() for p in line.split("=>")]
        if len(parts) == 2 and parts[0] == soname and os.path.isabs(parts[1]):
            return parts[1]
    return ""


def _resolve_runtime(compiler: str) -> str:
    """Ask the compiler that built the module where its libasan lives."""
    proc = subprocess.run(
        [compiler, "-print-file-name=libasan.so"],
        capture_output=True,
        text=True,
        check=False,
    )
    path = proc.stdout.strip()
    if proc.returncode != 0 or not os.path.isabs(path) or not os.path.isfile(path):
        _fail(
            f"'{compiler} -print-file-name=libasan.so' did not resolve the "
            f"runtime ('{path}'): the instrumented module cannot be imported "
            "without preloading it. Install the matching GCC runtime "
            "(libasan) — the ASan gap is closed by running under the "
            "sanitizer, not by skipping the run."
        )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-dir", required=True,
                        help="build tree dir holding the CAD_0/ package")
    parser.add_argument("--tests-dir", required=True,
                        help="bindings/python/tests source dir")
    parser.add_argument("--asan-runtime", default="",
                        help="libasan.so path resolved at configure time "
                             "(present iff CAD_0_SANITIZE=ON with GNU)")
    parser.add_argument("--asan-compiler", default="g++",
                        help="compiler used to resolve libasan.so")
    args = parser.parse_args()

    module = _find_module(args.python_dir)
    ldd_out = _ldd_output(module)
    instrumented = "libasan" in ldd_out

    # The build configuration and the module must tell the same story.
    if args.asan_runtime and not instrumented:
        _fail(
            f"the configuration says ASan (--asan-runtime given) but the "
            f"module {module} has no libasan dependency — a stale module or "
            "lost sanitizer flags; rebuild the instrumented tree"
        )

    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    # The launcher owns PYTHONPATH so the tests import the module THIS
    # build produced (the ctest property stays empty on purpose).
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in [args.python_dir, env.get("PYTHONPATH", "")] if p
    )

    if instrumented:
        runtime = (
            args.asan_runtime if os.path.isabs(args.asan_runtime)
            and os.path.isfile(args.asan_runtime)
            else _resolve_runtime(args.asan_compiler)
        )
        # The C++ runtime rides the preload TOO (see the module
        # docstring: the __cxa_throw CHECK failure of the first run) —
        # resolved from the module's own DT_NEEDED, ground truth.
        stdcpp = _ldd_resolve(ldd_out, "libstdc++.so.6")
        if not stdcpp:
            _fail(
                "the instrumented module does not resolve libstdc++.so.6 "
                "through ldd — without the C++ runtime in the initial "
                "link map, the first C++ exception under the preloaded "
                "sanitizer dies in ASan's __cxa_throw interceptor"
            )
        preload = env.get("LD_PRELOAD", "")
        env["LD_PRELOAD"] = os.pathsep.join(
            p for p in [runtime, stdcpp, preload] if p
        )
        # See the module docstring: the interpreter's shutdown noise is
        # not ours to report; UAF/overflow (the declared gap) stay armed.
        env["ASAN_OPTIONS"] = "detect_leaks=0"
        print(f"[bindings-suite] module: {os.path.basename(module)} — "
              f"ASan runtime preloaded: {runtime}; "
              f"C++ runtime preloaded: {stdcpp}")
    else:
        print(f"[bindings-suite] module: {os.path.basename(module)} — "
              "not instrumented, plain run")

    # 1) The plain-Python regression suite (no pytest dependency by
    #    design — Bug 1-5; pytest collection imports it as a no-op).
    standalone = os.path.join(args.tests_dir, "test_bug_fixes.py")
    if not os.path.isfile(standalone):
        _fail(f"missing {standalone}")
    step1 = subprocess.run(
        [sys.executable, standalone], env=env, check=False
    )

    # 2) The pytest suite minus the two GL-bound files (own workflow
    #    step under Xvfb — see the module docstring).
    gl_files = ["test_ui_navigation.py", "test_ui_startup.py"]
    pytest_cmd = [sys.executable, "-m", "pytest", args.tests_dir,
                  "-q", "-p", "no:cacheprovider"]
    for name in gl_files:
        pytest_cmd += ["--ignore", os.path.join(args.tests_dir, name)]
    step2 = subprocess.run(pytest_cmd, env=env, check=False)

    ok = step1.returncode == 0 and step2.returncode == 0
    print(f"[bindings-suite] standalone: "
          f"{'PASS' if step1.returncode == 0 else 'FAIL'}, "
          f"pytest: {'PASS' if step2.returncode == 0 else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
