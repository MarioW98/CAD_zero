#!/usr/bin/env python3
"""Detect backslashes inside f-string expressions — the Python 3.10 killer.

A backslash anywhere inside an f-string EXPRESSION part (the code between
braces) is a syntax error on every Python before 3.12: PEP 701 lifted the
restriction, so a 3.12/3.13 development interpreter accepts in silence
what the CI's 3.10 floor rejects at import time. This is exactly the red
CI run 38863633779 (2026-10-15): `f"...{'s\u00ec' if x else 'no'}..."`
compiled fine on 3.12 and broke `import CAD_0.ui.main_window` on 3.10.

What counts as a violation:
  * any token containing a backslash while the tokenizer is inside the
    braces of an f-string expression — including backslashes inside
    nested plain string literals in the expression (the original bug);
  * backslashes in f-string LITERAL parts (outside braces) are legal on
    3.10 and are NOT reported, nor are escapes in nested f-strings'
    own literal parts (stack-tracked, legal on 3.10).

Requirements: Python 3.12+ — the checker consumes the FSTRING_START /
FSTRING_MIDDLE / FSTRING_END tokens that PEP 701 introduced, which do
not exist in earlier tokenizers. On older interpreters it refuses to
run rather than silently scanning nothing: the CI's own 3.10 import
step remains the authoritative hard gate; this script is the fast local
pre-push feedback on dev machines (3.12+).

Usage:
  python3 scripts/check_fstring_backslashes.py            # all git-tracked *.py
  python3 scripts/check_fstring_backslashes.py FILE ...    # specific files

Exit status: 0 clean, 1 violations (or usage/environment errors).
"""
from __future__ import annotations

import io
import os
import subprocess
import sys
import tokenize

MINIMUM = (3, 12)


def iter_target_files(argv: list[str]):
    """Files to scan: explicit args, or every git-tracked *.py."""
    if argv:
        for arg in argv:
            if os.path.isdir(arg):
                for root, _dirs, names in os.walk(arg):
                    for name in sorted(names):
                        if name.endswith(".py"):
                            yield os.path.join(root, name)
            else:
                yield arg
        return
    try:
        out = subprocess.run(
            ["git", "ls-files", "*.py"],
            check=True, capture_output=True, text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        sys.exit(f"error: not inside a git repository and no files given ({exc})")
    for line in out.splitlines():
        yield line


def check_file(path: str) -> list[str]:
    """One finding per offending token: 'path:line:col ...'."""
    with open(path, "rb") as fh:
        src = fh.read()
    try:
        tokens = list(tokenize.tokenize(io.BytesIO(src).readline))
    except SyntaxError as exc:
        return [f"{path}: cannot tokenize ({exc}) — fix the syntax first"]

    findings: list[str] = []
    # Stack of per-f-string expression brace depths: an FSTRING_START
    # inside an expression (nested f-string) pushes a fresh context and
    # pops it at its FSTRING_END, so the outer expression's depth is
    # restored — its remaining tokens are still checked.
    expr_stack: list[int] = []

    for tok in tokens:
        if tok.type == tokenize.FSTRING_START:
            expr_stack.append(0)
        elif tok.type == tokenize.FSTRING_END:
            if expr_stack:
                expr_stack.pop()
        elif expr_stack:
            if tok.type == tokenize.OP and tok.string == "{":
                expr_stack[-1] += 1
            elif tok.type == tokenize.OP and tok.string == "}":
                expr_stack[-1] = max(0, expr_stack[-1] - 1)
            elif expr_stack[-1] > 0 and "\\" in tok.string:
                line, col = tok.start
                findings.append(
                    f"{path}:{line}:{col + 1}: backslash inside an f-string "
                    f"expression: {tok.string!r} (Python 3.10 floor — "
                    f"red run 38863633779; precompute outside the f-string)"
                )
    return findings


def main() -> int:
    if sys.version_info < MINIMUM:
        sys.exit(
            f"error: this checker requires Python 3.12+ "
            f"(PEP 701 FSTRING_* tokens); running on "
            f"{sys.version.split()[0]}. On 3.10 the CI import step is the "
            f"authoritative gate — run this locally on a 3.12+ interpreter."
        )

    findings: list[str] = []
    files = list(iter_target_files(sys.argv[1:]))
    for path in files:
        findings.extend(check_file(path))

    print(f"scanned {len(files)} file(s) on Python {sys.version.split()[0]}")
    if findings:
        for f in findings:
            print(f)
        print(f"{len(findings)} violation(s)")
        return 1
    print("no backslash inside f-string expressions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
