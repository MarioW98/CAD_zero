"""CAD_0 — dual-representation CAD kernel.

This __init__.py is used in the build/python/ directory where both the
compiled _CAD_0 extension and the Python UI live together.
"""

from __future__ import annotations

import sys

__version__ = "0.1.0"


def _promote_libstdcpp_global() -> None:
    """Put libstdc++ into the process-wide GLOBAL dlopen scope (Linux).

    Root cause of what this guard prevents (diagnosed 2026-10-14, see
    docs/audits/2026-10-13-warning-audit.md "Toolchain contract"):

    * gcc <= 11 headers emit the std::locale facet ``id`` statics
      (numpunct/num_put/collate...) as vague-linkage STB_GNU_UNIQUE
      symbols in every DSO compiled with them — gcc >= 12 TUs do not;
    * if the first libstdc++ in the process arrives through an
      RTLD_LOCAL dependency chain — the classic case is
      ``import numpy`` first: numpy.libs (openblas/libgfortran) pull
      libstdc++.so.6 as a local dependency — the unique-symbol
      unification between the gcc-11 DSO's own facet-id copies and
      libstdc++'s never happens;
    * the runtime symptom: ``std::use_facet`` throws std::bad_cast,
      which libstdc++'s formatted-output wrappers swallow as a plain
      failbit — so EVERY formatted stream write (``<<`` int/float)
      from gcc-11-compiled code fails silently. The kernel's text
      writers (io export_obj, native .CAD_0 save) write truncated
      files and return False; the scene survives, the file does not.

    Fix: promote libstdc++.so.6 to the global scope once, at package
    import — before the extension (and before/after any numpy import:
    verified both orders). Loader-level only: no computation, no
    evaluation values, no test goldens are affected (ADR-0014 safe).
    No-op where libstdc++.so.6 is absent (non-gcc toolchains, static
    builds) or where the promotion is not needed.
    """
    if not sys.platform.startswith("linux"):
        return
    try:
        import ctypes
        ctypes.CDLL("libstdc++.so.6", mode=ctypes.RTLD_GLOBAL)
    except (OSError, ImportError, AttributeError):
        pass  # not present / no dynamic loader quirk to neutralize


_promote_libstdcpp_global()

# Import the compiled extension and re-export its submodules.
# This works when _CAD_0.so/.pyd is in the same directory as this file.
try:
    from . import _CAD_0  # noqa: F401
    from ._CAD_0 import math, geometry, sdf, scene, camera, commands  # noqa: F401
    from ._CAD_0.geometry import (  # noqa: F401
        Shape, ShapeId, FeatureId, DatumCS, Transform,
        Metadata, Representation, Units,
    )
    # Python-side io wrapper (auto-detect export from file extension).
    # Bug fix: this import was missing, so `CAD_0.io` (documented in the
    # README) raised AttributeError after a plain `import CAD_0`.
    from . import io  # noqa: F401
    _HAS_KERNEL = True
except ImportError:
    _HAS_KERNEL = False
