"""Headless CLI for cadforge — evaluate SDFs, export STL, run scripts.

Usage:
    cadforge eval-sdf examples/sdf_sphere.py --output sphere.stl
    cadforge info examples/sdf_lattice.py
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import typer
    from rich.console import Console
    from rich.table import Table
    _HAS_RICH = True
except ImportError:
    _HAS_RICH = False

# Fall back to a minimal stub if typer/rich are not installed.
if not _HAS_RICH:
    class _Stub:
        def __init__(self, *a, **kw): pass
        def __call__(self, *a, **kw): pass
    typer = _Stub()
    Console = _Stub()
    Table = _Stub()

console = Console()

if _HAS_RICH:
    app = typer.Typer(add_completion=False, help="cadforge headless CLI")
else:
    app = None


def _load_module(path: Path):
    """Load a Python file as a module and return it."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("cadforge_user_script", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


if _HAS_RICH:
    @app.command()
    def info(path: Path):
        """Print metadata about an SDF script."""
        mod = _load_module(path)
        body = getattr(mod, "body", None) or getattr(mod, "shape", None)
        if body is None:
            console.print("[red]Script must define `body` or `shape`[/red]")
            raise typer.Exit(1)
        table = Table(title=str(path))
        table.add_column("Property")
        table.add_column("Value")
        table.add_row("describe", body.describe())
        table.add_row("lipschitz", str(body.lipschitz()))
        table.add_row("bounds.min", str(body.bounds().min))
        table.add_row("bounds.max", str(body.bounds().max))
        console.print(table)


    @app.command()
    def eval_sdf(path: Path, output: Path, resolution: int = 64):
        """Evaluate an SDF on a regular grid and save as a raw float32 volume."""
        import numpy as np
        from . import sdf as csdf

        mod = _load_module(path)
        body = getattr(mod, "body", None) or getattr(mod, "shape", None)

        b = body.bounds()
        x = np.linspace(b.min.x, b.max.x, resolution, dtype=np.float32)
        y = np.linspace(b.min.y, b.max.y, resolution, dtype=np.float32)
        z = np.linspace(b.min.z, b.max.z, resolution, dtype=np.float32)
        gx, gy, gz = np.meshgrid(x, y, z, indexing="ij")
        pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        out = csdf.evaluate(body, pts)
        vol = out.values.reshape(resolution, resolution, resolution)
        vol.tofile(output)
        console.print(f"[green]Wrote {output}[/green]  shape={vol.shape}")


def main():
    if app is None:
        print("cadforge CLI requires `typer` and `rich` (pip install cadforge[cli])")
        sys.exit(1)
    app()


if __name__ == "__main__":
    main()
