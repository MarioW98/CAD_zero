"""Headless CLI for CAD_0 — evaluate SDFs, export meshes, view results.

Usage:
    CAD_0 eval-sdf examples/sdf_sphere.py --output sphere.stl
    CAD_0 info examples/sdf_lattice_ball.py
    CAD_0 view examples/sdf_lattice_ball.py
    CAD_0 slice examples/sdf_lattice_ball.py --plane z --at 0.0
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

if _HAS_RICH:
    console = Console()
    app = typer.Typer(add_completion=False, help="CAD_0 headless CLI")
else:
    console = type('Console', (), {'print': lambda self, *a, **kw: print(*a)})()
    app = None


def _load_module(path: Path):
    """Load a Python file as a module and return it."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("cad_0_user_script", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _get_body(mod):
    """Extract the SDF body from a user script."""
    body = getattr(mod, "body", None) or getattr(mod, "shape", None)
    if body is None:
        raise ValueError("Script must define `body` or `shape`")
    return body


if _HAS_RICH:
    @app.command()
    def info(path: Path):
        """Print metadata about an SDF script."""
        import CAD_0
        mod = _load_module(path)
        body = _get_body(mod)
        table = Table(title=str(path))
        table.add_column("Property")
        table.add_column("Value")
        table.add_row("describe", body.describe())
        table.add_row("lipschitz", str(body.lipschitz()))
        b = body.bounds()
        table.add_row("bounds.min", f"({b.min.x:.3f}, {b.min.y:.3f}, {b.min.z:.3f})")
        table.add_row("bounds.max", f"({b.max.x:.3f}, {b.max.y:.3f}, {b.max.z:.3f})")
        # Sample a few points
        import numpy as np
        center = np.array([[0, 0, 0]], dtype=np.float32)
        result = CAD_0.sdf.evaluate(body, center)
        table.add_row("value(0,0,0)", f"{result.values[0]:.6f}")
        console.print(table)


    @app.command()
    def eval_sdf(path: Path, output: Path, resolution: int = 64, normals: bool = True):
        """Evaluate an SDF and export the mesh to a file."""
        import CAD_0
        mod = _load_module(path)
        body = _get_body(mod)
        console.print(f"[cyan]Extracting mesh (resolution={resolution})...[/cyan]")
        mesh = CAD_0.sdf.marching_cubes(body, resolution=resolution, compute_normals=normals)
        console.print(f"  vertices:  {mesh.vertex_count()}")
        console.print(f"  triangles: {mesh.triangle_count()}")
        CAD_0.io.export(str(output), mesh)
        console.print(f"[green]Exported: {output}[/green]")


    @app.command()
    def view(path: Path, resolution: int = 48):
        """Open a 3D viewer for the SDF body."""
        import CAD_0
        mod = _load_module(path)
        body = _get_body(mod)
        console.print(f"[cyan]Extracting mesh (resolution={resolution})...[/cyan]")
        mesh = CAD_0.sdf.marching_cubes(body, resolution=resolution, compute_normals=True)
        console.print(f"  vertices:  {mesh.vertex_count()}")
        console.print(f"  triangles: {mesh.triangle_count()}")
        console.print("[cyan]Opening 3D viewer...[/cyan]")
        CAD_0.viewer.view_mesh(mesh, title=f"CAD_0 — {path.name}")


    @app.command()
    def slice(path: Path, plane: str = "z", at: float = 0.0, resolution: int = 64):
        """View a 2D cross-section of the SDF."""
        import CAD_0
        mod = _load_module(path)
        body = _get_body(mod)
        console.print(f"[cyan]Computing cross-section (plane={plane}, at={at})...[/cyan]")
        CAD_0.viewer.view_sdf_slices(body, resolution=resolution, plane=plane, slice_at=at,
                                     title=f"CAD_0 — {path.name} ({plane}={at})")


    @app.command()
    def primitives():
        """List all available SDF primitives and operators."""
        table = Table(title="CAD_0 SDF Primitives & Operators")
        table.add_column("Name")
        table.add_column("Description")
        table.add_row("sphere(radius)", "Sphere centered at origin")
        table.add_row("box(extent)", "Axis-aligned box (half-extents)")
        table.add_row("cylinder(radius, height)", "Cylinder along Y axis")
        table.add_row("torus(R, r)", "Torus in XZ plane")
        table.add_row("cone(r, h)", "Cone along +Y")
        table.add_row("capsule(a, b, r)", "Capsule between points a,b")
        table.add_row("plane()", "Infinite half-space y > 0")
        table.add_row("", "")
        table.add_row("union(a, b)", "Boolean union")
        table.add_row("subtract(a, b)", "Boolean subtraction (a - b)")
        table.add_row("intersect(a, b)", "Boolean intersection")
        table.add_row("smooth_union(a, b, k)", "Smooth union (organic blend)")
        table.add_row("translate(body, offset)", "Translate SDF")
        table.add_row("rotate(body, quat)", "Rotate SDF")
        table.add_row("scale(body, s)", "Uniform scale")
        table.add_row("twist(body, k)", "Twist along Y axis")
        console.print(table)


def main():
    if app is None:
        print("CAD_0 CLI requires `typer` and `rich` (pip install CAD_0[cli])")
        sys.exit(1)
    app()


if __name__ == "__main__":
    main()
