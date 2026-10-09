"""cadforge.viewer — simple 3D mesh viewer using matplotlib.

Provides a quick way to visualize TriangleMesh objects produced by the
SDF kernel, without requiring PySide6 or OpenGL. Uses matplotlib's
3D plotting capabilities.

Usage:
    import CAD_0
    from CAD_0.viewer import view_mesh

    sph = CAD_0.sdf.sphere(radius=2.0)
    mesh = CAD_0.sdf.marching_cubes(sph, resolution=32)
    view_mesh(mesh)  # opens a 3D window
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import Optional, Any


def view_mesh(
    mesh: Any,
    *,
    title: str = "CAD_0 Mesh Viewer",
    color: str = "cyan",
    edge_color: str = "black",
    linewidth: float = 0.1,
    show_normals: bool = False,
    normal_scale: float = 0.05,
    equal_aspect: bool = True,
) -> plt.Figure:
    """Display a TriangleMesh in a 3D matplotlib window.

    Args:
        mesh: A CAD_0.sdf.TriangleMesh object (or any object with
              positions, normals, indices as numpy arrays).
        title: Window title.
        color: Face color of the triangles.
        edge_color: Edge color of the triangles.
        linewidth: Edge line width (0 = no edges).
        show_normals: If True, draw normal vectors as red arrows.
        normal_scale: Length of normal arrows relative to bbox size.
        equal_aspect: If True, force equal aspect ratio on all 3 axes.

    Returns:
        The matplotlib Figure object.
    """
    # Extract mesh data as numpy arrays
    positions = np.asarray(mesh.positions)
    indices = np.asarray(mesh.indices)

    if len(positions) == 0 or len(indices) == 0:
        raise ValueError("Mesh is empty — nothing to display")

    # Reshape indices into (N, 3) triangles
    triangles = indices.reshape(-1, 3)

    # Collect triangle vertices as a list of (3, 3) arrays
    polys = positions[triangles]  # shape: (N, 3, 3)

    # Create the figure
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Draw the mesh as a Poly3DCollection
    poly_collection = Poly3DCollection(
        polys,
        alpha=0.8,
        facecolor=color,
        edgecolor=edge_color,
        linewidths=linewidth,
    )
    ax.add_collection3d(poly_collection)

    # Set axis limits based on the mesh bounds
    mins = positions.min(axis=0)
    maxs = positions.max(axis=0)
    ax.set_xlim(mins[0], maxs[0])
    ax.set_ylim(mins[1], maxs[1])
    ax.set_zlim(mins[2], maxs[2])

    if equal_aspect:
        try:
            ax.set_box_aspect(
                (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])
            )
        except AttributeError:
            pass  # older matplotlib versions

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)

    # Optionally draw normals
    if show_normals and hasattr(mesh, 'normals') and len(mesh.normals) > 0:
        normals = np.asarray(mesh.normals)
        bbox_size = max(maxs - mins) * normal_scale
        for i in range(min(100, len(positions))):  # subsample for clarity
            p = positions[i]
            n = normals[i] * bbox_size
            ax.quiver(p[0], p[1], p[2], n[0], n[1], n[2], color='red', alpha=0.5)

    plt.tight_layout()
    plt.show()
    return fig


def view_sdf(
    body: Any,
    *,
    resolution: int = 32,
    title: str = "CAD_0 SDF Viewer",
    **kwargs: Any,
) -> plt.Figure:
    """Convenience: build an SDF body → extract mesh → view it.

    Args:
        body: A CAD_0.sdf.SDFBody.
        resolution: Marching cubes resolution.
        title: Window title.
        **kwargs: Passed to view_mesh().

    Returns:
        The matplotlib Figure object.
    """
    # Import here to avoid circular import
    import CAD_0
    mesh = CAD_0.sdf.marching_cubes(body, resolution=resolution, compute_normals=True)
    return view_mesh(mesh, title=title, **kwargs)


def view_sdf_slices(
    body: Any,
    *,
    resolution: int = 64,
    plane: str = 'z',
    slice_at: float = 0.0,
    title: str = "CAD_0 SDF Cross-Section",
) -> plt.Figure:
    """View a 2D cross-section of an SDF.

    Evaluates the SDF on a 2D grid at the given slice position and
    shows it as a contour plot.

    Args:
        body: A CAD_0.sdf.SDFBody.
        resolution: Grid resolution per axis.
        plane: Which plane to slice ('x', 'y', or 'z').
        slice_at: Position of the slice along the chosen axis.
        title: Window title.

    Returns:
        The matplotlib Figure object.
    """
    import CAD_0

    bounds = body.bounds()
    extent = bounds.max - bounds.min

    if plane == 'z':
        x = np.linspace(bounds.min.x, bounds.max.x, resolution, dtype=np.float32)
        y = np.linspace(bounds.min.y, bounds.max.y, resolution, dtype=np.float32)
        gx, gy = np.meshgrid(x, y, indexing='ij')
        gz = np.full_like(gx, slice_at)
        pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        result = CAD_0.sdf.evaluate(body, pts)
        grid = np.array(result.values).reshape(resolution, resolution)
        xlabel, ylabel = 'X', 'Y'
        extent_args = [bounds.min.x, bounds.max.x, bounds.min.y, bounds.max.y]
    elif plane == 'y':
        x = np.linspace(bounds.min.x, bounds.max.x, resolution, dtype=np.float32)
        z = np.linspace(bounds.min.z, bounds.max.z, resolution, dtype=np.float32)
        gx, gz = np.meshgrid(x, z, indexing='ij')
        gy = np.full_like(gx, slice_at)
        pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        result = CAD_0.sdf.evaluate(body, pts)
        grid = np.array(result.values).reshape(resolution, resolution)
        xlabel, ylabel = 'X', 'Z'
        extent_args = [bounds.min.x, bounds.max.x, bounds.min.z, bounds.max.z]
    elif plane == 'x':
        y = np.linspace(bounds.min.y, bounds.max.y, resolution, dtype=np.float32)
        z = np.linspace(bounds.min.z, bounds.max.z, resolution, dtype=np.float32)
        gy, gz = np.meshgrid(y, z, indexing='ij')
        gx = np.full_like(gy, slice_at)
        pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        result = CAD_0.sdf.evaluate(body, pts)
        grid = np.array(result.values).reshape(resolution, resolution)
        xlabel, ylabel = 'Y', 'Z'
        extent_args = [bounds.min.y, bounds.max.y, bounds.min.z, bounds.max.z]
    else:
        raise ValueError(f"plane must be 'x', 'y', or 'z', got '{plane}'")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Left: contour plot (inside = blue, outside = red)
    ax1 = axes[0]
    cs = ax1.contourf(grid, levels=50, extent=extent_args, cmap='RdBu')
    ax1.contour(grid, levels=[0.0], colors='black', linewidths=2, extent=extent_args)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    ax1.set_title(f'{title} — {plane}={slice_at:.2f}')
    ax1.set_aspect('equal')
    fig.colorbar(cs, ax=ax1, label='SDF value')

    # Right: binary inside/outside
    ax2 = axes[1]
    binary = (grid < 0).astype(float)
    ax2.imshow(binary.T, extent=extent_args, origin='lower', cmap='Blues', aspect='equal')
    ax2.set_xlabel(xlabel)
    ax2.set_ylabel(ylabel)
    ax2.set_title(f'Inside (blue) / Outside (white)')

    plt.tight_layout()
    plt.show()
    return fig
