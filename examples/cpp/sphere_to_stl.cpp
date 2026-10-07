// examples/cpp/sphere_to_stl.cpp
//
// End-to-end Phase B.5 example: build an SDF, extract a triangle mesh
// via marching cubes (with vertex welding), and export to STL / OBJ / 3MF.
//
// Run with:
//   ./bin/example_sphere_to_stl
//
// Output:
//   /home/z/my-project/download/lattice_ball.{stl,obj,3mf}
//
#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/operators.hpp"
#include "cadforge/sdf/transforms.hpp"
#include "cadforge/sdf/mesh_extract.hpp"
#include "cadforge/io/stl.hpp"
#include "cadforge/io/obj.hpp"
#include "cadforge/io/threemf.hpp"

#include <chrono>
#include <iostream>

int main() {
    using namespace cadforge;

    // Build a "lattice ball": sphere with a cylindrical hole through it,
    // then subtract two more perpendicular cylinders for a 3-axis hole pattern.
    auto ball = sdf::make_sphere(2.0f);

    auto cyl_y = sdf::make_cylinder(0.4f, 10.0f);                          // along Y
    auto cyl_z = sdf::sdf_rotate(sdf::make_cylinder(0.4f, 10.0f),
                                 math::Quatf::from_axis_angle({1, 0, 0}, 3.14159f / 2.0f));
    auto cyl_x = sdf::sdf_rotate(sdf::make_cylinder(0.4f, 10.0f),
                                 math::Quatf::from_axis_angle({0, 1, 0}, 3.14159f / 2.0f));

    auto holes = sdf::sdf_union(sdf::sdf_union(std::move(cyl_y), std::move(cyl_z)),
                                std::move(cyl_x));
    auto lattice_ball = sdf::sdf_subtract(std::move(ball), std::move(holes));

    std::cout << "SDF tree: " << lattice_ball.describe() << "\n";
    std::cout << "Bounds:   [" << lattice_ball.bounds().min.x << ", "
              << lattice_ball.bounds().max.x << "] etc\n";
    std::cout << "Lipschitz: " << lattice_ball.lipschitz() << "\n\n";

    // -------------------------------------------------------------------
    // Phase B.5: extract with vertex welding (default).
    // -------------------------------------------------------------------
    std::cout << "Marching cubes (resolution=64, with vertex welding)...\n";
    auto t_start = std::chrono::steady_clock::now();
    auto mesh = sdf::marching_cubes(lattice_ball, 64, /*compute_normals=*/true);
    auto t_end = std::chrono::steady_clock::now();
    const auto ms_mc = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::cout << "  vertices:  " << mesh.vertex_count() << "\n";
    std::cout << "  triangles: " << mesh.triangle_count() << "\n";
    std::cout << "  elapsed:   " << ms_mc << " ms\n";

    // Compare with Phase B (no weld) for reference.
    sdf::MarchingCubesOptions opts_noweld;
    opts_noweld.resolution = 64;
    opts_noweld.compute_normals = true;
    opts_noweld.weld_vertices = false;
    auto mesh_noweld = sdf::marching_cubes(lattice_ball, lattice_ball.bounds(), opts_noweld);
    std::cout << "\n  (Phase B baseline — no weld):\n";
    std::cout << "    vertices:  " << mesh_noweld.vertex_count() << "\n";
    std::cout << "    triangles: " << mesh_noweld.triangle_count() << "\n";
    const double reduction = 1.0 - double(mesh.vertex_count()) / double(mesh_noweld.vertex_count());
    std::cout << "    vertex reduction: " << (reduction * 100.0) << "%\n";

    // -------------------------------------------------------------------
    // Export to multiple formats.
    // -------------------------------------------------------------------
    const std::string out_dir = "/home/z/my-project/download/";

    t_start = std::chrono::steady_clock::now();
    io::export_stl_binary(out_dir + "lattice_ball.stl", mesh, "lattice_ball");
    t_end = std::chrono::steady_clock::now();
    const auto ms_stl = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
    io::export_obj(out_dir + "lattice_ball.obj", mesh, "lattice_ball");
    t_end = std::chrono::steady_clock::now();
    const auto ms_obj = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

    t_start = std::chrono::steady_clock::now();
    io::export_3mf(out_dir + "lattice_ball.3mf", mesh, "lattice_ball", "cadforge");
    t_end = std::chrono::steady_clock::now();
    const auto ms_3mf = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

    std::cout << "\nExports to " << out_dir << ":\n";
    std::cout << "  lattice_ball.stl  (" << ms_stl << " ms)\n";
    std::cout << "  lattice_ball.obj  (" << ms_obj << " ms)\n";
    std::cout << "  lattice_ball.3mf  (" << ms_3mf << " ms)\n";

    return 0;
}
