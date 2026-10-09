#!/usr/bin/env bash
# build.sh — convenience build script for CAD_0.
#
# Usage:
#   ./scripts/build.sh                  # C++ only (no Python bindings)
#   ./scripts/build.sh --python        # C++ + Python bindings (uses $(which python3))
#   ./scripts/build.sh --python /path/to/python3  # specify Python interpreter
#   ./scripts/build.sh --test          # build + run C++ tests
#   ./scripts/build.sh --clean        # remove build/ before building
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_PYTHON="OFF"
BUILD_TESTING="ON"
BUILD_EXAMPLES="ON"
USE_TBB="OFF"
PYTHON_EXECUTABLE=""
CLEAN="OFF"
RUN_TESTS="OFF"

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --python)
            BUILD_PYTHON="ON"
            shift
            # Optional: next arg might be a path to python3
            if [[ $# -gt 0 && "$1" != --* ]]; then
                PYTHON_EXECUTABLE="$1"
                shift
            fi
            ;;
        --test)        RUN_TESTS="ON"; shift ;;
        --clean)      CLEAN="ON"; shift ;;
        --debug)      BUILD_TYPE="Debug"; shift ;;
        --tbb)        USE_TBB="ON"; shift ;;
        -h|--help)
            cat <<EOF
Usage: $0 [OPTIONS]

Options:
  --python [PATH]   Build Python bindings (optionally specify python3 path)
  --test            Run C++ tests after build
  --clean           Remove build/ before building
  --debug           Build type = Debug (default: Release)
  --tbb             Enable oneTBB (default: OFF)
  -h, --help        Show this help

Environment variables:
  BUILD_TYPE        CMake build type (default: Release)
EOF
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Ensure cmake + ninja are available
if ! command -v cmake &> /dev/null; then
    echo "ERROR: cmake not found. Install via 'pip install cmake' or apt."
    exit 1
fi
if ! command -v ninja &> /dev/null; then
    echo "ERROR: ninja not found. Install via 'pip install ninja' or apt."
    exit 1
fi

# If --python was given without a path, try $(which python3)
if [[ "${BUILD_PYTHON}" == "ON" && -z "${PYTHON_EXECUTABLE}" ]]; then
    PYTHON_EXECUTABLE="$(command -v python3 || true)"
    if [[ -z "${PYTHON_EXECUTABLE}" ]]; then
        echo "ERROR: python3 not found in PATH. Pass a path: $0 --python /usr/bin/python3"
        exit 1
    fi
fi

# Optionally clean
if [[ "${CLEAN}" == "ON" ]]; then
    echo "Cleaning ${BUILD_DIR}..."
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

# Configure
echo "Configuring..."
CMAKE_ARGS=(
    -G Ninja
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DCAD_0_BUILD_PYTHON="${BUILD_PYTHON}"
    -DCAD_0_BUILD_TESTING="${BUILD_TESTING}"
    -DCAD_0_BUILD_EXAMPLES="${BUILD_EXAMPLES}"
    -DCAD_0_USE_TBB="${USE_TBB}"
    -DCMAKE_CXX_COMPILER=g++
    -DCMAKE_C_COMPILER=gcc
)
if [[ -n "${PYTHON_EXECUTABLE}" ]]; then
    CMAKE_ARGS+=(-DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}")
fi

cmake -B "${BUILD_DIR}" -S "${PROJECT_ROOT}" "${CMAKE_ARGS[@]}"

# Build
echo "Building..."
cmake --build "${BUILD_DIR}"

# Optionally test
if [[ "${RUN_TESTS}" == "ON" ]]; then
    echo "Running C++ tests..."
    "${BUILD_DIR}/bin/CAD_0_tests"
fi

# If Python bindings were built, test them
if [[ "${BUILD_PYTHON}" == "ON" ]]; then
    echo ""
    echo "Testing Python bindings..."
    PYTHONPATH="${BUILD_DIR}/python:${PYTHONPATH:-}" "${PYTHON_EXECUTABLE}" -c "
import CAD_0
print('CAD_0 version:', CAD_0.__version__)
sph = CAD_0.sdf.sphere(radius=1.5)
print('Sphere:', sph.describe())
print('Bounds:', sph.bounds().min, '->', sph.bounds().max)
mesh = CAD_0.sdf.marching_cubes(sph, resolution=16)
print(f'Mesh: {mesh.vertex_count()} vertices, {mesh.triangle_count()} triangles')
print()
print('Python bindings: OK')
"
fi

echo ""
echo "Build complete."
echo "  Build dir: ${BUILD_DIR}"
if [[ "${BUILD_PYTHON}" == "ON" ]]; then
    echo "  Python extension: ${BUILD_DIR}/python/CAD_0/_CAD_0*.so"
fi
