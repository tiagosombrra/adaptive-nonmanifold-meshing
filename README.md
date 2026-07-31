# Adaptive Non-Manifold Meshing

[![Linux](https://github.com/tiagosombrra/adaptive-nonmanifold-meshing/actions/workflows/linux-reproduction.yml/badge.svg)](https://github.com/tiagosombrra/adaptive-nonmanifold-meshing/actions/workflows/linux-reproduction.yml)
[![Windows](https://github.com/tiagosombrra/adaptive-nonmanifold-meshing/actions/workflows/windows-reproduction.yml/badge.svg)](https://github.com/tiagosombrra/adaptive-nonmanifold-meshing/actions/workflows/windows-reproduction.yml)

Adaptive Non-Manifold Meshing is a C++17 command-line application for compatibility-preserving adaptive triangular meshing of parametric patch complexes, including non-manifold configurations in which multiple patches share a curve.

The software reads a parametric patch-complex model, constructs an initial compatible mesh, estimates geometric error and mesh-quality indicators, adapts curve and patch discretizations, synchronizes shared boundaries, and writes triangular meshes and execution diagnostics.

## Capabilities

- adaptive discretization of Bezier and Hermite patch complexes;
- shared-curve compatibility for non-manifold configurations;
- geometric-error and mesh-quality driven refinement;
- configuration-driven command-line execution;
- OBJ mesh and diagnostic output;
- optional OpenMP and experimental MPI builds;
- Linux, Windows, and container workflows;
- maintained scientific regression cases.

## Requirements

Linux requires CMake 3.15 or later, a C++17 compiler, and Python 3. Ubuntu dependencies can be installed with:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3
```

Windows is supported through MSYS2 UCRT64, GCC/MinGW-w64, CMake, and Python 3. See `INSTALL_WINDOWS.txt`.

## Build

```bash
bash scripts/build_linux.sh
```

The executable is generated at `build/bin/ap_mesh`.

## Run

```bash
./build/bin/ap_mesh --config configs/eistute/article.conf
```

Maintained convenience commands include:

```bash
bash scripts/run_book_linux.sh
bash scripts/run_decor_shelf_linux.sh
bash scripts/run_eistute_linux.sh
```

The generic wrapper is:

```bash
bash scripts/run_case.sh --help
```

## Validation

The maintained Eistute regression associated with Figure 7 of the Computers & Graphics article can be reproduced with:

```bash
bash reproduce_eistute_linux.sh
```

Build and product tests can be run with:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

## Container

```bash
docker build -f container/Dockerfile -t adaptive-nonmanifold-meshing .
docker run --rm adaptive-nonmanifold-meshing
```

## Documentation

- `docs/BP_INPUT_FORMAT.md`: parametric input format;
- `docs/CONFIGURATION.md`: runtime configuration;
- `docs/INPUT_OUTPUT_CONTRACT.md`: supported inputs and generated outputs;
- `docs/SOFTWARE_ARCHITECTURE.md`: implementation architecture;
- `docs/QUALITY_ASSURANCE.md`: validation and quality controls;
- `docs/REPRODUCIBILITY.md`: maintained reproduction procedures.

## Research citation

The algorithm and its evaluation are reported in:

> Tiago Guimaraes Sombra, Joaquim Bento Cavalcante-Neto, and Creto Augusto Vidal. Adaptive Meshing of Non-Manifold Parametric Patch Complexes with Shared-Curve Compatibility. Computers & Graphics, article 104707, 2026. DOI: 10.1016/j.cag.2026.104707.

Software citation metadata are provided in `CITATION.cff`.

## License

This software is distributed under the MIT License. Third-party notices are recorded in `THIRD_PARTY_LICENSES.md`.
