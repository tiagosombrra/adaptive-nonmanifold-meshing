# Software architecture

## Purpose

Adaptive Non-Manifold Meshing is a C++17 scientific application for adaptive triangular discretization of parametric patch complexes. It supports patch complexes with shared curves and non-manifold configurations while preserving compatibility along shared parametric boundaries.

The repository should be interpreted as a scientific software package. Published experiments are validation cases for the implementation, not the definition of the software itself.

## Execution model

The supported command-line interface is configuration-driven:

```bash
./build/bin/ap_mesh --config <configuration.conf>
```

The executable reads a key-value configuration file, validates required fields, maps the selected values to the current runtime state, and invokes the adaptive generator.

A generic wrapper is available at:

```bash
bash scripts/run_case.sh --config <configuration.conf>
```

The wrapper adds optional building, result-directory management, log capture, and case-specific verification.

## Main processing flow

1. Parse and validate the runtime configuration.
2. Load the parametric patch-complex model.
3. Construct curves, patches, and shared-boundary relationships.
4. Generate an initial compatible triangular discretization.
5. Estimate geometric error and mesh-quality indicators.
6. Adapt curves and patch interiors according to the selected policy.
7. Synchronize discretization along shared curves.
8. Reconstruct or update affected submeshes.
9. Apply configured smoothing and local post-processing.
10. Persist meshes, metrics, and runtime diagnostics.

## Major source areas

| Area | Responsibility |
|---|---|
| `src/config`, `include/config` | Validated runtime configuration and mapping to the inherited execution state. |
| `src/data`, `include/data` | Core geometric and mesh entities: points, nodes, curves, patches, elements, submeshes, and models. |
| `src/generator`, `include/generator` | High-level meshing and adaptive-iteration orchestration. |
| `src/adapter`, `include/adapter` | Adaptation decisions and reconstruction of affected discretizations. |
| `src/crab_mesh`, `include/crab_mesh` | Advancing-front, quadtree, and parametric-domain meshing support. |
| `src/curvature`, `include/curvature` | Curvature and adjacency calculations used by adaptation and assessment. |
| `src/input_output`, `include/input_output` | Model parsing, mesh output, and diagnostic persistence. |
| `src/parallel`, `include/parallel` | Communication abstractions and optional parallel execution support. |
| `src/timer`, `include/timer` | Runtime measurements. |

## Configuration boundary

The current implementation defines and parses `RuntimeConfig` in the testable `src/config/runtime_config.cpp` module, while the deeper algorithm still consumes a substantial set of global runtime variables. `ApplyRuntimeConfig` transfers validated configuration values into that legacy state before invoking `GeneratorAdaptive`.

This boundary is intentionally retained for version 1.0.0 because replacing all global parameters would be a broad architectural change with significant regression risk. The configuration-driven entry point provides a stable user interface while the internal state model remains documented as a technical limitation.

## Legacy compatibility boundary

`GeneratorAdaptive::Execute` currently expects an argument-vector representation inherited from the original experimental application. In configuration mode, `main.cpp` creates a small synthetic argument vector after loading the configuration. This is an internal compatibility adapter; users should invoke only `ap_mesh --config ...` or `scripts/run_case.sh`.

A future library-oriented release may replace this adapter with a typed execution context passed directly to the generator.

## Ownership model

The codebase contains raw pointers because many geometric entities are shared across curves, patches, meshes, and submeshes. Some pointers are owning and others are non-owning references. Version 1.0.0 corrects definite allocation/deallocation defects but does not perform a global smart-pointer conversion without dedicated ownership tests.

Key policy for this release:

- preserve documented shared references;
- correct definite mismatched allocation and invalid deletion;
- avoid broad pointer-representation changes;
- document ownership assumptions when a class is audited;
- treat future RAII migration as a separate architectural milestone.

## Parallelism

OpenMP support is optional and enabled through CMake when available. MPI support is optional and requires an MPI compiler/toolchain. MPI remains experimental in version 1.0.0. Reproducibility workflows use deterministic single-process configurations unless a validation case or study protocol explicitly states otherwise.

## Output contract

The executable writes mesh and diagnostic files according to `OUTPUT_PREFIX`, `WRITE_MODE`, and the configured runtime options. Case wrappers capture standard output in a `run.log`. Published validation cases may add machine-readable metadata and independent verification reports.

The exact set of generated scientific files depends on the configuration. Documentation for each maintained example identifies its expected primary output and verification method.

Concrete ownership and borrowing rules are documented in [`OWNERSHIP.md`](OWNERSHIP.md). Deferred architectural work is tracked in [`ROADMAP.md`](ROADMAP.md).

## Validation hierarchy

The repository distinguishes four concerns:

- **software execution:** any valid configuration processed through the general CLI;
- **maintained examples:** representative Book, Eistute, and Decor Shelf uses;
- **published validation:** archived numerical targets associated with the algorithm article;

The Eistute stage-4 result is retained because it provides a strong published regression target. It is not the only supported usage scenario, and it must not be conflated with the separate baseline/candidate evidence used by the JSS study.

## Known architectural limitations

- a large set of algorithm parameters remains in global state;
- the main generator source is large and combines several responsibilities;
- the internal execution API still uses an inherited argument-vector contract;
- ownership semantics are not uniformly encoded in C++ types;
- some parsers and diagnostics retain legacy conventions;
- the project is currently an application, not a stable public C++ library API.

These limitations do not prevent reproducible use, but they define the scope of version 1.0.0 and the priorities for subsequent versions.
