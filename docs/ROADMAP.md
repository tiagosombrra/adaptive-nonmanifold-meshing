# Post-v1 roadmap

Version 1.0.0 prioritizes scientific stability, a strict command-line
contract, reproducibility and supported deployment environments. The following
work is intentionally outside that release.

## Architecture

- replace global generator parameters with a typed execution context;
- remove the inherited synthetic argument-vector adapter;
- encode all ownership transfers with RAII types;
- decompose the large generator orchestration unit;
- define a stable public C++ library API only after ABI and ownership tests
  exist.

## Geometry and numerics

- add anisotropic, curvature-direction-aligned adaptation;
- improve behavior under highly distorted patch parameterizations;
- expand local compatibility indicators for complex multi-incident junctions;
- compare against independent feature-aware meshing backends;
- grow the maintained non-manifold benchmark collection.

## Parallelism and performance

- promote MPI from experimental after deterministic multi-process regressions
  and supported deployment documentation exist;
- add parallel scaling studies for OpenMP and MPI;
- refine stage-level profiling around patch-interior reconstruction;
- optimize only profile-confirmed bottlenecks while retaining scientific
  equivalence gates.

## Interfaces and formats

- migrate remaining historical readers behind typed, line-numbered errors;
- evaluate a versioned machine-readable scientific summary format;
- generalize the container entry point beyond the canonical validation case;
- consider an application/library split after the v1 ownership model is fully
  encoded.
