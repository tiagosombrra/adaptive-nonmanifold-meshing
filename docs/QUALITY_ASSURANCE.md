# Quality assurance

The release audit is intentionally stricter than the default build. It
configures the project without MPI or OpenMP, enables the full project warning
set, promotes warnings to errors, runs `cppcheck`, and verifies the archived
Eistute reference.

## Compiler diagnostics

The pre-hardening Linux audit reported 328 diagnostic lines. Those lines
represented 251 compiler warnings; the remaining lines were source excerpts
and notes. The warning inventory was:

| Family | Warnings | Resolution |
|---|---:|---|
| Shadowed declarations | 124 | Renamed parameters, locals and one cell edge member without changing values. |
| Variable-length arrays | 45 | Replaced with checked `std::vector`-backed storage. |
| Intentionally unused patch handles | 68 | Marked the handles `[[maybe_unused]]`; `MakePatch` still registers every patch in the geometry. |
| Integral, float and sign conversions | 11 | Added explicit size conversions and checked legacy-ID narrowing. |
| Dead or unused values | 3 | Removed calculations with no consumers or marked an interface-only parameter. |

The warning-enabled Release build now completes with zero diagnostics. The
`cpp-code-quality-audit` workflow sets
`AP_MESH_WARNINGS_AS_ERRORS=ON` and rejects a non-empty compiler diagnostics
file, so regressions fail the check.

The only global warning exception is `-Wno-cast-function-type`, retained for
the legacy callback ABI. No other warning family is suppressed.

## Static and dynamic analysis

- `cppcheck`: zero diagnostics with warning, style, performance and portability
  checks enabled.
- ASan/UBSan: configuration tests, CLI tests and maintained smoke profiles run
  in the `sanitizers-and-smoke` job.
- Result integrity: manifests are generated only after `run.log` is final,
  then independently verified for exact paths, sizes and SHA-256 hashes.

## Scientific regression

Warning cleanup is limited to ownership, naming, standards compliance and
checked conversions. The published Eistute profile remains the scientific
invariant: final stage 4 with exactly 7,184 triangular faces.
