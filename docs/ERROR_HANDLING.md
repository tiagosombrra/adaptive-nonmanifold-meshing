# Error handling and exit-status contract

## Scope

The OSP package exposes a layered execution path. User-facing validation and
operational errors are handled before the legacy numerical core whenever
possible. This reduces ambiguous failures without changing the validated
meshing algorithms.

## Supported execution layers

```text
run_model_linux.sh / run_case.sh
        |
        +--> configuration and input preflight
        +--> ap_mesh --config <file>
        +--> structural or published-result verification
        +--> result manifest generation
```

## Wrapper exit statuses

The maintained shell wrappers use the following status classes:

| Status | Meaning |
|---:|---|
| `0` | Execution and requested verification completed successfully |
| `2` | Invalid wrapper arguments or unsupported option/model |
| `3` | Missing configuration or input asset |
| `4` | Missing executable/build artifact |
| `5` | Missing requested verifier |
| other nonzero | Status propagated from the executable or verification command |

`run_case.sh` preserves the exact status returned by `ap_mesh` so CI and calling
scripts can distinguish numerical execution failure from wrapper validation
failure.

## Executable exit statuses

The `ap_mesh` executable uses a smaller stable status contract:

| Status | Meaning |
|---:|---|
| `0` | Help, version, or configured execution completed successfully |
| `2` | Invalid command-line usage |
| `3` | Configuration or input-path validation failed |
| `4` | Meshing execution failed |

Diagnostics are written to standard error. `--help` and `--version` do not
initialize the numerical or optional parallel runtime.

## Preflight failures

`scripts/validate_case_input.py` rejects the run before invoking the C++ parser
when any of the following is detected:

- missing or malformed configuration;
- missing `INPUT_MODEL` or `OUTPUT_PREFIX`;
- missing model file;
- unsupported maintained input extension;
- malformed control-point records;
- non-finite coordinates;
- patch records with a control count other than 16;
- non-integer or out-of-range control-point indices;
- unsupported records in the maintained `.bp` workflow.

Both `#` and `//` full-line comments are accepted because both forms occur in
the distributed assets.

## Verification failures

Structural verification checks that a maintained case produced at least one
non-empty triangular stage mesh and did not exceed the configured stage limit.
Early stopping is valid for structural examples.

Published validation is stricter. The Eistute validation requires:

- final stage `4`;
- exactly `7,184` triangular faces;
- consistency between OBJ, JSON metadata and the verification report.

## Legacy-core limitations

The current C++ parser predates the OSP interface and has several limitations:

- some parser errors are printed to standard output instead of being returned as
  typed errors;
- an unreadable model may yield an empty patch collection rather than a detailed
  parse exception;
- configuration values are applied to process-wide global state;
- one process is therefore intended to execute one configured case at a time.

The preflight layer mitigates the common user-facing parser failures. Replacing
the legacy parser and global execution state with typed C++ result/context
objects is valuable but deferred until broader unit and integration coverage is
available.

## Logging

Every maintained wrapper captures combined standard output and standard error in
`run.log`. The result manifest records all produced files, sizes and SHA-256
hashes. This provides an auditable execution record even when a downstream
verification step fails.
