# Input and output contract

## Status

This document records the current externally visible contract of the OSP application. It distinguishes supported behavior from implementation details that still require parser-level documentation.

## Input model

The executable receives the model path through the required `INPUT_MODEL` configuration key.

Maintained models use the `.bp` representation stored under `input_models/`.
These files contain finite three-dimensional control points and bicubic
Bézier-patch connectivity. Repeated boundary control geometry establishes the
shared relationships used to construct the patch complex.

The maintained grammar and its zero-based 16-index patch ordering are
documented in [`BP_INPUT_FORMAT.md`](BP_INPUT_FORMAT.md). Historical readers
for other representations remain implementation details outside the v1
command-line contract.

### Current supported assumptions

- the input file must exist and be readable;
- each `v` record must contain exactly three finite coordinates;
- each `p` record must contain exactly 16 integer indices;
- every patch index must resolve to a control point in the same file;
- at least one control point and one patch must exist;
- record types outside the maintained grammar are rejected before meshing.

### Parser limitations

- the C++ reader retains inherited delimiter-based internals after preflight;
- strict line-numbered diagnostics are provided by the maintained Python
  preflight rather than every historical C++ reader;
- the current release validates the distributed `.bp` contract and does not
  claim support for arbitrary historical model formats.

## Configuration input

The executable is invoked with:

```bash
build/bin/ap_mesh --config <file.conf>
```

Configuration files are key-value text files. `INPUT_MODEL` and `OUTPUT_PREFIX` are required. The stable configuration subset is documented in [`CONFIGURATION.md`](CONFIGURATION.md).

## Output location

`OUTPUT_PREFIX` is consumed by the inherited writer and diagnostic code. It may contain a relative path and filename prefix. Parent directories required by the selected configuration should exist or be created by the calling workflow.

The repository wrappers create the directory passed through `--results` and
capture console output in:

```text
results/<case>/run.log
```

Scientific output files are written to the parent of `OUTPUT_PREFIX`. For the
maintained profiles, that parent is the same directory passed through
`--results`. Custom callers must keep those two locations aligned when they
want the manifest to cover every generated file.

After execution and optional scientific verification complete, each maintained
wrapper writes `result_manifest.json`. The manifest inventories every other
regular file below the case result directory using a relative path, byte count
and SHA-256 digest. It is generated only after `run.log` is final and can be
checked independently with:

```bash
python3 scripts/verify_result_manifest.py results/<case>
```

Verification fails if an inventoried file is missing or modified, if an
unrecorded file is present, or if the manifest structure contains duplicate or
unsafe paths.

## Mesh outputs

The primary geometric output is a Wavefront OBJ triangular mesh. Depending on the write mode and adaptive workflow, the application may emit intermediate and final meshes for multiple stages.

OBJ verification utilities count lines beginning with `f ` as triangular faces. The published Eistute validation requires exactly 7,184 faces in the stage-4 target mesh.

## Maintained profile inventory

Book, Decor Shelf and Eistute use the same output families, with `<prefix>`
equal to `book`, `decor_shelf` or `eistute` and `<stage>` ranging from zero to
the highest candidate produced:

| Pattern | Meaning | Write behavior |
|---|---|---|
| `<prefix>_passo_<stage>_malha_<stage>.obj` | Triangular mesh for an accepted or retained rejected candidate | Replaced for the same stage |
| `<prefix>_n.process__passo_<stage>_qualite_<stage>_rank_-1.log` | Per-stage mesh-quality distribution | Replaced for the same stage |
| `<prefix>_passo_<stage>_acceptance_<accepted-or-rejected>.log` | Hybrid acceptance decision and thresholds | Replaced for the same stage/status |
| `<prefix>_passo_<stage>_curve_adaptation_<accepted-or-rejected>.csv` | Per-curve refinement decisions | Replaced for the same stage/status |
| `<prefix>_passo_<stage>_patch_adaptation_<accepted-or-rejected>.csv` | Per-patch refinement, budget and quality diagnostics | Replaced for the same stage/status |
| `<prefix>_n.process__metrics_rank_-1.csv` | Machine-readable stage metrics | Created with a header, then appended by stage |
| `<prefix>_domain_adaptation.csv` | Domain reconstruction timing/diagnostics | Created for the run |
| `<prefix>_compatibility_summary.csv` | Shared-curve compatibility summary | Created with a header, then appended per execution |
| `<prefix>_runtime_summary.csv` | Timer-stage summary | Created with a header, then appended per execution |
| `run.log` | Complete wrapper and executable output | Replaced when `--clean-results` is used |
| `result_manifest.json` | Relative paths, sizes and SHA-256 hashes for all preceding files | Replaced after the log is finalized |

Stage zero does not have curve- or patch-adaptation CSV files because no
adaptation precedes the initial mesh. Later candidate stages retain their OBJ
and diagnostics even when the hybrid criterion rejects them; the acceptance
filename and contents distinguish that state.

The maintained Book profile can produce stages 0--3, Decor Shelf stages 0--5,
and Eistute stages 0--4 under their current article configurations. Book and
Decor Shelf have structural contracts: at least one non-empty triangular stage
mesh no later than the configured maximum. Eistute additionally requires its
stage-4 mesh to contain exactly 7,184 faces.

The wrappers use `--clean-results`, so a normal maintained run begins from an
empty case directory and does not mix appended summaries with an older
execution. Without that option, same-name truncated files are overwritten
while the documented append-mode CSVs retain prior rows. Concurrent writers
must never share one output prefix.

## Diagnostics and metrics

Enabled configurations may generate:

- runtime summaries;
- stage and iteration information;
- element counts;
- normalized error measurements;
- error reduction;
- triangle-quality distributions;
- shared-curve compatibility measurements;
- adaptation decisions and debugging reports;
- timing information;
- FindUV convergence statistics.

Not every diagnostic is guaranteed for every configuration. Stable machine-readable outputs must be identified per maintained workflow.

## Published validation package

The container environment workflow creates a compact result package containing:

```text
eistute_stage4.obj
run.log
verification.txt
result_metadata.json
```

`result_metadata.json` records the software name, validation identity, source mesh, output mesh, stage, actual triangle count, expected triangle count and verification status.

This package is a validation contract, not the complete output contract of the application.

## Exit status

The public executable returns:

- `0` for success;
- `2` for invalid command-line usage;
- `3` for invalid configuration or input;
- `4` for a propagated numerical execution failure.

The generic wrapper additionally returns distinct nonzero statuses for missing arguments, missing configuration, missing executable and failed verification.

## Output-safety recommendations

- use a separate output directory for each run;
- do not run two configurations in the same process;
- avoid sharing one output prefix among simultaneous processes;
- archive the configuration file with scientific results;
- use maintained verifiers when comparing against a known target;
- inspect `run.log` even when mesh generation succeeds.

## Post-v1 scope

The maintained `.bp` grammar, strict configuration boundary, output inventory,
directory behavior, result manifest and error statuses are release contracts.
Historical readers and naming modernization remain outside this contract and
are tracked in [`ROADMAP.md`](ROADMAP.md).
