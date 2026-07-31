# Configuration-driven execution

## Command line

The supported executable interface is:

```bash
build/bin/ap_mesh --config <path-to-configuration.conf>
```

Repository users can use the general wrapper:

```bash
scripts/run_model_linux.sh <model> [profile] [--no-verify]
```

Examples:

```bash
bash scripts/build_linux.sh
bash scripts/run_model_linux.sh book
bash scripts/run_model_linux.sh eistute article
bash scripts/run_model_linux.sh decor_shelf article --no-verify
```

## Required keys

Every configuration must define:

| Key | Meaning |
|---|---|
| `INPUT_MODEL` | Path to the parametric patch-complex input file |
| `OUTPUT_PREFIX` | Prefix/path used by generated meshes, logs and diagnostics |

If either value is absent, the executable returns an error before meshing.

Configuration parsing is strict. Non-comment lines must use `KEY=VALUE`;
duplicate or unknown keys are rejected; numeric values must be finite and
within the documented range; and `INPUT_MODEL` must identify a readable file.
Invalid configuration returns status `3` without starting the mesher. Both `#`
and `//` introduce full-line comments.

## Core execution keys

| Key | Default | Meaning |
|---|---:|---|
| `NUM_PROCESSES` | `1` | Requested process count; MPI support is optional at build time |
| `NUM_THREADS` | `1` | Requested worker-thread count |
| `WRITE_MODE` | `h` | Output mode consumed by the legacy generator interface |
| `USE_TEMPLATE` | `y` | Enable the template-based reconstruction path |
| `ADAPTIVE_MODE` | `adaptive_stable` | Adaptive strategy identifier |
| `ADAPTIVE_MAX_STEPS` | `6` | Highest adaptive stage/iteration requested |
| `ADAPTIVE_INTENSITY` | `0.45` | Overall adaptive intensity in `[0,1]` |
| `ADAPTIVE_QUALITY_PRIORITY` | `0.75` | Quality priority in `[0,1]` |
| `ADAPTIVE_TARGET_GROWTH` | `1.6` | Target element-growth factor, at least `1` |
| `ENABLE_SHARED_CURVE_SYNC` | `1` | Preserve synchronized discretization on shared curves |
| `ENABLE_HYBRID_RECONSTRUCTION` | `1` | Enable hybrid reconstruction; disabling it also disables templates |
| `WRITE_RUNTIME_SUMMARY` | `1` | Write the runtime summary and diagnostics |

## Adaptive controls

The implementation supports additional controls for:

- minimum error improvement and patience;
- local error tolerance;
- Laplacian smoothing;
- adaptation relaxation and maximum per-step variation;
- retry behavior;
- patch refinement/coarsening strengths;
- curve adaptation and point budgets;
- quadtree depth and quality thresholds;
- local advancing-front post-processing;
- shared-curve and patch-consistency controls;
- step-level element budgets and acceptance criteria.

The distributed profiles under `configs/<model>/` are the authoritative tested
examples. Not every internal global parameter is currently exposed through
`RuntimeConfig`; undocumented globals should be treated as implementation
details until they are migrated into the typed configuration layer.

## Profiles

A profile is simply a `.conf` file under a model directory:

```text
configs/
├── book/
│   └── article.conf
├── eistute/
│   └── article.conf
└── decor_shelf/
    └── article.conf
```

The name `article` identifies the configuration used for the published
evaluation package. Future reusable profiles may use names such as `fast`,
`balanced`, `quality`, or `custom`, provided that their assumptions and expected
runtime are documented.

## Validation versus normal execution

`run_model_linux.sh` runs the selected configuration and, by default, invokes
the model's available verifier. Use `--no-verify` for ordinary exploratory runs
where no archived target is expected:

```bash
scripts/run_model_linux.sh book custom --no-verify
```

A successful normal execution means that the program completed and generated
outputs. A successful published validation additionally means that selected
outputs matched an archived scientific target.

## Current limitation

The typed `RuntimeConfig` object is mapped into legacy process-wide global
variables before execution. Therefore, one process should perform one configured
run at a time. Independent simultaneous runs should use separate processes and
separate output directories.
