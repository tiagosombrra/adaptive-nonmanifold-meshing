# Eistute stage-4 published validation

## Role of this case

This validation case reproduces and verifies the Eistute stage-4 result associated with Figure 7 of the accepted *Computers & Graphics* article:

> Adaptive Meshing of Non-Manifold Parametric Patch Complexes with Shared-Curve Compatibility

It demonstrates that the current software preserves a published numerical and geometric target after build, portability and productization changes.

It is not the general software interface. General model execution is documented in the repository README and `examples/README.md`.

## Validation target

| Metric | Expected value |
|---|---:|
| Adaptive stage | 4 |
| Triangular faces | 7,184 |
| Normalized error | 0.036712 |
| Error reduction | 96.3288% |
| Triangles with quality `q >= 0.60` | 58.6303% |
| Mean shared-curve compatibility error | 0.0 |

## Reproduction

From the repository root:

```bash
bash reproduce_eistute_linux.sh
```

The maintained configuration is:

```text
configs/eistute/article.conf
```

The normal Eistute convenience wrapper is:

```bash
bash scripts/run_eistute_linux.sh
```

## Independent checks

The validation utilities cross-check:

- the reported adaptive stage;
- the expected number of triangular faces;
- the archived metrics and metadata;
- the OBJ face count;
- the consistency of the packaged JSON and textual report.

The archived reference data currently remain under `reference/` to avoid breaking already validated Linux, Docker and container environment workflows.

## container environment

The computational capsule uses this case because it produces a compact, deterministic and independently verifiable result package. A successful capsule run creates:

```text
eistute_stage4.obj
run.log
verification.txt
result_metadata.json
```

The metadata field `verified` is true only when the packaged mesh and reported target are consistent.

## Provenance

The exact source baseline inherited from the former GRSI package is recorded in `OSP_BASELINE.md` and `SOURCE_SNAPSHOT.txt`. The validation is retained as scientific provenance and regression evidence while the OSP repository is reorganized around the reusable software.
