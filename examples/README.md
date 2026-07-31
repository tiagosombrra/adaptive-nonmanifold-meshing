# Maintained examples

The examples demonstrate how to run Adaptive Non-Manifold Meshing through the general configuration-driven interface.

They are user-oriented entry points. Exact numerical reproduction of a published result is documented separately under `validation/`.

## Common workflow

Build once:

```bash
bash scripts/build_linux.sh
```

Run any configuration:

```bash
bash scripts/run_case.sh \
  --config <configuration.conf> \
  --results <results-directory> \
  --clean-results
```

A maintained verifier can be supplied with `--verify`.

## Book

```bash
bash scripts/run_book_linux.sh
```

Configuration:

```text
configs/book/article.conf
```

The Book case exercises a multi-patch model and verifies that the expected output structure is produced.

## Decor Shelf

```bash
bash scripts/run_decor_shelf_linux.sh
```

Configuration:

```text
configs/decor_shelf/article.conf
```

The Decor Shelf case provides an additional complex patch layout for integration testing and qualitative evaluation.

## Eistute

```bash
bash scripts/run_eistute_linux.sh
```

Configuration:

```text
configs/eistute/article.conf
```

Eistute is both a maintained use case and the source model for the strongest archived published validation. General execution belongs here; the exact stage-4 target and its provenance belong under `validation/`.

## Adding a new example

A new maintained example should provide:

1. an input model under `input_models/`;
2. a readable configuration under `configs/<case>/`;
3. a short description of the geometric purpose;
4. a command using `scripts/run_case.sh`;
5. a verifier when a stable structural or numerical contract is known;
6. provenance and redistribution information for the model.

Examples should not embed article-specific language unless they are explicitly linked to a published validation case.
