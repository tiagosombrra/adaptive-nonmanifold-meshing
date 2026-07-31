# Book example

The Book model is a maintained multi-patch use case for the general configuration-driven executable.

## Run

```bash
bash scripts/build_linux.sh
bash scripts/run_model_linux.sh book article
```

Equivalent low-level command:

```bash
bash scripts/run_case.sh \
  --config configs/book/article.conf \
  --results results/book \
  --verify "python3 scripts/verify_book.py results/book" \
  --clean-results
```

## Assets

- configuration: `configs/book/article.conf`;
- input model: referenced by `INPUT_MODEL` in the configuration;
- generated run log: `results/book/run.log`;
- verifier: `scripts/verify_book.py`.

## Interpretation

This example demonstrates normal reusable software execution. Its verifier currently checks the maintained output structure; it is not presented as an exact published numerical target.
