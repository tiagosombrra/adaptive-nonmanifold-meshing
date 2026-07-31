# Reproducibility

The repository provides maintained configurations, input models, reference outputs, and verification scripts for the public software.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

## Maintained cases

```bash
bash scripts/run_book_linux.sh
bash scripts/run_decor_shelf_linux.sh
bash scripts/run_eistute_linux.sh
```

The principal published regression is reproduced with:

```bash
bash reproduce_eistute_linux.sh
```

Environment and toolchain details should be recorded with each independent run. Generated result directories are intentionally excluded from version control.
