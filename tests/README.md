# Product tests

The public test suite covers configuration parsing, input validation, shared-boundary behavior, command-line execution, and maintained scientific regression checks.

Run the configured test suite with:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Tests are product-facing and do not depend on manuscript or editorial material.
