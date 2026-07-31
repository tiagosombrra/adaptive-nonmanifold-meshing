# Maintained `.bp` input format

## Scope

The OSP workflow currently treats the block-style `.bp` format as the maintained
user-facing input format. Other historical readers remain in the source tree,
but they are not part of the validated command-line contract until they receive
the same documentation and tests.

A `.bp` file describes a collection of bicubic tensor-product Bezier patches by
listing control points followed by patch connectivity records.

## Records

Blank lines and lines beginning with `#` are accepted by the independent
preflight validator.

### Control point

```text
v <x> <y> <z>
```

Each `v` record defines one finite three-dimensional control point. Control
points receive zero-based indices in file order.

Example:

```text
v 0.0 0.0 0.0
v 1.0 0.0 0.0
```

### Bicubic patch

```text
p i00 i10 i20 i30 i01 i11 i21 i31 i02 i12 i22 i32 i03 i13 i23 i33
```

Each `p` record must contain exactly 16 zero-based control-point indices. The
ordering is the ordering consumed by `PatchBezier`:

```text
u direction ->

p00 p10 p20 p30
p01 p11 p21 p31
p02 p12 p22 p32
p03 p13 p23 p33
```

Every index must refer to a previously or subsequently declared `v` record in
the same file; the independent validator checks the complete file before the
mesher starts.

## Shared boundaries and non-manifold configurations

Patches express shared geometry by referencing control points with identical
coordinates along corresponding boundary control polygons. During loading, the
implementation identifies repeated controls using a strict geometric tolerance
and later constructs shared curve relationships used by compatibility-preserving
meshing.

Users should not perturb nominally shared boundary controls unless a geometric
gap is intentional. The maintained examples under `input_models/` demonstrate
the expected convention.

## Preflight validation

The general runner validates configuration and `.bp` structure automatically:

```bash
bash scripts/run_case.sh --config configs/book/article.conf --build
```

Run the validator directly with:

```bash
python3 scripts/validate_case_input.py configs/book/article.conf
```

The validator checks:

- `INPUT_MODEL` and `OUTPUT_PREFIX` are present;
- the model file exists and uses the maintained `.bp` extension;
- every `v` record has exactly three finite numeric coordinates;
- every `p` record has exactly 16 integer indices;
- all indices are inside the zero-based control-point range;
- at least one control point and one patch exist;
- unsupported record types are rejected with a line number.

Use `--skip-input-check` only when diagnosing a historical input reader that is
outside the maintained OSP contract.

## Current parser limitations

The underlying C++ reader is legacy code and still assumes valid records after
the preflight stage. The release therefore treats the Python validator and C++
reader as a two-stage input boundary. Moving parsing into a typed C++ API with
structured errors is deferred until after the first OSP release because it can
affect geometric object construction and ownership.
