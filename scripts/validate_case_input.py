#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(f"Input validation failed: {message}")


def is_comment_or_empty(line: str) -> bool:
    return not line or line.startswith("#") or line.startswith("//")


def read_config(path: Path) -> dict[str, str]:
    if not path.is_file():
        fail(f"configuration file not found: {path}")
    values: dict[str, str] = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if is_comment_or_empty(line):
            continue
        if "=" not in line:
            fail(f"{path}:{line_number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            fail(f"{path}:{line_number}: empty configuration key")
        values[key] = value
    return values


def validate_bp(path: Path) -> tuple[int, int]:
    if not path.is_file():
        fail(f"input model not found: {path}")

    vertices: list[tuple[float, float, float]] = []
    patches: list[tuple[int, list[int]]] = []

    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if is_comment_or_empty(line):
            continue
        fields = line.split()
        record = fields[0]

        if record == "v":
            if len(fields) != 4:
                fail(f"{path}:{line_number}: vertex record must contain exactly 3 coordinates")
            try:
                point = tuple(float(value) for value in fields[1:])
            except ValueError:
                fail(f"{path}:{line_number}: vertex coordinates must be numeric")
            if not all(math.isfinite(value) for value in point):
                fail(f"{path}:{line_number}: vertex coordinates must be finite")
            vertices.append(point)  # type: ignore[arg-type]
            continue

        if record == "p":
            if len(fields) != 17:
                fail(
                    f"{path}:{line_number}: bicubic Bezier patch must reference "
                    "exactly 16 control-point indices"
                )
            try:
                indices = [int(value) for value in fields[1:]]
            except ValueError:
                fail(f"{path}:{line_number}: patch indices must be integers")
            patches.append((line_number, indices))
            continue

        fail(f"{path}:{line_number}: unsupported record type: {record}")

    if not vertices:
        fail(f"{path}: no vertex records were found")
    if not patches:
        fail(f"{path}: no patch records were found")

    max_index = len(vertices) - 1
    for line_number, indices in patches:
        for index in indices:
            if index < 0 or index > max_index:
                fail(
                    f"{path}:{line_number}: control-point index {index} is outside "
                    f"the zero-based range 0..{max_index}"
                )

    return len(vertices), len(patches)


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 scripts/validate_case_input.py <configuration.conf>")

    config_path = Path(sys.argv[1])
    values = read_config(config_path)

    input_value = values.get("INPUT_MODEL", "").strip()
    output_value = values.get("OUTPUT_PREFIX", "").strip()
    if not input_value:
        fail(f"{config_path}: INPUT_MODEL is required")
    if not output_value:
        fail(f"{config_path}: OUTPUT_PREFIX is required")

    input_path = Path(input_value)
    suffix = input_path.suffix.lower()
    if suffix != ".bp":
        fail(
            f"unsupported maintained input format '{suffix or '<none>'}'; "
            "the validated OSP workflow currently supports .bp"
        )

    vertex_count, patch_count = validate_bp(input_path)
    print("Input preflight passed")
    print(f"Configuration: {config_path}")
    print(f"Input model: {input_path}")
    print(f"Control points: {vertex_count}")
    print(f"Bicubic patches: {patch_count}")
    print(f"Output prefix: {output_value}")


if __name__ == "__main__":
    main()
