#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
from pathlib import Path

REFERENCE_DIR = Path("reference")
JSON_PATH = REFERENCE_DIR / "eistute_stage4.json"
MESH_PATH = REFERENCE_DIR / "eistute_stage4.obj"
CSV_PATH = REFERENCE_DIR / "eistute_metrics.csv"
TOLERANCE = 1.0e-6


def fail(message: str) -> None:
    raise SystemExit(f"Reference verification failed: {message}")


def close_enough(actual: float, expected: float) -> bool:
    return abs(actual - expected) <= TOLERANCE


for path in (JSON_PATH, MESH_PATH, CSV_PATH):
    if not path.is_file():
        fail(f"missing archived reference file: {path}")

expected = json.loads(JSON_PATH.read_text(encoding="utf-8"))
triangles = sum(
    1
    for line in MESH_PATH.open(encoding="utf-8", errors="ignore")
    if line.startswith("f ")
)
if triangles != int(expected["triangles"]):
    fail(f"archived mesh has {triangles} triangles, expected {expected['triangles']}")

matches: list[dict[str, str]] = []
with CSV_PATH.open(encoding="utf-8", newline="") as csv_file:
    for row in csv.DictReader(csv_file):
        try:
            if (
                int(row["step"]) == int(expected["stage"])
                and int(row["elements"]) == triangles
            ):
                matches.append(row)
        except (KeyError, TypeError, ValueError):
            continue

if not matches:
    fail("no CSV row matches the archived Eistute stage and triangle count")

metric_map = {
    "error_normalized": float(expected["normalized_error"]),
    "error_total_reduction_pct": float(expected["error_reduction_percent"]),
    "q_ge_0_60": float(expected["quality_q_ge_0_60_percent"]),
}

verified = None
for row in matches:
    try:
        if all(close_enough(float(row[column]), value) for column, value in metric_map.items()):
            verified = row
            break
    except (KeyError, TypeError, ValueError):
        continue

if verified is None:
    fail("archived CSV metrics do not match the Eistute JSON targets")

print("Archived Eistute reference verified.")
print(f"Triangles: {triangles}")
print(f"Normalized error: {verified['error_normalized']}")
print(f"Error reduction (%): {verified['error_total_reduction_pct']}")
print(f"Quality q >= 0.60 (%): {verified['q_ge_0_60']}")
print(
    "Shared-curve compatibility target recorded in JSON: "
    f"{expected['mean_shared_curve_compatibility_error']}"
)
