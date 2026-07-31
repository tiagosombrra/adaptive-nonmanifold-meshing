#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from verify_model import verify_result


@dataclass(frozen=True)
class Case:
    name: str
    label: str
    config: Path
    reference: Path | None = None
    require_final_stage: bool = False


CASES = (
    Case("book", "Book", Path("configs/book/article.conf")),
    Case(
        "decor_shelf",
        "Decor Shelf",
        Path("configs/decor_shelf/article.conf"),
    ),
    Case(
        "eistute",
        "Eistute",
        Path("configs/eistute/article.conf"),
        Path("reference/eistute_stage4.json"),
        True,
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare release runtime and peak memory against a baseline."
    )
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("reports/performance"))
    parser.add_argument("--threshold", type=float, default=0.10)
    parser.add_argument("--variation-threshold", type=float, default=0.05)
    return parser.parse_args()


def write_config(source: Path, destination: Path, output_prefix: Path) -> None:
    lines: list[str] = []
    replaced = False
    for line in source.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("OUTPUT_PREFIX="):
            lines.append(f"OUTPUT_PREFIX={output_prefix.as_posix()}")
            replaced = True
        else:
            lines.append(line)
    if not replaced:
        raise RuntimeError(f"OUTPUT_PREFIX is missing from {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_metrics(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, value = line.split("=", 1)
        values[key] = float(value)
    return {
        "seconds": values["elapsed_seconds"],
        "max_rss_kb": values["max_rss_kb"],
    }


def run_once(
    executable: Path,
    implementation: str,
    case: Case,
    run_number: int,
    output_root: Path,
) -> dict[str, float]:
    result_root = output_root / "results" / implementation / case.name
    if result_root.exists():
        shutil.rmtree(result_root)
    result_root.mkdir(parents=True)

    generated_config = (
        output_root / "configs" / implementation / f"{case.name}.conf"
    )
    output_prefix = result_root / case.name
    write_config(case.config, generated_config, output_prefix)

    metrics_path = (
        output_root
        / "measurements"
        / f"{case.name}-{implementation}-{run_number}.txt"
    )
    log_path = (
        output_root / "logs" / f"{case.name}-{implementation}-{run_number}.log"
    )
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    command = [
        "/usr/bin/time",
        "-f",
        "elapsed_seconds=%e\nmax_rss_kb=%M",
        "-o",
        str(metrics_path),
        str(executable),
        "--config",
        str(generated_config),
    ]
    with log_path.open("w", encoding="utf-8") as log:
        completed = subprocess.run(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )
    if completed.returncode != 0:
        raise RuntimeError(
            f"{case.name}/{implementation} run {run_number} failed "
            f"with status {completed.returncode}; see {log_path}"
        )

    verify_result(
        result_root,
        case.label,
        case.config,
        case.reference,
        case.require_final_stage,
    )
    return parse_metrics(metrics_path)


def relative_variation(values: list[float]) -> float:
    median = statistics.median(values)
    if median == 0.0:
        return 0.0 if max(values) == 0.0 else float("inf")
    return (max(values) - min(values)) / median


def summarize(samples: list[dict[str, float]]) -> dict[str, object]:
    times = [sample["seconds"] for sample in samples]
    memory = [sample["max_rss_kb"] for sample in samples]
    return {
        "runs": len(samples),
        "seconds": times,
        "max_rss_kb": memory,
        "median_seconds": statistics.median(times),
        "median_max_rss_kb": statistics.median(memory),
        "time_variation": relative_variation(times),
        "memory_variation": relative_variation(memory),
    }


def markdown_report(report: dict[str, object]) -> str:
    rows = [
        "# Release performance comparison",
        "",
        "| Case | Runs | Baseline time (s) | Candidate time (s) | Time change | "
        "Baseline RSS (KiB) | Candidate RSS (KiB) | RSS change | Gate |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for name, result in report["cases"].items():
        baseline = result["baseline"]
        candidate = result["candidate"]
        rows.append(
            "| {name} | {runs} | {bt:.3f} | {ct:.3f} | {td:+.2%} | "
            "{bm:.0f} | {cm:.0f} | {md:+.2%} | {gate} |".format(
                name=name,
                runs=max(baseline["runs"], candidate["runs"]),
                bt=baseline["median_seconds"],
                ct=candidate["median_seconds"],
                td=result["time_change"],
                bm=baseline["median_max_rss_kb"],
                cm=candidate["median_max_rss_kb"],
                md=result["memory_change"],
                gate="pass" if result["passed"] else "FAIL",
            )
        )
    rows.extend(
        (
            "",
            f"Regression threshold: {report['threshold']:.0%}.",
            "Cases with more than "
            f"{report['variation_threshold']:.0%} variation were extended "
            "from three to five runs.",
            "",
        )
    )
    return "\n".join(rows)


def main() -> int:
    args = parse_args()
    baseline = args.baseline.resolve()
    candidate = args.candidate.resolve()
    for executable in (baseline, candidate):
        if not executable.is_file():
            raise SystemExit(f"Executable not found: {executable}")
    if not Path("/usr/bin/time").is_file():
        raise SystemExit("GNU time is required at /usr/bin/time")

    output_root = args.output.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "threshold": args.threshold,
        "variation_threshold": args.variation_threshold,
        "cases": {},
    }
    failed = False

    for case in CASES:
        samples: dict[str, list[dict[str, float]]] = {
            "baseline": [],
            "candidate": [],
        }
        executables = {"baseline": baseline, "candidate": candidate}
        for run_number in range(1, 4):
            for implementation in ("baseline", "candidate"):
                samples[implementation].append(
                    run_once(
                        executables[implementation],
                        implementation,
                        case,
                        run_number,
                        output_root,
                    )
                )

        summaries = {
            name: summarize(measurements)
            for name, measurements in samples.items()
        }
        needs_more_runs = any(
            summary["time_variation"] > args.variation_threshold
            or summary["memory_variation"] > args.variation_threshold
            for summary in summaries.values()
        )
        if needs_more_runs:
            for run_number in range(4, 6):
                for implementation in ("baseline", "candidate"):
                    samples[implementation].append(
                        run_once(
                            executables[implementation],
                            implementation,
                            case,
                            run_number,
                            output_root,
                        )
                    )
            summaries = {
                name: summarize(measurements)
                for name, measurements in samples.items()
            }

        baseline_summary = summaries["baseline"]
        candidate_summary = summaries["candidate"]
        time_change = (
            candidate_summary["median_seconds"]
            / baseline_summary["median_seconds"]
            - 1.0
        )
        memory_change = (
            candidate_summary["median_max_rss_kb"]
            / baseline_summary["median_max_rss_kb"]
            - 1.0
        )
        passed = (
            time_change <= args.threshold and memory_change <= args.threshold
        )
        failed = failed or not passed
        report["cases"][case.name] = {
            "baseline": baseline_summary,
            "candidate": candidate_summary,
            "time_change": time_change,
            "memory_change": memory_change,
            "passed": passed,
        }

    json_path = output_root / "performance-report.json"
    markdown_path = output_root / "performance-report.md"
    json_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    markdown = markdown_report(report)
    markdown_path.write_text(markdown, encoding="utf-8")
    print(markdown)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
