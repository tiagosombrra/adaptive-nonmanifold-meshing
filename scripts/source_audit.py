#!/usr/bin/env python3
"""Generate a non-mutating source-quality inventory for the C++ codebase."""
from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

SOURCE_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
HEADER_EXTENSIONS = {".h", ".hh", ".hpp", ".hxx"}
LONG_LINE_LIMIT = 120
LARGE_FILE_LIMIT = 1000
MAX_EXAMPLES = 20

PATTERNS = {
    "todo_markers": re.compile(r"\b(?:TODO|FIXME|HACK|XXX)\b", re.IGNORECASE),
    "using_namespace": re.compile(r"\busing\s+namespace\s+[A-Za-z_][\w:]*\s*;"),
    "raw_new": re.compile(r"\bnew\s+(?!\()"),
    "raw_delete": re.compile(r"\bdelete(?:\s*\[\s*\])?\b"),
    "c_allocation": re.compile(r"\b(?:malloc|calloc|realloc|free)\s*\("),
    "c_stdio": re.compile(r"\b(?:printf|fprintf|sprintf|snprintf|puts|fputs)\s*\("),
    "process_exit": re.compile(r"\b(?:exit|abort)\s*\("),
    "goto": re.compile(r"\bgoto\b"),
    "null_macro": re.compile(r"\bNULL\b"),
    "assert": re.compile(r"\bassert\s*\("),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--json", type=Path, default=Path("reports/source-audit.json"))
    parser.add_argument("--markdown", type=Path, default=Path("reports/source-audit.md"))
    return parser.parse_args()


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for directory in ("src", "include"):
        base = root / directory
        if not base.is_dir():
            continue
        files.extend(
            path
            for path in base.rglob("*")
            if path.is_file() and path.suffix.lower() in SOURCE_EXTENSIONS
        )
    return sorted(set(files))


def has_header_guard(lines: list[str]) -> bool:
    head = "\n".join(lines[:60])
    if re.search(r"^\s*#\s*pragma\s+once\b", head, re.MULTILINE):
        return True
    match = re.search(r"^\s*#\s*ifndef\s+([A-Za-z_]\w*)\s*$", head, re.MULTILINE)
    if not match:
        return False
    macro = re.escape(match.group(1))
    return bool(re.search(rf"^\s*#\s*define\s+{macro}\b", head, re.MULTILINE))


def safe_excerpt(text: str, limit: int = 180) -> str:
    text = text.strip().replace("\t", "\\t").replace("`", "'")
    return text if len(text) <= limit else text[: limit - 3] + "..."


def analyze(root: Path) -> dict[str, object]:
    files = source_files(root)
    if not files:
        raise SystemExit("No C/C++ files found under src/ and include/.")

    stats: list[dict[str, object]] = []
    pattern_counts: Counter[str] = Counter()
    pattern_examples: dict[str, list[dict[str, object]]] = defaultdict(list)
    duplicate_names: dict[str, list[str]] = defaultdict(list)
    header_guard_issues: list[str] = []

    for path in files:
        raw = path.read_bytes()
        text = raw.decode("utf-8", errors="replace")
        lines = text.splitlines()
        rel = path.relative_to(root).as_posix()
        duplicate_names[path.name].append(rel)

        long_lines = 0
        tab_lines = 0
        trailing_whitespace = 0
        blank_lines = 0
        comment_only_lines = 0
        max_columns = 0
        in_block_comment = False

        for number, line in enumerate(lines, start=1):
            columns = len(line.expandtabs(4))
            max_columns = max(max_columns, columns)
            long_lines += columns > LONG_LINE_LIMIT
            tab_lines += "\t" in line
            trailing_whitespace += line.rstrip(" \t") != line
            blank_lines += not line.strip()

            stripped = line.strip()
            comment_only = False
            if in_block_comment:
                comment_only = True
                if "*/" in stripped:
                    in_block_comment = False
            elif stripped.startswith("//"):
                comment_only = True
            elif stripped.startswith("/*"):
                comment_only = True
                if "*/" not in stripped[2:]:
                    in_block_comment = True
            elif stripped.startswith("*"):
                comment_only = True
            comment_only_lines += comment_only

            for key, pattern in PATTERNS.items():
                matches = list(pattern.finditer(line))
                if not matches:
                    continue
                pattern_counts[key] += len(matches)
                if len(pattern_examples[key]) < MAX_EXAMPLES:
                    pattern_examples[key].append(
                        {
                            "path": rel,
                            "line": number,
                            "excerpt": safe_excerpt(line),
                        }
                    )

        if path.suffix.lower() in HEADER_EXTENSIONS and not has_header_guard(lines):
            header_guard_issues.append(rel)

        stats.append(
            {
                "path": rel,
                "extension": path.suffix.lower(),
                "bytes": len(raw),
                "lines": len(lines),
                "blank_lines": blank_lines,
                "comment_only_lines": comment_only_lines,
                "max_columns": max_columns,
                "long_lines": long_lines,
                "tab_lines": tab_lines,
                "trailing_whitespace_lines": trailing_whitespace,
                "crlf": b"\r\n" in raw,
                "final_newline": (not raw or raw.endswith(b"\n")),
            }
        )

    duplicates = {
        name: paths for name, paths in sorted(duplicate_names.items()) if len(paths) > 1
    }
    summary = {
        "source_files": len(stats),
        "header_files": sum(item["extension"] in HEADER_EXTENSIONS for item in stats),
        "implementation_files": sum(item["extension"] not in HEADER_EXTENSIONS for item in stats),
        "physical_lines": sum(int(item["lines"]) for item in stats),
        "blank_lines": sum(int(item["blank_lines"]) for item in stats),
        "comment_only_lines": sum(int(item["comment_only_lines"]) for item in stats),
        "large_files": sum(int(item["lines"]) > LARGE_FILE_LIMIT for item in stats),
        "long_lines": sum(int(item["long_lines"]) for item in stats),
        "files_with_tabs": sum(int(item["tab_lines"]) > 0 for item in stats),
        "files_with_trailing_whitespace": sum(
            int(item["trailing_whitespace_lines"]) > 0 for item in stats
        ),
        "crlf_files": sum(bool(item["crlf"]) for item in stats),
        "missing_final_newline_files": sum(not bool(item["final_newline"]) for item in stats),
    }
    return {
        "summary": summary,
        "files": stats,
        "patterns": {key: pattern_counts.get(key, 0) for key in sorted(PATTERNS)},
        "pattern_examples": {
            key: pattern_examples.get(key, []) for key in sorted(PATTERNS)
        },
        "header_guard_issues": sorted(header_guard_issues),
        "duplicate_basenames": duplicates,
    }


def table(headers: list[str], rows: list[list[str]]) -> str:
    output = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    output.extend("| " + " | ".join(row) + " |" for row in rows)
    return "\n".join(output)


def render_markdown(report: dict[str, object]) -> str:
    summary = report["summary"]
    files = report["files"]
    patterns = report["patterns"]
    examples = report["pattern_examples"]
    assert isinstance(summary, dict)
    assert isinstance(files, list)
    assert isinstance(patterns, dict)
    assert isinstance(examples, dict)

    inventory_rows = [[key.replace("_", " ").title(), str(value)] for key, value in summary.items()]
    largest = sorted(files, key=lambda item: int(item["lines"]), reverse=True)[:20]

    lines = [
        "# Source quality audit",
        "",
        "> Diagnostic baseline only. The audit does not rewrite scientific code.",
        "",
        "## Inventory",
        "",
        table(["Metric", "Value"], inventory_rows),
        "",
        "## Largest files",
        "",
        table(
            ["File", "Lines", "Max columns", ">120 columns"],
            [
                [
                    "`" + str(item["path"]) + "`",
                    str(item["lines"]),
                    str(item["max_columns"]),
                    str(item["long_lines"]),
                ]
                for item in largest
            ],
        ),
        "",
        "## Lexical indicators",
        "",
        table(
            ["Indicator", "Occurrences"],
            [["`" + key + "`", str(value)] for key, value in patterns.items()],
        ),
    ]

    for key, entries in examples.items():
        if not entries:
            continue
        lines.extend(["", "### `" + key + "` examples", ""])
        for entry in entries[:10]:
            lines.append(
                "- `{}:{}` — `{}`".format(
                    entry["path"], entry["line"], entry["excerpt"]
                )
            )

    lines.extend(["", "## File hygiene", ""])
    lines.append(
        table(
            ["Check", "Count"],
            [
                ["Headers without detected guard", str(len(report["header_guard_issues"]))],
                ["Duplicate basenames", str(len(report["duplicate_basenames"]))],
                ["Files containing tabs", str(summary["files_with_tabs"])],
                ["Files with trailing whitespace", str(summary["files_with_trailing_whitespace"])],
            ],
        )
    )

    guard_issues = report["header_guard_issues"]
    assert isinstance(guard_issues, list)
    if guard_issues:
        lines.extend(["", "### Headers without a detected guard", ""])
        lines.extend("- `" + path + "`" for path in guard_issues[:50])

    duplicates = report["duplicate_basenames"]
    assert isinstance(duplicates, dict)
    if duplicates:
        lines.extend(["", "### Duplicate basenames", ""])
        for name, paths in duplicates.items():
            lines.append("- `" + name + "`: " + ", ".join("`" + path + "`" for path in paths))

    lines.extend(
        [
            "",
            "## Cleanup constraints",
            "",
            "1. Preserve the verified Eistute stage-4 result of 7,184 triangles.",
            "2. Keep mechanical cleanup separate from algorithmic changes.",
            "3. Require Linux, Docker and packaged-result regression checks.",
            "4. Add targeted tests before changing ownership or numerical logic.",
            "5. Document intentionally retained legacy behavior.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    root = args.root.resolve()
    report = analyze(root)

    json_path = args.json if args.json.is_absolute() else root / args.json
    markdown_path = args.markdown if args.markdown.is_absolute() else root / args.markdown
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")

    summary = report["summary"]
    assert isinstance(summary, dict)
    print("Source files:", summary["source_files"])
    print("Physical lines:", summary["physical_lines"])
    print("Large files:", summary["large_files"])
    print("Long lines:", summary["long_lines"])
    print("Markdown report:", markdown_path)
    print("JSON report:", json_path)


if __name__ == "__main__":
    main()
