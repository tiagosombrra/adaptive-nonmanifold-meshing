#!/usr/bin/env python3
"""Summarize cppcheck XML output as Markdown and JSON."""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

MAX_EXAMPLES = 100


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml", type=Path)
    parser.add_argument("markdown", type=Path)
    parser.add_argument("json", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    tree = ET.parse(args.xml)
    root = tree.getroot()

    entries: list[dict[str, object]] = []
    severity_counts: Counter[str] = Counter()
    id_counts: Counter[str] = Counter()
    file_counts: Counter[str] = Counter()

    errors_node = root.find("errors")
    if errors_node is not None:
        for error in errors_node.findall("error"):
            severity = error.attrib.get("severity", "unknown")
            issue_id = error.attrib.get("id", "unknown")
            message = error.attrib.get("verbose") or error.attrib.get("msg", "")
            locations = error.findall("location")
            location = locations[0] if locations else None
            file_name = location.attrib.get("file", "") if location is not None else ""
            line_raw = location.attrib.get("line", "0") if location is not None else "0"
            try:
                line = int(line_raw)
            except ValueError:
                line = 0

            severity_counts[severity] += 1
            id_counts[issue_id] += 1
            if file_name:
                file_counts[file_name] += 1

            if len(entries) < MAX_EXAMPLES:
                entries.append(
                    {
                        "severity": severity,
                        "id": issue_id,
                        "file": file_name,
                        "line": line,
                        "message": message,
                    }
                )

    report = {
        "total": sum(severity_counts.values()),
        "severities": dict(severity_counts.most_common()),
        "issue_ids": dict(id_counts.most_common()),
        "files": dict(file_counts.most_common()),
        "examples": entries,
    }

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    lines = [
        "# cppcheck summary",
        "",
        f"Total diagnostics: **{report['total']}**",
        "",
        "## By severity",
        "",
        "| Severity | Count |",
        "|---|---:|",
    ]
    if severity_counts:
        lines.extend(f"| `{name}` | {count} |" for name, count in severity_counts.most_common())
    else:
        lines.append("| — | 0 |")

    lines.extend(
        [
            "",
            "## Most frequent diagnostic IDs",
            "",
            "| ID | Count |",
            "|---|---:|",
        ]
    )
    if id_counts:
        lines.extend(f"| `{name}` | {count} |" for name, count in id_counts.most_common(30))
    else:
        lines.append("| — | 0 |")

    lines.extend(
        [
            "",
            "## Files with most diagnostics",
            "",
            "| File | Count |",
            "|---|---:|",
        ]
    )
    if file_counts:
        lines.extend(f"| `{name}` | {count} |" for name, count in file_counts.most_common(30))
    else:
        lines.append("| — | 0 |")

    if entries:
        lines.extend(["", "## Diagnostic examples", ""])
        for entry in entries:
            location = entry["file"] or "<unknown>"
            if entry["line"]:
                location = f"{location}:{entry['line']}"
            message = str(entry["message"]).replace("`", "\\`")
            lines.append(
                f"- `{entry['severity']}` `{entry['id']}` at `{location}` — {message}"
            )

    lines.append("")
    args.markdown.write_text("\n".join(lines), encoding="utf-8")
    print(f"cppcheck diagnostics: {report['total']}")
    print(f"Markdown summary: {args.markdown}")
    print(f"JSON summary: {args.json}")


if __name__ == "__main__":
    main()
