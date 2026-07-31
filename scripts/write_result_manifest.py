#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "Usage: python3 scripts/write_result_manifest.py <results-dir> [config]"
        )

    root = Path(sys.argv[1])
    config = Path(sys.argv[2]) if len(sys.argv) == 3 else None
    if not root.is_dir():
        raise SystemExit(f"Result directory does not exist: {root}")

    manifest_path = root / "result_manifest.json"
    files = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path == manifest_path:
            continue
        files.append(
            {
                "path": path.relative_to(root).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )

    payload = {
        "software": "Adaptive Non-Manifold Meshing",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "results_root": root.as_posix(),
        "configuration": config.as_posix() if config is not None else None,
        "file_count": len(files),
        "files": files,
    }
    manifest_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Result manifest written: {manifest_path}")
    print(f"Files inventoried: {len(files)}")


if __name__ == "__main__":
    main()
