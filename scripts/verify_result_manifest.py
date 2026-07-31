#!/usr/bin/env python3
"""Verify the exact file inventory recorded in a result manifest."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path, PurePosixPath
from typing import NoReturn


MANIFEST_NAME = "result_manifest.json"


def fail(message: str) -> NoReturn:
    raise SystemExit(f"Manifest verification failed: {message}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_relative_path(raw_path: object) -> PurePosixPath:
    if not isinstance(raw_path, str) or not raw_path:
        fail("each file entry must contain a non-empty string path")
    if "\\" in raw_path:
        fail(f"path must use POSIX separators: {raw_path!r}")
    path = PurePosixPath(raw_path)
    if path.is_absolute() or ".." in path.parts or "." in path.parts:
        fail(f"unsafe relative path: {raw_path!r}")
    if path.as_posix() == MANIFEST_NAME:
        fail(f"{MANIFEST_NAME} must not inventory itself")
    return path


def load_manifest(manifest_path: Path) -> dict[str, object]:
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        fail(f"manifest does not exist: {manifest_path}")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"cannot read valid JSON from {manifest_path}: {exc}")
    if not isinstance(payload, dict):
        fail("manifest root must be a JSON object")
    return payload


def verify(root: Path) -> None:
    if not root.is_dir():
        fail(f"result directory does not exist: {root}")

    manifest_path = root / MANIFEST_NAME
    payload = load_manifest(manifest_path)
    entries = payload.get("files")
    if not isinstance(entries, list):
        fail("'files' must be a JSON array")
    if payload.get("file_count") != len(entries):
        fail("'file_count' does not match the number of file entries")

    recorded: dict[str, dict[str, object]] = {}
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            fail(f"file entry {index} must be a JSON object")
        relative = safe_relative_path(entry.get("path"))
        relative_text = relative.as_posix()
        if relative_text in recorded:
            fail(f"duplicate file entry: {relative_text}")
        size = entry.get("bytes")
        digest = entry.get("sha256")
        if not isinstance(size, int) or isinstance(size, bool) or size < 0:
            fail(f"invalid byte count for {relative_text}")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            fail(f"invalid SHA-256 for {relative_text}")
        recorded[relative_text] = entry

    actual = {
        path.relative_to(root).as_posix(): path
        for path in root.rglob("*")
        if path.is_file() and path != manifest_path
    }
    missing = sorted(set(recorded) - set(actual))
    unexpected = sorted(set(actual) - set(recorded))
    if missing:
        fail(f"recorded files are missing: {', '.join(missing)}")
    if unexpected:
        fail(f"unrecorded files are present: {', '.join(unexpected)}")

    for relative_text, path in sorted(actual.items()):
        entry = recorded[relative_text]
        actual_size = path.stat().st_size
        if entry["bytes"] != actual_size:
            fail(
                f"size mismatch for {relative_text}: "
                f"recorded {entry['bytes']}, actual {actual_size}"
            )
        actual_digest = sha256(path)
        if entry["sha256"] != actual_digest:
            fail(f"SHA-256 mismatch for {relative_text}")

    print(f"Result manifest verified: {manifest_path}")
    print(f"Files verified: {len(actual)}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: verify_result_manifest.py <results-dir>")
    verify(Path(sys.argv[1]))


if __name__ == "__main__":
    main()
