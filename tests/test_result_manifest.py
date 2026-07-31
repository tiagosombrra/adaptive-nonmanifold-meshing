from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
WRITE_MANIFEST = REPOSITORY_ROOT / "scripts" / "write_result_manifest.py"
VERIFY_MANIFEST = REPOSITORY_ROOT / "scripts" / "verify_result_manifest.py"


class ResultManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        (self.root / "nested").mkdir()
        (self.root / "run.log").write_text(
            "Started\nCompleted: 2026-07-27T00:00:00Z\n", encoding="utf-8"
        )
        (self.root / "nested" / "mesh.obj").write_text(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8"
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def run_script(self, script: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(script), str(self.root)],
            check=False,
            capture_output=True,
            text=True,
        )

    def write_manifest(self) -> None:
        result = self.run_script(WRITE_MANIFEST)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_final_log_hash_is_verified(self) -> None:
        self.write_manifest()
        result = self.run_script(VERIFY_MANIFEST)
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(
            (self.root / "result_manifest.json").read_text(encoding="utf-8")
        )
        log_entry = next(entry for entry in payload["files"] if entry["path"] == "run.log")
        expected = hashlib.sha256((self.root / "run.log").read_bytes()).hexdigest()
        self.assertEqual(log_entry["sha256"], expected)

    def test_modified_file_is_rejected(self) -> None:
        self.write_manifest()
        (self.root / "run.log").write_text("modified\n", encoding="utf-8")
        result = self.run_script(VERIFY_MANIFEST)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("mismatch", result.stderr)

    def test_missing_file_is_rejected(self) -> None:
        self.write_manifest()
        (self.root / "nested" / "mesh.obj").unlink()
        result = self.run_script(VERIFY_MANIFEST)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing", result.stderr)

    def test_unexpected_file_is_rejected(self) -> None:
        self.write_manifest()
        (self.root / "unexpected.txt").write_text("unexpected\n", encoding="utf-8")
        result = self.run_script(VERIFY_MANIFEST)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unrecorded", result.stderr)

    def test_unsafe_manifest_path_is_rejected(self) -> None:
        self.write_manifest()
        manifest_path = self.root / "result_manifest.json"
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        payload["files"][0]["path"] = "../outside"
        manifest_path.write_text(json.dumps(payload), encoding="utf-8")
        result = self.run_script(VERIFY_MANIFEST)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsafe", result.stderr)


if __name__ == "__main__":
    unittest.main()
