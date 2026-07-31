from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def configured_executable() -> Path | None:
    raw_path = os.environ.get("AP_MESH_TEST_EXECUTABLE")
    if not raw_path:
        return None
    return Path(raw_path).resolve()


@unittest.skipUnless(
    configured_executable() is not None, "AP_MESH_TEST_EXECUTABLE is not set"
)
class CommandLineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.executable = configured_executable()
        assert cls.executable is not None
        if not cls.executable.is_file():
            raise AssertionError(f"test executable does not exist: {cls.executable}")

    def run_cli(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(self.executable), *arguments],
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def write_config(self, contents: str) -> Path:
        temporary = tempfile.NamedTemporaryFile(
            mode="w", suffix=".conf", encoding="utf-8", delete=False
        )
        self.addCleanup(lambda: Path(temporary.name).unlink(missing_ok=True))
        with temporary:
            temporary.write(contents)
        return Path(temporary.name)

    def test_help(self) -> None:
        result = self.run_cli("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Usage: ap_mesh", result.stdout)

    def test_version(self) -> None:
        result = self.run_cli("--version")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "ap_mesh 1.0.0")

    def test_invalid_usage(self) -> None:
        result = self.run_cli()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Usage: ap_mesh", result.stderr)

    def test_missing_configuration(self) -> None:
        result = self.run_cli("--config", "does-not-exist.conf")
        self.assertEqual(result.returncode, 3)
        self.assertIn("Configuration error:", result.stderr)

    def test_malformed_line(self) -> None:
        config = self.write_config("INPUT_MODEL=input_models/eistute2.fixed.bp\nbad line\n")
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("KEY=VALUE", result.stderr)

    def test_duplicate_key(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/eistute2.fixed.bp\n"
            "OUTPUT_PREFIX=results/test/first\n"
            "OUTPUT_PREFIX=results/test/second\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("duplicates key", result.stderr)

    def test_unknown_key(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/eistute2.fixed.bp\n"
            "OUTPUT_PREFIX=results/test/output\n"
            "TYPO_OPTION=1\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("unsupported key", result.stderr)

    def test_invalid_number(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/eistute2.fixed.bp\n"
            "OUTPUT_PREFIX=results/test/output\n"
            "NUM_THREADS=not-a-number\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("NUM_THREADS must be an integer", result.stderr)

    def test_non_finite_number(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/eistute2.fixed.bp\n"
            "OUTPUT_PREFIX=results/test/output\n"
            "ADAPTIVE_INTENSITY=nan\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("finite number", result.stderr)

    def test_out_of_range_number(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/eistute2.fixed.bp\n"
            "OUTPUT_PREFIX=results/test/output\n"
            "ADAPTIVE_INTENSITY=2\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("ADAPTIVE_INTENSITY must be in", result.stderr)

    def test_missing_model(self) -> None:
        config = self.write_config(
            "INPUT_MODEL=input_models/missing.bp\n"
            "OUTPUT_PREFIX=results/test/output\n"
        )
        result = self.run_cli("--config", str(config))
        self.assertEqual(result.returncode, 3)
        self.assertIn("INPUT_MODEL does not exist", result.stderr)


if __name__ == "__main__":
    unittest.main()
