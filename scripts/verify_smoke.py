#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

from verify_model import verify_result


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(
            "Usage: verify_smoke.py <label> <configuration> <results-directory>"
        )
    verify_result(Path(sys.argv[3]), sys.argv[1], Path(sys.argv[2]))


if __name__ == "__main__":
    main()
