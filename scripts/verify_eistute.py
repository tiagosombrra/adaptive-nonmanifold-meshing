#!/usr/bin/env python3
import sys
from pathlib import Path
from verify_model import verify_result

root = Path(sys.argv[1] if len(sys.argv) > 1 else "results/eistute")
verify_result(
    root,
    "Eistute",
    Path("configs/eistute/article.conf"),
    Path("reference/eistute_stage4.json"),
    require_configured_final_stage=True,
)
print("Published Eistute target triangle count verified.")
