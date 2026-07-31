#!/usr/bin/env python3
import sys
from pathlib import Path
from verify_model import verify_result

root = Path(sys.argv[1] if len(sys.argv) > 1 else "results/decor_shelf")
verify_result(root, "Decor Shelf", Path("configs/decor_shelf/article.conf"))
print("Decor Shelf verification is structural; authoritative numerical targets are not bundled yet.")
