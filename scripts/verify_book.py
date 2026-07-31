#!/usr/bin/env python3
import sys
from pathlib import Path
from verify_model import verify_result

root = Path(sys.argv[1] if len(sys.argv) > 1 else "results/book")
verify_result(root, "Book", Path("configs/book/article.conf"))
print("Book verification is structural; authoritative numerical targets are not bundled yet.")
