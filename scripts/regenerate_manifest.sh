#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

rm -f MANIFEST.sha256

while IFS= read -r -d '' file; do
  [[ "${file}" == "MANIFEST.sha256" ]] && continue
  sha256sum -b "./${file}"
done < <(git ls-files -co --exclude-standard -z) > MANIFEST.sha256

sed -i '/^[[:space:]]*$/d' MANIFEST.sha256
sha256sum -c MANIFEST.sha256
