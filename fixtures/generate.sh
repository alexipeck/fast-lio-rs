#!/usr/bin/env bash
# Rebuilds the fixtures image and regenerates the golden files that the Rust
# tests compare against. Run from anywhere; requires Docker.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker build -q -t fast-lio-rs-fixtures "$here" >/dev/null
mkdir -p "$here/golden"
docker run --rm fast-lio-rs-fixtures /build/gen_m3 > "$here/golden/m3.json"
echo "wrote $here/golden/m3.json"
