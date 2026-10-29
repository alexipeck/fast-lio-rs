#!/usr/bin/env bash
# Rebuilds the fixtures image and regenerates the golden files that the Rust
# tests compare against. Run from anywhere; requires Docker.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker build -q -t fast-lio-rs-fixtures "$here" >/dev/null
mkdir -p "$here/golden"
for m in m3 m4 m5; do
  docker run --rm fast-lio-rs-fixtures "/build/gen_$m" > "$here/golden/$m.json"
  echo "wrote $here/golden/$m.json"
done
