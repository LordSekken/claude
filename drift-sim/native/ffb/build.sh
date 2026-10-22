#!/usr/bin/env bash
# Cross-compiles ffb_helper.exe for Windows (needs mingw-w64 + SDL3 mingw dev package).
#   SDL3_DIR=/path/to/SDL3-3.x.y/x86_64-w64-mingw32 ./build.sh <out_dir>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="${1:-$here/../../build/windows}"
mkdir -p "$out"
x86_64-w64-mingw32-gcc -O2 -Wall -o "$out/ffb_helper.exe" "$here/ffb_helper.c" \
  -I"$SDL3_DIR/include" -L"$SDL3_DIR/lib" -lSDL3 -lws2_32 -ldinput8 -ldxguid -luser32 -lwinmm -mconsole
cp "$SDL3_DIR/bin/SDL3.dll" "$out/"
echo "built $out/ffb_helper.exe"
