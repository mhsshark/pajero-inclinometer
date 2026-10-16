#!/usr/bin/env bash
# Render preview PNGs of the gauge on the host. Usage: tools/sim/run.sh [outdir]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$here/../.."
out="${1:-$here/out}"
mkdir -p "$out"
g++ -O2 -std=c++17 -I"$root/src" "$here/sim.cpp" "$root/src/gauge.cpp" "$root/src/canvas.cpp" -o "$out/sim" -lm
"$out/sim" "$out"
python3 - "$out" <<'PY'
import glob, os, sys
from PIL import Image
out = sys.argv[1]
files = sorted(glob.glob(os.path.join(out, "*.ppm")))
imgs = [Image.open(f) for f in files]
for f, im in zip(files, imgs):
    im.save(f[:-4] + ".png"); os.remove(f)
sheet = Image.new("RGB", (480 * 3, 480 * ((len(imgs) + 2) // 3)), "white")
for i, im in enumerate(imgs):
    sheet.paste(im, ((i % 3) * 480, (i // 3) * 480))
sheet.save(os.path.join(out, "sheet.png"))
print("wrote", os.path.join(out, "sheet.png"))
PY
