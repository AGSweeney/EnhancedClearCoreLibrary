"""Regenerate resources/icons/clearcnc_app_icon.ico from the SVG (run when art changes)."""
from __future__ import annotations

import sys
from io import BytesIO
from pathlib import Path

import cairosvg
from PIL import Image


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    svg = root / "resources" / "icons" / "clearcnc_app_icon.svg"
    ico = root / "resources" / "icons" / "clearcnc_app_icon.ico"
    if not svg.is_file():
        print(f"Missing {svg}", file=sys.stderr)
        return 1

    png = cairosvg.svg2png(url=str(svg), output_width=256, output_height=256)
    im = Image.open(BytesIO(png)).convert("RGBA")
    sizes = [(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
    im.save(ico, format="ICO", sizes=sizes)
    print(f"Wrote {ico}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
