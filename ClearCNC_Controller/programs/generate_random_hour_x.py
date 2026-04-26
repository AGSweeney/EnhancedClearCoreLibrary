#!/usr/bin/env python3
"""
Generate a ClearCNC G-code program: ~1 hour of random X moves between 0 and 7",
random feed 10–500 ipm, X axis only (no Y/Z/A words).

Time model: each G1 segment time (minutes) = |Delta X| / F  (F in in/min, inches).

Usage:
  python generate_random_hour_x.py -o random_hour_x.gcode
  python generate_random_hour_x.py -o out.gcode --target-min 58 --seed 42
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("random_hour_x_inch.gcode"),
        help="Output G-code file",
    )
    p.add_argument(
        "--target-min",
        type=float,
        default=60.0,
        help="Target run time in minutes (motion time only, ~1 hour default)",
    )
    p.add_argument(
        "--x-min",
        type=float,
        default=0.0,
        help="X minimum (inches)",
    )
    p.add_argument(
        "--x-max",
        type=float,
        default=10.0,
        help="X maximum (inches)",
    )
    p.add_argument(
        "--f-min",
        type=float,
        default=10.0,
        help="Min feed (ipm)",
    )
    p.add_argument(
        "--f-max",
        type=float,
        default=50.0,
        help="Max feed (ipm)",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed (optional, for reproducible files)",
    )
    args = p.parse_args()

    if args.x_max <= args.x_min:
        raise SystemExit("x_max must be greater than x_min")
    if args.f_max < args.f_min or args.f_min < 0.1:
        raise SystemExit("invalid feed range")

    if args.seed is not None:
        random.seed(args.seed)

    target_min = float(args.target_min)
    x_lo, x_hi = float(args.x_min), float(args.x_max)
    f_lo, f_hi = float(args.f_min), float(args.f_max)

    lines: list[str] = [
        "( X axis only; G20 inches; G90 absolute; ~{:.0f} min motion )".format(target_min),
        "( Random targets in [{:.1f}, {:.1f}] in, F in [{:.0f}, {:.0f}] ipm )".format(
            x_lo, x_hi, f_lo, f_hi
        ),
        "G20",
        "G90",
    ]

    # Start at 0; first move also establishes modal feed per line.
    x = 0.0
    total_min = 0.0
    n_moves = 0

    while total_min < target_min:
        x_new = random.uniform(x_lo, x_hi)
        if abs(x_new - x) < 1.0e-4:
            continue
        f = random.uniform(f_lo, f_hi)
        seg_in = abs(x_new - x)
        total_min += seg_in / f
        n_moves += 1
        lines.append("G1 X{:.4f} F{:.2f}".format(x_new, f))
        x = x_new

    lines.append("M02")
    text = "\n".join(lines) + "\n"
    args.output.write_text(text, encoding="utf-8")
    print("Wrote {}  (~{:.1f} min motion, {} G1 moves)".format(
        args.output, total_min, n_moves))


if __name__ == "__main__":
    main()
