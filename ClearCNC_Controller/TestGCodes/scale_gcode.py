#!/usr/bin/env python3
"""
Scale G-code geometry words by a factor.

Default behavior scales: X, Y, I, J, R
(typical XY linear + arc center/radius terms).

Examples:
  python scale_gcode.py Skunk.nc Skunk_x2.nc
  python scale_gcode.py Skunk.nc Skunk_x2.nc --factor 2
  python scale_gcode.py Skunk.nc Skunk_x2.nc --words X,Y
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


WORD_RE = re.compile(
    r"(?P<word>[A-Za-z])(?P<space>\s*)(?P<value>[+-]?(?:\d+\.\d*|\d+|\.\d+))"
)


def split_code_and_comment(line: str) -> tuple[str, str]:
    """
    Split a G-code line into (code, comment) where comment includes:
      - ';' and everything after it
      - parenthesis comments '(...)' (possibly nested) copied verbatim
    We keep parenthesis comments attached to the comment side only if they start
    at top-level after code scan reaches them; inline paren comments are still
    preserved because the scaler skips them by scanning segments.
    """
    # We keep this simple: scale only outside comments by scanning char-by-char.
    # Return full line in code segment; comment segment unused by caller.
    return line, ""


def scale_code_segment(code: str, factor: float, words_to_scale: set[str]) -> str:
    out: list[str] = []
    i = 0
    n = len(code)
    paren_depth = 0

    while i < n:
        ch = code[i]

        # ';' comment starts: copy rest unchanged
        if paren_depth == 0 and ch == ";":
            out.append(code[i:])
            break

        if ch == "(":
            paren_depth += 1
            out.append(ch)
            i += 1
            continue
        if ch == ")":
            if paren_depth > 0:
                paren_depth -= 1
            out.append(ch)
            i += 1
            continue

        if paren_depth == 0:
            m = WORD_RE.match(code, i)
            if m:
                word = m.group("word")
                space = m.group("space")
                value_text = m.group("value")
                upper_word = word.upper()

                if upper_word in words_to_scale:
                    try:
                        scaled = float(value_text) * factor
                    except ValueError:
                        out.append(m.group(0))
                    else:
                        # Keep compact, deterministic numeric formatting.
                        scaled_text = f"{scaled:.6f}".rstrip("0").rstrip(".")
                        if scaled_text in ("-0", "+0", ""):
                            scaled_text = "0"
                        out.append(f"{word}{space}{scaled_text}")
                else:
                    out.append(m.group(0))

                i = m.end()
                continue

        out.append(ch)
        i += 1

    return "".join(out)


def scale_gcode_text(text: str, factor: float, words_to_scale: set[str]) -> str:
    lines = text.splitlines(keepends=True)
    scaled_lines: list[str] = []
    for line in lines:
        code, _comment = split_code_and_comment(line)
        scaled_lines.append(scale_code_segment(code, factor, words_to_scale))
    return "".join(scaled_lines)


def parse_words_list(words_csv: str) -> set[str]:
    words = {w.strip().upper() for w in words_csv.split(",") if w.strip()}
    if not words:
        raise ValueError("No words provided to --words.")
    for w in words:
        if len(w) != 1 or not w.isalpha():
            raise ValueError(f"Invalid word '{w}'. Use single letters like X,Y,I,J,R.")
    return words


def main() -> int:
    parser = argparse.ArgumentParser(description="Scale G-code geometry by factor.")
    parser.add_argument("input", type=Path, help="Input G-code file path")
    parser.add_argument("output", type=Path, help="Output G-code file path")
    parser.add_argument(
        "--factor",
        type=float,
        default=2.0,
        help="Scale factor (default: 2.0)",
    )
    parser.add_argument(
        "--words",
        default="X,Y,I,J,R",
        help="Comma-separated words to scale (default: X,Y,I,J,R)",
    )

    args = parser.parse_args()
    if args.factor == 0:
        raise SystemExit("Scale factor cannot be 0.")

    words_to_scale = parse_words_list(args.words)

    src = args.input.read_text(encoding="utf-8", errors="ignore")
    dst = scale_gcode_text(src, args.factor, words_to_scale)
    args.output.write_text(dst, encoding="utf-8", newline="")

    print(
        f"Scaled '{args.input}' -> '{args.output}' by factor {args.factor} "
        f"for words: {','.join(sorted(words_to_scale))}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

