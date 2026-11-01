# Host tests

Tests for libClearCore code that runs without hardware, built with the PC's
own compiler (Linux, WSL, macOS or MSYS2 `g++`).

```sh
cd libClearCore/tests/host
make            # build and run
./arc_interpolator_test 20000 7    # more arcs, another seed
```

| Test | What it checks |
|------|----------------|
| `arc_interpolator_test` | `ArcInterpolator` driven at 5 kHz like `CoordinatedMotionController`: random 180° arcs (80 steps/mm, radius 20–300 mm, 60–1500 rpm at 800 steps/rev, accel 0.5–6 M mm/min², a third diagonal) plus fixed quarter and three-quarter turns. Every arc must land exactly on its end point, finish only after its last sample, never burst, stay within 2 steps of the circle, sweep the correct half and not run faster than the commanded feed. |

`stub/` holds stand-ins for headers that the code under test includes but
doesn't need off-target. Build output stays in this directory and is
git-ignored.
