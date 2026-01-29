; 10" square with 1" radius rounded corners (G20 inches, absolute)
; Outer bounds: X0..10, Y0..10
G20
G90
G0 X1 Y0
G1 F20
G1 X9 Y0
G3 X10 Y1 I0 J1
G1 X10 Y9
G3 X9 Y10 I-1 J0
G1 X1 Y10
G3 X0 Y9 I0 J-1
G1 X0 Y1
G3 X1 Y0 I1 J0
M30
