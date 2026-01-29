; Simple G-code test - small square
; Units: mm (G21). Use G20 for inches.
; Run from 0,0 or home first if you use G28.

G20
G90
G0 X0 Y0
G1 X10 Y0 F100
G1 X10 Y10 F100
G1 X0 Y10 F100
G1 X0 Y0 F100
M30
