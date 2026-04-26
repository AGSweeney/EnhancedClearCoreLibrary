( Native G2/G3 arcs — CONFIG SINGLE=0 coordinated XY )
( Host should send the next G3 after ok without waiting for motion to finish, or corners slow to a stop. )
( Same geometry: center 2.5,2.5 in, r=1.0 in, all coords positive )
( Alternative: XY_Circle_Arc_Positive_TwoHalfs.gcode — full circle in two 180° arcs. )
G20
G90
( Lead-in to start at 3 o'clock )
G0 X3.5000 Y2.5000
( Four CCW quadrants: G3, center offset I J from arc start )
G3 X2.5000 Y3.5000 I-1.0000 J0 F300
G3 X1.5000 Y2.5000 I0 J-1.0000 F300
G3 X2.5000 Y1.5000 I1.0000 J0 F300
G3 X3.5000 Y2.5000 I0 J1.0000 F300
G1 X4 Y0 F100
G1 X0 
M30
