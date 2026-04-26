%
(ClearCNC sample: X-axis moves between 0 and 8 inches)
(Assumes GUI/firmware are configured for inches and Port 0 mapped to X)

G20
G90
G92 X0

G01 X1.000 F50
G01 X3.000 F100
G01 X6.000 F150
G01 X8.000 F200
G01 X4.000 F120
G01 X0.000 F80
G01 X2.000 F60
G01 X7.000 F180
G01 X0.000 F100

M201
%
