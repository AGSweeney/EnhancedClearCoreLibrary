# Host (PC) tests for libClearCore code that has no hardware dependencies.
# Needs g++ (Linux, WSL, macOS or MSYS2).
#
#   make            build and run
#   make clean

CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -Wextra
INC       = -Istub -I../../inc
SRC       = ../../src

TESTS = arc_interpolator_test corexy_kinematics_test

all: run

arc_interpolator_test: arc_interpolator_test.cpp $(SRC)/ArcInterpolator.cpp $(SRC)/TrigLUT.cpp \
                       ../../inc/ArcInterpolator.h
	$(CXX) $(CXXFLAGS) $(INC) -I$(SRC) -o $@ arc_interpolator_test.cpp $(SRC)/ArcInterpolator.cpp $(SRC)/TrigLUT.cpp -lm

CXY_SRC = $(SRC)/ArcInterpolator.cpp $(SRC)/LinearInterpolator.cpp $(SRC)/TrigLUT.cpp

corexy_kinematics_test: corexy_kinematics_test.cpp $(CXY_SRC) ../../inc/CoordinatedKinematics.h \
                        ../../inc/ArcInterpolator.h ../../inc/LinearInterpolator.h
	$(CXX) $(CXXFLAGS) $(INC) -I$(SRC) -o $@ corexy_kinematics_test.cpp $(CXY_SRC) -lm

run: $(TESTS)
	./arc_interpolator_test
	./corexy_kinematics_test

clean:
	rm -f $(TESTS)

.PHONY: all run clean
