# Host (PC) tests for libClearCore code that has no hardware dependencies.
# Needs g++ (Linux, WSL, macOS or MSYS2).
#
#   make            build and run
#   make clean

CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -Wextra
INC       = -Istub -I../../inc
SRC       = ../../src

TESTS = arc_interpolator_test

all: run

arc_interpolator_test: arc_interpolator_test.cpp $(SRC)/ArcInterpolator.cpp $(SRC)/TrigLUT.cpp \
                       ../../inc/ArcInterpolator.h
	$(CXX) $(CXXFLAGS) $(INC) -I$(SRC) -o $@ arc_interpolator_test.cpp $(SRC)/ArcInterpolator.cpp $(SRC)/TrigLUT.cpp -lm

run: $(TESTS)
	./arc_interpolator_test

clean:
	rm -f $(TESTS)

.PHONY: all run clean
