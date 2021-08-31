#!/bin/sh

# generate sim input
echo "(1/5) generating simulation input data"
cd tests
./sim_input.py sim_points.txt sim_input.h || exit $?
cd ..

# compile simulation
echo "(2/5) compiling the simulation program"
./compile.sh sim || exit $?

# flash target with simulation program
echo "(3/5) flashing the target"
./flash.sh sim || exit $?

# redirect tether output to file
echo "(4/5) running the simulation"
./tether.py --format-csv tests/sim_output.csv || exit $?

# run tests
echo "(5/5) checking the simulation output"
./tests/sim_tests.py tests/sim_output.csv || exit $?
