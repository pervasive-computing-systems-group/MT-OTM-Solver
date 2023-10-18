# MT-SAR-Solver

This code can be used to solve the Minimum-Time problems. See below for how to build and run the code.

## Prerequisites

### Install Gurobi
Go to https://www.gurobi.com/academia/academic-program-and-licenses/, download the Linux version then get an academic license. Unpack the downloaded archive, start following the directions found in gurobi951/linux64/docs/quickstart_linux.pdf in the 'Software Installation Guide' section. This should get you started.

#### To Build Gurobi C++ Library
Follow these directions: https://stackoverflow.com/a/48867074

### Get LKH Solver

Go to http://webhotel4.ruc.dk/~keld/research/LKH-3/, download the latest version,
build the solver, then move the solver executable to 
`~/bin`

Add the following line to your .bashrc file:

`export PATH="/home/$USER/bin:$PATH"`

## To Generate Test Plots
Open test/plotGenerator.py. At the top of the script are various parameters that should be set in order to configure the generated plots. Take note of FILE_PATH as this is needed to run the program in the steps below.

Run test/plotGenerator.py. The generated plots should now show up in the designated file path.

## To Build & Run MT-SAR-Solver
Create a new directory from the root directory to run build commands

`mkdir build`

`cd build`

Create make file

`cmake ..`

Run make file

`make`

Move into test folder

`cd ../test`

Run sar-solver executable

`../build/mutli-otm`

This will print out the expected arguments and their order. There are executables that can run entire suites of test cases. These can be run using:

`./run_experiment_1 (NLP flag)`

The `(NLP flag)`, if set to '1', will run the Non-Linear Programming solution. Be advised, running the test cases with this flag set may take several days to complete.

