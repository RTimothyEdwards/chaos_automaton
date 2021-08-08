# Chaos Automaton (Caravel User Project)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

| :exclamation: Important Note            |
|-----------------------------------------|

## Please fill in your project documentation in this README.md file 


Refer to [README](docs/source/index.rst) for this sample project documentation. 

The Chaos automaton
----------------------------------------------------
Digital design for Caravel (user_project_wrapper)

This chip is a pure asynchronous cellular automaton.  Each cell has
four inputs from N, S, E, W and generates four outputs to N, S, E, W.
Each cell can be configured for any boolean function of the four
inputs.

Outputs on the periphery (or some selection thereof) are passed to the
chip GPIO.  Inputs may also come from the chip periphery;  choice of
input or output is programmable like the cell boolean function.

All inputs and outputs may be channeled through the logic analyzer to
set or grab the entire state of the system.

The logic analyzer may also be used to program the cell functions.

This can be used in a loop with an evolutionary algorithm to tune the
chip functions to achieve a specific behavior.

Most of the core circuitry is straightforward.  The total number of
cells is parameterized, so that the largest number of cells that will
fit in the caravel user project space can be determined.
