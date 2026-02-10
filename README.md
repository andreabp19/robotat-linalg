# Robotat Embedded C Libraries

Embedded C libraries for numerical computation, control and robotics algorithms. This project is developed at Universidad del Valle de Guatemala (UVG) and it's aimed at students, educators and enthusiasts that are interested in developing embedded control and robotics applications.

# Key features

* Self-Contained libraries: Only uses the C Standard Library (no external libraries needed).
* Memory-efficient: Static memory allocation used in all routines (no dynamic memory).
* Device flexibility: Applicable to different embedded devices, such as Arduino, ESP32 and more.

# Supported platforms

As of late 2025, all libraries have been tested in the following devices, with some considerations for each:

* ESP32 (Xtensa 32-Bits): All routines work in this device with no issue.
* Arduino MEGA 2560 (AVR 8-Bits):
    * The routines: quadprog_qp_ldlt, quadprog_qp_nullspace, and the model predictive control cannot be implemented in this device due to memory limitations.
    * The routine matf32_print doesn't work correctly in this device, as printf cannot be used in the Arduino MEGA (and so, calling printf will lead to issues). It's recommended to declare your own print function, using Serial, as this is due to board limitations.
* STM NUCLEO F446RE (Arm 32-Bits):
    * This device has the same limitation and solution described for the matf32_print routine using the Arduino MEGA 2560.

# Libraries in this project

## Robotat Linalg

For numerical computation, divided in the following library files:

* matf32: matrix linear algebra, including basic operations and matrix factorizations.
* linsolve: linear solver methods based on the matrix factorizations in matf32.
* quadprog: convex quadratic solver based on linsolve, includes both direct and active-set methods. 

## Robotat Control

For control algorithms, depends on matf32 and quadprog, and contains routines for:

* PID Controllers, with discretizations: Pure Discrete, Forward Euler, Backward Euler and Tustin.
* State Space representation for LTI systems
* Non Linear Systems: Linealization and simulation with numerical methods.
* Kalman Filter
* MPC: specifically, LTI shooting-based, with and without restrictions.

## Robotat Robotics

For robotics algorithms, based on MATLAB Robotics Toolbox by Peter Corke, adapting routines for:

* Reference frames
* Homogeneous transformations
* Rotation matrices
* Quaternion operations

# History and development

The initial idea was adapted from CControl of Daniel Martensson (https://github.com/DanielMartensson/CControl), leading to the first version of Robotat Linalg in 2022 (https://github.com/danielp96/robotat-linalg).

The current version (this repository) was developed in 2025 by revising and expanding upon the existing code of the 2022 version, completed the Robotat Control library, and finally, added the Robotat Robotics library based on MATLAB Robotics Toolbox of Peter Corke (https://github.com/petercorke/spatialmath-matlab).
