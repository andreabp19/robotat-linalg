# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices.

# About this Repository

Here you will find all of the robotat-linalg numerical computation libraries (matf32, linsolve and quadprog), additional control and robotics libraries implementing the numerical computation libraries and all of the projects and test files in matlab and platformio for the devices used (currently only the ESP32). Also included are the files for plotting results (inside the folder matlab_plotting), along with the respective output files .eps for the plots, so inside that folder you can find the most recent test values used in the .m file and check the plots yourself.

# List of Files

* constants.h: constant values to be used throughout the libraries, for example: maximum matrix and vector sizes.

* matf32.h: linear algebra library
    * auxiliary operations with floats (mean, set, randn, zeros, ones, etc.)
    * matrix definitions: based on a special struct (matf32_t)
    * matrix operations: addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, set, etc.
    * check functions for special matrix types: upper and lower triangular, square, symmetric, symmetric positive definite.

* linsolve.h: linear solver for Ax = b systems, implementing the following matrix decomposition methods:
    * forwards and backward substitution
    * Cholesky
    * LU
    * QR

* quadprog.h: quadratic solver por equality constrained quadratic programs (QPs), implementing the linsolve library

* robotat_control.h: control algorithms
    * PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet)
    * state spaces
    * Kalman filter

* robotat_robotics.h: robotics algorithms based on the Matlab Robotics Toolbox by Peter Corke
    * Reference frame structures and pose calculations
    * quaternion structures and operations (still in development).

* robotat_linalg.h: to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h

# History and Purpose

Robotat-Linalg has been in development at the Universidad del Valle de Guatemala (UVG) as part of undergraduate thesis projects in 2022 (see Previous Versions below) and now in 2025 (this repository). The 2022 version developed the matf32, linsolve and quadprog libraries and validated some functions. The current version (2025) is under development and focuses on testing and validating the functions in the aforementioned files, in addition to those in robotat_control.h, and developing and validating the robotat_robotics.h library.

The purpose of this library is to be implemented in microcontrollers, such as the ESP32, that are used in the robotics laboratory Robotat of the UVG in order to develop embedded applications for courses, projects and more. In addition to that, the library is designed to be adaptable to any other microcontroller, so that applications can be expanded to other devices such as Arduino and more.

# Validation of the Libraries

Validation of matf32, linsolve and quadprog are done by comparing the result and time of execution for the different functions with respect to their matlab equivalents (functions when possible or manually recreating them when necessary). This is currently a work in progress, so documentation and files will be updated accordingly. The same strategy is used for the robotics library, as it is compared against matlab using the Robotics Toolbox by Peter Corke, as the functions implemented are based on the latter. And the same approach will be used for the control library.

# Documentation

To be developed in doxygen or similar.

# Previous Versions

Documentation of the 2022 version of robotat-linalg: https://github.com/danielp96/robotat-linalg
