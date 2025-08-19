# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices. The library is designed to be adaptable to any microcontroller, so that applications can be expanded to other devices such as beyond those tested here (currently ESP32).

# About this repository

Here you will find all of the robotat-linalg numerical computation libraries (matf32, linsolve and quadprog), additional control and robotics libraries implementing the numerical computation libraries and all of the projects and test files in matlab and platformio for the devices used (currently only the ESP32).

## Robotat Linalg Files

* constants.h: constant values to be used throughout the libraries, for example: maximum matrix and vector sizes.

* matf32.h: linear algebra library
    * auxiliary operations with floats (mean, set, randn, zeros, ones, etc.)
    * matrix definitions: based on a custom struct (matf32_t)
    * matrix operations: addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, set, etc.
    * check functions for special matrix types: upper and lower triangular, square, symmetric, symmetric positive definite.

* linsolve.h: linear solver for Ax = b systems, implementing the following matrix decomposition methods:
    * Forward Substitution
    * Backward Substitution
    * Cholesky
    * LU
    * QR

* quadprog.h: quadratic solver for equality constrained quadratic programs (QPs), implementing the linsolve library.

* robotat_control.h: control algorithms
    * PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet)
    * State space representations for LTI and non-linear systems
    * Kalman filter

* robotat_robotics.h: robotics algorithms based on the Matlab Robotics Toolbox by Peter Corke
    * Homogeneous transformation matrix operations (generating and setting rotation matrices, setting coordinate vectors, applying homogeneous transformations, applying euler angles)
    * Quaternion structures and operations (addition, substraction, quaternion-scalar and quaternion-quaternion multiplication, conjugate, norm and inverse).
    * Functions for conversions between angles, homogeneous transformations, rotation matrices and quaternions.

* robotat_linalg.h: to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h

## Test Files

Test files are organized first by library and then by device. For example, in the case of matf32:
   * matf32_tests
      * matf32_test_esp32
      * matf32_test_matlab
    
The results exported from each file are saved and then plotted using MATLAB, all of which can be found in the matlab_plotting folder (the script for plotting and the exported .eps files organized in folders, by library).

# Testing and Validation of the Libraries

Validation of matf32, linsolve and quadprog are done by comparing the result and time of execution for the different functions with respect to their matlab equivalents (functions when possible or manually recreating them when necessary). This is currently a work in progress, so documentation and files will be updated accordingly. The same strategy is used for the robotics library, as it is compared against matlab using the Robotics Toolbox by Peter Corke, as the functions implemented are based on the latter. And the same approach will be used for the control library.

# History and Development

Robotat-Linalg has been in development at the Universidad del Valle de Guatemala (UVG) as part of undergraduate thesis projects in 2022 (see Previous Versions below) and now in 2025 (this repository). The 2022 version developed the matf32, linsolve and quadprog libraries and validated some functions. The current stage of development focuses on testing and validating the functions in the aforementioned libraries, in addition to those in robotat_control.h, and developing and validating the robotat_robotics.h library. The purpose of the library is to be used for developing embedded applications for courses and projects in the robotics laboratory Robotat of the UVG.

## Previous Versions

The version of Robotat Linalg developed here started by uploading the code as is from the 2022 version, which is available at: https://github.com/danielp96/robotat-linalg
