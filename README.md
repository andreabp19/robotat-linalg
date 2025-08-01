# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices.

# About this Repository

Description under development.

# Structure of robotat-linalg

constants.h | constant values to be used throughout the libraries, for example: maximum matrix and vector sizes.

matf32.h | linear algebra and matrix operations, including: addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, etc. Currently divided in four different files due to modular development and will be combined into a single matf32.h file soon!

linsolve.h | methods for solving linear systems of the form Ax = b, including: forwards and backward substitution, Cholesky, LU, QR and functions to determine automatically which method to use and solve the system.

quadprog.h | quadratic programming functions, with direct methods for equality-constrained quadratic programs (QPs) and an active-set method for solving inequality-constrained QPs (still not tested).

robotat_control.h | implements control algorithms such as PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet), state space LTI and non-linear systems and Kalman filters.

robotat_robotics.h | implements robotics algorithms based on the Matlab Robotics Toolbox by Peter Corke, including reference frame and pose calculations and quaternions (still in development).

robotat_linalg.h | to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h so that all can be used when importing this only file (currently not importing robotat_control and robotat_robotics as they're both currently under development/testing).

# History and Purpose

Robotat-Linalg has been in development at the Universidad del Valle de Guatemala (UVG) as part of undergraduate thesis projects in 2022 (see Previous Versions below) and now in 2025 (this repository). The 2022 version developed the matf32.h library files, linsolve.h and quadprog.h and validated some functions. The version currently developed and documented in this repository focuses on validating all of the functions in the aformentioned files, validating and adding some algorithms in robotat_control.h and developing and validating the robotat_robotics.h library.

The purpose of this library is to be implemented in microcontrollers, such as the ESP32, that are used in the robotics laboratory Robotat of the UVG, while being flexible enough to be used in any microcontroller so that more embedded applications are developed with control and robotics.

# Validation of the Libraries

Section under development.

# Documentation

Under development.

# Previous Versions

Documentation of the 2022 version of robotat-linalg: https://github.com/danielp96/robotat-linalg?tab=readme-ov-file 
