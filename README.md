# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices. The library is designed to be adaptable to any microcontroller, so that applications can be expanded to other devices such as beyond those tested here (currently ESP32).

# About this repository

Here you will find all of the robotat-linalg numerical computation libraries (matf32, linsolve and quadprog) and the control and robotics libraries, along with example files for testing the functions and to learn how to use each library.

## Robotat Linalg Files

* constants.h: constant values to be used throughout the libraries, for example: maximum matrix and vector sizes, maximum number of iterations for SVD, etc.

* matf32.h: linear algebra library
    * Auxiliary operations with floats (mean, set, randn, zeros, ones, etc.)
    * Matrix definitions: based on a custom struct (matf32_t)
    * Matrix operations (matf32_ functions): addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, set, etc.
    * Check functions for special matrix types: upper and lower triangular, square, symmetric, symmetric positive definite.
    * Matrix Factorizations: LU, QR, Cholesky and SVD (with One-Sided Jacobi).

* linsolve.h: linear solver for Ax = b systems, implementing the following methods:
    * Forward Substitution
    * Backward Substitution
    * Cholesky
    * LU
    * QR
    * SVD

* quadprog.h: quadratic solver for equality and inequality constrained quadratic programs (QPs), implementing the following methods: 
    * For QPs with equality restrictions:
       * LU
       * QR
       * SVD
       * LDL'
       * Nullspace
    * For QPs with inequality restrictions:
       * Active-Set 

* robotat_control.h: control algorithms
    * PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet)
    * State space representations for LTI and non-linear systems.
       * Continuous to discrete time conversion (ctr_c2d)
       * Nonlinear system state simulation with Forward Euler and Runge-Kutta4.
    * Kalman filter prediction and correction functions.

* robotat_robotics.h: robotics algorithms based on the Robotics Toolbox by Peter Corke for MATLAB
    * Homogeneous transformation matrix operations (generating and setting rotation matrices, setting coordinate vectors, applying homogeneous transformations, applying euler angles)
    * Quaternion structures and operations (addition, substraction, quaternion-scalar and quaternion-quaternion multiplication, conjugate, norm and inverse).
    * Functions for conversions between angles, homogeneous transformations, rotation matrices and quaternions.

* robotat_linalg.h: to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h in order to use all the libraries.

# History and Development

Robotat-Linalg has been in development at the Universidad del Valle de Guatemala (UVG) as part of undergraduate thesis projects in 2022 (see Previous Versions below) and now in 2025 (this repository). The 2022 version developed the matf32, linsolve and quadprog libraries and validated some functions. The current stage of development focuses on testing and validating the functions in the aforementioned libraries, in addition to those in robotat_control.h, and developing and validating the robotat_robotics.h library. The purpose of the library is to be used for developing embedded applications for courses and projects in the robotics laboratory Robotat of the UVG.

## Previous Versions

The version of Robotat Linalg developed here started by uploading the code as is from the 2022 version, which is available at: https://github.com/danielp96/robotat-linalg
