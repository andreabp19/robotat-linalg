# Robotat Embedded C Libraries

A collection of high-performance, statically-allocated, C libraries for solving small and dense problems in: matrix algebra, linear systems, convex quadratic programming, control and robotics. Tailored for resource-constrained embedded devices such as: Arduino, ESP32 and STM boards.

This project is aimed at students, educators, and any other user, that wants to develop applications in embedded numerical computation, control or robotics, but lacks the proper tools for this due to the low capacity of embedded devices compared to computers.

# Key features

* Self-Contained libraries: No external libraries required, only C's own libraries are used, such as math.h, stdio.h, and so on.
* Memory-efficient: All data is statically allocated (no dynamic allocation), allowing greater control in memory-constrained devices.
* Device flexibility: Can be implemented in different embedded devices, so there's no limitation to a single platform.

# About this repository

Out of legacy and continuity with the previous version, this repository is called robotat-linalg, but currently contains three different libraries:

* Robotat Linalg: Numerical Computation, subdidivided in three libraries for matrix algebra (matf32), linear systems (linsolve) and convex quadratic programming (quadprog).
* Robotat Control: Linear and nonlinear control algorithms.
* Robotat Robotics: Robotics algorithms.

# History and development

This project is being developed in the robotics laboratory Robotat of Universidad del Valle de Guatemala (UVG). The initial idea was adapted from CControl of Daniel Martensson (https://github.com/DanielMartensson/CControl) in late 2021, leading to the first version of Robotat Linalg in 2022 (https://github.com/danielp96/robotat-linalg). The second, and current, version (this repository) was developed in 2025 by revising and expanding upon the 2022 version, also adding the Robotat Control and Robotat Robotics libraries, the latter of which is based on MATLAB Robotics Toolbox of Peter Corke (https://github.com/petercorke/spatialmath-matlab).

# Supported platforms

As of late 2025, all libraries have been tested in the following devices, with some considerations for each:

* ESP32 (Xtensa 32-Bits): All routines work in this device with no issue.
* Arduino MEGA 2560 (AVR 8-Bits):
    * The routines: quadprog_qp_ldlt, quadprog_qp_nullspace, and the model predictive control cannot be implemented in this device due to memory limitations.
    * The routine matf32_print doesn't work correctly in this device, as printf cannot be used in the Arduino MEGA (and so, calling printf will lead to issues). It's recommended to declare your own printf function, using Serial print functions, in order to use matf32_print (but DO NOT change matf32_print itself, this limitation is due to device characteristics, not a functionality issue in itself).
* STM NUCLEO F446RE (Arm 32-Bits):
    * This device has the same limitation and solution described for the matf32_print routine using the Arduino MEGA 2560.

Additionally, a Tiva C (TM4C123GH6PM) from Texas Instruments was used during the validation of the 2022 version of Robotat Linalg. But, the Tiva C was not included in the testing and validation process of the 2025 version, in order to prioritize devices that can be used within the PlatformIO environment (where this project is intended to be installed). As such, Robotat Control, Robotat Robotics, and some Robotat Linalg routines, are not tested in the Tiva C.

# About each library

## Robotat Linalg

This library is divided in three sublibraries: matf32 for linear algebra, linsolve for the linear solver and quadprog for the convex quadratic solver. To call the full library, import robotat_linalg.h. If you only need matf32 or linsolve, import the respective file matf32.h or linsolve.h. On the other hand, quadprog depends both on matf32 or linsolve, so you can import quadprog.h or just robotat_linalg.h.

The constants.h contains configuration values, such as: matrix and vector size limits, number of iterations for certain operations, tolerance for precision comparison, etc. Currently, as the library is meant for small and dense problems, the size limit is set to vectors with 10 elements, and 10x10 matrices, when working with matf32_t structs. Outside of that scope, you can work with larger arrays as you normally would in C (100, 200, etc.). These dimension limits could be increased if needed, but be aware, that's outside the current validation scope of the library.

Here is a brief description of each library:

* matf32: 
    * Float operations (mean, set, randn, zeros, ones, etc.), which are used to build the main matrix operations. These are used to build the operations for the matf32_t matrix struct.
    * matf32_t routines: addition, substraction, multiplications (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, pseudoinverse, power, factorizations (LU, QR, Cholesky, and SVD with one-sided Jacobi rotations).
    * Routines to identify special matrix types: upper and lower triangular, square, symmetric, symmetric positive definite, and upper and lower Hessian.
    * Auxiliary functions: print matrices, get/set specific values, set identity matrices, set matrices to 1s or 0s, reshape, copy (entire matrices or submatrices), etc.

* linsolve: linear solver for Ax = b systems, implementing the following methods: Forward Substitution, Backward Substitution, Cholesky, LU, QR, SVD.

* quadprog: quadratic solver for equality and inequality constrained quadratic programs (QPs), implementing the following methods: 
    * For QPs with equality restrictions:
        * Direct linsolve methods for solving the associated Karush-Kuhn-Tucker (KKT) system: LU, QR, SVD
        * KKT-specific methods for solving these systems: LDL' factorization and Nullspace method.
    * For QPs with inequality restrictions: an active-set with binding-direction is implemented, solving the KKT systems directly with linsolve methods.

## Robotat Control

This library contains algorithms for the following:

* PID Controllers: update output, discretization methods (Pure Discrete, Forward Euler, Backward Euler and Tustin).
* State space representations for LTI and non-linear systems:
    * Continuous-to-discrete conversion methods (Forward Euler, Backward Euler and Tustin).
    * Nonlinear systems: linealization, state simulation with Forward Euler and Runge-Kutta4.
* Kalman filter prediction and correction functions.
* A shooting-based model predictive control (MPC), using the active-set method included in quadprog.

To use this library, import the robotat_control.h file. Currently, most functions depend only on matf32 from Robotat Linalg, with only the MPC functions depending on quadprog.

## Robotat Robotics

This library is based on the MATLAB Robotics Toolbox by Peter Corke, adapting several functions for: 

* Reference Frames: applying homogeneous transformation matrices (and their inverse), and setting coordinates and rotation matrices in the homogenous transformation matrices.
* Generate rotation matrices from a single angle, Euler and Cardan angles, and quaternions.
* Quaternion operations: addition, substraction, multiplication, conjugate, norm, and unit quaternions.
* Conversion functions to change between homogenous transformation matrices, rotation matrices, Euler and Cardan angles, and quaternions.

To use this function, import the robotat_robotics.h file. Currently, this library depends only on matf32.h from Robotat Linalg.