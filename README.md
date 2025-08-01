# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices.

# Structure of robotat-linalg

constants.h | constant values to be used throughout the libraries, for example: maximum matrix and vector sizes.

matf32.h | linear algebra and matrix operations, including: addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, etc.

linsolve.h | methods for solving linear systems of the form Ax = b, incluidng: forwards and backward substitution, Cholesky, LU, QR and functions to determine automatically which method to use and solve the system.

quadprog.h | quadratic programming functions, with direct methods for equality-constrained quadratic programs (QPs) and an active-set method for solving inequality-constrained QPs (still not tested).

robotat_control.h | implements control algorithms such as PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet), state space LTI and non-linear systems and Kalman filters.

robotat_robotics.h | implements robotics algorithms based on the Matlab Robotics Toolbox by Peter Corke, including reference frame and pose calculations and quaternions (still in development).

robotat_linalg.h | to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h so that all can be used when importing this only file (currently not importing robotat_control and robotat_robotics as they're both currently under development/testing).

# About this Repository

# Documentation


# Previous Versions

Previous version of the project can be found here: https://github.com/danielp96/robotat-linalg?tab=readme-ov-file 
