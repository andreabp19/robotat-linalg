# robotat-linalg
Low-level C library for numerical computation aimed at applications in optimization, control and robotics in embedded devices.

# About this Repository

Here you will find all of the robotat-linalg numerical computation libraries (matf32, linsolve and quadprog), additional control and robotics libraries implementing the numerical computation libraries and all of the projects and test files in matlab and platformio for the devices used (currently only the ESP32). Also included are the files for plotting results (inside the folder matlab_plotting), along with the respective output files .eps for the plots, so inside that folder you can find the most recent test values used in the .m file and check the plots yourself.

# Structure of robotat-linalg

constants.h | constant values to be used throughout the libraries, for example: maximum matrix and vector sizes.

matf32.h | linear algebra and matrix operations, including: addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, resizing, etc. Currently divided in four different files due to modular development and will be combined into a single matf32.h file soon!

linsolve.h | methods for solving linear systems of the form Ax = b, including: forwards and backward substitution, Cholesky, LU, QR and functions to determine automatically which method to use and solve the system.

quadprog.h | quadratic programming functions, with direct methods for equality-constrained quadratic programs (QPs) and an active-set method for solving inequality-constrained QPs (still not tested).

robotat_control.h | implements control algorithms such as PID discretizations (Forward and Backward Euler, Pure Discrete and Tustin, ZOH is not implemented yet), state space LTI and non-linear systems and Kalman filters.

robotat_robotics.h | implements robotics algorithms based on the Matlab Robotics Toolbox by Peter Corke, including reference frame and pose calculations and quaternions (still in development).

robotat_linalg.h | to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h so that all can be used when importing this only file (currently not importing robotat_control and robotat_robotics as they're both currently under development/testing).

# History and Purpose

Robotat-Linalg has been in development at the Universidad del Valle de Guatemala (UVG) as part of undergraduate thesis projects in 2022 (see Previous Versions below) and now in 2025 (this repository). The 2022 version developed the matf32.h library files, linsolve.h and quadprog.h and validated some functions. The current version (2025) is currently under development and focuses on testing and validating the functions in the aforementioned files, in addition to those in robotat_control.h, and developing and validating the robotat_robotics.h library.

The purpose of this library is to be implemented in microcontrollers, such as the ESP32, that are used in the robotics laboratory Robotat of the UVG in order to develop embedded applications for courses, projects and more. In addition to that, the library is designed to be adaptable to any other microcontroller, so that applications can be expanded to other devices such as Arduino and more.

# Validation of the Libraries

Validation of matf32, linsolve and quadprog are done by comparing the result and time of execution for the different functions with respect to their matlab equivalents. This is currently a work in progress, so documentation and files will be updated accordingly. The same strategy is used for the robotics library, as it is compared against matlab using the Robotics Toolbox by Peter Corke, as the functions implemented are based on the latter. And the same approach will be used for the control library.

# Documentation

To be developed in doxygen or similar.

# Previous Versions

Documentation of the 2022 version of robotat-linalg: https://github.com/danielp96/robotat-linalg
