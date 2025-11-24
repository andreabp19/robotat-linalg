# What is Robotat Linalg?

Robotat Linalg is the name of this project, encompassed by different embedded C libraries aimed at numerical optimization, control and robotics algorithms to be used in microcontroller boards, specifically for small and dense problems. This project was developed in Universidad del Valle de Guatemala (UVG), with initial planning in late 2021, (and is in continuous improvement and expansion), tailored to the needs and platforms of this university's robotics environment and laboratory: Robotat. Thus, the name Robotat Linalg means "Robotat Linear Algebra", and any subsequent libraries to be added will keep the Robotat prefix.

The original library (Robotat Linalg), was first developed in 2022, structured as three software libraries: matf32 for matrix linear algebra, linsolve for solving linear systems, and quadprog for quadratic programming. The 2025 version (most recent and current), revised and expanded upon the 2022 version, validating the original routines (while modifying some and adding new ones) and implemented Robotat Linalg to add new libraries: Robotat Control and Robotat Robotics, for control and robotics algorithms, respectively.

Out of legacy and continuation of the 2022 version, this repository and project themselves are called Robotat Linalg. Just know that, when referring to the libraries themselves: Robotat Linalg means numerical computation and optimization libraries, Robotat Control refers to the control library, and Robotat Robotics is the robotics library.

# Key advantages

* No external libraries used. Only C's own libraries, such as math.h, stdio.h, and so on.
* No dynamic memory allocation. All data (scalars, vectors, matrices, etc.) are statically allocated.
* No limitation to a single device. All libraries can be implemented in different microcontrollers.

Which means: no outer dependencies, greater memory control and platform flexibility!

# Tested in different devices

As of late 2025, all libraries have been tested in the following devices: ESP32 (Xtensa 32-Bits), Arduino MEGA 2560 (AVR 8-Bits) and STM NUCLEO F446RE (Arm 32-Bits). All these have different architectures and capacities, showcasing the libraries' platform flexibility. Though, being different devices, there are some considerations for each:

* ESP32: All routines work in this device with no issue.
* Arduino MEGA 2560:
    * The routines: quadprog_qp_ldlt, quadprog_qp_nullspace, and the model predictive control cannot be implemented in this device due to memory limitations.
    * The routine matf32_print doesn't work correctly in this device, as printf cannot be used in the Arduino MEGA (and so, calling printf will lead to issues). It's recommended to declare your own printf function, using Serial print functions, in order to use matf32_print (but DO NOT change matf32_print itself, this limitation is due to device characteristics, not a functionality issue in itself).
* STM NUCLEO F446RE:
    * This device has the same limitation and solution described for the matf32_print routine using the Arduino MEGA 2560.

Additionally, a Tiva C (TM4C123GH6PM) from Texas Instruments was used during the validation of the 2022 version of Robotat Linalg, with some differences in the data printing process compared to the ESP32. In the case of the 2025 version, the Tiva C was not included in the testing and validation process, in order to prioritize devices that can be used within the PlatformIO environment, where this project is intended to be installed. But, Robotat Linalg can indeed be used in a Tiva C.

# Files in this repository

In this repository you will find the most recent version of the Robotat Linalg libraries (matf32, linsolve and quadprog) and the Robotat Control and Robotat Robotics libraries, as well as any other file needed for configuration. Additionally, in the "examples" folder you can find demos with basic examples for use, while the "test" folder, you can find the test files for the routines (grouped by theme and all identified with the corresponding library name).

* robotat_linalg.h: to import matf32.h, linsolve.h and quadprog.h, robotat_control.h and robotat_robotics.h. If you need the full Robotat Linalg library, import this file.

* constants.h: constant values for configuration, for example: maximum matrix and vector sizes, maximum number of iterations for SVD, etc. Currently, and throughout development, the vector size is set to 10 elements, and matrix size to 100 elements (10 x 10 matrices at most), as the project is focused on solving small and dense problems. The maximum size could be increased if needed, but be aware that's outside the current purpose and validation of the project.

* matf32.h: linear algebra library
    * Auxiliary operations with floats (mean, set, randn, zeros, ones, etc.), which are used to build the main matrix operations.
    * Matrix definitions: based on a custom struct (matf32_t)
    * Matrix operations (matf32_ functions): addition, subtraction, multiplication (matrix-matrix, matrix-vector, vector-matrix, matrix-scalar, vector-scalar), inverse, pseudoinverse, power, reshaping, copying, set a value/row, get a value, etc.
    * Check functions for special matrix types: upper and lower triangular, square, symmetric, symmetric positive definite, and upper and lower Hessian.
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
    * A shooting-based model predictive control, with its corresponding data structures, functions and using the active-set method of quadprog.

* robotat_robotics.h: robotics algorithms based on the Robotics Toolbox by Peter Corke for MATLAB
    * Homogeneous transformation matrix operations (generating and setting rotation matrices, setting coordinate vectors, applying homogeneous transformations, applying euler angles)
    * Quaternion structures and operations (addition, substraction, quaternion-scalar and quaternion-quaternion multiplication, conjugate, norm and inverse).
    * Functions for conversions between angles, homogeneous transformations, rotation matrices and quaternions.

# History and Development

Robotat Linalg was developed at Universidad del Valle de Guatemala (UVG) as part of two undergraduate thesis in Mechatronics Engineering: the first version in 2022 (https://github.com/danielp96/robotat-linalg) and the second (and current) version in 2025 (this repository). Regarding other projects that influenced this work, the first ideas of Robotat Linalg were adapted from CControl (https://github.com/DanielMartensson/CControl) in late 2021, while the structure and routines in Robotat Robotics were influenced by the MATLAB Robotics Toolbox of Peter Corke (https://github.com/petercorke/spatialmath-matlab).