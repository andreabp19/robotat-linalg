
# Report of results for this test/challenge

## Summary

This report summarizes the results obtained during this challenge for the branch ```build-test-perf```.
As required, it's written for an engineer who hasn't read my work before, so it starts with a section about the
context about ```robotat-linalg```, including characteristics, motivation, scope and current state of the project.
Afterwards, the report explains the results for each task of the challenge, and finishes with some conclusions and recommendations.

## Background, motivation and purpose of Robotat Linalg

Robotics environments where accesibility is high-priority usually work with low-to-mid capacity embedded devices, such
as Arduino, ESP32, STM32, Tiva C, and other, development boards. These devices, while capable, have a much more limited
capacity compared to more powerful devices, and lack native support for the numerical routines required for real-time
control and robotics applications.

Robotat Linalg addresses this issue, by providing a numerical infrastructure tailored to the aforementioned devices,
including the essential linear algebra, linear and quadratic solvers, upon which control and robotics routines are built. All of this, written in C, with single-precision arithmetic, static memory, and no external dependencies, to ensure
portability across platforms and a more efficient use of the computational and memory resources of these devices.

This project has been in development at the Robotat laboratory, in Universidad del Valle de Guatemala, since 2021. With
the first public version presented in a paper by late 2026. However, the portability of Robotat Linalg allows it to be adapted to other environments, and as it's open source, it's accessible to anyone interested in embedded control and robotics.

## Robotat Linalg software architecture and routines

Robotat Linalg is a C library stack, currently divided in five libraries across two levels: 1) the Robotat Linalg Core, which corresponds to the numerical infrastructure, encompassing the library files ```matf32```, ```linsolve``` and ```quadprog```; and 2) the application layer, including ```robotat-control``` and ```robotat-robotics``` for control and robotics routines, respectively, which implement the Robotat Linalg Core libraries.

Below is a summary of the routines inside each library file:

- ```matf32```: Contains linear algebra routines, including basic matrix operations (addition, subtraction, multiplications, transpose, inverse, etc.) and matrix factorizations (LU, QR, Cholesky, SVD). Matrix representation
is established with the custom struct ```matf32_t``` upon which all matrix and vector operations are established.
- ```linsolve```: The linear solver, built using ```matf32```. The linear solver uses direct methods such as LU, QR,
Cholesky and SVD.
- ```quadprog```: The quadratic solver, built using both ```matf32``` and ```linsolve```. Similar to ```linsolve```, it
uses mostly direct methods whenever possible, and also includes an active-set method for solving mixed-constraint quadratic problems (QPs).
- ```robotat-control```: Control algorithms, including PID control, LTI systems, nonlinear systems, Kalman filter, 
and an LTI shooting-based MPC.
- ```robotat-robotics```: Robotics algorithms, mainly pose-computation routines, including homogeneous transformations,
rotation matrices, angle conversions and quaternion operations.

## Results of this challenge

This challenge was divided in two parts, with a total of seven tasks. The following sections summarize the result of each
task, with a brief discussion of the results.

## Part 1 Build and test

### Task 1.1 Build system

**Objective:** Write a Makefile or a CMake build for the ```robotat-linalg``` libraries using ```gcc``` and ```clang``` on a Linux machine, and for ```cortex-m4```, across four optimization levels: ```-O0```, ```-Os```, ```-O2```  and ```-O3```, using the indicated configuration and error flags.

**Approach and solution:**
- I used Makefiles as I'm a bit more familiar with those, compared to CMake builds.
- Implemented all the compiler and optimization level compilations such that it compiles for all with a single command ```make all```, to confirm it works for every combination.
- Wrote the error and warning messaged to ```.log``` files for each compiler and optimization level, in order to check with greater ease the differences across each one.

**Results:**

*Compilation and build of libraries and test files*

All the ```robotat-linalg``` libraries and test files compiled correctly for all compilers and optimization levels in the Linux machine. The instructions specified that the test files for ```matf32``` and ```linsolve``` had to be built, but just in case, I added all the relevant test files for the rest of the libraries. Similarly, the libraries built correctly for the ```Cortex M4```.

*Error warnings*

All error and warning messages printed correctly into the log files for each compiler and optimization level combination. Using the flags ```-Wall``` and ```-Wextra```, showed previously undetected error and warning messages. However, as all were related to unused variables, nonvoid functions without return type and implicit declarations, most were fixed with ease. Only some errors were left untouched, for the following reasons:
- Undefined behavior in the variable arguments of a PID routine, which was written by another contributor and I decided the right approach is to ask the person before making any changes to the function.
- Unused ```angle_units``` parameter in two robotics routines, which correspond to a not yet implemented units check to ensure the routine works for both degrees and radians. These parameters do not affect the functionalities of the routines, so for the purpose of this challenge, those are negligible for now and will be implemented after the challenge finishes.
- Unused parameter ```sys_input``` in ```pendulum_outputs``` routine, which is the output behavior of a simple pendulum modeled for a nonlinear system. This is normal behavior, as the nonlinear systems expect the system input for modeling the output, as some systems require it, but in this specific case, it doesn't so it remains unused. However, this has no impact on functionalities of the routines.

### Task 1.2 Self-checking tests



