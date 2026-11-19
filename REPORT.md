
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

**Objective:** Modify ```matf32_test.c``` and ```linsolve_test.c``` test files to self-check for PASS/FAIL the test cases.

**Approach and solution:** Previously I had both numerical and timing tests in the same file (for all tests), so I removed the timing tests and modified the numerical test cases to report PASS/FAIL, matrix conditioning and the largest absolute error observed between the resulting matrix and the MATLAB reference data. Additionally, the total number of tests and of failed tests are counted and printed out at the end of the program, and the ```main``` returns 1 if at least one test failed. 

In the case of ```linsolve_test.c```, I added an additional error metric, besides element-wise error. For this reason, there's now two linsolve test files: ```linsolve_test_residual_error.c```, using residual-based error to evaluate the stability of the solver methods; and ```linsolve_test_forward_error.c``` for the usual numerical error comparison against MATLAB. This was made to take advantage of the more automated tests and add the residual-based error metric, which is usually recommended in the theory books to evaluate the performance of solvers.

Both error metrics used a tolerance of 1E-05, so any error below that threshold would be considered correct and the test case would pass. As always, this was decided based on the limitations of accuracy in ```float32```, as the fifth decimal is the safest choice to provide as much accuracy while avoiding the last digits that could have error accumulation from garbage values beyond the limit of the single precision.

After changing the code, I modified the Makefile to add the command ```test```, which takes every selected test file (in this case, ```matf32``` and ```linsolve``` test files) and executes them, while printing the results and number of tests that failed the error threshold. 

**Results:**

Below is the output of the files, with the current test data, as printed by the summary created with ```make test``` from the Makefile. Approximately 96% of test cases passed successfully. Those who failed, are shown above in the summary, alongside with the conditioning for the input matrix and the maximum observed error, according to the chosen metrics.

```
make test
== matf32_test
matf32_exp, 3 x 3: FAIL, conditioning A: 23.7020, max. error: 0.000011444
matf32_pinv,basic, 3 x 3: FAIL, conditioning A: 23.7020, max. error: 0.000022411
matf32_pinv,basic, 5 x 5: FAIL, conditioning A: 50.8983, max. error: 0.000029802
matf32_pinv,basic, 6 x 6: FAIL, conditioning A: 30.5445, max. error: 0.000024080
matf32_pinv,basic, 7 x 7: FAIL, conditioning A: 32.6011, max. error: 0.000013709
matf32_exp, 8 x 8: FAIL, conditioning A: 47.8661, max. error: 0.000015259
matf32_pinv,basic, 8 x 8: FAIL, conditioning A: 47.8661, max. error: 0.000011444
matf32_exp, 9 x 9: FAIL, conditioning A: 25.5908, max. error: 0.000015259
matf32_exp, 10 x 10: FAIL, conditioning A: 33.1580, max. error: 0.000015259
matf32: 190/199 passed
== linsolve_test_forward_error
Cholesky, 1 x 1: FAIL, conditioning A: 1.0000, max. error: 0.000976562
SVD, 8 x 8: FAIL, conditioning A: 199.4066, max. error: 0.000028014
matf32: 58/60 passed
== linsolve_test_residual_error
matf32: 60/60 passed
SOME TESTS FAILED
make: *** [Makefile:50: test] Error 1
```