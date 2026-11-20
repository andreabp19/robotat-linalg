
# Report of results for this test/challenge

## Brief summary about Robotat Linalg

Robotat Linalg is an embedded C library stack that provides essential routines for linear algebra, numerical optimization, control and robotics, targeting single-precision devices such as Arduino, ESP32 and STM32. It's divided in five library files: 1) ```matf32```, for linear algebra, 2) ```linsolve``` for the linear solver, 3) ```quadprog``` for the quadratic solver, 4) ```robotat-control``` for control routines, and 5) ```robotat-robotics``` for robotics routines.

## Results of this challenge

## Part 1 Build and test

### Task 1.1 Build system

**Objective:** Write a Makefile for the ```robotat-linalg``` libraries using ```gcc``` and ```clang``` on a Linux machine, and Cortex-M4, across four optimization levels: ```-O0```, ```-Os```, ```-O2```  and ```-O3```.

**Approach and solution:**
- I used Makefiles as I'm a bit more familiar with those, compared to CMake builds.
- Implemented all build combinations so everything compiles with ```make all```.
- Saved the error and warnings to ```.log``` files for each compiler and optimization level.

**Results:**

*Compilation and build of libraries and test files*

- All the libraries and test files compiled correctly for all compilers and optimization levels in Linux and for the Cortex-M4.

*Error warnings*

The flags ```-Wall``` and ```-Wextra``` showed previously undetected error and warnings. However, all were related to unused variables, nonvoid functions without return type and implicit declarations, and so I fixed them with ease, with only some exceptions that would require more time and attention beyond the available time for this challenge:
- Undefined behavior in the variable arguments of a PID routine: routine written by another contributor, so it's best to ask before changing it.
- Unused ```angle_units``` parameter in two robotics routines: not-yet implemented feature to check angle units (but will be used).
- Unused parameter ```sys_input``` in ```pendulum_outputs``` routine: this argument might be needed depending on the specific nonlinear system, so in this case it is not used, but the routines expect it in case it is relevant.

### Task 1.2 Self-checking tests

**Objective:** Modify ```matf32_test.c``` and ```linsolve_test.c``` test files to self-check their results.

**Approach and solution:**
- Removed the timing tests from the test files, keeping only numerical tests.
- Separated ```linsolve``` tests in two files: ```linsolve_test_residual_error.c``` for residuals check, ```linsolve_test_forward_error.c``` for the usual numerical error comparison against MATLAB (this was to have more robust evaluations). 
- Total number of tests and failed tests is printed. Main returns 1 if a test failed.
- Error metrics used a tolerance of 1E-05, so any error below that threshold would be considered a ```PASS```. This is according to the limitations of accuracy in ```float32```.
- Added command ```test``` to the Makefile to execute all ```matf32``` and ```linsolve``` test files.

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

### Task 1.3 Look inside the binary

**Objective:** Evaluate memory usage of some of the built libraries for the Cortex-M4 and Linux.

**Aproach and solution:** Used the terminal and some commands of the arm compiler to check file size and memory usage. 

**Results:**

*Results 1.* How much RAM and FLASH does each module use and what uses the most RAM?

Command used: ```arm-none-eabi-size -t build/cortex-m4-Os/*.o```. The module that used the most RAM was ```quadprog```, with 5,712 bytes (5.58 Kb aprox.).

```
arm-none-eabi-size -t build/cortex-m4-Os/*.o
   text    data     bss     dec     hex filename
   1637       0    3304    4941    134d build/cortex-m4-Os/linsolve.o
   6841       0     884    7725    1e2d build/cortex-m4-Os/matf32.o
   6215       0    5712   11927    2e97 build/cortex-m4-Os/quadprog.o
   7103       0    2448    9551    254f build/cortex-m4-Os/robotat_control.o
   5922       0       0    5922    1722 build/cortex-m4-Os/robotat_robotics.o
  27718       0   12348   40066    9c82 (TOTALS)
```

*Results 2.* Which are the 10 largest functions?

The following command was used:

```
arm-none-eabi-nm -S -A -t d --defined-only build/cortex-m4-Os/*.o | grep -E ' [Tt] ' | sort -k2,2 -n -r | head -10
```

Memory use of the 10 largest functions (formatted to Kb):

```
Routine                     RAM (Kb)
quadprog_sqp                3.02
quadprog_qp_ldlt            1.73
ctr_linloc                  0.74
quadprog_qp_nullspace       0.65
matf32_qr                   0.65
rob_rot2quat                0.62
ctr_mpc_set_constraints     0.62
ctr_pid_update              0.61
matf32_one_sided_jacobi     0.53
linsolve_method             0.44
```

*Result 3.* How much stack do the five most stack-hungry functions need? (`-fstack-usage` reports this). Are there functions whose stack use the compiler can't tell you? Why not?

The following command was used:

```
make static_arm_library OPT=Os CORTEXM4_EXTRAFLAGS=-fstack-usage
```

which created ```.su``` files for each ```robotat-linalg``` module, showing the stack consumption of each routine and the type of memory they have at compile-time (static or dynamic).

The compiler cannot fully compute the memory usage of the following routines, as they use float arrays with size depending on variable matrices (hence, it's flaggged as dynamic memory):

```
grep -h dynamic build/cortex-m4-Os/*.su
src/linsolve.c:183:1:linsolve_method    48      dynamic
src/matf32.c:1249:1:matf32_qr   128     dynamic
src/quadprog.c:542:1:quadprog_sqp       1736    dynamic
```

The five most stack-hungry functions were checked and printed out as follows:

```
cat build/cortex-m4-Os/*.su | sort -t \t -k2 -n -r | head -5
src/matf32.c:967:1:matf32_pinv  2504    static
src/quadprog.c:542:1:quadprog_sqp       1736    dynamic
src/robotat_robotics.c:695:1:rob_eul2rot.part.0 1664    static
src/robotat_robotics.c:604:1:rob_rpy2rot.part.0 1664    static
src/robotat_robotics.c:271:1:rob_inv_transform  1264    static
```

*Results 4.* Which global symbols does the library export? (`nm` lists them.) Which ones should users not see, and what did you do about them?

All ```robotat-linalg``` modules export global functions (indicated by ```T``` when checking in the terminal). All modules use global variables to execute the operations, mainly stock matrices and vectors, but all of those are hidden from exporting by making them static. 

The command used to print on the terminal the exported symbols was:

```
arm-none-eabi-nm -g --defined-only build/cortex-m4-Os/librobotat.a
```

*Results 5.* Comparing executable test files using the static library ```.a``` and the shared library ```.so```

The Makefile was modified to compile the shared library and link the test files to them. Afterwards, the following commands were used to check the size of the files for GCC -O2:

```
ls -lh build/gcc-O2/test/ build/gcc-O2/test-pic/
ls -lh build/gcc-O2-pic/librobotat.so build/gcc-O2/librobotat.a
```

As shown in the summary below, the shared library was considerably smaller in size, and as consequence, also the test files linked to it. Aside from that, the output of ```ldd``` shows how the shared library is retrieved and linked to the test files at runtime, and if the ```.so``` file is deleted, ```ldd``` outputs: ```build/gcc-O2-pic/librobotat.so => not found```.

```
Name                              Size (Kb) 
build/gcc-O2/librobotat.a           145 
build/gcc-O2-pic/librobotat.so      90 

Name                          Size Kb (.a library)     Size Kb (.so library)
control_kalman_filter_test            78                        19
control_lti_sys_test                  99                        40
control_mpc_test                      78                        19
control_nonlin_sys_test               79                        20
control_pid_test                      77                        18
linsolve_test_forward_error           72                        40
linsolve_test_residual_error          72                        40
matf32_test                           96                        74
quadprog_qp_test                      72                        31
quadprog_sqp_test                     62                        21
robotics_test                         72                        36
```
