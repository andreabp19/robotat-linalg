
# Report of results for this test/challenge

## Brief summary about Robotat Linalg

Robotat Linalg is an embedded C library stack that provides essential routines for linear algebra, numerical optimization, control and robotics, targeting single-precision devices such as Arduino, ESP32 and STM32. It's divided in five library files: 1) ```matf32```, for linear algebra, 2) ```linsolve``` for the linear solver, 3) ```quadprog``` for the quadratic solver, 4) ```robotat-control``` for control routines, and 5) ```robotat-robotics``` for robotics routines.

## Results of this challenge

#### Task 1.1 Build system

**Approach and solution:**
- I used Makefiles as I'm a bit more familiar with those, compared to CMake builds. 
- Implemented all build combinations so everything compiles with ```make all```.
- Saved the error and warnings to ```.log``` files for each compiler and optimization level.

**Results:** All the libraries and test files compiled correctly for all compilers and optimization levels in Linux and for the Cortex-M4. The flags ```-Wall``` and ```-Wextra``` showed previously undetected error and warnings. However, all were related to unused variables, nonvoid functions without return type and implicit declarations, and so I fixed them with ease, with only some exceptions that would require more time and attention beyond the available time for this challenge:
- Undefined behavior in the variable arguments of a PID routine: routine written by another contributor, so it's best to ask before changing it.
- Unused ```angle_units``` parameter in two robotics routines: not-yet implemented feature to check angle units (but will be used).
- Unused parameter ```sys_input``` in ```pendulum_outputs``` routine: this argument might be needed depending on the specific nonlinear system, so in this case it is not used, but the routines expect it in case it is relevant.

#### Task 1.2 Self-checking tests

**Approach and solution:**
- Removed the timing tests from the test files, keeping only numerical tests. 
- Separated ```linsolve``` tests in two files: ```linsolve_test_residual_error.c``` for residuals check, ```linsolve_test_forward_error.c``` for the usual numerical error comparison against MATLAB (this was to have more robust evaluations).
- Total number of tests and failed tests is printed. Main returns 1 if a test failed. 
- Error metrics used a tolerance of 1E-05, so any error below that threshold would be considered a ```PASS```. This is according to the limitations of accuracy in ```float32```. 
- Added command ```test``` to the Makefile to execute all ```matf32``` and ```linsolve``` test files.

**Results:** Approximately 96% of test cases passed successfully. The tests that failed were due to slight numerical error above the threshold (error of 1E-04 instead of below 1E-05).

```
make test
== matf32_test
matf32: 190/199 passed
== linsolve_test_forward_error
matf32: 58/60 passed
== linsolve_test_residual_error
matf32: 60/60 passed
SOME TESTS FAILED
make: *** [Makefile:50: test] Error 1
```

#### Task 1.3 Look inside the binary

**Aproach and solution:** Used the terminal and some commands of the arm compiler to check file size and memory usage. 

**Results:**

**Results 1. How much RAM and FLASH does each module use and what uses the most RAM?**

I used the following command for this task: ```arm-none-eabi-size -t build/cortex-m4-Os/*.o```. The module that used the most RAM was ```quadprog```, with 5,712 bytes (5.58 Kb aprox.).

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

**Results 2. Which are the 10 largest functions?**

The command I used and the results (formatted for readability) are shown below. Results match with some of the heaviest routines that I had identified during previous tests (MPC, quadprog LDLT, QR, etc.), though I was surprised that ```ctr_linloc``` is amongst the heaviest routines, but it makes sense as it uses several stock matrices for operations.

```
arm-none-eabi-nm -S -A -t d --defined-only build/cortex-m4-Os/*.o | grep -E ' [Tt] ' | sort -k2,2 -n -r | head -10
```

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

**Result 3. How much stack do the five most stack-hungry functions need? Are there functions whose stack use the compiler can't tell you? Why not?**

The commands I used for this was: ```make static_arm_library OPT=Os CORTEXM4_EXTRAFLAGS=-fstack-usage```, which created ```.su``` files for each ```robotat-linalg``` module, showing the stack consumption of each routine and the memory type at compile-time (static or dynamic). The compiler cannot fully compute the memory usage of some routines (shown in a table below), as they use dynamic memory at runtime due to using float arrays with size depending on variable matrices. This was unexpected and I will change it after the challenge to use only static memory.

```
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

**Results 4. Which global symbols does the library export? (`nm` lists them.) Which ones should users not see, and what did you do about them?**

All ```robotat-linalg``` modules export global functions (indicated by ```T``` when checking in the terminal). All modules use global variables to execute the operations, mainly stock matrices and vectors, but all of those are hidden from exporting by making them static to prevent the user from seeing them. The command used to print on the terminal the exported symbols was:

```
arm-none-eabi-nm -g --defined-only build/cortex-m4-Os/librobotat.a
```

**Results 5. Comparing executable test files using the static and shared libraries.**

The Makefile was modified to compile the shared library and link the test files to them. Afterwards, the following commands were used to check the size of the files for GCC -O2:

```
ls -lh build/gcc-O2/test/ build/gcc-O2/test-pic/
ls -lh build/gcc-O2-pic/librobotat.so build/gcc-O2/librobotat.a
```

The memory comparison is shown below. The shared library, and test files linked to it, were considerably smaller. The output of ```ldd``` shows how the shared library is retrieved and linked to the test files at runtime, and if the ```.so``` file is deleted, ```ldd``` outputs: ```build/gcc-O2-pic/librobotat.so => not found```. I would use shared libraries when saving memory is critical, and static libraries when robustness is more important.

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

#### Task 2.1 Measurement program

A new PlatformIO project was created for this task, implementing there the measurements with the Cortex-M4's Cycle Counter and the test cases for the PID, Kalman filter and MPC. A summary of the measured time is shown below. The maximum times, in seconds were: 9.017E-06 for the PID, 2.716E-05 for the Kalman filter, and 6.294E-02 for the MPC. This is similar in magnitude, though slightly faster, than the previously measured values; the most notable difference being the PID measured with the Cycle Counter, which was one order of magnitude faster than the previous value. This indicates that the measures made sense despite the added time cost of using ```micros()```, but I would prefer to use the Cycle Counter in the future to obtain more precise measurements for all routines. 

```
routine,minimum(s),median(s),maximum(s),clock_frequency(Hz),compiler
PID,0.000007467,0.000008933,0.000009017,180000000,12.3.1 20230626
KF,0.000000000,0.000022889,0.000027161,180000000,12.3.1 20230626
MPC,0.005269622,0.061413455,0.062941465,180000000,12.3.1 20230626
```
*For all these measurements, -Os optimization level was used.

```
  PID      Kalman filter      MPC
1.30E-05 	  3.30E-05 	      6.37E-02
```
*Note: The website has a typo in the time of the PID for the STM32, and says 1.30E-02 when it's 1.30E-05 as stated in the paper (the value in the paper has been used for this report, and will be fixed in the website soon).