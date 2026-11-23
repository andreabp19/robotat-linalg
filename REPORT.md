
# Report of results for this test/challenge

Robotat Linalg is an embedded C library with routines targeting single-precision devices such as Arduino, ESP32 and STM32. It's divided in five modules: 1) ```matf32```, for linear algebra, 2) ```linsolve``` for the linear solver, 3) ```quadprog``` for the quadratic solver, 4) ```robotat-control``` for control routines, and 5) ```robotat-robotics``` for robotics routines. This challenge focused on performance tests regarding compilation and speed of certain routines.

## Challenge tasks and results

##### Task 1.1 Build system

**Approach and solution:**
- I used Makefiles as I'm a bit more familiar with those, compared to CMake builds. 
- Implemented all build combinations so everything compiles with ```make all```.
- Saved the error and warnings to ```.log``` files for each compiler and optimization level.

**Results:** All the libraries and test files compiled correctly for all compilers and optimization levels in Linux and for the Cortex-M4. The flags ```-Wall``` and ```-Wextra``` showed previously undetected error and warnings. However, all were related to unused variables, nonvoid functions without return type and implicit declarations, and so I fixed them with ease, with the following exceptions that require more time beyond this challenge:
- Undefined behavior in the variable arguments of a PID routine: routine written by another contributor, so it's best to ask before changing it.
- Unused ```angle_units``` parameter in two robotics routines: not-yet implemented feature to check angle units (but will be used).
- Unused parameter ```sys_input``` in ```pendulum_outputs``` routine: this argument might be needed depending on the specific nonlinear system, so in this case it is not used, but the routines expect it in case it is relevant.

##### Task 1.2 Self-checking tests

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

##### Task 1.3 Look inside the binary

**1. How much memory does each module use? What uses the most RAM?**

I used the command ```arm-none-eabi-size -t``` for this task. The module ```quadprog``` used the most RAM, with 5,712 bytes (5.58 Kb aprox.).

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

**2. Which are the 10 largest functions?**

Used bash commands to read the files and locate the largest functions sorting by memory. Results match with some of the heaviest routines that I had identified previously (MPC, quadprog LDLT, QR, etc.). Though I was surprised that ```ctr_linloc``` is amongst the heaviest routines, but it makes sense as it uses several stock matrices for operations.

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

**3. How much stack do the five most stack-hungry functions need? Are there functions whose stack use the compiler can't tell you? Why not?**

I used ```make static_arm_library OPT=Os CORTEXM4_EXTRAFLAGS=-fstack-usage```, which created ```.su``` files for each ```robotat-linalg``` module, showing the stack consumption of each routine and the memory type at compile-time (static or dynamic). The compiler cannot fully compute the memory usage of the routines shown in the table below, as they use dynamic memory at runtime due to declaring float arrays with their size depending on the dimensions of variable-sized matrices. This was unknown to me, and I will change it after the challenge to use only static memory (didn't change it during the challenge as it might require testing to ensure nothing is broken, especially in the solvers).

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

**4. Which global symbols does the library export? (`nm` lists them.) Which ones should users not see, and what did you do about them?**

All ```robotat-linalg``` modules export global functions (indicated by ```T``` when checking in the terminal). All modules use global variables to execute the operations, mainly stock matrices and vectors, but all of those are hidden from exporting by making them static to prevent the user from seeing them. To print on the terminal the exported symbols I used the command ```arm-none-eabi-nm -g --defined-only```.

**5. Comparing executable test files using the static and shared libraries.**

The Makefile was modified to compile the shared library and link the test files to them. Afterwards, I used the command ```ls -lh``` to check the size of the files using GCC -O2.

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

##### Task 2.1 Measurement program

A new PlatformIO project was created for this task, implementing there the measurements with the Cortex-M4's Cycle Counter and the test cases for the PID, Kalman filter and MPC. A summary of the measured time is shown below. The maximum times, in seconds were: 9.017E-06 for the PID, 2.716E-05 for the Kalman filter, and 6.294E-02 for the MPC. This is similar in magnitude, though slightly faster, than the previously measured values; the most notable difference being the PID measured with the Cycle Counter, which was one order of magnitude faster than the previous value. This indicates that the measures made sense despite the added time cost of using ```micros()```, but I would prefer to use the Cycle Counter in the future to obtain more precise measurements for all routines. 

```
routine,minimum(s),median(s),maximum(s),clock_frequency(Hz),compiler
PID,0.000007467,0.000008933,0.000009017,180000000,12.3.1 20230626
KF,0.000000000,0.000022889,0.000027161,180000000,12.3.1 20230626
MPC,0.005269622,0.061413455,0.062941465,180000000,12.3.1 20230626
```
*For all these measurements, -Os optimization level was used.

```
  PID      Kalman filter          MPC
1.30E-05 	  3.30E-05 	      6.37E-02
```
*Note: The website has a typo in the time of the PID for the STM32, and says 1.30E-02 when it's 1.30E-05 as stated in the paper (I will fix it after submitting the challenge).

##### 2.2 Stating point, measure matf32_mul and a solver

I used the cycle counter, as in the previous task, to measure the execution time of ```matf32_mul``` for 3x3, 6x6 and 10x10, matrices; and also to measure a Cholesky solve (instructions didn't specify a size for this one, so I measured it with 10x10 matrices). To check the memory use of each routine, I used the ```arm-none-eabi-nm -S --size-sort -t d``` on the respective files of each optimization level. The routine ```matf32_mul``` required the least memory in -O2, while for ```linsolve_method```, which was the routine used to execute the Cholesky solve, the least memory was used in -O3. In this case, if memory was critical, I would choose to use -O2 for an application, as the linear solver requires several multiplications, and so it would make sense to prioritize the memory usage of ```matf32_mul```.

*All RAM results are presented in bytes.
```
Routine           RAM (-Os)    RAM (-O2)   RAM (-O3)
matf32_mul           262         192         232                  
linsolve_method      620         412         400
```

*Maximum measured execution time, all values reported in cycles.
```
Routine          Dimension     Time (-Os)   Time (-O2)    Time (-O3)
matf32_mul         3 x 3          4,191          733           702
matf32_mul         6 x 6         15,985        3,359         3,287
matf32_mul         10x10         67,692       13,330        13,282
linsolve_method    10x10         67,692       13,330        13,282
```

##### 2.3 Make matf32_mul faster by changing the source code

All the following ```matf32_mul``` measurements were done with 10x10 matrices, time is reported in cycles, and RAM in bytes. The changes implemented are described in the table as "versions" due to table space constraints, but correspond to:
- version 2: original algorithm but saving the matrix dimensions before the loops, instead of reading them each loop.
- version 3: replacing with a Saxpi multiplication algorithm (algorithm 1.1.7, Golub).
- version 4: adding a local accumulator inside the loops for computing the multiplication elements, instead of accumulating on ```data_c``` each loop, which corresponds to reading upon ```p_src->p_data``` each time and increases the time.

```
version      RAM(Os)   RAM(O2)   RAM(O3)   Time(Os)   Time (O2)    Time(O3)
original     262        192       232      67,692     13,330       13,282
version 2    262        192       232      70,405     13,937       13,869
version 3    296        222       232      78,182     15,758       13,864
version 4    268        184       212      65,381     11,948       11,856
```

While only version 4 was faster, I kept the results of the other changes I implemented. I initially thought version 2 would be faster as it would reduce the number of reads of each 3 matrix-pointers inside the loops, but it turned out to be slower. Then, I tried finding a faster multiplication algorithm, but was slower as well. Version 4 follows a similar idea to version 2, but applied only to ```data_c``` that accumulates partial multiplication results, which reduces the number of reads from the matrix-pointer, and thus is faster.

##### Task 2.4 Read the Assembly of the Cortex-M4

The faster ```matf32_mul``` uses an accumulator to sum partial products of each element of the resultant matrix A*B: ```vfma.f32	s15, s13, s14```, and after the loop only makes one assignment to save the result on ```data_c``` before incrementing the direction of ```data_c``` for the next loop: ```vstmia	r6!, {s15}```. In contrast, the original version of the routine does something similar, but also writes each loop to the register pointed at through ```data_c```: ```vstr	s15, [r4, #-4]```, which increases time spent in the loop.

##### Conclusions and time log

With one more week, I would fix the pending error warnings I pointed out and the routines flagged as using dynamic memory, and would try and reduce solver computing time based on task 2.3. Finally, here's a summary of my time log:

- Task 1.1: 5 hours     - Task 1.2: 4 hours    - Task 1.3: 3.5 hours
- Task 2.1: 4 hours     - Task 2.2: 2 hours    - Task 2.3: 5 hours    - Task 2.4: 6 hours
- Fixing warnings and errors in the libraries: 1 hour
- Writing REPORT.md and AI_USAGE.md: 5 hours
- Total: 35.5 hours