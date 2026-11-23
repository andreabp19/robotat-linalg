
# Notes about AI usage for this test/challenge branch

As indicated in the test/challenge instructions, AI tools were allowed. I used Claude, but complemented with googling things myself now and then, and my use of AI included mostly: 1) researching about terms or concepts unknown to me, 2) searching for syntax or examples for unknown commands/routines, especially for bash commands and flags, 3) fixing my own syntax or logic mistakes to save time (AFTER writing the code on my own). For transparency, my AI usage during this test is summarized by task, so the reviewers know how I approached each one.

## Part 1: Build and test

### Task 1.1 Build system

My AI usage in this task consisted mostly of researching the meaning of unknown configurations for compilation and for the syntax that I needed for the Makefile, as my knowledge of Makefiles is very basic and I get confused easily with the syntax. I built my solution gradually, double-checking by testing it and asking the AI if I was writing it correctly at each step. Once I got a handle of the syntax, I progressed more on my own.

### Task 1.2 Self-checking tests

My AI use here consisted mostly of double-checking syntax and logic errors while modifying my code to make it self-checking, because I moved several sections to a function to reuse the code to check the results and removed other lines (which always risks breaking something inadvertently). Additionally, I used it to polish the logic and syntax of the function I wrote for computing residual-based error in the case of ```linsolve_test.c``` for the error of the solver methods.

### Task 1.3 Look inside the build

My use of AI in this task was to search the commands that I needed to read and print the characteristics of the built files on the terminal, as I'm on an arch-based linux machine and don't know from memory the commands. Also, to double-check my implementation of the build for the shared libraries and the test files with the shared libraries, as I made some syntax mistakes in my first try.

### Task 2.1 Measurement program

I used AI here to search up the configuration for the Cycle Counter, write an insertion sort function to organize the time arrays (and obtain that way the min, median and maximum), check the syntax to write bash commands inside platformio to be able to print through serial the library commit, and to check the flags and terminal commands needed to print the optimization level in the terminal. All the test code for the PID, Kalman filter and MPC were taken from the code I had written previously in my thesis files, and no AI was used to modify those.

### Task 2.2 Starting point, measure matf32_mul and a solver

Only used AI to check the commands needed to print the memory usage of each routine. I wrote the test cases for ```matf32_mul``` and the cholesky solve myself with code from the previous task and looking at my older test files.

### Task 2.3 Making matf32_mul faster by changing the source code

Used AI only to flag potential areas of issue that could be increasing execution time. Versions 2 and 3 of changes for matf32_mul were my idea based on assumptions and my available literature, version 4 was a suggestion by the AI. Any changes to the test case done during this stage I did them myself with the code from previous tasks and test files.

### Task 2.4 Read the assembly of the Cortex-M4

I used AI for this task to help me look up the meaning of some assembly commands I didn't know the meaning of, as I couldn't find online the documentation of the assembly commands for this platform. Outside of that, I did my best to interpret the assembly lines myself and understand what they were doing, solving occassional questions about some commands or syntax with the AI, but my discussion of the results is based on what I understood, as the goal of the task was to understand it myself.