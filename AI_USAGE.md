
# Notes about AI usage for this test/challenge branch

As indicated in the test/challenge instructions, AI tools were allowed. I used Claude, but complemented with googling things myself now and then, and my use of AI included: 1) researching about terms or concepts unknown to me, 2) searching for syntax or examples for unknown commands/routines, 3) fixing my own syntax or logic mistakes to save time (AFTER writing the code on my own). For transparency, my AI usage during this test is summarized by task, so the reviewers know how I approached each task.

## Part 1: Build and test

### Task 1.1 Build system

The objective of this task was to write a Makefile to compile the libraries and test files with ```gcc``` and ```clang```, across four optimization levels: ```-O0```, ```-Os```, ```-O1```, ```-O2```. My approach was to start with a command to compile a single file to a specific folder ```build/compiler-optimization-level/```, then modify that to include the rest of compiler and optimization level options until getting an optimated compilation system to generate each file.

My AI usage in this task consisted mostly of researching the meaning of unknown configurations for compilation and for the syntax that I needed for the Makefile, as my knowledge of Makefiles is very basic and I get confused easily with the syntax. I built my solution gradually, double-checking by testing it and asking the AI if I was writing it correctly at each step. Once I got a handle of the syntax, I progressed more on my own.

### Task 1.2 Self-checking tests

My AI use here consisted mostly of double-checking syntax and logic errors while modifying my code to make it self-checking, because I moved several sections to a function to reuse the code to check the results and removed other lines (which always risks breaking something inadvertently). Additionally, I used it to polish the logic and syntax of the function I wrote for computing residual-based error in the case of ```linsolve_test.c``` for the error of the solver methods.

### Task 1.3 Look inside the build

My use of AI in this task was to search the commands that I needed to read and print the characteristics of the built files on the terminal, as I'm on an arch-based linux machine and don't know from memory the commands. Also, to double-check my implementation of the build for the shared libraries and the test files with the shared libraries, as I made some syntax mistakes in my first try.

### Task 2.1 Measurement program

I used AI here to search up the configuration for the Cycle Counter, the syntax to write bash commands inside platformio to be able to print through serial the library commit, and to check the flags and terminal commands needed to print the optimization level in the terminal. All the test code for the PID, Kalman filter and MPC were taken from the code I had written previously in my thesis files, and no AI was used to modify those.