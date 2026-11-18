
# Notes about AI usage for this test/challenge branch

As indicated in the test/challenge instructions, AI tools were allowed. I used Claude, but complemented with googling things myself now and then, and my use of AI included: 1) researching about terms or concepts unknown to me, 2) searching for syntax or examples for unknown commands/routines, 3) fixing my own syntax or logic mistakes to save time (AFTER writing the code on my own). For transparency, my AI usage during this test is summarized by task, so the reviewers know how I approached each task.

## Part 1: Build and test

### Task 1.1 Build system.

The objective of this task was to write a Makefile to compile the libraries and test files with ```gcc``` and ```clang```, across four optimization levels: ```-O0```, ```-Os```, ```-O1```, ```-O2```. My approach was to start with a command to compile a single file to a specific folder ```build/compiler-optimization-level/```, then modify that to include the rest of compiler and optimization level options until getting an optimated compilation system to generate each file.

My AI usage in this task consisted mostly of researching the meaning of unknown configurations for compilation and for the syntax that I needed for the Makefile, as my knowledge of Makefiles is very basic and I get confused easily with the syntax. I built my solution gradually, double-checking by testing it and asking the AI if I was writing it correctly at each step. Once I got a handle of the syntax, I progressed more on my own.