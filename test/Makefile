
SRC = ../src/

CC = gcc

all: matf32 linsolve control robotics

matf32: lib
	$(CC) matf32_test.c $(SRC)/*.o -I$(SRC) -lm -o build/matf32_test

linsolve: lib
	$(CC) linsolve_test.c $(SRC)/*.o -I$(SRC) -lm -o build/linsolve_test

control: lib
	$(CC) control_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_test

robotics: lib
	$(CC) robotics_test.c $(SRC)/*.o -I$(SRC) -lm -o build/robotics_test

lib:
	$(MAKE) -C $(SRC)

clean:
	rm -f build/*

.PHONY: all clean lib
