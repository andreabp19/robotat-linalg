
SRC = ../src/

CC = gcc


main: lib
	$(CC) main.c $(SRC)*.o -I$(SRC) -lm -o build/main

matf32_demo: lib
	$(CC) matf32_demo.c $(SRC)*.o -I$(SRC) -lm -o build/matf32_demo

linsolve_demo: lib
	$(CC) linsolve_demo.c $(SRC)*.o -I$(SRC) -lm -o build/linsolve_demo

quadprog_demo: lib
	$(CC) quadprog_demo.c $(SRC)*.o -I$(SRC) -lm -o build/quadprog_demo

control_demo: lib
	$(CC) control_demo.c $(SRC)*.o -I$(SRC) -lm -o build/control_demo

robotics_demo: lib
	$(CC) robotics_demo.c $(SRC)*.o -I$(SRC) -lm -o build/robotics_demo

lib:
	$(MAKE) -C $(SRC)

clean:
	rm -f build/*

.PHONY: main lib clean matf32_demo
