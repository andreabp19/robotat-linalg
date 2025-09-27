

CC = gcc


all: matf32 linsolve quadprog control robotics

matf32:
	$(CC) -c matf32.c  -lm

linsolve:
	$(CC) -c linsolve.c

quadprog:
	$(CC) -c quadprog.c

control:
	$(CC) -c robotat_control.c

robotics:
	$(CC) -c robotat_robotics.c


clean:
	rm -f *.o


.PHONY: clean matf32 linsolve quadprog control robotics
