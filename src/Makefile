

CC = gcc


all: matf32 linsolve robotics

matf32:
	$(CC) -c matf32.c  -lm

linsolve:
	$(CC) -c linsolve.c

robotics:
	$(CC) -c robotat_robotics.c


clean:
	rm -f *.o


.PHONY: clean matf32 linsolve robotics
