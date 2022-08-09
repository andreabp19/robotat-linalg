

INCLUDE = ../include

CC = gcc


all: matf32 linsolve control

matf32:
	$(MAKE) -C matf32

linsolve:
	$(CC) -c linsolve.c -I$(INCLUDE)

control:
	$(CC) -c robotat_control.c -I$(INCLUDE)


clean:
	$(MAKE) clean -C matf32
	rm -f *.o


.PHONY: matf32 linsolve clean