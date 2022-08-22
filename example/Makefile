

INCLUDE = ../include

SRC = ../src/

CC = gcc


main: lib
	$(CC) main.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/main

lib:
	$(MAKE) -C $(SRC)

clean:
	rm -f build/*

.PHONY: main lib clean