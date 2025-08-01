
SRC = ../src/

CC = gcc


main: lib
	$(CC) main.c $(SRC)*.o -I$(SRC) -lm -o build/main

lib:
	$(MAKE) -C $(SRC)

clean:
	rm -f build/*

.PHONY: main lib clean