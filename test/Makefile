
INCLUDE = ../include

SRC = ../src/

CC = gcc


linalg: lib
	$(CC) test_linalg.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_linalg

matf32_add: lib
	$(CC) test_matf32_add.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_add

matf32_sub: lib
	$(CC) test_matf32_sub.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_sub

matf32_scale: lib
	$(CC) test_matf32_scale.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_scale

matf32_trans: lib
	$(CC) test_matf32_trans.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_trans

matf32_check_triangular_upper: lib
	$(CC) test_matf32_check_triangular_upper.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_check_triangular_upper

matf32_check_triangular_lower: lib
	$(CC) test_matf32_check_triangular_lower.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_check_triangular_lower

matf32_cholesky: lib
	$(CC) test_matf32_cholesky.c $(SRC)*.o $(SRC)matf32/*.o -I$(INCLUDE) -lm -o build/test_matf32_cholesky


lib:
	$(MAKE) -C $(SRC)


clean:
	rm -f build/*


.PHONY: linalg clean