
SRC = ../src/

CC = gcc

all: linalg matf32_add matf32_sub matf32_scale matf32_trans matf32_mul matf32_vecmul matf32_vecmul_col_row matf32_check_triangular_upper matf32_check_triangular_lower matf32_check_symmetric matf32_cholesky matf32_lu matf32_qr matf32_submatrix_copy matf32_linsolve

linalg: lib
	$(CC) test_linalg.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_linalg

matf32_add: lib
	$(CC) $(FLAGS) test_matf32_add.c $(SRC)*.o $(SRC)matf32/*.o -lm

matf32_sub: lib
	$(CC) test_matf32_sub.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_sub

matf32_scale: lib
	$(CC) test_matf32_scale.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_scale

matf32_trans: lib
	$(CC) test_matf32_trans.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_trans

matf32_mul: lib
	$(CC) test_matf32_mul.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_mul

matf32_vecmul: lib
	$(CC) test_matf32_vecmul.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_vecmul

matf32_vecmul_col_row: lib
	$(CC) test_matf32_vecmul_col_row.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_vecmul_col_row

matf32_check_triangular_upper: lib
	$(CC) test_matf32_check_triangular_upper.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_check_triangular_upper

matf32_check_triangular_lower: lib
	$(CC) test_matf32_check_triangular_lower.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_check_triangular_lower

matf32_check_symmetric: lib
	$(CC) test_matf32_check_symmetric.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_check_symmetric

matf32_cholesky: lib
	$(CC) test_matf32_cholesky.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_cholesky

matf32_lu: lib
	$(CC) test_matf32_lu.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_lu

matf32_qr: lib
	$(CC) test_matf32_qr.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_qr

matf32_submatrix_copy: lib
	$(CC) test_matf32_submatrix_copy.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_submatrix_copy

matf32_linsolve: lib
	$(CC) test_matf32_linsolve.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_matf32_linsolve

quadprog: lib
	$(CC) test_quadprog.c $(SRC)*.o $(SRC)matf32/*.o -lm -o build/test_quadprog


lib:
	$(MAKE) -C $(SRC)


clean:
	rm -f build/*


.PHONY: linalg clean