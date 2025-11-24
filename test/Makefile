
SRC = ../src/

CC = gcc

all: matf32 linsolve quadprog control robotics

matf32: lib
	$(CC) matf32_test.c $(SRC)/*.o -I$(SRC) -lm -o build/matf32_test

matf32_pinv: lib
	$(CC) matf32_pinv_test.c $(SRC)/*.o -I$(SRC) -lm -o build/matf32_pinv_test

linsolve: lib
	$(CC) linsolve_test.c $(SRC)/*.o -I$(SRC) -lm -o build/linsolve_test

quadprog_qp: lib
	$(CC) quadprog_qp_test.c $(SRC)/*.o -I$(SRC) -lm -o build/quadprog_qp_test

quadprog_sqp: lib
	$(CC) quadprog_sqp_test.c $(SRC)/*.o -I$(SRC) -lm -o build/quadprog_sqp_test

control_nonlin: lib
	$(CC) control_nonlin_sys_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_nonlin_test

control_lti: lib
	$(CC) control_lti_sys_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_lti_test

control_mpc: lib
	$(CC) control_mpc_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_mpc_test

control_pid: lib
	$(CC) control_pid_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_pid_test

control_kalman: lib
	$(CC) control_kalman_filter_test.c $(SRC)/*.o -I$(SRC) -lm -o build/control_kalman_filter_test

robotics: lib
	$(CC) robotics_test.c $(SRC)/*.o -I$(SRC) -lm -o build/robotics_test

lib:
	$(MAKE) -C $(SRC)

clean:
	rm -f build/*

.PHONY: all clean lib
