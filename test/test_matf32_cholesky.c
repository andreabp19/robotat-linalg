
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {1, 0, 1,
                  0, 2, 0,
                  1, 0, 3};;

float C_data[] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};

float Result_data[] = {1, -0.0/0.0, -0.0/0.0,
                       0, sqrtf(2), -1.0/0.0,
                       1,        0, sqrtf(2)};

int
main(void)
{
    matf32_t A, C, Result;

    matf32_init(&A, 3, 3, A_data);
    matf32_init(&C, 3, 3, C_data);
    matf32_init(&Result, 3, 3, Result_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("Matrix C: \n");
    matf32_print(&C);

    printf("Testing cholesky: \n");
    matf32_cholesky(&A, &C);

    printf("Matrix C: \n");
    matf32_print(&C);

    bool ans = matf32_is_equal(&C, &Result);

    if (ans)
    {
        printf("matf32_cholesky sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_cholesky failure.\n");
        return 1;
    }
}
