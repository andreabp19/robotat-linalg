
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {1, 2, 1,
                  3, 2, 1,
                  1, 4, 3};;

float L_data[] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};

float U_data[] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};

float Result_data[] = {1, -0.0/0.0, -0.0/0.0,
                       0, sqrtf(2), -1.0/0.0,
                       1,        0, sqrtf(2)};


int
main(void)
{
    matf32_t A, L, U, Result;

    matf32_init(&A, 3, 3, A_data);
    matf32_init(&L, 3, 3, L_data);
    matf32_init(&U, 3, 3, U_data);
    matf32_init(&Result, 3, 3, Result_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("Testing LU: \n");
    matf32_lu(&A, &L, &U);

    printf("Matrix L: \n");
    matf32_print(&L);

    printf("Matrix U: \n");
    matf32_print(&U);

    matf32_mul(&L, &U, &Result);

    printf("Matrix Result: \n");
    matf32_print(&Result);


    bool ans = matf32_is_equal(&A, &Result);

    if (ans)
    {
        printf("matf32_lu sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_lu failure.\n");
        return 1;
    }
}
