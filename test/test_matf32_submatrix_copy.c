
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = { 1,  2,  3,  4,
                   5,  6,  7,  8,
                   9, 10, 11, 12,
                  13, 14, 15, 16};

float S_data[] = {0, 0, 0, 0,
                  0, 0, 0, 0,
                  0, 0, 0, 0,
                  0, 0, 0, 0};

float Result_data[] = {0, 0, 0, 0,
                       0, 3, 4, 0,
                       0, 7, 8, 0,
                       0, 0, 0, 0};


int
main(void)
{
    matf32_t A, S, Result;

    matf32_init(&A, 4, 4, A_data);
    matf32_init(&S, 4, 4, S_data);
    matf32_init(&Result, 4, 4, Result_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("Matrix S: \n");
    matf32_print(&S);

    printf("Testing submatrix_copy: \n");
    err_status_print(matf32_submatrix_copy(&A, &S, 0, 2, 1, 1, 2, 2));

    printf("Matrix S: \n");
    matf32_print(&S);


    bool ans = matf32_is_equal(&S, &Result);

    if (ans)
    {
        printf("matf32_submatrix_copy sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_submatrix_copy failure.\n");
        return 1;
    }
}
