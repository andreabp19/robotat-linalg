
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {0, 1, 2,
                  3, 4, 5,
                  6, 7, 8};

float B_data[] = {1, 1, 1,
                  1, 1, 1,
                  1, 1, 1};

float Result_data[] = {0, 0, 0,
                       0, 0, 0,
                       0, 0, 0};

int
main(void)
{
    clock_t time;
    float time_data = 0;
    
    matf32_t A, B, Result;

    matf32_init(&A, 3, 3, A_data);
    matf32_init(&B, 3, 3, B_data);
    matf32_init(&Result, 3, 3, Result_data);

    printf("Matrix A: \n");
    matf32_print(&A);
    printf("Matrix B: \n");
    matf32_print(&B);

    printf("Testing sub: \n");
    for (int i = 0; i < 100; ++i)
    {
        time = clock();
        matf32_sub(&A, &A, &B);
        time_data += ((float)clock()-time)/CLOCKS_PER_SEC;
    }
    matf32_print(&B);
    
    printf("Time taken: %.9f seconds.\n", time_data/100);

    bool ans = matf32_is_equal(&B, &Result);

    if (ans)
    {
        printf("matf32_sub sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_sub failure.\n");
        return 1;
    }
}