
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {0, 0,
                  0, 0,
                  0, 0};

float v[] = {1.0, 2.0, 3.0};


float Result_data[] = {1, 2,
                       2, 4,
                       3, 6};

int
main()
{
    matf32_t A, Result;

    matf32_init(&A, 3, 2, A_data);
    matf32_init(&Result, 3, 2, Result_data);

    matf32_print(&A);

    printf("v: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", v[i]);
    }

    printf("\nTesting matf32_vecmul_col_row:\n");
    matf32_vecmul_col_row(v, v, &A);

    matf32_print(&A);

    bool ans = matf32_is_equal(&A, &Result);

    if (ans)
    {
        printf("matf32_vecposmult sucess.\n");
    }
    else
    {
        printf("matf32_vecposmult failure.\n");
    }

    return ans?0:1;
}