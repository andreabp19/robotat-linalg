
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"


float U_data[] = {1, 2, 3,
                  0, 4, 5,
                  0, 0, 6};

int
main(void)
{
    matf32_t U;

    matf32_init(&U, 3, 3, U_data);

    printf("Matrix U: \n");
    matf32_print(&U);

    printf("Testing check_triangular_upper: \n");

    bool ans = matf32_check_triangular_upper(&U) == true;

    if (ans)
    {
        printf("matf32_check_triangular_upper sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_check_triangular_upper failure.\n");
        return 1;
    }
}
