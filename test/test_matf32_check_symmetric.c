
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"


float A_data[] = {1, 0, 1,
                  0, 2, 0,
                  1, 0, 3};

int
main(void)
{
    matf32_t A;

    matf32_init(&A, 3, 3, A_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("Testing check_symmetric: \n");

    bool ans = matf32_check_symmetric(&A) == true;

    if (ans)
    {
        printf("matf32_check_symmetric sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_check_symmetric failure.\n");
        return 1;
    }
}
