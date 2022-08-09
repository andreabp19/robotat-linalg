
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"


float L_data[] = {1, 0, 0,
                  2, 3, 0,
                  4, 5, 6};

int
main(void)
{
    matf32_t L;

    matf32_init(&L, 3, 3, L_data);

    printf("Matrix L: \n");
    matf32_print(&L);

    printf("Testing check_triangular_lower: \n");

    bool ans = matf32_check_triangular_lower(&L) == true;

    if (ans)
    {
        printf("matf32_check_triangular_lower sucess.\n");
        return 0;
    }
    else
    {
        printf("matf32_check_triangular_lower failure.\n");
        return 1;
    }
}
