
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {1, 2, 3,
                  4, 5, 6,
                  7, 8, 9};

float v[] = {1, 2, 3};

float u1[] = {14, 32, 50};

float u2[] = {30, 36, 42};

float result[] = {0,0,0};

int
main()
{
    matf32_t A;

    matf32_init(&A, 3, 3, A_data);

    matf32_print(&A);

    printf("v: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", v[i]);
    }

    printf("\nTesting matf32_vecposmul:\n");
    matf32_vecposmul(&A, v, result);

    printf("result: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", result[i]);
    }

    bool ans1 = is_equal(result, u1, 3);

    if (ans1)
    {
        printf("matf32_vecposmul sucess.\n");
    }
    else
    {
        printf("matf32_vecposmul failure.\n");
    }


    printf("\nTesting matf32_vecpremul:\n");
    matf32_vecpremul(&A, v, result);

    printf("result: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", result[i]);
    }

    bool ans2 = is_equal(result, u2, 3);

    if (ans2)
    {
        printf("matf32_vecpremul sucess.\n");
    }
    else
    {
        printf("matf32_vecpremul failure.\n");
    }

    return (ans1 && ans2)?0:1;
}