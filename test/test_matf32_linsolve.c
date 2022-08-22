
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"


float m1_data[] = {1, 0, 0,
                   2, 3, 0,
                   4, 5, 6};

float b1[] = {1, 8, 32};
float x1[] = {1, 2,  3};


float m2_data[] = {1, 2, 3,
                   0, 4, 5,
                   0, 0, 6};

float b2[] = {10, 13, 6};
float x2[] = { 3,  2, 1};


float m3_data[] = {1, 0, 1,
                   0, 2, 0,
                   1, 0, 3};

float b3[] = {4, 5, 6};
float x3[] = {3, 2.5, 1};


float m4_data[] = {1, 2, 1,
                   3, 2, 1,
                   1, 4, 3};

float b4[] = {1, 2, 3};
float x4[] = {0.5, -0.5, 1.5};

float result[] = {0, 0, 0};

bool ans = false;

matf32_t A;

int
main(void)
{
    matf32_init(&A, 3, 3, m1_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("vector b: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", b1[i]);
    }
    printf("\n");

    printf("Solving... \n");
    matf32_linsolve(&A, b1, result);

    printf("vector x: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", result[i]);
    }
    printf("\n");

    ans = is_equal(result, x1, 3);

    if (ans)
    {
        printf("matf32_linsolve sucess.\n");
    }
    else
    {
        printf("matf32_linsolve failure.\n");
    }



    matf32_init(&A, 3, 3, m2_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("vector b: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", b2[i]);
    }
    printf("\n");

    printf("Solving... \n");
    matf32_linsolve(&A, b2, result);

    printf("vector x: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", result[i]);
    }
    printf("\n");

    ans = is_equal(result, x2, 3);

    if (ans)
    {
        printf("matf32_linsolve sucess.\n");
    }
    else
    {
        printf("matf32_linsolve failure.\n");
    }



    matf32_init(&A, 3, 3, m3_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("vector b: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", b3[i]);
    }
    printf("\n");

    printf("Solving... \n");
    matf32_linsolve(&A, b3, result);

    printf("vector x: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", result[i]);
    }
    printf("\n");

    ans = is_equal(result, x3, 3);

    if (ans)
    {
        printf("matf32_linsolve sucess.\n");
    }
    else
    {
        printf("matf32_linsolve failure.\n");
    }


    matf32_init(&A, 3, 3, m4_data);
    zeros(&result, 3, 3);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("vector b: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", b4[i]);
    }
    printf("\n");

    printf("Solving... \n");
    print_linsolve_method(matf32_linsolve_method(&A));
    matf32_linsolve(&A, b4, result);

    printf("vector x: \n");
    for (int i=0; i<3; ++i)
    {
        printf("%f\n", result[i]);
    }
    printf("\n");

    ans = is_equal(result, x4, 3);

    if (ans)
    {
        printf("matf32_linsolve sucess.\n");
    }
    else
    {
        printf("matf32_linsolve failure.\n");
    }
}
