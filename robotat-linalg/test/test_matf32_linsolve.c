
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "robotat_linalg.h"
#include "linsolve_data.h"

float* A_list[] = {A2_data, A3_data, A4_data, A5_data, A6_data, A7_data, A8_data, A9_data, A10_data};
float* b_list[] = {b2_data, b3_data, b4_data, b5_data, b6_data, b7_data, b8_data, b9_data, b10_data};
float* r_list[] = {r2_data, r3_data, r4_data, r5_data, r6_data, r7_data, r8_data, r9_data, r10_data};

float x_data[10];

bool ans = false;

matf32_t A, B, X, Result;


int
main(void)
{
    
    clock_t time;
    float time_data = 0;

    for (uint8_t i = 0; i < (11-2); ++i)
    {
        uint8_t n = i + 2;

        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&B, n, 1, b_list[i]);
        matf32_init(&X, n, 1, x_data);
        matf32_init(&Result, n, 1, r_list[i]);
        matf32_zeros(&X);

        // printf("Matrix A: \n");
        // matf32_print(&A);

        // printf("vector b: \n");
        // matf32_print(&B);

        // printf("Testing linsolve: \n");
        metal_led_on();
        for (int i = 0; i < 100; ++i)
        {
            time = clock();
            matf32_linsolve(&A, &B, &X);
            time_data += ((float)clock()-time)/CLOCKS_PER_SEC;
        }

        //printf("vector x: \n");
        //matf32_print(&X);

        bool ans = matf32_is_equal(&X, &Result);

        printf("Time taken n=%i: %f seconds, %s\n", n, time_data/100, ans?"sucess":"failure");
        printf("Precision:\n");
        matf32_sub(&Result, &X, &X);

        matf32_print(&X);
    }

}
