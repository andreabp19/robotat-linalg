
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "robotat_linalg.h"
#include "quadprog_data.h"

float x_data[10];

float* Q_list[] = {Q2_data, Q3_data, Q4_data, Q5_data, Q6_data, Q7_data, Q8_data, Q9_data, Q10_data};
float* c_list[] = {c2_data, c3_data, c4_data, c5_data, c6_data, c7_data, c8_data, c9_data, c10_data};
float* A_list[] = {A2_data, A3_data, A4_data, A5_data, A6_data, A7_data, A8_data, A9_data, A10_data};
float* b_list[] = {b2_data, b3_data, b4_data, b5_data, b6_data, b7_data, b8_data, b9_data, b10_data};
float* r_list[] = {r2_data, r3_data, r4_data, r5_data, r6_data, r7_data, r8_data, r9_data, r10_data};


int main(void)
{
    clock_t time;
    float time_data = 0;

    matf32_t Q, c, Aeq, beq, Ain, bin, x, x0, Result;


    quadprog_t problem;

    quadprog_init(&problem, &Q, &c, NULL, NULL, &Ain, &bin, NULL);

    for (int i = 0; i < (11-2); ++i)
    {
        int n = i+2;

        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&Ain, n, n, A_list[i]);
        matf32_init(&bin, n, 1, b_list[i]);
        matf32_init(&x, n, 1, x_data);
        matf32_init(&Result, n, 1, r_list[i]);
        matf32_zeros(&x);

        for (int j = 0; j < 100; ++j)
        {
            time = clock();
            quadprog_sqp(&problem, &x);
            time_data += ((float)clock()-time)/CLOCKS_PER_SEC;
        }
        
        bool ans = matf32_is_equal(&x, &Result);
        
        printf("Time taken n=%i: %.9f seconds, %s\n", n, time_data/100, ans?"sucess":"failure");
        
        matf32_sub(&Result, &x, &x);
        //matf32_print(&x);
    }
}