
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "robotat_linalg.h"
#include "math_data.h"

float* A_list[9] = {A2_data, A3_data, A4_data, A5_data, A6_data, A7_data, A8_data, A9_data, A10_data};
float* Rtrans_list[9] = {Result_trans_2_data, Result_trans_3_data, Result_trans_4_data, Result_trans_5_data, Result_trans_6_data, Result_trans_7_data, Result_trans_8_data, Result_trans_9_data, Result_trans_10_data};

float B_data[10];

int
main(void)
{
    clock_t time;
    float time_data = 0;

    matf32_t A, B, Result;

    for (int i = 0; i < (11-2); ++i)
    {
        int n = i+2;

        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&B, n, n, B_data);
        matf32_init(&Result, n, n, Rtrans_list[i]);

        for (int j = 0; j < 100; ++j)
        {
            time = clock();
            matf32_trans(&A, &B);
            time_data += ((float)clock()-time)/CLOCKS_PER_SEC;
        }
        
        bool ans = matf32_is_equal(&B, &Result);
        
        printf("Time taken n=%i: %.9f seconds, %s\n", n, time_data/100, ans?"sucess":"failure");
        
        matf32_sub(&Result, &B, &B);
        //matf32_print(&x);
    }
}