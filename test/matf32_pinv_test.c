
/**
 * @author Andrea Pineda
 * @date Created 8 Oct 2025
 * Last Modified 8 Oct 2025
 *          By: Andrea Pineda
 * 
 * Testing matf32_pinv (pseudoinverse of a matrix)
 */

// --------------------------------------------------
// Libraries 
// --------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "matf32_matlab_data.h"

// --------------------------------------------------
// Variables
// --------------------------------------------------

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* R_pinv_list[10] = {R_pinv1, R_pinv2, R_pinv3, R_pinv4, R_pinv5, R_pinv6, R_pinv7, R_pinv8, R_pinv9, R_pinv10};
matf32_t R;

float Apinv_data[MAX_MAT_SIZE];
matf32_t Apinv;

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

uint16_t samples = 100;

// --------------------------------------------------
// Testing
// --------------------------------------------------
int main(void)
{
    clock_t time;

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1;

        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n\n");

        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&R, n, n, R_pinv_list[i]);
        matf32_init(&Apinv, n, n, Apinv_data);
        matf32_init(&temp, n, n, temp_data);

        float condA = 0;
        matf32_cond(&A, &condA);
        printf("Condition Number A: %.9f\n\n", condA);

        // Method 1: Pseudoinverse calculated as: pinv(A) = (A'A)^-1 * A'
        float pinv_basic_time[samples];
        float mean_pinv_basic_time = 0;
        for (uint8_t j = 0; j < samples; j++)
        {
            time = clock();
            matf32_pinv(&A, &Apinv, BASIC_PINV);
            pinv_basic_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_pinv_basic_time = mean(pinv_basic_time, samples);

        // Check against MATLAB's result with a tolerance of 1E-05
        bool pinv_basic_ans = matf32_is_equal(&Apinv, &R);

        // If the tolerance check fails, print that result to see the difference
        if (!pinv_basic_ans)
        {
            matf32_sub(&Apinv, &R, &temp);

            printf("pinv(A) basic, result comparison: abs(pinv(A) - R):\n");
            for (uint8_t j = 0; j < n*n; j++)
            {
                temp.p_data[j] = fabs(temp.p_data[j]);
            }

            matf32_print(&temp);
        }

        // Method 2: Pseudoinverse calculated as: pinv(A) = V * S^-1 * U'
        float pinv_svd_time[samples];
        float mean_pinv_svd_time = 0;
        for (uint8_t j = 0; j < samples; j++)
        {
            time = clock();
            matf32_pinv(&A, &Apinv, SVD_PINV);
            pinv_svd_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_pinv_svd_time = mean(pinv_svd_time, samples);

        // Check against MATLAB's result with a tolerance of 1E-05
        bool pinv_svd_ans = matf32_is_equal(&Apinv, &R);

        // If the tolerance check fails, print that result to see the difference
        if (!pinv_svd_ans)
        {
            matf32_sub(&Apinv, &R, &temp);
            
            printf("pinv(A) SVD, result comparison: abs(pinv(A) - R):\n");
            for (uint8_t j = 0; j < n*n; j++)
            {
                temp.p_data[j] = fabs(temp.p_data[j]);
            }

            matf32_print(&temp);
        }

        //printf("matf32_pinv,basic%i\n",n);
        //printf("%.9f\n", mean_pinv_basic_time);
        //printf("matf32_pinv,svd%i\n",n);
        //printf("%.9f\n", mean_pinv_svd_time);

        printf("Basic, Mean Time (s): %.9f, 1E-05 Tolerance: %s\n", mean_pinv_basic_time, pinv_basic_ans?"success":"failure");
        printf("SVD  , Mean Time (s): %.9f, 1E-05 Tolerance: %s\n\n", mean_pinv_svd_time, pinv_svd_ans?"success":"failure");
    }
}