

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "linsolve.h"
#include "linsolve_test_results.h"

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp2_data[MAX_MAT_SIZE];
matf32_t temp2;

float* b_list[10] = {b_data1, b_data2, b_data3, b_data4, b_data5, b_data6, b_data7, b_data8, b_data9, b_data10};
matf32_t b;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* A_L_list[10] = {A_L_data1, A_L_data2, A_L_data3, A_L_data4, A_L_data5, A_L_data6, A_L_data7, A_L_data8, A_L_data9, A_L_data10};
matf32_t A_L;

float* A_U_list[10] = {A_U_data1, A_U_data2, A_U_data3, A_U_data4, A_U_data5, A_U_data6, A_U_data7, A_U_data8, A_U_data9, A_U_data10};
matf32_t A_U;

float* B_list[10] = {B_data1, B_data2, B_data3, B_data4, B_data5, B_data6, B_data7, B_data8, B_data9, B_data10};
matf32_t B;

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

float* D_list[10] = {D_data1, D_data2, D_data3, D_data4, D_data5, D_data6, D_data7, D_data8, D_data9};
matf32_t D;

float* R_fwd_subs_list[10] = {R_fwd_subs_data1, R_fwd_subs_data2, R_fwd_subs_data3, R_fwd_subs_data4, R_fwd_subs_data5, R_fwd_subs_data6, R_fwd_subs_data7, R_fwd_subs_data8, R_fwd_subs_data9, R_fwd_subs_data10};
matf32_t R_fwd_subs;

float* R_bwd_subs_list[10] = {R_bwd_subs_data1, R_bwd_subs_data2, R_bwd_subs_data3, R_bwd_subs_data4, R_bwd_subs_data5, R_bwd_subs_data6, R_bwd_subs_data7, R_bwd_subs_data8, R_bwd_subs_data9, R_bwd_subs_data10};
matf32_t R_bwd_subs;

float* R_cholesky_list[10] = {R_cholesky_data1, R_cholesky_data2, R_cholesky_data3, R_cholesky_data4, R_cholesky_data5, R_cholesky_data6, R_cholesky_data7, R_cholesky_data8, R_cholesky_data9, R_cholesky_data10};
matf32_t R_cholesky;

float* R_qr_list[10] = {R_qr_data1, R_qr_data2, R_qr_data3, R_qr_data4, R_qr_data5, R_qr_data6, R_qr_data7, R_qr_data8, R_qr_data9};
matf32_t R_qr;

float* R_lu_list[10] = {R_lu_data1, R_lu_data2, R_lu_data3, R_lu_data4, R_lu_data5, R_lu_data6, R_lu_data7, R_lu_data8, R_lu_data9, R_lu_data10};
matf32_t R_lu;

float cond_A = 0;
float cond_A_L = 0;
float cond_A_U = 0;
float cond_C = 0;
float cond_B = 0;

err_status_t status;

void main(void)
{
    clock_t time;

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1;  // matrix dimension to use

        matf32_init(&temp, n, 1, temp_data);
        matf32_init(&b, n, 1, b_list[i]);
        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&A_L, n, n, A_L_list[i]);
        matf32_init(&A_U, n, n, A_U_list[i]);
        matf32_init(&C, n, n, C_list[i]);
        matf32_init(&B, n, n, B_list[i]);
        matf32_init(&R_fwd_subs, n, 1, R_fwd_subs_list[i]);
        matf32_init(&R_bwd_subs, n, 1, R_bwd_subs_list[i]);
        matf32_init(&R_cholesky, n, 1, R_cholesky_list[i]);
        matf32_init(&R_lu, n, 1, R_lu_list[i]);

        matf32_cond(&A, &cond_A);
        matf32_cond(&A_L, &cond_A_L);
        matf32_cond(&A_U, &cond_A_U);
        matf32_cond(&B, &cond_B);
        matf32_cond(&C, &cond_C);

        if (n > 1)
        {
            matf32_init(&temp2, n-1, 1, temp2_data);
            matf32_init(&D, n, n-1, D_list[i-1]);
            matf32_init(&R_qr, n-1, 1, R_qr_list[i-1]);
        }

        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n");
        //printf("cond_A = %.9f\n", cond_A);
        //printf("cond_L = %.9f\n", cond_A_L);
        //printf("cond_U = %.9f\n", cond_A_U);
        //printf("cond_C = %.9f\n", cond_C);
        //printf("cond_B = %.9f\n\n", cond_B);

        //bool symposdef = false;
        //symposdef = matf32_check_symposdef(&C);

        //printf("\nsymposdef C = %s\n\n", symposdef?"true":"false");

        // ---------------------------------------------------------------------------
        // 1. linsolve - Forward Substitution
        // ---------------------------------------------------------------------------

        float fwd_subs_time[x];
        float mean_fwd_subs_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            linsolve(&A_L, &b, &temp);
            fwd_subs_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_fwd_subs_time = mean(fwd_subs_time, x);

        bool fwd_subs_ans = matf32_is_equal(&temp, &R_fwd_subs);
        //printf("linsolve_fwd_subs%i\n", n);
        //printf("%.9f\n", mean_fwd_subs_time);
        printf("forward-subs ,time(s):%.9f,%s\n", mean_fwd_subs_time, fwd_subs_ans?"success":"failure");

        printf("Difference fwd_subs_result:\n");
        matf32_sub(&temp, &R_fwd_subs, &temp);
        matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // 2. linsolve - Backward Substitution
        // ---------------------------------------------------------------------------

        float bwd_subs_time[x];
        float mean_bwd_subs_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            linsolve(&A_U, &b, &temp);
            bwd_subs_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_bwd_subs_time = mean(bwd_subs_time, x);
        
        bool bwd_subs_ans = matf32_is_equal(&temp, &R_bwd_subs);
        //printf("linsolve_bwd_subs%i\n", n);
        //printf("%.9f\n", mean_bwd_subs_time);
        printf("backward-subs,time(s):%.9f,%s\n", mean_bwd_subs_time, bwd_subs_ans?"success":"failure");

        printf("Difference bwd_subs_result:\n");
        matf32_sub(&temp, &R_bwd_subs, &temp);
        matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // 3. linsolve - Cholesky
        // ---------------------------------------------------------------------------
        
        float cholesky_time[x];
        float mean_cholesky_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            status = linsolve(&C, &b, &temp);
            cholesky_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
            
        mean_cholesky_time = mean(cholesky_time, x);
        
        bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);
        //printf("linsolve_cholesky%i\n", n);
        //printf("%.9f\n", mean_cholesky_time);
        printf("cholesky     ,time(s):%.9f,%s\n", mean_cholesky_time, cholesky_ans?"success":"failure");

        printf("Difference cholesky_result:\n");
        matf32_sub(&temp, &R_cholesky, &temp);
        matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // 4. linsolve - QR
        // ---------------------------------------------------------------------------

        float qr_time[x];
        float mean_qr_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            status = linsolve(&D, &b, &temp2);
            qr_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_qr_time = mean(qr_time, x);

        bool qr_ans = matf32_is_equal(&temp2, &R_qr);
        //printf("linsolve_qr%i\n", n);
        //printf("%.9f\n", mean_qr_time);
        printf("qr           ,time(s):%.9f,%s\n", mean_qr_time, qr_ans?"success":"failure");

        printf("Difference qr_result:\n");
        matf32_sub(&temp, &R_qr, &temp);
        matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // 5. linsolve - LU
        // ---------------------------------------------------------------------------

        float lu_time[x];
        float mean_lu_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            status = linsolve(&B, &b, &temp);
            lu_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        //printf("temp:\n");
        //matf32_print(&temp);

        //printf("R_lu:\n");
        //matf32_print(&R_lu);

        mean_lu_time = mean(lu_time, x);
        
        bool lu_ans = matf32_is_equal(&temp, &R_lu);
        //printf("linsolve_lu%i\n", n);
        //printf("%.9f\n", mean_lu_time);
        printf("lu           ,time(s):%.9f,%s\n", mean_lu_time, lu_ans?"success":"failure");

        printf("Difference lu_result:\n");
        matf32_sub(&temp, &R_lu, &temp);
        matf32_print(&temp);
    }
}