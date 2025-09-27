
/**
 * @author Andrea Pineda
 * @date Created 23 Aug 2025, last modified: 22 Sep 2025
 * 
 * For testing matf32 in computer
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "matf32_test_results.h"

float dot_result = 0;

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp_L_data[MAX_MAT_SIZE];
matf32_t temp_L;

float temp_U_data[MAX_MAT_SIZE];
matf32_t temp_U;

float temp_Q_data[MAX_MAT_SIZE];
matf32_t temp_Q;

float temp_R_data[MAX_MAT_SIZE];
matf32_t temp_R;

float U_data[MAX_MAT_SIZE];
matf32_t U;

float S_data[MAX_MAT_SIZE];
matf32_t S;

float V_data[MAX_MAT_SIZE];
matf32_t V;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* B_list[10] = {B_data1, B_data2, B_data3, B_data4, B_data5, B_data6, B_data7, B_data8, B_data9, B_data10};
matf32_t B;

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

float* D_list[9] = {D_data1, D_data2, D_data3, D_data4, D_data5, D_data6, D_data7, D_data8, D_data9};
matf32_t D;

float* rowvec_list[10] = {rowvec_data1, rowvec_data2, rowvec_data3, rowvec_data4, rowvec_data5, rowvec_data6, rowvec_data7, rowvec_data8, rowvec_data9, rowvec_data10};
matf32_t rowvec;

float* colvec_list[10] = {colvec_data1, colvec_data2, colvec_data3, colvec_data4, colvec_data5, colvec_data6, colvec_data7, colvec_data8, colvec_data9, colvec_data10};
matf32_t colvec;


// Matrix operations Matlab Results
float* R_add_list[10] = {R_add1, R_add2, R_add3, R_add4, R_add5, R_add6, R_add7, R_add8, R_add9, R_add10};
matf32_t R_add;

float* R_sub_list[10] = {R_sub1, R_sub2, R_sub3, R_sub4, R_sub5, R_sub6, R_sub7, R_sub8, R_sub9, R_sub10};
matf32_t R_sub;

float* R_scale_list[10] = {R_scale1, R_scale2, R_scale3, R_scale4, R_scale5, R_scale6, R_scale7, R_scale8, R_scale9, R_scale10};
matf32_t R_scale;

float* R_trans_list[10] = {R_trans1, R_trans2, R_trans3, R_trans4, R_trans5, R_trans6, R_trans7, R_trans8, R_trans9, R_trans10};
matf32_t R_trans;

float* R_mul_list[10] = {R_mul1, R_mul2, R_mul3, R_mul4, R_mul5, R_mul6, R_mul7, R_mul8, R_mul9, R_mul10};
matf32_t R_mul;

float* R_inv_list[10] = {R_inv1, R_inv2, R_inv3, R_inv4, R_inv5, R_inv6, R_inv7, R_inv8, R_inv9, R_inv10};
matf32_t R_inv;

float* R_dot_list[10] = {R_dot1, R_dot2, R_dot3, R_dot4, R_dot5, R_dot6, R_dot7, R_dot8, R_dot9, R_dot10};
matf32_t R_dot;

float* R_vecpremul_list[10] = {R_vecpremul1, R_vecpremul2, R_vecpremul3, R_vecpremul4, R_vecpremul5, R_vecpremul6, R_vecpremul7, R_vecpremul8, R_vecpremul9, R_vecpremul10};
matf32_t R_vecpremul;

float* R_vecposmul_list[10] = {R_vecposmul1, R_vecposmul2, R_vecposmul3, R_vecposmul4, R_vecposmul5, R_vecposmul6, R_vecposmul7, R_vecposmul8, R_vecposmul9, R_vecposmul10};
matf32_t R_vecposmul;

float* R_vecmul_list[10] = {R_vecmul1, R_vecmul2, R_vecmul3, R_vecmul4, R_vecmul5, R_vecmul6, R_vecmul7, R_vecmul8, R_vecmul9, R_vecmul10};
matf32_t R_vecmul;

float* R_arr_add_list[10] = {R_arr_add1, R_arr_add2, R_arr_add3, R_arr_add4, R_arr_add5, R_arr_add6, R_arr_add7, R_arr_add8, R_arr_add9, R_arr_add10};
matf32_t R_arr_add;

float* R_arr_sub_list[10] = {R_arr_sub1, R_arr_sub2, R_arr_sub3, R_arr_sub4, R_arr_sub5, R_arr_sub6, R_arr_sub7, R_arr_sub8, R_arr_sub9, R_arr_sub10};
matf32_t R_arr_sub;

float* R_arr_mul_list[10] = {R_arr_mul1, R_arr_mul2, R_arr_mul3, R_arr_mul4, R_arr_mul5, R_arr_mul6, R_arr_mul7, R_arr_mul8, R_arr_mul9, R_arr_mul10};
matf32_t R_arr_mul;

float* R_pinv_list[10] = {R_pinv1, R_pinv2, R_pinv3, R_pinv4, R_pinv5, R_pinv6, R_pinv7, R_pinv8, R_pinv9, R_pinv10};
matf32_t R_pinv;


// Matrix Factorizations Matlab Results
float* R_lu_L_list[10] = {R_lu_L1, R_lu_L2, R_lu_L3, R_lu_L4, R_lu_L5, R_lu_L6, R_lu_L7, R_lu_L8, R_lu_L9, R_lu_L10};
matf32_t R_lu_L;

float* R_lu_U_list[10] = {R_lu_U1, R_lu_U2, R_lu_U3, R_lu_U4, R_lu_U5, R_lu_U6, R_lu_U7,  R_lu_U8, R_lu_U9, R_lu_U10};
matf32_t R_lu_U;

float* R_cholesky_list[10] = {R_cholesky1, R_cholesky2, R_cholesky3, R_cholesky4, R_cholesky5, R_cholesky6, R_cholesky7, R_cholesky8, R_cholesky9, R_cholesky10};
matf32_t R_cholesky;

float* R_qr_Q_list[9] = {R_qr_Q1, R_qr_Q2, R_qr_Q3, R_qr_Q4, R_qr_Q5, R_qr_Q6, R_qr_Q7, R_qr_Q8, R_qr_Q9};
matf32_t R_qr_Q;

float* R_qr_R_list[9] = {R_qr_R1, R_qr_R2, R_qr_R3, R_qr_R4, R_qr_R5, R_qr_R6, R_qr_R7, R_qr_R8, R_qr_R9};
matf32_t R_qr_R;

// Values for testing and validating matf32_givens_pair
float givens_a_list[10] = {givens_a1, givens_a2, givens_a3, givens_a4, givens_a5, givens_a6, givens_a7, givens_a8, givens_a9};
float givens_b_list[10] = {givens_b1, givens_b2, givens_b3, givens_b4, givens_b5, givens_b6, givens_b7, givens_b8, givens_b9};
float R_givens_c_list[10] = {R_givens_c1, R_givens_c2, R_givens_c3, R_givens_c4, R_givens_c5, R_givens_c6, R_givens_c7, R_givens_c8, R_givens_c9};
float R_givens_s_list[10] = {R_givens_s1, R_givens_s2, R_givens_s3, R_givens_s4, R_givens_s5, R_givens_s6, R_givens_s7, R_givens_s8, R_givens_s9};

float givens_c = 0;
float givens_s = 0;

// Values for testing and validating matf32_symschur2_pair
float R_symschur2_c_list[10] = {R_symschur2_c1, R_symschur2_c2, R_symschur2_c3, R_symschur2_c4, R_symschur2_c5, R_symschur2_c6, R_symschur2_c7, R_symschur2_c8, R_symschur2_c9};
float R_symschur2_s_list[10] = {R_symschur2_s1, R_symschur2_s2, R_symschur2_s3, R_symschur2_s4, R_symschur2_s5, R_symschur2_s6, R_symschur2_s7, R_symschur2_s8, R_symschur2_s9};

uint16_t symschur2_p_list[10] = {symschur2_p1, symschur2_p2, symschur2_p3, symschur2_p4, symschur2_p5, symschur2_p6, symschur2_p7, symschur2_p8, symschur2_p9};
uint16_t symschur2_q_list[10] = {symschur2_q1, symschur2_q2, symschur2_q3, symschur2_q4, symschur2_q5, symschur2_q6, symschur2_q7, symschur2_q8, symschur2_q9};

float symschur2_c = 0;
float symschur2_s = 0;

// Condition numbers for all input matrices used
float cond_A = 0;
float cond_B = 0;
float cond_C = 0;
float cond_D = 0;

// Rotation matrices
float* R_givens_rotation_list[10] = {R_givens_rotation1, R_givens_rotation2, R_givens_rotation3, R_givens_rotation4, R_givens_rotation5, R_givens_rotation6, R_givens_rotation7, R_givens_rotation8, R_givens_rotation9};
matf32_t R_givens_rotation;

float* R_jacobi_rotation_list[10] = {R_jacobi_rotation1, R_jacobi_rotation2, R_jacobi_rotation3, R_jacobi_rotation4, R_jacobi_rotation5, R_jacobi_rotation6, R_jacobi_rotation7, R_jacobi_rotation8, R_jacobi_rotation9};
matf32_t R_jacobi_rotation;

int main(void)
{
    clock_t time;

    printf("\n\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&temp, n, n, temp_data);
        matf32_init(&temp_L, n, n, temp_L_data);
        matf32_init(&temp_U, n, n, temp_U_data);
        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&B, n, n, B_list[i]);
        matf32_init(&C, n, n, C_list[i]);
        matf32_init(&rowvec, 1, n, rowvec_list[i]);
        matf32_init(&colvec, n, 1, colvec_list[i]);
        matf32_init(&R_add, n, n, R_add_list[i]);
        matf32_init(&R_sub, n, n, R_sub_list[i]);
        matf32_init(&R_scale, n, n, R_scale_list[i]);
        matf32_init(&R_trans, n, n, R_trans_list[i]);
        matf32_init(&R_mul, n, n, R_mul_list[i]);
        matf32_init(&R_inv, n, n, R_inv_list[i]);
        matf32_init(&R_dot, 1, 1, R_dot_list[i]);
        matf32_init(&R_vecpremul, 1, n, R_vecpremul_list[i]);
        matf32_init(&R_vecposmul, n, 1, R_vecposmul_list[i]);
        matf32_init(&R_vecmul, n, n, R_vecmul_list[i]);
        matf32_init(&R_arr_add, n, n, R_arr_add_list[i]);
        matf32_init(&R_arr_sub, n, n, R_arr_sub_list[i]);
        matf32_init(&R_arr_mul, n, n, R_arr_mul_list[i]);
        matf32_init(&R_pinv, n, n, R_pinv_list[i]);
        matf32_init(&R_lu_L, n, n, R_lu_L_list[i]);
        matf32_init(&R_lu_U, n, n, R_lu_U_list[i]);
        matf32_init(&R_cholesky, n, n, R_cholesky_list[i]);
        
        // D, Q and R have only 9 elements in the list of values (no element for n=1 as D is a rectangular matrix n x (n-1))
        if (n > 1)
        {
            // QR-related matrices
            matf32_init(&temp_Q, n, n, temp_Q_data);
            matf32_init(&temp_R, n, n-1, temp_R_data);
            matf32_init(&D, n, n-1, D_list[i-1]);
            matf32_init(&R_qr_Q, n, n, R_qr_Q_list[i-1]);
            matf32_init(&R_qr_R, n, n-1, R_qr_R_list[i-1]);

            // Rotation matrices
            matf32_init(&R_givens_rotation, n, n, R_givens_rotation_list[i-1]);
            matf32_init(&R_jacobi_rotation, n, n, R_jacobi_rotation_list[i-1]);

            // SVD matrices
            matf32_init(&U, A.num_rows, A.num_cols, U_data);
            matf32_init(&S, A.num_rows, A.num_cols, S_data);
            matf32_init(&V, A.num_rows, A.num_cols, V_data);
        }

        const matf32_t* mat_array[3] = {&A, &A, &A};
        
        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n");

        matf32_cond(&A, &cond_A);
        //matf32_cond(&B, &cond_B);
        //matf32_cond(&C, &cond_C);
        //matf32_cond(&D, &cond_D);

        printf("cond_A = %.9f\n", cond_A);
        //printf("cond_B = %.9f\n", cond_B);
        //printf("cond_C = %.9f\n", cond_C);
        //printf("cond D = %.9f\n", cond_D);

        // ---------------------------------------------------------------------------
        // matf32_add
        // ---------------------------------------------------------------------------

        float add_time[x];
        float mean_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_add(&A, &A, &temp);
            add_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_add_time = mean(add_time, x);

        bool add_ans = matf32_is_equal(&temp, &R_add);
        //printf("matf32_add%i\n",n);
        //printf("%.9f\n", mean_add_time);
        //printf("add      ,time(s):%.9f,%s\n\n", mean_add_time, add_ans?"success":"failure");

        //printf("Difference matf32_add:\n");
        //matf32_sub(&temp, &R_add, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_sub
        // ---------------------------------------------------------------------------

        float sub_time[x];
        float mean_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_sub(&A, &A, &temp);
            sub_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_sub_time = mean(sub_time, x);

        bool sub_ans = matf32_is_equal(&temp, &R_sub);
        //printf("matf32_sub%i\n", n);
        //printf("%.9f\n", mean_sub_time);
        //printf("sub      ,time(s):%.9f,%s\n\n", mean_sub_time, sub_ans?"success":"failure");

        //printf("Difference matf32_sub:\n");
        //matf32_sub(&temp, &R_sub, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_scale
        // ---------------------------------------------------------------------------

        float scale_time[x];
        float mean_scale_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_scale(&A, c, &temp);
            scale_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_scale_time = mean(scale_time, x);

        bool scale_ans = matf32_is_equal(&temp, &R_scale);
        //printf("matf32_scale%i\n", n);
        //printf("%.9f\n", mean_scale_time);
        //printf("scale    ,time(s):%.9f,%s\n\n", mean_scale_time, scale_ans?"success":"failure");

        //printf("Difference matf32_scale:\n");
        //matf32_sub(&temp, &R_scale, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_trans
        // ---------------------------------------------------------------------------

        float trans_time[x];
        float mean_trans_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_trans(&A, &temp);
            trans_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_trans_time = mean(trans_time, x);

        bool trans_ans = matf32_is_equal(&temp, &R_trans);
        //printf("matf32_trans%i\n", n);
        //printf("%.9f\n", mean_trans_time);
        //printf("trans    ,time(s):%.9f,%s\n\n", mean_trans_time, trans_ans?"success":"failure");

        //printf("Difference matf32_trans:\n");
        //matf32_sub(&temp, &R_trans, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_mul
        // ---------------------------------------------------------------------------

        float mul_time[x];
        float mean_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_mul(&A, &A, &temp);
            mul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_mul_time = mean(mul_time, x);

        bool mul_ans = matf32_is_equal(&temp, &R_mul);
        //printf("matf32_mul%i\n", n);
        //printf("%.9f\n", mean_mul_time);
        //printf("mul      ,time(s):%.9f,%s\n\n", mean_mul_time, mul_ans?"success":"failure");

        //printf("Difference matf32_mul:\n");
        //matf32_sub(&temp, &R_mul, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_inv
        // ---------------------------------------------------------------------------

        float inv_time[x];
        float mean_inv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_inv(&A, &temp);
            inv_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
    
        mean_inv_time = mean(inv_time, x);

        bool inv_ans = matf32_is_equal(&temp, &R_inv);
        //printf("matf32_inv%i\n", n);
        //printf("%.9f\n", mean_inv_time);
        //printf("inv      ,time(s):%.9f,%s\n\n", mean_inv_time, inv_ans?"success":"failure");

        //printf("Difference matf32_inv:\n");
        //matf32_sub(&temp, &R_inv, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_dot
        // ---------------------------------------------------------------------------

        float dot_time[x];
        float mean_dot_time = 0;

        float dot_temp_data[1];
        matf32_t dot_temp;

        matf32_init(&dot_temp, 1, 1, dot_temp_data);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_dot(&rowvec, &colvec, dot_temp_data);
            dot_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        matf32_set(&dot_temp, 1, 1, *dot_temp_data);

        mean_dot_time = mean(dot_time, x);

        bool dot_ans = matf32_is_equal(&dot_temp, &R_dot);
        //printf("matf32_dot%i\n", n);
        //printf("%.9f\n", mean_dot_time);
        //printf("dot      ,time(s):%.9f,%s\n\n", mean_dot_time, dot_ans?"success":"failure");

        //printf("Difference matf32_dot:\n");
        //matf32_sub(&temp, &R_dot, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_vecposmul
        // ---------------------------------------------------------------------------
        
        float vecposmul_time[x];
        float mean_vecposmul_time = 0;
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
            vecposmul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecposmul_time = mean(vecposmul_time, x);

        bool vecposmul_ans = matf32_is_equal(&vecposmul_temp, &R_vecposmul);
        //printf("matf32_vecposmul%i\n", n);
        //printf("%.9f\n", mean_vecposmul_time);
        //printf("vecposmul,time(s):%.9f,%s\n\n", mean_vecposmul_time, vecposmul_ans?"success":"failure");

        //printf("Difference matf32_vecposmul:\n");
        //matf32_sub(&vecposmul_temp, &R_vecposmul, &vecposmul_temp);
        //matf32_print(&vecposmul_temp);

        // ---------------------------------------------------------------------------
        // matf32_vecpremul
        // ---------------------------------------------------------------------------

        float vecpremul_time[x];
        float mean_vecpremul_time = 0;
        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
            vecpremul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecpremul_time = mean(vecpremul_time, x);

        bool vecpremul_ans = matf32_is_equal(&vecpremul_temp, &R_vecpremul);
        //printf("matf32_vecpremul%i\n", n);
        //printf("%.9f\n", mean_vecpremul_time);
        //printf("vecpremul,time(s):%.9f,%s\n\n", mean_vecpremul_time, vecpremul_ans?"success":"failure");

        //printf("Difference matf32_vecpremul:\n");
        //matf32_sub(&vecpremul_temp, &R_vecpremul, &vecpremul_temp);
        //matf32_print(&vecpremul_temp);

        // ---------------------------------------------------------------------------
        // matf32_vecmul_col_row
        // ---------------------------------------------------------------------------
        
        float vecmul_time[x];
        float mean_vecmul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
            vecmul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecmul_time = mean(vecmul_time, x);

        bool vecmul_ans = matf32_is_equal(&temp, &R_vecmul);
        //printf("matf32_vecmul%i\n", n);
        //printf("%.9f\n", mean_vecmul_time);
        //printf("vecmul   ,time(s):%.9f,%s\n\n", mean_vecmul_time, vecmul_ans?"success":"failure");

        //printf("Difference matf32_vecmul:\n");
        //matf32_sub(&temp, &R_vecmul, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_arr_add
        // ---------------------------------------------------------------------------

        float arr_add_time[x];
        float mean_arr_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_add(mat_array, 3, &temp);
            arr_add_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_add_time = mean(arr_add_time, x);

        bool arr_add_ans = matf32_is_equal(&temp, &R_arr_add);
        //printf("matf32_arr_add%i\n", n);
        //printf("%.9f\n", mean_arr_add_time);
        //printf("arr_add  ,time(s):%.9f,%s\n\n", mean_arr_add_time, arr_add_ans?"success":"failure");

        //printf("Difference matf32_arr_add:\n");
        //matf32_sub(&temp, &R_arr_add, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_arr_sub
        // ---------------------------------------------------------------------------

        float arr_sub_time[x];
        float mean_arr_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_sub(mat_array, 3, &temp);
            arr_sub_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_sub_time = mean(arr_sub_time, x);

        bool arr_sub_ans = matf32_is_equal(&temp, &R_arr_sub);
        //printf("matf32_arr_sub%i\n", n);
        //printf("%.9f\n", mean_arr_sub_time);
        //printf("arr_sub  ,time(s):%.9f,%s\n\n", mean_arr_sub_time, arr_sub_ans?"success":"failure");

        //printf("Difference matf32_arr_sub:\n");
        //matf32_sub(&temp, &R_arr_sub, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_arr_mul
        // ---------------------------------------------------------------------------

        float arr_mul_time[x];
        float mean_arr_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_mul(mat_array, 3, &temp);
            arr_mul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_mul_time = mean(arr_mul_time, x);

        bool arr_mul_ans = matf32_is_equal(&temp, &R_arr_mul);
        //printf("matf32_arr_mul%i\n", n);
        //printf("%.9f\n", mean_arr_mul_time);
        //printf("arr_mul  ,time(s):%.9f,%s\n\n", mean_arr_mul_time, arr_mul_ans?"success":"failure");

        //printf("Difference matf32_arr_mul:\n");
        //matf32_sub(&temp, &R_arr_mul, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_pinv
        // ---------------------------------------------------------------------------

        float pinv_time[x];
        float mean_pinv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_pinv(&A, &temp);
            pinv_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_pinv_time = mean(pinv_time, x);

        bool pinv_ans = matf32_is_equal(&temp, &R_pinv);
        //printf("matf32_pinv%i\n",n);
        //printf("%.9f\n", mean_pinv_time);
        //printf("pinv     ,time(s):%.9f,%s\n\n", mean_pinv_time, pinv_ans?"success":"failure");

        //printf("Difference matf32_pinv:\n");
        //matf32_sub(&temp, &R_pinv, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_lu
        // ---------------------------------------------------------------------------
        
        float lu_time[x];
        float mean_lu_time = 0;
        
        uint16_t p_index[n];

        matf32_zeros(&temp_L);
        matf32_zeros(&temp_U);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_lu(&B, &temp_L, &temp_U, p_index);
            lu_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_lu_time = mean(lu_time, x);

        bool lu_L_ans = matf32_is_equal(&temp_L, &R_lu_L);
        bool lu_U_ans = matf32_is_equal(&temp_U, &R_lu_U);
        //printf("matf32_lu%i\n",n);
        //printf("%.9f\n", mean_lu_time);
        //printf("lu       ,time(s):%.9f,L:%s,U:%s\n\n", mean_lu_time, lu_L_ans?"success":"failure", lu_U_ans?"success":"failure");

        //printf("Difference L:\n");
        //matf32_sub(&temp_L, &R_lu_L, &temp_L);
        //matf32_print(&temp_L);

        //printf("Difference U:\n");
        //matf32_sub(&temp_U, &R_lu_U, &temp_U);
        //matf32_print(&temp_U);

        // ---------------------------------------------------------------------------
        // matf32_cholesky
        // ---------------------------------------------------------------------------

        float cholesky_time[x];
        float mean_cholesky_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_cholesky(&C, &temp);
            cholesky_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_cholesky_time = mean(cholesky_time, x);

        bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);
        //printf("matf32_cholesky%i\n", n);
        //printf("%.9f\n", mean_cholesky_time);
        //printf("cholesky ,time(s):%.9f,%s\n\n", mean_cholesky_time, cholesky_ans?"success":"failure");

        //printf("Difference matf32_cholesky:\n");
        //matf32_sub(&temp, &R_cholesky, &temp);
        //matf32_print(&temp);

        // ---------------------------------------------------------------------------
        // matf32_qr
        // ---------------------------------------------------------------------------

        if (n > 1)
        {
            float qr_time[x];
            float mean_qr_time = 0;

            matf32_zeros(&temp_Q);
            matf32_zeros(&temp_R);

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_qr(&D, &temp_Q, &temp_R);
                qr_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }
    
            mean_qr_time = mean(qr_time, x);

            bool qr_Q_ans = matf32_is_equal(&temp_Q, &R_qr_Q);
            bool qr_R_ans = matf32_is_equal(&temp_R, &R_qr_R);
            //printf("matf32_qr%i\n", n);
            //printf("%.9f\n", mean_qr_time);
            printf("qr       ,time(s):%.9f,Q:%s,R:%s\n\n", mean_qr_time, qr_Q_ans?"success":"failure", qr_R_ans?"success":"failure");
        
            //printf("Difference Q:\n");
            //matf32_sub(&temp_Q, &R_qr_Q, &temp_Q);
            //matf32_print(&temp_Q);

            //printf("Difference R:\n");
            //matf32_sub(&temp_R, &R_qr_R, &temp_R);
            //matf32_print(&temp_R);
        }

        // ---------------------------------------------------------------------------
        // matf32_givens_pair
        // ---------------------------------------------------------------------------

        /*if (n > 1)
        {
            float givens_pair_time[x];
            float mean_givens_pair_time = 0;

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_givens_pair(givens_a_list[i-1], givens_b_list[i-1], &givens_c, &givens_s);
                givens_pair_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }

            mean_givens_pair_time = mean(givens_pair_time, x);

            bool givens_c_ans = (fabs(givens_c-R_givens_c_list[i-1])<1E-05)?1:0;
            bool givens_s_ans = (fabs(givens_s-R_givens_s_list[i-1])<1E-05)?1:0;
            printf("matf32_givens_pair%i\n", n);
            printf("%.9f\n", mean_givens_pair_time);
            //printf("givens_pair, time(s):%.9f, c: %s, s: %s\n\n", mean_givens_pair_time, givens_c_ans?"success":"failure", givens_s_ans?"success":"failure");
        }*/

        // ---------------------------------------------------------------------------
        // matf32_symschur2_pair
        // ---------------------------------------------------------------------------

        /*if (n > 1)
        {
            float symschur2_pair_time[x];
            float mean_symschur2_pair_time = 0;

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_symschur2_pair(&C, symschur2_p_list[i-1]-1, symschur2_q_list[i-1]-1, &symschur2_c, &symschur2_s);
                symschur2_pair_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }

            mean_symschur2_pair_time = mean(symschur2_pair_time, x);

            bool symschur2_c_ans = (fabs(symschur2_c-R_symschur2_c_list[i-1])<1E-05)?1:0;
            bool symschur2_s_ans = (fabs(symschur2_s-R_symschur2_s_list[i-1])<1E-05)?1:0;
            printf("matf32_symschur2_pair%i\n", n);
            printf("%.9f\n", mean_symschur2_pair_time);
            //printf("symschur2_pair, time(s):%.9f, c: %s, s: %s\n\n", mean_symschur2_pair_time, symschur2_c_ans?"success":"failure", symschur2_s_ans?"success":"failure");
        }*/
            
        // ---------------------------------------------------------------------------
        // matf32_generate_rotation: Givens Rotation
        // ---------------------------------------------------------------------------

        /*if (n > 1)
        {
            float givens_rotation_time[x];
            float mean_givens_rotation_time = 0;

            matf32_zeros(&temp);

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_generate_rotation(&C, symschur2_p_list[i-1], symschur2_q_list[i-1], &temp, GIVENS, givens_a_list[i-1], givens_b_list[i-1]);
                givens_rotation_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }

            mean_givens_rotation_time = mean(givens_rotation_time, x);

            bool givens_rotation_ans = matf32_is_equal(&temp, &R_givens_rotation);
            printf("givens_rotation%i\n", n);
            printf("%.9f\n", mean_givens_rotation_time);
            //printf("givens_rotation,time(s):%.9f,%s\n\n", mean_givens_rotation_time, givens_rotation_ans?"success":"failure");

            //printf("Difference matf32_generate_rotation: givens:\n");
            //matf32_sub(&temp, &R_givens_rotation, &temp);
            //matf32_print(&temp);
        }*/

        // ---------------------------------------------------------------------------
        // matf32_generate_rotation: Jacobi Rotation
        // ---------------------------------------------------------------------------

        /*if (n > 1)
        {
            float jacobi_rotation_time[x];
            float mean_jacobi_rotation_time = 0;

            matf32_zeros(&temp);

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_generate_rotation(&C, symschur2_p_list[i-1], symschur2_q_list[i-1], &temp, JACOBI, 0, 0);
                jacobi_rotation_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }

            mean_jacobi_rotation_time = mean(jacobi_rotation_time, x);

            bool jacobi_rotation_ans = matf32_is_equal(&temp, &R_jacobi_rotation);
            printf("jacobi_rotation%i\n", n);
            printf("%.9f\n", mean_jacobi_rotation_time);
            //printf("jacobi_rotation,time(s):%.9f,%s\n\n", mean_jacobi_rotation_time, jacobi_rotation_ans?"success":"failure");

            //printf("Difference matf32_jacobi_rotation:\n");
            //matf32_sub(&temp, &R_jacobi_rotation, &temp);
            //matf32_print(&temp);
        }*/

        // ---------------------------------------------------------------------------
        // matf32_jacobi_svd
        // ---------------------------------------------------------------------------

        if (n > 1)
        {
            float jacobi_svd_time[x];
            float mean_jacobi_svd_time = 0;

            float A_copy_data[MAX_MAT_SIZE];
            matf32_t A_copy;
            matf32_init(&A_copy, A.num_rows, A.num_cols, A_copy_data);
            matf32_submatrix_copy(&A, &A_copy, 0, 0, 0, 0, A.num_rows, A.num_cols);

            float V_trans_data[MAX_MAT_SIZE];
            matf32_t V_trans;
            matf32_init(&V_trans, A.num_rows, A.num_cols, V_trans_data);

            float U_trans_data[MAX_MAT_SIZE];
            matf32_t U_trans;
            matf32_init(&U_trans, A.num_rows, A.num_cols, U_trans_data);

            float UtU_data[MAX_MAT_SIZE];
            matf32_t UtU;
            matf32_init(&UtU, U.num_rows, U.num_cols, UtU_data);

            float VtV_data[MAX_MAT_SIZE];
            matf32_t VtV;
            matf32_init(&VtV, U.num_rows, U.num_cols, VtV_data);

            float US_data[MAX_MAT_SIZE];
            matf32_t US;
            matf32_init(&US, U.num_rows, S.num_cols, US_data);

            float USVt_data[MAX_MAT_SIZE];
            matf32_t USVt;
            matf32_init(&USVt, A.num_rows, A.num_cols, USVt_data);

            //for (uint8_t j = 0; j < x; j++)
            //{
            //    time = clock();
            //    matf32_jacobi_svd(&A, &U, &S, &V);
            //    jacobi_svd_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            //}

            matf32_jacobi_svd(&A, &U, &S, &V);

            //mean_jacobi_svd_time = mean(jacobi_svd_time, x);

            //printf("A:\n");
            //matf32_print(&A_copy);
            //printf("U:\n");
            //matf32_print(&U);
            //printf("S:\n");
            //matf32_print(&S);
            //printf("V:\n");
            //matf32_print(&V);

            matf32_trans(&V, &V_trans);
            matf32_trans(&U, &U_trans);

            matf32_mul(&U_trans, &U, &UtU);
            //printf("UtU:\n");
            //matf32_print(&UtU);

            matf32_mul(&V_trans, &V, &VtV);
            //printf("VtV:\n");
            //matf32_print(&VtV);

            matf32_mul(&U, &S, &US);
            matf32_mul(&US, &V_trans, &USVt);
            //printf("USVt:\n");
            //matf32_print(&USVt);

            bool jacobi_svd_ans = matf32_is_equal(&A_copy, &USVt);
            //printf("jacobi_svd%i\n", n);
            //printf("%.9f\n", mean_jacobi_svd_time);
            printf("one_sided_jacobi,%s\n\n", jacobi_svd_ans?"success":"failure");
        
            // Test Ax = b with SVD: x = V * pinv(S) * U' * b

            float Si_data[MAX_MAT_SIZE];
            matf32_t Si;
            matf32_init(&Si, S.num_rows, S.num_cols, Si_data);
            matf32_zeros(&Si);

            for (uint16_t k = 0; k < n; ++k)
            {
                // For nonzero singular values (sigma), do: 1/sigma
                if (fabs(S.p_data[k*n + k]) > 1E-05)
                {
                    // Save the new value in Si, zero values will already be zero and untouched in Si.
                    Si.p_data[k*n + k] = 1/S.p_data[k*n + k];
                }
            }

            //printf("Si:\n");
            //matf32_print(&Si);

            float b_data[MAX_MAT_SIZE];
            matf32_t b;
            matf32_init(&b, n, 1, b_data);
            matf32_randn(&b,0,1);

            float x_data[MAX_MAT_SIZE];
            matf32_t x;
            matf32_init(&x, A.num_rows, 1, x_data);

            float Utb_data[MAX_MAT_SIZE];
            matf32_t Utb;
            matf32_init(&Utb, U_trans.num_rows, b.num_cols, Utb_data);
            
            matf32_mul(&U_trans, &b, &Utb);

            float SiUtb_data[MAX_MAT_SIZE];
            matf32_t SiUtb;
            matf32_init(&SiUtb, Si.num_rows, b.num_cols, SiUtb_data);

            matf32_mul(&Si, &Utb, &SiUtb);

            matf32_mul(&V, &SiUtb, &x);
            
            //printf("b:\n");
            //matf32_print(&b);

            //printf("x:\n");
            //matf32_print(&x);

            float Ax_data[MAX_MAT_SIZE];
            matf32_t Ax;
            matf32_init(&Ax, b.num_rows, b.num_cols, Ax_data);

            matf32_mul(&A_copy, &x, &Ax);
            //printf("Ax:\n");
            //matf32_print(&Ax);

            bool linsolve_svd_ans = matf32_is_equal(&b, &Ax);
            printf("linsolve_svd,%s\n\n", linsolve_svd_ans?"success":"failure");

            //matf32_sub(&A, &temp, &V_trans);
            //matf32_print(&V_trans);
        }
            
    }
}