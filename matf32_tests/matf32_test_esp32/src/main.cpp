
/**
 * @author Andrea Pineda
 * @date created 19 Jul 2025, last modified 17 Oct 2026
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "matf32.h"
#include "matf32_matlab_data.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------

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

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

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

float* R_exp_list[10] = {R_exp1, R_exp2, R_exp3, R_exp4, R_exp5, R_exp6, R_exp7, R_exp8, R_exp9, R_exp10};
matf32_t R_exp;

float* R_pinv_basic_list[10] = {R_pinv_basic1, R_pinv_basic2, R_pinv_basic3, R_pinv_basic4, R_pinv_basic5, R_pinv_basic6, R_pinv_basic7, R_pinv_basic8, R_pinv_basic9, R_pinv_basic10};
matf32_t R_pinv_basic;

float* R_pinv_svd_list[10] = {R_pinv_svd1, R_pinv_svd2, R_pinv_svd3, R_pinv_svd4, R_pinv_svd5, R_pinv_svd6, R_pinv_svd7, R_pinv_svd8, R_pinv_svd9, R_pinv_svd10};
matf32_t R_pinv_svd;


// Matrix Factorizations Matlab Results
float* R_lu_list[10] = {R_lu1, R_lu2, R_lu3, R_lu4, R_lu5, R_lu6, R_lu7, R_lu8, R_lu9, R_lu10};
matf32_t R_lu;

float* R_cholesky_list[10] = {R_cholesky1, R_cholesky2, R_cholesky3, R_cholesky4, R_cholesky5, R_cholesky6, R_cholesky7, R_cholesky8, R_cholesky9, R_cholesky10};
matf32_t R_cholesky;

float* R_qr_list[10] = {R_qr1, R_qr2, R_qr3, R_qr4, R_qr5, R_qr6, R_qr7, R_qr8, R_qr9, R_qr10};
matf32_t R_qr;

float* R_jacobi_svd_list[9] = {R_jacobi_svd2, R_jacobi_svd3, R_jacobi_svd4, R_jacobi_svd5, R_jacobi_svd6, R_jacobi_svd7, R_jacobi_svd8, R_jacobi_svd9, R_jacobi_svd10};
matf32_t R_jacobi_svd;

float A_QR_data[MAX_MAT_SIZE];
matf32_t A_QR;

float PA_data[MAX_MAT_SIZE];
matf32_t PA;

// Condition numbers for all input matrices used
float cond_A = 0;
float cond_B = 0;
float cond_C = 0;
float cond_D = 0;

// Variables for measuring time
unsigned long tic = 0;
float toc = 0;

void print_matrix(float* matrix, uint16_t rows, uint16_t cols);

void setup()
{
    // ---------------------------------------------------------------------------
    // Serial config
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // Testing
    // ---------------------------------------------------------------------------

    Serial.println();
    Serial.println();

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&temp, n, n, temp_data);
        matf32_init(&temp_L, n, n, temp_L_data);
        matf32_init(&temp_U, n, n, temp_U_data);
        matf32_init(&temp_Q, n, n, temp_Q_data);
        matf32_init(&temp_R, n, n, temp_R_data);
        matf32_init(&A, n, n, A_list[i]);
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
        matf32_init(&R_exp, n, n, R_exp_list[i]);
        matf32_init(&R_pinv_basic, n, n, R_pinv_basic_list[i]);
        matf32_init(&R_pinv_svd, n, n, R_pinv_svd_list[i]);
        matf32_init(&R_lu, n, n, R_lu_list[i]);
        matf32_init(&R_cholesky, n, n, R_cholesky_list[i]);
        matf32_init(&R_qr, n, n, R_qr_list[i]);
        matf32_init(&A_QR, A.num_rows, A.num_cols, A_QR_data);
        matf32_init(&PA, A.num_rows, A.num_cols, PA_data);
        
        if (n > 1)
        {
            // SVD matrices
            matf32_init(&U, A.num_rows, A.num_cols, U_data);
            matf32_init(&S, A.num_rows, A.num_cols, S_data);
            matf32_init(&V, A.num_rows, A.num_cols, V_data);
            matf32_init(&R_jacobi_svd, n, n, R_jacobi_svd_list[i]);
        }

        const matf32_t* mat_array[3] = {&A, &A, &A};
        
        Serial.println();
        Serial.println("---------------------------------------------------------------------------");
        Serial.print("n = ");
        Serial.println(n);
        Serial.println("---------------------------------------------------------------------------");

        matf32_cond(&A, &cond_A);
        matf32_cond(&C, &cond_C);

        // ---------------------------------------------------------------------------
        // matf32_add
        // ---------------------------------------------------------------------------

        /*float add_time[x];
        float mean_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_add(&A, &A, &temp);
            add_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_add");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(add_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool add_ans = matf32_is_equal(&temp, &R_add);*/
        //Serial.print("matf32_add");
        //Serial.println(n);
        //Serial.println(mean_add_time,9);
        //Serial.print("matf32_add           , Mean Time (s): ");
        //Serial.print(mean_add_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(add_ans?"True":"False");
        

        // ---------------------------------------------------------------------------
        // matf32_sub
        // ---------------------------------------------------------------------------

        /*float sub_time[x];
        float mean_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_sub(&A, &A, &temp);
            sub_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_sub");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(sub_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool sub_ans = matf32_is_equal(&temp, &R_sub);*/
        //Serial.print("matf32_sub");
        //Serial.println(n);
        //Serial.println(mean_sub_time,9);
        //Serial.print("matf32_sub           , Mean Time (s): ");
        //Serial.print(mean_sub_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(sub_ans?"True":"False");
        

        // ---------------------------------------------------------------------------
        // matf32_scale
        // ---------------------------------------------------------------------------

        /*float scale_time[x];
        float mean_scale_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_scale(&A, c, &temp);
            scale_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_scale");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(scale_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool scale_ans = matf32_is_equal(&temp, &R_scale);*/
        //Serial.print("matf32_scale");
        //Serial.println(n);
        //Serial.println(mean_scale_time,9);
        //Serial.print("matf32_scale         , Mean Time (s): ");
        //Serial.print(mean_scale_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(scale_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_trans
        // ---------------------------------------------------------------------------

        /*float trans_time[x];
        float mean_trans_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_trans(&A, &temp);
            trans_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_trans");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(trans_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool trans_ans = matf32_is_equal(&temp, &R_trans);*/
        //Serial.print("matf32_trans");
        //Serial.println(n);
        //Serial.println(mean_trans_time,9);
        //Serial.print("matf32_trans         , Mean Time (s): ");
        //Serial.print(mean_trans_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(trans_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_mul
        // ---------------------------------------------------------------------------

        /*float mul_time[x];
        float mean_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_mul(&A, &A, &temp);
            mul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_mul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(mul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool mul_ans = matf32_is_equal(&temp, &R_mul);*/
        //Serial.print("matf32_mul");
        //Serial.println(n);
        //Serial.println(mean_mul_time,9);
        //Serial.print("matf32_mul           , Mean Time (s): ");
        //Serial.print(mean_mul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(mul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_inv
        // ---------------------------------------------------------------------------

        /*float inv_time[x];
        float mean_inv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_inv(&A, &temp);
            inv_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_inv");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(inv_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool inv_ans = matf32_is_equal(&temp, &R_inv);*/
        //Serial.print("matf32_inv");
        //Serial.println(n);
        //Serial.println(mean_inv_time,9);
        //Serial.print("matf32_inv           , Mean Time (s): ");
        //Serial.print(mean_inv_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(inv_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_dot
        // ---------------------------------------------------------------------------

        /*float dot_time[x];
        float mean_dot_time = 0;
        float dot_temp_data = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_dot(&rowvec, &colvec, &dot_temp_data);
            dot_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_dot");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(dot_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_dot");
        //Serial.println(n);
        //Serial.println(mean_dot_time,9);
        //Serial.print("matf32_dot           , Mean Time (s): ");
        //Serial.print(mean_dot_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(dot_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecposmul
        // ---------------------------------------------------------------------------
        
        /*float vecposmul_time[x];
        float mean_vecposmul_time = 0;
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
            vecposmul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecposmul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecposmul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool vecposmul_ans = matf32_is_equal(&vecposmul_temp, &R_vecposmul);*/
        //Serial.print("matf32_vecposmul");
        //Serial.println(n);
        //Serial.println(mean_vecposmul_time,9);
        //Serial.print("matf32_vecposmul     , Mean Time (s): ");
        //Serial.print(mean_vecposmul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecposmul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecpremul
        // ---------------------------------------------------------------------------

        /*float vecpremul_time[x];
        float mean_vecpremul_time = 0;
        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
            vecpremul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecpremul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecpremul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool vecpremul_ans = matf32_is_equal(&vecpremul_temp, &R_vecpremul);*/
        //Serial.print("matf32_vecpremul");
        //Serial.println(n);
        //Serial.println(mean_vecpremul_time,9);
        //Serial.print("matf32_vecpremul     , Mean Time (s): ");
        //Serial.print(mean_vecpremul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecpremul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecmul_col_row
        // ---------------------------------------------------------------------------
        
        /*float vecmul_time[x];
        float mean_vecmul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
            vecmul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecmul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecmul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool vecmul_ans = matf32_is_equal(&temp, &R_vecmul);*/
        //Serial.print("matf32_vecmul");
        //Serial.println(n);
        //Serial.println(mean_vecmul_time,9);
        //Serial.print("matf32_vecmul_col_row, Mean Time (s): ");
        //Serial.print(mean_vecmul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecmul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_add
        // ---------------------------------------------------------------------------

        /*float arr_add_time[x];
        float mean_arr_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_add(mat_array, 3, &temp);
            arr_add_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_add");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_add_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool arr_add_ans = matf32_is_equal(&temp, &R_arr_add);*/
        //Serial.print("matf32_arr_add");
        //Serial.println(n);
        //Serial.println(mean_arr_add_time,9);
        //Serial.print("matf32_arr_add       , Mean Time (s): ");
        //Serial.print(mean_arr_add_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_add_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_sub
        // ---------------------------------------------------------------------------

        /*float arr_sub_time[x];
        float mean_arr_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_sub(mat_array, 3, &temp);
            arr_sub_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_sub");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_sub_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool arr_sub_ans = matf32_is_equal(&temp, &R_arr_sub);*/
        //Serial.print("matf32_arr_sub");
        //Serial.println(n);
        //Serial.println(mean_arr_sub_time,9);
        //Serial.print("matf32_arr_sub       , Mean Time (s): ");
        //Serial.print(mean_arr_sub_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_sub_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_mul
        // ---------------------------------------------------------------------------

        /*float arr_mul_time[x];
        float mean_arr_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_mul(mat_array, 3, &temp);
            arr_mul_time[j] = (float)((micros() - tic)/1000000.0);   
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_mul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_mul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool arr_mul_ans = matf32_is_equal(&temp, &R_arr_mul);*/
        //Serial.print("matf32_arr_mul");
        //Serial.println(n);
        //Serial.println(mean_arr_mul_time,9);
        //Serial.print("matf32_arr_mul       , Mean Time (s): ");
        //Serial.print(mean_arr_mul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_mul_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_exp
        // ---------------------------------------------------------------------------

        /*float exp_time[x];
        float mean_exp_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_exp(&A, &temp, 4);
            exp_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_exp");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(exp_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool exp_ans = matf32_is_equal(&temp, &R_exp);*/
        //Serial.print("matf32_exp");
        //Serial.println(n);
        //Serial.println(mean_exp_time,9);
        //Serial.print("matf32_exp           , Mean Time (s): ");
        //Serial.print(mean_exp_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(exp_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_pinv
        // ---------------------------------------------------------------------------

        // ------------------------- BASIC PINV -------------------------
        /*float pinv_basic_time[x];
        float mean_pinv_basic_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_pinv(&A, &temp, BASIC_PINV);
            pinv_basic_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_pinv_basic");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(pinv_basic_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool pinv_basic_ans = matf32_is_equal(&temp, &R_pinv_basic);*/
        //Serial.print("matf32_pinv_basic");
        //Serial.println(n);
        //Serial.println(mean_pinv_basic_time,9);
        //Serial.print("matf32_pinv: Basic   , Mean Time (s): ");
        //Serial.print(mean_pinv_basic_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(pinv_basic_ans?"True":"False");

        // ------------------------- SVD PINV -------------------------
        /*float pinv_svd_time[x];
        float mean_pinv_svd_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_pinv(&A, &temp, SVD_PINV);
            pinv_svd_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_pinv_svd");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(pinv_svd_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool pinv_svd_ans = matf32_is_equal(&temp, &R_pinv_svd);*/
        //Serial.print("matf32_pinv_svd");
        //Serial.println(n);
        //Serial.println(mean_pinv_svd_time,9);
        //Serial.print("matf32_pinv: SVD     , Mean Time (s): ");
        //Serial.print(mean_pinv_svd_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(pinv_svd_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_lu
        // ---------------------------------------------------------------------------
        
        float lu_time[x];
        float mean_lu_time = 0;
        
        uint16_t p_index[n];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_lu(&A, &temp_L, &temp_U, p_index);
            lu_time[j] = (float)((micros() - tic)/1000000.0);
        }
        matf32_mul(&temp_L, &temp_U, &PA);

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_lu");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(lu_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool lu_ans = matf32_is_equal(&PA, &R_lu);
        //Serial.print("matf32_lu");
        //Serial.println(n);
        //Serial.println(mean_lu_time,9);
        //Serial.print("matf32_lu            , Mean Time (s): ");
        //Serial.print(mean_lu_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(lu_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_cholesky
        // ---------------------------------------------------------------------------

        float cholesky_time[x];
        float mean_cholesky_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_cholesky(&C, &temp);
            cholesky_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_cholesky");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(cholesky_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);
        //Serial.print("matf32_cholesky");
        //Serial.println(n);
        //Serial.println(mean_cholesky_time,9);
        //Serial.print("matf32_cholesky      , Mean Time (s): ");
        //Serial.print(mean_cholesky_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(cholesky_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_qr
        // ---------------------------------------------------------------------------

        float qr_time[x];
        float mean_qr_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_qr(&A, &temp_Q, &temp_R);
            qr_time[j] = (float)((micros() - tic)/1000000.0);
        }
        matf32_mul(&temp_Q, &temp_R, &A_QR);

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_qr");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(qr_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool qr_ans = matf32_is_equal(&A_QR, &R_qr);
        //Serial.print("matf32_qr");
        //Serial.println(n);
        //Serial.println(mean_qr_time,9);
        //Serial.print("matf32_qr            , Mean Time (s): ");
        //Serial.print(mean_qr_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qr_ans?"True":"False");
        

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

            float Vt_data[MAX_MAT_SIZE];
            matf32_t Vt;
            matf32_init(&Vt, V.num_cols, V.num_rows, Vt_data);

            float US_data[MAX_MAT_SIZE];
            matf32_t US;
            matf32_init(&US, U.num_rows, S.num_cols, US_data);

            float USVt_data[MAX_MAT_SIZE];
            matf32_t USVt;
            matf32_init(&USVt, U.num_rows, Vt.num_cols, USVt_data);

            for (uint8_t j = 0; j < x; j++)
            {
                tic = micros();
                matf32_jacobi_svd(&A_copy, &U, &S, &V);
                matf32_copy(&A, &A_copy); // Reset A_copy = A, because it's modified in matf32_jacobi_svd
                jacobi_svd_time[j] = (float)((micros() - tic)/1000000.0);
            }
            matf32_trans(&V, &Vt);
            matf32_mul(&U, &S, &US);
            matf32_mul(&US, &Vt, &USVt);

            // Print time values formatted to a .mat for plotting in matlab
            Serial.print("matf32_jacobi_svd");
            Serial.println(n);
            for (uint8_t j = 0; j < x; j++)
            {
                Serial.print(jacobi_svd_time[j], 9);
                Serial.print(" ");
            }
            Serial.println();

            bool jacobi_svd_ans = matf32_is_equal(&USVt, &A);
            //Serial.print("matf32_jacobi_svd");
            //Serial.println(n);
            //Serial.println(mean_jacobi_svd_time,9);
            //Serial.print("matf32_jacobi_svd    , Mean Time (s): ");
            //Serial.print(mean_jacobi_svd_time,9);
            //Serial.print(", 1E-05 Tolerance: ");
            //Serial.println(jacobi_svd_ans?"True":"False");
        }
    }
}

void loop()
{
    
}

void print_matrix(float* matrix, uint16_t rows, uint16_t cols)
{
  for (uint16_t i = 0; i < rows; ++i)
  {
    for (uint16_t j = 0; j < cols; ++j)
    {
      Serial.print(matrix[i + j*rows],9);
      Serial.print("  ");
    }
    Serial.println();
  }
  Serial.println();
}
