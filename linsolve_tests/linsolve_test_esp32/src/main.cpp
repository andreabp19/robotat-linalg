/**
 * @author Andrea Pineda
 * @date created 2 Aug 2025, 17 Oct 2026
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#include "matf32.h"
#include "linsolve.h"
#include "linsolve_matlab_data.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp2_data[MAX_MAT_SIZE];
matf32_t temp2;

float temp3_data[MAX_MAT_SIZE];
matf32_t temp3;

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

float* R_qr_list[9] = {R_qr_data1, R_qr_data2, R_qr_data3, R_qr_data4, R_qr_data5, R_qr_data6, R_qr_data7, R_qr_data8, R_qr_data9};
matf32_t R_qr;

float* R_lu_list[10] = {R_lu_data1, R_lu_data2, R_lu_data3, R_lu_data4, R_lu_data5, R_lu_data6, R_lu_data7, R_lu_data8, R_lu_data9, R_lu_data10};
matf32_t R_lu;

float* R_svd_list[9] = {R_svd_data2, R_svd_data3, R_svd_data4, R_svd_data5, R_svd_data6, R_svd_data7, R_svd_data8, R_svd_data9, R_svd_data10};
matf32_t R_svd;

float cond_A = 0;
float cond_A_L = 0;
float cond_A_U = 0;
float cond_C = 0;
float cond_B = 0;

err_status_t status;

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
            matf32_init(&temp3, n, 1, temp3_data);
            matf32_init(&D, n, n-1, D_list[i-1]);
            matf32_init(&R_qr, n-1, 1, R_qr_list[i-1]);
            matf32_init(&R_svd, n, 1, R_svd_list[i-1]);
        }

        Serial.println();
        Serial.println("--------------------------------------------------");
        Serial.print("n = ");
        Serial.println(n);
        Serial.println("--------------------------------------------------");

        // ---------------------------------------------------------------------------
        // linsolve - Forward Substitution
        // ---------------------------------------------------------------------------

        float fwd_subs_time[x];
        float mean_fwd_subs_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&A_L, &b, &temp);
            fwd_subs_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_fwd_subs");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(fwd_subs_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool fwd_subs_ans = matf32_is_equal(&temp, &R_fwd_subs);
        //Serial.print("linsolve_fwd_subs");
        //Serial.println(n);
        //Serial.println(mean_fwd_subs_time,9);
        //Serial.print("forward-subs , Mean Time (s): ");
        //Serial.print(mean_fwd_subs_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(fwd_subs_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // linsolve - Backward Substitution
        // ---------------------------------------------------------------------------

        float bwd_subs_time[x];
        float mean_bwd_subs_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&A_U, &b, &temp);
            bwd_subs_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_bwd_subs");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(bwd_subs_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();
        
        bool bwd_subs_ans = matf32_is_equal(&temp, &R_bwd_subs);
        //Serial.print("linsolve_bwd_subs");
        //Serial.println(n);
        //Serial.println(mean_bwd_subs_time,9);
        //Serial.print("forward-subs , Mean Time (s): ");
        //Serial.print(mean_bwd_subs_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(bwd_subs_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // linsolve - Cholesky
        // ---------------------------------------------------------------------------
        
        float cholesky_time[x];
        float mean_cholesky_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&C, &b, &temp);
            cholesky_time[j] = (float)((micros() - tic)/1000000.0);
        }
            
        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_cholesky");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(cholesky_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();
        
        bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);
        //Serial.print("linsolve_cholesky");
        //Serial.println(n);
        //Serial.println(mean_cholesky_time,9);
        //Serial.print("cholesky     , Mean Time (s): ");
        //Serial.print(mean_cholesky_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(cholesky_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // linsolve - QR
        // ---------------------------------------------------------------------------
    
        float qr_time[x];
        float mean_qr_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&D, &b, &temp2);
            qr_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_qr");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(qr_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool qr_ans = matf32_is_equal(&temp2, &R_qr);
        //Serial.print("linsolve_qr");
        //Serial.println(n);
        //Serial.println(mean_qr_time,9);
        //Serial.print("linsolve_qr  , Mean Time (s): ");
        //Serial.print(mean_qr_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qr_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // linsolve - LU
        // ---------------------------------------------------------------------------

        float lu_time[x];
        float mean_lu_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&B, &b, &temp);
            lu_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_lu");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(lu_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();
        
        bool lu_ans = matf32_is_equal(&temp, &R_lu);
        //Serial.print("linsolve_lu");
        //Serial.println(n);
        //Serial.println(mean_lu_time,9);
        //Serial.print("lu           , Mean Time (s): ");
        //Serial.print(mean_lu_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(lu_ans?"success":"failure");


        // ---------------------------------------------------------------------------
        // linsolve - SVD Jacobi
        // ---------------------------------------------------------------------------

        if (n > 1)
        {
            float svd_time[x];
            float mean_svd_time = 0;
            bool linsolve_svd_ans = 0;

            for (uint8_t j = 0; j < x; j++)
            {
                tic = micros();
                linsolve_method(&A, &b, &temp3, SVD, SQUARE);
                svd_time[j] = (float)((micros() - tic)/1000000.0);
            }

            // Print time values formatted to a .mat for plotting in matlab
            Serial.print("linsolve_svd");
            Serial.println(n);
            for (uint8_t j = 0; j < x; j++)
            {
                Serial.print(svd_time[j], 9);
                Serial.print(" ");
            }
            Serial.println();

            bool svd_ans = matf32_is_equal(&temp3, &R_svd);
            //Serial.print("linsolve_svd");
            //Serial.println(n);
            //Serial.println(mean_svd_time,9);
            //Serial.print("svd          , Mean Time (s): ");
            //Serial.print(mean_svd_time,9);
            //Serial.print(", 1E-05 Tolerance: ");
            //Serial.println(svd_ans?"success":"failure");
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
