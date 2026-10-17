
/**
 * @author Andrea Pineda
 * @date Created 27 Oct 2025, Last Modified 17 Oct 2026
 * 
 * To test linsolve functions in the Arduino MEGA
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

//float temp_data[MAX_MAT_SIZE];
//matf32_t temp;

//float temp2_data[MAX_MAT_SIZE];
//matf32_t temp2;

float temp3_data[MAX_MAT_SIZE];
matf32_t temp3;

float* b_list[1] = {b_data10};
matf32_t b;

float* A_list[1] = {A_data10};
matf32_t A;

//float* A_L_list[1] = {A_L_data10};
//matf32_t A_L;

//float* A_U_list[1] = {A_U_data10};
//matf32_t A_U;

//float* B_list[1] = {B_data10};
//matf32_t B;

//float* C_list[1] = {C_data10};
//matf32_t C;

//float* D_list[1] = {D_data9};
//matf32_t D;

err_status_t status;

// Variables for measuring time
unsigned long tic = 0;

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

    for (uint8_t i = 0; i < 1; i++)
    {
        //uint8_t n = i + 1;  // matrix dimension to use

        uint8_t n = 10;

        //matf32_init(&temp, n, 1, temp_data);
        matf32_init(&b, n, 1, b_list[i]);
        matf32_init(&A, n, n, A_list[i]);
        //matf32_init(&A_L, n, n, A_L_list[i]);
        //matf32_init(&A_U, n, n, A_U_list[i]);
        //matf32_init(&C, n, n, C_list[i]);
        //matf32_init(&B, n, n, B_list[i]);

        if (n > 1)
        {
            //matf32_init(&temp2, n-1, 1, temp2_data);
            matf32_init(&temp3, n, 1, temp3_data);
            //matf32_init(&D, n, n-1, D_list[i-1]);
        }

        //Serial.println();
        //Serial.println("--------------------------------------------------");
        //Serial.print("n = ");
        //Serial.println(n);
        //Serial.println("--------------------------------------------------");

        // ---------------------------------------------------------------------------
        // linsolve - Forward Substitution
        // ---------------------------------------------------------------------------

        /*float fwd_subs_time[x];

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
        Serial.println();*/

        //bool fwd_subs_ans = matf32_is_equal(&temp, &R_fwd_subs);
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

        /*float bwd_subs_time[x];
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
        Serial.println();*/
        
        //bool bwd_subs_ans = matf32_is_equal(&temp, &R_bwd_subs);*/
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
        
        /*float cholesky_time[x];
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
        Serial.println();*/
        
        //bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);*/
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
    
        /*float qr_time[x];
        float mean_qr_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            linsolve(&D, &b, &temp2);
            qr_time[j] = (float)((micros() - tic)/1000000.0);
            Serial.println(j+1);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("linsolve_qr");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(qr_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //bool qr_ans = matf32_is_equal(&temp2, &R_qr);*/
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

        /*float lu_time[x];

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
        Serial.println();*/
        
        //bool lu_ans = matf32_is_equal(&temp, &R_lu);*/
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

            for (uint8_t j = 0; j < x; j++)
            {
                tic = micros();
                linsolve_method(&A, &b, &temp3, SVD, SQUARE);
                svd_time[j] = (float)((micros() - tic)/1000000.0);
                Serial.println(j+1);
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

            //bool svd_ans = matf32_is_equal(&temp3, &R_svd);
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
