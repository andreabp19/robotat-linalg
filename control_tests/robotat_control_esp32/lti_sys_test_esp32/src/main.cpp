
/**
 * @author Andrea Pineda
 * @date Created 5 Nov 2025, last modified 17 Oct 2026
 * 
 * Testing the lti sys functions in the ESP32
 */


#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "robotat_control.h"
#include "control_lti_sys_test_data.h"

// --------------------------------------------------
// LTI State Space System
// --------------------------------------------------

uint16_t state_dim = 1;

float state_data[MAX_MAT_SIZE];
matf32_t state;

float* A_fwd_eul_list[9] = {A_fwd_eul2, A_fwd_eul3, A_fwd_eul4, A_fwd_eul5, A_fwd_eul6, A_fwd_eul7, A_fwd_eul8, A_fwd_eul9, A_fwd_eul10};
matf32_t A_fwd_eul;
float* B_fwd_eul_list[9] = {B_fwd_eul2, B_fwd_eul3, B_fwd_eul4, B_fwd_eul5, B_fwd_eul6, B_fwd_eul7, B_fwd_eul8, B_fwd_eul9, B_fwd_eul10};
matf32_t B_fwd_eul;

float* A_bwd_eul_list[9] = {A_bwd_eul2, A_bwd_eul3, A_bwd_eul4, A_bwd_eul5, A_bwd_eul6, A_bwd_eul7, A_bwd_eul8, A_bwd_eul9, A_bwd_eul10};
matf32_t A_bwd_eul;
float* B_bwd_eul_list[9] = {B_bwd_eul2, B_bwd_eul3, B_bwd_eul4, B_bwd_eul5, B_bwd_eul6, B_bwd_eul7, B_bwd_eul8, B_bwd_eul9, B_bwd_eul10};
matf32_t B_bwd_eul;

float* A_tustin_list[9] = {A_tustin2, A_tustin3, A_tustin4, A_tustin5, A_tustin6, A_tustin7, A_tustin8, A_tustin9, A_tustin10};
matf32_t A_tustin;
float* B_tustin_list[9] = {B_tustin2, B_tustin3, B_tustin4, B_tustin5, B_tustin6, B_tustin7, B_tustin8, B_tustin9, B_tustin10};
matf32_t B_tustin;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float sample_time = 0.001;

ctr_sys_lti_t sys_fwd_eul;
ctr_sys_lti_t sys_bwd_eul;
ctr_sys_lti_t sys_tustin;

// MATLAB results
float* R_c2d_fwd_eul_list_Ad[9] = {R_c2d_fwd_eul_Ad2, R_c2d_fwd_eul_Ad3, R_c2d_fwd_eul_Ad4, R_c2d_fwd_eul_Ad5, R_c2d_fwd_eul_Ad6, R_c2d_fwd_eul_Ad7, R_c2d_fwd_eul_Ad8, R_c2d_fwd_eul_Ad9, R_c2d_fwd_eul_Ad10};
matf32_t R_c2d_fwd_eul_Ad;
float* R_c2d_fwd_eul_list_Bd[9] = {R_c2d_fwd_eul_Bd2, R_c2d_fwd_eul_Bd3, R_c2d_fwd_eul_Bd4, R_c2d_fwd_eul_Bd5, R_c2d_fwd_eul_Bd6, R_c2d_fwd_eul_Bd7, R_c2d_fwd_eul_Bd8, R_c2d_fwd_eul_Bd9, R_c2d_fwd_eul_Bd10};
matf32_t R_c2d_fwd_eul_Bd;

float* R_c2d_bwd_eul_list_Ad[9] = {R_c2d_bwd_eul_Ad2, R_c2d_bwd_eul_Ad3, R_c2d_bwd_eul_Ad4, R_c2d_bwd_eul_Ad5, R_c2d_bwd_eul_Ad6, R_c2d_bwd_eul_Ad7, R_c2d_bwd_eul_Ad8, R_c2d_bwd_eul_Ad9, R_c2d_bwd_eul_Ad10};
matf32_t R_c2d_bwd_eul_Ad;
float* R_c2d_bwd_eul_list_Bd[9] = {R_c2d_bwd_eul_Bd2, R_c2d_bwd_eul_Bd3, R_c2d_bwd_eul_Bd4, R_c2d_bwd_eul_Bd5, R_c2d_bwd_eul_Bd6, R_c2d_bwd_eul_Bd7, R_c2d_bwd_eul_Bd8, R_c2d_bwd_eul_Bd9, R_c2d_bwd_eul_Bd10};
matf32_t R_c2d_bwd_eul_Bd;

float* R_c2d_tustin_list_Ad[9] = {R_c2d_tustin_Ad2, R_c2d_tustin_Ad3, R_c2d_tustin_Ad4, R_c2d_tustin_Ad5, R_c2d_tustin_Ad6, R_c2d_tustin_Ad7, R_c2d_tustin_Ad8, R_c2d_tustin_Ad9, R_c2d_tustin_Ad10};
matf32_t R_c2d_tustin_Ad;
float* R_c2d_tustin_list_Bd[9] = {R_c2d_tustin_Bd2, R_c2d_tustin_Bd3, R_c2d_tustin_Bd4, R_c2d_tustin_Bd5, R_c2d_tustin_Bd6, R_c2d_tustin_Bd7, R_c2d_tustin_Bd8, R_c2d_tustin_Bd9, R_c2d_tustin_Bd10};
matf32_t R_c2d_tustin_Bd;

// Variables for measuring time
unsigned long tic = 0;
float toc = 0;

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
    for (uint16_t i = 0; i < 9; ++i)
    {
        state_dim += 1;

        Serial.println();
        Serial.print("----------------------------------------------------------------------");
        Serial.print("n = ");
        Serial.println(state_dim);
        Serial.print("----------------------------------------------------------------------");
        Serial.println();

        matf32_init(&state, state_dim, 1, state_data);
        matf32_init(&A_fwd_eul, state_dim, state_dim, A_fwd_eul_list[i]);
        matf32_init(&B_fwd_eul, state_dim, 1, B_fwd_eul_list[i]);

        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);
        
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);
    
        matf32_init(&C, 1, state_dim, C_data);
        matf32_init(&D, C.num_rows, B_tustin.num_cols, D_data);

        ctr_sys_lti_init(&sys_fwd_eul, &state, &A_tustin, &B_tustin, &C, &D, sample_time);
        ctr_sys_lti_init(&sys_bwd_eul, &state, &A_bwd_eul, &B_bwd_eul, &C, &D, sample_time);
        ctr_sys_lti_init(&sys_tustin, &state, &A_tustin, &B_tustin, &C, &D, sample_time);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_fwd_eul_Ad, A_fwd_eul.num_rows, A_fwd_eul.num_cols, R_c2d_fwd_eul_list_Ad[i]);
        matf32_init(&R_c2d_fwd_eul_Bd, B_fwd_eul.num_rows, B_fwd_eul.num_cols, R_c2d_fwd_eul_list_Bd[i]);
        matf32_init(&R_c2d_bwd_eul_Ad, A_bwd_eul.num_rows, A_bwd_eul.num_cols, R_c2d_bwd_eul_list_Ad[i]);
        matf32_init(&R_c2d_bwd_eul_Bd, B_bwd_eul.num_rows, B_bwd_eul.num_cols, R_c2d_bwd_eul_list_Bd[i]);
        matf32_init(&R_c2d_tustin_Ad, A_tustin.num_rows, A_tustin.num_cols, R_c2d_tustin_list_Ad[i]);
        matf32_init(&R_c2d_tustin_Bd, B_tustin.num_rows, B_tustin.num_cols, R_c2d_tustin_list_Bd[i]);

        // ---------------------------------------------------------------------------
        // ctr_c2d - Forward Euler
        // ---------------------------------------------------------------------------

        //Serial.println();
        //Serial.println("\n--------------------------------------------------\n");
        //Serial.print("c2d - Forward Euler\n");
        //Serial.println("--------------------------------------------------\n");
        //Serial.println();

        float c2d_fwd_eul_time[K];
        float c2d_fwd_eul_mean_time = 0;

        // As ctr_c2d overwrites A and B, to test it 100 times with the same values, it's either
        // to reset the value to the original each loop, or test 100 random matrices and not worry about the result vs matlab when measuring time.
        // Either way, sys_lti.is_continuous must be set to true each cycle in order to be able to operate with ctr_c2d (outside the time measurement).
        for (uint8_t i = 0; i < K; i++)
        {
            matf32_randn(&A_fwd_eul,0,1);
            matf32_randn(&B_fwd_eul,0,1);
            sys_fwd_eul.is_continuous = true;
            tic = micros();
            ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);
            c2d_fwd_eul_time[i] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("c2d_fwd_eul");
        Serial.println(state_dim);
        for (uint8_t j = 0; j < K; j++)
        {
            Serial.print(c2d_fwd_eul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        // Uncomment this section and comment the loop to test a single operation to compare results with matlab
        sys_fwd_eul.is_continuous = true;
        ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);

        //c2d_fwd_eul_mean_time = mean(c2d_fwd_eul_time, K);
        //Serial.print("c2d,fwd_euler");
        //Serial.println(state_dim);
        //Serial.println(c2d_fwd_eul_mean_time,9);

        bool c2d_fwd_eul_Ad_ans = matf32_is_equal(&A_fwd_eul, &R_c2d_fwd_eul_Ad);
        bool c2d_fwd_eul_Bd_ans = matf32_is_equal(&B_fwd_eul, &R_c2d_fwd_eul_Bd);
        float cond_A_fwd_eul = 0;
        float cond_B_fwd_eul = 0;
        matf32_cond(&A_fwd_eul, &cond_A_fwd_eul);
        matf32_cond(&B_fwd_eul, &cond_B_fwd_eul);
        //Serial.print("Condition number A: ");
        //Serial.println(cond_A_fwd_eul);
        //Serial.print("Condition number B: ");
        //Serial.println(cond_B_fwd_eul);
        //Serial.print("1E-05 Tolerance A: ");
        //Serial.println(c2d_fwd_eul_Ad_ans?"True":"False");
        //Serial.print("1E-05 Tolerance B: ");
        //Serial.println(c2d_fwd_eul_Bd_ans?"True":"False");

        //if (!c2d_fwd_eul_Ad_ans || !c2d_fwd_eul_Bd_ans)
        //{
        //    Serial.print("Ad result difference:\n");
        //    matf32_sub(&A_fwd_eul, &R_c2d_fwd_eul_Ad, &A_fwd_eul);
        //    matf32_print(&A_fwd_eul);
        //    Serial.print("Bd result difference:\n");
        //    matf32_sub(&B_fwd_eul, &R_c2d_fwd_eul_Bd, &B_fwd_eul);
        //    matf32_print(&B_fwd_eul);
        //}
            
        // ---------------------------------------------------------------------------
        // ctr_c2d - Backward Euler
        // ---------------------------------------------------------------------------

        //Serial.println();
        //Serial.println("\n--------------------------------------------------\n");
        //Serial.print("c2d - Backward Euler\n");
        //Serial.println("--------------------------------------------------\n");
        //Serial.println();

        // Initialize again A and B
        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_bwd_eul_Ad, A_bwd_eul.num_rows, A_bwd_eul.num_cols, R_c2d_bwd_eul_list_Ad[i]);
        matf32_init(&R_c2d_bwd_eul_Bd, B_bwd_eul.num_rows, B_bwd_eul.num_cols, R_c2d_bwd_eul_list_Bd[i]);

        float c2d_bwd_eul_time[K];
        float c2d_bwd_eul_mean_time = 0;

        for (uint8_t i = 0; i < K; i++)
        {
            matf32_randn(&A_bwd_eul,0,1);
            matf32_randn(&B_bwd_eul,0,1);
            sys_bwd_eul.is_continuous = true;
            tic = micros();
            ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);
            c2d_bwd_eul_time[i] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("c2d_bwd_eul");
        Serial.println(state_dim);
        for (uint8_t j = 0; j < K; j++)
        {
            Serial.print(c2d_bwd_eul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        sys_bwd_eul.is_continuous = true;
        ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);

        //c2d_bwd_eul_mean_time = mean(c2d_bwd_eul_time, K);
        //Serial.print("c2d,bwd_euler");
        //Serial.println(state_dim);
        //Serial.println(c2d_bwd_eul_mean_time,9);

        bool c2d_bwd_eul_Ad_ans = matf32_is_equal(&A_bwd_eul, &R_c2d_bwd_eul_Ad);
        bool c2d_bwd_eul_Bd_ans = matf32_is_equal(&B_bwd_eul, &R_c2d_bwd_eul_Bd);
        float cond_A_bwd_eul = 0;
        float cond_B_bwd_eul = 0;
        matf32_cond(&A_bwd_eul, &cond_A_bwd_eul);
        matf32_cond(&B_bwd_eul, &cond_B_bwd_eul);
        //Serial.print("Condition number A: ");
        //Serial.println(cond_A_bwd_eul);
        //Serial.print("Condition number B: ");
        //Serial.println(cond_B_bwd_eul);
        //Serial.print("1E-05 Tolerance A: ");
        //Serial.println(c2d_bwd_eul_Ad_ans?"True":"False");
        //Serial.print("1E-05 Tolerance B: ");
        //Serial.println(c2d_bwd_eul_Bd_ans?"True":"False");

        //if (!c2d_bwd_eul_Ad_ans || !c2d_bwd_eul_Bd_ans)
        //{
        //    Serial.print("\nAd result difference:\n");
        //    matf32_sub(&A_bwd_eul, &R_c2d_bwd_eul_Ad, &A_bwd_eul);
        //    matf32_print(&A_bwd_eul);
        //    Serial.print("Bd result difference:\n");
        //    matf32_sub(&B_bwd_eul, &R_c2d_bwd_eul_Bd, &B_bwd_eul);
        //    matf32_print(&B_bwd_eul);
        //}

        // ---------------------------------------------------------------------------
        // ctr_c2d - Tustin
        // ---------------------------------------------------------------------------

        //Serial.println();
        //Serial.println("\n--------------------------------------------------\n");
        //Serial.print("c2d - Tustin\n");
        //Serial.println("--------------------------------------------------\n");
        //Serial.println();

        // Initialize again A and B
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_tustin_Ad, A_tustin.num_rows, A_tustin.num_cols, R_c2d_tustin_list_Ad[i]);
        matf32_init(&R_c2d_tustin_Bd, B_tustin.num_rows, B_tustin.num_cols, R_c2d_tustin_list_Bd[i]);

        float c2d_tustin_time[K];
        float c2d_tustin_mean_time = 0;

        for (uint8_t i = 0; i < K; i++)
        {
            matf32_randn(&A_tustin,0,1);
            matf32_randn(&B_tustin,0,1);
            sys_tustin.is_continuous = true;
            tic = micros();
            ctr_c2d(&sys_tustin, sample_time, TUSTIN);
            c2d_tustin_time[i] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("c2d_tustin");
        Serial.println(state_dim);
        for (uint8_t j = 0; j < K; j++)
        {
            Serial.print(c2d_tustin_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        sys_tustin.is_continuous = true;
        ctr_c2d(&sys_tustin, sample_time, TUSTIN);

        //c2d_tustin_mean_time = mean(c2d_tustin_time, K);
        //Serial.print("c2d,tustin");
        //Serial.println(state_dim);
        //Serial.println(c2d_tustin_mean_time,9);

        bool c2d_tustin_Ad_ans = matf32_is_equal(&A_tustin, &R_c2d_tustin_Ad);
        bool c2d_tustin_Bd_ans = matf32_is_equal(&B_tustin, &R_c2d_tustin_Bd);
        float cond_A_tustin = 0;
        float cond_B_tustin = 0;
        matf32_cond(&A_tustin, &cond_A_tustin);
        matf32_cond(&B_tustin, &cond_B_tustin);
        //Serial.print("Condition number A: ");
        //Serial.println(cond_A_tustin);
        //Serial.print("Condition number B: ");
        //Serial.println(cond_B_tustin);
        //Serial.print("1E-05 Tolerance A: ");
        //Serial.println(c2d_tustin_Ad_ans?"True":"False");
        //Serial.print("1E-05 Tolerance B: ");
        //Serial.println(c2d_tustin_Bd_ans?"True":"False");
    
        //if (!c2d_tustin_Ad_ans || !c2d_tustin_Bd_ans)
        //{
        //    Serial.print("Ad result difference:\n");
        //    matf32_sub(&A_tustin, &R_c2d_tustin_Ad, &A_tustin);
        //    matf32_print(&A_tustin);
        //    Serial.print("Bd result difference:\n");
        //    matf32_sub(&B_tustin, &R_c2d_tustin_Bd, &B_tustin);
        //    matf32_print(&B_tustin);
        //}
    }
}

void loop()
{

}
