/**
 * @author Andrea Pineda
 * @date Created 7 Nov 2025, last modified 17 Oct 2026
 * 
 * Testing the kalman filter functions in the STM NUCLEO F446RE
 */

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "robotat_control.h"
#include "control_kalman_filter_test_data.h"

// ---------------------------------------------------------------------------
// Variables
// ---------------------------------------------------------------------------

float A_data[1] = {1};
matf32_t A;

float B_data[1] = {1};
matf32_t B;

float C_data[1] = {1};
matf32_t C;

float D_data[1] = {0};
matf32_t D;

float Qw_data[1] = {0};
matf32_t Qw;

float Qv_data[1] = {1};
matf32_t Qv;

float u_k_data[MAX_MAT_SIZE];
matf32_t u_k;

float x_k_data[MAX_MAT_SIZE];
matf32_t x_k;

float y_k_data[MAX_MAT_SIZE];
matf32_t y_k;

float xhat_data[MAX_MAT_SIZE];
matf32_t xhat;

float P_data[1] = {1};
matf32_t P;

float F_data[1] = {1};
matf32_t F;

float w_data[MAX_MAT_SIZE];
matf32_t w;

float v_data[MAX_MAT_SIZE];
matf32_t v;

#define K (100)

float sample_time = 0.1;

float xhat_history[K];
float x_history[K];
float u_history[K];
float y_history[K];

ctr_sys_lti_t sys;
ctr_kalman_t kf;

// Variables for measuring time
unsigned long tic = 0;
float toc = 0;

// ---------------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------------

//void print_matrix(float* matrix, uint16_t rows, uint16_t cols);

// ---------------------------------------------------------------------------
// Main Code
// ---------------------------------------------------------------------------
void setup()
{
    err_status_t status;

    // ---------------------------------------------------------------------------
    // Serial config
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    Serial.print("CPU frequency: ");
    Serial.print(F_CPU / 1000000UL);
    Serial.println(" MHz");

    // ---------------------------------------------------------------------------
    // Testing
    // ---------------------------------------------------------------------------
    matf32_init(&A, 1, 1, A_data);
    matf32_init(&B, 1, 1, B_data);
    matf32_init(&C, 1, 1, C_data);
    matf32_init(&D, 1, 1, D_data);
    matf32_init(&xhat, 1, 1, xhat_data);
    matf32_init(&x_k, 1, 1, x_k_data);
    matf32_init(&u_k, 1, 1, u_k_data);
    matf32_init(&y_k, 1, 1, y_k_data);
    matf32_init(&F, 1, 1, F_data);
    matf32_init(&P, 1, 1, P_data);
    matf32_init(&Qw, 1, 1, Qw_data);
    matf32_init(&Qv, 1, 1, Qv_data);
    matf32_init(&w, 1, 1, w_data);
    matf32_init(&v, 1, 1, v_data);

    // Initial values
    matf32_set(&x_k, 1, 1, 1);
    matf32_set(&u_k, 1, 1, 0);
    matf32_mul(&C, &x_k, &y_k);
    matf32_zeros(&xhat);

    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, sample_time);
    ctr_kalman_init(&kf, &sys, &F, &Qw, &Qv, &xhat, &P);

    float Ax_data[MAX_MAT_SIZE];
    matf32_t Ax;
    matf32_init(&Ax, A.num_rows, x_k.num_cols, Ax_data);

    float Bu_data[MAX_MAT_SIZE];
    matf32_t Bu;
    matf32_init(&Bu, B.num_rows, u_k.num_cols, Bu_data);

    float Fw_data[MAX_MAT_SIZE];
    matf32_t Fw;
    matf32_init(&Fw, F.num_rows, w.num_cols, Fw_data);

    // Save initial values
    xhat_history[0] = xhat.p_data[0];
    x_history[0] = x_k.p_data[0];
    u_history[0] = u_k.p_data[0];
    y_history[0] = y_k.p_data[0];

    float kalman_time[K];
    float kalman_mean_time = 0;
    for (uint16_t k = 1; k < K; ++k)
    {
        //matf32_set(&u_k, 1, 1, -1); // u = -1

        // u = -0.1*xhat
        matf32_copy(&xhat, &u_k); // u = xhat
        matf32_scale(&u_k, -0.1, &u_k); // u = -0.1*x_hat

        // Generate noise: in this case, assign the value for this iteration from the .h
        w.p_data[0] = w_data100[k];
        v.p_data[0] = v_data100[k];

        matf32_mul(&A, &x_k, &Ax); // A*x
        matf32_mul(&B, &u_k, &Bu); // B*u
        matf32_mul(&F, &w, &Fw);
        matf32_add(&Ax, &Bu, &x_k); // x = A*x + B*u
        matf32_add(&x_k, &Fw, &x_k); // x = A*x + B*u + F*w

        matf32_mul(&C, &x_k, &y_k); // y = C*x
        matf32_add(&y_k, &v, &y_k); // y = C*x + v

        tic = micros();
        ctr_kalman_predict(&kf, &u_k); // Prediction: xhat = xhat_prior, P = P_prior
        ctr_kalman_correct(&kf, &y_k); // Correction: xhat = xhat_post, P = P_post
        kalman_time[k] = (float)((micros() - tic)/1000000.0);

        // Save new values
        xhat_history[k] = xhat.p_data[0];
        x_history[k] = x_k.p_data[0];
        u_history[k] = u_k.p_data[0];
        y_history[k] = y_k.p_data[0];
    }
    kalman_mean_time = mean(kalman_time, K);
    //Serial.print("Kalman Filter mean time (s): ");
    //Serial.println(kalman_mean_time, 9);

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("kalman_filter");
    for (uint8_t j = 0; j < K; j++)
    {
        Serial.print(kalman_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();

    // Print xhat
    //Serial.println("kalman_xhat");
    //for (uint8_t k = 0; k < K; k++)
    //{
    //    Serial.print(xhat_history[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print x
    //Serial.println("kalman_state");
    //for (uint8_t k = 0; k < K; k++)
    //{
    //    Serial.print(x_history[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print u
    //Serial.println("kalman_u");
    //for (uint8_t k = 0; k < K; k++)
    //{
    //    Serial.print(u_history[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print y
    //Serial.println("kalman_y");
    //for (uint8_t k = 0; k < K; k++)
    //{
    //    Serial.print(y_history[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();
}

void loop()
{

}
