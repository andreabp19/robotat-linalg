/**
 * @author: Andrea Pineda
 * @date: Created 7 Nov 2025, Last Modified 17 Oct 2026
 * 
 * For testing pid functions of robotat_control in the STM NUCLEO F446RE
 */

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "robotat_control.h"
#include "control_pid_test_data.h"

// --------------------------------------------------
// PID
// --------------------------------------------------

ctr_pid_t pid;

float u_k_pure_discrete[SAMPLES];
float x_k_pure_discrete[SAMPLES];
float y_k_pure_discrete[SAMPLES];

float u_k_fwd_eul[SAMPLES];
float x_k_fwd_eul[SAMPLES];
float y_k_fwd_eul[SAMPLES];

float u_k_bwd_eul[SAMPLES];
float x_k_bwd_eul[SAMPLES];
float y_k_bwd_eul[SAMPLES];

float u_k_tustin[SAMPLES];
float x_k_tustin[SAMPLES];
float y_k_tustin[SAMPLES];

// --------------------------------------------------
// LTI System for testing the PID
// --------------------------------------------------

float A_data[MAX_MAT_SIZE];
matf32_t A;

float B_data[MAX_MAT_SIZE];
matf32_t B;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float x_k_data[MAX_MAT_SIZE];
matf32_t x_k;

float y_k_data[MAX_MAT_SIZE];
matf32_t y_k;

float u_k_data[MAX_MAT_SIZE];
matf32_t u_k;

float Ax_k_data[MAX_MAT_SIZE];
matf32_t Ax_k;

float Bu_k_data[MAX_MAT_SIZE];
matf32_t Bu_k;

// --------------------------------------------------
// Other Variables
// --------------------------------------------------

uint8_t t = 100;

unsigned long tic = 0;

void setup()
{
    err_status_t status;

    // ---------------------------------------------------------------------------
    // Serial config
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // PID TESTING
    // ---------------------------------------------------------------------------

    ctr_sys_lti_t sys;
    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, dt);

    matf32_init(&A, 1, 1, A_data);
    matf32_init(&B, 1, 1, B_data);
    matf32_init(&C, 1, 1, C_data);
    matf32_init(&x_k, 1, 1, x_k_data);
    matf32_init(&y_k, 1, 1, y_k_data);
    matf32_init(&u_k, 1, 1, u_k_data);

    matf32_init(&Ax_k, 1, 1, Ax_k_data);
    matf32_init(&Bu_k, 1, 1, Bu_k_data);

    matf32_set(&A, 1, 1, 0.9);
    matf32_set(&B, 1, 1, 0.1);
    matf32_set(&C, 1, 1, 1);

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Pure Discrete
    // ---------------------------------------------------------------------------

    Serial.println("--------------------------------------------------");
    Serial.println("PID Update - Pure Discrete");
    Serial.println("--------------------------------------------------");

    ctr_pid_init(&pid, 0.9, 1.95, 0.001, PURE_DISCRETE, 0);
    pid.tau = tau;
    pid.dt = dt;

    float pid_pure_discrete_time[SAMPLES];
    float pid_pure_discrete_mean_time = 0;

    // Set initial values
    u_k_pure_discrete[0] = 0;
    x_k_pure_discrete[0] = 0;
    y_k_pure_discrete[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        tic = micros();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_pure_discrete[k]);
        pid_pure_discrete_time[k] = (float)((micros() - tic)/1000000.0);

        // Update system
        matf32_mul(&A, &x_k, &Ax_k); // A*x_k
        matf32_mul(&B, &u_k, &Bu_k); // B*u_k
        matf32_add(&Ax_k, &Bu_k, &x_k); // x_k = A*x_k + B*u_k

        matf32_mul(&C, &x_k, &y_k); // y_k = C*x_k

        // Save new data
        u_k_pure_discrete[k+1] = u_k.p_data[0];
        x_k_pure_discrete[k+1] = x_k.p_data[0];
        y_k_pure_discrete[k+1] = y_k.p_data[0];
    }
    pid_pure_discrete_mean_time = mean(pid_pure_discrete_time, SAMPLES);

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("pid_pure_discrete");
    for (uint8_t j = 0; j < SAMPLES; j++)
    {
        Serial.print(pid_pure_discrete_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();

    //Serial.print("PID Pure Discrete mean time (s): ");
    //Serial.println(pid_pure_discrete_mean_time, 9);

    // Print u_k
    //Serial.print("\n\npure_discrete_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(u_k_pure_discrete[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print x_k
    //Serial.print("\n\npure_discrete_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(x_k_pure_discrete[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print y_k
    //Serial.print("\n\npure_discrete_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(y_k_pure_discrete[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Forward Euler
    // ---------------------------------------------------------------------------

    Serial.println("--------------------------------------------------");
    Serial.println("PID Update - Forward Euler");
    Serial.println("--------------------------------------------------");

    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    pid.tau = tau;
    pid.dt = dt;

    //ctr_pid_print(&pid);

    float pid_fwd_eul_time[SAMPLES];
    float pid_fwd_eul_mean_time = 0;
    
    //time = clock();

    // Set initial values
    u_k_fwd_eul[0] = 0;
    x_k_fwd_eul[0] = 0;
    y_k_fwd_eul[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        tic = micros();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_fwd_eul[k]);
        pid_fwd_eul_time[k] = (float)((micros() - tic)/1000000.0);
        
        // Update system
        matf32_mul(&A, &x_k, &Ax_k); // A*x_k
        matf32_mul(&B, &u_k, &Bu_k); // B*u_k
        matf32_add(&Ax_k, &Bu_k, &x_k); // x_k = A*x_k + B*u_k

        matf32_mul(&C, &x_k, &y_k); // y_k = C*x_k

        // Save new data
        u_k_fwd_eul[k+1] = u_k.p_data[0];
        x_k_fwd_eul[k+1] = x_k.p_data[0];
        y_k_fwd_eul[k+1] = y_k.p_data[0];
    }
    //pid_fwd_eul_mean_time = (float)(clock()-time)/SAMPLES/CLOCKS_PER_SEC;

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("pid_fwd_eul");
    for (uint8_t j = 0; j < SAMPLES; j++)
    {
        Serial.print(pid_fwd_eul_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();   

    // Print u_k
    //Serial.print("\n\nfwd_eul_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(u_k_fwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print x_k
    //Serial.print("\n\nfwd_eul_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(x_k_fwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print y_k
    //Serial.print("\n\nfwd_eul_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(y_k_fwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Backward Euler
    // ---------------------------------------------------------------------------

    Serial.println("--------------------------------------------------");
    Serial.println("PID Update - Backward Euler");
    Serial.println("--------------------------------------------------");

    ctr_pid_init(&pid, kp, ki, kd, BWD_EULER, 0);
    pid.tau = tau;
    pid.dt = dt;

    //ctr_pid_print(&pid);

    float pid_bwd_eul_time[SAMPLES];
    float pid_bwd_eul_mean_time = 0;
    
    //time = clock();

    // Set initial values
    u_k_bwd_eul[0] = 0;
    x_k_bwd_eul[0] = 0;
    y_k_bwd_eul[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        tic = micros();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_bwd_eul[k]);
        pid_bwd_eul_time[k] = (float)((micros() - tic)/1000000.0);
        
        // Update system
        matf32_mul(&A, &x_k, &Ax_k); // A*x_k
        matf32_mul(&B, &u_k, &Bu_k); // B*u_k
        matf32_add(&Ax_k, &Bu_k, &x_k); // x_k = A*x_k + B*u_k

        matf32_mul(&C, &x_k, &y_k); // y_k = C*x_k

        // Save new data
        u_k_bwd_eul[k+1] = u_k.p_data[0];
        x_k_bwd_eul[k+1] = x_k.p_data[0];
        y_k_bwd_eul[k+1] = y_k.p_data[0];
    }
    //pid_bwd_eul_mean_time = (float)(clock()-time)/SAMPLES/CLOCKS_PER_SEC;

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("pid_bwd_eul");
    for (uint8_t j = 0; j < SAMPLES; j++)
    {
        Serial.print(pid_bwd_eul_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();

    // Print u_k
    //Serial.print("\n\nbwd_eul_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(u_k_bwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print x_k
    //Serial.print("\n\nbwd_eul_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(x_k_bwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print y_k
    //Serial.print("\n\nbwd_eul_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(y_k_bwd_eul[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Tustin
    // ---------------------------------------------------------------------------

    Serial.println("--------------------------------------------------");
    Serial.println("PID Update - Tustin");
    Serial.println("--------------------------------------------------");

    ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 0);
    pid.tau = tau;
    pid.dt = dt;

    //ctr_pid_print(&pid);

    float pid_tustin_time[SAMPLES];
    float pid_tustin_mean_time = 0;
    
    //time = clock();

    // Set initial values
    u_k_tustin[0] = 0;
    x_k_tustin[0] = 0;
    y_k_tustin[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        tic = micros();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_tustin[k]);
        pid_tustin_time[k] = (float)((micros() - tic)/1000000.0);
        
        // Update system
        matf32_mul(&A, &x_k, &Ax_k); // A*x_k
        matf32_mul(&B, &u_k, &Bu_k); // B*u_k
        matf32_add(&Ax_k, &Bu_k, &x_k); // x_k = A*x_k + B*u_k

        matf32_mul(&C, &x_k, &y_k); // y_k = C*x_k

        // Save new data
        u_k_tustin[k+1] = u_k.p_data[0];
        x_k_tustin[k+1] = x_k.p_data[0];
        y_k_tustin[k+1] = y_k.p_data[0];
    }
    //pid_tustin_mean_time = (float)(clock()-time)/SAMPLES/CLOCKS_PER_SEC;

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("pid_tustin");
    for (uint8_t j = 0; j < SAMPLES; j++)
    {
        Serial.print(pid_tustin_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();

    // Print u_k
    //Serial.print("\n\ntustin_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(u_k_tustin[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print x_k
    //Serial.print("\n\ntustin_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(x_k_tustin[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Print y_k
    //Serial.print("\n\ntustin_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    Serial.print(y_k_tustin[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();
}

void loop()
{

}
