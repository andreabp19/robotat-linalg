/**
 * @author: Andrea Pineda
 * @date: Created 20 Oct 2025, Last Modified 23 Oct 2025
 * 
 * For testing pid functions of robotat_control in a computer
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

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

int main(void)
{
    clock_t time;
    err_status_t status;

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

    printf("\n--------------------------------------------------\n");
    printf("PID Update - Pure Discrete\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, 2.5, 0.95, 0.5, PURE_DISCRETE, 0);
    pid.tau = tau;
    pid.dt = dt;

    //ctr_pid_print(&pid);

    float pid_pure_discrete_time[SAMPLES];
    float pid_pure_discrete_mean_time = 0;
    
    //time = clock();

    // Set initial values
    u_k_pure_discrete[0] = 0;
    x_k_pure_discrete[0] = 0;
    y_k_pure_discrete[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        time = clock();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_pure_discrete[k]);
        pid_pure_discrete_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;

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
    printf("PID Pure Discrete mean time (s): %.9f\n", pid_pure_discrete_mean_time);

    // Print u_k
    //printf("\n\npure_discrete_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", u_k_pure_discrete[k]);
    //}
    //printf("\n\n");

    // Print x_k
    //printf("\n\npure_discrete_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", x_k_pure_discrete[k]);
    //}
    //printf("\n\n");

    // Print y_k
    //printf("\n\npure_discrete_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", y_k_pure_discrete[k]);
    //}
    //printf("\n\n");

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Forward Euler
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("PID Update - Forward Euler\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    pid.tau = tau;
    pid.dt = dt;

    float pid_fwd_eul_time[SAMPLES];
    float pid_fwd_eul_mean_time = 0;

    // Set initial values
    u_k_fwd_eul[0] = 0;
    x_k_fwd_eul[0] = 0;
    y_k_fwd_eul[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        time = clock();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_fwd_eul[k]);
        pid_fwd_eul_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
        
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
    pid_fwd_eul_mean_time = mean(pid_fwd_eul_time, SAMPLES);
    printf("PID Forward Euler mean time (s): %.9f\n", pid_fwd_eul_mean_time);

    // Print u_k
    //printf("\n\nfwd_eul_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", u_k_fwd_eul[k]);
    //}
    //printf("\n\n");

    // Print x_k
    //printf("\n\nfwd_eul_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", x_k_fwd_eul[k]);
    //}
    //printf("\n\n");

    // Print y_k
    //printf("\n\nfwd_eul_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", y_k_fwd_eul[k]);
    //}
    //printf("\n\n");

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Backward Euler
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("PID Update - Backward Euler\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, kp, ki, kd, BWD_EULER, 0);
    pid.tau = tau;
    pid.dt = dt;

    float pid_bwd_eul_time[SAMPLES];
    float pid_bwd_eul_mean_time = 0;
    
    // Set initial values
    u_k_bwd_eul[0] = 0;
    x_k_bwd_eul[0] = 0;
    y_k_bwd_eul[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        time = clock();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_bwd_eul[k]);
        pid_bwd_eul_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
        
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
    pid_bwd_eul_mean_time = mean(pid_bwd_eul_time, SAMPLES);
    printf("PID Backward Euler mean time (s): %.9f\n", pid_bwd_eul_mean_time);

    // Print u_k
    //printf("\n\nbwd_eul_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", u_k_bwd_eul[k]);
    //}
    //printf("\n\n");

    // Print x_k
    //printf("\n\nbwd_eul_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", x_k_bwd_eul[k]);
    //}
    //printf("\n\n");

    // Print y_k
    //printf("\n\nbwd_eul_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", y_k_bwd_eul[k]);
    //}
    //printf("\n\n");

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Tustin
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("PID Update - Tustin\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 0);
    pid.tau = tau;
    pid.dt = dt;

    float pid_tustin_time[SAMPLES];
    float pid_tustin_mean_time = 0;

    // Set initial values
    u_k_tustin[0] = 0;
    x_k_tustin[0] = 0;
    y_k_tustin[0] = 0;

    for (uint8_t k = 0; k < SAMPLES-1; k++)
    {
        // Discretize signal
        time = clock();
        u_k.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], y_k_tustin[k]);
        pid_tustin_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
        
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
    pid_tustin_mean_time = mean(pid_tustin_time, SAMPLES);
    printf("PID Tustin mean time (s): %.9f\n", pid_tustin_mean_time);

    // Print u_k
    //printf("\n\ntustin_u_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", u_k_tustin[k]);
    //}
    //printf("\n\n");

    // Print x_k
    //printf("\n\ntustin_x_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", x_k_tustin[k]);
    //}
    //printf("\n\n");

    // Print y_k
    //printf("\n\ntustin_y_k\n");
    //for (uint8_t k = 0; k < SAMPLES; k++)
    //{
    //    printf("%.9f ", y_k_tustin[k]);
    //}
    //printf("\n\n");
}