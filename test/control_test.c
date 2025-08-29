

/**
 * @author: Andrea Pineda
 * @date: Created 23 Aug 2025, Last Modified 27 Aug 2025
 * 
 * For testing robotat_control in computer
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "linsolve.h"
#include "robotat_control.h"

// --------------------------------------------------
// PID
// --------------------------------------------------

float kp = 0.75;
float ki = 0.1;
float kd = 0.25;

ctr_pid_t pid;

float r_k = 6;
float y_k = 3;

// --------------------------------------------------
// LTI State Space System
// --------------------------------------------------

float state_dim = 3;

float state_data[MAX_MAT_SIZE];
matf32_t state;

float A_data[MAX_MAT_SIZE];
matf32_t A;

float B_data[MAX_MAT_SIZE];
matf32_t B;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float u_data[MAX_MAT_SIZE];
matf32_t u_k;

float x_data[MAX_MAT_SIZE];
matf32_t x_k;

float sample_time = FLT_EPSILON;

ctr_sys_lti_t sys_lti;

// --------------------------------------------------
// LTI State Space System
// --------------------------------------------------

float F_data[MAX_MAT_SIZE];
matf32_t F;

float Qw_data[MAX_MAT_SIZE];
matf32_t Qw;

float Qv_data[MAX_MAT_SIZE];
matf32_t Qv;

float xhat_data[MAX_MAT_SIZE];
matf32_t xhat;

float P_data[MAX_MAT_SIZE];
matf32_t P;

float kf_inputs_data[MAX_MAT_SIZE];
matf32_t kf_inputs;

float kf_measurements_data[MAX_MAT_SIZE];
matf32_t kf_measurements;

ctr_kalman_t kf;

uint8_t t = 100;

int main(void)
{
    clock_t time;
    err_status_t status;

    // ---------------------------------------------------------------------------
    // PID TESTING
    // ---------------------------------------------------------------------------

    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    
    // ---------------------------------------------------------------------------
    // 1. ctr_pid_update
    // ---------------------------------------------------------------------------

    float pid_update_time[t];
    float pid_update_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_pid_update(&pid, r_k, y_k);
        pid_update_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    pid_update_mean_time = mean(pid_update_time, t);

    //bool pid_update_ans = matf32_is_equal(F_AB.p_T, &R_pid_update);
    //printf("pid_update          ,%s,mean_time(s): %.9f\n", pid_update_ans?"success":"failure", pid_update_mean_time);
    printf("pid_update,mean_time(s):%.9f\n", pid_update_mean_time);

    // ---------------------------------------------------------------------------
    // STATE SPACE TESTING
    // ---------------------------------------------------------------------------

    matf32_init(&state, state_dim, 1, state_data);
    matf32_init(&A, state_dim, state_dim, A_data);
    matf32_init(&B, state_dim, 1, B_data);
    matf32_init(&C, 1, state_dim, C_data);
    matf32_init(&D, C.num_rows, B.num_cols, D_data);
    matf32_init(&u_k, B.num_cols, 1, u_data);
    matf32_init(&x_k, A.num_cols, 1, x_data);

    matf32_randn(&A, 0, 1);
    matf32_randn(&B, 0, 1);
    matf32_randn(&C, 0, 1);
    matf32_randn(&D, 0, 1);

    // System starts with is_continuous = 1
    ctr_sys_lti_init(&sys_lti, &state, &A, &B, &C, &D, sample_time);

    // ---------------------------------------------------------------------------
    // 1. ctr_c2d
    // ---------------------------------------------------------------------------

    float c2d_time[t];
    float c2d_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        status = ctr_c2d(&sys_lti, sample_time, FWD_EULER);
        c2d_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    c2d_mean_time = mean(c2d_time, t);

    //bool c2d_ans = matf32_is_equal(F_AB.p_T, &R_c2d);
    //printf("c2d          ,%s,mean_time(s): %.9f\n", c2d_ans?"success":"failure", c2d_mean_time);
    printf("c2d,mean_time(s):%.9f\n", c2d_mean_time);

    // ---------------------------------------------------------------------------
    // 2. ctr_linloc
    // ---------------------------------------------------------------------------

    

    // ---------------------------------------------------------------------------
    // KALMAN FILTER TESTING
    // ---------------------------------------------------------------------------

    matf32_init(&F, 1, 1, F_data);
    matf32_init(&Qw, sys_lti.input_dim, 1, Qw_data);
    matf32_init(&Qv, sys_lti.output_dim, 1, Qv_data);
    matf32_init(&xhat, 1, 1, xhat_data);
    matf32_init(&P, A.num_rows, A.num_cols, P_data);

    ctr_kalman_init(&kf, &sys_lti, &F, &Qw, &Qv, &xhat, &P);

    matf32_init(&kf_inputs, kf.sys->input_dim, 1, kf_inputs_data);
    matf32_init(&kf_measurements, kf.sys->output_dim, 1, kf_measurements_data);
    matf32_randn(&kf_inputs, 0, 1);
    matf32_randn(&kf_measurements, 0, 1);

    // ---------------------------------------------------------------------------
    // 1. ctr_kalman_predict
    // ---------------------------------------------------------------------------

    float kalman_predict_time[t];
    float kalman_predict_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_kalman_predict(&kf, &kf_inputs);
        kalman_predict_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    kalman_predict_mean_time = mean(kalman_predict_time, t);

    //bool kalman_predict_ans = matf32_is_equal(F_AB.p_T, &R_kalman_predict);
    //printf("kalman_predict          ,%s,mean_time(s): %.9f\n", kalman_predict_ans?"success":"failure", kalman_predict_mean_time);
    printf("kalman_predict,mean_time(s):%.9f\n", kalman_predict_mean_time);

    ctr_kalman_print(&kf);
    printf("Kalman Filter Inputs:\n");
    matf32_print(&kf_inputs);

    // ---------------------------------------------------------------------------
    // 2. ctr_kalman_correct
    // ---------------------------------------------------------------------------

    float kalman_correct_time[t];
    float kalman_correct_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        status = ctr_kalman_correct(&kf, &kf_inputs);
        kalman_correct_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    kalman_correct_mean_time = mean(kalman_correct_time, t);

    //bool kalman_correct_ans = matf32_is_equal(F_AB.p_T, &R_kalman_correct);
    //printf("kalman_correct          ,%s,mean_time(s): %.9f\n", kalman_correct_ans?"success":"failure", kalman_correct_mean_time);
    printf("kalman_correct,mean_time(s):%.9f\n", kalman_correct_mean_time);

    ctr_kalman_print(&kf);
    printf("Kalman Filter Measurements:\n");
    matf32_print(&kf_measurements);
}