

/**
 * @author: Andrea Pineda
 * @date: Created 23 Aug 2025, Last Modified 8 Sep 2025
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
#include "control_test_results.h"

// --------------------------------------------------
// PID
// --------------------------------------------------

ctr_pid_t pid;

float u_k_fwd_eul[100];
float u_k_bwd_eul[100];

// --------------------------------------------------
// LTI State Space System
// --------------------------------------------------

uint16_t state_dim = 1;

float state_data[MAX_MAT_SIZE];
matf32_t state;

float* A_fwd_eul_list[9] = {A_fwd_eul2, A_fwd_eul3, A_fwd_eul4, A_fwd_eul5, A_fwd_eul6, A_fwd_eul7, A_fwd_eul8, A_fwd_eul9, A_fwd_eul10};
matf32_t A_fwd_eul;

float* A_bwd_eul_list[9] = {A_bwd_eul2, A_bwd_eul3, A_bwd_eul4, A_bwd_eul5, A_bwd_eul6, A_bwd_eul7, A_bwd_eul8, A_bwd_eul9, A_bwd_eul10};
matf32_t A_bwd_eul;

float* A_tustin_list[9] = {A_tustin2, A_tustin3, A_tustin4, A_tustin5, A_tustin6, A_tustin7, A_tustin8, A_tustin9, A_tustin10};
matf32_t A_tustin;

float* B_fwd_eul_list[9] = {B_fwd_eul2, B_fwd_eul3, B_fwd_eul4, B_fwd_eul5, B_fwd_eul6, B_fwd_eul7, B_fwd_eul8, B_fwd_eul9, B_fwd_eul10};
matf32_t B_fwd_eul;

float* B_bwd_eul_list[9] = {B_bwd_eul2, B_bwd_eul3, B_bwd_eul4, B_bwd_eul5, B_bwd_eul6, B_bwd_eul7, B_bwd_eul8, B_bwd_eul9, B_bwd_eul10};
matf32_t B_bwd_eul;

float* B_tustin_list[9] = {B_tustin2, B_tustin3, B_tustin4, B_tustin5, B_tustin6, B_tustin7, B_tustin8, B_tustin9, B_tustin10};
matf32_t B_tustin;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float sample_time = 2;

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

// --------------------------------------------------
// LTI State Space System
// --------------------------------------------------

// Transference function data in control_test_results.h in H_num and H_den

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

float ye_data[MAX_MAT_SIZE];
matf32_t ye;

ctr_sys_lti_t kf_sys;
ctr_kalman_t kf;

uint8_t t = 100;

int main(void)
{
    clock_t time;
    err_status_t status;

    // ---------------------------------------------------------------------------
    // PID TESTING
    // ---------------------------------------------------------------------------

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Forward Euler
    // ---------------------------------------------------------------------------

    /*printf("\n--------------------------------------------------\n");
    printf("PID Update - Forward Euler\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    //ctr_pid_print(&pid);

    float pid_fwd_eul_time[t];
    float pid_fwd_eul_mean_time = 0;

    // Iterate over each of the values in r_k (doubles as iterations for measuring the time)
    for (uint8_t k = 0; k < samples; k++)
    {
        time = clock();
        u_k_fwd_eul[k] = ctr_pid_update(&pid, r_k[k], y_k);
        pid_fwd_eul_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/

    // Print time values formatted to a .mat for plotting in matlab
    //printf("pid_fwd_eul\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", pid_fwd_eul_time[j]);
    //}
    //printf("\n\n");

    // Print results for comparison
    //printf("ESP32 u_k          MATLAB u_k,        Difference:\n");
    //for (uint8_t j = 0; j < samples; j++)
    //{
    //    printf("%.9f,       %.9f,       %s\n", u_k_fwd_eul[j], R_u_k_fwd_eul[j], u_k_fwd_eul[j]==R_u_k_fwd_eul[j]?"success":"failure");
    //}
    //printf("\n");

    //pid_fwd_eul_mean_time = mean(pid_fwd_eul_time, t);

    //printf("pid_fwd_eul,mean_time(s):%.9f\n", pid_fwd_eul_mean_time);

    // ---------------------------------------------------------------------------
    // ctr_pid_update - Backward Euler
    // ---------------------------------------------------------------------------

    /*printf("\n--------------------------------------------------\n");
    printf("PID Update - Backward Euler\n");
    printf("--------------------------------------------------\n");

    ctr_pid_init(&pid, kp, ki, kd, BWD_EULER, 0);
    //ctr_pid_print(&pid);

    float pid_bwd_eul_time[t];
    float pid_bwd_eul_mean_time = 0;

    // Iterate over each of the values in r_k (doubles as iterations for measuring the time)
    for (uint8_t k = 0; k < samples; k++)
    {
        time = clock();
        u_k_bwd_eul[k] = ctr_pid_update(&pid, r_k[k], y_k);
        pid_bwd_eul_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("pid_bwd_eul\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", pid_bwd_eul_time[j]);
    //}
    //printf("\n\n");

    // Print results for comparison
    //printf("ESP32 u_k          MATLAB u_k,        Difference:\n");
    //for (uint8_t j = 0; j < samples; j++)
    //{
    //    printf("%.9f,       %.9f,       %s\n", u_k_bwd_eul[j], R_u_k_bwd_eul[j], u_k_bwd_eul[j]==R_u_k_bwd_eul[j]?"success":"failure");
    //}
    //printf("\n");

    //pid_bwd_eul_mean_time = mean(pid_bwd_eul_time, t);

    //printf("pid_bwd_eul,mean_time(s):%.9f\n", pid_bwd_eul_mean_time);

    for (uint16_t i = 0; i < 9; ++i)
    {
        state_dim += 1;

        //printf("\n--------------------------------------------------\n");
        //printf("n = %i\n\n", state_dim);
        //printf("--------------------------------------------------\n");

        // ---------------------------------------------------------------------------
        // STATE SPACE TESTING
        // ---------------------------------------------------------------------------

        matf32_init(&state, state_dim, 1, state_data);
        matf32_init(&A_fwd_eul, state_dim, state_dim, A_fwd_eul_list[i]);
        matf32_init(&B_fwd_eul, state_dim, 1, B_fwd_eul_list[i]);

        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);
        
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);
        
        matf32_init(&C, 1, state_dim, C_data);
        matf32_init(&D, C.num_rows, B_bwd_eul.num_cols, D_data);

        ctr_sys_lti_init(&sys_fwd_eul, &state, &A_fwd_eul, &B_fwd_eul, &C, &D, sample_time);
        //ctr_sys_lti_print(&sys_fwd_eul);

        ctr_sys_lti_init(&sys_bwd_eul, &state, &A_bwd_eul, &B_bwd_eul, &C, &D, sample_time);
        //ctr_sys_lti_print(&sys_bwd_eul);

        ctr_sys_lti_init(&sys_tustin, &state, &A_tustin, &B_tustin, &C, &D, sample_time);
        //ctr_sys_lti_print(&sys_tustin);

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

        //printf("\n--------------------------------------------------\n");
        //printf("c2d - Forward Euler\n");
        //printf("--------------------------------------------------\n");

        float c2d_fwd_eul_time[t];
        float c2d_fwd_eul_mean_time = 0;

        // As ctr_c2d overwrites A and B, to test it 100 times with the same values, it's either
        // to reset the value to the original each loop, or test 100 random matrices and not worry about the result vs matlab when measuring time.
        // Either way, sys_lti.is_continuous must be set to true each cycle in order to be able to operate with ctr_c2d (outside the time measurement).
        for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_fwd_eul,0,1);
            matf32_randn(&B_fwd_eul,0,1);
            sys_fwd_eul.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);
            c2d_fwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        // Uncomment this section and comment the loop to test a single operation to compare results with matlab
        //sys_fwd_eul.is_continuous = true;
        //status = ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);

        /*printf("ESP32 - Ad:\n");
        matf32_print(&A_fwd_eul);
        printf("ESP32 - Bd:\n");
        matf32_print(&B_fwd_eul);

        printf("MATLAB - Ad:\n");
        matf32_print(&R_c2d_fwd_eul_Ad);
        printf("MATLAB - Bd:\n");
        matf32_print(&R_c2d_fwd_eul_Bd);*/

        c2d_fwd_eul_mean_time = mean(c2d_fwd_eul_time, t);

        //bool c2d_fwd_eul_Ad_ans = matf32_is_equal(&A_fwd_eul, &R_c2d_fwd_eul_Ad);
        //bool c2d_fwd_eul_Bd_ans = matf32_is_equal(&B_fwd_eul, &R_c2d_fwd_eul_Bd);
        //printf("c2d_fwd_eul,Ad:%s,Bd:%s\n", c2d_fwd_eul_Ad_ans?"success":"failure", c2d_fwd_eul_Bd_ans?"success":"failure"); // Results comparison
        printf("c2d_fwd_eul%i\n", state_dim);
        printf("%.9f\n", c2d_fwd_eul_mean_time);
        //printf("c2d_fwd_eul,mean_time(s):%.9f\n", c2d_fwd_eul_mean_time); // Time measurement

        /*printf("Ad result difference:\n");
        matf32_sub(&A_fwd_eul, &R_c2d_fwd_eul_Ad, &A_fwd_eul);
        matf32_print(&A_fwd_eul);
        printf("Bd result difference:\n");
        matf32_sub(&B_fwd_eul, &R_c2d_fwd_eul_Bd, &B_fwd_eul);
        matf32_print(&B_fwd_eul);*/

        // ---------------------------------------------------------------------------
        // ctr_c2d - Backward Euler
        // ---------------------------------------------------------------------------

        //printf("\n--------------------------------------------------\n");
        //printf("c2d - Backward Euler\n");
        //printf("--------------------------------------------------\n");

        // Initialize again A and B
        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_bwd_eul_Ad, A_bwd_eul.num_rows, A_bwd_eul.num_cols, R_c2d_bwd_eul_list_Ad[i]);
        matf32_init(&R_c2d_bwd_eul_Bd, B_bwd_eul.num_rows, B_bwd_eul.num_cols, R_c2d_bwd_eul_list_Bd[i]);

        float c2d_bwd_eul_time[t];
        float c2d_bwd_eul_mean_time = 0;

        for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_bwd_eul,0,1);
            matf32_randn(&B_bwd_eul,0,1);
            sys_bwd_eul.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);
            c2d_bwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        //sys_bwd_eul.is_continuous = true;
        //status = ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);

        /*printf("ESP32 - Ad:\n");
        matf32_print(&A_bwd_eul);
        printf("ESP32 - Bd:\n");
        matf32_print(&B_bwd_eul);

        printf("MATLAB - Ad:\n");
        matf32_print(&R_c2d_bwd_eul_Ad);
        printf("MATLAB - Bd:\n");
        matf32_print(&R_c2d_bwd_eul_Bd);*/

        c2d_bwd_eul_mean_time = mean(c2d_bwd_eul_time, t);

        //bool c2d_bwd_eul_Ad_ans = matf32_is_equal(&A_bwd_eul, &R_c2d_bwd_eul_Ad);
        //bool c2d_bwd_eul_Bd_ans = matf32_is_equal(&B_bwd_eul, &R_c2d_bwd_eul_Bd);
        //printf("c2d_bwd_eul,Ad:%s,Bd:%s\n", c2d_bwd_eul_Ad_ans?"success":"failure", c2d_bwd_eul_Bd_ans?"success":"failure"); // Results comparison
        printf("c2d_bwd_eul%i\n", state_dim);
        printf("%.9f\n", c2d_bwd_eul_mean_time);
        //printf("c2d_bwd_eul,mean_time(s):%.9f\n", c2d_bwd_eul_mean_time); // Time measurement

        /*printf("Ad result difference:\n");
        matf32_sub(&A_bwd_eul, &R_c2d_bwd_eul_Ad, &A_bwd_eul);
        matf32_print(&A_bwd_eul);
        printf("Bd result difference:\n");
        matf32_sub(&B_bwd_eul, &R_c2d_bwd_eul_Bd, &B_bwd_eul);
        matf32_print(&B_bwd_eul);*/

        // ---------------------------------------------------------------------------
        // ctr_c2d - Tustin
        // ---------------------------------------------------------------------------

        //printf("\n--------------------------------------------------\n");
        //printf("c2d - Tustin\n");
        //printf("--------------------------------------------------\n");

        // Initialize again A and B
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_tustin_Ad, A_tustin.num_rows, A_tustin.num_cols, R_c2d_tustin_list_Ad[i]);
        matf32_init(&R_c2d_tustin_Bd, B_tustin.num_rows, B_tustin.num_cols, R_c2d_tustin_list_Bd[i]);

        float c2d_tustin_time[t];
        float c2d_tustin_mean_time = 0;

        for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_tustin,0,1);
            matf32_randn(&B_tustin,0,1);
            sys_tustin.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_tustin, sample_time, TUSTIN);
            c2d_tustin_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        //sys_tustin.is_continuous = true;
        //status = ctr_c2d(&sys_tustin, sample_time, TUSTIN);

        //printf("ESP32 - Ad:\n");
        //matf32_print(&A_tustin);
        //printf("ESP32 - Bd:\n");
        //matf32_print(&B_tustin);

        //printf("MATLAB - Ad:\n");
        //matf32_print(&R_c2d_tustin_Ad);
        //printf("MATLAB - Bd:\n");
        //matf32_print(&R_c2d_tustin_Bd);

        c2d_tustin_mean_time = mean(c2d_tustin_time, t);

        //bool c2d_tustin_Ad_ans = matf32_is_equal(&A_tustin, &R_c2d_tustin_Ad);
        //bool c2d_tustin_Bd_ans = matf32_is_equal(&B_tustin, &R_c2d_tustin_Bd);
        //printf("c2d_tustin,Ad:%s,Bd:%s\n", c2d_tustin_Ad_ans?"success":"failure", c2d_tustin_Bd_ans?"success":"failure"); // Results comparison
        printf("c2d_tustin%i\n", state_dim);
        printf("%.9f\n", c2d_tustin_mean_time);
        //printf("c2d_tustin,mean_time(s):%.9f\n", c2d_tustin_mean_time); // Time measurement
    
        /*printf("Ad result difference:\n");
        matf32_sub(&A_tustin, &R_c2d_tustin_Ad, &A_tustin);
        matf32_print(&A_tustin);
        printf("Bd result difference:\n");
        matf32_sub(&B_tustin, &R_c2d_tustin_Bd, &B_tustin);
        matf32_print(&B_tustin);*/
    }
        

    

    // ---------------------------------------------------------------------------
    // ctr_linloc vs loclin_fast.m
    // ---------------------------------------------------------------------------

    

    // ---------------------------------------------------------------------------
    // KALMAN FILTER TESTING
    // ---------------------------------------------------------------------------

    /*float m = 3;
    float n = 1;

    matf32_init(&state, m, 1, state_data);
    matf32_init(&A, m, m, Kf_A_data);
    matf32_init(&B, m, n, Kf_B_data);
    matf32_init(&C, 1, m, Kf_C_data);
    matf32_init(&D, 1, n, Kf_D_data);

    ctr_sys_lti_init(&kf_sys, &state, &A, &B, &C, &D, sample_time);
    ctr_sys_lti_print(&kf_sys);
    ctr_c2d(&kf_sys, sample_time, FWD_EULER);

    matf32_init(&F, 1, 1, F_data);
    matf32_init(&Qw, kf_sys.input_dim, kf_sys.input_dim, Kf_Qw_data);
    matf32_init(&Qv, kf_sys.output_dim, kf_sys.output_dim, Kf_Qv_data);
    matf32_init(&xhat, m, 1, xhat_data);
    matf32_init(&P, A.num_rows, A.num_cols, P_data);
    matf32_init(&ye, xhat.num_rows, xhat.num_cols, ye_data);

    ctr_kalman_init(&kf, &kf_sys, &F, &Qw, &Qv, &xhat, &P);
    ctr_kalman_print(&kf);

    matf32_init(&kf_inputs, kf.sys->input_dim, 1, &Kf_u_data[0]);
    matf32_init(&kf_measurements, kf.sys->output_dim, 1, kf_measurements_data);
    matf32_randn(&kf_measurements, 0, 1);

    printf("Qw:\n");
    matf32_print(&Qw);
    printf("Qv:\n");
    matf32_print(&Qv);

    printf("kf_inputs:\n");
    matf32_print(&kf_inputs);
    printf("kf_measurements:\n");
    matf32_print(&kf_measurements);*/

    // ---------------------------------------------------------------------------
    // ctr_kalman_predict
    // ---------------------------------------------------------------------------

    //printf("\n--------------------------------------------------\n");
    //printf("Kalman Filter - Predict \n");
    //printf("--------------------------------------------------\n");

    /*float kalman_predict_time[t];
    float kalman_predict_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_kalman_predict(&kf, &kf_inputs);
        kalman_predict_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    kalman_predict_mean_time = mean(kalman_predict_time, t);*/

    //bool kalman_predict_ans = matf32_is_equal(F_AB.p_T, &R_kalman_predict);
    //printf("kalman_predict          ,%s,mean_time(s): %.9f\n", kalman_predict_ans?"success":"failure", kalman_predict_mean_time);
    //printf("kalman_predict,mean_time(s):%.9f\n", kalman_predict_mean_time);

    //ctr_kalman_print(&kf);
    //printf("Kalman Filter Inputs:\n");
    //matf32_print(&kf_inputs);

    // ---------------------------------------------------------------------------
    // ctr_kalman_correct
    // ---------------------------------------------------------------------------

    //printf("\n--------------------------------------------------\n");
    //printf("Kalman Filter - Correct\n");
    //printf("--------------------------------------------------\n");

    /*float kalman_correct_time[t];
    float kalman_correct_mean_time = 0;

    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        status = ctr_kalman_correct(&kf, &kf_inputs);
        kalman_correct_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    kalman_correct_mean_time = mean(kalman_correct_time, t);*/

    //bool kalman_correct_ans = matf32_is_equal(F_AB.p_T, &R_kalman_correct);
    //printf("kalman_correct          ,%s,mean_time(s): %.9f\n", kalman_correct_ans?"success":"failure", kalman_correct_mean_time);
    //printf("kalman_correct,mean_time(s):%.9f\n", kalman_correct_mean_time);

    //ctr_kalman_print(&kf);
    //printf("Kalman Filter Measurements:\n");
    //matf32_print(&kf_measurements);

    // ---------------------------------------------------------------------------
    // Kalman Filter
    // ---------------------------------------------------------------------------

    /*printf("\n--------------------------------------------------\n");
    printf("Kalman Filter - Full implementation \n");
    printf("--------------------------------------------------\n");

    float kalman_filter_time[t];
    float kalman_filter_mean_time = 0;

    //printf("xhat:\n");
    // Iterate over each of the values in Kf_u (doubles as iterations for measuring the time)
    /*for (uint8_t k = 0; k < samples; k++)
    {
        time = clock();
        status = ctr_kalman_predict(&kf, &kf_inputs); // Calculates preliminar xhat and calculates P
        status = ctr_kalman_correct(&kf, &kf_measurements); // Calculates L and updates xhat
        kalman_filter_time[k] = (float)(clock()-time)/CLOCKS_PER_SEC;
        //printf("%.9f\n", *kf.xhat->p_data);
    }*/
    
    /*status = ctr_kalman_predict(&kf, &kf_inputs); // Calculates preliminar xhat and calculates P
    status = ctr_kalman_correct(&kf, &kf_measurements); // Calculates L and updates xhat
    printf("xhat:\n");
    matf32_print(&xhat);
    printf("C:\n");
    matf32_print(&C);
    matf32_vecmul_col_row(xhat.p_data, C.p_data, &ye);
    printf("ye:\n");
    matf32_print(&ye);*/

    //ctr_kalman_print(&kf);

    //kalman_filter_mean_time = mean(kalman_filter_time, t);

    //bool kalman_filter_ans = matf32_is_equal(F_AB.p_T, &R_kalman_filter);
    //printf("kalman_filter          ,%s,mean_time(s): %.9f\n", kalman_filter_ans?"success":"failure", kalman_filter_mean_time);
    //printf("kalman_filter,mean_time(s):%.9f\n", kalman_filter_mean_time);

    //ctr_kalman_print(&kf);
    //printf("Kalman Filter Inputs:\n");
    //matf32_print(&kf_inputs);
    
    printf("\n");
}