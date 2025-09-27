

/**
 * @author: Andrea Pineda
 * @date: Created 23 Aug 2025, Last Modified 16 Sep 2025
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
// Kalman Filter
// --------------------------------------------------

ctr_sys_lti_t kf_sys;
ctr_kalman_t kf;

// --------------------------------------------------
// Nonlinear systems
// --------------------------------------------------

ctr_sys_nonlin_t nonlin_pendulum;

float nonlin_state_data[MAX_MAT_SIZE];
matf32_t nonlin_state;

uint16_t nonlin_input_dim = 1;
uint16_t nonlin_output_dim = 1;

float sys_state_data[MAX_MAT_SIZE];
matf32_t sys_state;

float sys_input_data[MAX_MAT_SIZE];
matf32_t sys_input;

float xdot_data[MAX_MAT_SIZE];
matf32_t xdot;

float sys_output_data[MAX_MAT_SIZE];
matf32_t sys_output;

err_status_t pendulum_dynamics(matf32_t* const xdot, const matf32_t* sys_state, const matf32_t* sys_input);
err_status_t pendulum_outputs(matf32_t* const sys_output, const matf32_t* sys_state, const matf32_t* sys_input);

// --------------------------------------------------
// Other variables
// --------------------------------------------------

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
    // Turn this into a function to operate over r_k of any length
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

    //pid_bwd_eul_mean_time = mean(pid_bwd_eul_time, t);
    //printf("pid_bwd_eul,mean_time(s):%.9f\n", pid_bwd_eul_mean_time);

    // ---------------------------------------------------------------------------
    // STATE SPACE TESTING
    // ---------------------------------------------------------------------------
    for (uint16_t i = 0; i < 9; ++i)
    {
        state_dim += 1;

        printf("\n\n----------------------------------------------------------------------\n");
        printf("n = %i\n", state_dim);
        printf("----------------------------------------------------------------------\n");

        matf32_init(&state, state_dim, 1, state_data);
        matf32_init(&A_fwd_eul, state_dim, state_dim, A_fwd_eul_list[i]);
        matf32_init(&B_fwd_eul, state_dim, 1, B_fwd_eul_list[i]);

        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);
        
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);
        
        // TODO: Change this C and D to C_lti and D_lti to avoid confusing it with the pendulum's C and D.
        matf32_init(&C, 1, state_dim, C_data);
        matf32_init(&D, C.num_rows, B_bwd_eul.num_cols, D_data);

        ctr_sys_lti_init(&sys_fwd_eul, &state, &A_fwd_eul, &B_fwd_eul, &C, &D, sample_time);
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

        //printf("\n--------------------------------------------------\n");
        printf("\n\n--------------- c2d - Forward Euler ---------------\n\n");
        //printf("--------------------------------------------------\n");

        float c2d_fwd_eul_time[t];
        float c2d_fwd_eul_mean_time = 0;

        // As ctr_c2d overwrites A and B, to test it 100 times with the same values, it's either
        // to reset the value to the original each loop, or test 100 random matrices and not worry about the result vs matlab when measuring time.
        // Either way, sys_lti.is_continuous must be set to true each cycle in order to be able to operate with ctr_c2d (outside the time measurement).
        /*for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_fwd_eul,0,1);
            matf32_randn(&B_fwd_eul,0,1);
            sys_fwd_eul.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);
            c2d_fwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }*/

        // Uncomment this section and comment the loop to test a single operation to compare results with matlab
        sys_fwd_eul.is_continuous = true;
        status = ctr_c2d(&sys_fwd_eul, sample_time, FWD_EULER);

        c2d_fwd_eul_mean_time = mean(c2d_fwd_eul_time, t);

        bool c2d_fwd_eul_Ad_ans = matf32_is_equal(&A_fwd_eul, &R_c2d_fwd_eul_Ad);
        bool c2d_fwd_eul_Bd_ans = matf32_is_equal(&B_fwd_eul, &R_c2d_fwd_eul_Bd);
        float cond_A_fwd_eul = 0;
        float cond_B_fwd_eul = 0;
        matf32_cond(&A_fwd_eul, &cond_A_fwd_eul);
        matf32_cond(&B_fwd_eul, &cond_B_fwd_eul);
        printf("Condition number A: %.9f, B: %.9f\n", cond_A_fwd_eul, cond_B_fwd_eul);
        printf("c2d_fwd_eul,Ad:%s,Bd:%s\n", c2d_fwd_eul_Ad_ans?"success":"failure", c2d_fwd_eul_Bd_ans?"success":"failure"); // Results comparison
        //printf("c2d_fwd_eul%i\n", state_dim);
        //printf("%.9f\n", c2d_fwd_eul_mean_time);
        //printf("c2d_fwd_eul,mean_time(s):%.9f\n", c2d_fwd_eul_mean_time); // Time measurement

        if (!c2d_fwd_eul_Ad_ans || !c2d_fwd_eul_Bd_ans)
        {
            printf("Ad result difference:\n");
            matf32_sub(&A_fwd_eul, &R_c2d_fwd_eul_Ad, &A_fwd_eul);
            matf32_print(&A_fwd_eul);
            printf("Bd result difference:\n");
            matf32_sub(&B_fwd_eul, &R_c2d_fwd_eul_Bd, &B_fwd_eul);
            matf32_print(&B_fwd_eul);
        }
            
        // ---------------------------------------------------------------------------
        // ctr_c2d - Backward Euler
        // ---------------------------------------------------------------------------

        //printf("\n--------------------------------------------------\n");
        printf("\n\n--------------- c2d - Backward Euler ---------------\n\n");
        //printf("--------------------------------------------------\n");

        // Initialize again A and B
        matf32_init(&A_bwd_eul, state_dim, state_dim, A_bwd_eul_list[i]);
        matf32_init(&B_bwd_eul, state_dim, 1, B_bwd_eul_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_bwd_eul_Ad, A_bwd_eul.num_rows, A_bwd_eul.num_cols, R_c2d_bwd_eul_list_Ad[i]);
        matf32_init(&R_c2d_bwd_eul_Bd, B_bwd_eul.num_rows, B_bwd_eul.num_cols, R_c2d_bwd_eul_list_Bd[i]);

        float c2d_bwd_eul_time[t];
        float c2d_bwd_eul_mean_time = 0;

        /*for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_bwd_eul,0,1);
            matf32_randn(&B_bwd_eul,0,1);
            sys_bwd_eul.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);
            c2d_bwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }*/

        sys_bwd_eul.is_continuous = true;
        status = ctr_c2d(&sys_bwd_eul, sample_time, BWD_EULER);

        c2d_bwd_eul_mean_time = mean(c2d_bwd_eul_time, t);

        bool c2d_bwd_eul_Ad_ans = matf32_is_equal(&A_bwd_eul, &R_c2d_bwd_eul_Ad);
        bool c2d_bwd_eul_Bd_ans = matf32_is_equal(&B_bwd_eul, &R_c2d_bwd_eul_Bd);
        float cond_A_bwd_eul = 0;
        float cond_B_bwd_eul = 0;
        matf32_cond(&A_bwd_eul, &cond_A_bwd_eul);
        matf32_cond(&B_bwd_eul, &cond_B_bwd_eul);
        printf("Condition number A: %.9f, B: %.9f\n", cond_A_bwd_eul, cond_B_bwd_eul);
        printf("c2d_bwd_eul,Ad:%s,Bd:%s\n", c2d_bwd_eul_Ad_ans?"success":"failure", c2d_bwd_eul_Bd_ans?"success":"failure"); // Results comparison
        //printf("c2d_bwd_eul%i\n", state_dim);
        //printf("%.9f\n", c2d_bwd_eul_mean_time);
        //printf("c2d_bwd_eul,mean_time(s):%.9f\n", c2d_bwd_eul_mean_time); // Time measurement

        if (!c2d_bwd_eul_Ad_ans || !c2d_bwd_eul_Bd_ans)
        {
            printf("\nAd result difference:\n");
            matf32_sub(&A_bwd_eul, &R_c2d_bwd_eul_Ad, &A_bwd_eul);
            matf32_print(&A_bwd_eul);
            printf("Bd result difference:\n");
            matf32_sub(&B_bwd_eul, &R_c2d_bwd_eul_Bd, &B_bwd_eul);
            matf32_print(&B_bwd_eul);
        }

        // ---------------------------------------------------------------------------
        // ctr_c2d - Tustin
        // ---------------------------------------------------------------------------

        //printf("\n--------------------------------------------------\n");
        printf("\n\n--------------- c2d - Tustin ---------------\n\n");
        //printf("--------------------------------------------------\n");

        // Initialize again A and B
        matf32_init(&A_tustin, state_dim, state_dim, A_tustin_list[i]);
        matf32_init(&B_tustin, state_dim, 1, B_tustin_list[i]);

        // Init MATLAB result matrices
        matf32_init(&R_c2d_tustin_Ad, A_tustin.num_rows, A_tustin.num_cols, R_c2d_tustin_list_Ad[i]);
        matf32_init(&R_c2d_tustin_Bd, B_tustin.num_rows, B_tustin.num_cols, R_c2d_tustin_list_Bd[i]);

        float c2d_tustin_time[t];
        float c2d_tustin_mean_time = 0;

        /*for (uint8_t i = 0; i < t; i++)
        {
            matf32_randn(&A_tustin,0,1);
            matf32_randn(&B_tustin,0,1);
            sys_tustin.is_continuous = true;
            time = clock();
            status = ctr_c2d(&sys_tustin, sample_time, TUSTIN);
            c2d_tustin_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }*/

        sys_tustin.is_continuous = true;
        status = ctr_c2d(&sys_tustin, sample_time, TUSTIN);

        c2d_tustin_mean_time = mean(c2d_tustin_time, t);

        bool c2d_tustin_Ad_ans = matf32_is_equal(&A_tustin, &R_c2d_tustin_Ad);
        bool c2d_tustin_Bd_ans = matf32_is_equal(&B_tustin, &R_c2d_tustin_Bd);
        float cond_A_tustin = 0;
        float cond_B_tustin = 0;
        matf32_cond(&A_tustin, &cond_A_tustin);
        matf32_cond(&B_tustin, &cond_B_tustin);
        printf("Condition number A: %.9f, B: %.9f\n", cond_A_tustin, cond_B_tustin);
        printf("c2d_tustin,Ad:%s,Bd:%s\n", c2d_tustin_Ad_ans?"success":"failure", c2d_tustin_Bd_ans?"success":"failure"); // Results comparison
        //printf("c2d_tustin%i\n", state_dim);
        //printf("%.9f\n", c2d_tustin_mean_time);
        //printf("c2d_tustin,mean_time(s):%.9f\n", c2d_tustin_mean_time); // Time measurement
    
        if (!c2d_tustin_Ad_ans || !c2d_tustin_Bd_ans)
        {
            printf("Ad result difference:\n");
            matf32_sub(&A_tustin, &R_c2d_tustin_Ad, &A_tustin);
            matf32_print(&A_tustin);
            printf("Bd result difference:\n");
            matf32_sub(&B_tustin, &R_c2d_tustin_Bd, &B_tustin);
            matf32_print(&B_tustin);
        }
    }
        

    // ---------------------------------------------------------------------------
    // NONLINEAR SYSTEMS TESTING
    // ---------------------------------------------------------------------------

    matf32_init(&nonlin_state, 2, 1, nonlin_state_data);
    matf32_init(&sys_state, 2, 1, sys_state_data);
    matf32_init(&sys_input, 1, 1, sys_input_data);
    matf32_init(&xdot, 2, 1, xdot_data);
    matf32_init(&sys_output, 1, 1, sys_output_data);

    ctr_sys_nonlin_init(&nonlin_pendulum, &nonlin_state, nonlin_input_dim, nonlin_output_dim, pendulum_dynamics, pendulum_outputs, sample_time);

    float fss_data[MAX_MAT_SIZE];
    float hss_data[MAX_MAT_SIZE];

    matf32_t fss, hss;

    matf32_init(&fss, 2, 1, fss_data);
    matf32_init(&hss, 1, 1, hss_data);
    
    // ---------------------------------------------------------------------------
    // ctr_linloc vs loclin_fast.m
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("linloc \n");
    printf("--------------------------------------------------\n");

    float delta = 2;

    float lti_state_data[MAX_MAT_SIZE];
    matf32_t lti_state;

    float A_data[MAX_MAT_SIZE];
    float B_data[MAX_MAT_SIZE];
    float C_data[MAX_MAT_SIZE];
    float D_data[MAX_MAT_SIZE];
    float xss_data[MAX_MAT_SIZE];
    float uss_data[MAX_MAT_SIZE];

    matf32_t A, B, C, D, xss, uss;

    matf32_init(&A, 2, 2, A_data);
    matf32_init(&B, 2, 1, B_data);
    matf32_init(&C, 1, 2, C_data);
    matf32_init(&D, 1, 1, D_data);
    matf32_init(&xss, 2, 1, xss_data);
    matf32_init(&uss, 1, 1, uss_data);
    
    matf32_zeros(&A);
    matf32_zeros(&B);
    matf32_zeros(&C);
    matf32_zeros(&D);

    matf32_set(&xss, 1, 1, -1);
    matf32_set(&xss, 2, 1, 5);
    matf32_set(&uss, 1, 1, 2);

    ctr_sys_lti_t linearized_pendulum;
    ctr_sys_lti_init(&linearized_pendulum, &nonlin_state, &A, &B, &C, &D, sample_time);

    // Time measurement for ctr_linloc
    float linloc_time[t];
    float mean_linloc_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_linloc(&nonlin_pendulum, &linearized_pendulum, &xss, &uss, delta);
        linloc_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    } 

    // Time measurement for pendulum_dynamics
    /*float pendulum_dynamics_time[t];
    float mean_pendulum_dynamics_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        pendulum_dynamics(&fss, &xss, &uss);
        pendulum_dynamics_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/

    // Time measurement for pendulum_outputs
    /*float pendulum_outputs_time[t];
    float mean_pendulum_outputs_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        pendulum_outputs(&hss, &xss, &uss);
        pendulum_outputs_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/
        
    // Print linearized system
    ctr_sys_lti_print(&linearized_pendulum);

    // Print time values formatted to a .mat for plotting in matlab
    /*printf("linloc\n");
    for (uint8_t j = 0; j < t; j++)
    {
        printf("%.9f ", linloc_time[j]);
    }
    printf("\n\n");

    printf("pendulum_dynamics\n");
    for (uint8_t j = 0; j < t; j++)
    {
        printf("%.9f ", pendulum_dynamics_time[j]);
    }
    printf("\n\n");

    printf("pendulum_outputs\n");
    for (uint8_t j = 0; j < t; j++)
    {
        printf("%.9f ", pendulum_outputs_time[j]);
    }
    printf("\n\n");*/

    mean_linloc_time = mean(linloc_time, t);
    //mean_pendulum_dynamics_time = mean(pendulum_dynamics_time, t);
    //mean_pendulum_outputs_time = mean(pendulum_outputs_time, t);

    // For comparing results with loclin_fast.m, add an array of random values for x and u and iterate over it.

    printf("linloc,mean_time(s):%.9f\n\n", mean_linloc_time); // Time measurement

    // ---------------------------------------------------------------------------
    // ctr_linear_state_feedback
    // ---------------------------------------------------------------------------

    /*float x_data[MAX_MAT_SIZE];
    float u_data[MAX_MAT_SIZE];
    float K_data[MAX_MAT_SIZE];

    matf32_t x, u, K;

    matf32_init(&x, xss.num_rows, xss.num_cols, x_data);
    matf32_init(&u, uss.num_rows, uss.num_cols, u_data);
    matf32_init(&K, uss.num_rows, x.num_rows, K_data);

    matf32_set(&K, 1, 1, 1);
    matf32_set(&K, 1, 2, 2);
    matf32_set(&x, 1, 1, 7);
    matf32_set(&x, 2, 1, 5);

    float linear_state_feedback_time[t];
    float mean_linear_state_feedback_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_linear_state_feedback(&u, &K, &x, &xss, &uss);
        linear_state_feedback_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    printf("linear_state_feedback\n");
    for (uint8_t j = 0; j < t; j++)
    {
        printf("%.9f ", linear_state_feedback_time[j]);
    }
    printf("\n\n");

    mean_linear_state_feedback_time = mean(linear_state_feedback_time, t);*/

    // ---------------------------------------------------------------------------
    // ctr_sys_nonlin_simulate
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("ctr_sys_nonlin_simulate \n");
    printf("--------------------------------------------------\n");

    float x_k_data[MAX_MAT_SIZE];
    float x_k_1_data[MAX_MAT_SIZE];
    float u_k_data[MAX_MAT_SIZE];

    matf32_t x_k, x_k_1, u_k, R_x_k_1_fwd_eul, R_x_k_1_rk4;

    matf32_init(&x_k, fss.num_rows, fss.num_cols, x_k_data);
    matf32_init(&x_k_1, fss.num_rows, fss.num_cols, x_k_1_data);
    matf32_init(&u_k, fss.num_rows, fss.num_cols, u_k_data);
    matf32_init(&R_x_k_1_fwd_eul, fss.num_rows, fss.num_cols, R_x_k_1_fwd_eul1);
    matf32_init(&R_x_k_1_rk4, fss.num_rows, fss.num_cols, R_x_k_1_rk41);

    matf32_set(&x_k, 1, 1, 4);
    matf32_set(&x_k, 2, 1, 5);
    matf32_set(&u_k, 1, 1, 2);

    printf("\n");

    // ------------------------- Forward euler -------------------------

    // Time measurement for ctr_nonlin_simulate using Forward Euler
    float nonlin_sim_fwd_eul_time[t];
    float mean_nonlin_sim_fwd_eul_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_sys_nonlin_simulate(&nonlin_pendulum, &x_k, &x_k_1, &u_k, delta, FWD_EULER);
        nonlin_sim_fwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    } 

    printf("x_k_1 - Forward Euler:\n");
    matf32_print(&x_k_1);

    mean_nonlin_sim_fwd_eul_time = mean(nonlin_sim_fwd_eul_time, t);

    bool nonlin_sim_fwd_eul_ans = matf32_is_equal(&x_k_1, &R_x_k_1_fwd_eul);
    printf("nonlin_simulate, mean_time(s): %.9f, ForwardEuler: %s\n\n", mean_nonlin_sim_fwd_eul_time, nonlin_sim_fwd_eul_ans?"success":"failure");

    // Print nonlin_simulate Forward Euler time results for plotting in matlab
    //printf("nonlin_simulate_fwd_eul\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", nonlin_sim_fwd_eul_time[j]);
    //}
    //printf("\n\n");

    // ------------------------- Runge-Kutta 4 (RK4) -------------------------

    // Time measurement for ctr_nonlin_simulate using Runge-Kutta4
    float nonlin_sim_rk4_time[t];
    float mean_nonlin_sim_rk4_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        time = clock();
        ctr_sys_nonlin_simulate(&nonlin_pendulum, &x_k, &x_k_1, &u_k, delta, RK4);
        nonlin_sim_rk4_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    printf("x_k_1 - Runge-Kutta4:\n");
    matf32_print(&x_k_1);

    mean_nonlin_sim_rk4_time = mean(nonlin_sim_rk4_time, t);

    bool nonlin_sim_rk4_ans = matf32_is_equal(&x_k_1, &R_x_k_1_rk4);
    printf("nonlin_simulate, mean_time(s): %.9f, RK4: %s\n", mean_nonlin_sim_rk4_time, nonlin_sim_rk4_ans?"success":"failure");

    // Print nonlin_simulate Runge-Kutta4 time results for plotting in matlab
    //printf("nonlin_simulate_rk4\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", nonlin_sim_rk4_time[j]);
    //}
    //printf("\n\n");


    // ---------------------------------------------------------------------------
    // KALMAN FILTER TESTING
    // ---------------------------------------------------------------------------

    float m = 2;

    float kf_A_data[4] = {1,2,3,4};
    float kf_B_data[2] = {1,2};
    float kf_C_data[2] = {1,2};
    float kf_D_data[1] = {0};
    float kf_Qw_data[1] = {0};
    float kf_Qv_data[1] = {1};
    float kf_x_data[MAX_MAT_SIZE];
    float kf_u_data[1] = {0};
    float kf_P_data[4] = {5,6,7,8};
    float kf_F_data[2] = {1,2};
    float kf_ye_data[MAX_MAT_SIZE];
    float kf_y_data[MAX_MAT_SIZE];
    float kf_xhat_data[MAX_MAT_SIZE];
    float kf_meas_data[MAX_MAT_SIZE];

    matf32_t kf_A, kf_B, kf_C, kf_D, kf_Qw, kf_Qv, kf_x, kf_u, kf_P, kf_F, kf_xhat, kf_ye, kf_y;

    matf32_init(&state, m, 1, state_data);
    matf32_init(&kf_A, m, m, kf_A_data);
    matf32_init(&kf_B, m, 1, kf_B_data);
    matf32_init(&kf_C, 1, m, kf_C_data);
    matf32_init(&kf_D, 1, 1, kf_D_data);

    ctr_sys_lti_init(&kf_sys, &state, &kf_A, &kf_B, &kf_C, &kf_D, sample_time);
    //ctr_sys_lti_print(&kf_sys);
    ctr_c2d(&kf_sys, sample_time, PURE_DISCRETE);

    matf32_init(&kf_F, 2, 1, kf_F_data);
    matf32_init(&kf_Qw, 1, 1, kf_Qw_data);
    matf32_init(&kf_Qv, 1, 1, kf_Qv_data);
    matf32_init(&kf_xhat, 2, 1, kf_xhat_data);
    matf32_init(&kf_P, kf_A.num_rows, kf_A.num_cols, kf_P_data);
    matf32_init(&kf_ye, kf_xhat.num_rows, kf_xhat.num_cols, kf_ye_data);

    // Initial state x0
    matf32_set(&kf_xhat, 1, 1, 1);
    matf32_set(&kf_xhat, 2, 1, 1);

    ctr_kalman_init(&kf, &kf_sys, &kf_F, &kf_Qw, &kf_Qv, &kf_xhat, &kf_P);
    //ctr_kalman_print(&kf);

    matf32_init(&kf_u, kf_sys.input_dim, 1, kf_u_data);
    matf32_init(&kf_y, 1, 1, kf_y_data);
    
    // Initial measurement
    matf32_mul(&kf_C, &kf_xhat, &kf_y);

    //printf("kf_u:\n");
    //matf32_print(&kf_u);
    //printf("kf_y:\n");
    //matf32_print(&kf_y);

    // ---------------------------------------------------------------------------
    // ctr_kalman_predict
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("Kalman Filter - Predict \n");
    printf("--------------------------------------------------\n");

    /*float kalman_predict_time[t];
    float kalman_predict_mean_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        // Reset values before measuring to get the same result, just for the sake of time measurements
        matf32_set(&kf_xhat, 1, 1, 1);
        matf32_set(&kf_xhat, 2, 1, 1);
        kf_P.p_data[0] = 5;
        kf_P.p_data[1] = 6;
        kf_P.p_data[2] = 7;
        kf_P.p_data[3] = 8;
        time = clock();
        ctr_kalman_predict(&kf, &kf_u);
        kalman_predict_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/

    // Uncomment this to test once
    ctr_kalman_predict(&kf, &kf_u);
    //ctr_kalman_print(&kf);
    printf("\nKalman predict result:\n");
    matf32_print(kf.xhat);
    matf32_print(kf.P);
    float cond_P = 0;
    matf32_cond(kf.P, &cond_P);
    printf("Condition number P: %.9f\n", cond_P);

    // Print kalman_predict time results for plotting in matlab
    //printf("kalman_predict\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", kalman_predict_time[j]);
    //}
    //printf("\n\n");

    //kalman_predict_mean_time = mean(kalman_predict_time, t);

    //bool kalman_predict_ans = matf32_is_equal(F_AB.p_T, &R_kalman_predict);
    //printf("kalman_predict          ,%s,mean_time(s): %.9f\n", kalman_predict_ans?"success":"failure", kalman_predict_mean_time);
    //printf("kalman_predict,mean_time(s):%.9f\n", kalman_predict_mean_time);

    // ---------------------------------------------------------------------------
    // ctr_kalman_correct
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("Kalman Filter - Correct\n");
    printf("--------------------------------------------------\n");

    /*float kalman_correct_time[t];
    float kalman_correct_mean_time = 0;
    for (uint8_t i = 0; i < t; i++)
    {
        // Reset values before measuring to get the same result, just for the sake of time measurements
        matf32_set(&kf_xhat, 1, 1, 1);
        matf32_set(&kf_xhat, 2, 1, 1);
        kf_P.p_data[0] = 5;
        kf_P.p_data[1] = 6;
        kf_P.p_data[2] = 7;
        kf_P.p_data[3] = 8;
        time = clock();
        status = ctr_kalman_correct(&kf, &kf_y);
        kalman_correct_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }*/

    // Uncomment to test once
    ctr_kalman_correct(&kf, &kf_y);
    //ctr_kalman_print(&kf);
    printf("\nKalman correct result:\n");
    matf32_print(kf.xhat);
    matf32_print(kf.P);
    matf32_cond(kf.P, &cond_P);
    printf("Condition number P: %.9f\n", cond_P);

    // Print kalman_correct time results for plotting in matlab
    //printf("kalman_correct\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", kalman_correct_time[j]);
    //}
    //printf("\n\n");

    //kalman_correct_mean_time = mean(kalman_correct_time, t);

    //bool kalman_correct_ans = matf32_is_equal(F_AB.p_T, &R_kalman_correct);
    //printf("kalman_correct          ,%s,mean_time(s): %.9f\n", kalman_correct_ans?"success":"failure", kalman_correct_mean_time);
    //printf("kalman_correct,mean_time(s):%.9f\n", kalman_correct_mean_time);
    
    printf("\n");
}

err_status_t pendulum_dynamics(matf32_t* const xdot, const matf32_t* sys_state, const matf32_t* sys_input)
{
    /**
     * Simple Pendulum Dynamics:
     * 
     * xdot = f(x,u) = |              x2             |
     *                 | -(g/l)sin(x1) + (1/m(l^2))u |
     * 
     */

    float m = 2;    // kg
    float l = 1;    // m
    float g = 9.81;  // m/s^2

    float x1 = sys_state->p_data[0];
    float x2 = sys_state->p_data[1];
    float u = sys_input->p_data[0];

    matf32_set(xdot, 1, 1, x2); // x2
    //printf("xdot(1): %.9f\n", xdot->p_data[0]);
    matf32_set(xdot, 2, 1, (-1.0*(g/l)*sinf(x1)) + ((1/m*(l*l))*u)); // -(g/l)sin(x1) + (1/m(l^2))u
    //printf("xdot(2): %.9f\n", xdot->p_data[1]);
    return MATH_SUCCESS;
}

err_status_t pendulum_outputs(matf32_t* const sys_output, const matf32_t* sys_state, const matf32_t* sys_input)
{
    /**
     * Simple Pendulum Output:
     * 
     * y = h(x,u) = arctan(x1)
     */

    float x1 = sys_state->p_data[0];

    matf32_set(sys_output, 1, 1, atanf(x1)); // y = h(x,u) = arctan(x1)
    //printf("y(1): %.9f\n", sys_output->p_data[0]);

    return MATH_SUCCESS;
}