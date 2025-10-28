
/**
 * @author: Andrea Pineda
 * @date: Created 28 Oct 2025, Last Modified 28 Oct 2025
 * 
 * For testing nonlinear systems functions of robotat_control in computer
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "robotat_control.h"
#include "control_nonlin_sys_test_data.h"

// --------------------------------------------------
// Definitions
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

float sample_time = 0.001;

// --------------------------------------------------
// Function declarations
// --------------------------------------------------

err_status_t pendulum_dynamics(matf32_t* const xdot, const matf32_t* sys_state, const matf32_t* sys_input);
err_status_t pendulum_outputs(matf32_t* const sys_output, const matf32_t* sys_state, const matf32_t* sys_input);

// --------------------------------------------------
// Main
// --------------------------------------------------
int main(void)
{
    clock_t time;
    err_status_t status;

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
    float linloc_time[samples];
    float mean_linloc_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        ctr_linloc(&nonlin_pendulum, &linearized_pendulum, &xss, &uss, delta);
        linloc_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    } 
    mean_linloc_time = mean(linloc_time, samples);

    // Time measurement for pendulum_dynamics
    float pendulum_dynamics_time[samples];
    float mean_pendulum_dynamics_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        pendulum_dynamics(&fss, &xss, &uss);
        pendulum_dynamics_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    mean_pendulum_dynamics_time = mean(pendulum_dynamics_time, samples);

    // Time measurement for pendulum_outputs
    float pendulum_outputs_time[samples];
    float mean_pendulum_outputs_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        pendulum_outputs(&hss, &xss, &uss);
        pendulum_outputs_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    mean_pendulum_outputs_time = mean(pendulum_outputs_time, samples);
        
    // Print linearized system
    printf("\n-------------------------\n");
    printf("Linearized Pendulum:\n");
    printf("-------------------------\n");
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

    // For comparing results with loclin_fast.m, add an array of random values for x and u and iterate over it.

    printf("linloc,mean_time(s):%.9f\n\n", mean_linloc_time); // Time measurement

    // ---------------------------------------------------------------------------
    // ctr_linear_state_feedback
    // ---------------------------------------------------------------------------

    // Add print and comparison to matlab's result

    float x_data[MAX_MAT_SIZE];
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

    float linear_state_feedback_time[samples];
    float mean_linear_state_feedback_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        ctr_linear_state_feedback(&u, &K, &x, &xss, &uss);
        linear_state_feedback_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    mean_linear_state_feedback_time = mean(linear_state_feedback_time, samples);

    //printf("linear_state_feedback\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", linear_state_feedback_time[j]);
    //}
    //printf("\n\n");

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
    float nonlin_sim_fwd_eul_time[samples];
    float mean_nonlin_sim_fwd_eul_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        ctr_sys_nonlin_simulate(&nonlin_pendulum, &x_k, &x_k_1, &u_k, delta, FWD_EULER);
        nonlin_sim_fwd_eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    } 

    printf("x_k_1 - Forward Euler:\n");
    matf32_print(&x_k_1);

    mean_nonlin_sim_fwd_eul_time = mean(nonlin_sim_fwd_eul_time, samples);

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
    float nonlin_sim_rk4_time[samples];
    float mean_nonlin_sim_rk4_time = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        time = clock();
        ctr_sys_nonlin_simulate(&nonlin_pendulum, &x_k, &x_k_1, &u_k, delta, RK4);
        nonlin_sim_rk4_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    printf("x_k_1 - Runge-Kutta4:\n");
    matf32_print(&x_k_1);

    mean_nonlin_sim_rk4_time = mean(nonlin_sim_rk4_time, samples);

    bool nonlin_sim_rk4_ans = matf32_is_equal(&x_k_1, &R_x_k_1_rk4);
    printf("nonlin_simulate, mean_time(s): %.9f, RK4: %s\n", mean_nonlin_sim_rk4_time, nonlin_sim_rk4_ans?"success":"failure");

    // Print nonlin_simulate Runge-Kutta4 time results for plotting in matlab
    //printf("nonlin_simulate_rk4\n");
    //for (uint8_t j = 0; j < t; j++)
    //{
    //    printf("%.9f ", nonlin_sim_rk4_time[j]);
    //}
    //printf("\n\n");
}

// --------------------------------------------------
// Functions
// --------------------------------------------------
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


