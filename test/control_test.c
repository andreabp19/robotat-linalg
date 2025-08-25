

/**
 * @author: Andrea Pineda
 * @date: Created 23 Aug 2025, Last Modified 25 Aug 2025
 * 
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

float sample_time = 1;

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

ctr_kalman_t kf;

int main(void)
{
    // ---------------------------------------------------------------------------
    // PID TESTING
    // ---------------------------------------------------------------------------

    ctr_pid_init(&pid, kp, ki, kd, FWD_EULER, 0);
    //ctr_pid_print(&pid);

    // ---------------------------------------------------------------------------
    // STATE SPACE TESTING
    // ---------------------------------------------------------------------------

    matf32_init(&state, state_dim, 1, state_data);
    matf32_init(&A, state_dim, state_dim, A_data);
    matf32_init(&B, state_dim, 1, B_data);
    matf32_init(&C, 1, state_dim, C_data);
    matf32_init(&D, C.num_rows, B.num_cols, D_data);

    matf32_eye(&A);
    matf32_set(&B, 1, 1, 4);
    matf32_set(&B, 2, 1, 5);
    matf32_set(&B, 3, 1, 6);
    matf32_set(&C, 1, 1, 1);
    matf32_set(&C, 1, 2, 2);
    matf32_set(&C, 1, 3, 3);
    matf32_set(&D, 1, 1, 1);
    
    ctr_sys_lti_init(&sys_lti, &state, &A, &B, &C, &D, sample_time);
    ctr_sys_lti_print(&sys_lti);

    // ---------------------------------------------------------------------------
    // KALMAN FILTER TESTING
    // ---------------------------------------------------------------------------

    matf32_init(&F, 1, 1, F_data);
    matf32_init(&Qw, sys_lti.input_dim, 1, Qw_data);
    matf32_init(&Qv, sys_lti.output_dim, 1, Qv_data);
    matf32_init(&xhat, 1, 1, xhat_data);
    matf32_init(&P, A.num_rows, A.num_cols, P_data);

    ctr_kalman_init(&kf, &sys_lti, &F, &Qw, &Qv, &xhat, &P);
    //ctr_kalman_print(&kf);
}