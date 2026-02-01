
/**
 * @author Andrea Pineda
 * @date Created 31 Jan. 2026
 * Last Modified 31 Jan. 2026
 *            By: Andrea Pineda
 * 
 * robotat_control PID controller demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "robotat_control.h"

// ---------------------------------------------------------------------------
// PID and LTI system variables
// ---------------------------------------------------------------------------

#define K (100) // Amount of samples to be used

ctr_pid_t pid;
ctr_sys_lti_t sys;

float dt = 0.001;   // Sample period
float tau = 0.2;    // Tau

float kp = 1;       // Proportional coefficient
float ki = 0.2;     // Integral coefficient
float kd = 0.5;     // Derivative coefficient

float U[K]; // Controlled output trajectory
float X[K]; // State trajectory
float Y[K]; // Output trajectory
float R[K]; // Input signal

// ---------------------------------------------------------------------------
// Matrices and variables definitions
// ---------------------------------------------------------------------------

float A_data[MAX_MAT_SIZE];
matf32_t A;

float B_data[MAX_MAT_SIZE];
matf32_t B;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float x_data[MAX_MAT_SIZE];
matf32_t x;

float y_data[MAX_MAT_SIZE];
matf32_t y;

float u_data[MAX_MAT_SIZE];
matf32_t u;

float Ax_data[MAX_MAT_SIZE];
matf32_t Ax;

float Bu_data[MAX_MAT_SIZE];
matf32_t Bu;

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void)
{
    ctr_sys_lti_init(&sys, &x, &A, &B, &C, &D, dt);

    matf32_init(&A, 1, 1, A_data);
    matf32_init(&B, 1, 1, B_data);
    matf32_init(&C, 1, 1, C_data);
    matf32_init(&x, 1, 1, x_data);
    matf32_init(&y, 1, 1, y_data);
    matf32_init(&u, 1, 1, u_data);

    matf32_init(&Ax, 1, 1, Ax_data);
    matf32_init(&Bu, 1, 1, Bu_data);

    matf32_set(&A, 1, 1, 0.9);
    matf32_set(&B, 1, 1, 0.1);
    matf32_set(&C, 1, 1, 1);

    ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 0);
    pid.tau = tau;
    pid.dt = dt;

    ctr_pid_print(&pid);
    //ctr_gen_impulse(R, K, 25); // Generate impulse signal with impulse at index 25
    ctr_gen_step(R, K, 25); // Generate step signal with step starting at index 25

    // Set initial values
    U[0] = 0;
    X[0] = 0;
    Y[0] = 0;

    // Iterate over the input signal with the PID controller
    for (uint8_t k = 0; k < K-1; k++)
    {
        // Discretize signal
        u.p_data[0] = ctr_pid_update(&pid, R[k], Y[k]);
        
        // Update system
        matf32_mul(&A, &x, &Ax); // A*x
        matf32_mul(&B, &u, &Bu); // B*u
        matf32_add(&Ax, &Bu, &x); // x = A*x + B*u

        matf32_mul(&C, &x, &y); // y = C*x

        // Save new data
        U[k+1] = u.p_data[0];
        X[k+1] = x.p_data[0];
        Y[k+1] = y.p_data[0];
    }

    // Print U
    printf("\n\nU\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", U[k]);
    }
    printf("\n\n");

    // Print X
    printf("\n\nX\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", X[k]);
    }
    printf("\n\n");

    // Print Y
    printf("\n\nY\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", Y[k]);
    }
    printf("\n\n");
}

