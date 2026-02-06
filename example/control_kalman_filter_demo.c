
/**
 * @author Andrea Pineda
 * @date Created 6 Jan. 2026
 * Last Modified 6 Jan. 2026
 * 
 * Kalman Filter demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "robotat_control.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define K (100) // Number of iterations for the Kalman Filter

// ---------------------------------------------------------------------------
// LTI system definition
// ---------------------------------------------------------------------------

float sample_time = 0.1;

float A_data[1] = {1};
float B_data[1] = {1};
float C_data[1] = {1};
float D_data[1] = {0};
float u_k_data[MAX_MAT_SIZE];
float x_k_data[MAX_MAT_SIZE];
float y_k_data[MAX_MAT_SIZE];

float Ax_data[MAX_MAT_SIZE];
float Bu_data[MAX_MAT_SIZE];
float Fw_data[MAX_MAT_SIZE];

matf32_t A, B, C, D, u_k, x_k, y_k; // LTI system matrices
matf32_t Ax, Bu, Fw; // Extra matrices for updating the LTI system

ctr_sys_lti_t sys;

// ---------------------------------------------------------------------------
// Kalman Filter definition
// ---------------------------------------------------------------------------

float Qw_data[1] = {0};
float Qv_data[1] = {1};
float w_data[1];
float v_data[1];
float P_data[1] = {1};
float F_data[1] = {1};
float xhat_data[MAX_MAT_SIZE];

matf32_t Qw, Qv, w, v; // Noise matrices
matf32_t xhat, P, F; // Kalman Filter matrices

ctr_kalman_t kf;

// ---------------------------------------------------------------------------
// Data arrays for storing trajectories
// ---------------------------------------------------------------------------

float XHAT[K];
float X[K];
float U[K];
float Y[K];

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // ---------------------------------------------------------------------------
    // Initialization and initial values
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
    matf32_init(&Ax, A.num_rows, x_k.num_cols, Ax_data);
    matf32_init(&Bu, B.num_rows, u_k.num_cols, Bu_data);
    matf32_init(&Fw, F.num_rows, w.num_cols, Fw_data);

    // Initial values
    matf32_set(&x_k, 1, 1, 1);
    matf32_set(&u_k, 1, 1, 0);
    matf32_mul(&C, &x_k, &y_k);
    matf32_zeros(&xhat);

    // Initialize LTI system and Kalman Filter
    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, sample_time);
    ctr_kalman_init(&kf, &sys, &F, &Qw, &Qv, &xhat, &P);

    // Save initial trajectories
    XHAT[0] = xhat.p_data[0];
    X[0] = x_k.p_data[0];
    U[0] = u_k.p_data[0];
    Y[0] = y_k.p_data[0];

    // ---------------------------------------------------------------------------
    // Kalman Filter Main Loop
    // ---------------------------------------------------------------------------
    for (uint16_t k = 1; k < K; ++k)
    {
        // u = -0.1*xhat
        matf32_copy(&xhat, &u_k); // u = xhat
        matf32_scale(&u_k, -0.1, &u_k); // u = -0.1*x_hat

        // Generate noise
        matf32_randn(&w, 0, 1);
        matf32_randn(&v, 0, 1);

        // Update LTI system
        matf32_mul(&A, &x_k, &Ax); // A*x
        matf32_mul(&B, &u_k, &Bu); // B*u
        matf32_mul(&F, &w, &Fw);
        matf32_add(&Ax, &Bu, &x_k); // x = A*x + B*u
        matf32_add(&x_k, &Fw, &x_k); // x = A*x + B*u + F*w

        matf32_mul(&C, &x_k, &y_k); // y = C*x
        matf32_add(&y_k, &v, &y_k); // y = C*x + v

        // Update Kalman Filter
        ctr_kalman_predict(&kf, &u_k); // Prediction: xhat = xhat_prior, P = P_prior
        ctr_kalman_correct(&kf, &y_k); // Correction: xhat = xhat_post, P = P_post

        // Save trajectories
        XHAT[k] = xhat.p_data[0];
        X[k] = x_k.p_data[0];
        U[k] = u_k.p_data[0];
        Y[k] = y_k.p_data[0];
    }

    // ---------------------------------------------------------------------------
    // Print trajectories
    // ---------------------------------------------------------------------------

    // Print xhat
    printf("\n\nkalman_xhat\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", XHAT[k]);
    }
    printf("\n\n");

    // Print x
    printf("\n\nkalman_state\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", X[k]);
    }
    printf("\n\n");

    // Print u
    printf("\n\nkalman_u\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", U[k]);
    }
    printf("\n\n");

    // Print y
    printf("\n\nkalman_y\n");
    for (uint8_t k = 0; k < K; k++)
    {
        printf("%.9f ", Y[k]);
    }
    printf("\n\n");
}
