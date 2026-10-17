
/**
 * @author Andrea Pineda
 * @date Created 29 Oct 2025, last modified 17 Oct 2026
 * 
 * Testing the mpc functions in the Arduino Mega
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "matf32.h"
#include "linsolve.h"
#include "quadprog.h"
#include "robotat_control.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------
#define N (4) // Horizon length
#define ROWS (2)
#define COLS (1)

float A_data[ROWS*ROWS] = {1.1, 2.0, 0.0, 0.95};
matf32_t A;

float B_data[ROWS*COLS] = {0.0, 0.0787};
matf32_t B;

float C_data[COLS*ROWS] = {-1, 1};
matf32_t C;

float D_data[1];
matf32_t D;

float u_k_data[N];
matf32_t u_k;

float x_k_data[N];
matf32_t x_k;

float qp_Q_data[N*ROWS*N*ROWS];
matf32_t qp_Q;

float qp_c_data[N*ROWS];
matf32_t qp_c;

// Memory stack and pointer arrays for MPC matrix M
float M1_data[ROWS*ROWS];
matf32_t M1;
float M2_data[ROWS*ROWS];
matf32_t M2;
float M3_data[ROWS*ROWS];
matf32_t M3;
float M4_data[ROWS*ROWS];
matf32_t M4;

float* mpc_M_data[N] = {M1_data, M2_data, M3_data, M4_data};
//float* mpc_M_data[N] = {M1_data, M2_data, M3_data, M4_data, M5_data, M6_data, M7_data, M8_data};
matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4}; // Fix matrix names inside quadprog to avoid conflict with naming this M.
//matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4, &M5, &M6, &M7, &M8}; // Fix matrix names inside quadprog to avoid conflict with naming this M.

// Memory stack and pointer arrays for MPC convolution matrix C
float AB_data[ROWS*COLS];
matf32_t AB;
float A2B_data[ROWS*COLS];
matf32_t A2B;
float A3B_data[ROWS*COLS];
matf32_t A3B;
float C_null_mat_data[ROWS*COLS];
matf32_t C_null_mat;

// Put C matrices in an array to iterate over them
float* mpc_C_stack_data[N+1] = {B_data, AB_data, A2B_data, A3B_data, C_null_mat_data};
matf32_t* mpc_C_stack[N+1] = {&B, &AB, &A2B, &A3B, &C_null_mat};
//matf32_t* mpc_C_stack[N+1] = {&B, &AB, &A2B, &A3B, &A4B, &A5B, &A6B, &A7B, &C_null_mat};
matf32_t* mpc_C[N*N]; // Matrix of matrices C. Don't initialize yet.

float mpc_Q_data[ROWS*ROWS];
matf32_t mpc_Q;

float mpc_R_data[COLS*COLS];
matf32_t mpc_R;

//float U[2*N];
//float cost[2*N];
//float X1[2*N];
//float X2[2*N];

//float utQ_data[N*ROWS];
//matf32_t utQ;

//float utQu_data[COLS*COLS];
//matf32_t utQu;

//float ut_data[COLS*N];
//matf32_t ut;

//float ct_data[COLS*N*COLS];
//matf32_t ct;

//float ctu_data[COLS*N];
//matf32_t ctu;

bool state_constraints = false;

quadprog_t qp;
ctr_sys_lti_t sys;
ctr_mpc_lti_shooting_t mpc;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup()
{
    // ---------------------------------------------------------------------------
    // Serial config
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // Testing
    // ---------------------------------------------------------------------------

    matf32_init(&A, ROWS, ROWS, A_data);
    matf32_init(&B, ROWS, COLS, B_data);
    matf32_init(&C, COLS, ROWS, C_data);
    matf32_init(&D, COLS, COLS, D_data);
    matf32_init(&x_k, ROWS, COLS, x_k_data);
    matf32_init(&u_k, N, COLS, u_k_data);
    matf32_init(&qp_Q, N*COLS, N*COLS, qp_Q_data);
    matf32_init(&qp_c, N*COLS, COLS, qp_c_data);
    matf32_init(&mpc_R, COLS, COLS, mpc_R_data);
    matf32_init(&mpc_Q, ROWS, ROWS, mpc_Q_data);

    //matf32_init(&ut, u_k.num_cols, u_k.num_rows, ut_data);
    //matf32_init(&utQ, ut.num_rows, qp_Q.num_cols, utQ_data);
    //matf32_init(&utQu, ut.num_rows, u_k.num_cols, utQu_data);
    //matf32_init(&ct, qp_c.num_cols, qp_c.num_rows, ct_data);
    //matf32_init(&ctu, ct.num_rows, u_k.num_cols, ctu_data);

    matf32_set(&mpc_Q, 1, 1, 1);
    matf32_set(&mpc_Q, 1, 2, -1);
    matf32_set(&mpc_Q, 2, 1, -1);
    matf32_set(&mpc_Q, 2, 2, 1);

    matf32_set(&mpc_R, 1, 1, 0.01);

    matf32_ones(&x_k);

    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, 0.01); // x_k_1 = A*x_k + B*u_k, y_k_1 = C*x_k + D*u_k
    ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, NULL, NULL, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    ctr_mpc_set_M(&mpc, mpc_M_data); // Initialize and define the matrices for mpc_M
    ctr_mpc_set_C(&mpc, mpc_C_stack, mpc_C_stack_data); // Initialize and define matrices and pointer array for mpc_C
    ctr_mpc_set_qpQ(&mpc, &qp_Q); // Compute values for qp_Q
    ctr_mpc_set_qpc(&mpc, &qp_c); // Compute vales for qp_c

    //mpc.state_constraints = false;
    //ctr_mpc_set_constraints(&mpc, 10, -10);

    //Serial.print("Ain:\n");
    //matf32_print(&Ain);
    //Serial.print("bin:\n");
    //matf32_print(&bin);

    for (uint16_t k = 0; k < 2*N; ++k)
    {
        //Serial.println("-------------------------");
        //Serial.print("Iteration: ");
        //Serial.println(k+1);
        //Serial.println("-------------------------");

        //ctr_mpc_set_qpQ(&mpc, &qp_Q);
        ctr_mpc_set_qpc(&mpc, &qp_c);
        //ctr_mpc_set_constraints(&mpc, 10, -10);
        ctr_mpc_update(&mpc, &qp_Q, &qp_c, &x_k, &u_k);
        //U[k] = u_k.p_data[0];
        //X1[k] = x_k.p_data[0];
        //X2[k] = x_k.p_data[1];

        //Serial.println("x1");
        //Serial.println(x_k.p_data[0]);
        //Serial.println("x2");
        //Serial.println(x_k.p_data[1]);
        //Serial.println("u");
        //Serial.println(u_k.p_data[0]);
        //Serial.println("cost");

        // q = (1/2)*u'Qu + c'u
        //matf32_trans(&u_k, &ut); // u'
        //matf32_mul(&ut, &qp_Q, &utQ); // u'Q
        //matf32_mul(&utQ, &u_k, &utQu); // u'Qu
        //matf32_scale(&utQu, 0.5, &utQu); // (1/2)*u'Qu

        //matf32_trans(&qp_c, &ct); // c'
        //matf32_scale(&ct, -1, &ct); // For some reason the values flipped sign when transposing
        //matf32_mul(&ct, &u_k, &ctu); // c'u
        //matf32_add(&utQu, &ctu, &utQu); // (1/2)*u'Qu + c'u
        //cost[k] = utQu.p_data[0];

        //Serial.println(utQu.p_data[0]);
    }

    //Serial.println("x1");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(X1[k]);
    //    Serial.print(" ");
    //}
    //Serial.println();

    //Serial.println("x2");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(X2[k]);
    //    Serial.print(" ");
    //}
    //Serial.println();

    //Serial.println("u");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(U[k]);
    //    Serial.print(" ");
    //}
    //Serial.println();
//
    //Serial.println("cost");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(cost[k]);
    //    Serial.print(" ");
    //}
    //Serial.println();

    // Mean Time Measurement for solving the MPC
    //float mean_mpc_lti_time = 0;
    //for (uint16_t i = 0; i < 100; ++i)
    //{
    //    matf32_ones(&x_k);
    //    matf32_zeros(&u_k);
    //    time = clock();
    //    for (uint16_t k = 0; k < 2*N; ++k)
    //    {
    //        ctr_mpc_set_qpc(&mpc, &qp_c);
    //        //ctr_mpc_set_constraints(&mpc, 10, -10);
    //        ctr_mpc_update(&mpc, &qp_Q, &qp_c, &x_k, &u_k);
    //    }
    //    mean_mpc_lti_time += (float)(clock()-time)/CLOCKS_PER_SEC;
    //}
    //Serial.print("Mean Time (s): %.9f\n\n", mean_mpc_lti_time/100);
}

void loop()
{

}
