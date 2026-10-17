/**
 * @author Andrea Pineda
 * @date created 7 Nov 2025, last modified 17 Oct 2026
 * 
 * To test the mpc functions in the STM NUCLEO F446RE
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

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

static float tmpmat1_data[MAX_MAT_SIZE];
static matf32_t tmpmat1;

float A_data[ROWS*ROWS] = {1.1, 2.0, 0.0, 0.95};
matf32_t A;

float B_data[ROWS*COLS] = {0.0, 0.0787};
matf32_t B;

float C_data[COLS*ROWS] = {-1, 1};
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float u_k_data[MAX_MAT_SIZE];
matf32_t u_k;

float x_k_data[MAX_MAT_SIZE];
matf32_t x_k;

float qp_Q_data[MAX_MAT_SIZE];
matf32_t qp_Q;

float qp_c_data[MAX_MAT_SIZE];
matf32_t qp_c;

float Ain_data[MAX_MAT_SIZE];
matf32_t Ain;

float bin_data[MAX_MAT_SIZE];
matf32_t bin;

// Memory stack and pointer arrays for MPC matrix M
float M1_data[MAX_MAT_SIZE];
matf32_t M1;
float M2_data[MAX_MAT_SIZE];
matf32_t M2;
float M3_data[MAX_MAT_SIZE];
matf32_t M3;
float M4_data[MAX_MAT_SIZE];
matf32_t M4;

float* mpc_M_data[N] = {M1_data, M2_data, M3_data, M4_data};
matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4}; // Fix matrix names inside quadprog to avoid conflict with naming this M.

// Memory stack and pointer arrays for MPC convolution matrix C
float AB_data[MAX_MAT_SIZE];
matf32_t AB;
float A2B_data[MAX_MAT_SIZE];
matf32_t A2B;
float A3B_data[MAX_MAT_SIZE];
matf32_t A3B;
float C_null_mat_data[MAX_MAT_SIZE];
matf32_t C_null_mat;

// Put C matrices in an array to iterate over them
float* mpc_C_stack_data[N+1] = {B_data, AB_data, A2B_data, A3B_data, C_null_mat_data};
matf32_t* mpc_C_stack[N+1] = {&B, &AB, &A2B, &A3B, &C_null_mat};
matf32_t* mpc_C[N*N]; // Matrix of matrices C. Don't initialize yet.

float mpc_Q_data[MAX_MAT_SIZE];
matf32_t mpc_Q;

float mpc_R_data[MAX_MAT_SIZE];
matf32_t mpc_R;

float U[2*N];
float cost[2*N];
float X1[2*N];
float X2[2*N];

float utQ_data[MAX_MAT_SIZE];
matf32_t utQ;

float utQu_data[MAX_MAT_SIZE];
matf32_t utQu;

float ut_data[MAX_MAT_SIZE];
matf32_t ut;

float ct_data[MAX_MAT_SIZE];
matf32_t ct;

float ctu_data[MAX_MAT_SIZE];
matf32_t ctu;

bool state_constraints = false;

quadprog_t qp;
ctr_sys_lti_t sys;
ctr_mpc_lti_shooting_t mpc;

float sample_time = 0.01;

void print_matrix(float* matrix, uint16_t rows, uint16_t cols);

unsigned long tic = 0;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup()
{
    clock_t time;
    err_status_t status;

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
    matf32_init(&Ain, N, N, Ain_data);
    matf32_init(&bin, N, COLS, bin_data);
    matf32_init(&qp_Q, N*COLS, N*COLS, qp_Q_data);
    matf32_init(&qp_c, N*COLS, COLS, qp_c_data);
    matf32_init(&mpc_R, COLS, COLS, mpc_R_data);
    matf32_init(&mpc_Q, ROWS, ROWS, mpc_Q_data);

    matf32_init(&ut, u_k.num_cols, u_k.num_rows, ut_data);
    matf32_init(&utQ, ut.num_rows, qp_Q.num_cols, utQ_data);
    matf32_init(&utQu, ut.num_rows, u_k.num_cols, utQu_data);
    matf32_init(&ct, qp_c.num_cols, qp_c.num_rows, ct_data);
    matf32_init(&ctu, ct.num_rows, u_k.num_cols, ctu_data);

    matf32_set(&mpc_Q, 1, 1, 1);
    matf32_set(&mpc_Q, 1, 2, -1);
    matf32_set(&mpc_Q, 2, 1, -1);
    matf32_set(&mpc_Q, 2, 2, 1);

    matf32_set(&mpc_R, 1, 1, 0.01);

    matf32_ones(&x_k);

    //Serial.println("------------------------------ LTI System Matrices ------------------------------");
    Serial.println("A:");
    print_matrix(A.p_data, A.num_cols, A.num_rows);
    Serial.println("B:");
    print_matrix(B.p_data, B.num_cols, B.num_rows);
    Serial.println("C:");
    print_matrix(C.p_data, C.num_cols, C.num_rows);

    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, sample_time); // x_k_1 = A*x_k + B*u_k, y_k_1 = C*x_k + D*u_k
    //ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, NULL, NULL, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, &Ain, &bin, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    ctr_mpc_set_M(&mpc, mpc_M_data); // Initialize and define the matrices for mpc_M
    ctr_mpc_set_C(&mpc, mpc_C_stack, mpc_C_stack_data); // Initialize and define matrices and pointer array for mpc_C
    ctr_mpc_set_qpQ(&mpc, &qp_Q); // Compute values for qp_Q
    ctr_mpc_set_qpc(&mpc, &qp_c); // Compute vales for qp_c

    mpc.state_constraints = true;
    ctr_mpc_set_constraints(&mpc, 10, -10);

    Serial.println("Ain:");
    print_matrix(Ain.p_data, Ain.num_cols, Ain.num_rows);
    Serial.println("bin:");
    print_matrix(bin.p_data, bin.num_cols, bin.num_rows);

    Serial.println("----------------------------------------------------------------------");
    Serial.println("MPC Solution");
    Serial.println("----------------------------------------------------------------------");

    float mpc_time[100];
    float mpc_mean_time = 0;

    for (uint16_t i = 0; i < 100; i++)
    {
      tic = micros();
      for (uint16_t k = 0; k < 2*N; ++k)
      {
          //Serial.println("-------------------------");
          //Serial.print("Iteration: ");
          //Serial.println(k);
          //Serial.println("-------------------------");

          ctr_mpc_set_qpQ(&mpc, &qp_Q);
          ctr_mpc_set_qpc(&mpc, &qp_c);
          ctr_mpc_set_constraints(&mpc, 10, -10);
          ctr_mpc_update(&mpc, &qp_Q, &qp_c, &x_k, &u_k);
          
          //Serial.print("x_k:\n");
          //print_matrix(x_k.p_data, x_k.num_rows, x_k.num_cols);
          //Serial.print("u_k:\n");
          //print_matrix(u_k.p_data, u_k.num_rows, u_k.num_cols);
          
          //U[k] = u_k.p_data[0];
          //X1[k] = x_k.p_data[0];
          //X2[k] = x_k.p_data[1];

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
      }
      mpc_time[i] = (float)((micros() - tic)/1000000.0);
    }

    mpc_mean_time = mean(mpc_time, 2*N);

    // Print time values formatted to a .mat for plotting in matlab
    Serial.println("mpc");
    for (uint8_t j = 0; j < 100; j++)
    {
        Serial.print(mpc_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();

    //Serial.print("MPC mean time (s): ");
    //Serial.println(mpc_mean_time, 9);

    // Print horizontally (in a single line) for plotting in MATLAB, and vertically for visually showing convergence.
    //Serial.println("x1");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(X1[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    //Serial.println("x2");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(X2[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    //Serial.println("u");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(U[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();

    //Serial.println("cost");
    //for (uint16_t k = 0; k < 2*N; ++k)
    //{
    //    Serial.print(cost[k],9);
    //    Serial.print(" ");
    //}
    //Serial.println();
}

void loop()
{
 
}

void print_matrix(float* matrix, uint16_t rows, uint16_t cols)
{
  for (uint16_t i = 0; i < rows; ++i)
  {
    for (uint16_t j = 0; j < cols; ++j)
    {
      Serial.print(matrix[i + j*rows],9);
      Serial.print("  ");
    }
    Serial.println();
  }
  Serial.println();
}
