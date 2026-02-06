
/**
 * @author Andrea Pineda
 * @date Created 6 Feb. 2026
 * Last modified 6 Feb. 2026
 * 
 * MPC example
 */

#include "robotat_control.h"

// This demo can be used with a horizon length of 4 or 8, to exemplify usage.
// It's set by default to horizon length = 4, comment/uncomment the indicated
// lines to change to a horizon length of 8.

// Be mindful though, that increasing the horizon length might cause the QP's
// KKT matrix dimensions to surpass 10x10, the matrix size limit configured
// for the library, and the algorithms witll crash if that happens.

// Given that, this example is meant to showcase N = 8 without constraints
// (N = 8 with constraints will crash), and N = 4 either with or without
// constraints. 

#define N (4) // Horizon length
//#define N (8)
#define ROWS (2)
#define COLS (1)

// ---------------------------------------------------------------------------
// LTI system definition
// ---------------------------------------------------------------------------

float sample_time = 0.01;

float A_data[ROWS*ROWS] = {1.1, 2.0, 0.0, 0.95};
float B_data[ROWS*COLS] = {0.0, 0.0787};
float C_data[COLS*ROWS] = {-1, 1};
float D_data[MAX_MAT_SIZE];
float u_k_data[N*COLS];
float x_k_data[ROWS*COLS];

matf32_t A, B, C, D, u_k, x_k;
ctr_sys_lti_t sys;

// ---------------------------------------------------------------------------
// QP definition
// ---------------------------------------------------------------------------

float qp_Q_data[N*COLS*N*COLS];
float qp_c_data[N*COLS];
float Ain_data[N*N];
float bin_data[N*COLS];

matf32_t qp_Q, qp_c, Ain, bin;
quadprog_t qp;

// ---------------------------------------------------------------------------
// MPC definition
// ---------------------------------------------------------------------------

// M prediction matrix:
// M = [A, A^2, ..., A^N] = [M1, M2, ..., MN]

float M1_data[ROWS*ROWS];
float M2_data[ROWS*ROWS];
float M3_data[ROWS*ROWS];
float M4_data[ROWS*ROWS];
float M5_data[ROWS*ROWS];
float M6_data[ROWS*ROWS];
float M7_data[ROWS*ROWS];
float M8_data[ROWS*ROWS];

// If N = 4, use the next line (and comment the case for N = 8):
 float* mpc_M_data[N] = {M1_data, M2_data, M3_data, M4_data};

// If N = 8, use the next line (and comment the case for N = 4):
//float* mpc_M_data[N] = {M1_data, M2_data,
//                        M3_data, M4_data,
//                        M5_data, M6_data,
//                        M7_data, M8_data};

matf32_t M1, M2, M3, M4, M5, M6, M7, M8;

// If N = 4, use the next line (and comment the case for N = 8):
matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4};

// If N = 8, use the next line (and comment the case for N = 4):
// matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4, &M5, &M6, &M7, &M8};

// C prediction matrix:
// C = [B 0 0 ... 0; AB B ... 0; A^2B AB B ... 0; and so on... ]

// Memory stack and pointer arrays for MPC convolution matrix C
float AB_data[ROWS*COLS];
float A2B_data[ROWS*COLS];
float A3B_data[ROWS*COLS];
float A4B_data[ROWS*COLS];
float A5B_data[ROWS*COLS];
float A6B_data[ROWS*COLS];
float A7B_data[ROWS*COLS];
float C_null_mat_data[ROWS*COLS];

// If N = 4, use the next line (and comment the case for N = 8):
float* mpc_C_stack_data[N+1] = {B_data, AB_data, A2B_data, A3B_data, C_null_mat_data};

// If N = 8, use the next line (and comment the case for N = 4):
//float* mpc_C_stack_data[N+1] = {B_data, AB_data,
//                                A2B_data, A3B_data,
//                                A4B_data, A5B_data,
//                                A6B_data, A7B_data,
//                                C_null_mat_data};

matf32_t AB, A2B, A3B, A4B, A5B, A6B, A7B, C_null_mat;

// If N = 4, use the next line (and comment the case for N = 8):
matf32_t* mpc_C_stack[N+1] = {&B, &AB, &A2B, &A3B, &C_null_mat};

// If N = 8, use the next line (and comment the case for N = 4):
// matf32_t* mpc_C_stack[N+1] = {&B, &AB, &A2B, &A3B, &A4B, &A5B, &A6B, &A7B, &C_null_mat};

matf32_t* mpc_C[N*N]; // Matrix of matrices C.

// Penalization matrices Q and R:
float mpc_Q_data[ROWS*ROWS];
float mpc_R_data[COLS*COLS];

matf32_t mpc_Q, mpc_R;

bool state_constraints = false;
ctr_mpc_lti_shooting_t mpc;

// ---------------------------------------------------------------------------
// Arrays for saving variable trajectories
// ---------------------------------------------------------------------------

float U[2*N];
float cost[2*N];
float X1[2*N];
float X2[2*N];

// ---------------------------------------------------------------------------
// Additional matrices to compute the QP cost
// ---------------------------------------------------------------------------

float utQ_data[MAX_MAT_SIZE];
float utQu_data[MAX_MAT_SIZE];
float ut_data[MAX_MAT_SIZE];
float ct_data[MAX_MAT_SIZE];
float ctu_data[MAX_MAT_SIZE];

matf32_t utQ, utQu, ut, ct, ctu;

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // ---------------------------------------------------------------------------
    // Initialize all matrices for the LTI system, QP and MPC
    // ---------------------------------------------------------------------------

    // LTI system matrices
    matf32_init(&A, ROWS, ROWS, A_data);
    matf32_init(&B, ROWS, COLS, B_data);
    matf32_init(&C, COLS, ROWS, C_data);
    matf32_init(&D, COLS, COLS, D_data);
    matf32_init(&x_k, ROWS, COLS, x_k_data);
    matf32_init(&u_k, N, COLS, u_k_data);
    
    // QP matrices
    matf32_init(&Ain, N, N, Ain_data);
    matf32_init(&bin, N, COLS, bin_data);
    matf32_init(&qp_Q, N*COLS, N*COLS, qp_Q_data);
    matf32_init(&qp_c, N*COLS, COLS, qp_c_data);

    // QP cost matrices
    matf32_init(&ut, u_k.num_cols, u_k.num_rows, ut_data);
    matf32_init(&utQ, ut.num_rows, qp_Q.num_cols, utQ_data);
    matf32_init(&utQu, ut.num_rows, u_k.num_cols, utQu_data);
    matf32_init(&ct, qp_c.num_cols, qp_c.num_rows, ct_data);
    matf32_init(&ctu, ct.num_rows, u_k.num_cols, ctu_data);

    // MPC penalization matrices
    matf32_init(&mpc_R, COLS, COLS, mpc_R_data);
    matf32_init(&mpc_Q, ROWS, ROWS, mpc_Q_data);

    // Initial values (for the matrices where this applies)
    matf32_set(&mpc_Q, 1, 1, 1);
    matf32_set(&mpc_Q, 1, 2, -1);
    matf32_set(&mpc_Q, 2, 1, -1);
    matf32_set(&mpc_Q, 2, 2, 1);
    matf32_set(&mpc_R, 1, 1, 0.01);

    matf32_ones(&x_k);

    // ---------------------------------------------------------------------------
    // Print LTI system matrices to see the values (optional)
    // ---------------------------------------------------------------------------

    printf("\n--------------------------------------------------\n");
    printf("LTI System:\n");
    printf("--------------------------------------------------\n");

    printf("A:\n");
    matf32_print(&A);
    printf("B:\n");
    matf32_print(&B);
    printf("C:\n");
    matf32_print(&C);
    printf("u_k:\n");
    matf32_print(&u_k);
    printf("x_k:\n");
    matf32_print(&x_k);

    // ---------------------------------------------------------------------------
    // Configure the MPC
    // ---------------------------------------------------------------------------

    // 1. Initialize the LTI system
    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, sample_time);

    // 2. Initialize the MPC

    // 2.2 If the system is unconstrained, use the following line (comment 2.2)
    ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, NULL, NULL, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    
    // 2.2 If the system has constraints, use the following line (comment 2.1)
    //ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, &Ain, &bin, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    
    // 3. Compute and set the MPC prediction matrices
    ctr_mpc_set_M(&mpc, mpc_M_data); // Initialices, computes and sets prediction matrix M
    ctr_mpc_set_C(&mpc, mpc_C_stack, mpc_C_stack_data); // Initialices, computes and set preditcion matrix C

    // 4. Compute and set the QP matrices
    ctr_mpc_set_qpQ(&mpc, &qp_Q); // Compute and set matrix Q for the cost of the QP
    ctr_mpc_set_qpc(&mpc, &qp_c); // Compute and set vector c for the cost of the QP

    // 5. Configure the type of constraints
    // If input-only constraints, set state_constraints to false
    // If constraints must affect states as well, set state_constraints to true
    //mpc.state_constraints = true;
    //ctr_mpc_set_constraints(&mpc, 10, -10); // Compute constraints according to state_constraints

    // ---------------------------------------------------------------------------
    // MPC Main Loop
    // ---------------------------------------------------------------------------
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        // Update the QP and constraints
        ctr_mpc_set_qpQ(&mpc, &qp_Q); // Computes and sets matrix Q of the QP
        ctr_mpc_set_qpc(&mpc, &qp_c); // Computes and sets vector c of the QP
        //ctr_mpc_set_constraints(&mpc, 10, -10); // Updates QP constraints (comment if unconstrained)

        // Solve the QP and update the LTI system
        ctr_mpc_update(&mpc, &qp_Q, &qp_c, &x_k, &u_k);

        // Compute the QP cost: q = (1/2)*u'Qu + c'u
        matf32_trans(&u_k, &ut); // u'
        matf32_mul(&ut, &qp_Q, &utQ); // u'Q
        matf32_mul(&utQ, &u_k, &utQu); // u'Qu
        matf32_scale(&utQu, 0.5, &utQu); // (1/2)*u'Qu

        matf32_trans(&qp_c, &ct); // c'
        matf32_scale(&ct, -1, &ct); // For some reason the values flipped sign when transposing
        matf32_mul(&ct, &u_k, &ctu); // c'u
        matf32_add(&utQu, &ctu, &utQu); // (1/2)*u'Qu + c'u
        
        // Save trajectories
        U[k] = u_k.p_data[0];
        X1[k] = x_k.p_data[0];
        X2[k] = x_k.p_data[1];
        cost[k] = utQu.p_data[0];
    }

    // ---------------------------------------------------------------------------
    // Print trajectories of u_k, x_k and the cost function to see the results
    // ---------------------------------------------------------------------------
    
    // In this specific example, all trajectories must converge towards zero.

    printf("\n--------------------------------------------------\n");
    printf("Trajectories:\n");
    printf("--------------------------------------------------\n");

    printf("x1\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", X1[k]);
    }
    printf("\n");

    printf("x2\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", X2[k]);
    }
    printf("\n");

    printf("u\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", U[k]);
    }
    printf("\n");

    printf("cost\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", cost[k]);
    }
    printf("\n");
}