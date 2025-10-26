
/**
 * @author: Andrea Pineda
 * @date: Created 14 Oct 2025, Last Modified 26 Oct 2025
 * 
 * For testing mpc functions in robotat_control
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "linsolve.h"
#include "quadprog.h"
#include "robotat_control.h"

#define N (4) // Horizon length
#define ROWS (2)
#define COLS (1)



float u_k_data[MAX_MAT_SIZE];
matf32_t u_k;

float x_k_data[MAX_MAT_SIZE];
matf32_t x_k;

float qp_Q_data[MAX_MAT_SIZE];
matf32_t qp_Q;

float qp_c_data[MAX_MAT_SIZE];
matf32_t qp_c;

float x0_data[MAX_MAT_SIZE];
matf32_t x0;

float Ain_data[MAX_MAT_SIZE];
matf32_t Ain;

float bin_data[MAX_MAT_SIZE];
matf32_t bin;

float m1[ROWS*ROWS];
float m2[ROWS*ROWS];
float m3[ROWS*ROWS];
float m4[ROWS*ROWS];
float* mpc_M[N] = {m1, m2, m3, m4};

float c1[ROWS*ROWS];
float c2[ROWS*ROWS];
float c3[ROWS*ROWS];
float c4[ROWS*ROWS];
float c5[ROWS*ROWS];
float c6[ROWS*ROWS];
float c7[ROWS*ROWS];
float c8[ROWS*ROWS];
float c9[ROWS*ROWS];
float c10[ROWS*ROWS];
float c11[ROWS*ROWS];
float c12[ROWS*ROWS];
float c13[ROWS*ROWS];
float c14[ROWS*ROWS];
float c15[ROWS*ROWS];
float c16[ROWS*ROWS];
float* mpc_C[N*N] = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16};

float q1[ROWS*ROWS];
float q2[ROWS*ROWS];
float q3[ROWS*ROWS];
float q4[ROWS*ROWS];
float q5[ROWS*ROWS];
float q6[ROWS*ROWS];
float q7[ROWS*ROWS];
float q8[ROWS*ROWS];
float q9[ROWS*ROWS];
float q10[ROWS*ROWS];
float q11[ROWS*ROWS];
float q12[ROWS*ROWS];
float q13[ROWS*ROWS];
float q14[ROWS*ROWS];
float q15[ROWS*ROWS];
float q16[ROWS*ROWS];
float* mpc_Q[N*N] = {q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14, q15, q16};

float Qhat_data[MAX_MAT_SIZE];
matf32_t Qhat;

float r1[ROWS*ROWS];
float r2[ROWS*ROWS];
float r3[ROWS*ROWS];
float r4[ROWS*ROWS];
float r5[ROWS*ROWS];
float r6[ROWS*ROWS];
float r7[ROWS*ROWS];
float r8[ROWS*ROWS];
float r9[ROWS*ROWS];
float r10[ROWS*ROWS];
float r11[ROWS*ROWS];
float r12[ROWS*ROWS];
float r13[ROWS*ROWS];
float r14[ROWS*ROWS];
float r15[ROWS*ROWS];
float r16[ROWS*ROWS];
float* mpc_R[N*N] = {r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16};

float Rhat_data[MAX_MAT_SIZE];
matf32_t Rhat;

float u_k_history[2*N];
float cost_history[2*N];

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

quadprog_t qp;
ctr_sys_lti_t sys;
ctr_mpc_lti_shooting_t mpc;

float A_data[ROWS*ROWS] = {1.1, 2.0, 0.0, 0.95};
matf32_t A;

float B_data[ROWS*COLS] = {0.0, 0.0787};
matf32_t B;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float D_data[MAX_MAT_SIZE];
matf32_t D;

float sample_time = 0.5;

int main(void)
{

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
    matf32_init(&Qhat, ROWS, ROWS, Qhat_data);
    matf32_init(&Rhat, COLS, COLS, Rhat_data);

    matf32_init(&ut, u_k.num_cols, u_k.num_rows, ut_data);
    matf32_init(&utQ, ut.num_rows, qp_Q.num_cols, utQ_data);
    matf32_init(&utQu, ut.num_rows, u_k.num_cols, utQu_data);
    matf32_init(&ct, qp_c.num_cols, qp_c.num_rows, ct_data);
    matf32_init(&ctu, ct.num_rows, u_k.num_cols, ctu_data);

    matf32_set(&x_k, 1, 1, 1);
    matf32_set(&x_k, 2, 1, 1);
    matf32_set(&C, 1, 1, -1);
    matf32_set(&C, 1, 2, 1);
    matf32_set(&Rhat, 1, 1, 0.01);
    
    printf("\nA:\n");
    matf32_print(&A);
    printf("B:\n");
    matf32_print(&B);
    printf("C:\n");
    matf32_print(&C);
    printf("u_k:\n");
    matf32_print(&u_k);
    printf("x_k:\n");
    matf32_print(&x_k);

    float Ct_data[MAX_MAT_SIZE];
    matf32_t Ct;
    matf32_init(&Ct, C.num_cols, C.num_rows, Ct_data);
    matf32_trans(&C, &Ct); // C'
    matf32_mul(&Ct, &C, &Qhat);
    printf("Qhat:\n");
    matf32_print(&Qhat);
    printf("Rhat:\n");
    matf32_print(&Rhat);

    ctr_sys_lti_init(&sys, &x_k, &A, &B, &C, &D, sample_time); // x_k_1 = A*x_k + B*u_k, y_k_1 = C*x_k + D*u_k
    ctr_mpc_unconstrained_lti_init(&mpc, &qp, &sys, &u_k, &x_k, NULL, NULL, mpc_Q, mpc_R, mpc_C, mpc_M, N);

    err_status_t status;

    ctr_mpc_set_M(&mpc);
    ctr_mpc_set_C(&mpc);
    ctr_mpc_set_Q(&mpc, &Qhat, &Qhat);
    ctr_mpc_set_R(&mpc, &Rhat);
    ctr_mpc_set_qpQ(&mpc, &qp_Q);
    //ctr_mpc_set_constraints(&mpc, 1, -1, 0);

    printf("qp_Q:\n");
    matf32_print(&qp_Q);

    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("\n-------------------------\n");
        printf("k = %i:\n", k);
        printf("-------------------------\n");
        ctr_mpc_set_qpc(&mpc, &qp_c);
        //printf("qp_c:\n");
        //matf32_print(&qp_c);
        //printf("u_k0: %.9f\n", u_k.p_data[0]);
        //ctr_mpc_set_constraints(&mpc, 10, -10, 1);
        ctr_mpc_update(&mpc, &qp_Q, &qp_c, &x_k, &u_k);
        u_k_history[k] = u_k.p_data[0];
        printf("x_k:\n");
        matf32_print(&x_k);
        printf("u_k:\n");
        matf32_print(&u_k);

        // q = (1/2)*u'Qu + c'u
        matf32_trans(&u_k, &ut); // u'
        matf32_mul(&ut, &qp_Q, &utQ); // u'Q
        matf32_mul(&utQ, &u_k, &utQu); // u'Qu
        matf32_scale(&utQu, 0.5, &utQu); // (1/2)*u'Qu

        matf32_trans(&qp_c, &ct); // c'
        matf32_mul(&ct, &u_k, &ctu); // c'u
        matf32_add(&utQu, &ctu, &utQu); // (1/2)*u'Qu + c'u
        cost_history[k] = utQu.p_data[0];
    }

    printf("\n-------------------------\n");
    printf("u_k_history:\n");
    printf("-------------------------\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", u_k_history[k]);
    }
    printf("\n");

    printf("\n-------------------------\n");
    printf("cost_history: q = (1/2)*u'Qu + c'u\n");
    printf("-------------------------\n");
    for (uint16_t k = 0; k < 2*N; ++k)
    {
        printf("%.9f\n", cost_history[k]);
    }
    printf("\n");
}
