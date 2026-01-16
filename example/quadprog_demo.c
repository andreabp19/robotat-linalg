
/**
 * @author Andrea Pineda
 * @date Created 30 Dec 2025
 * Last Modified 30 Dec 2025
 *            By: Andrea Pineda
 * 
 * Quadprog demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "quadprog.h"

// ---------------------------------------------------------------------------
// Matrices and variables definitions
// ---------------------------------------------------------------------------

static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float m3data[MAX_MAT_SIZE];
static matf32_t m3;
static float m4data[MAX_MAT_SIZE];
static matf32_t m4;
static float m5data[MAX_MAT_SIZE];
static matf32_t m5;
static float m6data[MAX_MAT_SIZE];
static matf32_t m6;
static float m7data[MAX_MAT_SIZE];
static matf32_t m7;
static float m8data[MAX_MAT_SIZE];
static matf32_t m8;
static float m9data[MAX_MAT_SIZE];
static matf32_t m9;
static float m10data[MAX_MAT_SIZE];
static matf32_t m10;

matf32_t* Q = &m1;
matf32_t* c = &m2;
matf32_t* Aeq = &m3;
matf32_t* beq = &m4;
matf32_t* x = &m5;

matf32_t* Aeqx = &m6;
matf32_t* xt = &m7;
matf32_t* xtc = &m8;
matf32_t* xtQ = &m9;
matf32_t* xtQx = &m10;

quadprog_t qp;
quadprog_t sqp;

int main(void)
{
    bool tolerance_check = false;

    uint16_t n = 5;

    matf32_init(Q, n, n, m1data);
    matf32_init(c, n, 1, m2data);
    matf32_init(Aeq, n, n, m3data);
    matf32_init(beq, n, 1, m4data);
    matf32_init(x, n, 1, m5data);

    matf32_init(Aeqx, n, 1, m6data);
    matf32_init(xt, 1, n, m7data);
    matf32_init(xtc, xt->num_rows, c->num_cols, m8data);
    matf32_init(xtQ, xt->num_rows, Q->num_cols, m9data);
    matf32_init(xtQx, xt->num_rows, x->num_cols, m10data);

    matf32_randn(Q, 0, 1);
    matf32_randn(c, 0, 1);
    matf32_randn(Aeq, 0, 1);
    matf32_randn(beq, 0, 1);

    // ---------------------------------------------------------------------------
    // 1. QPs with equality restrictions
    // ---------------------------------------------------------------------------

    quadprog_init(&qp, Q, c, Aeq, beq, NULL, NULL, NULL); // NULL: No Ain, bin, x0

    printf("\n---------------------------------------------------------------------------\n");
    printf("Matrices of the QP");
    printf("\n---------------------------------------------------------------------------\n");

    printf("Q:\n");
    matf32_print(Q);
    printf("c:\n");
    matf32_print(c);
    printf("A:\n");
    matf32_print(Aeq);
    printf("b:\n");
    matf32_print(beq);

    // ---------------------------------------------------------------------------
    // 1.1 quadprog_qp_linsolve with LU
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_qp_linsolve with LU");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_zeros(x);
    quadprog_qp_linsolve(&qp, x, LU);

    printf("\nSolution of the KKT system using LU:\n");
    matf32_print(x);

    // Calculate cost: q = (1/2)x'*Q*x + x'*c
    matf32_trans(x, xt);
    matf32_mul(xt, c, xtc);
    matf32_mul(xt, Q, xtQ);
    matf32_mul(xtQ, x, xtQx);
    matf32_scale(xtQx, 0.5, xtQx);

    printf("Cost of the QP:\n");
    matf32_print(xtQx);

    matf32_mul(Aeq, x, Aeqx);
    printf("Solution Ax of the restrictions: \n");
    matf32_print(Aeqx);

    tolerance_check = matf32_is_equal(Aeqx, beq);
    printf("Difference of Ax and b smaller than 1E-05? %s\n", tolerance_check?"True":"False");

    // ---------------------------------------------------------------------------
    // 1.2 quadprog_qp_linsolve with QR
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_qp_linsolve with QR");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_zeros(x);
    quadprog_qp_linsolve(&qp, x, QR_SQUARE);

    printf("\nSolution of the KKT system using QR:\n");
    matf32_print(x);

    // Calculate cost: q = (1/2)x'*Q*x + x'*c
    matf32_trans(x, xt);
    matf32_mul(xt, c, xtc);
    matf32_mul(xt, Q, xtQ);
    matf32_mul(xtQ, x, xtQx);
    matf32_scale(xtQx, 0.5, xtQx);

    printf("Cost of the QP:\n");
    matf32_print(xtQx);

    matf32_mul(Aeq, x, Aeqx);
    printf("Solution Ax of the restrictions: \n");
    matf32_print(Aeqx);

    tolerance_check = matf32_is_equal(Aeqx, beq);
    printf("Difference of Ax and b smaller than 1E-05? %s\n", tolerance_check?"True":"False");

    // ---------------------------------------------------------------------------
    // 1.3 quadprog_qp_linsolve with SVD
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_qp_linsolve with SVD");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_zeros(x);
    quadprog_qp_linsolve(&qp, x, SVD);

    printf("\nSolution of the KKT system using SVD:\n");
    matf32_print(x);

    // Calculate cost: q = (1/2)x'*Q*x + x'*c
    matf32_trans(x, xt);
    matf32_mul(xt, c, xtc);
    matf32_mul(xt, Q, xtQ);
    matf32_mul(xtQ, x, xtQx);
    matf32_scale(xtQx, 0.5, xtQx);

    printf("Cost of the QP:\n");
    matf32_print(xtQx);
    
    matf32_mul(Aeq, x, Aeqx);
    printf("Solution Ax of the restrictions: \n");
    matf32_print(Aeqx);

    tolerance_check = matf32_is_equal(Aeqx, beq);
    printf("Difference of Ax and b smaller than 1E-05? %s\n", tolerance_check?"True":"False");

    // ---------------------------------------------------------------------------
    // 1.4 quadprog_qp_ldlt (symmetric indefinite factorization)
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_qp_ldlt (symmetric indefinite factorization)");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_zeros(x);
    quadprog_qp_ldlt(&qp, x);

    printf("\nSolution of the KKT system using LDL':\n");
    matf32_print(x);

    // Calculate cost: q = (1/2)x'*Q*x + x'*c
    matf32_trans(x, xt);
    matf32_mul(xt, c, xtc);
    matf32_mul(xt, Q, xtQ);
    matf32_mul(xtQ, x, xtQx);
    matf32_scale(xtQx, 0.5, xtQx);

    printf("Cost of the QP:\n");
    matf32_print(xtQx);

    matf32_mul(Aeq, x, Aeqx);
    printf("Solution Ax of the restrictions: \n");
    matf32_print(Aeqx);

    tolerance_check = matf32_is_equal(Aeqx, beq);
    printf("Difference of Ax and b smaller than 1E-05? %s\n", tolerance_check?"True":"False");

    // ---------------------------------------------------------------------------
    // 1.5 quadprog_qp_nullspace
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_qp_nullspace");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_init(Aeq, n, n+1, m3data);
    matf32_init(x, n+1, 1, m5data);

    matf32_randn(Aeq, 0, 1);
    printf("A:\n");
    matf32_print(Aeq);

    matf32_zeros(x);
    quadprog_qp_nullspace(&qp, x);

    printf("\nSolution of the KKT system using the Nullspace method:\n");
    matf32_print(x);

    // Calculate cost: q = (1/2)x'*Q*x + x'*c
    matf32_trans(x, xt);
    matf32_mul(xt, c, xtc);
    matf32_mul(xt, Q, xtQ);
    matf32_mul(xtQ, x, xtQx);
    matf32_scale(xtQx, 0.5, xtQx);

    printf("Cost q = (1/2)x'*Q*x + x'*c:\n");
    matf32_print(xtQx);

    matf32_mul(Aeq, x, Aeqx);
    printf("Solution Ax of the restrictions: \n");
    matf32_print(Aeqx);

    tolerance_check = matf32_is_equal(Aeqx, beq);
    printf("Difference of Ax and b smaller than 1E-05? %s\n", tolerance_check?"True":"False");

    // ---------------------------------------------------------------------------
    // 2. QPs with inequality restrictions (active-set method)
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("quadprog_sqp (active-set method)");
    printf("\n---------------------------------------------------------------------------\n");

    

}