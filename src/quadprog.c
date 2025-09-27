/**
 * @file quadprog.c
 * 
 * Last modified: 24 Sep 2025
 */

#include <stdint.h>

#include "quadprog.h"

float M_data[MAX_MAT_SIZE];
matf32_t M;

float y_data[MAX_MAT_SIZE];
matf32_t y;

float n_data[MAX_MAT_SIZE];
matf32_t n;

float temp_Aeq_data[MAX_MAT_SIZE];
matf32_t temp_Aeq;


void
quadprog_init(quadprog_t* const p_qp,
              const matf32_t* const p_Q, const matf32_t* const p_c,
              const matf32_t* const p_Aeq, const matf32_t* const p_beq,
              const matf32_t* const p_Ain, const matf32_t* const p_bin,
              const matf32_t* const p_x0)
{
    p_qp->p_Q = p_Q;
    p_qp->p_c = p_c;
    p_qp->p_Aeq = p_Aeq;
    p_qp->p_beq = p_beq;
    p_qp->p_Ain = p_Ain;
    p_qp->p_bin = p_bin;
    p_qp->p_x0 = p_x0;
}

void
quadprog_status_print(quadprog_status_t status)
{
    switch(status)
    {
        case QP_SUCESS:
            printf("QP_SUCESS\n");
            break;
        
        case QP_SIZE_MISMATCH:
            printf("QP_SIZE_MISMATCH\n");
            break;
        
        case QP_NOT_RESTRICTED:
            printf("QP_NOT_RESTRICTED\n");
            break;
        
        case QP_NOT_CONVEX:
            printf("QP_NOT_CONVEX\n");
            break;
    }
}

// TODO: Add linsolve method in the arguments to select solver method for equality-restricted QPs
quadprog_status_t
quadprog(quadprog_t* p_qp, matf32_t* const p_x)
{
    // badly defined
    if ((NULL == p_qp->p_Q) && (NULL == p_qp->p_c))
    {
        return QP_BAD_DEFINED;
    }

    // not restricted
    if ((NULL == p_qp->p_Aeq) && (NULL == p_qp->p_beq) &&
        (NULL == p_qp->p_Ain) && (NULL == p_qp->p_bin))
    {
        return QP_NOT_RESTRICTED;
    }

    // iterative method
    if (NULL != p_qp->p_x0)
    {
        // todo
    }

    // equality restrictions
    if ((NULL == p_qp->p_Ain) && (NULL == p_qp->p_bin))
    {
        // Add a condition to check the dimensions of Aeq: if square, use quadprog_qp, if rectangular use quadprog_qp_nullspace

        return quadprog_qp(p_qp, p_x, LU);
    }

    // inequality restrictions
    if ((NULL != p_qp->p_Ain) && (NULL != p_qp->p_bin))
    {
        return quadprog_sqp(p_qp, p_x);
    }

    // general solver? something that can solve a QP with any mix of conditions
}

// TODO: reorder in
// quadprog_qp for simple case
// quadprog_sqp for inequality case
// better error handling
quadprog_status_t
quadprog_qp(quadprog_t* p_qp, matf32_t* const p_x, linsolve_method_t method)
{
    /** 
     * Creates a KKT system and applies a linsolve method to it in order to solve the QP
     * 
     * Linear system: My=n
     * 
     * create M = [Q Aeq'; Aeq 0]
     * copy Q, Aeq in M
     * transpose Aeq in temp Aeq
     * copy Aeq' en M
     * 
     * create y = [x; lambda]
     * create n = [-c, -beq]
     *      copy c y beq in n
     *      n = -n
     * 
     * solve My=n
     * copy x from y
     *  
    */

    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_beq = p_qp->p_beq;


#ifdef MATH_MATRIX_CHECK
    // TODO: add size checks for matrices, use above comment as guide
#endif

    err_status_t status;

    uint16_t rows = p_qp->p_Q->num_rows + p_qp->p_Aeq->num_rows; 
    uint16_t cols = p_qp->p_Q->num_cols + p_qp->p_Aeq->num_rows; // Aeq is used transposed here

    // init matrices
    matf32_init(&M, rows, cols, M_data);
    matf32_zeros(&M);

    matf32_init(&y, rows, 1, y_data); // y_data includes lambda
    matf32_zeros(&y);

    matf32_init(&n, rows, 1, n_data);
    matf32_zeros(&n);

    matf32_init(&temp_Aeq, p_Aeq->num_cols, p_Aeq->num_rows, temp_Aeq_data);

    /** Create KKT matrix M = [Q Aeq'; Aeq 0] */
    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols); // Save Q in M(1:Q_rows, 1:Q_cols)
    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols); // Save Aeq in M(Q_rows:Aeq_rows, 1:Aeq_cols)
    matf32_trans(p_Aeq, &temp_Aeq); // Aeq'
    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows); // Save Aeq' in M(1:Aeq'_rows, Q_cols:Aeq'_cols)

    /** Create matrix n = [c, beq], as the algorithm says: n = [-c -beq] and then invert the sign of n */
    matf32_submatrix_copy(p_c, &n, 0, 0, 0, 0, p_c->num_rows, 1); // Save c in n(1:c_rows, 1)
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1); // Save beq in n(c_rows:beq_rows, 1)

    /** Solve linear system My = n using linsolve's methods */
    status = linsolve_method(&M, &n, &y, method);
    
    // For debugging: Print method, and also print conditioning number of the input matrix M to check which is using

    float cond_M = 0;
    float cond_Q = 0;
    float cond_Aeq = 0;

    //printf("Q (%i,%i):\n", p_Q->num_rows, p_Q->num_cols);
    //matf32_print(p_Q);

    //printf("Aeq (%i,%i):\n", p_Aeq->num_rows, p_Aeq->num_cols);
    //matf32_print(p_Aeq);

    //printf("M (%i,%i):\n", M.num_rows, M.num_cols);
    //matf32_print(&M);

    //matf32_cond(p_Q, &cond_Q);
    //matf32_cond(p_Aeq, &cond_Aeq);
    //matf32_cond(&M, &cond_M);
    //printf("Condition number of Q: %.9f\n", cond_Q);
    //printf("Condition number of A: %.9f\n", cond_Aeq);
    //printf("Condition number of M: %.9f\n", cond_M);

    /** Save the part of y that corresponds to the solution x of the QP */
    matf32_submatrix_copy(&y, p_x, 0, 0, 0, 0, p_c->num_rows, 1);

    return QP_SUCESS;
}


// In progress
quadprog_status_t 
quadprog_qp_nullspace(quadprog_t* p_qp, matf32_t* const p_x)
{
    /**
     * Solves the KKT system of an equality-constrained QP using the null-space method
     * 
     * From the KKT system: |Q A'||  -p  | = |g| , where A (m x n) with m <= n with full rank (m)
     *                      |A 0 ||lambda|   |h|   also, h = Ax - b (n x 1), and g = c + Gx (G_cols x 1) 
     * 
     * Defining p = Y*py + Z*pz where:
     *      Y is (n x m)
     *      Z is (n x (n-m))
     *      such that [Y|Z] is nonsingular
     * 
     * In other words, from A' = QR, we subdivide Q = [Q1|Q2], such that:
     *      Y = Q1 (n x m)
     *      Z = Q2 (n x (n-m))
     * 
     * To get py, solve: (AY)*py = -h
     * (If m = n, the problem gets reduced to solving the above as Z = 0)
     * 
     * To get pz, solve: (Z'*G*Z)*pz = -Z'*G*Y*py - Z'*g
     * 
     * Then, compute p = Y*py + Z*pz
     * 
     * And for the Lagrange multipliers solve for lambda: (AY)'*lambda = Y'*(g + G*p)
     */

    // Add dimension checks to avoid computing extra steps if m = n
    // TODO: Rewrite quadprog_qp_nullspace to reutilize some matrices instead of creating so many matrices

    uint16_t m = p_qp->p_Aeq->num_rows;
    uint16_t n = p_qp->p_Aeq->num_cols;

    float Aeqt_data[MAX_MAT_SIZE];
    matf32_t Aeqt;
    matf32_init(&Aeqt, p_qp->p_Aeq->num_cols, p_qp->p_Aeq->num_rows, Aeqt_data);

    float Q_data[MAX_MAT_SIZE];
    matf32_t Q;
    matf32_init(&Q, n, n, Q_data);

    float R_data[MAX_MAT_SIZE];
    matf32_t R;
    matf32_init(&R, n, n-1, R_data);

    float Y_data[MAX_MAT_SIZE];
    matf32_t Y;
    matf32_init(&Y, n, m, Y_data);

    float Z_data[MAX_MAT_SIZE];
    matf32_t Z;
    matf32_init(&Z, n, n-m, Z_data);

    float py_data[MAX_MAT_SIZE];
    matf32_t py;
    matf32_init(&py, m, 1, py_data);

    float pz_data[MAX_MAT_SIZE];
    matf32_t pz;
    matf32_init(&pz, n-m, p_x->num_cols, pz_data);

    float AY_data[MAX_MAT_SIZE];
    matf32_t AY;
    matf32_init(&AY, p_qp->p_Aeq->num_rows, Y.num_cols, AY_data);

    float Zt_data[MAX_MAT_SIZE];
    matf32_t Zt;
    matf32_init(&Zt, Z.num_cols, Z.num_rows, Zt_data);

    float ZtG_data[MAX_MAT_SIZE];
    matf32_t ZtG;
    matf32_init(&ZtG, Zt.num_rows, p_qp->p_Q->num_cols, ZtG_data);

    float ZtGZ_data[MAX_MAT_SIZE];
    matf32_t ZtGZ;
    matf32_init(&ZtGZ, Z.num_cols, Z.num_cols, ZtGZ_data);

    float Ztc_data[MAX_MAT_SIZE];
    matf32_t Ztc;
    matf32_init(&Ztc, Zt.num_rows, p_qp->p_c->num_cols, Ztc_data);

    float ZtGY_data[MAX_MAT_SIZE];
    matf32_t ZtGY;
    matf32_init(&ZtGY, Zt.num_rows, Y.num_cols, ZtGY_data);

    float ZtGYpy_data[MAX_MAT_SIZE];
    matf32_t ZtGYpy;
    matf32_init(&ZtGYpy, Zt.num_rows, py.num_cols, ZtGYpy_data);

    float Ypy_data[MAX_MAT_SIZE];
    matf32_t Ypy;
    matf32_init(&Ypy, Y.num_rows, py.num_cols, Ypy_data);

    float Zpz_data[MAX_MAT_SIZE];
    matf32_t Zpz;
    matf32_init(&Zpz, Z.num_rows, pz.num_cols, Zpz_data);

    /** Calculate py (if A is square, this is the only part executed, which may not lead to the optimal solution) */

    matf32_trans(p_qp->p_Aeq, &Aeqt); // Aeq'
    matf32_qr(&Aeqt, &Q, &R); // Aeq' = QR
    matf32_submatrix_copy(&Q, &Y, 0, 0, 0, 0, Q.num_rows, m); // Y = Q(:,1:m)
    matf32_mul(p_qp->p_Aeq, &Y, &AY); // py = Y*(Aeq\beq) = (Aeq*Y)\b

    linsolve(&AY, p_qp->p_beq, &py); // Solve Aeq*py = beq for py
    matf32_mul(&Y, &py, &Ypy);

    /** Calculate pz */

    matf32_submatrix_copy(&Q, &Z, 0, m, 0, 0, Q.num_rows, n-m); // Z = Q(:,m+1:end)
    matf32_trans(&Z, &Zt); // Z'

    matf32_mul(&Zt, p_qp->p_Q, &ZtG); // Z'G
    matf32_mul(&ZtG, &Z, &ZtGZ); // Z'GZ

    matf32_mul(&Zt, p_qp->p_c, &Ztc); // Z'c

    matf32_mul(&ZtG, &Y, &ZtGY); // Z'GY
    matf32_mul(&ZtGY, &py, &ZtGYpy); // Z'GYpy

    matf32_init(&y, Zt.num_rows, p_qp->p_c->num_cols, y_data);
    matf32_add(&ZtGYpy, &Ztc, &y); // y = Z'GYpy - Z'c
    matf32_scale(&y, -1.0, &y); // y = -Z'GYpy - Z'c

    linsolve(&ZtGZ, &y, &pz); // Solve (Z'Gz)pz = -Z'GYpy - Z'c for pz
    matf32_mul(&Z, &pz, &Zpz); // Zpz

    matf32_add(&Ypy, &Zpz, p_x); // Solution for the system

    return QP_SUCESS;
}


quadprog_status_t
quadprog_qp_ldlt(quadprog_t* p_qp, matf32_t* const p_x)
{
    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_beq = p_qp->p_beq;

    err_status_t status;

    uint16_t rows = p_qp->p_Q->num_rows + p_qp->p_Aeq->num_rows; 
    uint16_t cols = p_qp->p_Q->num_cols + p_qp->p_Aeq->num_rows; // Aeq is used transposed here

    // init matrices
    matf32_init(&M, rows, cols, M_data);
    matf32_zeros(&M);

    matf32_init(&y, rows, 1, y_data); // y_data includes lambda
    matf32_zeros(&y);

    matf32_init(&n, rows, 1, n_data);
    matf32_zeros(&n);

    matf32_init(&temp_Aeq, p_Aeq->num_cols, p_Aeq->num_rows, temp_Aeq_data);

    /** Create KKT matrix M = [Q Aeq'; Aeq 0] */
    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols); // Save Q in M(1:Q_rows, 1:Q_cols)
    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols); // Save Aeq in M(Q_rows:Aeq_rows, 1:Aeq_cols)
    matf32_trans(p_Aeq, &temp_Aeq); // Aeq'
    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows); // Save Aeq' in M(1:Aeq'_rows, Q_cols:Aeq'_cols)

    /** Create matrix n = [c, beq], as the algorithm says: n = [-c -beq] and then invert the sign of n */
    matf32_submatrix_copy(p_c, &n, 0, 0, 0, 0, p_c->num_rows, 1); // Save c in n(1:c_rows, 1)
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1); // Save beq in n(c_rows:beq_rows, 1)

    //printf("M:\n");
    //matf32_print(&M);
    //printf("n:\n");
    //matf32_print(&n);

    /** Generate LDL' Factorization */

    float alpha = (1 + sqrt(17))/8; // As Bunch and Parlett's algorithm says
    float mu0 = 0;
    float mu1 = 0;
    uint16_t s = 0;
    uint16_t exi = 0;
    uint16_t exj = 0;
    uint16_t exii = 0;

    //printf("alpha = %.9f\n\n", alpha);

    float P_data[MAX_MAT_SIZE];
    matf32_t P; // Permutation matrix
    matf32_init(&P, M.num_rows, M.num_cols, P_data);
    matf32_eye(&P); // Initialize as the identity matrix

    float Pi_row_data[MAX_MAT_SIZE];
    matf32_t Pi_row;
    matf32_init(&Pi_row, 1, M.num_cols, Pi_row_data);

    float Pj_row_data[MAX_MAT_SIZE];
    matf32_t Pj_row;
    matf32_init(&Pj_row, 1, M.num_cols, Pj_row_data);

    float P_row_data[MAX_MAT_SIZE];
    matf32_t P_row;
    matf32_init(&P_row, 1, M.num_cols, P_row_data);

    // Identify indices for permutations
    for (uint16_t i = 0; i < M.num_rows; ++i)
    {
        for (uint16_t j = 0; j < M.num_rows; ++j)
        {
            if (fabs(M.p_data[i*M.num_rows + j]) > mu0)
            {
                mu0 = fabs(M.p_data[i*M.num_rows + j]);
                exi = i;
                exj = j;
            }
        }
    }

    for (uint16_t i = 0; i < M.num_rows; ++i)
    {
        if (fabs(M.p_data[i*M.num_rows + i]) > mu1)
        {
            mu1 = fabs(M.p_data[i*M.num_rows + i]);
            exii = i;
        }
    }

    //printf("exii: %i\n", exii);
    //printf("exi: %i\n", exi);
    //printf("exj: %i\n", exj);
    //printf("mu0: %.9f\n", mu0);
    //printf("mu1: %.9f\n\n", mu1);

    //printf("P:\n");
    //matf32_print(&P);

    // Generate permutation matrix
    if (mu1 >= alpha*mu0)
    {
        s = 1; // 1-by-1 pivot matrix E

        // Save current rows P(exii,:) and P(1,:)
        matf32_submatrix_copy(&P, &Pi_row, exii, 0, 0, 0, 1, M.num_cols); // Pi_row = P(exii,:)
        matf32_submatrix_copy(&P, &P_row, 0, 0, 0, 0, 1, M.num_cols); // P_row = P(1,:)

        // Switch rows P(exii,:) and P(1,:) with each other
        matf32_submatrix_copy(&Pi_row, &P, 0, 0, 0, 0, 1, M.num_cols); // P(1,:) = P(exii,:)
        matf32_submatrix_copy(&P_row, &P, 0, 0, exii, 0, 1, M.num_cols); // P(exii,:) = P(1,:)
    }
    else
    {
        // TODO: Debug this case because LDL' reconstruction doesn't match PMP'
        s = 2; // 2-by-2 pivot matrix E

        // Switch rows P(exi,:) and P(1,:)
        matf32_submatrix_copy(&P, &Pi_row, exi, 0, 0, 0, 1, M.num_cols); // Pi_row = P(exi,:)
        matf32_submatrix_copy(&P, &P_row, 0, 0, 0, 0, 1, M.num_cols); // P_row = P(1,:)

        matf32_submatrix_copy(&Pi_row, &P, 0, 0, 0, 0, 1, M.num_cols); // P(1,:) = P(exi,:)
        matf32_submatrix_copy(&P_row, &P, 0, 0, exi, 0, 1, M.num_cols); // P(exi,:) = P(1:0)

        // Switch rows P(exj,:) and P(2,:)
        matf32_submatrix_copy(&P, &Pj_row, exj, 0, 0, 0, 1, M.num_cols); // Pj_row = P(exj,:)
        matf32_submatrix_copy(&P, &P_row, 1, 0, 0, 0, 1, M.num_cols); // P_row = P(2,:)

        matf32_submatrix_copy(&Pj_row, &P, 0, 0, 1, 0, 1, M.num_cols); // P(2,:) = P(exj,:)
        matf32_submatrix_copy(&P_row, &P, 0, 0, exj, 0, 1, M.num_cols); // P(exj,:) = P(1:0)
    }

    //printf("s: %i\n\n", s);

    //printf("P:\n");
    //matf32_print(&P);

    // Calculate PMP'

    float Pt_data[MAX_MAT_SIZE];
    matf32_t Pt;
    matf32_init(&Pt, P.num_cols, P.num_rows, Pt_data);

    float PM_data[MAX_MAT_SIZE];
    matf32_t PM;
    matf32_init(&PM, P.num_rows, M.num_cols, PM_data);

    float PMPt_data[MAX_MAT_SIZE];
    matf32_t PMPt;
    matf32_init(&PMPt, P.num_rows, P.num_rows, PMPt_data);

    matf32_mul(&P, &M, &PM); // PM = P*M
    //printf("PM:\n");
    //matf32_print(&PM);

    matf32_trans(&P, &Pt); // P = P'
    //printf("P':\n");
    //matf32_print(&Pt);
    
    matf32_mul(&PM, &Pt, &PMPt); // PMPt = P*M*P'
    //printf("PMP':\n");
    //matf32_print(&PMPt);

    // Extract submatrices from PMP'

    float E_data[MAX_MAT_SIZE];
    matf32_t E;
    matf32_init(&E, s, s, E_data);

    float C_data[MAX_MAT_SIZE];
    matf32_t C;
    matf32_init(&C, M.num_rows-s, s, C_data);

    float B_data[MAX_MAT_SIZE];
    matf32_t B;
    matf32_init(&B, M.num_rows-s, M.num_cols-s, B_data);

    matf32_submatrix_copy(&PMPt, &E, 0, 0, 0, 0, E.num_rows, E.num_cols); // E = PMPt(1:s, 1:s)
    matf32_submatrix_copy(&PMPt, &C, s, 0, 0, 0, C.num_rows, C.num_cols); // C = PMPt(s+1:end, 1:s)
    matf32_submatrix_copy(&PMPt, &B, s, s, 0, 0, B.num_rows, B.num_cols); // B = PMPt(s+1:end, s+1:end)

    //printf("E:\n");
    //matf32_print(&E);
    //printf("C:\n");
    //matf32_print(&C);

    //float condB = 0;
    //matf32_cond(&B, &condB);
    //printf("Condition number B: %.9f\n\n", condB);

    //printf("B:\n");
    //matf32_print(&B);

    // Generate lower triangle matrix L and diagonal matrix D

    float L_data[MAX_MAT_SIZE];
    matf32_t L;
    matf32_init(&L, M.num_rows, M.num_cols, L_data);
    matf32_zeros(&L);
    matf32_eye(&L);

    float Ei_data[MAX_MAT_SIZE];
    matf32_t Ei;
    matf32_init(&Ei, E.num_rows, E.num_cols, Ei_data);
    matf32_inv(&E, &Ei);

    //float condE = 0;
    //matf32_cond(&E, &condE);
    //printf("Condition number E: %.9f\n\n", condE);

    //printf("inv(E):\n");
    //matf32_print(&Ei);

    //float condEi = 0;
    //matf32_cond(&Ei, &condEi);
    //printf("Condition number inv(E): %.9f\n\n", condEi);

    float CEi_data[MAX_MAT_SIZE];
    matf32_t CEi;
    matf32_init(&CEi, C.num_rows, Ei.num_cols, CEi_data);

    matf32_mul(&C, &Ei, &CEi);
    //printf("C*E^-1:\n");
    //matf32_print(&CEi);

    matf32_submatrix_copy(&CEi, &L, 0, 0, s, 0, CEi.num_rows, CEi.num_cols); // L = [Is 0; C*E^-1 In-s]

    //printf("L:\n");
    //matf32_print(&L);

    float D_data[MAX_MAT_SIZE];
    matf32_t D;
    matf32_init(&D, M.num_rows, M.num_cols, D_data);
    matf32_zeros(&D);
    
    float Ct_data[MAX_MAT_SIZE];
    matf32_t Ct;
    matf32_init(&Ct, C.num_cols, C.num_rows, Ct_data);

    float CEiCt_data[MAX_MAT_SIZE];
    matf32_t CEiCt;
    matf32_init(&CEiCt, C.num_rows, C.num_rows, CEiCt_data);

    float B_CEiCt_data[MAX_MAT_SIZE];
    matf32_t B_CEiCt;
    matf32_init(&B_CEiCt, C.num_rows, C.num_rows, B_CEiCt_data);

    matf32_trans(&C, &Ct); // Ct = C'
    //printf("C':\n");
    //matf32_print(&Ct);
    
    matf32_mul(&CEi, &Ct, &CEiCt); // C = C*E^-1*C'
    //printf("C*E^-1*C':\n");
    //matf32_print(&CEiCt);

    matf32_sub(&B, &CEiCt, &B_CEiCt); // CEiCt = B - C*E^-1*C'
    //printf("B - C*E^-1*C':\n");
    //matf32_print(&B_CEiCt);

    // D = [E 0; 0 CEiCt]
    matf32_submatrix_copy(&E, &D, 0, 0, 0, 0, E.num_rows, E.num_cols);
    matf32_submatrix_copy(&B_CEiCt, &D, 0, 0, E.num_rows, E.num_cols, B_CEiCt.num_rows, B_CEiCt.num_cols);

    //printf("D:\n");
    //matf32_print(&D);

    // Reconstruction PAP' = LDL' for debugging
    float Lt_data[MAX_MAT_SIZE];
    matf32_t Lt;
    matf32_init(&Lt, L.num_cols, L.num_rows, Lt_data);

    float LD_data[MAX_MAT_SIZE];
    matf32_t LD;
    matf32_init(&LD, L.num_rows, D.num_cols, LD_data);

    float LDLt_data[MAX_MAT_SIZE];
    matf32_t LDLt;
    matf32_init(&LDLt, L.num_rows, Lt.num_cols, LDLt_data);

    matf32_mul(&L, &D, &LD);
    //printf("LD:\n");
    //matf32_print(&LD);
    matf32_trans(&L, &Lt);
    matf32_mul(&LD, &Lt, &LDLt);
    //printf("LDL':\n");
    //matf32_print(&LDLt);

    /** Solve My = n with LDL' Factorization */

    float z1_data[MAX_MAT_SIZE];
    matf32_t z1;
    matf32_init(&z1, M.num_rows, 1, z1_data);

    float Pb_data[MAX_MAT_SIZE];
    matf32_t Pb;
    matf32_init(&Pb, P.num_rows, n.num_cols, Pb_data);

    matf32_mul(&P, &n, &Pb); // Pb = P*n
    //printf("Pb:\n");
    //matf32_print(&Pb);
    linsolve(&L, &Pb, &z1); // Solve L*z1 = Pb
    //printf("z1:\n");
    //matf32_print(&z1);

    float z2_data[MAX_MAT_SIZE];
    matf32_t z2;
    matf32_init(&z2, M.num_rows, 1, z2_data);

    linsolve(&D, &z1, &z2); // Solve D*z2 = z1
    //printf("z2:\n");
    //matf32_print(&z2);

    float z3_data[MAX_MAT_SIZE];
    matf32_t z3;
    matf32_init(&z3, M.num_rows, 1, z3_data);

    matf32_trans(&L, &Lt); // L = L'
    //printf("L':\n");
    //matf32_print(&Lt);
    linsolve(&Lt, &z2, &z3); // Solve L'*z3 = z2
    //printf("z3:\n");
    //matf32_print(&z3);

    matf32_init(&y, M.num_rows, 1, y_data);

    matf32_mul(&Pt, &z3, &y); // x = P'*z3
    //printf("y:\n");
    //matf32_print(&y);

    float My_data[MAX_MAT_SIZE];
    matf32_t My;
    matf32_init(&My, M.num_rows, y.num_cols, My_data);
    matf32_mul(&M, &y, &My);
    //printf("My:\n");
    //matf32_print(&My);

    /** Save the part of y that corresponds to the solution x of the QP */
    matf32_submatrix_copy(&y, p_x, 0, 0, 0, 0, p_c->num_rows, 1);

    return QP_SUCESS;
}


// active set binding direction method
quadprog_status_t
quadprog_sqp(quadprog_t* p_qp, matf32_t* const p_x)
{
    /*
        Express active set of inequalities as equalities

        solve each iteration with quadsolve_qp

        next iteration x = x + alpha*p;

    */

    // 3 Sep 2025: Temporal fix to "discard const qualifier from pointer target type" error in p_Q
    // Because in matf32_trans(p_Q, p_Q) it's trying to save in a constant matrix
    float temp_Q_data[MAX_MAT_SIZE];
    matf32_t temp_Q;
    matf32_init(&temp_Q, p_qp->p_Q->num_rows, p_qp->p_Q->num_cols, temp_Q_data);

    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_beq = p_qp->p_beq;
    const matf32_t* p_Ain = p_qp->p_Ain;
    const matf32_t* p_bin = p_qp->p_bin;

    matf32_t Aeq_zero, beq_zero;
    matf32_init(&Aeq_zero, 0, 0, NULL);
    matf32_init(&beq_zero, 0, 0, NULL);

    if (NULL == p_Aeq)
    {
        p_Aeq = &Aeq_zero;
    }

    if (NULL == p_beq)
    {
        p_beq = &beq_zero;
    }

#ifdef MATH_MATRIX_CHECK
    // TODO: check size, positive definite
#endif

    // if available, set starting point
    if (NULL == p_qp->p_x0)
    {
        matf32_zeros(p_x);
    }
    else
    {
        matf32_copy(p_qp->p_x0, p_x);
    }

    float p_data[MAX_MAT_SIZE];
    matf32_t p;
    matf32_init(&p, p_x->num_rows, 1, p_data);


    matf32_t sigma;
    matf32_init(&sigma, p_Ain->num_rows, 1, y_data+p_Aeq->num_rows+1);


    float sub_c_data[MAX_MAT_SIZE];
    matf32_t sub_c;
    matf32_init(&sub_c, p_c->num_rows, 1, sub_c_data);


    float sub_Aeq_data[MAX_MAT_SIZE];
    matf32_t sub_Aeq;
    matf32_init(&sub_Aeq, p_Aeq->num_rows + p_Ain->num_rows, p_Ain->num_cols, sub_Aeq_data);
    matf32_zeros(&sub_Aeq);
    matf32_submatrix_copy(p_Aeq, &sub_Aeq, 0, 0, 0, 0, p_Aeq->num_rows, p_Aeq->num_cols);


    float sub_beq_data[MAX_MAT_SIZE];
    matf32_t sub_beq;
    matf32_init(&sub_beq, p_beq->num_rows + p_bin->num_rows, 1, sub_beq_data);
    matf32_zeros(&sub_beq);

    bool flags_active_ineqs[p_Ain->num_rows];
    float alpha_list[p_Ain->num_rows];

    for (uint16_t i = 0; i < p_Ain->num_rows; ++i)
    {
        flags_active_ineqs[i] = false;
    }


    float Ain_row_data[MAX_MAT_SIZE];
    matf32_t Ain_row;
    matf32_init(&Ain_row, 1, p_Ain->num_cols, Ain_row_data);

    // printf("Q:\n");
    // matf32_print(p_Q);

    // printf("sub_c:\n");
    // matf32_print(&sub_c);

    // printf("sub_Aeq:\n");
    // matf32_print(&sub_Aeq);

    // printf("sub_beq:\n");
    // matf32_print(&sub_beq);

    quadprog_t subproblem;
    quadprog_init(&subproblem, p_Q, &sub_c, &sub_Aeq, &sub_beq, NULL, NULL, NULL);



    for (uint16_t i = 0; i < MAX_ITERATION_COUNT_SQP; ++i)
    {
        printf("--------------------------------------------------\n");
        printf("QUADPROG_SQP ITERATION: %i\n", i+1);
        printf("--------------------------------------------------\n\n");

        // prepare subproblem c vector
        matf32_trans(p_Q, &temp_Q);
        matf32_mul(&temp_Q, p_x, &sub_c);
        matf32_trans(&temp_Q, &temp_Q);
        matf32_add(p_c, &sub_c, &sub_c);
        matf32_scale(&sub_c, -1, &sub_c);

        quadprog_qp(&subproblem, &p, LU);

        printf("sub_c:\n");
        matf32_print(&sub_c);

        printf("p:\n");
        matf32_print(&p);

        matf32_scale(&sigma, -1, &sigma);

        // p < err
        if (matf32_is_equal_scalar(&p, 0))
        {
            if (matf32_is_equal_less_scalar(&sigma, 0))
            {
                return QP_SUCESS;
            }

            for (uint16_t j = 0; j < sigma.num_rows; ++j)
            {
                if (sigma.p_data[j] > 0)
                {
                    matf32_set_row(&sub_Aeq, j, 0);
                    flags_active_ineqs[j] = 0;
                }
            }
        }
        else
        {
            float alpha = 1.0/0.0;
            float alpha_temp = 0;
            uint16_t alpha_index = 0;

            for (uint16_t j = 0; j < p_Ain->num_rows; ++j)
            {
                matf32_submatrix_copy(p_Ain, &Ain_row, j, 0, 0, 0, 1, p_Ain->num_cols);

                float ain_row_p = 0;
                matf32_dot(&Ain_row, &p, &ain_row_p);

                float ain_row_x = 0;
                matf32_dot(&Ain_row, p_x, &ain_row_x);

                if (1 == flags_active_ineqs[j] || ain_row_p >= 0)
                {
                    continue;
                }
                else
                {
                    alpha_temp = (ain_row_x + p_bin->p_data[j])/ain_row_p;
                }

                if (alpha_temp < alpha)
                {
                    alpha = alpha_temp;
                    alpha_index = j;
                }
            }

            if (alpha < 1)
            {
                matf32_submatrix_copy(p_Ain, &sub_Aeq,
                                        alpha_index, 0,
                                        p_Aeq->num_rows + alpha_index, 0,
                                        1, p_Ain->num_cols);

                flags_active_ineqs[alpha_index] = 1;

                matf32_scale(&p, alpha, &p);
            }

            matf32_sub(p_x, &p, p_x);
        }
    }
}
