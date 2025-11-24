/**
 * @addtogroup quadprog
 * @{
 * @file quadprog.c
 * 
 * Last modified: 4 Nov 2025
 */

#include <stdint.h>

#include "quadprog.h"

// TODO: Change M, y and n to kkt_M, kkt_y and kkt_n to avoid errors when naming variables M, y and n in the main program.
float M_data[MAX_MAT_SIZE];
matf32_t M;

float y_data[MAX_MAT_SIZE];
matf32_t y;

float n_data[MAX_MAT_SIZE];
matf32_t n;

float temp_Aeq_data[MAX_MAT_SIZE];
matf32_t temp_Aeq;

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
quadprog(quadprog_t* p_qp, matf32_t* const p_x, bool set_linsolve_method, quadprog_method_t method)
{
    // Add checks for the dimensions of Aeq and beq (rows should match) and same for Ain and bin

    // Badly defined
    if ((NULL == p_qp->p_Q) && (NULL == p_qp->p_c))
    {
        return QP_BAD_DEFINED;
    }

    // Unrestricted QP (solve with linsolve)
    if ((NULL == p_qp->p_Aeq) && (NULL == p_qp->p_beq) &&
        (NULL == p_qp->p_Ain) && (NULL == p_qp->p_bin))
    {
        matf32_init(&m1, p_qp->p_c->num_rows, p_qp->p_c->num_cols, m1data);

        matf32_scale(p_qp->p_c, -1.0, &m1); // m = -c
        return linsolve(p_qp->p_Q, &m1, p_x); // LU or SVD
    }

    // Equality restrictions (solve with normal linsolve methods or specific KKT methods)
    if ((NULL == p_qp->p_Ain) && (NULL == p_qp->p_bin))
    {
        if (!set_linsolve_method) // Chooses between KKT-specific methods 
        {
            if (!matf32_check_square_matrix(p_qp->p_Aeq)) // Rectangular Aeq
            {
                return quadprog_qp_nullspace(p_qp, p_x); // Nullspace method for the KKT system
            }
            else
            {
                return quadprog_qp_ldlt(p_qp, p_x); // LDL' factorization of the KKT system
            }
        }
        else // Solve KKT system with linsolve methods (not specific methods for KKT but valid)
        {
            switch (method)
            {
                case QP_LU: 
                    return quadprog_qp_linsolve(p_qp, p_x, LU);
                    break;

                case QP_SVD:
                    return quadprog_qp_linsolve(p_qp, p_x, SVD);
                    break;

                case QP_QR:
                    return quadprog_qp_linsolve(p_qp, p_x, QR);
                    break;
            }
        }

         
    }

    // Inequality restrictions (solve with active set)
    if ((NULL != p_qp->p_Ain) && (NULL != p_qp->p_bin))
    {
        return quadprog_sqp(p_qp, p_x);
    }
}

quadprog_status_t
quadprog_qp_linsolve(quadprog_t* p_qp, matf32_t* const p_x, linsolve_method_t method)
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

    err_status_t status;

    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_beq = p_qp->p_beq;


#ifdef MATH_MATRIX_CHECK
    // TODO: add size checks for matrices, use above comment as guide
#endif

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

    // Create KKT matrix M = [Q Aeq'; Aeq 0]
    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols); // Save Q in M(1:Q_rows, 1:Q_cols)
    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols); // Save Aeq in M(Q_rows:Aeq_rows, 1:Aeq_cols)
    matf32_trans(p_Aeq, &temp_Aeq); // Aeq'
    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows); // Save Aeq' in M(1:Aeq'_rows, Q_cols:Aeq'_cols)

    // Create matrix n = [c, beq], as the algorithm says: n = [-c -beq] and then invert the sign of n
    matf32_submatrix_copy(p_c, &n, 0, 0, 0, 0, p_c->num_rows, 1); // Save c in n(1:c_rows, 1)
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1); // Save beq in n(c_rows:beq_rows, 1)

    // Solve linear system My = n using linsolve's methods
    status = linsolve_method(&M, &n, &y, method, SQUARE);

    // Save the part of y that corresponds to the solution x of the QP
    matf32_submatrix_copy(&y, p_x, 0, 0, 0, 0, p_c->num_rows, 1);

    return QP_SUCESS;
}


// TODO: Test again after changing the matrices to reusable pointers
quadprog_status_t 
quadprog_qp_nullspace(quadprog_t* p_qp, matf32_t* const p_x)
{
    /**
     *  Procedure:
     *  
     *  Solves the KKT system of an equality-constrained QP using the null-space method. Based on
     *  Nocedal, Numerical Optimization, 16.2 Direct Solution of the KKT System.
     *  
     *  From the KKT system:
     * 
     *      |Q   Aeq'||  -p  | = |g| 
     *      |Aeq  0  ||lambda|   |h|   
     *  
     *  where Aeq (m x n) with m <= n with full rank (m), while for h and g:
     * 
     *      h = Ajp_k - b (n x 1),
     *      g = c + Gx (G_cols x 1) 
     * 
     *  Defining:
     * 
     *      p = Y*py + Z*pz
     * 
     *  where Y (n x m) and Z (n x (n-m)), such that a matrix [Y|Z] is nonsingular. In other words,
     *  calculate the QR factorization of the transposed matrix Aeq and subdivide the Q factor: 
     * 
     *      Aeq' = QR,
     * 
     *      Q = [Q1|Q2],
     * 
     *      Y = Q1 (n x m)
     *      Z = Q2 (n x (n-m))
     *  
     *  To get py, solve:
     * 
     *      (AY)*py = -h
     *  
     *  (If m = n, the problem gets reduced to solving the above as Z = 0, which means the Q and c of the QP are not
     *  actually taken into account and the result may be valid but may not be the minimum for both the cost and constraints).
     *  
     *  To get pz, solve:
     * 
     *      (Z'*Q*Z)*pz = -Z'*Q*Y*py - Z'*g
     *  
     *  Then, having all needed matrices, compute:
     * 
     *      p = Y*py + Z*pz
     *  
     *  And if the Lagrange Multipliers are needed, solve the following system for lambda:
     * 
     *      (AY)'*lambda = Y'*(g + G*p)
     */

    // TODO: Add dimension checks to avoid computing extra steps if m = n

    uint16_t m = p_qp->p_Aeq->num_rows;
    uint16_t n = p_qp->p_Aeq->num_cols;

    matf32_t* Aeqt = &m1;
    matf32_init(Aeqt, p_qp->p_Aeq->num_cols, p_qp->p_Aeq->num_rows, m1data);
    matf32_t* Q = &m2;
    matf32_init(Q, n, n, m2data);
    matf32_t* R = &m3;
    matf32_init(R, n, n-1, m3data);

    matf32_trans(p_qp->p_Aeq, Aeqt); // Aeq'
    matf32_qr(Aeqt, Q, R); // Aeq' = QR

    // Define matrices for p = Y*py + Z*pz
    matf32_t* Y = &m3;
    matf32_init(Y, n, m, m3data);
    matf32_zeros(Y);
    matf32_t* py = &m4;
    matf32_init(py, m, 1, m4data);
    matf32_t* Z = &m5;
    matf32_init(Z, n, n-m, m5data);
    matf32_t* pz = &m6;
    matf32_init(pz, n-m, p_x->num_cols, m6data);

    // Calculate Y*py

    matf32_submatrix_copy(Q, Y, 0, 0, 0, 0, Q->num_rows, m); // Y = Q(:,1:m)
    
    matf32_t* AY = &m7;
    matf32_init(AY, p_qp->p_Aeq->num_rows, Y->num_cols, m7data);

    matf32_mul(p_qp->p_Aeq, Y, AY); // py = Y*(Aeq\beq) = (Aeq*Y)\b
    linsolve(AY, p_qp->p_beq, py); // py = AY\beq
    matf32_mul(Y, py, p_x); // p_x = Y*py

    // Calculate Z*pz

    matf32_submatrix_copy(Q, Z, 0, m, 0, 0, Q->num_rows, n-m); // Z = Q(:,m+1:end)
    
    matf32_t* ZtG = &m7;
    matf32_init(ZtG, Z->num_cols, p_qp->p_Q->num_cols, m7data);
    matf32_zeros(ZtG); // Erase previous data in m7
    matf32_t* ZtGZ = &m1;
    matf32_init(ZtGZ, Z->num_cols, Z->num_cols, m1data);
    matf32_t* ZtGY = &m8;
    matf32_init(ZtGY, Z->num_cols, Y->num_cols, m8data);
    matf32_t* Ztc = &m9;
    matf32_init(Ztc, Z->num_cols, p_qp->p_c->num_cols, m9data);
    matf32_t* Zt = &m10;
    matf32_init(Zt, Z->num_cols, Z->num_rows, m10data);
    
    matf32_trans(Z, Zt); // Z = Z'
    matf32_mul(Zt, p_qp->p_c, Ztc); // Z'c
    matf32_mul(Zt, p_qp->p_Q, ZtG); // Z'G
    matf32_mul(ZtG, Z, ZtGZ); // Z'GZ
    matf32_mul(ZtG, Y, ZtGY); // Z'GY

    matf32_t* ZtGYpy = &m7;
    matf32_init(ZtGYpy, Z->num_cols, py->num_cols, m7data);
    matf32_zeros(ZtGYpy); // Erase previous data in m7
    matf32_mul(ZtGY, py, ZtGYpy); // Z'GYpy
    
    matf32_t* p = &m8;
    matf32_init(p, Z->num_cols, p_qp->p_c->num_cols, m8data);
    matf32_zeros(p);
    matf32_add(ZtGYpy, Ztc, p); // p = Z'GYpy + Z'c
    matf32_scale(p, -1.0, p); // p = -Z'GYpy - Z'c

    matf32_t* Zpz = &m10;
    matf32_init(Zpz, Z->num_rows, pz->num_cols, m10data);
    matf32_zeros(Zpz);
    linsolve(ZtGZ, p, pz); // Solve (Z'Gz)pz = -Z'GYpy - Z'c for pz
    matf32_mul(Z, pz, Zpz); // Zpz

    matf32_add(p_x, Zpz, p_x); // p_x = Y*py + Z*pz

    return QP_SUCESS;
}


// TODO: Rewrite to implement reusable matrix pointers (consider moving the factorization to matf32 and creating linsolve_ldlt to shorten de function)
quadprog_status_t
quadprog_qp_ldlt(quadprog_t* p_qp, matf32_t* const p_x)
{
    // ---------------------------------------------------------------------------
    // 0. Define some pointers and variables to use
    // ---------------------------------------------------------------------------

    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_beq = p_qp->p_beq;

    err_status_t status;

    uint16_t rows = p_qp->p_Q->num_rows + p_qp->p_Aeq->num_rows; 
    uint16_t cols = p_qp->p_Q->num_cols + p_qp->p_Aeq->num_rows; // Aeq is used transposed here

    // ---------------------------------------------------------------------------
    // 1. Build KKT System My = n
    // ---------------------------------------------------------------------------

    // --------------- Initialize matrices ---------------
    
    matf32_init(&M, rows, cols, M_data);
    matf32_zeros(&M);
    matf32_init(&y, rows, 1, y_data); // y_data includes lambda
    matf32_zeros(&y);
    matf32_init(&n, rows, 1, n_data);
    matf32_zeros(&n);
    matf32_init(&temp_Aeq, p_Aeq->num_cols, p_Aeq->num_rows, temp_Aeq_data);
    
    // --------------- Create KKT matrix M = [Q Aeq'; Aeq 0] ---------------
    
    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols); // Save Q in M(1:Q_rows, 1:Q_cols)
    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols); // Save Aeq in M(Q_rows:Aeq_rows, 1:Aeq_cols)
    matf32_trans(p_Aeq, &temp_Aeq); // Aeq'
    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows); // Save Aeq' in M(1:Aeq'_rows, Q_cols:Aeq'_cols)

    // --------------- Create KKT matrix n = [c, beq] ---------------
    
    matf32_submatrix_copy(p_c, &n, 0, 0, 0, 0, p_c->num_rows, 1); // Save c in n(1:c_rows, 1)
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1); // Save beq in n(c_rows:beq_rows, 1)

    // ---------------------------------------------------------------------------
    // 2. Generate LDL' Factorization
    // ---------------------------------------------------------------------------

    // --------------- Variables to be used ---------------
    
    float alpha = (1 + sqrt(17))/8; // Best value for alpha, according to Bunch and Parlett's algorithm
    float mu0 = 0;
    float mu1 = 0;
    uint16_t s = 0;
    uint16_t exi = 0;
    uint16_t exj = 0;
    uint16_t exii = 0;

    // --------------- Matrices for the permutations ---------------
    
    matf32_t* P = &m1; // Full permutation matrix P
    matf32_init(P, M.num_rows, M.num_cols, m1data); // Initialize P
    matf32_eye(P); // P = I (Identity matrix)

    // Pi_row, Pj_row and P_row to save rows of P to be permutted
    matf32_t* Pi_row = &m2; // For saving row i 
    matf32_init(Pi_row, 1, M.num_cols, m2data); // Initialize Pi_row

    matf32_t* Pj_row = &m3; // For saving row j
    matf32_init(Pj_row, 1, M.num_cols, m3data); // Initialize Pj_row

    matf32_t* P_row = &m4; // For saving a copy of a row
    matf32_init(P_row, 1, M.num_cols, m4data); // Initialize P_row 

    // --------------- Identify indices for the permutations ---------------
    
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

    // --------------- Generate Permutation Matrix P ---------------
    
    if (mu1 >= alpha*mu0)
    {
        s = 1; // 1-by-1 pivot matrix E

        // Save current rows P(exii,:) and P(1,:)
        matf32_submatrix_copy(P, Pi_row, exii, 0, 0, 0, 1, M.num_cols); // Pi_row = P(exii,:)
        matf32_submatrix_copy(P, P_row, 0, 0, 0, 0, 1, M.num_cols); // P_row = P(1,:)

        // Switch rows P(exii,:) and P(1,:) with each other
        matf32_submatrix_copy(Pi_row, P, 0, 0, 0, 0, 1, M.num_cols); // P(1,:) = P(exii,:)
        matf32_submatrix_copy(P_row, P, 0, 0, exii, 0, 1, M.num_cols); // P(exii,:) = P(1,:)
    }
    else
    {
        // TODO: Check this TODO because I don't remember if I fixed it or not D:
        // TODO: Debug this case because LDL' reconstruction doesn't match PMP'
        s = 2; // 2-by-2 pivot matrix E

        // Switch rows P(exi,:) and P(1,:)
        matf32_submatrix_copy(P, Pi_row, exi, 0, 0, 0, 1, M.num_cols); // Pi_row = P(exi,:)
        matf32_submatrix_copy(P, P_row, 0, 0, 0, 0, 1, M.num_cols); // P_row = P(1,:)

        matf32_submatrix_copy(Pi_row, P, 0, 0, 0, 0, 1, M.num_cols); // P(1,:) = P(exi,:)
        matf32_submatrix_copy(P_row, P, 0, 0, exi, 0, 1, M.num_cols); // P(exi,:) = P(1:0)

        // Switch rows P(exj,:) and P(2,:)
        matf32_submatrix_copy(P, Pj_row, exj, 0, 0, 0, 1, M.num_cols); // Pj_row = P(exj,:)
        matf32_submatrix_copy(P, P_row, 1, 0, 0, 0, 1, M.num_cols); // P_row = P(2,:)

        matf32_submatrix_copy(Pj_row, P, 0, 0, 1, 0, 1, M.num_cols); // P(2,:) = P(exj,:)
        matf32_submatrix_copy(P_row, P, 0, 0, exj, 0, 1, M.num_cols); // P(exj,:) = P(1:0)
    }

    matf32_t* Pt = &m2;
    matf32_init(Pt, P->num_cols, P->num_rows, m2data);
    matf32_zeros(Pt);

    matf32_t* PM = &m3;
    matf32_init(PM, P->num_rows, M.num_cols, m3data);
    matf32_zeros(PM);

    matf32_t* PMPt = &m4;
    matf32_init(PMPt, P->num_rows, P->num_rows, m4data);
    matf32_zeros(PMPt);

    matf32_mul(P, &M, PM); // PM = P*M
    matf32_trans(P, Pt); // P = P'
    matf32_mul(PM, Pt, PMPt); // PMPt = P*M*P'

    // --------------- Extract Submatrices from PMP' ---------------

    matf32_t* E = &m3; // E for saving: PMPt(1:s, 1:s)
    matf32_init(E, s, s, m3data); // Initialize E
    matf32_zeros(E); // Erase previous data in m3data

    matf32_t* C = &m5; // C for saving: PMPt(s+1:end, 1:s)
    matf32_init(C, M.num_rows-s, s, m5data); // Initialize C

    matf32_t* B = &m6; // B for saving: PMPt(s+1:end, s+1:end)
    matf32_init(B, M.num_rows-s, M.num_cols-s, m6data); // Initialize B

    // Assign values for matrices E, C and B
    matf32_submatrix_copy(PMPt, E, 0, 0, 0, 0, E->num_rows, E->num_cols); // E = PMPt(1:s, 1:s)
    matf32_submatrix_copy(PMPt, C, s, 0, 0, 0, C->num_rows, C->num_cols); // C = PMPt(s+1:end, 1:s)
    matf32_submatrix_copy(PMPt, B, s, s, 0, 0, B->num_rows, B->num_cols); // B = PMPt(s+1:end, s+1:end)

    // --------------- Generate lower triangle matrix L and diagonalmatrix D ---------------

    float L_data[MAX_MAT_SIZE];
    matf32_t L;
    matf32_init(&L, M.num_rows, M.num_cols, L_data);
    matf32_zeros(&L);
    matf32_eye(&L);

    matf32_t* Ei = &m7; // Ei for saving: E^-1
    matf32_init(Ei, E->num_rows, E->num_cols, m7data); // Initialize Ei
    matf32_inv(E, Ei); // Ei = E^-1

    matf32_t* CEi = &m8; // CEi for saving: C*E^-1
    matf32_init(CEi, C->num_rows, Ei->num_cols, m8data); // Initialize CEi
    matf32_mul(C, Ei, CEi); // CEi = C*E^-1

    // Assign values for L = [Is 0; C*E^-1 In-s]
    matf32_submatrix_copy(CEi, &L, 0, 0, s, 0, CEi->num_rows, CEi->num_cols); 

    matf32_t* D = &m7; // D for saving [E 0; 0 C*E^-1*C']
    matf32_init(D, M.num_rows, M.num_cols, m7data); // Initialize D
    matf32_zeros(D); // Erase previous data in m7data
    
    matf32_t* Ct = &m9; // Ct for the transpose of C
    matf32_init(Ct, C->num_cols, C->num_rows, m9data); // Initialize Ct

    matf32_t* CEiCt = &m10; // CEiCt for saving: C*E^-1*C'
    matf32_init(CEiCt, C->num_rows, C->num_rows, m10data); // Initialize CEiCt
    matf32_trans(C, Ct); // Ct = C'
    matf32_mul(CEi, Ct, CEiCt); // C = C*E^-1*C'

    matf32_t* B_CEiCt = &m8; // B_CEiCt for saving: B - C*E^-1*C'
    matf32_init(B_CEiCt, C->num_rows, C->num_rows, m8data);
    matf32_zeros(B_CEiCt); // Erase previous data in m8data
    matf32_sub(B, CEiCt, B_CEiCt); // CEiCt = B - C*E^-1*C'

    // Assign values for D = [E 0; 0 CEiCt]
    matf32_submatrix_copy(E, D, 0, 0, 0, 0, E->num_rows, E->num_cols);
    matf32_submatrix_copy(B_CEiCt, D, 0, 0, E->num_rows, E->num_cols, B_CEiCt->num_rows, B_CEiCt->num_cols);

    matf32_t* Lt = &m3; // Lt for the transpose of L
    matf32_init(Lt, L.num_cols, L.num_rows, m3data); // Initialize Lt
    matf32_zeros(Lt); // Erase previous data in m3data

    // ---------------------------------------------------------------------------
    // 3. Solve KKT system My = n with the LDL' Factorization
    // ---------------------------------------------------------------------------

    // --------------- Solve linear system: L*z1 = Pb ---------------
    
    matf32_t* z1 = &m5; // Define z1
    matf32_init(z1, M.num_rows, 1, m5data); // Initialize z1
    matf32_zeros(z1); // Erase previous data in m5data

    matf32_t* Pb = &m6; // Define Pb
    matf32_init(Pb, P->num_rows, n.num_cols, m6data); // Initialize Pb
    matf32_zeros(Pb); // Erase previous data in m6data

    matf32_mul(P, &n, Pb); // Pb = P*n (n = b, just different notation)
    linsolve(&L, Pb, z1); // Solve L*z1 = Pb

    // --------------- Solve linear system: D*z2 = z1 ---------------
    
    matf32_t* z2 = &m6; // z2 for the solution of D*z2 = z1
    matf32_init(z2, M.num_rows, 1, m6data); // Initialize z2
    matf32_zeros(z2); // Erase previous data in m6data

    linsolve(D, z1, z2); // Solve D*z2 = z1

    // --------------- Solve linear system: L'*z3 = z2 ---------------
    
    matf32_t* z3 = &m5; // Define z3
    matf32_init(z3, M.num_rows, 1, m5data); // Initialize z3
    matf32_zeros(z3); // Erase previous data in m5data

    matf32_trans(&L, Lt); // L = L'
    linsolve(Lt, z2, z3); // Solve L'*z3 = z2

    // --------------- Calculate QP solution vector y = P'*z3 ---------------
    
    matf32_init(&y, M.num_rows, 1, y_data); // Initialize y
    matf32_mul(Pt, z3, &y); // y = P'*z3

    // y = [x lambda]', so save x and discard the lambda (lagrange multipliers)
    matf32_submatrix_copy(&y, p_x, 0, 0, 0, 0, p_c->num_rows, 1);

    return QP_SUCESS;
}


// active set binding direction method
quadprog_status_t
quadprog_sqp(quadprog_t* p_qp, matf32_t* const p_x)
{
    /**
     *  Based in the Algorithm 5.4 of Martins, Engineering Design Optimization, modified to manage cases with both equality and inequality constraints.
     *  The original algorithm in the book solves the KKT system: 
     * 
     *      | Q   Aeq' A_w' | |   p_k     |     | Q*x_k + c |
     *      | Aeq  0    0   | | lambda_eq | = - |     0     | 
     *      | A_w  0    0   | | lambda_wk |     |     0     |
     *  
     *  which is explained to apply assuming the equality contraints were already fulfilled.
     *
     *  In this function, it's applied without that assumption, so that all constraints are considered in the algorithm. So it's broken down in four cases:
     *
     *      - Case 1: Equality and Active Inequality Constraints:    | Q   Aeq' A_w' | |   p_k     |     |   Q*x_k + c   |
     *                                                               | Aeq  0    0   | | lambda_eq | = - | Aeq*x_k - beq | 
     *                                                               | A_w  0    0   | | lambda_wk |     | A_w*x_k - b_w |
     *
     *      - Case 2: Equality Constraints (no Active Inequalities):     | Q    Aeq' | |    p_k    | = - |   Q*x_k + c   |  
     *                                                                   | Aeq   0   | | lambda_eq |     | Aeq*x_k - beq |
     *
     *      - Case 3: Active Inequality Constraints:     | Q    A_w' | |    p_k    | = - |   Q*x_k + c   |
     *                                                   | A_w   0   | | lambda_wk |     | A_w*x_k - b_w |
     *
     *      - Case 4: No constraints: Solve the unconstrained system Q*p_k = -(Q*x_k + c). There are no Lagrange Multipliers in this case.
     *
     *  From the systems above, the variables names are as follows:
     * 
     *      Q         : Quadratic term matrix of the QP
     *      c         : Linear term matrix of the QP
     *      Aeq       : Equality constraints matrix of the QP
     *      beq       : Equality constraints vector of the QP
     *      A_w       : Active inequality constraints matrix (Working Set Matrix)
     *      b_w       : Active inequality constraints vector (Working Set Vector) 
     * 
     *      p_k       : Actual solution to the KKT system of a given iteration.
     *      lambda_eq : Lagrange Multipliers corresponding to equality constraints.
     *      lambda_wk : Lagrange Multipliers corresponding to inequality constraints. 
     * 
     *  This way, if there are no equality constraints or all constraints are inactive, the KKT system is built only with what is actually active/exists,
     *  which prevents building matrices with too many zeros, and so, reduces the risk of ill-conditioned KKT matrices.
     *  
     *  Lagrange multipliers are only considered for active inequality constraints (lambda_wk: lambda working set) to identify the most negative one (sigma),
     *  and the row corresponding to that multiplier in the Working Set is removed to deactivate that constraint before the next iteration.
     *  
     *  In case there's no negative multiplier, sigma is set to a large enough positive value to fulfill the conditions of the remaining steps (sigma = 1/0 = Inf).
     *  
     *  The inactive inequality constraints are used to calculate a coefficient alpha, which is used to compute the next step: x_k = x_k + alpha*p_k;
    */

    err_status_t err_status;
    quadprog_status_t quadprog_status;

    const matf32_t* Q = p_qp->p_Q;
    const matf32_t* c = p_qp->p_c;
    const matf32_t* Aeq = p_qp->p_Aeq;
    const matf32_t* beq = p_qp->p_beq;
    const matf32_t* Ain = p_qp->p_Ain;
    const matf32_t* bin = p_qp->p_bin;

    float x_k_data[MAX_MAT_SIZE];
    matf32_t x_k;
    matf32_init(&x_k, Q->num_rows, 1, x_k_data);

    float p_k_data[MAX_MAT_SIZE]; // Subproblem Solution Vector
    matf32_t p_k;
    matf32_init(&p_k, Q->num_rows, 1, p_k_data);

    matf32_t* A_w = &m1; // Working Set constraints matrix
    matf32_t* b_w = &m2; // Working Set constraints vector

    float lambda_data[MAX_MAT_SIZE];
    matf32_t lambda; // Lagrange Multipliers Vector

    // Flags
    bool index_flags[Ain->num_rows]; // Flags for active/inactive inequality rows
    bool equality_constraints = false; // Flag to check if there exist any equlaity constraint
    uint16_t working_set_rows = 0; // Counter for the amount of active rows

#ifdef MATH_MATRIX_CHECK
    // TODO: check size, positive definite
#endif

    // If available, set initial point
    if (NULL == p_qp->p_x0)
    {
        matf32_zeros(&x_k);
    }
    else
    {
        matf32_copy(p_qp->p_x0, &x_k);
    }

    // Set all flags to zero to ensure there's no garbage values
    for (uint16_t i = 0; i < Ain->num_rows; ++i)
    {
        index_flags[i] = false;
    }

    // Check Aeq and beq dimensions
    if (NULL != Aeq && NULL != beq)
    {
        if (Aeq->num_rows > 0 && beq->num_rows > 0)
            equality_constraints = true;
    }

    matf32_t* temp_row = &m3;

    uint16_t active_index = 0;
    for (uint16_t i = 0; i < Ain->num_rows; ++i)
    {
        matf32_init(temp_row, 1, Ain->num_cols, m3data);
        matf32_zeros(temp_row);
        matf32_submatrix_copy(Ain, temp_row, i, 0, 0, 0, 1, Ain->num_cols); // temp_row = Ain(i,:)

        // Condition to identify active rows: Ain(i,:)*x_k - bin
        matf32_init(temp_row, 1, Ain->num_cols, m3data);
        matf32_zeros(temp_row);
        matf32_submatrix_copy(Ain, temp_row, i, 0, 0, 0, 1, Ain->num_cols); // temp_row = Ain(i,:)

        float Ajp_b = 0;
        // Calculate Ain(i,:)*x - bin(i,:)
        matf32_dot(temp_row, &x_k, &Ajp_b); // Ajp_k_b = Ain(i,:)*x
        Ajp_b = Ajp_b - bin->p_data[i]; // Ajp_k_b = Ain(i,:)*x - bin(i,:)

        // Active rows that fullfill Ajp_k - b smaller than the tolerance 1E-05
        if (fabs(Ajp_b) < 1E-05)
        {
            index_flags[i] = true;
            working_set_rows += 1;
            active_index += 1;
        }
    }

    // Main Loop to solve the problem
    for (uint16_t i = 0; i < 30; ++i)
    {
        matf32_t* l = &m4;
        matf32_zeros(&p_k);

        if (equality_constraints) // Equality contraints exist
        {
            if (working_set_rows > 0) // Equality Constraints + Active Inequality Constraints
            {
                //printf("Iteration %i, Case 1: Active Constraints + Active Inequality Constraints QP\n\n", i+1);

                // Create A_w and b_w
                matf32_init(A_w, working_set_rows, Ain->num_cols, m1data);
                matf32_zeros(A_w);
                matf32_init(b_w, working_set_rows, 1, m2data);
                matf32_zeros(b_w);

                // Solve the problem depending on the active rows
                uint16_t current_row = 0;
                for (uint16_t i = 0; i < Ain->num_rows; ++i)
                {
                    // Assign active rows to the Working set matrices A_w and b_w
                    if (index_flags[i])
                    {
                        matf32_init(temp_row, 1, Ain->num_cols, m3data); // Define matrix for A_w row
                        matf32_submatrix_copy(Ain, temp_row, i, 0, 0, 0, 1, Ain->num_cols); // temp_row = Ain(i,:)
                        matf32_submatrix_copy(temp_row, A_w, 0, 0, current_row, 0, 1, Ain->num_cols); // A_w(current_row,:) = Ain(i,:)
                        
                        matf32_set_row(b_w, current_row, bin->p_data[i]); // Assign b_w row

                        current_row += 1;
                    }
                }

                // Create KKT Matrix M 
                matf32_init(&M, Q->num_rows+Aeq->num_rows+A_w->num_rows, Q->num_cols+Aeq->num_rows+A_w->num_rows, M_data);
                matf32_zeros(&M);
                
                // M = [Q 0 0; Aeq 0 0; A_w 0 0]
                matf32_submatrix_copy(Q, &M, 0, 0, 0, 0, Q->num_rows, Q->num_cols); // Copy Q into M
                matf32_submatrix_copy(Aeq, &M, 0, 0, Q->num_rows, 0, Aeq->num_rows, Aeq->num_cols); // Copy Aeq into M
                matf32_submatrix_copy(A_w, &M, 0, 0, Q->num_rows+Aeq->num_rows, 0, A_w->num_rows, A_w->num_cols); // Copy A_w into M

                // M = [Q Aeq' 0; Aeq 0 0; A_w 0 0]
                matf32_t* Aeqt = &m5;
                matf32_init(Aeqt, Aeq->num_cols, Aeq->num_rows, m5data);
                matf32_trans(Aeq, Aeqt); // Aeq'
                matf32_submatrix_copy(Aeqt, &M, 0, 0, 0, Q->num_cols, Aeqt->num_rows, Aeqt->num_cols); // Copy Aeq' into M

                // M = [Q Aeq' A_w'; Aeq 0 0; A_w 0 0]
                matf32_t* A_wt = &m5;
                matf32_init(A_wt, A_w->num_cols, A_w->num_rows, m5data);
                matf32_zeros(A_wt);
                matf32_trans(A_w, A_wt);
                matf32_submatrix_copy(A_wt, &M, 0, 0, 0, Q->num_cols+Aeq->num_rows, A_wt->num_rows, A_wt->num_cols); // Save A_w' into M

                // Create KKT vector l
                matf32_init(l, Q->num_rows+Aeq->num_rows+A_w->num_rows, 1, m4data);

                // l = [Q*x_k + c; 0; 0]
                matf32_t* Qx_c = &m5;
                matf32_init(Qx_c, Q->num_rows, x_k.num_cols, m5data);
                matf32_mul(Q, &x_k, Qx_c);
                matf32_add(Qx_c, c, Qx_c);
                matf32_submatrix_copy(Qx_c, l, 0, 0, 0, 0, Qx_c->num_rows, 1); // Copy -Q*x_k -b into l

                // l = [Q*x_k + c; Aeq*x_k - beq; 0] 
                matf32_t* Aeqx_b = &m6;
                matf32_init(Aeqx_b, Aeq->num_rows, x_k.num_cols, m6data);
                matf32_mul(Aeq, &x_k, Aeqx_b);
                matf32_sub(Aeqx_b, beq, Aeqx_b);
                matf32_submatrix_copy(Aeqx_b, l, 0, 0, Qx_c->num_rows, 0, Aeqx_b->num_rows, 1); // Copy Aeq*x_k - beq into l

                // l = [Q*x_k + c; Aeq*x_k - beq; A_w*x_k - b_w]
                matf32_t* Awx_b = &m5;
                matf32_init(Awx_b, A_w->num_rows, x_k.num_cols, m5data);
                matf32_mul(A_w, &x_k, Awx_b);
                matf32_sub(Awx_b, b_w, Awx_b);
                matf32_submatrix_copy(Awx_b, l, 0, 0, Q->num_rows+Aeq->num_rows, 0, Awx_b->num_rows, 1); // Copy A_w*x_k - b_w into l

                // l = -[Q*x_k + c; Aeq*x_k - beq; A_w*x_k - b_w]
                matf32_scale(l, -1.0, l);

                float p_data[MAX_MAT_SIZE];
                matf32_t p;
                matf32_init(&p, l->num_rows, 1, p_data);

                err_status = linsolve(&M, l, &p);
                matf32_init(&lambda, A_w->num_rows, 1, lambda_data);
                matf32_submatrix_copy(&p, &p_k, 0, 0, 0, 0, p_k.num_rows, 1);
                matf32_submatrix_copy(&p, &lambda, Q->num_rows+Aeq->num_rows, 0, 0, 0, lambda.num_rows, 1);
            }
            else // Equality Constraints + Inactive inequality constraints
            {
                //printf("Iteration %i, Case 2: Active Constraints QP + No Inequality Constraints\n\n", i+1);

                // M = [Q Aeq'; Aeq 0]
                matf32_init(&M, Q->num_rows+Aeq->num_rows, Q->num_cols+Aeq->num_rows, M_data);
                matf32_zeros(&M);
                matf32_submatrix_copy(Q, &M, 0, 0, 0, 0, Q->num_rows, Q->num_cols); // Copy Q into M
                matf32_submatrix_copy(Aeq, &M, 0, 0, Q->num_rows, 0, Aeq->num_rows, Aeq->num_cols); // Copy Aeq into M

                matf32_t* Aeqt = &m5;
                matf32_init(Aeqt, Aeq->num_cols, Aeq->num_rows, m5data);
                matf32_trans(Aeq, Aeqt); // Aeq'
                matf32_submatrix_copy(Aeqt, &M, 0, 0, 0, Q->num_cols, Aeqt->num_rows, Aeqt->num_cols); // Copy Aeq' into M

                // l = [-Q*x_k - c; Aeq*x_k - beq]
                matf32_init(l, Q->num_rows + Aeq->num_rows, 1, m4data);

                // -Q*x_k - c
                matf32_t* Qx_c = &m5;
                matf32_init(Qx_c, Q->num_rows, x_k.num_cols, m5data);
                matf32_mul(Q, &x_k, Qx_c);
                matf32_scale(Qx_c, -1.0, Qx_c);
                matf32_sub(Qx_c, c, Qx_c);
                matf32_submatrix_copy(Qx_c, l, 0, 0, 0, 0, Qx_c->num_rows, 1); // Copy -Q*x_k -b into l

                // Aeq*x_k - beq
                matf32_t* Ax_b = &m6;
                matf32_init(Ax_b, Aeq->num_rows, x_k.num_cols, m6data);
                matf32_mul(Aeq, &x_k, Ax_b);
                matf32_sub(Ax_b, beq, Ax_b);
                matf32_scale(Ax_b, -1.0, Ax_b);
                matf32_submatrix_copy(Ax_b, l, 0, 0, Qx_c->num_rows, 0, Ax_b->num_rows, 1); // Copy Aeq*x_k - beq into l

                float p_data[MAX_MAT_SIZE];
                matf32_t p;
                matf32_init(&p, l->num_rows, 1, p_data);

                err_status = linsolve(&M, l, &p);
                matf32_init(&lambda, 1, 1, lambda_data);
                matf32_zeros(&lambda);
                matf32_submatrix_copy(&p, &p_k, 0, 0, 0, 0, p_k.num_rows, 1);
            }
        }
        else // No equality constraints exist
        {
            if (working_set_rows > 0) // No equality constraints + Inequality constraints active
            {
                //printf("Iteration %i, Case 3: Inequality Constraints QP + No Equality Constraints\n\n", i+1);

                // Create A_w and b_w
                matf32_init(A_w, working_set_rows, Ain->num_cols, m1data);
                matf32_zeros(A_w);
                matf32_init(b_w, working_set_rows, 1, m2data);
                matf32_zeros(b_w);

                // Solve the problem depending on the active rows
                uint16_t current_row = 0;
                for (uint16_t i = 0; i < Ain->num_rows; ++i)
                {
                    // Assign active rows to the Working set matrices A_w and b_w
                    // For now, this will be zero as no flags are being activates, so I'll assume this works.
                    if (index_flags[i])
                    {
                        matf32_init(temp_row, 1, Ain->num_cols, m3data); // Define matrix for A_w row
                        matf32_submatrix_copy(Ain, temp_row, i, 0, 0, 0, 1, Ain->num_cols); // temp_row = Ain(i,:)
                        matf32_submatrix_copy(temp_row, A_w, 0, 0, current_row, 0, 1, Ain->num_cols); // A_w(current_row,:) = Ain(i,:)
                        
                        matf32_set_row(b_w, current_row, bin->p_data[i]); // Assign b_w row

                        current_row += 1;
                    }
                }

                // M = [Q A_w'; A_w 0]
                matf32_init(&M, Q->num_rows+A_w->num_rows, Q->num_cols+A_w->num_rows, M_data);
                matf32_zeros(&M);
                matf32_submatrix_copy(Q, &M, 0, 0, 0, 0, Q->num_rows, Q->num_cols);
                matf32_submatrix_copy(A_w, &M, 0, 0, Q->num_rows, 0, A_w->num_rows, A_w->num_cols);

                matf32_t* A_wt = &m5;
                matf32_init(A_wt, A_w->num_cols, A_w->num_rows, m5data);
                matf32_zeros(A_wt);
                matf32_trans(A_w, A_wt);
                matf32_submatrix_copy(A_wt, &M, 0, 0, 0, Q->num_cols, A_wt->num_rows, A_wt->num_cols);

                // n = [-Q*x_k - c; A_w*x_k - b_w]
                matf32_init(l, Q->num_rows + A_w->num_rows, 1, m4data);
                matf32_zeros(l);

                matf32_t* Qx_c = &m5;
                matf32_init(Qx_c, Q->num_rows, x_k.num_cols, m5data);
                matf32_mul(Q, &x_k, Qx_c);
                matf32_scale(Qx_c, -1.0, Qx_c);
                matf32_sub(Qx_c, c, Qx_c);
                matf32_submatrix_copy(Qx_c, l, 0, 0, 0, 0, Qx_c->num_rows, 1);

                matf32_t* Ax_b = &m6;
                matf32_init(Ax_b, A_w->num_rows, x_k.num_cols, m6data);
                matf32_mul(A_w, &x_k, Ax_b);
                matf32_sub(Ax_b, b_w, Ax_b);
                matf32_submatrix_copy(Ax_b, l, 0, 0, Qx_c->num_rows, 0, Ax_b->num_rows, 1);

                float p_data[MAX_MAT_SIZE];
                matf32_t p;
                matf32_init(&p, l->num_rows, 1, p_data);

                //printf("M:\n");
                //matf32_print(&M);
                //printf("l:\n");
                //matf32_print(l);

                err_status = linsolve(&M, l, &p);
                matf32_init(&lambda, Ax_b->num_rows, 1, lambda_data);
                matf32_submatrix_copy(&p, &p_k, 0, 0, 0, 0, p_k.num_rows, 1);
                matf32_submatrix_copy(&p, &lambda, Qx_c->num_rows, 0, 0, 0, lambda.num_rows, 1);
            }
            else // No equality constraints + No inequality constraints (unconstrained problem)
            {
                //printf("Iteration %i, Case 4: Unconstrained QP\n\n", i+1);

                matf32_init(l, Q->num_rows, 1, m4data);
                matf32_mul(Q, &x_k, l); // n = Q*x
                matf32_scale(l, -1.0, l); // n = -Q*x
                matf32_sub(l, c, l); // n = -Q*x - c
                //printf("Q:\n");
                //matf32_print(Q);
                //printf("l:\n");
                //matf32_print(l);
                
                err_status = linsolve(Q, l, &p_k); // Solving Qp = -c because in this case there are no constraints
                matf32_init(&lambda, 1, 1, lambda_data);
                matf32_zeros(&lambda); // No constraints = no Lagrange Multipliers
            }
        }

        // Identify sigma from the values in lambda
        float sigma = 0; // To save the most negative Lagrange Multiplier in lambda
        uint16_t sigma_index = 0; // Index of lambda where sigma is located

        // Find most negative sigma (if available)
        for (uint16_t k = 0; k < lambda.num_rows; ++k)
        {
            //printf("sign lambda.p_data[k]: %.9f\n", sign(lambda.p_data[k]));
            if (sign(lambda.p_data[k]) < 0) // Get most negative Lagrange Multiplier
            {
                //printf("Inside sigma loop\n\n");
                if (lambda.p_data[k] < sigma)
                {
                    sigma = lambda.p_data[k];
                    sigma_index = k;
                }
            }
        }

        if (sign(sigma) >= 0) // In case there's no negative sigma
        {
            sigma = 1.0/0.0; // Infinite to ensure sigma it's big enough
        }

        // Check whether norm(p_k) smaller than tolerance
        //printf("norm p_k: %.9f\n", norm(p_k.p_data, p_k.num_rows, p_k.num_cols));
        if (norm(p_k.p_data, p_k.num_rows, p_k.num_cols) < 1E-05)
        {
            // If sigma is positive, that means the KKT conditions are fulfilled = Problem solved
            if (sigma >= 0)
            {
                matf32_copy(&x_k, p_x);
                return QP_SUCESS;
            }
            else // sigma is negative, then remove the corresponding condition from the working set
            {
                // Zero out the corresponding index to exclude it from the working set
                index_flags[sigma_index] = 0;

                // Reduce working_set_rows count
                working_set_rows = working_set_rows - 1;

                // Zero out the corresponding rows in the Working Set
                matf32_set_row(A_w, sigma_index, 0);
                matf32_set_row(b_w, sigma_index, 0);
            }
        }
        else
        {
            float alpha = 1.0; // Initial value as the book indicates
            float alpha_b = 0;
            int16_t blocking_index = -1; // 0 is a valid index, so control value is -1

            float Ajp = 0;
            float Ajx = 0;
            for (uint16_t j = 0; j < Ain->num_rows; ++j)
            {
                if (!index_flags[j])
                {
                    matf32_init(temp_row, 1, Ain->num_cols, m3data);
                    matf32_zeros(temp_row); // Clean temp_row from previous data
                    matf32_submatrix_copy(Ain, temp_row, j, 0, 0, 0, 1, Ain->num_cols); // temp_row = A_w(j,:)
                    matf32_dot(temp_row, &p_k, &Ajp); // Ajp_k = A_w(j,:)*p_k

                    // If A_w(j,:)*p_k is greater than a tolerance
                    if (Ajp > 1E-05)
                    {
                        matf32_dot(temp_row, &x_k, &Ajx); // A_w(j,:)*x_k
                        Ajx = bin->p_data[j] - Ajx; // b_w(j) - A_w(j,:)*x_k
                        alpha_b = Ajx/Ajp;

                        // If alpha_b is smaller than alpha, update alpha and set blocking index
                        if (alpha_b < alpha)
                        {
                            alpha = alpha_b;
                            blocking_index = j;
                        }
                    }
                }
            }

            // If there's a blocking index (blocking_index 0 or greater), add it to the working set
            if (blocking_index > -1)
            {
                working_set_rows = working_set_rows + 1;
                index_flags[blocking_index] = 1;
            }

            // x_k = x_k + alpha*p_k
            matf32_scale(&p_k, alpha, &p_k); // p_k = alpha*p_k
            matf32_add(&x_k, &p_k, &x_k); // x_k = x_k + alpha*p_k
        }
    }
}

/**
 * @}
 */