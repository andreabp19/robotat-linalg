/**
 * @file quadprog.c
 */

#include <stdint.h>

#include "quadprog.h"


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

quadprog_status_t
quadprog(quadprog_t* p_qp, matf32_t* const p_x)
{

    /*
        * Sistema lineal: My=n

        crear M = [Q Aeq'; Aeq 0]
        copiar Q, Aeq en M
        transponer Aeq en temp Aeq
        copiar Aeq' en M

        crear y = [x; lambda] <- you left here

        crear n = [-c, -beq]
        copiar c y beq en n
        n = -n

        resolver My=n

        copiar x desde y
    */

    const matf32_t* p_Q = p_qp->p_Q;
    const matf32_t* p_Aeq = p_qp->p_Aeq;
    const matf32_t* p_c = p_qp->p_c;
    const matf32_t* p_beq = p_qp->p_beq;

    // HERE size check M matrices

    uint16_t rows = p_qp->p_Q->num_rows + p_qp->p_Aeq->num_rows; 
    uint16_t cols = p_qp->p_Q->num_cols + p_qp->p_Aeq->num_rows; // Aeq is used transposed here


    // temp, use file-level defined matrices/arrays
    matf32_t M;
    float M_data[100];
    matf32_init(&M, rows, cols, M_data);
    matf32_zeros(&M);

    matf32_t y;
    float y_data[rows];
    matf32_init(&y, rows, 1, y_data);
    matf32_zeros(&y);

    matf32_t n;
    float n_data[rows];
    matf32_init(&n, rows, 1, n_data);
    matf32_zeros(&n);


    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols);

    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols);

    float temp_Aeq_data[100];
    matf32_t temp_Aeq;
    matf32_init(&temp_Aeq, p_Aeq->num_cols, p_Aeq->num_rows, temp_Aeq_data);

    // transpose A
    matf32_trans(p_Aeq, &temp_Aeq);

    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows);


    matf32_submatrix_copy(p_c,   &n, 0, 0,               0, 0,   p_c->num_rows, 1);
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1);


    matf32_scale(&n, -1, &n);

    matf32_linsolve(&M, &n, &y);

    matf32_submatrix_copy(&y, p_x, 0, 0, 0, 0, p_c->num_rows, 1);
}