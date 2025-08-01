/**
 * @file quadprog.c
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
        return quadprog_qp(p_qp, p_x);
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
quadprog_qp(quadprog_t* p_qp, matf32_t* const p_x)
{

    /*
        * Linear system: My=n

        create M = [Q Aeq'; Aeq 0]
        copy Q, Aeq in M
        transpose Aeq in temp Aeq
        copy Aeq' en M

        create y = [x; lambda]

        create n = [-c, -beq]
        copy c y beq in n
        n = -n

        solve My=n

        copy x from y
    */

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

    matf32_init(&y, rows, 1, y_data);
    matf32_zeros(&y);

    matf32_init(&n, rows, 1, n_data);
    matf32_zeros(&n);

    matf32_init(&temp_Aeq, p_Aeq->num_cols, p_Aeq->num_rows, temp_Aeq_data);


    matf32_submatrix_copy(p_Q, &M, 0, 0, 0, 0, p_Q->num_rows, p_Q->num_cols);

    matf32_submatrix_copy(p_Aeq, &M, 0, 0, p_Q->num_rows, 0, p_Aeq->num_rows, p_Aeq->num_cols);


    // transpose A
    matf32_trans(p_Aeq, &temp_Aeq);

    matf32_submatrix_copy(&temp_Aeq, &M, 0, 0, 0, p_Q->num_cols, p_Aeq->num_cols, p_Aeq->num_rows);


    matf32_submatrix_copy(p_c,   &n, 0, 0,               0, 0,   p_c->num_rows, 1);
    matf32_submatrix_copy(p_beq, &n, 0, 0, p_c->num_rows, 0, p_beq->num_rows, 1);


    matf32_scale(&n, -1, &n);

    matf32_linsolve(&M, &n, &y);

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
        // prepare subproblem c vector
        matf32_trans(p_Q, p_Q);
        matf32_mul(p_Q, p_x, &sub_c);
        matf32_trans(p_Q, p_Q);
        matf32_add(p_c, &sub_c, &sub_c);
        matf32_scale(&sub_c, -1, &sub_c);

        quadprog_qp(&subproblem, &p);

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
