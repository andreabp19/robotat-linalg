/**
 * @file linsolve.c
 * 
 * Modified 3 Aug 2025
 *      By: Andrea Pineda
 */

#include "linsolve.h"

static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float p1[MAX_VEC_SIZE];


void
linsolve_print_method(linsolve_method_t lsm)
{
    switch (lsm)
    {
        case FORWARD_SUBS:
            printf("FORWARD_SUBS\n");
            break;

        case BACKWARD_SUBS:
            printf("BACKWARD_SUBS\n");
            break;

        case CHOLESKY:
            printf("CHOLESKY\n");
            break;

        case QR:
            printf("QR\n");
            break;

        case LU:
            printf("LU\n");
            break;
    }
}

linsolve_method_t
linsolve_get_method(const matf32_t* const p_a)
{
    if (!matf32_check_square_matrix(p_a))
    {
        return QR;
    }

    // triangular matrix
    if (matf32_check_triangular_lower(p_a))
    {
        return FORWARD_SUBS;
    }

    if (matf32_check_triangular_upper(p_a))
    {
        return BACKWARD_SUBS;
    }

    // definite positive matrix
    // TODO: add check for definite positive
    // cholesky only should be used on postive definite
    if (matf32_check_symmetric(p_a) && false)
    {
        return CHOLESKY;
    }

    // add method for hessenberg?

    return LU; //general square solver
}


err_status_t
linsolve_fwdsubs_matf32(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_triangular_lower(p_l))
    {
        return MATH_ARGUMENT_ERROR;
    }
#endif

    float* p_data_l = p_l->p_data;
    float* p_data_x = p_x->p_data;
    float* p_data_b = p_b->p_data;

    float lx = 0; // sum accumulator

    for (uint16_t i = 0; i < p_l->num_rows; ++i) 
    {
        // reset lx
        lx = 0;

        // calculate sum x_i * l_(i,j)
        for (uint16_t j = 0; j < i; ++j)
        {
            lx += p_data_x[j]*p_data_l[i*p_l->num_rows + j];
        }

        // calculate x_i
        p_data_x[i] = (p_data_b[i] - lx)/p_data_l[i*p_l->num_rows + i];
    }

    return MATH_SUCCESS;
}


err_status_t
linsolve_bwdsubs_matf32(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_triangular_upper(p_u))
    {
        return MATH_ARGUMENT_ERROR;
    }
#endif

    float* p_data_u = p_u->p_data;
    float* p_data_x = p_x->p_data;
    float* p_data_b = p_b->p_data;

    float ux = 0; // sum accumulator

    for (int16_t i = p_u->num_rows-1; i >= 0; --i) 
    {
        // reset ux
        ux = 0;
        // calculate sum x_i * u_(i,j)
        for (uint16_t j = p_u->num_cols-1; j>i; --j)
        {
            ux += p_data_x[j]*p_data_u[i*p_u->num_rows + j];
        }

        // calculate x_i
        p_data_x[i] = (p_data_b[i] - ux)/p_data_u[i*p_u->num_rows + i];
    }
    return MATH_SUCCESS;
}


// make inline to reduce call stack?
err_status_t
linsolve_matf32(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* const p_x)
{
    linsolve_method_t method = linsolve_get_method(p_a);

    return linsolve_matf32_method(p_a, p_b, p_x, method);
}

// TODO:
// qr_solve
err_status_t
linsolve_matf32_method(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t*  const p_x, linsolve_method_t method)
{
    err_status_t status;

    switch (method)
    {
        case FORWARD_SUBS:
            return linsolve_fwdsubs_matf32(p_a, p_b, p_x);
            break;

        case BACKWARD_SUBS:
            return linsolve_bwdsubs_matf32(p_a, p_b, p_x);
            break;

        case CHOLESKY:
            
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);
            matf32_zeros(&m1);


            status = matf32_cholesky(p_a, &m1);

            if (MATH_SUCCESS != status)
            {
                return status;
            }


            status = linsolve_cholesky_matf32(&m1, p_b, p_x);

            return status;
            break;

        case QR:
            // TODO
            break;

        case LU:
            // matrix L
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);
            matf32_zeros(&m1);

            // matrix U
            matf32_init(&m2, p_a->num_rows, p_a->num_rows, m2data);
            matf32_zeros(&m2);

            status = matf32_lu(p_a, &m1, &m2);

            if (MATH_SUCCESS != status)
            {
                return status;
            }

            status = matf32_lu_solve(&m1, &m2, p_b, p_x);

            return status;
            break;
    }
}

err_status_t
matf32_lu_solve(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x)
{
    err_status_t status;

    float* y_data = p1;
    matf32_t y;
    matf32_init(&y, p_l->num_rows, 1, y_data);
    matf32_zeros(&y);

    status = linsolve_fwdsubs_matf32(p_l, p_b, &y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    status = linsolve_bwdsubs_matf32(p_u, &y, p_x);

    return status;
}

err_status_t
linsolve_cholesky_matf32(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x)
{
    err_status_t status;

    float* y_data = p1;
    matf32_t y;
    matf32_init(&y, p_c->num_rows, 1, y_data);
    matf32_zeros(&y);

    status = linsolve_fwdsubs_matf32(p_c, p_b, &y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    matf32_trans(p_c, p_c);

    status = linsolve_bwdsubs_matf32(p_c, &y, p_x);

    return status;
}