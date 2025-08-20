/**
 * @file linsolve.c
 * 
 * Last modified 20 Aug 2025
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
    // Solve with QR if matrix is not square
    if (!matf32_check_square_matrix(p_a))
    {
        return QR;
    }

    // Solve with Forward Substitution if matrix is lower triangular
    if (matf32_check_triangular_lower(p_a))
    {
        return FORWARD_SUBS;
    }

    // Solve with Backward Substitution if matrix is upper triangular
    if (matf32_check_triangular_upper(p_a))
    {
        return BACKWARD_SUBS;
    }

    // Solve with Cholesky if matrix is symmetric positive definite
    if (matf32_check_symposdef(p_a))
    {
        return CHOLESKY;
    }

    // Add check for hermitian matrices
    // This should be checked before the Cholesky and LU factorizations,
    // based on matlab's linear solver diagram.

    // Add method for hessenberg?

    // Add method for tridiagonal matrices

    // Add method for permuted triangular matrices

    // Add method for solving if the conditioning number of the matrix is to high

    return LU; //general square solver
}


err_status_t
linsolve_forward_substitution(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x)
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
linsolve_backward_substitution(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x)
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
linsolve(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* const p_x)
{
    linsolve_method_t method = linsolve_get_method(p_a);

    return linsolve_method(p_a, p_b, p_x, method);
}

// TODO:
// qr_solve
err_status_t
linsolve_method(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t*  const p_x, linsolve_method_t method)
{
    err_status_t status;

    switch (method)
    {
        case FORWARD_SUBS:
            return linsolve_forward_substitution(p_a, p_b, p_x);
            break;

        case BACKWARD_SUBS:
            return linsolve_backward_substitution(p_a, p_b, p_x);
            break;

        case CHOLESKY:
            
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);
            matf32_zeros(&m1);


            status = matf32_cholesky(p_a, &m1);

            if (MATH_SUCCESS != status)
            {
                return status;
            }

            status = linsolve_cholesky(&m1, p_b, p_x);

            return status;
            break;

        case QR:
            
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);
            matf32_zeros(&m1);

            matf32_init(&m2, p_a->num_rows, p_a->num_cols, m2data);
            matf32_zeros(&m2);

            status = matf32_qr(p_a, &m1, &m2);

            if (MATH_SUCCESS != status)
            {
                return status;
            }

            status = linsolve_qr(&m1, &m2, p_b, p_x);
            
            return status;
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

            status = linsolve_LU(&m1, &m2, p_b, p_x);

            return status;
            break;
    }
}

err_status_t
linsolve_qr(matf32_t* const p_q, matf32_t* const p_r, const matf32_t* const p_b, matf32_t* const p_x)
{
    err_status_t status;

    float* y_data = p1;
    matf32_t y;
    matf32_init(&y, p_q->num_rows, 1, y_data);
    matf32_zeros(&y);

    float trans_q_data[MAX_MAT_SIZE];
    matf32_t trans_q;
    matf32_init(&trans_q, p_q->num_rows, p_q->num_cols, trans_q_data);
    matf32_zeros(&trans_q);

    float sub_R_data[MAX_MAT_SIZE];
    matf32_t sub_R;
    matf32_init(&sub_R, p_r->num_rows-1, p_r->num_cols, sub_R_data);
    matf32_zeros(&sub_R);

    // Compute y = Q'b
    matf32_trans(p_q, &trans_q);
    matf32_mul(&trans_q, p_b, &y);

    // R from full QR (as matf32_qr) is n x (n-1), with an extra row of zeros.
    // Take away the extra row of zeros to turn it into upper triangular and solve with backward substitution.
    matf32_submatrix_copy(p_r, &sub_R, 0, 0, 0, 0, sub_R.num_rows, sub_R.num_cols);

    // Solve Rx = y with backward substitution as R is upper triangular
    status = linsolve_backward_substitution(&sub_R, &y, p_x);

    return status;
}

err_status_t
linsolve_lu(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x)
{
    err_status_t status;

    float* y_data = p1;
    matf32_t y;
    matf32_init(&y, p_l->num_rows, 1, y_data);
    matf32_zeros(&y);

    status = linsolve_forward_substitution(p_l, p_b, &y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    status = linsolve_backward_substitution(p_u, &y, p_x);

    return status;
}

err_status_t
linsolve_cholesky(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x)
{
    err_status_t status;

    float* y_data = p1;
    matf32_t y;
    matf32_init(&y, p_c->num_rows, 1, y_data);
    matf32_zeros(&y);

    // Assuming the cholesky decomposition arrives as an upper triangular as matf32_cholesky gives it out by default
    // Necessary to transpose it because forward substitution accepts lower triangular matrices
    matf32_trans(p_c, p_c);

    status = linsolve_forward_substitution(p_c, p_b, &y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    matf32_trans(p_c, p_c);

    status = linsolve_backward_substitution(p_c, &y, p_x);

    return status;
}
