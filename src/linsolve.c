/**
 * @file linsolve.c
 */

#include "linsolve.h"

// temporary
// fix to use the temp matrices at matf32_math.c
static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static uint16_t p1[MAX_VEC_SIZE];


void
print_linsolve_method(linsolve_method_t lsm)
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
matf32_linsolve_get_method(const matf32_t* const p_a)
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


    // symmetric matrix
    // TODO: add check for hermitian and self-adjoint?

    if (matf32_check_symmetric(p_a))
    {
        return CHOLESKY;
    }

    // add method for hessenberg?

    return LU;
}


err_status_t
matf32_forward_substitution(const matf32_t* const p_l, const float* const p_b, float* p_x)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_triangular_lower(p_l))
    {
        return MATH_ARGUMENT_ERROR;
    }
#endif

    float* p_data_src = p_l->p_data;

    float lx = 0; // sum accumulator

    for (uint16_t i = 0; i < p_l->num_rows; ++i) 
    {
        // reset lx
        lx = 0;

        // calculate sum x_i * l_(i,j)
        for (uint16_t j = 0; j < i; ++j)
        {
            lx += p_x[j]*p_data_src[i*p_l->num_rows + j];
        }

        // calculate x_i
        p_x[i] = (p_b[i] - lx)/p_data_src[i*p_l->num_rows + i];
    }

    return MATH_SUCCESS;
}


err_status_t
matf32_backward_substitution(const matf32_t* const p_u, const float* const p_b, float* p_x)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_triangular_upper(p_u))
    {
        return MATH_ARGUMENT_ERROR;
    }
#endif

    float* p_data_src = p_u->p_data;

    float ux = 0; // sum accumulator

    for (int16_t i = p_u->num_rows-1; i >= 0; --i) 
    {
        // reset ux
        ux = 0;
        // calculate sum x_i * u_(i,j)
        for (uint16_t j = p_u->num_cols-1; j>i; --j)
        {
            ux += p_x[j]*p_data_src[i*p_u->num_rows + j];
        }

        // calculate x_i
        p_x[i] = (p_b[i] - ux)/p_data_src[i*p_u->num_rows + i];
    }
    return MATH_SUCCESS;
}


// https://www.math.umd.edu/~petersd/401/cholesk.pdf
// revise later
err_status_t
matf32_cholesky(const matf32_t* const p_a, matf32_t* const p_c)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_square_matrix(p_a) || !matf32_check_symmetric(p_a))
    {
        return MATH_ARGUMENT_ERROR;
    }

    // add check for definite positive

    if (!matf32_is_same_size(p_a, p_c))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    float* p_data_a = p_a->p_data;
    float* p_data_c = p_c->p_data;

    uint16_t size = p_a->num_cols;

    float temp_v[size];

    float sum = 0;

    for (uint16_t i = 0; i < p_a->num_rows; ++i)
    {
        // initialize temp_v
        for (int16_t j = 0; j < i; ++j)
        {
            temp_v[j] = p_data_c[i*size + j];
        }

        for (uint16_t j = 0; j < p_a->num_cols; ++j)
        {
            // sum = A(j,j) - v'*v
            sum = p_data_a[i*size + j];
            for (int16_t k = 0; k < i; ++k)
            {
                sum -= temp_v[k] * temp_v[k];
            }

            if (i == j)
            {
                if (sum <= 0.0)
                {
                    return MATH_DECOMPOSITION_FAILURE;
                }

                p_data_c[i*size + i] = sqrtf(sum);
            }
            else
            {
                p_data_c[j*size + i] = sum / p_data_c[i*size + i];
            }
        }
    }

    zero_patch(p_data_c, size*size);

    return MATH_SUCCESS;
}


// doolittle algoritm
err_status_t
matf32_lu(const matf32_t* p_a, matf32_t* p_l, matf32_t* p_u)
{
#ifdef MATH_MATRIX_CHECK 
    if (!matf32_is_same_size(p_a, p_l) || !matf32_is_same_size(p_a, p_u))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    float* p_a_data = p_a->p_data;
    float* p_l_data = p_l->p_data;
    float* p_u_data = p_u->p_data;

    uint16_t rows = p_a->num_rows;

    for (uint16_t i = 0; i < rows; ++i)
    {
        for (uint16_t j = i; j < rows; ++j)
        {
            float sum = 0;
            for (uint16_t k = 0; k < i; ++k)
            {
                sum += p_l_data[i*rows + k] * p_u_data[k*rows + j];
            }

            p_u_data[i*rows + j] = p_a_data[i*rows + j] - sum;
        }

        for (uint16_t j = i; j < rows; ++j)
        {
            if (i == j)
            {
                p_l_data[i*rows + i] = 1;
                continue;
            }

            float sum = 0;
            for (uint16_t k = 0; k < i; ++k)
            {
                sum += p_l_data[j*rows + k] * p_u_data[k*rows + i];
            }

            p_l_data[j*rows + i] = (p_a_data[j*rows + i] - sum) / p_u_data[i*rows + i];
        }
    }
}

// TODO:
// implement a function to check which method to use (DONE)
// a function which solves whith the selected method (DONE)
// and one which automatically solves (this one) (QR PENDING)
// define method as a enum linsolve_method (DONE)
err_status_t
matf32_linsolve(const matf32_t* const p_a, const float* const p_b, float* const p_x)
{
    if (!matf32_check_square_matrix(p_a))
    {
        // QR
    }

    // triangular matrix
    if (matf32_check_triangular_lower(p_a))
    {
        return matf32_forward_substitution(p_a, p_b, p_x);
    }

    if (matf32_check_triangular_upper(p_a))
    {
        return matf32_backward_substitution(p_a, p_b, p_x);
    }


    // symmetric matrix
    // TODO: add check for hermitian and self-adjoint?

    if (matf32_check_symmetric(p_a))
    {
        matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);

        err_status_t status = matf32_cholesky(p_a, &m1);

        if (MATH_SUCCESS != status)
        {
            return status;
        }

        float y[p_a->num_rows];
        zeros(y, p_a->num_rows, 1);

        status = matf32_forward_substitution(&m1, p_b, y);

        if (MATH_SUCCESS != status)
        {
            return status;
        }

        matf32_trans(&m1, &m1);

        status = matf32_backward_substitution(&m1, y, p_x);

        return status;
    }


    // solve hessenberg here?


    // general solver LU

    matf32_t* l = &m1;
    matf32_init(l, p_a->num_rows, p_a->num_rows, m1data);

    matf32_t* u = &m2;
    matf32_init(u, p_a->num_rows, p_a->num_rows, m2data);

    err_status_t status = matf32_lu(p_a, l, u);

    float y[p_a->num_rows];
    zeros(y, p_a->num_rows, 1);

    status = matf32_forward_substitution(l, p_b, y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    status = matf32_backward_substitution(u, y, p_x);

    return status;
}

// simplify with lu_solve, cholesky_solve, etc.
err_status_t
matf32_linsolve_method(const matf32_t* const p_a, const float* const p_b, float* p_x, linsolve_method_t method)
{
    err_status_t status;

    float y[p_a->num_rows];
    zeros(y, p_a->num_rows, 1);

    switch (method)
    {
        case FORWARD_SUBS:
            return matf32_forward_substitution(p_a, p_b, p_x);
            break;

        case BACKWARD_SUBS:
            return matf32_backward_substitution(p_a, p_b, p_x);
            break;

        case CHOLESKY:
            
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);

            status = matf32_cholesky(p_a, &m1);

            if (MATH_SUCCESS != status)
            {
                return status;
            }


            status = matf32_forward_substitution(&m1, p_b, y);

            if (MATH_SUCCESS != status)
            {
                return status;
            }

            matf32_trans(&m1, &m1);

            status = matf32_backward_substitution(&m1, y, p_x);

            return status;
            break;

        case QR:
            // TODO
            break;

        case LU:
            ; // solves error: a label can only be part of a statement and a declaration is not a statement
            matf32_t* l = &m1;
            matf32_init(l, p_a->num_rows, p_a->num_rows, m1data);
    
            matf32_t* u = &m2;
            matf32_init(u, p_a->num_rows, p_a->num_rows, m2data);

            status = matf32_lu(p_a, l, u);

            status = matf32_forward_substitution(l, p_b, y);

            if (MATH_SUCCESS != status)
            {
                return status;
            }

            status = matf32_backward_substitution(u, y, p_x);

            return status;
            break;
    }
}