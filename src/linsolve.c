/**
 * @file linsolve.c
 */

#include "linsolve.h"

// temporary
// fix to use the temp matrices at matf32_math.c
// or move all preallocations to a single file
static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float p1[MAX_VEC_SIZE];


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

    return LU; //general square solver
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
matf32_lu(const matf32_t* p_a, matf32_t* const p_l, matf32_t* const p_u)
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

    return MATH_SUCCESS;
}

// https://www.cs.cornell.edu/~bindel/class/cs6210-f09/lec18.pdf
err_status_t
matf32_qr(const matf32_t* const p_a, matf32_t* const p_q, matf32_t* const p_r)
{
    // add size checks
    // size(A) == size(R)

    float* p_a_data = p_a->p_data;
    float* p_q_data = p_q->p_data;
    float* p_r_data = p_r->p_data;

    uint16_t rows = p_a->num_rows;
    uint16_t cols = p_a->num_cols;

    uint16_t min_size = rows < cols? rows : cols;

    // init Q and R
    matf32_eye(p_q);
    matf32_copy(p_a, p_r);

    float normx = 0;
    float u1 = 0;
    float tau = 0;

    // init temp vector
    float temp_v[rows];
    float w_vec[rows];
    float w_tau_vec[rows];
    float temp_n[rows];


    // sub matrix from R will always be smaller than R
    float rsub_data[rows*cols];
    matf32_t r_sub;
    matf32_init(&r_sub, rows, cols, rsub_data);

    float rtemp_data[rows*cols];
    matf32_t r_sub_temp;
    matf32_init(&r_sub_temp, rows, cols, rtemp_data);
    matf32_zeros(&r_sub_temp);

    // sub matrix from Q will always be smaller than Q
    float qsub_data[rows*rows];
    matf32_t q_sub;
    matf32_init(&q_sub, rows, rows, qsub_data);
    matf32_zeros(&q_sub);

    float qtemp_data[rows*rows];
    matf32_t q_sub_temp;
    matf32_init(&q_sub_temp, rows, rows, qtemp_data);
    matf32_zeros(&q_sub_temp);

    for (uint16_t i = 0; i < min_size; ++i)
    {
        zeros(temp_v, rows, 1);
        zeros(w_vec, rows, 1);

        //printf("v:\n");
        for (uint16_t j = i; j < rows; ++j)
        {
            temp_v[j-i] = p_r_data[j*cols + i];
            //printf("%i,%i: %f\n", j, i, temp_v[j-i]);
        }

        normx = norm(temp_v, rows, 1);
        //printf("norm: %f\n\n", normx);

        u1 = p_r_data[i*cols + i] + sign(p_r_data[i*cols + i])*normx;

        //printf("w:\n");
        for (uint16_t j = 0; j < rows-i; ++j)
        {
            w_vec[j] = temp_v[j]/u1;
            //printf("%f\n", w_vec[j]);
        }
        w_vec[0] = 1;

        tau = sign(p_r_data[i*cols + i]) * u1 / normx;

        // tau*w
        scale(w_vec, rows-i, tau, w_tau_vec);

        //R(i:end,:)
        matf32_reshape(&r_sub, rows-i, cols);
        matf32_reshape(&r_sub_temp, rows-i, cols);
        matf32_submatrix_copy(p_r, &r_sub, i, 0, 0, 0, rows-i, cols);

        // w'Rsub
        matf32_vecpremul(&r_sub, w_vec, temp_n);

        //(tau*w)*(w'*Rsub)
        matf32_vecmul_col_row(w_tau_vec, temp_n, &r_sub_temp);
        //matf32_print(&r_sub_temp);

        // Rsub -= (tau*w)*(w'*Rsub)
        matf32_sub(&r_sub, &r_sub_temp, &r_sub);

        matf32_submatrix_copy(&r_sub, p_r, 0, 0, i, 0, rows-i, cols);


        // Calculate Q
        matf32_reshape(&q_sub, rows, rows-i);
        matf32_reshape(&q_sub_temp, rows, rows-i);
        matf32_submatrix_copy(p_q, &q_sub, 0, i, 0, 0, rows, rows-i);

        // Qsub*w
        matf32_vecposmul(&q_sub, w_vec, temp_n);

        //(Qsub*w)*(tau*w)'
        matf32_vecmul_col_row(temp_n, w_tau_vec, &q_sub_temp);

        // Qsub -= (Qsub*w)*(tau*w)'
        matf32_sub(&q_sub, &q_sub_temp, &q_sub);

        matf32_submatrix_copy(&q_sub, p_q, 0, 0, 0, i, rows, rows-i);

    }

}

err_status_t
matf32_linsolve(const matf32_t* const p_a, const float* const p_b, float* const p_x)
{
    linsolve_method_t method = matf32_linsolve_get_method(p_a);

    return matf32_linsolve_method(p_a, p_b, p_x, method);
}

// TODO:
// qr_solve
err_status_t
matf32_linsolve_method(const matf32_t* const p_a, const float* const p_b, float* p_x, linsolve_method_t method)
{
    err_status_t status;

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


            status = matf32_cholesky_solve(&m1, p_b, p_x);

            return status;
            break;

        case QR:
            // TODO
            break;

        case LU:
            // matrix L
            matf32_init(&m1, p_a->num_rows, p_a->num_rows, m1data);

            // matrix U
            matf32_init(&m2, p_a->num_rows, p_a->num_rows, m2data);


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
matf32_lu_solve(const matf32_t* const p_l, const matf32_t* const p_u,  const float* const p_b, float* const p_x)
{
    err_status_t status;

    float* y = p1;
    zeros(y, p_l->num_rows, 1);

    status = matf32_forward_substitution(p_l, p_b, y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    status = matf32_backward_substitution(p_u, y, p_x);

    return status;
}

err_status_t
matf32_cholesky_solve(matf32_t* const p_c,  const float* const p_b, float* const p_x)
{
    err_status_t status;

    float* y = p1;
    zeros(y, p_c->num_rows, 1);

    status = matf32_forward_substitution(p_c, p_b, y);

    if (MATH_SUCCESS != status)
    {
        return status;
    }

    matf32_trans(p_c, p_c);

    status = matf32_backward_substitution(p_c, y, p_x);

    return status;
}