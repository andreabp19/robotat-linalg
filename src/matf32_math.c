/**
 * @file matf32_math.c
 */

#include "matf32_math.h"


// ====================================================================================================
// Private variables
// ====================================================================================================
// Preallocated auxiliary matrices and vectors to use inside matrix operations. This reduces memory 
// and data structure initialization overhead.
static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float v1[MAX_VEC_SIZE];
static uint16_t p1[MAX_VEC_SIZE];


err_status_t
matf32_add(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (matf32_is_same_size(p_srca, p_srcb))
        if (matf32_is_same_size(p_srca, p_dst));
        else return MATH_SIZE_MISMATCH;
    else return MATH_SIZE_MISMATCH;
#endif

    float* p_data_srca = p_srca->p_data;
    float* p_data_srcb = p_srcb->p_data;
    float* p_data_dst = p_dst->p_data;

    for (int i = 0; i < p_srca->num_rows * p_srca->num_cols; i++)
    {
        *(p_data_dst++) = *(p_data_srca++) + *(p_data_srcb++);
    }

    return MATH_SUCCESS;
}


err_status_t
matf32_sub(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (matf32_is_same_size(p_srca, p_srcb))
    {
        if (!matf32_is_same_size(p_srca, p_dst))
        {
            return MATH_SIZE_MISMATCH;
        }
    }
    else
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    float* p_data_srca = p_srca->p_data;
    float* p_data_srcb = p_srcb->p_data;
    float* p_data_dst = p_dst->p_data;

    for (int i = 0; i < p_srca->num_rows * p_srca->num_cols; i++)
    {
        p_data_dst[i] = p_data_srca[i] - p_data_srcb[i];
    }

    return MATH_SUCCESS;
}


err_status_t
matf32_scale(const matf32_t* p_src, float scalar, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (!matf32_is_same_size(p_src, p_dst))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    uint16_t size = p_src->num_rows*p_src->num_cols;

    scale(p_src->p_data, size, scalar, p_dst->p_data);

    return MATH_SUCCESS;
}


err_status_t
matf32_trans(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (!matf32_size_check(p_dst, p_src->num_cols, p_src->num_rows))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif
    float* p_data_src = p_src->p_data;
    float* p_trans;
    matf32_t* p_tmpmat = &m1;

    matf32_init(p_tmpmat, p_src->num_rows, p_src->num_cols, m1data);

    p_dst->num_rows = p_src->num_cols;
    p_dst->num_cols = p_src->num_rows;

    for (int i = 0; i < p_src->num_rows; i++)
    {
        p_trans = &p_tmpmat->p_data[i];
        for (int j = 0; j < p_src->num_cols; j++)
        {
            *p_trans = *(p_data_src++);
            p_trans += p_src->num_rows;
        }
    }

    memcpy(p_dst->p_data, p_tmpmat->p_data, p_dst->num_rows * p_dst->num_cols * sizeof(float));
    return MATH_SUCCESS;
}


err_status_t
matf32_mul(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    // Check size consistency
    if (!matf32_size_check(p_dst, p_srca->num_rows, p_srcb->num_cols) || (p_srca->num_cols != p_srcb->num_rows))
    {
        return MATH_SIZE_MISMATCH;
    }

    /*if ((p_srca->num_cols != p_srcb->num_rows) || (p_srca->num_rows != p_dst->num_rows) || (p_srcb->num_cols != p_dst->num_cols))
        return MATH_SIZE_MISMATCH;*/
#else
    // Set output matrix dimensions
    p_dst->num_rows = p_srca->num_rows;
    p_dst->num_cols = p_srcb->num_cols;
#endif

    // Checks if one of the inputs is being used to store the output (this is NOT allowed even in the square matrix case)
    if ((p_srca->p_data == p_dst->p_data) || (p_srcb->p_data == p_dst->p_data))
    {
        return MATH_ARGUMENT_ERROR;
    }

    // Data matrix
    float* data_a;
    float* data_b;
    float* data_c = p_dst->p_data;

    for (uint16_t i = 0; i < p_srca->num_rows; i++)
    {
        // Then we go through every column of b
        for (uint16_t j = 0; j < p_srcb->num_cols; j++)
        {
            data_a = &p_srca->p_data[i * p_srca->num_cols];
            data_b = &p_srcb->p_data[j];

            *data_c = 0; // Reset
            // And we multiply rows from a with columns of b
            for (uint16_t k = 0; k < p_srca->num_cols; k++)
            {
                *data_c += *data_a * *data_b;
                data_a++;
                data_b += p_srcb->num_cols;
            }
            data_c++;
        }
    }

    return MATH_SUCCESS;
}


// fix
// move to linsolve
err_status_t
matf32_lup(const matf32_t* p_src, matf32_t* p_lu, uint16_t* pivot)
{
#ifdef MATH_MATRIX_CHECK 
    if (matf32_is_same_size(p_src, p_lu));
    else return MATH_SIZE_MISMATCH;
#endif

    // Check if the input matrix is square 
    if (p_src->num_cols != p_src->num_rows)
    {
        return MATH_SIZE_MISMATCH;
    }

    uint16_t ind_max;
    uint16_t tmp_int;
    uint16_t row = p_lu->num_rows;

    // Don't copy if the pointer to the decomposition data is the same as the input
    if (p_src->p_data != p_lu->p_data)
    {
        memcpy(p_lu->p_data, p_src->p_data, p_src->num_rows * p_src->num_cols * sizeof(float));
    }

    // Create the pivot vector
    for (uint16_t i = 0; i < row; ++i)
    {
        pivot[i] = i;
    }

    for (uint16_t i = 0; i < row - 1; ++i)
    {
        ind_max = i;
        for (uint16_t j = i + 1; j < p_src->num_rows; ++j)
        {
            if (fabsf(p_lu->p_data[row * pivot[j] + i]) > fabsf(p_lu->p_data[row * pivot[ind_max] + i]))
            {
                ind_max = j;
            }
        }

        tmp_int = pivot[i];
        pivot[i] = pivot[ind_max];
        pivot[ind_max] = tmp_int;

        if (fabsf(p_lu->p_data[row * pivot[i] + i]) < FLT_EPSILON)
        {
            return MATH_SINGULAR; // matrix is singular (up to tolerance)
        }

        for (uint16_t j = i + 1; j < row; ++j)
        {
            p_lu->p_data[row * pivot[j] + i] = p_lu->p_data[row * pivot[j] + i] / p_lu->p_data[row * pivot[i] + i];

            for (uint16_t k = i + 1; k < row; ++k)
            {
                p_lu->p_data[row * pivot[j] + k] = p_lu->p_data[row * pivot[j] + k]
                - p_lu->p_data[row * pivot[i] + k] * p_lu->p_data[row * pivot[j] + i];
            }
        }
    }

    return MATH_SUCCESS;
}

// move to linsolve
void 
solve(float* A, float* x, float* b, uint16_t* P, float* LU, uint16_t row) 
{
    // Forward substitution with pivoting
    for (uint16_t i = 0; i < row; ++i) 
    {
        x[i] = b[P[i]];

        for (uint16_t j = 0; j < i; ++j)
            x[i] = x[i] - LU[row * P[i] + j] * x[j];
    }

    // Backward substitution with pivoting
    for (int16_t i = row - 1; i >= 0; --i) 
    {
        for (int16_t j = i + 1; j < row; ++j)
            x[i] = x[i] - LU[row * P[i] + j] * x[j];

        x[i] = x[i] / LU[row * P[i] + i];
    }
}

// move to linsolve
err_status_t
matf32_inv(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (!matf32_is_same_size(p_src, p_dst))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    // Get number of rows
    uint16_t row = p_src->num_rows;

    // Check if the input matrix is square 
    if (p_src->num_cols != row)
        return MATH_SIZE_MISMATCH;

    // Define and reset temporary vectors and matrices
    matf32_t* lu = &m1;
    matf32_t* invmat = &m2;
    float* tmpvec = v1;
    uint16_t* p = p1;

    matf32_init(lu, row, row, m1data);
    matf32_init(invmat, row, row, m2data);
    memset(tmpvec, 0, row * sizeof(float));

    // Check if the determinant is 0
    if (matf32_lup(p_src, lu, p) == MATH_SINGULAR)
    {
        return MATH_SINGULAR; // matrix is singular
    }

    // Create the inverse
    for (uint16_t i = 0; i < row; ++i)
    {
        tmpvec[i] = 1.0;
        solve(p_src->p_data, &invmat->p_data[row * i], tmpvec, p, lu->p_data, row);
        tmpvec[i] = 0.0;
    }

    // Transpose of temp A^-1
    matf32_trans(invmat, invmat);

    // Copy data from temp to A^-1 (this allows to overwrite the input matrix for the inverse)
    memcpy(p_dst->p_data, invmat->p_data, row * row * sizeof(float));

    return MATH_SUCCESS;
}


err_status_t
matf32_dot(const matf32_t* const p_srca, const matf32_t* const p_srcb, float* const p_dst)
{
#ifdef MATH_MATRIX_CHECK
    if (p_srca->num_cols*p_srca->num_rows != p_srcb->num_cols*p_srcb->num_rows)
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    *p_dst = dot(p_srca->p_data, p_srcb->p_data, p_srca->num_cols*p_srca->num_rows);
}


void
matf32_vecposmul(const matf32_t* const p_srcm, float* const p_srcv, float* const p_dst)
{
    float* res = v1;
    zeros(res, p_srcm->num_rows, 1);

    for (uint16_t i = 0; i < p_srcm->num_rows; i++)
    {
        for (uint16_t j = 0; j < p_srcm->num_cols; j++)
        {
            res[i] += p_srcm->p_data[i*p_srcm->num_cols + j] * p_srcv[j];
        }
    }

    memcpy(p_dst, res, p_srcm->num_rows * sizeof(float));
}


void
matf32_vecpremul(const matf32_t* const p_srcm, float* const p_srcv, float* const p_dst)
{
    float* tmpvec;
    float* res = v1;
    zeros(res, 1, p_srcm->num_cols);

    for (uint16_t i = 0; i < p_srcm->num_cols; ++i)
    {
        for (uint16_t j = 0; j < p_srcm->num_rows; ++j)
        {
            res[i] += p_srcm->p_data[j*p_srcm->num_cols + i] * p_srcv[j];
        }
    }

    memcpy(p_dst, res, p_srcm->num_cols * sizeof(float));
}

void
matf32_vecmul_col_row(const float* const col_vec, const float* const row_vec, matf32_t* const p_dst)
{
    for (uint16_t i = 0; i < p_dst->num_rows; ++i)
    {
        for (uint16_t j = 0; j < p_dst->num_cols; ++j)
        {
            // revisar todos los iteradores, i*num_rows + j es incorrecto
            p_dst->p_data[i*p_dst->num_cols + j] = col_vec[i] * row_vec[j];
        }
    }
}


err_status_t
matf32_arr_add(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst)
{
    if (length < 3)
    {
        return MATH_ARGUMENT_ERROR;
    }

    matf32_t* tmpmat = &m1;
    matf32_init(tmpmat, p_matarray[0]->num_rows, p_matarray[0]->num_cols, m1data);
    matf32_zeros(tmpmat);

    for (uint16_t i = 0; i < length; i++)
    {
#ifdef MATH_MATRIX_CHECK 
        if (matf32_add(tmpmat, p_matarray[i], tmpmat) == MATH_SIZE_MISMATCH)
        {
            return MATH_SIZE_MISMATCH;
        }
#else
        matf32_add(tmpmat, p_matarray[i], tmpmat);
#endif
    }
    matf32_copy(tmpmat, p_dst);
    return MATH_SUCCESS;
}


err_status_t
matf32_arr_sub(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst)
{
    if (length < 3)
    {
        return MATH_ARGUMENT_ERROR;
    }

    matf32_t* tmpmat = &m1;
    matf32_init(tmpmat, p_matarray[0]->num_rows, p_matarray[0]->num_cols, m1data);
    matf32_zeros(tmpmat);

#ifdef MATH_MATRIX_CHECK 
    if (matf32_sub(p_matarray[0], p_matarray[1], tmpmat) == MATH_SIZE_MISMATCH)
    {
        return MATH_SIZE_MISMATCH;
    }
#else
    matf32_sub(p_matarray[0], p_matarray[1], tmpmat)
#endif

    for (uint16_t i = 2; i < length; i++)
    {
#ifdef MATH_MATRIX_CHECK 
        if (matf32_sub(tmpmat, p_matarray[i], tmpmat) == MATH_SIZE_MISMATCH)
        {
            return MATH_SIZE_MISMATCH;
        }
#else
        matf32_sub(tmpmat, p_matarray[i], tmpmat);
#endif
    }
    matf32_copy(tmpmat, p_dst);
    return MATH_SUCCESS;
}


err_status_t
matf32_arr_mul(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst)
{
    if (length < 3)
    {
        return MATH_ARGUMENT_ERROR;
    }

    matf32_t* tmpmat1 = &m1;
    matf32_t* tmpmat2 = &m2;

    matf32_init(tmpmat1, p_matarray[0]->num_rows, p_matarray[1]->num_cols, m1data);
    matf32_init(tmpmat2, p_matarray[0]->num_rows, p_matarray[1]->num_cols, m2data);

#ifdef MATH_MATRIX_CHECK
    if (matf32_mul(p_matarray[0], p_matarray[1], tmpmat1) == MATH_SIZE_MISMATCH)
    {
        return MATH_SIZE_MISMATCH;
    }
#else 
    matf32_mul(p_matarray[0], p_matarray[1], tmpmat1);
#endif

    for (uint16_t i = 2; i < length; i++)
    {
        matf32_reshape(tmpmat2, tmpmat1->num_rows, p_matarray[i]->num_cols);
#ifdef MATH_MATRIX_CHECK 
        if (matf32_mul(tmpmat1, p_matarray[i], tmpmat2) == MATH_SIZE_MISMATCH)
        {
            return MATH_SIZE_MISMATCH;
        }
#else
        matf32_mul(tmpmat1, p_matarray[i], tmpmat2);
#endif
        matf32_reshape(tmpmat1, tmpmat2->num_rows, tmpmat2->num_cols);
        matf32_copy(tmpmat2, tmpmat1);
    }

    matf32_copy(tmpmat2, p_dst);
    return MATH_SUCCESS;
}