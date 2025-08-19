/**
 * @file matf32.h
 * @author Andrea Pineda
 * @date Created 2 Aug 2025
 * 
 * Last Modified: 18 Aug 2025
 *      By: Andrea Pineda
 *
 */

#include "matf32.h"

// ====================================================================================================
// 1. Base utility functions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 1.1. Float-based operations
// ----------------------------------------------------------------------------------------------------

static float 
generate_gauss(float mu, float sigma) 
{
    float U1, U2, W, scalar;
    static float X1, X2;
    static int call = 0;

    if (call == 1) 
    {
        call = !call;
        return (mu + sigma * (float)X2);
    }

    // Compute the uniform norm
    do 
    {
        U1 = -1 + ((float)rand() / RAND_MAX) * 2;
        U2 = -1 + ((float)rand() / RAND_MAX) * 2;
        W = pow(U1, 2) + pow(U2, 2);
    } while (W >= 1 || W == 0);

    scalar = sqrt((-2 * log(W)) / W);
    X1 = U1 * scalar;
    X2 = U2 * scalar;

    call = !call;

    return (mu + sigma * (float)X1);
}


float
dot(float* p_srca, float* p_srcb, uint16_t length)
{
    float sum = 0;  // Reset;

    // Multiply each row
    for (int i = 0; i < length; ++i)
        sum += (*(p_srca++)) * (*(p_srcb++));
    return sum;
}


void
eye(float* p_dst, uint16_t row, uint16_t column)
{
    // Reset first
    memset(p_dst, 0, row * column * sizeof(float));

    for (int i = 0; i < row; i++)
    {
        *p_dst = 1.0;
        p_dst += row + 1;
    }
}


void
diag(float* p_src, float* p_dst, int row_d, int column_d)
{
    // Reset the matrix array
    memset(p_dst, 0, row_d * column_d * sizeof(float));

    for (int i = 0; i < row_d; i++) {
        for (int j = 0; j < column_d; j++) {
            if (j == i) {
                *p_dst = p_src[i];
                p_dst += column_d + 1;
            }
        }
    }
}


void
zeros(float* p_dst, int row, int column)
{
    memset(p_dst, 0, row * column * sizeof(float));
}


void
ones(float* p_dst, int row, int column)
{
    memset(p_dst, 1, row * column * sizeof(float));
}


void
randn(float* p_dst, uint16_t length, float mu, float sigma)
{
    srand(time(NULL));
    for (uint16_t i = 0; i < length; i++)
        p_dst[i] = generate_gauss(mu, sigma);
}


float
norm(float* p_src, int row, int column)
{
    uint16_t size = row*column;

    float sum = 0;

    for (uint16_t i = 0; i < size; ++i)
    {
        sum += p_src[i]*p_src[i];
    }

    return sqrtf(sum);
}


void
scale(float* p_src, uint16_t length, float scalar, float* p_dst)
{
    for (int i = 0; i < length; ++i)
    {
        p_dst[i] = p_src[i]*scalar;
    }
}


// ----------------------------------------------------------------------------------------------------
// 1.2. Miscellaneous functions
// ----------------------------------------------------------------------------------------------------

void
copy(float* p_src, float* p_dst, int row, int column)
{
    memcpy(p_dst, p_src, column * row * sizeof(float));
}


void 
print(float* p_src, uint16_t row, uint16_t column)
{
    for (uint16_t i = 0; i < row; i++) 
    {
        for (uint16_t j = 0; j < column; j++)
            printf("%0.18f\t", *(p_src++));
        printf("\n");
    }
    printf("\n");
}


float 
saturation(float input, float lower_limit, float upper_limit) 
{
    if (input > upper_limit)
        return upper_limit;
    else if (input < lower_limit)
        return lower_limit;
    else 
        return input; // No action
}


float 
sign(float number) 
{
    if (number > 0) 
        return 1;
    else if (number < 0) 
        return -1;
    else 
        return 0;
}


float
mean(float* p_src, uint16_t length)
{
    float s = 0;

    for (uint16_t i = 0; i < length; i++)
        s += p_src[i];
    return s / ((float)length);
}


float
std_dev(float* p_src, uint16_t length)
{
    float mu = mean(p_src, length);
    float sigma = 0;
    
    for (uint16_t i = 0; i < length; i++)
        sigma += (p_src[i] - mu) * (p_src[i] - mu);
    return sqrtf(sigma / ((float)length));
}


bool
is_equal(float* p_a, float* p_b, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        // skip if both are NaN
        if (isnan(p_a[i]) && isnan(p_b[i]))
        {
            continue;
        }

        if (!is_equal_margin(p_a[i], p_b[i]))
        {
            return false;
        }
    }

    return true;
}


void
zero_patch(float* p_a, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        // skip if both are NaN
        if (isnan(p_a[i]) || isinf(p_a[i]))
        {
            p_a[i] = 0.0;
        }
    }
}



// ====================================================================================================
// 2. matf32: Matrix definitions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 2.2. Utility functions for the matf32 structs
// ----------------------------------------------------------------------------------------------------

void
matf32_init(matf32_t* const instance, uint16_t num_rows, uint16_t num_cols, float* p_data)
{
    instance->num_rows = num_rows;
    instance->num_cols = num_cols;
    instance->p_data = p_data;
}


void
matf32_print(const matf32_t* p_src)
{
    float* p_data_src = p_src->p_data;

    for (uint16_t i = 0; i < p_src->num_rows; i++)
    {
        for (uint16_t j = 0; j < p_src->num_cols; j++)
        {
            printf("%0.18f\t", *(p_data_src++));
        }
        printf("\n");
    }
    printf("\n");
}


void
err_status_print(err_status_t err)
{
    switch(err)
    {
        case MATH_SUCCESS:
            printf("MATH_SUCCESS\n");
            break;
            
        case MATH_ARGUMENT_ERROR:
            printf("MATH_ARGUMENT_ERROR\n");
            break;
            
        case MATH_LENGTH_ERROR:
            printf("MATH_LENGTH_ERROR\n");
            break;
            
        case MATH_SIZE_MISMATCH:
            printf("MATH_SIZE_MISMATCH\n");
            break;
            
        case MATH_NANINF:
            printf("MATH_NANINF\n");
            break;
            
        case MATH_SINGULAR:
            printf("MATH_SINGULAR\n");
            break;
            
        case MATH_TEST_FAILURE:
            printf("MATH_TEST_FAILURE\n");
            break;
            
        case MATH_DECOMPOSITION_FAILURE:
            printf("MATH_DECOMPOSITION_FAILURE\n");
            break;
            
    }
}

void matf32_cond(matf32_t* const p_src, float* p_cond)
{
    float temp_data[MAX_MAT_SIZE];
    matf32_t temp;

    matf32_init(&temp, p_src->num_rows, p_src->num_cols, temp_data);

    matf32_inv(p_src, &temp);

    *p_cond = norm(p_src->p_data, p_src->num_rows, p_src->num_cols) * norm(temp.p_data, temp.num_rows, temp.num_cols);
}

// ----------------------------------------------------------------------------------------------------
// 2.3. Submatrix operations for the matf32 structs
// ----------------------------------------------------------------------------------------------------

err_status_t
matf32_submatrix_copy(const matf32_t* const p_src, matf32_t* const p_dst,
                      const uint16_t src_row, const uint16_t src_col,
                      const uint16_t dst_row, const uint16_t dst_col,
                      const uint16_t rows,    const uint16_t cols)
{
#ifdef MATH_MATRIX_CHECK
    if (    (p_src->num_rows < (src_row+rows) )
         || (p_src->num_cols < (src_col+cols) )
         || (p_dst->num_rows < (dst_row+rows) )
         || (p_dst->num_cols < (dst_col+cols) ))
    {
        return MATH_SIZE_MISMATCH;
    }
#endif

    // A(i,j)
    for (uint16_t i = 0; i < rows; ++i)
    {
        for (uint16_t j = 0; j < cols; ++j)
        {
            p_dst->p_data[(dst_row+i)*p_dst->num_cols + (dst_col+j)] = p_src->p_data[(src_row+i)*p_src->num_cols + (src_col+j)];
        }
    }

    return MATH_SUCCESS;
}

void 
matf32_set_row(matf32_t* const p_dst, uint16_t row, float val)
{
    float* p_dst_data = p_dst->p_data;

    for (int i = 0; i < p_dst->num_cols; ++i)
    {
        p_dst_data[p_dst->num_cols*row + i] = val;
    }
}

void 
matf32_set_col(matf32_t* const p_dst, uint16_t col, float val)
{
    float* p_dst_data = p_dst->p_data;

    for (int i = 0; i < p_dst->num_rows; ++i)
    {
        p_dst_data[p_dst->num_cols*i + col] = val;
    }
}


// ----------------------------------------------------------------------------------------------------
// 2.4. Special matrix initializations
// ----------------------------------------------------------------------------------------------------

void
matf32_eye(matf32_t* const p_dst)
{
    eye(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_diag(float* p_src, matf32_t* const p_dst)
{
    diag(p_src, p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_zeros(matf32_t* const p_dst)
{
    zeros(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_ones(matf32_t* const p_dst)
{
    ones(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_randn(matf32_t* const p_dst, float mu, float sigma)
{
    randn(p_dst->p_data, p_dst->num_rows * p_dst->num_cols, mu, sigma);
}



// ====================================================================================================
// 3. Check functions for matf32 structs
// ====================================================================================================

bool
matf32_check_triangular_upper(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 1; i < p_mat->num_rows; ++i)
    {
        for (uint16_t j = 0; j < i; ++j)
        {
            if (!is_equal_margin(0, p_data_src[i*p_mat->num_rows + j]))
            {
                return false;
            }
        }
    }

    return true;
}

bool
matf32_check_triangular_lower(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 0; i < p_mat->num_rows-1; ++i)
    {
        for (uint16_t j = p_mat->num_cols-1; j > i; --j)
        {
            if (!is_equal_margin(0, p_data_src[i*p_mat->num_rows + j]))
            {
                return false;
            }
        }
    }

    return true;
}

bool
matf32_is_equal(const matf32_t* const p_mat_a, const matf32_t* const p_mat_b)
{
#ifdef MATH_MATRIX_CHECK
    if ((p_mat_a->num_rows != p_mat_b->num_rows) || (p_mat_a->num_cols != p_mat_b->num_cols))
    {
        return false;
    }
#endif
    return is_equal(p_mat_a->p_data, p_mat_b->p_data, p_mat_a->num_rows * p_mat_a->num_cols);
}

bool
matf32_is_equal_scalar(const matf32_t* const p_mat, float scalar)
{
    float* p_mat_data = p_mat->p_data;

    uint16_t size = p_mat->num_cols * p_mat->num_rows;

    for (uint16_t i = 0; i < size; ++i)
    {

        if (!is_equal_margin(p_mat_data[i], scalar))
        {
            return false;
        }
    }

    return true;
}


bool
matf32_is_equal_less_scalar(const matf32_t* const p_mat, float scalar)
{
    float* p_mat_data = p_mat->p_data;

    uint16_t size = p_mat->num_cols * p_mat->num_rows;

    for (uint16_t i = 0; i < size; ++i)
    {

        if (!is_equal_margin(p_mat_data[i], scalar) && (p_mat_data[i] > scalar))
        {
            return false;
        }
    }

    return true;

}

bool
matf32_check_symmetric(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (!matf32_check_square_matrix(p_mat))
    {
        return false;
    }
#endif

    float* p_data_source = p_mat->p_data;

    uint16_t size = p_mat->num_cols;

    for (uint16_t i = 0; i < size-1; ++i)
    {
        for (int j = 1; j < size; ++j)
        {
            // skip diagonal
            if (i == j)
            {
                continue;
            }

            // compare within presicion
            if (!is_equal_margin(p_data_source[i*size + j], p_data_source[j*size + i]))
            {
                return false;
            }
        }
    }

    return true;
}


bool
matf32_check_hessenberg_upper(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 2; i < p_mat->num_rows; ++i)
    {
        for (uint16_t j = 0; j < i-1; ++j)
        {
            if (!is_equal_margin(0, p_data_src[i*p_mat->num_rows + j]))
            {
                return false;
            }
        }
    }

    return true;
}


bool
matf32_check_hessenberg_lower(const matf32_t* const p_mat)
{
#ifdef MATH_MATRIX_CHECK
    if (p_mat->num_rows != p_mat->num_cols)
    {
        return false;
    }
#endif

    float* p_data_src = p_mat->p_data;

    for (uint16_t i = 0; i < p_mat->num_rows-2; ++i)
    {
        for (uint16_t j = p_mat->num_cols-1; j > i+1; --j)
        {
            if (!is_equal_margin(0, p_data_src[i*p_mat->num_rows + j]))
            {
                return false;
            }
        }
    }

    return true;
}


bool
matf32_check_symposdef(const matf32_t* const p_mat)
{
    err_status_t status;

    float temp_data[MAX_MAT_SIZE];
    matf32_t temp;

    matf32_init(&temp, p_mat->num_rows, p_mat->num_cols, temp_data);
    matf32_zeros(&temp);
    
    if (!matf32_check_symmetric(p_mat))
    {
        return false;
    }
    
    status = matf32_cholesky(p_mat, &temp);

    if (MATH_DECOMPOSITION_FAILURE == status)
    {
        return false;
    }

    return true;
}

// ====================================================================================================
// 4. Matrix operations based on matf32 structs
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Matrix and vector operations
// ----------------------------------------------------------------------------------------------------

// Private variables
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


// ----------------------------------------------------------------------------------------------------
// 4.2. Array of matrices operations
// ----------------------------------------------------------------------------------------------------

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

// Tested => Works
err_status_t
matf32_pinv(const matf32_t* const p_a, matf32_t* const p_pinv)
{
    float trans_a_data[MAX_MAT_SIZE];
    matf32_t trans_a;
    matf32_init(&trans_a, p_a->num_rows, p_a->num_cols, trans_a_data);

    float temp_data[MAX_MAT_SIZE];
    matf32_t temp;
    matf32_init(&temp, p_a->num_rows, p_a->num_cols, temp_data);

    // A'
    matf32_trans(p_a, &trans_a);
    //printf("A':\n");
    //matf32_print(&trans_a);

    // A'A
    matf32_mul(&trans_a, p_a, &temp);
    //printf("A'A:\n");
    //matf32_print(&temp);

    // (A'A)^-1
    matf32_inv(&temp, &temp);
    //printf("(A'A)^-1:\n)");
    //matf32_print(&temp);

    // (A'A)^-1 * A'
    matf32_mul(&temp, &trans_a, p_pinv); 
    //printf("(A'A)^-1 * A':\n");
    //matf32_print(p_pinv);
}

// ----------------------------------------------------------------------------------------------------
// 4.3. Matrix factorization/decomposition methods
// ----------------------------------------------------------------------------------------------------

// https://www.cs.cornell.edu/~bindel/class/cs6210-f09/lec18.pdf
// update to use matf32 vectors instead of array of floats
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

        // ------------------------------------------------------------
        // Calculate R
        // ------------------------------------------------------------

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

        // ------------------------------------------------------------
        // Calculate Q
        // ------------------------------------------------------------

        // Q(:,i:end)
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
                //p_u_data[i*rows + i] = 1;
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


// https://www.math.umd.edu/~petersd/401/cholesk.pdf
// revise later ----> revised
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

    err_status_t status;

    float* p_data_a = p_a->p_data;
    float* p_data_c = p_c->p_data;

    uint16_t size = p_a->num_cols;

    float temp_v[size];

    float sum = 0;
    float dot = 0;

    for (uint16_t i = 0; i < p_a->num_rows; ++i)
    {
        for (uint16_t j = 0; j < p_a->num_cols; ++j)
        {
            if (i == j)
            {
                // sum = A(j,j) - v'*v
                // Executes starting from i = 1
                sum = p_data_a[i*size + i];
                for (int16_t k = 0; k < i; ++k)
                {
                    sum -= p_data_c[k*size + i] * p_data_c[k*size + i];
                }

                if (sum <= 0.0)
                {
                    return MATH_DECOMPOSITION_FAILURE;
                }

                p_data_c[i*size + i] = sqrtf(sum);
            }
            else if (i < j)
            {
                dot = 0;

                // Added the sum that was lacking from the referenced pdf in the comments above
                for (int16_t k = 0; k < i; ++k)
                {
                    dot += p_data_c[k*size + i] * p_data_c[k*size + j];
                }

                p_data_c[i*size + j] = (p_data_a[i*size + j] - dot)/p_data_c[i*size + i];

                // If you need the cholesky decomposition in square form, uncomment this and comment the next line
                // In that case, modify the linsolve_cholesky_matf32 function to extract the upper triangular
                //p_data_c[j*size + i] = p_data_c[i*size + j];

                // Turn lower triangular values to zero to ensure it's upper triangular
                p_data_c[j*size + i] = 0;
            }
        }
    }

    zero_patch(p_data_c, size*size);

    return MATH_SUCCESS;
}



