/**
 * @file matf32.h
 *
 * Single header to include all matrix related functions.
 * 
 * Last modified: 31 Aug 2025
 *          By: Andrea Pineda
 */

#ifndef ROBOTAT_MATF32_H_
#define ROBOTAT_MATF32_H_

#include <string.h>                     // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdlib.h>                     // Standard library.
#include <stdint.h>                     // For uint8_t, uint16_t and uint16_t.
#include <math.h>                       // For sqrtf.
#include <float.h>                      // Required for FLT_EPSILON.
#include <stdbool.h>                    // For bool datatype.
#include <time.h>                       // For srand, clock.

#include "constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// 1. Base utility functions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 1.1. Float-based operations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Find the dot product of two vectors, pointed by p_srca and p_srcb, of the same size.
 *
 * @param[in]   p_srca  Points to vector 1.
 * @param[in]   p_srcb  Points to vector 2.
 * @param[in]   length  Length of vectors.
 *
 * @return Dot product between the vectors.
 */
float
dot(float* p_srca, float* p_srcb, uint16_t length);


/**
 * @brief   Create an identity matrix array with size row x column.
 *
 * @param[in, out]  p_dst   Points to array to allocate the identity matrix.
 * @param[in]       row     Number of required rows.
 * @param[in]       column  Number of required columns.
 *
 * @return None.
 */
void
eye(float* p_dst, uint16_t row, uint16_t column);


/**
 * @brief   Creates a diagonal matrix array pointed by p_dst with the size row x column,
 * from a vector pointed by p_src. Notice that the row of vector x need to be the same
 * length as the column of the matrix.
 *
 * @param[in]       p_src       Points to vector with diagonal entries.
 * @param[in,out]   p_dst       Points to array to allocate the diagonal matrix.
 * @param[in]       row_d       Number of required rows.
 * @param[in]       column_d    Number of required columns.
 *
 * @return None.
 */
void
diag(float* p_src, float* p_dst, int row_d, int column_d);


/**
 * @brief   Turn all elements of the matrix array pointed by p_dst, size row x column, into 0.
 *
 * @param[in,out]   p_dst   Points to zero matrix array.
 * @param[in]       row     Number or rows.
 * @param[in]       column  Number of columns.
 *
 * @return None.
 */
void
zeros(float* p_dst, int row, int column);


/**
 * @brief   Turn all elements of the matrix array pointed by p_dst, size row x column, into 1.
 *
 * @param[in,out]   p_dst   Points to ones matrix array.
 * @param[in]       row     Number or rows.
 * @param[in]       column  Number of columns.
 *
 * @return None.
 */
void
ones(float* p_dst, int row, int column);


/**
 * @brief   Creates a random array with values sampled from a normal (Gaussian) distribution.
 *
 * @param[in, out]  p_dst   Points to random vector to create.
 * @param[in]       length  Vector length.
 * @param[in]       mu      Mean.
 * @param[in]       sigma   Standar deviation.
 *
 * @return  None.
 */
void
randn(float* p_dst, uint16_t length, float mu, float sigma);


/**
 * @brief   Calculates the norm of a given matrix (euclidean for vectors, frobenius for matrices).
 *
 * @param[in,out]   p_src   Points to matrix array.
 * @param[in]       row     Number or rows.
 * @param[in]       column  Number of columns.
 *
 * @return Norm of matrix or vector.
 */
float
norm(float* p_src, int row, int column);


/**
 * @brief   Multiplies a vector by a scalar.
 *
 * @param[in]    p_src   Vector to scale.
 * @param[in]    length  Vector size.
 * @param[in]    scalar     Factor to scale the vector.
 * @param[out]   p_dst   Scaled vector.
 *
 */
void
scale(float* p_src, uint16_t length, float scalar, float* p_dst);


// ----------------------------------------------------------------------------------------------------
// 1.2. Miscellaneous functions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Size-aware matrix copy.
 *
 * @param[in]       p_src   Points to matrix to copy from.
 * @param[in, out]  p_dst   Points to matrix to copy to.
 * @param[in]       row     Amount of rows.
 * @param[in]       column  Amount of columns.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
void
copy(float* p_src, float* p_dst, int row, int column);


/**
 * @brief   Prints array to console (formatted).
 *
 * @param[in]   p_src   Points to array to print.
 * @param[in]   row     Number of rows.
 * @param[in]   column  Number of columns.
 *
 * @return  None.
 */
void
print(float* p_src, uint16_t row, uint16_t column);


/**
 * @brief   Saturates the input, given upper and lower limits.
 * 
 * @param[in]   input           Input value.
 * @param[in]   lower_limit     Lower saturation threshold.
 * @param[in]   upper_limit     Upper saturation threshold.
 * 
 * @return  Saturated output.
 */
float
saturation(float input, float lower_limit, float upper_limit);


/**
 * @brief   Gets the input's sign.
 *
 * @param[in]   number      Input value.
 * 
 * @return  Sign of input.
 */
float
sign(float number);


/**
 * @brief   Gets the mean of a given array.
 *
 * @param[in]   p_src   Points to input array.
 * @param[in]   length  Array length.
 *
 * @return  Mean of array.
 */
float
mean(float* p_src, uint16_t length);


/**
 * @brief   Gets the standard deviation of a given array.
 * Named this way instead of "std" to avoid conflict with std namesapce in c++
 *
 * @param[in]   p_src   Points to input array.
 * @param[in]   length  Array length.
 *
 * @return  Standard deviation of array.
 */
float
std_dev(float* p_src, uint16_t length);


/**
 * @brief   Compares two arrays.
 *
 * @param[in]   p_a     Points to first array.
 * @param[in]   p_b     Points to second array.
 * @param[in]   length  Arrays length.
 *
 * @return  If arrays values are equal.
 */
bool
is_equal(float* p_a, float* p_b, uint16_t length);


/**
 * @brief   Compares two floats with a given precision.
 *
 * @param[in]   a   First value.
 * @param[in]   b   Second vaue.
 *
 * @return  If values are equal within precision.
 */
// static keyword fixes unexplained error of undefined ref when incuding constants.h in math_util.h
static inline bool
is_equal_margin(float a, float b)
{
    return (fabsf(a-b) <= MATH_EQUAL_PRECISION);
}


/**
 * @brief   Checks if input is greater or lesser than given bound.
 * 
 * @param[in]   num     Value to check
 * 
 * @return Returns FLT_MAX or -FLT_MAX depending on input value.
 */
static inline float
inf_bound(float num)
{
    return (num > FLT_MAX? FLT_MAX : (num < -FLT_MAX? -FLT_MAX: num));
}


/**
 * @brief   Changes NaN and Inf alues to zero.
 * Needed to avoid errors in cases where irrelevant elements end set as NaN or Inf.
 *
 * @param[in]   p_a     Points to array.
 * @param[in]   length  Array length.
 *
 */
void
zero_patch(float* p_a, uint16_t length);



// ====================================================================================================
// 2. matf32: Matrix definitions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 2.1. Structs
// ----------------------------------------------------------------------------------------------------

/**
 * @brief Floating point matrix data structure.
 * 
 * Used for readability and compatibility with ARM's CMSIS-DSP library.
 */
typedef struct
{ 
    uint16_t num_rows;  /**< Number of rows of the matrix. */
    uint16_t num_cols;  /**< Number of columns of the matrix. */
    float* p_data;      /**< Points to the data of the matrix. */
} matf32_t;


/**
 * @brief Error status from matrix operations.
 * 
 * Defined this way for compatibility with ARM's CMSIS-DSP library.
 */
typedef enum
{
    MATH_SUCCESS,
    MATH_ARGUMENT_ERROR,
    MATH_LENGTH_ERROR,
    MATH_SIZE_MISMATCH,
    MATH_NANINF,
    MATH_SINGULAR,
    MATH_TEST_FAILURE,
    MATH_DECOMPOSITION_FAILURE
} err_status_t;


// ----------------------------------------------------------------------------------------------------
// 2.2. Utility functions for the matf32 structs
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Constructor for the floating point matrix data structure.
 * 
 * @param[in, out]  instance    Points to an instance of the floating-point matrix structure.
 * @param[in]       num_rows    Number of rows in the matrix.
 * @param[in]       num_cols    Number of columns in the matrix.
 * @param[in]       p_data      Points to the matrix data array.
 * 
 * @return  None
 */
void
matf32_init(matf32_t* const instance, uint16_t num_rows, uint16_t num_cols, float* p_data);


/**
 * @brief   Prints matrix data to console (formatted).
 *
 * @param[in]   p_src   Points to input matrix.
 *
 * @return  None.
 */
void
matf32_print(const matf32_t* p_src);


/**
 * @brief   Prints error status to console.
 *
 * @param[in]   err   Error stats value to print.
 *
 * @return  None.
 */
void
err_status_print(err_status_t err);

/**
 * @brief   Calculates conditioning number of a matrix
 * 
 * @param[in]   p_src   Input matrix
 * @param[in]   p_cond  Points to float to store conditioning number
 * 
 * @return  None
 */
void 
matf32_cond(matf32_t* const p_src, float* p_cond);

/**
 * @brief   Gets an specific element from a matrix.
 * 
 * WARNING: this routine uses mathematical indexing, which means that the first element of the matrix has
 * index (1,1), i.e. matf32_get(A, i, j, &element) => element = A->p_data[(i-1)*A->num_cols + (j-1)].
 * 
 * @param[in]       p_src   Points to matrix.
 * @param[in]       row     Row of element.
 * @param[in]       col     Column of element.
 * @param[in, out]  dst     Points to variable to store element.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_get(const matf32_t* p_src, uint16_t row, uint16_t col, float* dst)
{
#ifdef MATH_MATRIX_CHECK 
    if ((row <= p_src->num_rows) && (col <= p_src->num_cols) && (row > 0) && (col > 0));
    else return MATH_SIZE_MISMATCH;
#endif
    *dst = p_src->p_data[(--row)*p_src->num_cols + (--col)];
    return MATH_SUCCESS;
}


/**
 * @brief   Sets an specific element in a matrix.
 * 
 * WARNING: this routine uses mathematical indexing, which means that the first element of the matrix has
 * index (1,1), i.e. matf32_set(A, i, j, value) => A->p_data[(i-1)*A->num_cols + (j-1)] = value.
 *
 * @param[in, out]  p_src   Points to matrix.
 * @param[in]       row     Row of element.
 * @param[in]       col     Column of element.
 * @param[in]       value     Value of element to set.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_set(matf32_t* const p_src, uint16_t row, uint16_t col, float value)
{
#ifdef MATH_MATRIX_CHECK 
    if ((row <= p_src->num_rows) && (col <= p_src->num_cols) && (row > 0) && (col > 0));
    else return MATH_SIZE_MISMATCH;
#endif
    p_src->p_data[(--row) * p_src->num_cols + (--col)] = value;
    return MATH_SUCCESS;
}

/**
 * Checks whether or not two matrices have the same size.
 *
 * @param[in]   p_srca  Points to first input matrix.
 * @param[in]   p_srcb  Points to second input matrix.
 *
 * @return  true if matrices have the same size, false otherwise.
 */
static inline bool
matf32_is_same_size(const matf32_t* p_srca, const matf32_t* p_srcb)
{
    return ((p_srca->num_rows == p_srcb->num_rows) && (p_srca->num_cols == p_srcb->num_cols));
}


/**
 * Checks if a matrix has a specified size.
 *
 * @param[in]   p_src   Points to input matrix.
 * @param[in]   rows    Amount of rows.
 * @param[in]   cols    Amount of cols.
 *
 * @return  true if the matrix has the specified size, false otherwise.
 */
static inline bool
matf32_size_check(const matf32_t* p_src, uint16_t rows, uint16_t cols)
{
    return ((p_src->num_rows == rows) && (p_src->num_cols == cols));
}


/**
 * @brief   Size-aware matrix copy.
 *
 * @param[in]       p_src   Points to matrix to copy from.
 * @param[in, out]  p_dst   Points to matrix to copy to.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_copy(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (matf32_is_same_size(p_src, p_dst));
    else return MATH_SIZE_MISMATCH;
#endif
    memcpy(p_dst->p_data, p_src->p_data, p_src->num_rows * p_src->num_cols * sizeof(float));
    return MATH_SUCCESS;
}


/**
 * @brief   Changes the shape of a given matrix structure. 
 * 
 * WARNING: this routine is unable to check if the data array is large enough to allow a reshape, 
 * as matrices are statically allocated. Can only be used safely if the matrix was originally 
 * defined to have the max number of rows and columns. Use matf32_reshape_safe if you want to
 * automatically verify if the reshape is possible given the input matrix dimensions.
 *
 * @param[in, out]  p_src       Points to matrix to reshape.
 * @param[in]       new_rows    New number of rows.
 * @param[in]       new_cols    New number of columns.
 *
 * @return  None.
 */
static inline void
matf32_reshape(matf32_t* const p_src, uint16_t new_rows, uint16_t new_cols)
{
    p_src->num_rows = new_rows;
    p_src->num_cols = new_cols;
}


/**
 * @brief   Changes the shape of a given matrix structure if possible, given the input matrix dimensions.
 *
 * WARNING: this routine is unable to reshape a matrix to a bigger size. Use matf32_reshape if you want
 * to do this.
 *
 * @param[in, out]  p_src       Points to matrix to reshape.
 * @param[in]       new_rows    New number of rows.
 * @param[in]       new_cols    New number of columns.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_reshape_safe(matf32_t* const p_src, uint16_t new_rows, uint16_t new_cols)
{
    if ((new_rows * new_cols) <= (p_src->num_rows * p_src->num_cols))
    {
        p_src->num_rows = new_rows;
        p_src->num_cols = new_cols;
        return MATH_SUCCESS;
    }
    else
        return MATH_SIZE_MISMATCH;
}

// ----------------------------------------------------------------------------------------------------
// 2.3. Submatrix operations for the matf32 structs
// ----------------------------------------------------------------------------------------------------

// hacer vesion indice progra y version indice matematico

err_status_t
matf32_submatrix_copy(const matf32_t* const p_src, matf32_t* const p_dst,
                      const uint16_t src_row, const uint16_t src_col,
                      const uint16_t dst_row, const uint16_t dst_col,
                      const uint16_t rows,    const uint16_t cols);


/**
 * @brief   Sets a matrix row to a value.
 *
 * @param[in, out]  p_dst   Points to matrix to edit.
 * @param[in]       row     Number of row to set as a value.
 * @param[in]       val     Value to set the row to.
 *
 * @return None.
 */
void 
matf32_set_row(matf32_t* const p_dst, uint16_t row, float val);


/**
 * @brief   Sets a matrix col to a value.
 *
 * @param[in, out]  p_dst   Points to matrix to edit.
 * @param[in]       col     Number of column to set as a value.
 * @param[in]       val     Value to set the row to.
 *
 * @return None.
 */
void 
matf32_set_col(matf32_t* const p_dst, uint16_t col, float val);


// ----------------------------------------------------------------------------------------------------
// 2.4. Special matrix initializations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Sets a matrix structure to the identity matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the identity matrix.
 *
 * @return None.
 */
void
matf32_eye(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a diagonal matrix, created from a given vector.
 *
 * @param[in]       p_src       Points to vector with diagonal entries.
 * @param[in,out]   p_dst       Points to matrix to allocate the diagonal matrix.
 *
 * @return None.
 */
void
matf32_diag(float* p_src, matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to the zero matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the zero matrix.
 *
 * @return None.
 */
void 
matf32_zeros(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a ones matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the ones matrix.
 *
 * @return None.
 */
void
matf32_ones(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a matrix with random entries, sampled from a normal
 * distribution.
 *
 * @param[in, out]  p_dst   Points to random matrix.
 * @param[in]       mu      Mean.
 * @param[in]       sigma   Standard deviation.
 *
 * @return  None.
 */
void
matf32_randn(matf32_t* const p_dst, float mu, float sigma);



// ====================================================================================================
// 3. Check functions for matf32 structs
// ====================================================================================================

/**
 * @brief Checks if matrix is square
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not square.
 */
// static keyword fixes explained error of undefined ref when incuding constants.h in math_util.h
static inline bool 
matf32_check_square_matrix(const matf32_t* const p_mat)
{
    return (p_mat->num_cols == p_mat->num_rows);
}

/**
 * @brief Checks if matrix is upper triangular
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not upper triangular.
 */
bool
matf32_check_triangular_upper(const matf32_t* const p_mat);

/**
 * @brief Checks if matrix is lower triangular
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not lower triangular.
 */
bool
matf32_check_triangular_lower(const matf32_t* const p_mat);


/**
 * @brief Checks if two matrices are equal
 *
 * @param[in]   p_mat_a Points to first matrix to compare.
 * @param[in]   p_mat_b Points to second matrix to compare.
 *
 * @return  Execution status
 *              true  :     Matrices are equal.
 *              false :     Matrices are not equal.
 */
bool
matf32_is_equal(const matf32_t* const p_mat_a, const matf32_t* const p_mat_b);


/**
 * @brief Checks if a matrix values are equal to a scalar
 *
 * @param[in]   p_mat  Points to the matrix to compare.
 * @param[in]   scalar Value to compare against.
 *
 * @return  Execution status
 *              true  :     Matrix values are equal to scalar.
 *              false :     Matrix values are not equal to scalar.
 */
bool
matf32_is_equal_scalar(const matf32_t* const p_mat, float scalar);


/**
 * @brief Checks if a matrix values are equal or lesser than a scalar
 *
 * @param[in]   p_mat  Points to the matrix to compare.
 * @param[in]   scalar Value to compare against.
 *
 * @return  Execution status
 *              true  :     Matrix values are equal or less than scalar.
 *              false :     Matrix values are not equal or less than scalar.
 */
bool
matf32_is_equal_less_scalar(const matf32_t* const p_mat, float scalar);

/**
 * @brief Checks if a matrix is symmetric
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrix is symmetrical.
 *              false :     Matrix is not symmetrical.
 */
bool
matf32_check_symmetric(const matf32_t* const p_mat);

/**
 * @brief Checks if a matrix is upper hessenberg.
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrices is upper hessenberg.
 *              false :     Matrices is not upper hessenberg.
 */
bool
matf32_check_hessenberg_upper(const matf32_t* const p_mat);

/**
 * @brief Checks if a matrix is lower hessenberg.
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrices is lower hessenberg.
 *              false :     Matrices is not lower hessenberg.
 */
bool
matf32_check_hessenberg_lower(const matf32_t* const p_mat);

/**
 * @brief Checks if a matrix is symmetric positive definite.
 * 
 * @param[in]   p_mat   Points to matrix to check.
 * 
 * @return Execution status
 *              true  :     Matrix is symmetric positive definite
 *              false :     Matrix is not symmetric positive definite (cannot perform cholesky)
 */
bool
matf32_check_symposdef(const matf32_t* const p_mat);

// ====================================================================================================
// 4. Matrix operations based on matf32 structs
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Matrix and vector operations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Calculates the frobenius norm of a matrix.
 *
 * @param[in]       p_src  Points input matrix.
 *
 * @return  Norm of te matrix.
 */
inline float
matf32_norm(const matf32_t* p_src)
{
    return norm(p_src->p_data, p_src->num_rows, p_src->num_cols);
}


/**
 * @brief   Adds two matrices. Both need to be of the same dimension.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_add(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Substracts two matrices. Both need to be of the same dimension.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_sub(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Multiplies a matrix by a scalar, element-wise.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in]       scalar  Scaling factor.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_scale(const matf32_t* p_src, float scalar, matf32_t* p_dst);


/**
 * @brief   Transposes a matrix.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_trans(const matf32_t* p_src, matf32_t* p_dst);


/**
 * @brief   Multiplies two matrices. The number of columns of the first matrix must be the same as
 * the number of rows of the second matrix. Output matrix cannot be the same as one of the inputs.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_mul(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Computes the LU decomposition (with partial pivoting) of a square matrix A, pointed by p_src,
 * such that PA = LU.
 *
 * @param[in]       p_src   Points to square matrix to decompose.
 * @param[in, out]  p_lu    Points to the result of the decomposition.
 * @param[in, out]  pivot   Points to the pivot vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_lup(const matf32_t* p_src, matf32_t* p_lu, uint16_t* pivot);


/**
 * @brief   Computes the inverse of a square, non-singular matrix.
 * 
 * NOTE: use only as a last resort, solving the linear system Ax = b should always be the first choice.
 * This routine is also very numerically sensitive, as the matrix are defined with 32-bit floating point
 * data.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_inv(const matf32_t* p_src, matf32_t* p_dst);


/**
 * @brief   Dot product between two vectors (wether row or column).
 *
 * @param[in]       p_srca  Points to first input vector.
 * @param[in]       p_srcb  Points to second input vector.
 * @param[in]       p_dst   Points to scalar result.
 *
 * @return  Execution status.
 */
err_status_t
matf32_dot(const matf32_t* const p_srca, const matf32_t* const p_srcb, float* const p_dst);


/**
 * @brief   Matrix-vector post multiplication, i.e. Ax. Assumes a column vector.
 *
 * @param[in]       p_srcm  Points to input matrix.
 * @param[in]       p_srcv  Points to input vector.
 * @param[in, out]  p_dst   Points to result vector.
 *
 * @return  None.
 */
void
matf32_vecposmul(const matf32_t* p_srcm, float* p_srcv, float* p_dst);


/**
 * @brief   vector-Matrix pre multiplication, i.e. xA. Assumes a row vector.
 *
 * @param[in]       p_srcm  Points to input matrix.
 * @param[in]       p_srcv  Points to input vector.
 * @param[in, out]  p_dst   Points to result vector.
 *
 * @return  None.
 */
void
matf32_vecpremul(const matf32_t* p_srcm, float* p_srcv, float* p_dst);

// vector lengths taken from matrix dimensions
void
matf32_vecmul_col_row(const float* const col_vec, const float* const row_vec, matf32_t* const p_dst);


// ----------------------------------------------------------------------------------------------------
// 4.2. Array of matrices operations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Adds an array of matrices sequentially. The size of all matrices must be the same.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_add(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst);


/**
 * @brief   Subtracts an array of matrices sequentially. The size of all matrices must be the same.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_sub(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst);


/**
 * @brief   Multiplies an array of matrices sequentially. The number of columns of any matrix must be the same as
 * the number of rows of the next. Output matrix cannot be the same as one of the inputs.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_mul(const matf32_t** const p_matarray, uint16_t length, matf32_t* p_dst);


/**
 * @brief   Calculates the pseudoinverse of a given matrix
 * 
 * @param[in]       p_a     Matrix to calculate the inverse from
 * @param[in,out]   p_pinv  Matrix to save the pseudoinverse in
 * 
 * @return None
 */
err_status_t
matf32_pinv(const matf32_t* const p_a, matf32_t* const p_pinv);


// ----------------------------------------------------------------------------------------------------
// 4.3. Matrix factorization/decomposition methods
// ----------------------------------------------------------------------------------------------------

// Maybe move this to matf32.h
err_status_t
matf32_qr(const matf32_t* const p_a, matf32_t* const p_q, matf32_t* const p_r);


// Maybe move this to matf32.h
/**
 * @brief   Computes the LU decomposition of a square matrix A, pointed by p_a,
 * such that PA = LU.
 *
 * @param[in]       p_a   Points to square matrix to decompose.
 * @param[in, out]  p_l     Points to the lower result of the decomposition.
 * @param[in, out]  p_u     Points to the upper of the decomposition.
 * @param[in, out]  p_index Saved indexes for permutation
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_lu(const matf32_t* p_a, matf32_t* const p_l, matf32_t* const p_u, uint16_t* p_index);


// Maybe move this to matf32.h
/**
 * @brief   Calculates the Cholesky decomposition of a matrix.
 *
 * @param[in]           p_a    Points to matrix to factorize.
 * @param[in,out]       p_c    Points to lower triangular factorized matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_cholesky(const matf32_t* const p_a, matf32_t* const p_c);

/**
 * @brief   Generates householder transformation vector and constant for a given matrix
 * 
 * @warning Assumes p_x is a column vector and works accordingly, so for now will fail if a row vector is introduced.
 * 
 * @param[in]       p_x     Points to vector to work with
 * @param[in,out]   p_v     Points to vector v to save the first householder vector
 * @param[in,out]   p_b     Points to scalar b to save the constant produced
 * 
 * @return err_status_t
 */
err_status_t
matf32_house(const matf32_t* const p_x, matf32_t* p_v, float* p_b);

/**
 * @brief   Generates a householder bidiagonalization (A = U'BV) based on a given matrix
 * 
 * @param[in]       p_a         Points to matrix A from which to generate the bidiagonalization
 * @param[in,out]   p_u         Points to U matrix
 * @param[in,out]   p_b         Points to B matrix
 * @param[in,out]   p_v         Points to V matrix
 * 
 * @return err_status_t
 */
err_status_t
matf32_house_bidiagonalization(const matf32_t* const p_a, matf32_t* p_u, matf32_t* p_b, matf32_t* p_v);

/**
 * @brief   Computes a Givens rotation matrix (square matrix) based on two input values
 * 
 * @warning This routine uses mathematical indexing, which means indices i and j start in 1 instead of 0.
 * 
 * @param[in]       a       Scalar a
 * @param[in]       b       Scalar b
 * @param[in]       i       Index i for rows of G to modify
 * @param[in]       j       Index j for cols of G to modify
 * @param[in,out]   p_g     Points to output matrix (2x2)
 * 
 * @return err_status_t
 */
err_status_t
matf32_givens_rotation(float a, float b, uint16_t i, uint16_t j, matf32_t* p_g);


#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_MATF32_H_

