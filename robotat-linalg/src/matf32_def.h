/**
 * @file matf32_def.h
 * @author Daniel Pineda
 *
 * Matrix type definition and related functions.
 *
 */


#ifndef ROBOTAT_MATF32_DEF_H_
#define ROBOTAT_MATF32_DEF_H_

 /**
  * Dependencies.
  */

#include <string.h>                     // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdint.h>                     // For uint8_t, uint16_t and uint16_t.
#include <math.h>                       // For sqrtf.
#include <stdbool.h>                    // For bool datatype.

#include "math_util.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================

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


// ====================================================================================================
// Util functions directly related to the matf32 datatype
// ====================================================================================================


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


// ====================================================================================================
// Sub-matrix operattions
// ====================================================================================================

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

// ====================================================================================================
// Special matrix initializations
// ====================================================================================================


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

#ifdef __cplusplus
}
#endif



#endif // ROBOTAT_MATF32_DEF_H_