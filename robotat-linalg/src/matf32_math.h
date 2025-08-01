/**
 * @file matf32_math.h
 *
 * Matrix type based mathematical operations.
 *
 */


#ifndef ROBOTAT_MATF32_MATH_H_
#define ROBOTAT_MATF32_MATH_H_


 /**
  * Dependencies.
  */

#include <string.h>                     // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdint.h>                     // For uint8_t, uint16_t and uint16_t.
#include <math.h>                       // For sqrtf.
#include <stdbool.h>                    // For bool datatype.

#include "matf32_def.h"
#include "constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// Matrix datatype-based linear algebra routines
// TODO: add inline function wrappers for native library (like CMS_DSP on ARM, ESP-IDF on esp, etc)
// ====================================================================================================


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

// ====================================================================================================
// Array of matrices operations
// ====================================================================================================

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

#ifdef __cplusplus
}
#endif



#endif // ROBOTAT_MATF32_MATH_H_