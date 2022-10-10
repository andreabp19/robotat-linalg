/**
 * @file linsolve.h
 * 
 * Linear solvers based on matf32 datatype.
 *
 */

#ifndef ROBOTAT_LINSOLVE_H_
#define ROBOTAT_LINSOLVE_H_

#include "matf32.h"


// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================


/**
 * @brief Linear solver method.
 * 
 */
typedef enum
{
    FORWARD_SUBS,
    BACKWARD_SUBS,
    CHOLESKY,
    QR,
    LU
} linsolve_method_t;

/**
*  @brief   Prints string representing the linear method.
*/
void
print_linsolve_method(linsolve_method_t lsm);


/**
 * @brief   Solves a system a Lx=b system through forward substitution. L must be a lower triangular matrix,
 * the length of b and x must be the same as L amount of rows.
 *
 * @param[in]       p_a    Points to system matrix.
 *
 * @return  linsolve_method_t
 *              FORWARD_SUBS :  Forward substitution.
 *              BACKWARD_SUBS : Backward substitution.
 *              CHOLESKY :      Cholesky factorization
 *              QR :            QR factorization.
 *              LU :            LU factorization.
 */
linsolve_method_t
matf32_linsolve_get_method(const matf32_t* const p_a);


// ====================================================================================================
// Matrix datatype-based linear solvers
// ====================================================================================================


/**
 * @brief   Solves a system a Lx=b system through forward substitution. L must be a lower triangular matrix,
 * the length of b and x must be the same as L amount of rows.
 *
 * @param[in]       p_l    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_forward_substitution(const matf32_t* const p_l, const float* const p_b, float* p_x);


/**
 * @brief   Solves a system a Ux=b system through backward substitution. U must be a lower triangular matrix,
 * the length of b and x must be the same as U amount of rows.
 *
 * @param[in]       p_u    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_backward_substitution(const matf32_t* const p_u, const float* const p_b, float* p_x);


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

err_status_t
matf32_cholesky_solve(matf32_t* const p_c,  const float* const p_b, float* const p_x);


/**
 * @brief   Computes the LU decomposition of a square matrix A, pointed by p_a,
 * such that PA = LU.
 *
 * @param[in]       p_a   Points to square matrix to decompose.
 * @param[in, out]  p_l     Points to the lower result of the decomposition.
 * @param[in, out]  p_u     Points to the upper of the decomposition.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_lu(const matf32_t* p_a, matf32_t* const p_l, matf32_t* const p_u);

err_status_t
matf32_lu_solve(const matf32_t* const p_l, const matf32_t* const p_u,  const float* const p_b, float* const p_x);


err_status_t
matf32_qr(const matf32_t* const p_a, matf32_t* const p_q, matf32_t* const p_r);


/**
 * @brief   Solve the linear system Ax=b,
 * automatically selecting the method to use according to A matrix type.
 *
 * @param[in]       p_a    Points to system matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :                  Operation successful.
 *              MATH_SIZE_MISMATCH :            Matrix size check failed.
 *              MATH_DECOMPOSITION_FAILURE :    Failed decomposition method.
 *              MATH_ARGUMENT_ERROR :           Incorrect arguments passed.
 */
err_status_t
matf32_linsolve(const matf32_t* const p_a, const float* const p_b, float* p_x);


/**
 * @brief   Solve the linear system Ax=b, with specified method.
 *
 * @param[in]       p_a     Points to system matrix.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output x vector.
 * @param[in]       method  Method to use.
 *
 * @return  Execution status
 *              MATH_SUCCESS :                  Operation successful.
 *              MATH_SIZE_MISMATCH :            Matrix size check failed.
 *              MATH_DECOMPOSITION_FAILURE :    Failed decomposition method.
 *              MATH_ARGUMENT_ERROR :           Incorrect arguments passed.
 */
err_status_t
matf32_linsolve_method(const matf32_t* const p_a, const float* const p_b, float* p_x, linsolve_method_t method);


#endif // ROBOTAT_LINSOLVE_H_