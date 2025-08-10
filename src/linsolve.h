/**
 * @file linsolve.h
 * 
 * Linear solvers based on matf32 datatype.
 * 
 * Modified 4 Aug 2025
 *      By: Andrea Pineda | Adjusted some function names and moved matf32_qr, matf32_lu and matf32_cholesky
 *                          to matf32, to keep the operations there so that here is only the implementation.
 */

#ifndef ROBOTAT_LINSOLVE_H_
#define ROBOTAT_LINSOLVE_H_

#include "matf32.h"

#ifdef __cplusplus
extern "C" {
#endif

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
    LU,
    SCHUR
} linsolve_method_t;

/**
*  @brief   Prints string representing the linear method.
*/
void
linsolve_print_method(linsolve_method_t lsm);


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
linsolve_get_method(const matf32_t* const p_a);


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
linsolve_forward_substitution_matf32(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x);


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
linsolve_backward_substitution_matf32(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x);


/**
 * @brief   Pending
 * 
 * 
 */
err_status_t
linsolve_cholesky_matf32(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x);


err_status_t
matf32_lu_solve(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x);


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
linsolve_matf32(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* const p_x);


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
linsolve_matf32_method(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* p_x, linsolve_method_t method);  

#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_LINSOLVE_H_
