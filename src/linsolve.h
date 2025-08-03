/**
 * @file linsolve.h
 * 
 * Linear solvers based on matf32 datatype.
 * 
 * Modified 3 Aug 2025
 *      By: Andrea Pineda | Changed function names to start with linsolve and end with matf32.
 *                          Exception: matf32_cholesky, matf32_lu and matf32_qr, because I don't know
 *                          if it would be better to move those to matf32.h as they only calculate the
 *                          factorizations, while linsolve_cholesky_matf32 solves a linear system with
 *                          the cholesky method, for example.
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
linsolve_get_method_matf32(const matf32_t* const p_a);


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
linsolve_forward_subs_matf32(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x);


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
linsolve_backward_subs_matf32(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x);


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
 * @brief   Pending
 * 
 * 
 */
err_status_t
linsolve_cholesky_matf32(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x);

// Maybe move this to matf32.h
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
matf32_lu_solve(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x);

// Maybe move this to matf32.h
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
