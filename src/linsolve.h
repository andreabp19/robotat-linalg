/**
 * @file linsolve.h
 * 
 * Linear solver based on matf32_t datatype.
 * 
 * Created: 2022
 *      By: Daniel Pineda
 * Last modified: 21 Sep 2025
 *      By: Andrea Pineda
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
 * @brief Enumerated list for the available methods of the linear solver.
 */
typedef enum
{
    FORWARD_SUBS,
    BACKWARD_SUBS,
    CHOLESKY,
    QR,
    LU,
    SVD
} linsolve_method_t;


/**
 * @brief Enumerated list for the specific shapes that can be operated in certain methods such as QR.
 */
typedef enum
{
    SQUARE,
    RECT
} linsolve_matrix_shape_t;


/**
* @brief   Prints string representing the linear method.
* 
* @param[in]    lsm     Method to use
*
* @return None.
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
 *              CHOLESKY :      Cholesky factorization.
 *              QR :            QR factorization.
 *              LU :            LU factorization.
 *              SVD :           Singular Values Decomposition (SVD).
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
 *              MATH_ARGUMENT_ERROR :   Lower triangular matrix check failed.
 */
err_status_t
linsolve_forward_substitution(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x);


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
 *              MATH_ARGUMENT_ERROR :   Upper triangular matrix check failed.
 */
err_status_t
linsolve_backward_substitution(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x);


/**
 * @brief   Solves a system Cx = b through Cholesky factorization. C must be an upper triangle matrix,
 * the length of b and must be the same as the amount of rows in C.
 * 
 * @warning This method assumes that the Cholesky factor matrix is given as an upper triangular matrix,
 * exactly as the matf32_cholesky algorithm gives it currently (L' instead of L). If you modify matf32-cholesky,
 * to either output the cholesky factor matrix as lower triangular or saving both upper and lower parts in a single
 * matrix, make sure to update linsolve_cholesky accordingly, otherwise it won't operate correctly.
 * 
 * @param[in]       p_c     Points to upper triangular matrix.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful
 *              MATH_ARGUMENT_ERROR :   Input matrix checks in forward or backward substitution failed.
 */
err_status_t
linsolve_cholesky(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solves a system Ax = b through QR factorization. For A = QR, first computes y = Qb,
 * then solves Rx = y. Can be applied to either square or rectangular matrices, according to the
 * matrix used to generate the QR decomposition with matf32_qr.
 * 
 * @param[in]       p_q     Points to matrix q from QR decomposition of A.
 * @param[in]       p_r     Points to matrix R from QR decomposition of A.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful
 *              MATH_ARGUMENT_ERROR :   Input matrix check in backward substitution failed.
 */
err_status_t
linsolve_qr(matf32_t* const p_q, matf32_t* const p_r, const matf32_t* const p_b, matf32_t* const p_x, linsolve_matrix_shape_t shape);


/**
 * @brief   Solves a system Ax = b through LU factorization.
 * 
 * @param[in]       p_l     Points to matrix L from LU decomposition of A.
 * @param[in]       p_u     Points to matrix U from LU decomposition of A.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * @param[in]       p_index List of permutations executed during the LU decomposition of A.
 * 
 * @return  Execution status
 *              MATH_SUCESS :           Operation successful
 *              MATH_ARGUMENT_ERROR :   Input matrix checks in forward or backward substitution failed.
 */
err_status_t
linsolve_lu(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x, uint16_t* p_index);

/**
 * @brief   Solves a system Ax = b through SVD Factorization A = USV
 * 
 * @param[in]       p_u     Points to matrix U from SVD of A
 * @param[in]       p_s     Points to matrix S from SVD of A
 * @param[in]       p_v     Points to matrix V from SVD of A
 * @param[in]       p_b     Points to b vector
 * @param[in,out]   p_x     Points to output vector x
 * 
 * @return  Execution status     
 *              MATH_SUCCESS :          Operation successful
 *              ***Pending to add error status
 */
err_status_t
linsolve_svd(const matf32_t* const p_u, const matf32_t* const p_s, const matf32_t* const p_v, const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solve the linear system Ax=b, automatically selecting the method to use based on input matrix type.
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
linsolve(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solve the linear system Ax=b, with a specific method. It's recomended to use
 * this function if the problem, or the user, requires a specific method to solve linear systems.
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
linsolve_method(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* p_x, linsolve_method_t method, linsolve_matrix_shape_t shape);  

#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_LINSOLVE_H_
