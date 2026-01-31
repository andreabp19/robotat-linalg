/**
 * @defgroup linsolve
 * @{
 * 
 * @brief Linear solver based on matf32_t data type of the Matf32 Library.
 * The following methods are available for the solver: LU, QR, Cholesky,
 * Forward Substitution, Backward Subtitution and SVD.
 * 
 * @date Created on: 2022
 *      By: Daniel Pineda \n
 * Last modified: 14 jan. 2026
 *      By: Andrea Pineda \n
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
    FORWARD_SUBS,    /**< Solve system with Forward Substitution */
    BACKWARD_SUBS,   /**< Solve system with Backward Substitution */
    CHOLESKY,        /**< Solve system with Cholesky Factorization */
    QR_SQUARE,       /**< Solve system with QR Factorization for square matrices */
    QR_RECT,         /**< Solve system with QR Factorization for rectangular matrices (vertical or horitzontal) */
    LU,              /**< Solve system with LU Factorization */
    SVD              /**< Solve system with the Singular Value Decomposition (SVD) */
} linsolve_method_t;


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
 * @brief   Defines which method is the best to use depending on the shape ant type of matrix
 * in the system.
 *
 * @param[in]       p_a    Points to system matrix.
 *
 * @return  Returns a value of type linsolve_method_t:
 *              FORWARD_SUBS  : Forward substitution,
 *              BACKWARD_SUBS : Backward substitution,
 *              CHOLESKY      : Cholesky factorization,
 *              QR_SQUARE     : QR factorization for square matrices,
 *              QR_RECT       : QR factorization for rectangular matrices,
 *              LU            : LU factorization,
 *              SVD           : Singular Values Decomposition.
 */
linsolve_method_t
linsolve_get_method(const matf32_t* const p_a);



// ====================================================================================================
// Matrix datatype-based linear solvers
// ====================================================================================================


/**
 * @brief   Solves a linear system through forward substitution.
 * 
 * @warning Only applicable to systems where the matrix is lower triangular.
 *
 * @param[in]       p_l    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS        :   Operation successful,
 *              MATH_ARGUMENT_ERROR :   Lower triangular matrix check failed.
 */
err_status_t
linsolve_forward_substitution(const matf32_t* const p_l, const matf32_t* const p_b, matf32_t* p_x);


/**
 * @brief   Solves a system a linear system through backward substitution.
 * 
 * @warning Only applicable to systems where the matrix is upper triangular.
 *
 * @param[in]       p_u    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS        :   Operation successful,
 *              MATH_ARGUMENT_ERROR :   Upper triangular matrix check failed.
 */
err_status_t
linsolve_backward_substitution(const matf32_t* const p_u, const matf32_t* const p_b, matf32_t* p_x);


/**
 * @brief   Solves a linear system through Cholesky factorization. The factorization must be performed
 * before executing this routine.
 * 
 * 1. Solve \f$ \textbf{L}\textbf{y} = \textbf{b} \f$
 * 2. Solve \f$ \textbf{L$^\top$}\textbf{x} = \textbf{y} \f$
 * 
 * @warning This method assumes that the Cholesky factor matrix is introduced as an upper triangular matrix,
 * exactly as the matf32_cholesky algorithm generates it. The solution to the system will be incorrect
 * if the Cholesky Factor is introduced as a lower triangular matrix into the function.
 * 
 * @param[in]       p_c     Points to upper triangular matrix.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * 
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS        :   Operation successful,
 *              MATH_ARGUMENT_ERROR :   Input matrix checks in forward or backward substitution failed.
 */
err_status_t
linsolve_cholesky(matf32_t* const p_c,  const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solves a linear system through QR factorization. The factorization must be computed
 * before using this routine. Can be applied to systems with either square or rectangular matrices,
 * by introducing the QR_SQUARE or QR_RECT, respectively, in the linsolve_method_t argument. The system is
 * solved as follows:
 * 
 * 1. Compute the vector  \f$ \textbf{y} = \textbf{Q}^\top \textbf{b} \f$
 * 2. Solve the system \f$ \textbf{Rx} = \textbf{y} \f$
 * 
 * @param[in]       p_q     Points to matrix q from QR decomposition of A.
 * @param[in]       p_r     Points to matrix R from QR decomposition of A.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * 
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS        :   Operation successful,
 *              MATH_ARGUMENT_ERROR :   Input matrix check in backward substitution failed.
 */
err_status_t
linsolve_qr(matf32_t* const p_q, matf32_t* const p_r, const matf32_t* const p_b, matf32_t* const p_x, linsolve_method_t qr_shape);


/**
 * @brief   Solves a linear system through LU factorization. The factorization must be computed
 * before executing this routine, as the factorization's matrices are introduced as arguments. The
 * system is solved as follows:
 *
 * 1. Solve the system \f$ \textbf{Ly} = \textbf{b} \f$
 * 2. Then solve the system \f$ \textbf{Ux} = \textbf{y} \f$
 * 
 * @param[in]       p_l     Points to matrix L from LU decomposition of A.
 * @param[in]       p_u     Points to matrix U from LU decomposition of A.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output vector x.
 * @param[in]       p_index List of permutations executed during the LU decomposition of A.
 * 
 * @return  Returns execution status err_status_t:
 *              MATH_SUCESS         :   Operation successful,
 *              MATH_ARGUMENT_ERROR :   Input matrix checks in forward or backward substitution failed.
 */
err_status_t
linsolve_lu(const matf32_t* const p_l, const matf32_t* const p_u,  const matf32_t* const p_b, matf32_t* const p_x, uint16_t* p_index);

/**
 * @brief   Solves a linear system through SVD Factorization. The SVD must be computed
 * before executing this routine as it receives the matrices of the decomposition as arguments.
 * The solution to the system is computed as \f$ \textbf{x} = \textbf{V}\textbf{S}^{-1}\textbf{U}^\top\textbf{b} \f$
 *
 * @param[in]       p_u     Points to matrix U from SVD of A
 * @param[in]       p_s     Points to matrix S from SVD of A
 * @param[in]       p_v     Points to matrix V from SVD of A
 * @param[in]       p_b     Points to b vector
 * @param[in,out]   p_x     Points to output vector x
 * 
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS :          Operation successful
 */
err_status_t
linsolve_svd(const matf32_t* const p_u, const matf32_t* const p_s, const matf32_t* const p_v, const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solve a linear system of the form \f$ \textbf{Ax} = \textbf{b} \f$, automatically selecting the method to use based on the shape and type of input matrix.
 * 
 * @warning This solver does uses QR only for systems with rectangular matrices. To solve square matrix systems with QR, use
 * linsolve_method instead. Additionally, the ill-conditioned theshold can be modified in constants.h, according to your needs.
 * 
 * @param[in]       p_a    Points to system matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to output x vector.
 *
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS                :    Operation successful,
 *              MATH_SIZE_MISMATCH          :    Matrix size check failed,
 *              MATH_DECOMPOSITION_FAILURE  :    Failed decomposition method,
 *              MATH_ARGUMENT_ERROR         :    Incorrect arguments passed.
 */
err_status_t
linsolve(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* const p_x);


/**
 * @brief   Solve a linear system of the form \f$ \textbf{Ax} = \textbf{b} \f$ with a specific method selected in the arguments.
 *
 * @param[in]       p_a     Points to system matrix.
 * @param[in]       p_b     Points to b vector.
 * @param[in,out]   p_x     Points to output x vector.
 * @param[in]       method  Method to use.
 *
 * @return  Returns execution status err_status_t:
 *              MATH_SUCCESS                  : Operation successful,
 *              MATH_SIZE_MISMATCH            : Matrix size check failed,
 *              MATH_DECOMPOSITION_FAILURE    : Failed decomposition method,
 *              MATH_ARGUMENT_ERROR           : Incorrect arguments passed.
 */
err_status_t
linsolve_method(const matf32_t* const p_a, const matf32_t* const p_b, matf32_t* p_x, linsolve_method_t method);  

#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_LINSOLVE_H_

/**
 * @}
 */