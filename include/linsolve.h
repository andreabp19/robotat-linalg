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
// Matrix datatype-based linear solvers
// ====================================================================================================


/*
 * @brief   Solves a system a Lx=b system through forward substitution. L must be a lower triangular matrix,
 * the length of b and x must be the same as L amount of rows.
 *
 * @param[in]       p_l    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to otput x vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_forward_substitution(const matf32_t* const p_l, float* const p_b, float* p_x);


/*
 * @brief   Solves a system a Ux=b system through backward substitution. U must be a lower triangular matrix,
 * the length of b and x must be the same as U amount of rows.
 *
 * @param[in]       p_u    Points to lower triangular matrix.
 * @param[in]       p_b    Points to b vector.
 * @param[in,out]   p_x    Points to otput x vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_backward_substitution(const matf32_t* const p_u, float* const p_b, float* p_x);


err_status_t
matf32_cholesky(const matf32_t* const p_a, matf32_t* const p_l);

#endif // ROBOTAT_LINSOLVE_H_