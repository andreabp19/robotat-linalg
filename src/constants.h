/**
 * @file constants.h
 *
 * Library configuration macros.
 * 
 * Last modified: 21 Sep 2025
 *      By: Andrea Pineda
 *
 */

#ifndef ROBOTAT_CONSTANTS_H_
#define ROBOTAT_CONSTANTS_H_

// ====================================================================================================
// Constant macro definitions
// ====================================================================================================
#define MAX_ITERATION_COUNT_SVD (100)    /**< Maximum number of iterations for svd_jacobi_one_sided.c */
#define MAX_ITERATION_COUNT_SQP (30)    /**< Maximum number of iterations for quadprog_sqp */
#define MAX_VEC_SIZE            (10)   /**< Maximum number of elements allowed for a single row vector. */
#define MAX_MAT_SIZE            (MAX_VEC_SIZE*MAX_VEC_SIZE)     /**< Maximum number of elements allowed for a matrix. */
#define MATH_MATRIX_CHECK               /**< Comment this to disable matrix size checking. */
#define MATH_EQUAL_PRECISION    (1E-5f)  /**< Precision of equal comparisons. WARNING: Algorithms may break if they can't reach specified precision. Adjust as needed.*/
#define ILL_CONDITIONING_THRESHOLD (0.0) /** Number starting from which to consider ill-conditioning and apply SVD in linsolve */

#endif // ROBOTAT_CONSTANTS_H_
