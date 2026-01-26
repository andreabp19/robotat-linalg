/**
 * Library configuration constants.
 * 
 * Last modified: 22 Jan. 2026
 *      By: Andrea Pineda
 */

#ifndef ROBOTAT_CONSTANTS_H_
#define ROBOTAT_CONSTANTS_H_

// ====================================================================================================
// Constant macro definitions
// ====================================================================================================
#define MAX_ITERATION_COUNT_SVD (100)    /**< Maximum number of iterations for svd_jacobi_one_sided.c. Reduce or increase as needed. */
#define MAX_ITERATION_COUNT_SQP (30)    /**< Maximum number of iterations for quadprog_sqp. The algorithm will end when it converges, even if it hasn't reached the limit of iterations. */
#define MAX_VEC_SIZE            (10)   /**< Maximum number of elements allowed for a single row vector. */
#define MAX_MAT_SIZE            (MAX_VEC_SIZE*MAX_VEC_SIZE)     /**< Maximum number of elements allowed for a matrix. WARNING: Routines are tested up to 10x10 matrices, increase with care. */
#define MATH_MATRIX_CHECK               /**< Comment this to disable matrix size checking. */
#define MATH_EQUAL_PRECISION    (1E-5f)  /**< Precision of equal comparisons. WARNING: Algorithms may break if they can't reach specified precision. Adjust as needed.*/
#define ILL_CONDITIONING_THRESHOLD (100.0) /** Condition Number starting from which linsolve will use SVD to solve linear systems */

#endif // ROBOTAT_CONSTANTS_H_