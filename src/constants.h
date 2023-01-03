/**
 * @file constants.h
 *
 * Library configuration macros.
 *
 */

#ifndef ROBOTAT_CONSTANTS_H_
#define ROBOTAT_CONSTANTS_H_

// ====================================================================================================
// Constant macro definitions
// ====================================================================================================
#define MAX_ITERATION_COUNT_SVD (30)    /**< Maximum number of iterations for svd_jacobi_one_sided.c */
#define MAX_ITERATION_COUNT_SQP (30)    /**< Maximum number of iterations for quadprog_sqp */
#define MAX_VEC_SIZE            (10)   /**< Maximum number of elements allowed for a single row vector. */
#define MAX_MAT_SIZE            (MAX_VEC_SIZE*MAX_VEC_SIZE)     /**< Maximum number of elements allowed for a matrix. */
#define MATH_MATRIX_CHECK               /**< Comment this to disable matrix size checking. */
#define MATH_EQUAL_PRECISION    (1E-5)  /**< Precision of equal comparisons. WARNING: Algorithms may break if they can't reach specified precision. Adjust as needed.*/

#endif // ROBOTAT_CONSTANTS_H_
