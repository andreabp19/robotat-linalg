/**
 * @file matf32_check.h
 *
 * Matrix mathematical checks.
 *
 */


#ifndef ROBOTAT_MATF32_CHECK_H_
#define ROBOTAT_MATF32_CHECK_H_


 /**
  * Dependencies.
  */
#include "matf32_def.h"
#include "math_util.h"

// ====================================================================================================
// Matrix datatype-based checks
// ====================================================================================================


/**
 * @brief Checks if matrix is square
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not square.
 */
// static keyword fixes explained error of undefined ref when incuding constants.h in math_util.h
static inline bool 
matf32_check_square_matrix(const matf32_t* const p_mat)
{
    return (p_mat->num_cols == p_mat->num_rows);
}

/**
 * @brief Checks if matrix is upper triangular
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not upper triangular.
 */
bool
matf32_check_triangular_upper(const matf32_t* const p_mat);

/**
 * @brief Checks if matrix is lower triangular
 *
 * @param[in]   p_mat Points to the matrix to test.
 *
 * @return  Execution status
 *              true  :     Operation successful.
 *              false :     Matrix is not lower triangular.
 */
bool
matf32_check_triangular_lower(const matf32_t* const p_mat);


/**
 * @brief Checks if two matrices are equal
 *
 * @param[in]   p_mat_a Points to first matrix to compare.
 * @param[in]   p_mat_b Points to second matrix to compare.
 *
 * @return  Execution status
 *              true  :     Matrices are equal.
 *              false :     Matrices are not equal.
 */
bool
matf32_is_equal(const matf32_t* const p_mat_a, const matf32_t* const p_mat_b);


/**
 * @brief Checks if a matrix is symmetric
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrix is symmetrical.
 *              false :     Matrix is not symmetrical.
 */
bool
matf32_check_symmetric(const matf32_t* const p_mat);

/**
 * @brief Checks if a matrix is upper hessenberg.
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrices is upper hessenberg.
 *              false :     Matrices is not upper hessenberg.
 */
bool
matf32_check_hessenberg_upper(const matf32_t* const p_mat);

/**
 * @brief Checks if a matrix is lower hessenberg.
 *
 * @param[in]   p_mat Points to matrix to check.
 *
 * @return  Execution status
 *              true  :     Matrices is lower hessenberg.
 *              false :     Matrices is not lower hessenberg.
 */
bool
matf32_check_hessenberg_lower(const matf32_t* const p_mat);

#endif // ROBOTAT_MATF32_CHECK_H_
