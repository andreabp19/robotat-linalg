/**
 * @file quadsprog.h
 *
 */

#ifndef ROBOTAT_QUADPROG_H_
#define ROBOTAT_QUADPROG_H_

#include "matf32.h"
#include "linsolve.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================


/**
 * @brief Quadprogam problem structure.
 * 
 */
typedef struct qp_t
{
    const matf32_t* p_Q;      /** Cost function matrix*/
    const matf32_t* p_c;      /** Cost function vector */
    const matf32_t* p_Ain;    /** Inequality conditions matrix */
    const matf32_t* p_bin;    /** Inequality conditions vector */
    const matf32_t* p_Aeq;    /** Equality conditions matrix */
    const matf32_t* p_beq;    /** Equality conditions vector */
    const matf32_t* p_x0;     /** Starting point */
} quadprog_t;


/**
 * @brief Operation status from quadprog.
 * 
 */
typedef enum
{
    QP_SUCESS,
    QP_SIZE_MISMATCH,   /** Matrices/vectors are not the correct size */
    QP_NOT_RESTRICTED,  /** Missing restrictions */
    QP_NOT_CONVEX       /** Problem is not convex */
} quadprog_status_t;


/**
 * @brief   Print quadprog status.
 *
 * @param[in]  p_qp quadprog execution status.
 *
 * @return  None
 */
void
quadprog_status_print(quadprog_status_t status);

/**
 * @brief   Constructor for the quadratic problem data structure.
 * Set as null any argment that isn't needed.
 * 
 * @param[in, out]  p_qp    Points to structure representing the problem.
 * @param[in]       p_Q     Cost function matrix.
 * @param[in]       p_c     Cost function vector.
 * @param[in]       p_Ain   Inequality conditions matrix.
 * @param[in]       p_vin   Inequality conditions vector.
 * @param[in]       p_Aeq   Equality conditions matrix.
 * @param[in]       p_veq   Equality conditions vector.
 * @param[in]       p_x0    Starting point.
 * 
 * @return  None
 */
void
quadprog_init(quadprog_t* const p_qp,
              const matf32_t* const p_Q, const matf32_t* const p_c,
              const matf32_t* const p_Aeq, const matf32_t* const p_beq,
              const matf32_t* const p_Ain, const matf32_t* const p_bin,
              const matf32_t* const p_x0);


/**
 * @brief   Quadratic convex problem solver.
 * 
 * @param[in]  p_qp Points to the structure representing the problem to solve.
 * @param[out] p_x  Points to the vector to store the result.
 * 
 * @return  Execution status.
 */
quadprog_status_t
quadprog(quadprog_t* p_qp, matf32_t* const p_x);


#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_QUADPROG_H_
