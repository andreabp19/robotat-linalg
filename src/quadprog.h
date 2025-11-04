/**
 * @file quadsprog.h
 *
 * Last modified: 26 Oct 2025
 *          By: Andrea Pineda
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
 * @brief Quadratic Program (QP or quadprogram) problem structure.
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
 */
typedef enum
{
    QP_SUCESS,          /** Operation completed correctly */
    QP_SIZE_MISMATCH,   /** Matrices/vectors are not the correct size */
    QP_NOT_RESTRICTED,  /** Missing restrictions */
    QP_NOT_CONVEX,      /** Problem is not convex */
    QP_BAD_DEFINED      /** Problem is not correctly defined */
} quadprog_status_t;

/**
 * @brief Quadprog-specific methods for solving QPs
 */
typedef enum
{
    QP_KKT_NULLSPACE,   /** Direct method for solving a KKT system using the null-space matrix of Aeq */
    QP_KKT_LDLT,        /** Direct method for solving a KKT system using the LDL' symmetric indefinite factorization */
    QP_LU,              /** Solve an equality constrained QP with linsolve using LU (method not specific for KKT systems)*/
    QP_SVD,             /** Solve an equality constrained QP using SVD (method not specific for KKT systems) */
    QP_QR,              /** Solve an equality constrained QP using QR (method not specific for KKT systems) */
    SQP_ACTIVE_SET      /** Solve a constrained QP with an active set binding direction method */
} quadprog_method_t;


/**
 * @brief   Print quadprog status.
 *
 * @param[in]  p_qp quadprog execution status.
 *
 * @return  None.
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
 * @brief   Quadratic convex problem solver, choosing automatically the method to be used
 * 
 * @param[in]       p_qp    Points to the structure representing the problem to solve.
 * @param[in,out]   p_x     Points to the vector to store the result.
 * 
 * @return  Execution status.
 */
quadprog_status_t
quadprog(quadprog_t* p_qp, matf32_t* const p_x, bool set_linsolve_method, quadprog_method_t method);


/**
 * @brief   Equality restricted quadratic convex problem solver using linsolve methods: LU, SVD or QR
 *
 * @param[in]  p_qp     Points to the structure representing the problem to solve.
 * @param[out] p_x      Points to the vector to store the solution to the QP.
 * @param[in]  method   Linsolve method to use for solving the system: LU, SVD or QR.
 *
 * @return  Execution status.
 */
quadprog_status_t
quadprog_qp_linsolve(quadprog_t* p_qp, matf32_t* const p_x, linsolve_method_t method);

/**
 * @brief   Solves a convex quadratic problem with equality restrictions using the null-space method for KKT systems
 * 
 * @param[in]       p_qp Points to the structure representing the problem to solve.
 * @param[in,out]   p_x  Points to the vector to store the solution for the QP.
 * 
 * @return Execution status.
 */
quadprog_status_t 
quadprog_qp_nullspace(quadprog_t* p_qp, matf32_t* const p_x);

/**
 * @brief Solves a convex quadratic problemwith equality restrictions using the LDL' factorization,
 * Based on LDL' with Bunch and Parlett Pivoting presented in section 4.4.3 of Golub, Matrix Computations.
 * 
 * @param[in]       p_qp Points to the structure representing the problem to solve.
 * @param[in,out]   p_x  Points to the vector to store the solution for the QP.
 * 
 * @return Execution status
 */
quadprog_status_t
quadprog_qp_ldlt(quadprog_t* p_qp, matf32_t* const p_x);

/**
 * @brief   Inequality restricted quadratic convex problem solver.
 *
 * @param[in]  p_qp Points to the structure representing the problem to solve.
 * @param[in,out] p_x  Points to the vector to store the result.
 *
 * @return  Execution status.
 */
quadprog_status_t
quadprog_sqp(quadprog_t* p_qp, matf32_t* const p_x);

#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_QUADPROG_H_
