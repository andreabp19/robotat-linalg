/**
 * @defgroup quadprog
 * @{
 * 
 * @brief Convex quadratic problem solver,
 * with direct methods for inequality constrained problems and an
 * active-set method for inequality constrained problems.
 *
 * Last modified: 15 Jan. 2026
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
typedef struct
{
    const matf32_t* p_Q;      /**< Cost function matrix*/
    const matf32_t* p_c;      /**< Cost function vector */
    const matf32_t* p_Ain;    /**< Inequality conditions matrix */
    const matf32_t* p_bin;    /**< Inequality conditions vector */
    const matf32_t* p_Aeq;    /**< Equality conditions matrix */
    const matf32_t* p_beq;    /**< Equality conditions vector */
    const matf32_t* p_x0;     /**< Starting point */
} quadprog_t;


/**
 * @brief Operation status from quadprog.
 */
typedef enum
{
    QP_SUCESS,          /**< Operation completed correctly */
    QP_SIZE_MISMATCH,   /**< Matrices/vectors are not the correct size */
    QP_NOT_RESTRICTED,  /**< Missing restrictions */
    QP_NOT_CONVEX,      /**< Problem is not convex */
    QP_BAD_DEFINED      /**< Problem is not correctly defined */
} quadprog_status_t;

/**
 * @brief Quadprog-specific methods for solving QPs
 */
typedef enum
{
    QP_KKT_NULLSPACE,   /**< Direct method for solving a KKT system using the null-space matrix of Aeq */
    QP_KKT_LDLT,        /**< Direct method for solving a KKT system using the LDL' symmetric indefinite factorization */
    QP_LU,              /**< Solve an equality constrained QP with linsolve using LU (method not specific for KKT systems)*/
    QP_SVD,             /**< Solve an equality constrained QP using SVD (method not specific for KKT systems) */
    QP_QR,              /**< Solve an equality constrained QP using QR (method not specific for KKT systems) */
    SQP_ACTIVE_SET      /**< Solve a constrained QP with an active set binding direction method */
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
 * @brief   Equality restricted quadratic convex problem solver by constructing a KKT system
 * and then solving it through one of the following linsolve methods: LU, SVD or QR (the method
 * must be specificed in the arguments of the function). The KKT system is constructed as follows:
 * 
 * \f[ 
 * \begin{bmatrix}
 * \textbf{Q} & \textbf{A}^\top \\
 * \textbf{A} & \textbf{0}
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{-p} \\
 * \boldsymbol{\lambda}
 * \end{bmatrix}
 * =
 * \begin{bmatrix}
 * \textbf{c} \\
 * \textbf{b} \\
 * \end{bmatrix}
 * \f]
 * 
 * according to the matrices of the cost function and restrictions of the QP, and the definition
 * of the KKT system (Nocedal, Numerical Optimization).
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
 * @brief   Solves a convex quadratic problem with equality restrictions using the null-space
 * method for KKT systems. This method does NOT construct and solve the KKT system directly,
 * instead, it does the following procedure as described in Nocedal, Numerical Optimization:
 * 
 * Starting from the definition of a KKT system for the problem with equality restrictions:
 * 
 * \f[ 
 * \begin{bmatrix}
 * \textbf{Q} & \textbf{A}^\top \\
 * \textbf{A} & \textbf{0}
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{-p} \\
 * \boldsymbol{\lambda}
 * \end{bmatrix}
 * =
 * \begin{bmatrix}
 * \textbf{c} \\
 * \textbf{b} \\
 * \end{bmatrix}
 * \f]
 * 
 * The solution vector \f$ \textbf{p} \f$ is calculated through partial solutions, which
 * are as follows:
 * 
 * \f[ \textbf{p} = \textbf{Yp}_y + \textbf{Zp}_z \f]
 * 
 * where \f$ \textbf{Y} \f$ is (n x m) and \f$ \textbf{Z} \f$ is (n x (n-m)), and are generated
 * from the \f$ \textbf{Q} \f$ matrix of the QR factorization.
 * 
 * Then, the vectors \f$ \textbf{p}_y \f$ and \f$ \textbf{p}_z \f$ of the partial solutions
 * are calculated, by solving the following linear systems:
 * 
 * \f[ \textbf{AYp}_y = - \textbf{h} \f]
 * \f[ \textbf{Z}^\top \textbf{QZp}_z = -\textbf{Z}^\top \textbf{Q} \textbf{Yp}_y - \textbf{Z}^\top \textbf{g} \f]
 * 
 * After which, \f$ \textbf{p} \f$ can be calculated.
 * 
 * Currently, this routine does NOT calculate the Lagrange multipliers associated to the solution
 * of the QP, but if you need them, the following operation should be added:
 * 
 * \f[ \textbf{AY}^\top \boldsymbol{\lambda} = \textbf{Y}^\top (\textbf{g} + \textbf{Gp}) \f] 
 * 
 * 
 * 
 * @param[in]       p_qp Points to the structure representing the problem to solve.
 * @param[in,out]   p_x  Points to the vector to store the solution for the QP.
 * 
 * @return Execution status.
 */
quadprog_status_t 
quadprog_qp_nullspace(quadprog_t* p_qp, matf32_t* const p_x);

/**
 * @brief Solves a convex quadratic problem with equality restrictions using the LDL' factorization, as 
 * explained in Nocedal, Numerical Optimization.
 * 
 * Starting with the KKT system for the problem with equality restrictions:
 * 
 * \f[ 
 * \begin{bmatrix}
 * \textbf{Q} & \textbf{A}^\top \\
 * \textbf{A} & \textbf{0}
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{-p} \\
 * \boldsymbol{\lambda}
 * \end{bmatrix}
 * =
 * \begin{bmatrix}
 * \textbf{c} \\
 * \textbf{b} \\
 * \end{bmatrix}
 * \f]
 * 
 * The \f$ \textbf{LDL}^\top \f$ factorization is applied to the matrix of the system, to get
 * the resulting structure: 
 * 
 * \f[ \textbf{P}^\top \textbf{MP} = \textbf{LDL}^\top \f]
 * 
 * where \f$ \textbf{P} \f$ is a permutation matrix, while \f$ \textbf{L} \f$
 * and \f$ \textbf{D} \f$ are, respectively, a lower triangular and diagonal matrix,
 * all generated during the procedure of the factorization. 
 * 
 * Afterwards, the KKT system is solved as follows:
 * 
 * \f[ \textbf{Lz}_1 = \textbf{P}^\top \textbf{b} \f]
 * \f[ \textbf{Bz}_2 = \textbf{z}_1 \f]
 * \f[ \textbf{L}^\top \textbf{z}_3 = \textbf{z}_2 \f]
 * \f[ \textbf{y} = \textbf{Pz}_3 \f]
 * 
 * where:
 * 
 * \f[ \textbf{b} =
 * \begin{bmatrix}
 * \textbf{g} \\
 * \textbf{h}
 * \end{bmatrix}
 * \f]
 * 
 * \f[ \textbf{y} = 
 * \begin{bmatrix}
 * - \textbf{p} \\
 * \boldsymbol{\lambda}
 * \end{bmatrix}
 * \f]
 * 
 * The specific algorithm in this routine for the factorization corresponds to
 * the symmetric indefinite factorization with Bunch and Parlett Pivoting as presented
 * in section 4.4.3 of Golub, Matrix Computations. The different linear systems in the
 * procedure are solved using linsolve.
 * 
 * @param[in]       p_qp Points to the structure representing the problem to solve.
 * @param[in,out]   p_x  Points to the vector to store the solution for the QP.
 * 
 * @return Execution status
 */
quadprog_status_t
quadprog_qp_ldlt(quadprog_t* p_qp, matf32_t* const p_x);

/**
 * @brief   Solves a convex quadratic problem with an active-set binding direction method, based on
 * Martins, Engineering Design Optimization, algorithm 5.4, but divided in four cases depending on
 * the combination of restrictions that can be found in each iteration, so that the KKT system
 * that's built and solved in each iteration is as small as possible (increasing solving speed and
 * stability). As such, the procedure implemented in this routine is as follows:
 * 
 * Case 1: There are both equality and inequality restrictions
 * 
 * \f[
 * \begin{bmatrix}
 * \textbf{Q}       & \textbf{A}_{eq}^\top   & \textbf{A}_w^\top\\
 * \textbf{A}_{eq}  &  0            & 0 \\
 * \textbf{A}_w     &  0            & 0
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{p}_k \\
 * \boldsymbol{\lambda}_{eq} \\
 * \boldsymbol{\lambda}_w
 * \end{bmatrix} = -
 * \begin{bmatrix}
 * \textbf{Qx}_k + \textbf{c} \\
 * \textbf{A}_{eq}\textbf{x}_k - \textbf{b}_{eq} \\
 * \textbf{A}_w \textbf{x}_k - \textbf{b}_w
 * \end{bmatrix}
 * \f]
 * 
 * Case 2: There are only equality restrictions
 * 
 * \f[
 * \begin{bmatrix}
 * \textbf{Q}   & \textbf{A}_{eq}^\top \\
 * \textbf{A}_{eq} &  0 
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{p}_k \\
 * \boldsymbol{\lambda}_eq
 * \end{bmatrix} = -
 * \begin{bmatrix}
 * \textbf{Qx}_k + \textbf{c} \\
 * \textbf{A}_{eq} - \textbf{b}_{eq}
 * \end{bmatrix}
 * \f]
 * 
 * Case 3: There are only inequality restrictions (and at least one is active)
 * 
 * \f[
 * \begin{bmatrix}
 * \textbf{Q}   & \textbf{A}_w^\top \\
 * \textbf{A}_w &  0 
 * \end{bmatrix}
 * \begin{bmatrix}
 * \textbf{p}_k \\
 * \boldsymbol{\lambda}_w
 * \end{bmatrix} = -
 * \begin{bmatrix}
 * \textbf{Qx}_k + \textbf{c} \\
 * \textbf{A}_w - \textbf{b}_w
 * \end{bmatrix}
 * \f]
 * 
 * Case 4: There are no restrictions (either the problem was unconstrained or had
 * only inequality restrictions such as, currently, all are inactive)
 * 
 * \f[ \textbf{Qp}_k = -(\textbf{Qx}_k + \textbf{c}) \f]
 * 
 * @warning This routine currently uses quadprog_qp_linsolve to solve the KKT system
 * inside each iteration of the active-set method, and it's currently set to use
 * the LU factorization to solve the system. this can be changed to use SVD or QR
 * by modifying the corresponding argument in quadprog_qp_linsolve. Take into
 * consideration that using the SVD makes the routine slower. On the other hand,
 * quadprog_qp_nullspace and quadprog_qp_ldlt are not currently used nor tested in the
 * active-set method, so if you want to use those routines inside the active-set,
 * further modifications might be needed to quadprog_sqp. Though, for devices
 * with very limited FLASH memory capacity, quadprog_qp_ldlt is NOT recommended as it's
 * the most memory-expensive method of the available quadprog routines and in combination
 * with quadprog_sqp, could increase the memory usage considerably.
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

/**
 * @}
 */
