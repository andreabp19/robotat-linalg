/**
 * @defgroup RobotatControl
 * @{
 * @author Miguel Zea (mezea@uvg.edu.gt)
 * @brief Library for control algorithms based on Robotat Linalg. Includes routines
 * for PID controllers, LTI systems, non linear systems, Kalman Filters and
 * an LTI, shooting-based MPC.
 * @version 0.1
 * @date 2021-08-12
 * 
 * Last modified: 26 Jan. 2026
 *      By: Andrea Pineda
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef ROBOTAT_CONTROL_H_
#define ROBOTAT_CONTROL_H_

 /**
  * Dependencies. 
  */
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>

#include "matf32.h"
#include "linsolve.h"
#include "quadprog.h"

#ifdef __cplusplus
extern "C" {
#endif


// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================
// NOTE: these should not be manipulated directly, use the init, setter and getter routines instead.

/**
 * @brief   Discretization specification data type.
 * 
 * Used to specify the numerical integration scheme to use when discretizing LTI systems and controllers.
 */
typedef enum
{
    PURE_DISCRETE,  /**< Sampling period independent. */
    FWD_EULER,      /**< Forward Euler integration. */
    BWD_EULER,      /**< Backward Euler integration. */
    TUSTIN,         /**< Trapezoidal rule. */
    ZOH,            /**< Zero-order hold. */
    RK4,            /**< 4th order Runge-Kutta. */
} ctr_discretizations_t;


/**
 * @brief   PID controller data structure, to represent a discretized PID, based on the
 * following definition of a PID and selecting one of the available discretization methods:
 * 
 * \f[
 * C(s) = k_P + \frac{k_I}{s} + k_D s
 * \f]
 * 
 */
typedef struct
{
    float kp;                       /**< Proportional gain. */
    float ki;                       /**< Integral gain. */
    float kd;                       /**< Derivative gain. */
    float e_k;                      /**< Error */
    float e_k_1;                    /**< Last error. */
    float e_k_2;                    /**< Error previous to last error */
    float u_k_1;                    /**< Last controller output. */
    float u_k_2;                    /**< Control output previous to last controller output */
    float i_min;                    /**< Lower integrator saturation threshold. */
    float i_max;                    /**< Upper integrator saturation threshold. */
    float tau;                      /**< Time constant of the derivative HPF. */
    float dt;                       /**< Sampling period. */
    ctr_discretizations_t pid_alg;  /**< Specifies the discretization scheme to be used. */
} ctr_pid_t;

/**
 * @brief   State space LTI system data structure, based on the following definition:
 * 
 * \f[
 * \begin{aligned}
 * \textbf{x}(t) &= \textbf{Ax}(t) + \textbf{Bu}(t) \\
 * \textbf{y}(t) &= \textbf{Cx}(t) + \textbf{Du}(t) \\ 
 * \end{aligned}
 * \f]
 */
typedef struct
{
    matf32_t* state;        /**< State at the current time step (used for simulation/integration). */
    uint16_t state_dim;     /**< Number of state variables. This is redundant but we'll keep it for completeness. */
    uint16_t input_dim;     /**< Number of inputs/actuators/controls. */
    uint16_t output_dim;    /**< Number of outputs/measurements. */
    matf32_t* A;            /**< System matrix. */
    matf32_t* B;            /**< Input/actuator matrix. */
    matf32_t* C;            /**< Output/sensor matrix. */
    matf32_t* D;            /**< Feedforward terms. */
    float dt;               /**< Sampling period (for discrete time systems). */
    bool is_continuous;     /**< System time domain specification. */
} ctr_sys_lti_t;


/**
 * @brief   State space nonlinear system data structure.
 */
typedef struct
{
    matf32_t* state;                                                                /**< State at the current time step (used for simulation/integration). */
    uint16_t state_dim;                                                             /**< Number of state variables. This is redundant but we'll keep it for completeness. */
    uint16_t input_dim;                                                             /**< Number of inputs/actuators/controls. */
    uint16_t output_dim;                                                            /**< Number of outputs/measurements. */
    err_status_t (*dynamics)(matf32_t* const, const matf32_t*, const matf32_t*);    /**< System dynamics. */
    err_status_t (*outputs)(matf32_t* const, const matf32_t*, const matf32_t*);     /**< System outputs. */
    float dt;                                                                       /**< Sampling period (for discrete time systems). */
    bool is_continuous;                                                             /**< System time domain specification. */
} ctr_sys_nonlin_t;


/**
 * @brief   Linear time-varying Kalman filter data structure, based on the following
 * definition:
 * 
 * \f[
 * \begin{aligned}
 * \textbf{x}_{k+1} &= \textbf{F} \textbf{x}_k + \textbf{G} \textbf{u}_k + \textbf{v}_k, \\
 * \textbf{z}_{k+1} &= \textbf{H} \textbf{x}_k + \textbf{w}_k
 * \end{aligned}
 * \f]
 */
typedef struct
{
    ctr_sys_lti_t* sys;     /**< LTI system model(has to be discrete time). */
    matf32_t* F;            /**< Coupling matrix for the process noise. */
    matf32_t* Qw;           /**< Process noise covariance matrix. */
    matf32_t* Qv;           /**< Measurement noise covariance matrix. */
    matf32_t* xhat;         /**< State estimate. */
    matf32_t* P;            /**< Estimation covariance matrix. */
} ctr_kalman_t;

/**
 * @brief   Struct for the matrices to be used in the MPC (Unconstrained, Shooting-Based, LTI)
 * Can be implemented with or without constraints.
 */
typedef struct
{
    bool state_constraints; /**< Boolean: 0=input-only constraints, 1=include state limits as input constraints */
    float N;                /**< Horizon length */
    float ub;               /**< Upper bounds limit */
    float lb;               /**< Lower bounds limit */
    ctr_sys_lti_t* sys;     /**< LTI system struct */
    quadprog_t* qp;         /**< Quadratic Program (QP) struct */
    matf32_t** mpc_C;       /**< Points to convolution matrix C data arrays */
    matf32_t** mpc_M;       /**< Points to data arrays for the powers of A matrix M */
    matf32_t* mpc_Q;        /**< Points to penalization matrix Q */
    matf32_t* mpc_R;        /**< Points to penalization matrix R */
    matf32_t* mpc_S;        /**< Points to penalization matrix S */
    matf32_t* u_k;          /**< Predicted input trajectory */
    matf32_t* x_k;          /**< Predicted state trajectory */
    matf32_t* Ain;          /**< Inequality restrictions matrix */
    matf32_t* bin;          /**< Inequality restrictions vector */
} ctr_mpc_lti_shooting_t;



// ====================================================================================================
// Public function prototypes
// ====================================================================================================
// ====================================================================================================
// 1. PID Control
// ====================================================================================================

/**
 * @brief   Initializes a PID controller structure.
 * 
 * The controller transfer function (except for the PURE_DISCRETE case) is given by:
 * C(s) = kP + kI/s + kD * tau*s / (s + tau).
 * Depending on the discretization scheme and whether or not the integrator saturates, the function 
 * can ask for additional parameters:
 * 
 * 1) ctr_pid_init(pid, kp, ki, kd, PURE_DISCRETE, 0); 
 * 2) ctr_pid_init(pid, kp, ki, kd, PURE_DISCRETE, 1, i_min, i_max); // With saturation limits.
 * 3) ctr_pid_init(pid, kp, ki, kd, pid_alg, 0, dt, tau); // Needs sampling period and time constant.
 * 4) ctr_pid_init(pid, kp, ki, kd, pid_alg, 1, dt, tau, i_min, i_max); // Needs all info. 
 * 
 * @param[in, out]  pid             PID controller data structure.
 * @param[in]       kp              Proportional gain.
 * @param[in]       ki              Integral gain.
 * @param[in]       kd              Derivative gain.
 * @param[in]       pid_alg         PID discretization scheme specification.
 * @param[in]       set_i_limits    Allows to set lower and upper saturation thresholds for the integrator.    
 * @param[in]       dt              Sampling period.
 * @param[in]       tau             Derivative HPF time constant.
 * @param[in]       i_min           Lower saturation threshold.
 * @param[in]       i_max           Upper saturation threshold.
 * 
 * @return  None.
 */
void
ctr_pid_init(ctr_pid_t* const pid, float kp, float ki, float kd, ctr_discretizations_t pid_alg, bool set_i_limits, ...);

/**
 * @brief   Sets new gains for the PID controller.
 * 
 * @param[in, out]  pid     PID controller data structure.
 * @param[in]       kp      New proportional gain.
 * @param[in]       ki      New integral gain.
 * @param[in]       kd      New derivative gain.
 * 
 * @return  None.
 */
static inline void
ctr_pid_set_gains(ctr_pid_t* const pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}


/**
 * @brief   Updates a previously initialized PID controller based on the discretization method
 * selected for the controller. The equations corresponding to each method are described below:
 * 
 * Pure Discrete:
 * \f[
 * \begin{aligned}
 * \end{aligned}
 * u_k = k_Pe_k + k_I(e_{k-1} + e_k) + k_D(e_k - e_{k-1})
 * \f]
 * 
 * Forward Euler:
 * \f[
 * \begin{aligned}
 * a_0 &= \tau \\
 * u_k &= k_P \tau + k_I \tau + k_D)e_k \\
 * &+ (k_P(T - 2\tau) + k_I(T- \tau) - 2k_D)e_{k-1} \\
 * &+ (k_P(\tau - T) + k_D)e_{k-2} \\
 * &- (T - 2\tau)e_{k-2} \\
 * &- (T - 2\tau) u_{k-1} \\
 * &- (\tau - T)u_{k-2} \\
 * u_k &= u_k / a_0
 * \end{aligned}
 * \f]
 * 
 * Backward Euler:
 * \f[
 * \begin{aligned}
 * a_0 &= T + \tau \\
 * u_k &= (k_P(T + \tau) + k_I(T + \tau) + k_D)e_k \\
 * &+ (-k_P(2\tau + T) - k_I\tau - 2k_D)e_{k-1}) \\
 * &+ (k_P\tau + k_D)e_{k-2} \\
 * &+ (2\tau + T)u_{k-1} \\
 * &- \tau u_{k-2} \\
 * u_k &= u_k / a_0
 * \end{aligned}
 * \f]
 * 
 * Tustin:
 * \f[
 * \begin{aligned}
 * a_0 &= 2\tau + T \\
 * u_k &= (k_P(2\tau + T) + \frac{1}{2} k_I(2\tau + T) + 2k_D)e_k \\
 * &+ (-4k_P\tau + \frac{1}{2} k_I(-2\tau + T) - 4k_D)e_{k-1} \\
 * &+ (k_P(2\tau + T) + 2k_D)u_{k-2} \\
 * &+ (4\tau)u_{k-1} \\
 * &- (2\tau - T)u_{k-2} \\
 * u_k &= u_k / a_0
 * \end{aligned}
 * \f]
 * 
 * WARNING: this routine does NOT check whether or not the controller was previously initialized. It also
 * does NOT take measurement compensation into consideration.
 * 
 * @param[in]   pid     PID controller data structure to update.
 * @param[in]   r_k     Reference signal at the current time step.
 * @param[in]   y_k     Measurement (after compensation) at current time step.
 * 
 * @return  Controller output.
 */
float
ctr_pid_update(ctr_pid_t* const pid, float r_k, float y_k);


// ====================================================================================================
// 2. State Space Representation
// ====================================================================================================

/**
 * @brief   Initializes a state space LTI model.
 * 
 * WARNING: this routine is written to provide somewhat of a MATLAB compatibility but does NOT initialize
 * the complete data structure. The state vector will be initialized until it's needed.
 * 
 * @param[in]       A               System matrix.
 * @param[in]       B               System input/actuator matrix.
 * @param[in]       C               System output/sensor matrix.
 * @param[in]       D               System feedforward terms.
 * @param[in]       sample_time     Sampling period, in case of a discrete time system.
 * @param[in, out]  sys             LTI state space system data structure.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
ctr_ss_lti(matf32_t* A, matf32_t* B, matf32_t* C, matf32_t* D, float sample_time, ctr_sys_lti_t* const sys);


/**
 * @brief   Discretizes a continuous time state space LTI system model, to get the following form:
 * 
 * \f[
 * \begin{aligned}
 * \textbf{x}_{k+1} &= \textbf{Ax} + \textbf{Bu} \\
 * \textbf{y} &= \textbf{Cx} + \textbf{Du} \\
 * \end{aligned} 
 * \f]
 *
 * WARNING: this routine overwrites the original continuous time system. This also does NOT
 * work for discrete time systems (as it's meant for continuous time systems).
 *
 * @param[in, out]  sys             Continuous time LTI system data structure.
 * @param[in]       sample_time     Sampling period.
 * @param[in]       method          Discretization scheme.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed. 
 */
err_status_t
ctr_c2d(ctr_sys_lti_t* const sys, float sample_time, ctr_discretizations_t method);

/**
 * @brief   Initialices an instance of the ctr_sys_lti_t struct for lti linear systems.
 * 
 * @param[in,out]   sys             lti system struct instance
 * @param[in]       state           Points to matrix for the system state
 * @param[in]       A               Points to matrix A for the system
 * @param[in]       B               Points to matrix B for the system
 * @param[in]       C               Points to matrix C for the system
 * @param[in]       D               Points to matrix D for the system
 * @param[in]       sample_time     Float for the sample time of the system
 * 
 * @return Execution status
 */
err_status_t
ctr_sys_lti_init(ctr_sys_lti_t* const sys, matf32_t* const state, matf32_t* const A, matf32_t* const B, matf32_t* const C, matf32_t* const D, float sample_time);

/**
 * @brief   Initializes an instance of the ctr_sys_nonlin_t struct for nonlinear systems.
 * 
 * @param[in,out]   sys             Nonlinear system struct instance
 * @param[in]       state           Points to matrix for the system state
 * @param[in]       input_dim       Number of inputs of the system
 * @param[in]       output_dim      Number of outputs of the system
 * @param[in]       dynamics        Points to function representing the dynamics of the system: output_matrix, xss, uss  
 * @param[in]       outputs         Points to function representing the outputs of the system: output_matrix, xss, uss
 * @param[in]       sample_time     Float for the sample time of the system
 * 
 * @return Execution status
 */
err_status_t
ctr_sys_nonlin_init(ctr_sys_nonlin_t* const sys, matf32_t* const state, uint16_t input_dim, uint16_t output_dim, err_status_t (*dynamics)(matf32_t* const, const matf32_t*, const matf32_t*), err_status_t (*outputs)(matf32_t* const, const matf32_t*, const matf32_t*), float sample_time);


/**
 * @brief   Linearizes a nonlinear system: input a nonlinear system, outputs a linear version, using the corresponding structs: ctr_sys_nonlin_t and ctr_sys_lti_t
 * 
 * @param[in]       src_sys         Points to nonlinear system struct instance
 * @param[in,out]   dst_sys         Points to lti system struct instance
 * @param[in]       xss             xss for the operation point (xss,uss)
 * @param[in]       uss             uss for the operation point (xss,uss)
 * @param[in]       delta           Diferences to divide by for the linearization
 * 
 * @return Execution status
 */
err_status_t
ctr_linloc(ctr_sys_nonlin_t* const src_sys, ctr_sys_lti_t* const dst_sys, const matf32_t* const xss, const matf32_t* const uss, float delta);



// ----------------------------------------------------------------------------------------------------
// 2.1. Linear State Space Controllers
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Updates/computes the linear state feedback controller u = -K * (x - xss) + uss.
 *
 * @param[in, out]  u       Controller output.
 * @param[in]       K       Gain matrix.
 * @param[in]       x       State vector.
 * @param[in]       xss     Operation point (desired state).
 * @param[in]       uss     Feedforward input to reach the desired state.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
ctr_linear_state_feedback(matf32_t* const u, const matf32_t* K, const matf32_t* x, const matf32_t* xss, const matf32_t* uss);


/**
 * @brief   Updates the step for a nonlinear system with either forward euler or runge-kutta-4 methods.
 * 
 * Forward Euler: \f[ x_{k+1} = x_k + f(x_k) \Delta t \f]
 * 
 * Runge-Kutta4: \f[ x_{k+1} = x_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4) \f]
 * 
 * 
 * @param[in]   sys     Points to the nonlinear system to work with
 * @param[in]   x_k     Points to the current state
 * @param[in]   x_k_1   Points to the next state to save it
 * @param[in]   u_k     Points to system input
 * @param[in]   delta   Float for the time step
 * @param[in]   method  Method to work with: Forward Euler or Runge-Kutta-4
 * 
 * @return Execution status
 */
err_status_t
ctr_sys_nonlin_simulate(ctr_sys_nonlin_t* sys, const matf32_t* const x_k, matf32_t* const x_k_1, const matf32_t* const u_k, float delta, ctr_discretizations_t method);


// ====================================================================================================
// 3. Kalman Filter (Linear time-varying, discrete time)
// ====================================================================================================

/**
 * @brief   Initializes a linear, time-varying Kalman filter structure.
 * 
 * @param[in, out]  kf      Kalman filter data structure.
 * @param[in]       sys     LTI system model.
 * @param[in]       F       Coupling matrix of the process noise.
 * @param[in]       Qw      Process noise covariance matrix.
 * @param[in]       Qv      Measurement noise covariance matrix.
 * @param[in]       xhat    State estimate.
 * @param[in]       P       Estimation covariance matrix.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_ARGUMENT_ERROR :   LTI system model is not discrete time.
 */
err_status_t
ctr_kalman_init(ctr_kalman_t* const kf, ctr_sys_lti_t* const sys, matf32_t* F, matf32_t* Qw, matf32_t* Qv, matf32_t* const xhat, matf32_t* const P);

/**
 * @brief Computes the prediction step for the Kalman Filter, based on the following
 * equations:
 * 
 * \f[
 * \begin{aligned}
 * \hat{\textbf{x}}_{k+1|k} &= \textbf{F} \hat{\textbf{x}}_k + \textbf{Gu}_k, \\
 * \hat{\textbf{P}}_{k+1|k} &= \textbf{F} \hat{\textbf{P}}_{k|k} \textbf{F}^\top + \hat{\textbf{V}}
 * \end{aligned}
 * \f]
 * 
 * @param[in,out]   kf      Kalman filter data structure.
 * @param[in]       inputs  Estimate from the input of the system.
 * 
 * @return Execution status
 *          MATH_SUCCESS : Operation successful.
 *          MATH_SIZE_MISMATCH : Matrix size check failed.
 */
err_status_t
ctr_kalman_predict(ctr_kalman_t* const kf, const matf32_t* inputs);

/**
 * @brief Computes the correction step for the Kalman Filter, based on the following
 * equations:
 * 
 * \f[
 * \begin{aligned}
 * \boldsymbol{\nu}_{k+1} &= \textbf{z}_{k+1} - \textbf{H} \hat{\textbf{x}}_{k+1|k}, \\
 * \hat{\textbf{x}}_{k+1|k+1} &= \hat{\textbf{x}}_{k+1|k} + \textbf{K}_{k+1} \boldsymbol{\nu}_{k+1}, \\
 * \hat{\textbf{P}}_{k+1|k+1} &= \hat{\textbf{P}}_{k+1|k} - \textbf{K}_{k+1} \textbf{H} \hat{\textbf{P}}_{k+1|k}
 * \end{aligned}
 * \f]
 * 
 * @param[in,out]   kf              Kalman filter data structure.
 * @param[in]       measurements    Estimate from the output of the system.
 * 
 * @return Execution status
 *          MATH_SUCCESS : Operation successful.
 *          MATH_SIZE_MISMATCH : Matrix size check failed.
 */
err_status_t
ctr_kalman_correct(ctr_kalman_t* const kf, const matf32_t* measurements);


/**
 * @brief Executes both the prediction and correction step by calling the routines
 * ctr_kalman_predict and ctr_kalman_correct.
 * 
 * @param[in,out]   kf              Kalman filter data structure.
 * @param[in]       inputs          Estimate from the input of the system.
 * @param[in]       measurements    Estimate from the output of the system.
 * 
 * @return Execution status
 *          MATH_SUCCESS : Operation successful.
 *          MATH_SIZE_MISMATCH : Matrix size check failed.
 */
static inline err_status_t
ctr_kalman_update(ctr_kalman_t* const kf, const matf32_t* inputs, const matf32_t* measurements)
{
    ctr_kalman_predict(kf, inputs);
    return ctr_kalman_correct(kf, measurements);
}

/**
 * @brief Returns the estimate of the kalman system, by reading into the struct's data.
 * 
 * @param[in]       kf          Kalman filter data structure.
 * @param[in,out]   estimate    Estimate from the Kalman filter.
 * 
 * @return None.
 */
static inline void
ctr_kalman_get_estimate(ctr_kalman_t* const kf, float* const estimate)
{
    memcpy(estimate, kf->xhat->p_data, kf->xhat->num_rows * sizeof(float));
}

// ====================================================================================================
// 4. Model Predictive Control (MPC)
// ====================================================================================================

/**
 * @brief   Initializes an unconstrained, shooting-based, LTI, MPC struct.
 * All needed data must be declared and initialized beforehand, including matrices,
 * arrays of matrices, along with the quadratic problem and MPC structs.
 * 
 * For an unconstrained MPC, set Ain and bin equal to NULL. Otherwise, declare, initialize
 * and set the corresponding matrices.
 * 
 * @param[in,out]   mpc                 Points to the corresponding MPC struct.
 * @param[in]       x0                  Points to the initial operation point matrix
 * @param[in]       Q                   Points to the quadratic term matrix for the cost function of the QP
 * @param[in]       c                   Points to the linear term matrix for the cost function of the QP
 * @param[in]       Qhat                Points to the penalization matrix Q
 * @param[in]       Rhat                Points to the penalization matrix R
 * @param[in]       C                   Points to the convolution matrix C
 * @param[in]       horizon_length      Horizon length (samples).
 * @param[in]       M                   Points to the horizon length matrix M
 * @param[in]       u_k                 Points to the input trajectory matrix
 * @param[in]       x_k                 Points to the state trajectory matrix
 * 
 * @return Execution status.
 */
err_status_t
ctr_mpc_lti_init(ctr_mpc_lti_shooting_t* mpc, quadprog_t* qp, ctr_sys_lti_t* sys, matf32_t* const u_k, matf32_t* const x_k, matf32_t* const Ain,
    matf32_t* const bin, matf32_t* const mpc_Q, matf32_t* const mpc_R, matf32_t* const mpc_S, matf32_t** const mpc_C, matf32_t** const mpc_M, float N, bool state_constraints);


/**
 * @brief   Generates the prediction matrix \f$ \textbf{M}_x \f$ for the MPC, generated as an array of matrices, such that:
 * 
 * \f[ \textbf{M}_x = [\textbf{A}, \textbf{A}^2, ... , \textbf{A}^N] \f]
 * 
 * according to the definition of the matrix in Kouvaritakis, Model Predictive Control.
 * 
 * In terms of code, that means that \f$ \textbf{M}_x \f$ is an array of pointers, and each index in it
 * contains a different matrix, that is: M = [&M1, &M2, ... , &MN], where each matrix M corresponds
 * to a power of A. Both the array of pointers and the individual matrices must be declared (and the matrices
 * initialized with matf32_init) before calling this routine. The content of each matrix is computed and set
 * with this routine upon execution, except for \f$ \textbf{A} \f$ which should be already defined with the values of the
 * LTI system to be optimized.
 * 
 * @param[in,out]   mpc         Points to the MPC struct.
 * @param[in,out]   mpc_M_data  Points to the data arrays for the submatrices of M
 * 
 * @return Execution status.
 *              MATH_SUCCESS : Operation completed succesfully.
 */
err_status_t
ctr_mpc_set_M(ctr_mpc_lti_shooting_t* mpc, float** mpc_M_data);


/**
 * @brief   Generates the convolution matrix \f$ \textbf{M}_c \f$ for the MPc, generated as an array of matrices,
 * such that inside the MPC routines it's operated as a matrix of matrices. In simple terms, that is:
 * 
 * \f[ \textbf{M}_c =
 * \begin{bmatrix} 
 *  \textbf{B}                      &   \textbf{0}        & \cdots & \textbf{0} \\
 *  \boldsymbol{A} \textbf{B}       &   \textbf{B}        & \cdots & \textbf{0} \\
 *  \vdots                          &   \vdots            & \ddots & \vdots     \\
 *  \boldsymbol{A} ^{N-1}\textbf{B} & A ^{N-2} \textbf{B} & \cdots & \textbf{B}
 * \end{bmatrix} \f]
 * 
 * Similar to ctr_mpc_set_M, the array of pointers must be declared before calling this routine
 * (ctr_mpc_set_C) and the necessary matrices must be declared and initialized before as well.
 * That is, define and initialize matf32_t matrices for all matrices in the first column of
 * \f$ \textbf{M}_c \f$, as well as an additional null matrix to match the dimensiones in the
 * operations that will be done with other routines. Save those matrices in their own array
 * (mpc_C_stack) in the same order as in the first column of the convolution matrix (with the
 * null matrix at the end of the array). This routine computes all matrices individually,
 * except \f$ \textbf{B} \f$ which should be defined with the LTI system to be optimized.
 * 
 * @param[in,out]   mpc                 Points to the MPC struct       
 * @param[in,out]   mpc_C_stack         Points to the stack of matrices needed to build C
 * @param[in,out]   mpc_C_stack_data    Points to the array of pointers for C
 * 
 * @return Execution status.
 *              MATH_SUCCESS : Operation completed succesfully.
 */
err_status_t
ctr_mpc_set_C(ctr_mpc_lti_shooting_t* mpc, matf32_t** mpc_C_stack, float** mpc_C_stack_data);


/**
 * @brief   Generates the quadratic term matrix \f$ \textbf{Q} \f$ for the quadratic program associated to the
 * MPC, following the equation:
 * 
 * \f[ \textbf{Q} = 2 \cdot (\textbf{M}^\top_c \tilde{\textbf{Q}} \textbf{M}_c + \tilde{\textbf{R}}) \f]
 * 
 * where \f$ \tilde{\textbf{Q}} \f$ and \f$ \tilde{\textbf{R}} \f$ correspond the penalization matrices
 * as implemented in the Linear-Quadratic Regulator (LQR), while \f$ \textbf{M}_c \f$ is one of the prediction
 * matrices of the MPC (see ctr_mpc_set_M). 
 * 
 * The above equation was adapted from the definition of the MPC as explained in Kouvaritakis,
 * Model Predictive Control. 
 * 
 * @param[in]       mpc     Points to the MPC struct.
 * @param[in,out]   qp_Q    Points to the Q matrix for the quadratic program.
 * 
 * @return Execution status.
 *              MATH_SUCCESS : Operation completed succesfully.
 */
err_status_t
ctr_mpc_set_qpQ(ctr_mpc_lti_shooting_t* mpc, matf32_t* qp_Q);

/**
 * @brief   Generates the linear term matrix \f$ c \f$ for the quadratic program of the associated to the
 * MPC, following the equation:
 * 
 * \f[ \textbf{c} = 2 \textbf{F} \mathrm{\textbf{x}}_k \f]
 * 
 * \f[ \textbf{F} = \textbf{M}^\top_c \tilde{\textbf{Q}} \textbf{M}_x \f]
 * 
 * where \f$ \textbf{x}_k \f$ is the state vector of the LTI system, \f$ \textbf{M}^\top_c \f$ and
 * \f$ \textbf{M}_x \f$ are the prediction matrices of the MPC, and \f$ \tilde{\textbf{Q}} \f$ is the
 * penalization matrix as used in the Linear-Quadratic Regulator (LQR).
 * 
 * The equations above were adapted from the definition of the MPC in Kouvaritakis,
 * Model Predictive Control.  
 * 
 * @param[in]       mpc     Points to the MPC struct.
 * @param[in,out]   qp_c    Points to the c matrix for the quadratic program.
 * 
 * @return  Execution status.
 *              MATH_SUCESS : Operation completed succesfully.
 */
err_status_t
ctr_mpc_set_qpc(ctr_mpc_lti_shooting_t* mpc, matf32_t* qp_c);

/**
 * @brief   Calculates next state trajectory of the MPC, executing the following steps:
 * 
 * 1. Solving the quadratic program (QP) associated to the MPC. If the QP is unconstrained,
 * a simple solution with linsolve is calculated. Otherwise, if there are constraints (whether
 * of equality or inequality), then the active-set method of quadprog is used (see quadprog_sqp).
 * 
 * 2. Updating the LTI system, by using the result of the solution of the QP. In this case, the
 * QP is used to recalculate the input vector, generating N values according as a projection to
 * the entire horizon length of the MPC, so only the first value of the new input vector is used
 * to compute the next state of the LTI system, and this is done through the definition of the system:
 * 
 * \f[ \textbf{x}_{k+1} = \textbf{Ax}_k + \textbf{Bu} \f]
 * 
 * @param[in,out]   mpc     Points to the corresponding MPC struct.
 * @param[in]       x_k     Points to the current state trajectory
 * @param[in]       u_k     Points to the current input trajectory
 * 
 * @return Execution status
 *              MATH_SUCCESS :
 *              MATH_SIZE_MISMATCH : 
 */
err_status_t
ctr_mpc_update(ctr_mpc_lti_shooting_t* mpc, matf32_t* const qp_Q, matf32_t* const qp_c, matf32_t* const x_k, matf32_t* const u_k);

/**
 * @brief   Generates the inequality matrix and vector according to whether only the input or both the input
 * and state are constrained, according to the following definitions:
 * 
 * 1. If only the input vector is to be constrained:
 * 
 * \f[
 * \begin{bmatrix}
 * lb \\
 * \vdots \\
 * lb
 * \end{bmatrix}
 * \le
 * \textbf{u}_k
 * \le
 * \begin{bmatrix}
 * ub \\
 * \vdots \\
 * ub
 * \end{bmatrix},
 * \f]
 * 
 * 2. If the state is to be constrained as well: in this case, this is done through the restrictions of the
 * input vector, given that this is for a shooting-based MPC. That is, the state constraints are written and
 * computed as follows:
 * 
 * \f[
 * \textbf{x}_{lb} - \textbf{Bu}_1
 * \le
 * \textbf{Au}_k
 * \le
 * \textbf{x}_{ub} - \textbf{B}\textbf{u}_1
 * \f]
 * 
 * In both cases, \f$ lb \f$ and \f$ ub \f$ refer to the lower and upper constraints (numerical values), while
 * all the matrices in the above equations correspond to those of the LTI system.
 * 
 * @param[in,out]   mpc                 Points to the corresponding MPC struct
 * @param[in]       ub                  Upper bound
 * @param[in]       lb                  Lower bound
 * @param[in]       state_constraints   Boolean: 0=only input constraints, 1=include state constraints as input constraints
 * 
 * @return Execution status      
 */
err_status_t
ctr_mpc_set_constraints(ctr_mpc_lti_shooting_t* mpc, float ub, float lb);


// ====================================================================================================
// 5. Utility functions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 5.1. Printing functions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Prints the ctr_pid_t struct data in formatted text
 * 
 * @param[in]   p_pid   Pointer to the pid to print
 * 
 * @return None
 */
void 
ctr_pid_print(ctr_pid_t* const p_pid);

/**
 * @brief   Prints the elements of ctr_discretizations_t
 * 
 * @param[in]   pid_alg     Enumerate list of discretization algorithms
 * 
 * @return None
 */
void
ctr_discretizations_print(ctr_discretizations_t pid_alg);

/**
 * @brief   Prints the ctr_sys_lti_t struct in formatted text
 * 
 * @param[in]   p_sys_lti   Pointer to the struct to print
 * 
 * @return None
 */
void 
ctr_sys_lti_print(ctr_sys_lti_t* p_sys_lti);

/**
 * @brief   Prints the elements of ctr_kalman_t
 * 
 * @param[in]   p_kalman    Pointer to the struct to print
 * 
 * @return None
 */
void
ctr_kalman_print(ctr_kalman_t* p_kalman);

// ====================================================================================================
// TODO Notes
// ====================================================================================================


// TODO:
// 1. Nonlinear system linearization
// 2. Nonlinear system discretization
// 3. Extended Kalman Filter
// 4. Linear time-varying LQR
// 5. Linear MPC

#ifdef __cplusplus
}
#endif

#endif /* ROBOTAT_CONTROL_H_ */

/** @} */