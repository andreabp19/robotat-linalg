/**
 * @file robotat_control.h
 * @author Miguel Zea (mezea@uvg.edu.gt)
 * @brief 
 * @version 0.1
 * @date 2021-08-12
 * 
 * Last modified: 15 Sep 2025
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
//#include "robotat_linalg.h"
#include "matf32.h"
#include "linsolve.h"

#ifdef __cplusplus
extern "C" {
#endif


// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================
// NOTE: these should not be manipulated directly, use the init, setter and getter routines instead.

// POSSIBLE TODO:
// LQI
// MPC
// Funciones con espacio de estados:
//      - ej. función para un ciclo de simulación, con forward euler y runge kutta para propagar un step hacia adelante en la simulacion del estado siguiente,
//              usando el valor del estado actual y las condiciones iniciales.

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
 * @brief   PID controller data structure.
 */
typedef struct
{
    float kp;                       /**< Proportional gain. */
    float ki;                       /**< Integral gain. */
    float kd;                       /**< Derivative gain. */
    float e_k_1;                    /**< Last error. */
    float u_k_1;                    /**< Last controller output. */
    float i_min;                    /**< Lower integrator saturation threshold. */
    float i_max;                    /**< Upper integrator saturation threshold. */
    float tau;                      /**< Time constant of the derivative HPF. */
    float dt;                       /**< Sampling period. */
    ctr_discretizations_t pid_alg;  /**< Specifies the discretization scheme to be used. */
} ctr_pid_t;


/**
 * @brief   State space LTI system data structure.
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
    err_status_t (*dynamics)(matf32_t* const, const matf32_t*, const matf32_t*);    /** System dynamics. */
    err_status_t (*outputs)(matf32_t* const, const matf32_t*, const matf32_t*);     /** System outputs. */
    float dt;                                                                       /**< Sampling period (for discrete time systems). */
    bool is_continuous;                                                             /**< System time domain specification. */
} ctr_sys_nonlin_t;


/**
 * @brief   Linear time-varying Kalman filter data structure.
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
// Tested => Works
static inline void
ctr_pid_set_gains(ctr_pid_t* const pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}


/**
 * @brief   Updates a previously initialized PID controller.
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
 * @brief   Discretizes a continuous time LTI system model.
 *
 * WARNING: this routine overwrites the original continuous time system. This also does NOT
 * work for discrete time systems.
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
 * @brief   Initialices an instance of the ctr_sys_lti_t struct for lti linear systems
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
 * @brief   Initializes an instance of the ctr_sys_nonlin_t struct for nonlinear systems
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


err_status_t
ctr_kalman_predict(ctr_kalman_t* const kf, const matf32_t* inputs);


err_status_t
ctr_kalman_correct(ctr_kalman_t* const kf, const matf32_t* measurements);


static inline err_status_t
ctr_kalman_update(ctr_kalman_t* const kf, const matf32_t* inputs, const matf32_t* measurements)
{
    ctr_kalman_predict(kf, inputs);
    return ctr_kalman_correct(kf, measurements);
}


static inline void
ctr_kalman_get_estimate(ctr_kalman_t* const kf, float* const estimate)
{
    memcpy(estimate, kf->xhat->p_data, kf->xhat->num_rows * sizeof(float));
}


// ====================================================================================================
// 4. Utility functions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Printing functions
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
ctr_discretizations_print(ctr_discretizations_t pid_ag);

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


//void
//kalman_predict(ctr_kalman_t* const kf, float* const inputs);
//err_status_t
//kalman_correct(ctr_kalman_t* const kf, float* const measurements);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTAT_CONTROL_H_ */
