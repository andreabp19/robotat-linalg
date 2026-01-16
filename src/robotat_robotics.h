
/**
 * @defgroup Robotat-Robotics
 * @{
 * @author Andrea Pineda
 * @brief Robotics algorithms, based on MATLAB Robotics 
 * Toolbox of Peter Corke. Includes routines for homogeneous
 * transformations, rotationes matrices and quaternions.
 * @date created 19 jul. 2025, last modified 13 jan. 2026
*/

#ifndef ROBOTAT_ROBOTICS_H_
#define ROBOTAT_ROBOTICS_H_

/**
 * Dependencies
 */

#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <stdarg.h>

#define _USE_MATH_DEFINES       // To be able to use constants such as M_PI
#include <math.h>               // For sin() and cos()

// M_PI is included in math.h but if it gives error of being "undefined", defining it fixes it.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "matf32.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================================
// 0. Data structures, enums and type definitions
// ====================================================================================================

/**
 * @brief   List of possible status or errors
 */
typedef enum
{
    ROB_MATH_SUCCESS,   /**< Success in the operation */
    ROT_SIZE_MISMATCH,  /**< Incorrect dimensions for rotation matrix */
    TR_SIZE_MISMATCH,   /**< Incorrect dimensions for homogeneous transformation matrix */
    VEC_SIZE_MISMATCH,  /**< Incorrect dimensions for coordinates vector */
    TRANSFORM_FRAMES_MISMATCH, /**< Reference frames are not compatible to transform */
    NULL_QUATERNION_ERR /**< Null quaternion detected for operation with division */
} rob_status_t;


/**
 * @brief   Enumerates list for reference frames IDs. More frames can be added if needed.
 */
typedef enum
{
    FRAME_A,  /**< Reference frame A */
    FRAME_B,  /**< Reference frame B */
    FRAME_C,  /**< Reference frame C */
    FRAME_O   /**< Global reference frame O */
} rob_frame_id_t;


/**
 * @brief   Enumerates list of id for possible rotation sequences to implement.
 */
typedef enum
{
    XYX, /**< Euler angles rotation sequence: X-axis -> Y-axis -> X-axis */
    XZX, /**< Euler angles rotation sequence: X-axis -> Z-axis -> X-axis */
    YXY, /**< Euler angles rotation sequence: Y-axis -> X-axis -> Y-axis */
    YZY, /**< Euler angles rotation sequence: Y-axis -> Z-axis -> Y-axis */
    ZXZ, /**< Euler angles rotation sequence: Z-axis -> X-axis -> Z-axis */
    ZYZ, /**< Euler angles rotation sequence: Z-axis -> Y-axis -> Z-axis */
    XYZ, /**< Cardan angles rotation sequence: X-axis -> Y-axis -> Z-axis */
    XZY, /**< Cardan angles rotation sequence: X-axis -> Z-axis -> Y-axis */
    YZX, /**< Cardan angles rotation sequence: Y-axis -> Z-axis -> X-axis */
    YXZ, /**< Cardan angles rotation sequence: Y-axis -> X-axis -> Z-axis */
    ZXY, /**< Cardan angles rotation sequence: Z-axis -> X-axis -> Y-axis */
    ZYX  /**< Cardan angles rotation sequence: Z-axis -> Y-axis -> X-axis */
} rob_angle_sequences_t;


/**
 * @brief   Reference frame struct for robot pose calculations.
 * Includes the homogeneous transformation matrix, the corresponding
 * rotation matrix and coordinates vector, the labels or IDs for the 
 * frame and the next frame to which the transformation will lead, and
 * the angle units (radians or degrees) to be used.
 * 
 * An homogeneous transformation matrix is defined as follows:
 * 
 * \f[
 * \begin{bmatrix}
 * ^A\textbf{R}_B & \textbf{t} \\
 * \textbf{0} & 1 
 * \end{bmatrix},
 * \f]
 * 
 * where \f$^A\textbf{R}_B\f$ is the rotation matrix \f$(3 \times 3)\f$,
 * \f$\textbf{t}\f$ is the coordinates vector \f$(3 \times 1)\f$,
 * and \f$\textbf{0}\f$ is a zeros vector \f$(1 \times 3)\f$.
 */
typedef struct 
{   
    matf32_t* p_T;                /**< Points to homogeneous transformation matrix */
    matf32_t* p_R;                /**< Points to rotation matrix */
    matf32_t* p_v;                /**< Points to coordinates vector */
    rob_frame_id_t ref_frame;     /**< Label or ID of this reference frame */
    rob_frame_id_t dst_frame;     /**< Label or ID of the frame to which the transform will lead */
    bool angle_units;             /**< Angle units to be used, choose between RADIANS or DEGREES */
} rob_frame_t;


/**
 * @brief Struct for reference points associated with a specific reference frame.
 * Contains the coordinates vector of the points and the label or ID of the associated frame.
 * 
 * The coordinates vector for the reference point is defined as follows:
 * 
 * \f[\f]
 * ^A\textbf{p} = \begin{bmatrix}
 * \mathrm{x} \\
 * \mathrm{y} \\
 * \mathrm{z} \\
 * 1
 * \end{bmatrix}
 * \f]
 * 
 * with \f$ \mathrm{x} \f$, \f$ \mathrm{y} \f$ and \f$ \mathrm{z} \f$ being the xyz coordinates,
 * and 1 being an arbitrary value to match the dimensions of the homogeneous transformation
 * matrix.
 */
typedef struct
{
    const matf32_t* p_v;        /**< Points to coordinates vector for this reference point */
    rob_frame_id_t ref_frame;   /**< Label or ID for the associated reference frame */
} rob_point_t;


/**
 * @brief   Quaternion struct to define the scalars of each term of the quaternion.
 * 
 * A quaternion is a complex number defined as follows:
 * 
 * \f[
 * \mathring{q} = s + \textbf{v}
 * \f]
 * 
 * where \f$ \textbf{v} = v_1 \textbf{i} + v_2 \textbf{j} + v_3 \textbf{k} \f$,
 * with \f$ v_1 \f$, \f$ v_2 \f$ and \f$ v_3 \f$ being scalars accompanied by their
 * respective unitary vectors.
 * 
 * Depending on the notation, in code quaternions are identified as \f$ \mathrm{q} \f$
 * (without the ring or circle above).
 */
typedef struct
{
    float* p_s;     /**< Real number s of the quaternion */
    float* p_i;     /**< Imaginary number i of the quaternion */
    float* p_j;     /**< Imaginary number j of the quaternion */
    float* p_k;     /**< Imaginary number k of the quaternion */
} rob_quat_t;



// ====================================================================================================
// Function prototypes
// ====================================================================================================
// ====================================================================================================
// 1. Reference frame and point initializations
// ====================================================================================================

/**
 * @brief   Constructor for the reference frame data structure, defines 4x4 dimensiones, sets the rotation matrix,
 * coordinates vector and creates the homogeneous transformation matrix.
 * 
 * @param[in]   p_T             Transformation matrix for the reference frame.
 * @param[in]   p_R             Rotation matrix.
 * @param[in]   p_v             Coordinates vector.
 * @param[in]   ref_frame       Enumerate tag for the current reference frame for the transformation.
 * @param[in]   dst_frame       Enumerate tag for the destination frame for the transformation.
 * @param[in]   angle_units     Enumerate tag for the units of the angles.
 * 
 * @return Execution status
 *              
 */
rob_status_t
rob_frame_init(rob_frame_t* p_F, matf32_t* p_T, matf32_t* p_R, matf32_t* p_v, rob_frame_id_t ref_frame, rob_frame_id_t dst_frame, bool angle_units);


/**
 * @brief   Constructor for the reference frame data structure, defines 4x4 dimensiones, sets the rotation matrix,
 * coordinates vector and creates the homogeneous transformation matrix.
 * 
 * @param[in]   p_p         Points to the point struct to use.
 * @param[in]   p_data      Points to data float to use
 * @param[in]   ref_frame   Enum tag to indiate the associated reference frame
 * 
 * @return None
 */
void
rob_refpoint_init(rob_point_t* const p_p, matf32_t* p_v, float* p_data, rob_frame_id_t ref_frame);



// ====================================================================================================
// 2. Homogeneous transformation matrix operations
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 2.1. Set translation
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Set coordinates vector in the homogeneous transformation matrix
 * 
 * @param[in,out]   p_F     Points to the reference frame struct
 * @param[in]       p_v     Points to the coordinates vector to be implemented
 * 
 * @return  rob_status_t
 */
rob_status_t
rob_transl(matf32_t* p_v, rob_frame_t* p_F);


// ----------------------------------------------------------------------------------------------------
// 2.2. Generate and set rotation matrices
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates a rotation matrix in the x axis, defined as follows:
 * 
 * \f[
 * \textbf{R}_x = \begin{bmatrix}
 * 1 &     0       &      0 \\
 * 0 & cos(\theta) & -sin(\theta) \\
 * 0 & sin(\theta) & cos(\theta)
 * \end{bmatrix}
 * \f]
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix.
 * @param[in]       p_theta     Points to the value of degrees by which to rotate the frame.
 * 
 * @return err_status_t
 */
rob_status_t
rob_rotx(matf32_t* p_R, float theta, bool angle_units);


/**
 * @brief   Generates rotation in the y axis, defined as follows:
 * 
 * \f[
 * \textbf{R}_y = \begin{bmatrix}
 * cos(\theta)  & 0 & sin(\theta) \\
 *     0        & 1 &     0      \\
 * -sin(\theta) & 0 & cos(\theta)
 * \end{bmatrix}
 * \f]
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix.
 * @param[in]       theta       Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
rob_status_t
rob_roty(matf32_t* p_R, float theta, bool angle_units);


/**
 * @brief   Generates rotation in the z axis, defined as follows:
 * 
 * \f[
 * \textbf{R}_z = \begin{bmatrix}
 * cos(\theta)  & -sin(\theta) & 0 \\
 * sin(\theta)  &  cos(\theta) & 0 \\
 *     0        &      0       & 1
 * \end{bmatrix}
 * \f]
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix
 * @param[in]       theta       Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
rob_status_t
rob_rotz(matf32_t* p_R, float theta, bool angle_units);


/**
 * @brief   Generates a rotation matrix around the x axis, using rob_rotx
 * and assigns it to an homogeneous transformation matrix T.
 * 
 * @param[in,out]   p_F       Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initiatializing the frame)
 * 
 * @return None
 */
void
rob_trotx(rob_frame_t* p_F, float theta, bool angle_units);


/**
 * @brief  Generates a rotation matrix around the y axis, using rob_rotx
 * and assigns it to an homogeneous transformation matrix T.
 * 
 * @param[in,out]   p_F       Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initializing the frame)
 * 
 * @return None
 */
void
rob_troty(rob_frame_t* p_F, float theta, bool angle_units);


/**
 * @brief   Generates a rotation matrix around the z axis, using rob_rotx
 * and assigns it to an homogeneous transformation matrix T.
 * 
 * @param[in,out]   p_F       Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initializing the frame)
 * 
 * @return None
 */
void
rob_trotz(rob_frame_t* p_F, float theta, bool angle_units);


// ----------------------------------------------------------------------------------------------------
// 2.3. Applying transformations and rotation sequences
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Applies the homogeneous transformation to calculate the pose. The operation is defined as follows:
 * 
 * \f[
 * ^Ap = ^AT_B \cdot ^Bp
 * \f]
 *  
 * As the operation has a fixed size, the most efficient way is to operate element-by-element.
 * As such, it's implemented manually as follows:
 * 
 * \f[
 * ^Ap_{1,1} = T_{1,1}\cdot {^Bp_{1,1}} + T_{1,2}\cdot {^Bp_{2,1}} + T_{1,3}\cdot {^Bp_{3,1}} + T_{1,4}\cdot {^Bp_{4,1}} 
 * \f]
 * 
 * \f[
 * ^Ap_{2,1} = T_{2,1}\cdot {^Bp_{1,1}} + T_{2,2}\cdot {^Bp_{2,1}} + T_{2,3}\cdot {^Bp_{3,1}} + T_{2,4}\cdot {^Bp_{4,1}} 
 * \f]
 * 
 * \f[
 * ^Ap_{3,1} = T_{3,1}\cdot {^Bp_{1,1}} + T_{3,2}\cdot {^Bp_{2,1}} + T_{3,3}\cdot {^Bp_{3,1}} + T_{3,4}\cdot {^Bp_{4,1}}
 * \f]
 * 
 * \f[
 * ^Ap_{4,1} = T_{4,1}\cdot {^Bp_{1,1}} + T_{4,2}\cdot {^Bp_{2,1}} + T_{4,3}\cdot {^Bp_{3,1}} + T_{4,4}\cdot {^Bp_{4,1}}
 * \f]
 * 
 * @param[in]   p_F     Pointer to reference frame struct to work with.
 * @param[in]   p_srcp  Coordinates vector with reference to original frame.
 * @param[out]  p_dstp  Coordinates vector to the new frame of reference.
 * 
 * @return None
 */
void
rob_apply_transform(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp);


/**
 * @brief   Applies inverse of the homogeneous transformation of the pose.
 * 
 * \f[
 * \begin{bmatrix}
 * R^T  & -R^T \cdot t \\
 * 0    &      1  
 * \end{bmatrix}
 * \f]
 *
 * @param[in]       p_F     Pointer to reference frame struct.
 * @param[in,out]   p_Tinv  Pointer to output matrix to save the inverse.     
 * 
 * @return None
 */
void
rob_inv_transform(rob_frame_t* const p_F, matf32_t* const p_Tinv);


/**
 * @brief   Apply an homogeneous transform based on Euler angles sequences: XYX, XZX,
 * YXY, YZY, ZXZ, ZYZ, or Cardan angles sequences: XYZ, XZY, YXZ, YZX, ZXY, XYZ.
 * All these sequences are available and to select one, it must be introduced in
 * the arguments of the function.
 * 
 * @param[in]   p_F         Points to reference frame struct to work with.  
 * @param[in]   p_srcp      Pointss to reference point for the frame to transform from.
 * @param[out]  p_dstp      Points to reference point with respect to the new frame.
 * @param[in]   euler_tag   Tag to indicate the rotation sequence to apply, for example: XYZ, ZYZ, etc.
 * 
 * @return None
 */
void
rob_apply_rot_sequence(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_angle_sequences_t angle_sequence, float phi, float theta, float psi, bool angle_units);



// ====================================================================================================
// 3. Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 3.1. Quaternion initialization
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Quaternion struct initialization
 * 
 * @param[in,out]   p_q     Pointer to quaternion struct
 * @param[in]   q_s     Real element of the quaternion
 * @param[in]   q_i     Imaginary element i of the quaternion
 * @param[in]   q_j     Imaginary element j of the quaternion
 * @param[in]   q_k     Imaginary element k of the quaternion
 * 
 * @return None
 */
void
rob_quat_init(rob_quat_t* p_q, float* q_s, float* q_i, float* q_j, float* q_k);


/**
 * @brief   Quaternion struct initialization
 * 
 * @param[in,out]   p_uq    Pointer to quaternion struct
 * @param[in]   q_s     Real element of the quaternion
 * @param[in]   q_i     Imaginary element i of the quaternion
 * @param[in]   q_j     Imaginary element j of the quaternion
 * @param[in]   q_k     Imaginary element k of the quaternion
 * 
 * @return None
 */
void
rob_unitquat_init(rob_quat_t* p_uq, float* q_s, float* q_i, float* q_j, float* q_k);


// ----------------------------------------------------------------------------------------------------
// 3.2. Quaternion operations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Add two quaternions
 * 
 * @param[in]   p_srcq1    Pointer to the first quaternion
 * @param[in]   p_srcq2    Pointer to the second quaternion
 * @param[out]  p_dstq     Pointer to the destination quaternion
 * 
 * @return  None
 */
void
rob_quat_add(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq);


/**
 * @brief   Substract two quaternions
 * 
 * @param[in]   p_srcq1    Pointer to the first quaternion
 * @param[in]   p_srcq2    Pointer to the second quaternion
 * @param[out]  p_dstq     Pointer to the destination quaternion
 * 
 * @return  None
 */
void
rob_quat_sub(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq);


/**
 * @brief   Multiplies a quaternion by a scalar
 * 
 * @param[in]   p_srcq  Pointer to the first quaternion
 * @param[in]   p_c     Float to multiply with
 * @param[out]  p_dstq  Pointer to the destination quaternion
 * 
 * @return  None
 */
void
rob_quat_scale(const rob_quat_t* p_srcq, const float p_c, rob_quat_t* p_dstq);


/**
 * @brief   Multiplies two quaternions (Hamilton product) such that, for two quaternions:
 * 
 * \f[ q_1 = a_1 + b_1i + c_1j + d_1k \f]
 * \f[ q_2 = a_2 + b_2i + c_2j + d_2k \f]
 * 
 * the multiplication is as follows for each component of the result quaternion:
 * \f[ (a_1 a_2 - b_1 b_2 - c_1 c_2 - d_1 d_2) \f]
 * \f[ + (a_1 b_2 + b_1 a_2 + c_1 d_2 - d_1 c_2)i \f]
 * \f[ + (a_1 c_2 - b_2 c_2 + c_1 a_2 + d_1 b_2)j \f]
 * \f[ + (a_1 d_2 + b_1 c_2 - c_1 b_2 + d_1 a_2)k \f]
 * 
 * 
 * @param[in]   p_srcq1    Pointer to the first quaternion
 * @param[in]   p_srcq2    Pointer to the second quaternion
 * @param[out]  p_dstq     Pointer to the destination quaternion
 * 
 * @return  None
 */
void
rob_quat_mul(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq);


/**
 * @brief   Calculates the conjugate of a quaternion
 * 
 * @param[in]   p_srcq  Pointer to a quaternion
 * @param[out]  p_dstq  Pointer to the quaternion struct to save the conjugate
 * 
 * @return None
 */
void
rob_quat_conj(const rob_quat_t* p_srcq, rob_quat_t* p_dstq);


/**
 * @brief   Calculates the norm of a quaternion
 * 
 * @param[in]   p_srcq  Pointer to a quaternion
 * @param[out]  q_norm  Float to save the norm of the quaternion
 * 
 * @return None
 */
void
rob_quat_norm(const rob_quat_t* p_srcq, float* q_norm);


/**
 * @brief   Calculates the inverse of a quaternion
 * 
 * @param[in]   p_srcq  Pointer to a quaternion
 * @param[out]  p_dstq  Pointer to save the inverse of the quaternion
 * 
 * @return None
 */
rob_status_t
rob_quat_inv(const rob_quat_t* p_srcq, rob_quat_t* p_dstq);


/**
 * @brief   Takes a quaternion and makes conversions to then apply it as an homogeneous transformation
 * 
 * @param[in]       p_srcq      Points to quaternion
 * @param[in]       p_F         Points to reference frame struct
 * @param[in]       p_srcp      Points to original reference point
 * @param[in,out]   p_dstp      Points to destination reference point
 * 
 * @return None
 */
void
rob_quat_apply_transform(rob_quat_t* p_srcq, rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp);



// ====================================================================================================
// 4. Conversions between Homogeneous Transformations, Rotation Matrices, Angles and Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Rotation Matrix <--> Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Assigns a rotation matrix to an homogeneous transformation matrix
 * 
 * @param[in]   p_R     Points to a rotation matrix
 * @param[out]  p_F     Points to a reference frame structure
 * 
 * @return None
 */
rob_status_t
rob_rot2tr(matf32_t* p_R, rob_frame_t* p_F);


/**
 * @brief   Converts an homogeneous transformation matrix to a rotation matrix
 * 
 * @param[in]   p_F     Points to a reference frame structure
 * @param[out]  p_R     Points to a rotation matrix
 * 
 * @return None
 */
rob_status_t
rob_tr2rot(rob_frame_t* p_F, matf32_t* p_R);


/**
 * @brief   Assigns both a rotation matrix and translation vector to an homogeneous transformation matrix
 * 
 * @param[in]       p_R     Points to rotation matrix
 * @param[in]       p_v     Points to translation vector
 * @param[in,out]   p_F     Points to reference frame structure
 * 
 * @return None
 */
rob_status_t
rob_update_transform(rob_frame_t* p_F, matf32_t* p_R, matf32_t* p_v);


// ----------------------------------------------------------------------------------------------------
// 4.2. Roll-Pitch-Yaw Angles -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates a rotation matrix from Cardan angles, based on their
 * definitions in Craig, Introduction to Robotics.
 * 
 * @param[in]       roll            roll angle (Z axis)
 * @param[in]       pitch           pitch angle (Y axis)
 * @param[in]       yaw             yaw angle (X axis)
 * @param[in]       rot_sequence         roll-pitch-yaw sequence enum tag
 * @param[in]       angle_units     radians or degrees indicator
 * @param[in,out]   p_R             pointer to reference frame where the rotation matrix is
 *  
 * @return None
 */
rob_status_t 
rob_rpy2rot(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R);


/**
 * @brief   Generates an homogeneous transformation from Cardan angles specified in the argument of the function.
 * For that, it first generates a rotation matrix from the Cardan angles, then assigns the rotation matrix to 
 * an homogeneous transformation matrix associated to a specific reference frame (the frame must be created beforehand).
 * 
 * @param[in]       roll            roll angle (Z axis)
 * @param[in]       pitch           pitch angle (Y axis)
 * @param[in]       yaw             yaw angle (X axis)
 * @param[in]       rot_sequence         roll-pitch-yaw sequence enum tag
 * @param[in]       angle_units     radians or degrees indicator
 * @param[in,out]   p_F             pointer reference frame where the rotation matrix is
 * 
 * @return None     
 */
void
rob_rpy2tr(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, rob_frame_t* p_F);


// ----------------------------------------------------------------------------------------------------
// 4.3. Euler Angles -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates a rotation matrix from Euler angles, based on their
 * definitions in Craig, Introduction to Robotics.
 * 
 * @param[in]       phi             phi angle
 * @param[in]       theta           theta angle
 * @param[in]       psi             psi angle
 * @param[in]       rot_sequence         roll-pitch-yaw sequence enum tag
 * @param[in]       angle_units     radians or degrees indicator
 * @param[in,out]   p_R             pointer to reference frame where the rotation matrix is
 *  
 * @return None
 */
rob_status_t 
rob_eul2rot(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R);


/**
 * @brief   Generates an homogeneous transformation from Euler angles specified in the argument of the function.
 * For that, it first generates a rotation matrix from the Euler angles, then assigns the rotation matrix to 
 * an homogeneous transformation matrix associated to a specific reference frame (the frame must be created beforehand).
 * 
 * @param[in]       phi             phi angle
 * @param[in]       theta           theta angle
 * @param[in]       psi             psi angle
 * @param[in]       rot_sequence         roll-pitch-yaw sequence enum tag
 * @param[in]       angle_units     radians or degrees indicator
 * @param[in,out]   p_F             pointer reference frame where the rotation matrix is
 * 
 * @return None     
 */
void
rob_eul2tr(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, rob_frame_t* p_F);


// ----------------------------------------------------------------------------------------------------
// 4.4. Rotation Matrices and Homogeneous Transformations -> Quaternions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates an unitary quaternion from a rotation matrix.
 * 
 * @param[in]       p_R     Points to rotation matrix
 * @param[out]      p_uq    Points to a unit quaternion
 * 
 * @return None
 */
rob_status_t
rob_rot2quat(matf32_t* p_R, rob_quat_t* p_uq);


/**
 * @brief   Generates an unitary quaternion from the homogeneous transformation matrix associated to a specific frame
 * (the frame should be created beforehand). For this, first extracts the rotation matrix from the homogenous
 * transformation matrix (see rob_tr2rot), then converts the rotation matrix to a quaternion (see rob_rot2quat).
 * 
 * @param[in]       p_F     Points to reference frame struct where the homogeneouse transformation matrix is.
 * @param[in,out]   p_uq    Points to unit quaternion
 * 
 * @return None
 */
void
rob_tr2quat(rob_frame_t* p_F, rob_quat_t* p_uq);


// ----------------------------------------------------------------------------------------------------
// 4.5. Roll-Pitch-Yaw and Euler Angles -> Quaternions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates a unitary quaternion from Cardan angles. For that, first generates a rotation
 * matrix from the Cardan angles (see rob_rpy2rot) and then converts the rotation matrix to a quaternion.
 * 
 * @param[in]       roll        Roll angle (Z axis)
 * @param[in]       pitch       Pitch angle (Y axis)
 * @param[in]       yaw         Yaw angle (X asis)
 * @param[in]       rot_sequence     Enumerate element to indicate the rotation sequence
 * @param[in]       angle_units Boolean to indicate radians or degrees
 * @param[in]       p_R         Points to a rotation matrix
 * @param[out]      p_uq        Points to a unit quaternion
 * 
 * @return None
 */
rob_status_t
rob_rpy2quat(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq);


/**
 * @brief   Generates a quaternion based on Euler angles. For that, first generates
 * a rotation matrix from the Euler angles, then converts the rotation matrix to a 
 * unitary quaternion.
 * 
 * @param[in]       phi             Phi angle
 * @param[in]       theta           Theta angle
 * @param[in]       psi             Psi angle
 * @param[in]       rot_sequence    Enumerate element to indicate the rotation sequence
 * @param[in]       angle_units     Boolean to indicate radians or degrees
 * @param[in]       p_R             Points to a rotation matrix
 * @param[out]      p_uq            Points to a unit quaternion
 * 
 * @return None
 */
rob_status_t
rob_eul2quat(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq);


// ----------------------------------------------------------------------------------------------------
// 4.6. Rotation Matrices and Homogeneous Transformations -> Roll-Pitch-Yaw and Euler Angles 
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates Cardan angles from a rotation matrix, based on the definitions in
 * Craig, Introduction to Robotics.
 * 
 * @param[in]       p_R          Points to rotation matrix
 * @param[in]       angle_units  Degree or radians indicator
 * @param[in]       rot_sequence      Roll-Pitch-Yaw sequence indicator
 * @param[in,out]   roll         Roll Angle
 * @param[in,out]   pitch        Pitch Angle
 * @param[in,out]   yaw          Yaw Angle
 * 
 * @return None
 */
rob_status_t 
rob_rot2rpy(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw);


/**
 * @brief   Generates Cardan angles from the homogeneous transformation matrix associated to a reference frame
 * (the frame must be created beforehand). For that, first extracts the rotation matrix from the homogenous
 * transformation matrix (see rob_tr2rot), then converts the rotation matrix to Cardan angles (see rob_rot2rpy).
 * 
 * @param[in]       p_F             Points to reference frame struct
 * @param[in]       angle_units     Degree or radians indicator
 * @param[in]       rot_sequence         Roll-Pitch-Yaw sequence indicator
 * @param[in,out]   roll            Roll Angle
 * @param[in,out]   pitch           Pitch Angle
 * @param[in,out]   yaw             Yaw Angle
 * 
 * @return None
 */
void
rob_tr2rpy(rob_frame_t* p_F, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw);


/**
 * @brief   Generates Euler angles from a rotation matrix, based on the definitions in
 * Craig, Introduction to Robotics.
 * 
 * @param[in]       p_R             Points to a rotation matrix
 * @param[in]       angle_units     Boolean to indicate if angles should be calculated in radians or degrees
 * @param[in,out]   phi             Phi angle
 * @param[in,out]   theta           Theta angle
 * @param[in,out]   psi             Psi angle
 * 
 * @return None
 */
rob_status_t
rob_rot2eul(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi);


/**
 * @brief   Generates Euler angles from the homogeneous transformation matrix associated to a reference frame
 * (the frame must be created beforehand). For that, first extracts the rotation matrix from the homogenous
 * transformation matrix (see rob_tr2rot), then converts the rotation matrix to Cardan angles (see rob_rot2eul).
 * 
 * @param[in]       p_F             Points to a reference frame struct
 * @param[in]       angle_units     Boolean to indicate if angles should be calculated in radians or degrees
 * @param[in,out]   phi             Phi angle
 * @param[in,out]   theta           Theta angle
 * @param[in,out]   psi             Psi angle
 * 
 * @return None
 */
void
rob_tr2eul(rob_frame_t* p_F, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi);


// ----------------------------------------------------------------------------------------------------
// 4.7. Quaternions -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates a rotation matrix from an unitary quaternion.
 * 
 * @param[in]       p_uq    Points to a unit quaternion
 * @param[in,out]   p_R     Points to a rotation matrix
 * 
 * @return None
 */
rob_status_t
rob_quat2rot(rob_quat_t* p_uq, matf32_t* p_R);


/**
 * @brief   Generates an homogeneous transformation matrix from an unitary quaternion. For that,
 * first generates a rotation matrix from the quaternion and then assigns the rotation matrix 
 * to the homogeneous transformation matrix associated to a reference frame (the frame must exist
 * beforehand).
 * 
 * @param[in]       p_uq    Points to a unit quaternion
 * @param[in,out]   p_F     Points to a reference frame struct
 * 
 * @return None
 */
void
rob_quat2tr(rob_quat_t* p_uq, rob_frame_t* p_F);


// ----------------------------------------------------------------------------------------------------
// 4.8. Quaternions -> Roll-Pitch-Yaw and Euler Angles
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Converts an unitary quaternion to Cardan angles. For that, first generates a rotation matrix
 * from a quaternion (rob_quat2rot), then converts the rotation matrix to Cardan angles (see rob_rot2rpy).
 * 
 * @param[in]       p_uq            Points to unit quaternion
 * @param[in]       angle_units     Radians or degrees indicator
 * @param[in]       rot_sequence         Roll-Pitch-Yaw sequence
 * @param[in,out]   roll            Roll Angle
 * @param[in,out]   pitch           Pitch Angle
 * @param[in,out]   yaw             Yaw Angle
 * 
 * @return None
 */
void
rob_quat2rpy(rob_quat_t* p_uq, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw);


/**
 * @brief   Converts an unitary quaternion to Euler angles. For that, first generates a rotation matrix
 * from a quaternion (rob_quat2rot), then converts the rotation matrix to Euler angles (see rob_rot2rpy).
 * 
 * @param[in]       p_uq            Points to unit quaternion
 * @param[in]       angle_units     Radians or degrees indicator
 * @param[in]       rot_sequence         Roll-Pitch-Yaw sequence
 * @param[in,out]   phi             Phi Angle
 * @param[in,out]   theta           Theta Angle
 * @param[in,out]   psi             Psi Angle
 * 
 * @return None
 */
void
rob_quat2eul(rob_quat_t* p_uq, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi);



// ====================================================================================================
// 5. Utility functions (printing, angle unit conversions, etc.)
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 5.1. Angle units conversions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Converts degrees to radians: \f[ \theta \cdot \frac{\pi}{180} \f]
 * 
 * @return float
 */
float
deg2rad(float* p_theta);


/**
 * @brief   Converts radians to degrees: \f[ \theta \cdot \frac{180}{\pi} \f]
 * 
 * @return float
 */
float
rad2deg(float* p_theta);


// ----------------------------------------------------------------------------------------------------
// 5.2. Print functions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Prints rob_status_t messages
 * 
 * @param[in]   rob_status  Error status value to print
 * 
 * @return None
 */
void
rob_status_print(rob_status_t rob_status);


/**
 * @brief   Prints the data in a rob_frame_t struct for reference frames.
 * 
 * @param[in]   p_F     Pointer to reference matrix.
 * 
 * @return None
 */
void
rob_frame_print(rob_frame_t* p_F);


/**
 * @brief   Prints the data in a rob_refpoint_t struct for reference points.
 * 
 * @param[in]   p_p     Pointer to a given point.
 * 
 * @return None
 */
void
rob_refpoint_print(rob_point_t* const p_p);


/**
 * @brief   Prints angle units (radians or degrees) to be used.
 */
void
rob_angle_units_print(bool angle_units);


/**
 * @brief   Prints the ID of the reference frame, based on available IDs in rob_frame_id_t.
 * 
 * @param[in]   id  Frame id enum
 * 
 * @return None
 */
void
rob_frame_id_print(rob_frame_id_t id);


/**
 * @brief   Prints the data of a rob_quat_t struct for quaternions.
 * 
 * @param[in]   p_srcq    Pointer to the quaternion to print
 * @return  None
 */
void
rob_quat_print(const rob_quat_t* p_srcq);


// ----------------------------------------------------------------------------------------------------
// 5.3. Check functions
// ----------------------------------------------------------------------------------------------------

/**
 * @brief    Check if a matrix is 3x3, according to the requirement for rotation matrices.
 * 
 * @param[in]   p_R     Points to matrix to be checked (matf32_t matrix, 3,3).
 * 
 * @return bool
 */
bool
rob_isrot(matf32_t* p_R);


/**
 * @brief   Check if a matrix is 4x4, according to the requirement for homogeneous transformation matrices.
 * 
 * @param[in]   p_T     Points to matrix to be checked (matf32_t matrix, 4x4).
 * 
 * @return bool
 */
bool
rob_istr(matf32_t* p_T);


/**
 * @brief   Check if a vector has 3 elements, according to the requirement of
 * coordinates vectors to be used in homogeneous transformation matrices
 * 
 * @param[in]   p_v     Points to vector to be checked (matf32_t matrix, 1x3 or 3x1);
 * 
 * @return bool
 */
bool
rob_isvec(matf32_t* p_v);


/**
 * @brief   Checks whether the reference framese to operate are in the correct order,
 * based on their IDs, according to the definnition of the homogeneous transformation
 * (see rob_apply_transform).
 * 
 * @param[in]   p_F     Pointer to reference frame structure
 * @param[in]   p_srcp  Points to original reference point
 * @param[in]   p_dstp  Points to the new reference point    
 * 
 * @return None
 */
rob_status_t
rob_check_transform_frames(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp);


/**
 * @brief   Checks if a quaternion is zero and returns an rob_status_t message. This is an 
 * auxiliary routine meant to prevent division by zero when operating quaterions, 
 * for example, is used some other routines of this library.
 * 
 * @param[in]       p_q     Pointer to quaternion or unit quaternion  
 * 
 * @return None
 */
bool
rob_check_null_quaternion(const rob_quat_t* p_q);


/**
 * @brief   Checks if two quaternions are equal in all their values.
 * 
 * @param[in]   p_q1    Points to the first quaternion to compare
 * @param[in]   p_q2    Points to the second quaternion to compare
 * 
 * @return None
 */
bool
rob_quat_is_equal(rob_quat_t* p_q1, rob_quat_t* p_q2);


#ifdef __cplusplus
}
#endif

#endif // ROBOTAT_ROBOTICS_H_

/**
 * @}
 */