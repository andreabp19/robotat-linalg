
/**
 * @file robotat_robotics.h
 * @author Andrea Pineda
 * @date created 19 Jul. 2025, last modified 16 Aug 2025
 *
 * Robotics algorithms, based on Robotics Toolbox by Peter Corke for Matlab
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
#include <math.h>               // For sin() and cos()
#include "matf32.h"
#include "linsolve.h"

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
    ROB_SUCCESS,
    ROT_SIZE_MISMATCH,
    HOMOG_SIZE_MISMATCH,
    VEC_SIZE_MISMATCH,
    TRANSFORM_FRAMES_MISMATCH,
    NULL_QUATERNION_ERR
} rob_status_t;

/**
 * @brief   Enumerates list for matrix tags.
 */
typedef enum
{
    FRAME_A,  
    FRAME_B,
    FRAME_C,
    FRAME_O   // Global reference frame
} rob_frame_tags_t;

/**
 * @brief   Enumerates list of tags for possible rotation sequences to implement.
 */
typedef enum
{
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ,
    ZYZ,
    XYZ,
    XZY,
    YZX,
    YXZ,
    ZXY,
    ZYX
} rob_angle_sequences_t;

/**
 * @brief   Homogeneneous transformation matrix construct for reference frames' pose calcultions
 */
typedef struct 
{   
    matf32_t* p_T;                /** Points to homogeneous transformation matrix */
    matf32_t* p_R;                /** Points to rotation matrix */
    matf32_t* p_v;                /** Points to coordinates vector */
    rob_frame_tags_t ref_tag;     /** Current reference frame for the transformation */
    rob_frame_tags_t dst_tag;     /** Destination reference frame for the transformation */
    bool angle_units;             /** Angle unit tags, between RADIANS or DEGREES */
} rob_frame_t;

typedef struct
{
    const matf32_t* p_v;        /** Coordinates vector for a given point with respect to the frame */
    rob_frame_tags_t ref_tag;   /** Enumerate tags for frame reference and destination */
} rob_point_t;

/**
 * @brief   Quaternion struct
 */
typedef struct
{
    float* p_s;     /** Real number s of the quaternion */
    float* p_i;     /** Imaginary number i of the quaternion */
    float* p_j;     /** Imaginary number j of the quaternion */
    float* p_k;     /** Imaginary number k of the quaternion */
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
 * @param[in]   ref_tag         Enumerate tag for the current reference frame for the transformation.
 * @param[in]   dst_tag         Enumerate tag for the destination frame for the transformation.
 * @param[in]   angle_units     Enumerate tag for the units of the angles.
 * 
 * @return None
 */
void
rob_frame_init(rob_frame_t* p_F, matf32_t* p_T, matf32_t* p_R, matf32_t* p_v, rob_frame_tags_t ref_tag, rob_frame_tags_t dst_tag, bool angle_units);

/**
 * @brief   Constructor for the reference frame data structure, defines 4x4 dimensiones, sets the rotation matrix,
 * coordinates vector and creates the homogeneous transformation matrix.
 * 
 * @param[in]   p_p         Points to the point struct to use.
 * @param[in]   p_data      Points to data float to use
 * @param[in]   ref_tag     Enum tag to indiate the associated reference frame
 * 
 * @return None
 */
void
rob_refpoint_init(rob_point_t* const p_p, matf32_t* p_v, float* p_data, rob_frame_tags_t ref_tag);



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
 * @return  err_status_t
 */
void
rob_transl(matf32_t* p_v, rob_frame_t* p_F);

// ----------------------------------------------------------------------------------------------------
// 2.2. Generate and set rotation matrices
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Generates rotation in the x axis.
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix.
 * @param[in]       p_theta     Points to the value of degrees by which to rotate the frame.
 * 
 * @return err_status_t
 */
void
rob_rotx(matf32_t* p_R, float theta, bool angle_units);

/**
 * @brief   Generates rotation in the y axis.
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix.
 * @param[in]       theta       Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_roty(matf32_t* p_R, float theta, bool angle_units);

/**
 * @brief   Generates rotation in the z axis.
 * 
 * @param[in,out]   p_R         Pointer to rotation matrix
 * @param[in]       theta       Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_rotz(matf32_t* p_R, float theta, bool angle_units);

/**
 * @brief   Applies x rotation matrix to the homogeneous transformation matrix T
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_trotx(rob_frame_t* p_F, float theta, bool angle_units);

/**
 * @brief   Applies y rotation matrix to the homogeneous transformation matrix T
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_troty(rob_frame_t* p_F, float theta, bool angle_units);

/**
 * @brief   Applies z rotation matrix to the homogeneous transformation matrix T
 * 
 * @param[in,out]   p_F         Pointer to reference frame struct.
 * @param[in]       theta     Angle for the rotation (either radians or degrees, which must be indicated when initiation the frame)
 * 
 * @return None
 */
void
rob_trotz(rob_frame_t* p_F, float theta, bool angle_units);

// ----------------------------------------------------------------------------------------------------
// 2.3. Applying transformations and rotation sequences
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Applies the homogeneous transformation to calculate the pose.
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
 * @param[in]       p_F     Pointer to reference frame struct.
 * @param[in,out]   p_Tinv  Pointer to output matrix to save the inverse.     
 * 
 * @return None
 */
void
rob_inv_transform(rob_frame_t* const p_F, matf32_t* const p_Tinv);

/**
 * @brief   Apply transforms based on Euler angles sequences.
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
 * @brief   Multiplies two quaternions
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
void
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
void
rob_rot2tr(matf32_t* p_R, rob_frame_t* p_F);

/**
 * @brief   Converts an homogeneous transformation matrix to a rotation matrix
 * 
 * @param[in]   p_F     Points to a reference frame structure
 * @param[out]  p_R     Points to a rotation matrix
 * 
 * @return None
 */
void
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
void
rob_update_transform(rob_frame_t* p_F, matf32_t* p_R, matf32_t* p_v);

// ----------------------------------------------------------------------------------------------------
// 4.2. Roll-Pitch-Yaw Angles -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Converts roll-pitch-yaw sequence angles to a rotation matrix
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
void 
rob_rpy2rot(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R);

/**
 * @brief   Applies roll-pitch-yaw sequence angles to an homogeneous transformation matrix
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
 * @brief   Converts euler angles sequence to a rotation matrix
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
void 
rob_eul2rot(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R);

/**
 * @brief   Applies Euler angle sequence to an homogeneous transformation matrix
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
 * @brief   Converts a rotation matrix into a quaternion
 * 
 * @param[in]       p_R     Points to rotation matrix
 * @param[out]      p_uq    Points to a unit quaternion
 * 
 * @return None
 */
void
rob_rot2quat(matf32_t* p_R, rob_quat_t* p_uq);

/**
 * @brief   Converts homogeneous transformation matrix into a quaternion
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
 * @brief   Converts a roll-pitch-yaw sequence into a quaternion
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
void
rob_rpy2quat(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq);


/**
 * @brief   Converts an euler angle sequence into a quaternion
 * 
 * @param[in]       phi         Phi angle
 * @param[in]       theta       Theta angle
 * @param[in]       psi         Psi angle
 * @param[in]       rot_sequence     Enumerate element to indicate the rotation sequence
 * @param[in]       angle_units Boolean to indicate radians or degrees
 * @param[in]       p_R         Points to a rotation matrix
 * @param[out]      p_uq        Points to a unit quaternion
 * 
 * @return None
 */
void
rob_eul2quat(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq);


// ----------------------------------------------------------------------------------------------------
// 4.6. Rotation Matrices and Homogeneous Transformations -> Roll-Pitch-Yaw and Euler Angles 
// ----------------------------------------------------------------------------------------------------

/**
 * @brief   Converts a rotation matrix to roll-pitch-yaw angles
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
void 
rob_rot2rpy(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw);


/**
 * @brief   Converts an homogeneous transformation matrix to roll-pitch-yaw angles
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
 * @brief   Converts an homogeneous transformation matrix to roll-pitch-yaw angles
 * 
 */

/**
 * @brief   Converts a rotation matrix to euler angles
 * 
 * @param[in]       p_R             Points to a rotation matrix
 * @param[in]       angle_units     Boolean to indicate if angles should be calculated in radians or degrees
 * @param[in,out]   phi             Phi angle
 * @param[in,out]   theta           Theta angle
 * @param[in,out]   psi             Psi angle
 * 
 * @return None
 */
void
rob_rot2eul(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi);

/**
 * @brief   Converts an homogeneous transformation matrix to euler angles
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
 * @brief   Converts a quaternion to a rotation matrix
 * 
 * @param[in]       p_uq    Points to a unit quaternion
 * @param[in,out]   p_R     Points to a rotation matrix
 * 
 * @return None
 */
void
rob_quat2rot(rob_quat_t* p_uq, matf32_t* p_R);

/**
 * @brief   Converts a quaternion to an homogeneous transformation matrix
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
 * @brief   Converts a quaternion to roll-pitch-yaw angles
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
 * @brief   Converts a quaternion to euler angles
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
 * @brief   Converts degrees to radians.
 * 
 * @return float
 */
float
deg2rad(float* p_theta);

/**
 * @brief   Converts radians to degrees.
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
 * @brief   Prints the data and configurations of a reference frame.
 * 
 * @param[in]   p_F     Pointer to reference matrix.
 * 
 * @return None
 */
void
rob_frame_print(rob_frame_t* p_F);

/**
 * @brief   Prints a point's data and configurations to use with it.
 * 
 * @param[in]   p_p     Pointer to a given point.
 * 
 * @return None
 */
void
rob_refpoint_print(rob_point_t* const p_p);

/**
 * @brief   Prints text corresponding to the tags in rob_angle_unit_tags_t.
 */
void
rob_angle_units_print(bool angle_units);

/**
 * @brief   Prints text corresponding to the tags in rob_frame_tags_t.
 * 
 * @param[in]   frame_tags  Frame tags enum
 * 
 * @return None
 */
void
rob_frame_tags_print(rob_frame_tags_t tags);


/**
 * @brief   Prints a quaternion as formatted text
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
 * @brief    Check if a matrix is 3x3 to be a rotation matrix
 * 
 * @param[in]   p_R     Points to matrix to be checked.
 * 
 * @return rob_status_t
 */
rob_status_t
rob_isrot(matf32_t* p_R);

/**
 * @brief   Check if a matrix is 4x4 to be an homogeneous transformation matrix
 * 
 * @param[in]   p_T     Points to matrix to be checked.
 * 
 * @return rob_status_t
 */
rob_status_t
rob_ishom(matf32_t* p_T);

/**
 * @brief   Check if a vector has 3 elements to be used as coordinates vector in the homogeneous transformation matrix
 * 
 * @param[in]   p_v     Points to vector to be checked (matrix, 1x3 or 3x1);
 * 
 * @return rob_status_t
 */
rob_status_t
rob_isvec(matf32_t* p_v);

/**
 * @brief   Checks frame tags to determine if the operations were done in the right order
 * 
 * @param[in]   p_F     Pointer to reference frame structure
 * @param[in]   p_srcp  Points to original reference point
 * @param[in]   p_dstp  Points to the new reference point    
 * 
 * @return None
 */
rob_status_t
rob_check_transform_frames(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp);

// Add checks for quaternions and unit quaternions?

/**
 * @brief   Checks if a quaternion is zero and returns an rob_status_t message
 * 
 * @param[in]       p_q     Pointer to quaternion or unit quaternion  
 * 
 * @return None
 */
rob_status_t
rob_check_null_quaternion(rob_quat_t* p_q);

/**
 * @brief   Checks if two quaternions are equal
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
