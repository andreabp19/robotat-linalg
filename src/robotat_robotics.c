
/**
 * @file robotat_robotics.c
 * @author Andrea Pineda
 * @date created 19 Jul. 2025, last modified 11 Aug 2025
 * 
 * Robotics algorithms
 */

#include "robotat_robotics.h"

/* TO-DO: cuaterniones
- matriz de rotación a cuaternion
- transformacion homogenea a cuaternion y viceversa
- aplicacion de rotacion a un vector directamente en cuaterniones
- inversa de la transformación homogénea?
- Add unit quaternions. Robotics Toolbox makes the conversion between hom. transformations and quaternions through unit quaternion.
*/

/** DONE :D
* crear función trotx, troty, trotz para afectar toda la matriz de transformacion homogenea
* definir cuaternion
* multiplicacion de cuaterniones
* add rob_quat_print() to print quaternions (formatted to identify the real and imaginary parts)
* inversa de cuaterniones
*/

// ====================================================================================================
// 1. Reference frame and point initializations
// ====================================================================================================

void
rob_frame_init(rob_frame_t* p_F, matf32_t* p_T, matf32_t* p_R, matf32_t* p_v,
                rob_frame_tags_t ref_tag, rob_frame_tags_t dst_tag, bool angle_units)
{
    // Set origin, destination and angle tags
    p_F->ref_tag = ref_tag;
    p_F->dst_tag = dst_tag;
    p_F->angle_units = angle_units;

    // ADD SIZE CHECK TO ENSURE p_T points to a 4x4 matrix

    // Set matrices to the frame struct
    p_F->p_T = p_T;
    p_F->p_R = p_R;
    p_F->p_v = p_v;

    // Set last element as 1, because the last row must always be: 0 0 0 1
    matf32_set(p_T, 4, 4, 1);
}


void
rob_refpoint_init(rob_point_t* const p_p, matf32_t* p_v, float* p_data, rob_frame_tags_t ref_tag)
{
    p_p->p_v = p_v;
    p_p->ref_tag = ref_tag;

    matf32_init(p_v, 4, 1, p_data);
    matf32_set(p_v, 4, 1, 1);
}



// ====================================================================================================
// 2. Homogeneous transformation matrix operations
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 2.1. Set translation
// ----------------------------------------------------------------------------------------------------

void
rob_transl(rob_frame_t* p_F, matf32_t* p_v)
{
    matf32_t* p_T = p_F->p_T;
    matf32_t* p_R = p_F->p_R;
    matf32_t* p_t = p_F->p_v;

    // Set coordinates vector inside the transformation matrix
    matf32_submatrix_copy(p_v, p_T, 0, 0, 0, p_R->num_cols, p_v->num_rows, p_v->num_cols);
}

// ----------------------------------------------------------------------------------------------------
// 2.2. Generate and set rotation matrices
// ----------------------------------------------------------------------------------------------------

void
rob_rotx(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix X-axis
     *  | 1     0           0      |
     *  | 0 cos(theta) -sin(theta) |
     *  | 0 sin(theta)  cos(theta) |
     */

    float rads = theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1: // DEGREES
            rad2deg(&rads); //math.h revisar si hay una
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(theta*M_PI/180);
    float sin_theta = sinf(theta*M_PI/180);
    float neg_sin_theta = sin_theta*(-1);

    // Set values in the rotation matrix
    matf32_zeros(p_R);  // Reset to zero to clean from old values
    matf32_set(p_R, 1, 1, 1);
    matf32_set(p_R, 2, 2, cos_theta);
    matf32_set(p_R, 2, 3, neg_sin_theta);
    matf32_set(p_R, 3, 2, sin_theta);
    matf32_set(p_R, 3, 3, cos_theta);
}


void
rob_roty(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix Y-axis
     *  | cos(theta)  0  sin(theta)|
     *  |      0      1      0     |
     *  |-sin(theta)  0  cos(theta)|
     */

    float rads = theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1:
            rad2deg(&rads);
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(theta*M_PI/180);
    float sin_theta = sinf(theta*M_PI/180);
    float neg_sin_theta = sin_theta*(-1);

    // Set values in the rotation matrix
    matf32_zeros(p_R);  // Reset to zero to clean from old values
    matf32_set(p_R, 1, 1, cos_theta);
    matf32_set(p_R, 1, 3, sin_theta);
    matf32_set(p_R, 2, 2, 1);
    matf32_set(p_R, 3, 1, neg_sin_theta);
    matf32_set(p_R, 3, 3, cos_theta);
}


void
rob_rotz(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix Z-axis
     *  | cos(theta) -sin(theta) 0 |
     *  | sin(theta)  cos(theta) 0 |
     *  |     0           0      1 |
     */

    float rads = theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1:
            rad2deg(&rads);
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(theta*M_PI/180);
    float sin_theta = sinf(theta*M_PI/180);
    float neg_sin_theta = sin_theta*(-1);

    // Set values in the rotation matrix
    matf32_zeros(p_R);  // Reset to zero to clean from old values
    matf32_set(p_R, 1, 1, cos_theta);
    matf32_set(p_R, 1, 2, neg_sin_theta);
    matf32_set(p_R, 2, 1, sin_theta);
    matf32_set(p_R, 2, 2, cos_theta);
    matf32_set(p_R, 3, 3, 1);
}


void
rob_trotx(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_rotx(p_F->p_R, theta, angle_units);
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}


void
rob_troty(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_roty(p_F->p_R, theta, angle_units);
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}


void
rob_trotz(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_rotz(p_F->p_R, theta, angle_units);
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}


// ----------------------------------------------------------------------------------------------------
// 2.3. Applying transformations and rotation sequences
// ----------------------------------------------------------------------------------------------------

void
rob_apply_transform(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp)
{
    /**
     *  Solve: A_p = A_T_B * B_p;
     *  
     *  The most efficient way is element per element, instead of using matf32 functions.
     *  This because the matrices and vectors are fixed size always.
     * 
     *  Dimensiones breakdown:
     *  A_p     (4x1) 
     *  A_T_B   (4x4) -> I'll call it T in the following deduction for simplicity
     *  B_p     (4x1)
     * 
     *  A_p[1,1] = T[1,1]*B_p[1,1] + T[1,2]*B_p[2,1] + T[1,3]*B_p[3,1] + T[1,4]*B_p[4,1]
     *  A_p[2,1] = T[2,1]*B_p[1,1] + T[2,2]*B_p[2,1] + T[2,3]*B_p[3,1] + T[2,4]*B_p[4,1]
     *  A_p[3,1] = T[3,1]*B_p[1,1] + T[3,2]*B_p[2,1] + T[3,3]*B_p[3,1] + T[3,4]*B_p[4,1]
     *  A_p[4,1] = T[4,1]*B_p[1,1] + T[4,2]*B_p[2,1] + T[4,3]*B_p[3,1] + T[4,4]*B_p[4,1]
     */
     
    float *T_data = p_F->p_T->p_data;
    float *src_data = p_srcp->p_v->p_data;
    float *dst_data = p_dstp->p_v->p_data;
    
    // Working directly with floats, adjust indexes as necessary to fulfill the equations in the comment above.
    dst_data[0] = T_data[0]*src_data[0] + T_data[1]*src_data[1] + T_data[2]*src_data[2] + T_data[3]*src_data[3];
    dst_data[1] = T_data[4]*src_data[0] + T_data[5]*src_data[1] + T_data[6]*src_data[2] + T_data[7]*src_data[3];
    dst_data[2] = T_data[8]*src_data[0] + T_data[9]*src_data[1] + T_data[10]*src_data[2] + T_data[11]*src_data[3];
    dst_data[3] = T_data[12]*src_data[0] + T_data[13]*src_data[1] + T_data[14]*src_data[2] + T_data[15]*src_data[3];
}


// Change this to a more efficient version
// I think, that after reading the code of rpy2tr, it can be changed to calculate a single rotation matrix
// and then apply the angles, but I have to check that. I feel the current version of this function is inefficient
void
rob_apply_rot_sequence(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_angle_sequences_t angle_sequence, float phi, float theta, float psi, bool angle_units)
{
    /**
     *  Switch-case to execute transformations based onthe angle_sequence introduced
     *  
     *  For each case, sequence goes as follows:
     *      - rotate first axis,
     *      - apply transformation
     *      - repeat previous steps for the following axis
     */
   
    switch(angle_sequence)
    {
        // --------------------------------------------------
        // Euler Angles Sequences
        // --------------------------------------------------

        case XYX:
            // X
            //printf("ROTX:\n");
            rob_trotx(p_F, phi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            // Y
            //printf("ROTY:\n");
            rob_troty(p_F, theta, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            // X
            //printf("ROTX:\n");
            rob_trotx(p_F, psi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            break;

        case XZX:
            // X
            rob_trotx(p_F, phi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_trotz(p_F, theta, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_trotx(p_F, psi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXY:
            // Y
            rob_troty(p_F, phi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_trotx(p_F, theta, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_troty(p_F, psi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXZ:
            // Z
            rob_trotz(p_F, phi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_trotx(p_F, theta, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_trotz(p_F, psi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYZ:
            // Z
            rob_trotz(p_F, phi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_troty(p_F, theta, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_trotz(p_F, psi, angle_units);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        // --------------------------------------------------
        // Roll-Pitch-Yaw Sequences
        // --------------------------------------------------

        case XYZ:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XZY:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YZX:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXZ:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXY:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYX:
            rob_rpy2tr(phi, theta, psi, angle_sequence, 0, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;    
    }
}


// Inverse of T?



// ====================================================================================================
// 3. Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 3.1. Quaternion initialization
// ----------------------------------------------------------------------------------------------------

void
rob_quat_init(rob_quat_t* p_q, float* p_s, float* p_i, float* p_j, float* p_k)
{
    p_q->p_s = p_s;
    p_q->p_i = p_i;
    p_q->p_j = p_j;
    p_q->p_k = p_k;
}


// ----------------------------------------------------------------------------------------------------
// 3.2. Quaternion operations
// ----------------------------------------------------------------------------------------------------

void
rob_quat_add(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s + *p_srcq2->p_s;
    *p_dstq->p_i = *p_srcq1->p_i + *p_srcq2->p_i;
    *p_dstq->p_j = *p_srcq1->p_j + *p_srcq2->p_j;
    *p_dstq->p_k = *p_srcq1->p_k + *p_srcq2->p_k;
}


void
rob_quat_sub(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s - *p_srcq2->p_s;
    *p_dstq->p_i = *p_srcq1->p_i - *p_srcq2->p_i;
    *p_dstq->p_j = *p_srcq1->p_j - *p_srcq2->p_j;
    *p_dstq->p_k = *p_srcq1->p_k - *p_srcq2->p_k;
}


void
rob_quat_scale(const rob_quat_t* p_srcq1, const float p_c, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s*(p_c);
    *p_dstq->p_i = *p_srcq1->p_i*(p_c);
    *p_dstq->p_j = *p_srcq1->p_j*(p_c);
    *p_dstq->p_k = *p_srcq1->p_k*(p_c);
}


void
rob_quat_mul(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq)
{
    /**
     * Quaternion multiplication -> Hamilton product
     * 
     * For two given quaternions:   a1 + b1*i + c1*j + d1*k
     *                              a2 + b2*i + c2*j + d2*k
     * 
     * This gives the product as:       (a1*a2 - b1*b2 - c1*c2 - d1*d2)
     *                              +   (a1*b2 + b1*a2 + c1*d2 - d1+c2)i
     *                              +   (a1*c2 - b1*d2 + c1*a2 + d1*b2)j
     *                              +   (a1*d2 + b1*c2 - c1*b2 + d1*a2)k
     * 
     * For the notation of rob_quat_t quaternions:      (p_s1*p_s2 - p_i1*p_i2 - p_j1*p_j2 - p_k1*p_k2)
     *                                              +   (p_s1*p_i2 + p_i1*p_s2 + p_j1*p_k2 - p_k1*p_j2)  (corresponds to i)
     *                                              +   (p_s1*p_j2 - p_i1*p_k2 + p_j1*p_s2 + p_k1*p_i2)  (corresponds to j)
     *                                              +   (p_s1*p_k2 + p_i1*p_j2 - p_j1*p_i2 + p_k1*p_s2)  (corresponds to k)
     * 
     */

    // Pointers to values of the first quaternion
    const float* p_s1 = p_srcq1->p_s;
    const float* p_i1 = p_srcq1->p_i;
    const float* p_j1 = p_srcq1->p_j;
    const float* p_k1 = p_srcq1->p_k;

    // Pointers to values of the second quaternion
    const float* p_s2 = p_srcq2->p_s;
    const float* p_i2 = p_srcq2->p_i;
    const float* p_j2 = p_srcq2->p_j;
    const float* p_k2 = p_srcq2->p_k;

    // Element by element multiplication following the deduction above
    *p_dstq->p_s = *p_s1*(*p_s2) - *p_i1*(*p_i2) - *p_j1*(*p_j2) - *p_k1*(*p_k2);
    *p_dstq->p_i = *p_s1*(*p_i2) + *p_i1*(*p_s2) + *p_j1*(*p_k2) - *p_k1*(*p_j2);
    *p_dstq->p_j = *p_s1*(*p_j2) - *p_i1*(*p_k2) + *p_j1*(*p_s2) + *p_k1*(*p_i2);
    *p_dstq->p_k = *p_s1*(*p_k2) + *p_i1*(*p_j2) - *p_j1*(*p_i2) + *p_k1*(*p_s2);
}


void
rob_quat_conj(const rob_quat_t* p_srcq, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq->p_s;
    *p_dstq->p_i = *p_srcq->p_i * (-1.0);
    *p_dstq->p_j = *p_srcq->p_j * (-1.0);
    *p_dstq->p_k = *p_srcq->p_k * (-1.0);
}

void
rob_quat_norm(const rob_quat_t* p_srcq, float* p_norm)
{
    *p_norm = sqrt((*p_srcq->p_s)*(*p_srcq->p_s) + (*p_srcq->p_i)*(*p_srcq->p_i) + (*p_srcq->p_j)*(*p_srcq->p_j) + (*p_srcq->p_k)*(*p_srcq->p_k));
}

void
rob_quat_inv(const rob_quat_t* p_srcq, rob_quat_t* p_dstq)
{
    float q_norm = 0;
    float q_norm_squared = 0;

    float q_conj_data[4] = {0, 0, 0, 0};
    rob_quat_t q_conj;
    rob_quat_init(&q_conj, &q_conj_data[0], &q_conj_data[1], &q_conj_data[2], &q_conj_data[3]);

    rob_quat_conj(p_srcq, &q_conj);
    rob_quat_norm(p_srcq, &q_norm);

    q_norm_squared = q_norm*q_norm;

    *p_dstq->p_s = *q_conj.p_s / q_norm_squared;
    *p_dstq->p_i = *q_conj.p_i / q_norm_squared;
    *p_dstq->p_j = *q_conj.p_j / q_norm_squared;
    *p_dstq->p_k = *q_conj.p_k / q_norm_squared;
}



// ====================================================================================================
// 4. Conversions between Homogeneous Transformations and Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Roll-Pitch-Yaw and Rotation Matrices Conversions
// ----------------------------------------------------------------------------------------------------

void
rob_rpy2r(float roll, float pitch, float yaw, rob_angle_sequences_t rpy_tag, bool angle_units, matf32_t* p_R)
{
    /**
     * Rotation for roll-pitch-yaw angles, which correspond to Z, Y, X axes rotations, respectively.
     * So: roll: Z, pitch: Y, yaw: X
     * Options: XYZ, XZY, YXZ, YZX, ZXY, ZYX
     * 
     * Steps:
     * - Generate rotation matrices for each angle
     * - Multiply the rotation matrices
     * - Assign the result to the rotation matrix of the reference frame
     */

    float R_roll_data[MAX_MAT_SIZE];
    matf32_t R_roll;

    float R_pitch_data[MAX_MAT_SIZE];
    matf32_t R_pitch;

    float R_yaw_data[MAX_MAT_SIZE];
    matf32_t R_yaw;

    float temp_R_data[MAX_MAT_SIZE];
    matf32_t temp_R;

    matf32_init(&R_roll, 3, 3, R_roll_data);
    matf32_init(&R_pitch, 3, 3, R_pitch_data);
    matf32_init(&R_yaw, 3, 3, R_yaw_data);
    matf32_init(&temp_R, 3, 3, temp_R_data);

    // Generate rotation matrices for roll-pitch-yaw as indicated by the desired sequence.
    switch (rpy_tag)
    {
        case XYZ:
            rob_rotx(&R_yaw, yaw, angle_units);
            rob_roty(&R_pitch, pitch, angle_units);
            rob_rotz(&R_roll, roll, angle_units);
            break;

        case XZY:
            rob_rotx(&R_yaw, yaw, angle_units);
            rob_rotz(&R_pitch, pitch, angle_units);
            rob_roty(&R_roll, roll, angle_units);
            break;

        case YXZ:
            rob_roty(&R_yaw, yaw, angle_units);
            rob_rotx(&R_pitch, pitch, angle_units);
            rob_rotz(&R_roll, roll, angle_units);
            break;

        case YZX:
            rob_roty(&R_yaw, yaw, angle_units);
            rob_rotz(&R_pitch, pitch, angle_units);
            rob_rotx(&R_roll, roll, angle_units);
            break;

        case ZXY:
            rob_rotz(&R_yaw, yaw, angle_units);
            rob_rotx(&R_pitch, pitch, angle_units);
            rob_roty(&R_roll, roll, angle_units);
            break;

        case ZYX:
            rob_rotz(&R_yaw, yaw, angle_units);
            rob_roty(&R_pitch, pitch, angle_units);
            rob_rotx(&R_roll, roll, angle_units);
            break;
    }

    // Multiply the generated rotation matrices in the order: R_yaw * R_pitch * R_roll, then save to p_R
    matf32_mul(&R_yaw, &R_pitch, &temp_R);
    matf32_mul(&temp_R, &R_roll, p_R);
}


void
rob_rpy2tr(float roll, float pitch, float yaw, rob_angle_sequences_t rpy_tag, bool angle_units, rob_frame_t* p_F)
{
    /**
     * 1. Execute rob_rpy2r to generate rotation matrix based on a roll-pitch-yaw sequence
     * 2. Assign the calculated rotation matrix to the homogeneous transformation matrix in the frame p_F
     */

    rob_rpy2r(roll, pitch, yaw, rpy_tag, angle_units, p_F->p_R);
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}

// ====================================================================================================
// 5. Utility functions (printing, angle conversions, etc.)
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 5.1. Angle units conversions
// ----------------------------------------------------------------------------------------------------

float
deg2rad(float* p_theta)
{
    return *p_theta = *p_theta*(float)(M_PI/180);
}


float 
rad2deg(float* p_theta)
{
    return *p_theta = *p_theta*(float)(180/M_PI);
}


// ----------------------------------------------------------------------------------------------------
// 5.2. Print functions
// ----------------------------------------------------------------------------------------------------

void
rob_frame_print(rob_frame_t* p_F)
{
    const matf32_t* p_T = p_F->p_T;
    const matf32_t* p_R = p_F->p_R;
    const matf32_t* p_v = p_F->p_v;

    printf("-------------------------\n");
    printf("REFERENCE FRAME\n");
    printf("-------------------------\n");
    printf("Origin Tag: ");
    rob_frame_tags_print(p_F->ref_tag);
    printf("\n");
    printf("Destination Tag: ");
    rob_frame_tags_print(p_F->dst_tag);
    printf("\n");
    printf("Angle Units: ");
    rob_angle_units_print(p_F->angle_units);
    printf("\n");
    printf("Homogeneous Transformation Matrix T:\n");
    matf32_print(p_T);
    printf("Rotation Matrix R:\n");
    matf32_print(p_R);
    printf("Coordinates Vector v:\n");
    matf32_print(p_v);
    printf("-------------------------\n");
}


void
rob_refpoint_print(rob_point_t* p_p)
{
    printf("-------------------------\n");
    printf("REFERENCE POINT\n");
    printf("-------------------------\n");
    printf("Origin tag: ");
    rob_frame_tags_print(p_p->ref_tag);
    printf("\n");
    matf32_print(p_p->p_v);
    printf("-------------------------\n");
}


void
rob_frame_tags_print(rob_frame_tags_t tags)
{
    switch(tags)
    {
        case FRAME_A:
            printf("FRAME_A\n");
            break;
        
        case FRAME_B:
            printf("FRAME_B\n");
            break;
        
        case FRAME_C:
            printf("FRAME_C\n");
            break;

        case FRAME_O:
            printf("FRAME_O\n");
            break;
    }
}


void
rob_angle_units_print(bool angle_units)
{
    switch(angle_units)
    {
        case 0:
            printf("RADIANS\n");
            break;

        case 1:
            printf("DEGREES\n");
            break;
    }
}


void
rob_quat_print(const rob_quat_t* p_srcq)
{
    printf("\n%.9f + %.9fi + %.9fj + %.9fk\n\n", *p_srcq->p_s, *p_srcq->p_i, *p_srcq->p_j, *p_srcq->p_k);
}


void
rob_status_print(rob_status_t rob_status)
{
    switch(rob_status)
    {
        case ROB_SUCCESS:
            printf("ROB_SUCCESS\n");
            break;

        case ROT_SIZE_MISMATCH:
            printf("ROT_SIZE_MISMATCH\n");
            break;
        
        case HOMOG_SIZE_MISMATCH:
            printf("HOMOG_SIZE_MISMATCH\n");
            break;

        case VEC_SIZE_MISMATCH:
            printf("VEC_SIZE_MISMATCH\n");
            break;
    }
}


// ----------------------------------------------------------------------------------------------------
// 5.3. Check functions
// ----------------------------------------------------------------------------------------------------

// Verified that these work
rob_status_t
rob_isrot(matf32_t* p_R)
{
    if (p_R->num_rows != 3 && p_R->num_cols != 3)
    {
        return ROT_SIZE_MISMATCH;
    }

    return ROB_SUCCESS;
}


rob_status_t
rob_ishom(matf32_t* p_T)
{
    if (p_T->num_rows != 4 && p_T->num_cols != 4)
    {
        return HOMOG_SIZE_MISMATCH;
    }

    return ROB_SUCCESS;
}


rob_status_t
rob_isvec(matf32_t* p_v)
{
    // Defined as a column vector for convenience for all operations
    if (p_v->num_rows != 3 && p_v->num_cols != 1)
    {
        return VEC_SIZE_MISMATCH;
    }

    return ROB_SUCCESS;
}

