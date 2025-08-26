
/**
 * @file robotat_robotics.c
 * @author Andrea Pineda
 * @date created 19 Jul. 2025, last modified 16 Aug 2025
 * 
 * Robotics algorithms, based on Robotics Toolbox by Peter Corke for Matlab
 */

#include "robotat_robotics.h"

/* TO-DO:
- Implementar checkeos para evitar singularidades
- Implementar checkeo de cuaterniones nulos para evitar divisiones entre 0
*/

// ====================================================================================================
// 1. Reference frame and point initializations
// ====================================================================================================

// Tested => Works
rob_status_t
rob_frame_init(rob_frame_t* p_F, matf32_t* p_T, matf32_t* p_R, matf32_t* p_v,
                rob_frame_tags_t ref_tag, rob_frame_tags_t dst_tag, bool angle_units)
{
    // Set origin, destination and angle tags
    p_F->ref_tag = ref_tag;
    p_F->dst_tag = dst_tag;
    p_F->angle_units = angle_units;

    // ADD SIZE CHECK TO ENSURE p_T points to a 4x4 matrix
    if (rob_istr(p_T))
    {
        return TR_SIZE_MISMATCH;
    }

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    if (rob_isvec(p_v))
    {
        return VEC_SIZE_MISMATCH;
    }

    // Set matrices to the frame struct
    p_F->p_T = p_T;
    p_F->p_R = p_R;
    p_F->p_v = p_v;

    // Set last element as 1, because the last row must always be: 0 0 0 1
    matf32_set(p_T, 4, 4, 1);

    return ROB_SUCCESS;
}


// Tested => Works
// No dimension check needed because the vector is initialized inside the function with the necessary dimensions
// Warning, the matf32_t matrix used will be overwritten to a 4x1 matrix, no previous check.
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

// Tested => Works
rob_status_t
rob_transl(matf32_t* p_v, rob_frame_t* p_F)
{
    if (rob_isvec(p_v))
    {
        return VEC_SIZE_MISMATCH;
    }

    matf32_submatrix_copy(p_v, p_F->p_T, 0, 0, 0, p_F->p_R->num_cols, p_v->num_rows, p_v->num_cols);

    return ROB_SUCCESS;
}

// ----------------------------------------------------------------------------------------------------
// 2.2. Generate and set rotation matrices
// ----------------------------------------------------------------------------------------------------

// Tested => Works
rob_status_t
rob_rotx(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix X-axis
     *  | 1     0           0      |
     *  | 0 cos(theta) -sin(theta) |
     *  | 0 sin(theta)  cos(theta) |
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

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

    return ROB_SUCCESS;
}


// Tested => Works
rob_status_t
rob_roty(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix Y-axis
     *  | cos(theta)  0  sin(theta)|
     *  |      0      1      0     |
     *  |-sin(theta)  0  cos(theta)|
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

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

    return ROB_SUCCESS;
}


// Tested => Works
rob_status_t
rob_rotz(matf32_t* p_R, float theta, bool angle_units)
{
    /**
     *  Rotation Matrix Z-axis
     *  | cos(theta) -sin(theta) 0 |
     *  | sin(theta)  cos(theta) 0 |
     *  |     0           0      1 |
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

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

    return ROB_SUCCESS;
}


// Tested => Works
void
rob_trotx(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_rotx(p_F->p_R, theta, angle_units);
    rob_rot2tr(p_F->p_R, p_F);
}


// Tested => Works
void
rob_troty(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_roty(p_F->p_R, theta, angle_units);
    rob_rot2tr(p_F->p_R, p_F);
}


// Tested => Works
void
rob_trotz(rob_frame_t* p_F, float theta, bool angle_units)
{
    // Generate x rotation matrix, then set into the homogeneous transformation matrix
    rob_rotz(p_F->p_R, theta, angle_units);
    rob_rot2tr(p_F->p_R, p_F);
}


// ----------------------------------------------------------------------------------------------------
// 2.3. Applying transformations and rotation sequences
// ----------------------------------------------------------------------------------------------------

// Tested => It's not working O.o
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
     *  F_AB_T  (4x4) -> I'll call it T in the following deduction for simplicity
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
    
    // Working directly with floats, adjusting indexes as necessary to fulfill the equations in the comment above.
    dst_data[0] = T_data[0]*src_data[0] + T_data[1]*src_data[1] + T_data[2]*src_data[2] + T_data[3]*src_data[3];
    dst_data[1] = T_data[4]*src_data[0] + T_data[5]*src_data[1] + T_data[6]*src_data[2] + T_data[7]*src_data[3];
    dst_data[2] = T_data[8]*src_data[0] + T_data[9]*src_data[1] + T_data[10]*src_data[2] + T_data[11]*src_data[3];
    dst_data[3] = T_data[12]*src_data[0] + T_data[13]*src_data[1] + T_data[14]*src_data[2] + T_data[15]*src_data[3];
}


// Tested => Works
void
rob_inv_transform(rob_frame_t* const p_F, matf32_t* const p_Tinv)
{
    /**
     *  Tinv =  | R^T  -R^T * p |
     *          |  0      1     |
     */

    float trans_R_data[MAX_MAT_SIZE];
    matf32_t trans_R;
    matf32_init(&trans_R, 3, 3, trans_R_data);

    float neg_trans_R_data[MAX_MAT_SIZE];
    matf32_t neg_trans_R;
    matf32_init(&neg_trans_R, 3, 3, neg_trans_R_data);

    float temp_data[MAX_MAT_SIZE];
    matf32_t temp;
    matf32_init(&temp, 3, 1, temp_data);

    // Set p_Tinv to zeros, just in case
    matf32_zeros(p_Tinv);   

    matf32_trans(p_F->p_R, &trans_R);           // R'
    matf32_scale(&trans_R, -1, &neg_trans_R);   // -R'
    matf32_mul(&neg_trans_R, p_F->p_v, &temp);  // -R' * p

    // Copy R' to Tinv
    matf32_submatrix_copy(&trans_R, p_Tinv, 0, 0, 0, 0, trans_R.num_rows, trans_R.num_cols);
    
    // Copy -R' * p to Tinv
    matf32_submatrix_copy(&temp, p_Tinv, 0, 0, 0, 3, temp.num_rows, temp.num_cols);

    // Set last cell to 1
    matf32_set(p_Tinv, 4, 4, 1);
}


// Tested => Works
void
rob_apply_rot_sequence(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_angle_sequences_t angle_sequence, float phi, float theta, float psi, bool angle_units)
{
    /**
     * Switch-case to execute transformations based on the angle_sequence introduced
     *  
     * For each case:
     * - Check angle_sequences (XYX, XYZ, etc.)
     * - Call appropiate function: rob_eul2tr for euler angles (XYX, YZY, etc.) or rob_rpy2tr (XYZ, YZX. etc)
     * - Apply transform (solve for p_A = F_AB_T * p_B)
     */
   
    switch(angle_sequence)
    {
        // ----------------------------------------------------------------------
        // Euler Angles Sequences
        // ----------------------------------------------------------------------

        case XYX:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XZX:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXY:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YZY:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXZ:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYZ:
            rob_eul2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        // ----------------------------------------------------------------------
        // Roll-Pitch-Yaw Sequences
        // ----------------------------------------------------------------------

        case XYZ:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XZY:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YZX:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXZ:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXY:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYX:
            rob_rpy2tr(phi, theta, psi, angle_sequence, angle_units, p_F);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;    
    }
}



// ====================================================================================================
// 3. Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 3.1. Quaternion initialization
// ----------------------------------------------------------------------------------------------------

// Tested => Works
void
rob_quat_init(rob_quat_t* p_q, float* q_s, float* q_i, float* q_j, float* q_k)
{
    p_q->p_s = q_s;
    p_q->p_i = q_i;
    p_q->p_j = q_j;
    p_q->p_k = q_k;
}

// Tested => Works
void
rob_unitquat_init(rob_quat_t* p_uq, float* q_s, float* q_i, float* q_j, float* q_k)
{
    p_uq->p_s = q_s;
    p_uq->p_i = q_i;
    p_uq->p_j = q_j;
    p_uq->p_k = q_k;

    float uq_norm = 0;
    rob_quat_norm(p_uq, &uq_norm);

    *p_uq->p_s = *p_uq->p_s / uq_norm;
    *p_uq->p_i = *p_uq->p_i / uq_norm;
    *p_uq->p_j = *p_uq->p_j / uq_norm;
    *p_uq->p_k = *p_uq->p_k / uq_norm; 
}


// ----------------------------------------------------------------------------------------------------
// 3.2. Quaternion operations
// ----------------------------------------------------------------------------------------------------

// Tested => Works
void
rob_quat_add(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s + *p_srcq2->p_s;
    *p_dstq->p_i = *p_srcq1->p_i + *p_srcq2->p_i;
    *p_dstq->p_j = *p_srcq1->p_j + *p_srcq2->p_j;
    *p_dstq->p_k = *p_srcq1->p_k + *p_srcq2->p_k;
}


// Tested => Works
void
rob_quat_sub(const rob_quat_t* p_srcq1, const rob_quat_t* p_srcq2, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s - *p_srcq2->p_s;
    *p_dstq->p_i = *p_srcq1->p_i - *p_srcq2->p_i;
    *p_dstq->p_j = *p_srcq1->p_j - *p_srcq2->p_j;
    *p_dstq->p_k = *p_srcq1->p_k - *p_srcq2->p_k;
}


// Tested => Works
void
rob_quat_scale(const rob_quat_t* p_srcq1, const float p_c, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq1->p_s*(p_c);
    *p_dstq->p_i = *p_srcq1->p_i*(p_c);
    *p_dstq->p_j = *p_srcq1->p_j*(p_c);
    *p_dstq->p_k = *p_srcq1->p_k*(p_c);
}


// Tested => Works
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


// Tested => Works
void
rob_quat_conj(const rob_quat_t* p_srcq, rob_quat_t* p_dstq)
{
    *p_dstq->p_s = *p_srcq->p_s;
    *p_dstq->p_i = *p_srcq->p_i * (-1.0);
    *p_dstq->p_j = *p_srcq->p_j * (-1.0);
    *p_dstq->p_k = *p_srcq->p_k * (-1.0);
}


// Tested => Works
void
rob_quat_norm(const rob_quat_t* p_srcq, float* q_norm)
{
    *q_norm = sqrt((*p_srcq->p_s)*(*p_srcq->p_s) + (*p_srcq->p_i)*(*p_srcq->p_i) + (*p_srcq->p_j)*(*p_srcq->p_j) + (*p_srcq->p_k)*(*p_srcq->p_k));
}


// Tested => Works
rob_status_t
rob_quat_inv(const rob_quat_t* p_srcq, rob_quat_t* p_dstq)
{
    // If the quaternion is null, return error to prevent division by zero
    if (rob_check_null_quaternion(p_srcq))
    {
        return NULL_QUATERNION_ERR;
    }

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

    return ROB_SUCCESS;
}


// Tested => Works
// By nature, affects only the rotation matrix, to add a translation use rob_transl() before applying the transform.
void
rob_quat_apply_transform(rob_quat_t* p_srcq, rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp)
{
    rob_quat2tr(p_srcq, p_F);
    rob_apply_transform(p_F, p_srcp, p_dstp);
}



// ====================================================================================================
// 4. Conversions between Homogeneous Transformations and Quaternions
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Rotation Matrix <--> Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------


// Tested => Works
rob_status_t
rob_rot2tr(matf32_t* p_R, rob_frame_t* p_F)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);

    return ROB_SUCCESS;
}


// Tested => Works
rob_status_t
rob_tr2rot(rob_frame_t* p_F, matf32_t* p_R)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    matf32_submatrix_copy(p_F->p_T, p_R, 0, 0, 0, 0, p_R->num_rows, p_R->num_cols);

    return ROB_SUCCESS;
}


// Tested => Works
rob_status_t
rob_update_transform(rob_frame_t* p_F, matf32_t* p_R, matf32_t* p_v)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    if (rob_isvec(p_v))
    {
        return VEC_SIZE_MISMATCH;
    }

    rob_rot2tr(p_R, p_F);
    rob_transl(p_v, p_F);

    return ROB_SUCCESS;
}

// ----------------------------------------------------------------------------------------------------
// 4.2. Roll-Pitch-Yaw Angles -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------


// Tested => Works
// Change the angles names to avoid confusion
rob_status_t
rob_rpy2rot(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R)
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

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

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
    switch (rot_sequence)
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

    return ROB_SUCCESS;
}


// Tested => Works
void
rob_rpy2tr(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, rob_frame_t* p_F)
{
    /**
     * 1. Execute rob_rpy2rot to generate rotation matrix based on a roll-pitch-yaw sequence
     * 2. Assign the calculated rotation matrix to the homogeneous transformation matrix in the frame p_F
     */

    rob_rpy2rot(roll, pitch, yaw, rot_sequence, angle_units, p_F->p_R);
    rob_rot2tr(p_F->p_R, p_F);
}


// ----------------------------------------------------------------------------------------------------
// 4.3. Euler Angles -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

// Tested => Works
rob_status_t
rob_eul2rot(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R)
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

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    float R_phi_data[MAX_MAT_SIZE];
    matf32_t R_phi;

    float R_theta_data[MAX_MAT_SIZE];
    matf32_t R_theta;

    float R_psi_data[MAX_MAT_SIZE];
    matf32_t R_psi;

    float temp_R_data[MAX_MAT_SIZE];
    matf32_t temp_R;

    matf32_init(&R_phi, 3, 3, R_phi_data);
    matf32_init(&R_theta, 3, 3, R_theta_data);
    matf32_init(&R_psi, 3, 3, R_psi_data);
    matf32_init(&temp_R, 3, 3, temp_R_data);

    // Generate rotation matrices for phi-theta-psi as indicated by the desired sequence.
    switch (rot_sequence)
    {
        case XYX:
            rob_rotx(&R_phi, phi, angle_units);
            rob_roty(&R_theta, theta, angle_units);
            rob_rotx(&R_psi, psi, angle_units);
            break;

        case XZX:
            rob_rotx(&R_phi, phi, angle_units);
            rob_rotz(&R_theta, theta, angle_units);
            rob_rotx(&R_psi, psi, angle_units);
            break;

        case YXY:
            rob_roty(&R_phi, phi, angle_units);
            rob_rotx(&R_theta, theta, angle_units);
            rob_roty(&R_psi, psi, angle_units);
            break;

        case YZY:
            rob_roty(&R_phi, phi, angle_units);
            rob_rotz(&R_theta, theta, angle_units);
            rob_roty(&R_psi, psi, angle_units);
            break;

        case ZXZ:
            rob_rotz(&R_phi, phi, angle_units);
            rob_rotx(&R_theta, theta, angle_units);
            rob_rotz(&R_psi, psi, angle_units);
            break;

        case ZYZ:
            rob_rotz(&R_phi, phi, angle_units);
            rob_roty(&R_theta, theta, angle_units);
            rob_rotz(&R_psi, psi, angle_units);
            break;
    }

    // Multiply the generated rotation matrices in the order: R_psi * R_theta * R_phi, then save to p_R
    matf32_mul(&R_phi, &R_theta, &temp_R);
    matf32_mul(&temp_R, &R_psi, p_R);

    return ROB_SUCCESS;
}


// Tested => Works
void
rob_eul2tr(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, rob_frame_t* p_F)
{
    /**
     * 1. Execute rob_eul2rot to generate rotation matrix based on an euler angles sequence.
     * 2. Assign the calculated rotation matrix to the homogeneous transformation matrix in the frame p_F
     */

    rob_eul2rot(phi, theta, psi, rot_sequence, angle_units, p_F->p_R);
    rob_rot2tr(p_F->p_R, p_F);
}


// ----------------------------------------------------------------------------------------------------
// 4.4. Rotation Matrices and Homogeneous Transformations -> Quaternions
// ----------------------------------------------------------------------------------------------------

// Tested => Works
rob_status_t
rob_rot2quat(matf32_t* p_R, rob_quat_t* p_uq)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    float diag_1 = p_R->p_data[0];
    float diag_2 = p_R->p_data[4];
    float diag_3 = p_R->p_data[8];

    float sgn = 0;
    
    float kx = 0;
    float kx1 = 0;

    float ky = 0;
    float ky1 = 0;
    
    float kz = 0;
    float kz1 = 0;

    float uq_s = 0;
    float uq_i = 0;
    float uq_j = 0;
    float uq_k = 0;

    float v_norm = 0;

    uq_s = sqrt((diag_1 + diag_2 + diag_3)+1.0)/2.0;

    kx = p_R->p_data[7] - p_R->p_data[5];
    ky = p_R->p_data[2] - p_R->p_data[6];
    kz = p_R->p_data[3] - p_R->p_data[1];

    // Check for the max diagonal value of the rotation matrix.
    // In case two are equal and max, then it will pick the first ocurrence (that's why it's >= instead of >)
    if (diag_1 >= diag_2 && diag_1 >= diag_3)       // diag_1 is the max diagonal value
    {
        kx1 = p_R->p_data[0] - p_R->p_data[4] - p_R->p_data[8] + 1;
        ky1 = p_R->p_data[3] + p_R->p_data[1];
        kz1 = p_R->p_data[6] + p_R->p_data[2];

        if (kx > 0)
        {
            sgn = 1;
        }
        else if (kx < 0)
        {
            sgn = -1;
        }
        else
        {
            sgn = 0;
        }
    }
    else if (diag_2 >= diag_1 && diag_2 >= diag_3)  // diag 2 is the max diagonal value
    {
        kx1 = p_R->p_data[3] + p_R->p_data[1];
        ky1 = p_R->p_data[4] - p_R->p_data[0] - p_R->p_data[8] + 1;
        kz1 = p_R->p_data[7] + p_R->p_data[5];

        if (ky > 0)
        {
            sgn = 1;
        }
        else if (ky < 0)
        {
            sgn = -1;
        }
        else
        {
            sgn = 0;
        }
    }
    else if (diag_3 >= diag_1 && diag_3 >= diag_2)  // diag_3 is the max diagonal value
    {
        kx1 = p_R->p_data[6] + p_R->p_data[2];
        ky1 = p_R->p_data[7] + p_R->p_data[5];
        kz1 = p_R->p_data[8] - p_R->p_data[0] - p_R->p_data[4] + 1;

        if (kz > 0)
        {
            sgn = 1;
        }
        else if (kz < 0)
        {
            sgn = -1;
        }
        else
        {
            sgn = 0;
        }
    }

    uq_i = kx + sgn*kx1;
    uq_j = ky + sgn*ky1;
    uq_k = kz + sgn*kz1;

    v_norm = sqrt(uq_i*uq_i + uq_j*uq_j + uq_k*uq_k);

    uq_i = uq_i * sqrt(1 - (uq_s*uq_s)) / v_norm;
    uq_j = uq_j * sqrt(1 - (uq_s*uq_s)) / v_norm;
    uq_k = uq_k * sqrt(1 - (uq_s*uq_s)) / v_norm;

    *p_uq->p_s = uq_s;
    *p_uq->p_i = uq_i;
    *p_uq->p_j = uq_j;
    *p_uq->p_k = uq_k;

    return ROB_SUCCESS;
}


// Tested => Works
void 
rob_tr2quat(rob_frame_t* p_F, rob_quat_t* p_uq)
{
    rob_rot2quat(p_F->p_R, p_uq);
}


// ----------------------------------------------------------------------------------------------------
// 4.5. Roll-Pitch-Yaw and Euler Angles -> Quaternions
// ----------------------------------------------------------------------------------------------------

// Tested => Works
rob_status_t
rob_rpy2quat(float roll, float pitch, float yaw, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    rob_rpy2rot(roll, pitch, yaw, rot_sequence, angle_units, p_R);
    rob_rot2quat(p_R, p_uq);

    return ROB_SUCCESS;
}


// Tested => Works
rob_status_t
rob_eul2quat(float phi, float theta, float psi, rob_angle_sequences_t rot_sequence, bool angle_units, matf32_t* p_R, rob_quat_t* p_uq)
{
    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    rob_eul2rot(phi, theta, psi, rot_sequence, angle_units, p_R);
    rob_rot2quat(p_R, p_uq);

    return ROB_SUCCESS;
}


// ----------------------------------------------------------------------------------------------------
// 4.6. Rotation Matrices and Homogeneous Transformations -> Roll-Pitch-Yaw and Euler Angles 
// ----------------------------------------------------------------------------------------------------

// Still not fully tested
rob_status_t 
rob_rot2rpy(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw)
{
    /**
     * Working from the matrix rotation definitions in Craig - Introduction to Robotics and comparing with Robotics Toolbox
     * 
     * Solving for each angle in rotation sequence, using acos, asin and atan2, to avoid divisions.
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    switch(rot_sequence)
    {
        case XYZ:
            *yaw = atan2f(-1.0*p_R->p_data[5], p_R->p_data[8]);
            *pitch = asinf(p_R->p_data[2]);
            *roll = atan2f(-1.0*p_R->p_data[1], p_R->p_data[0]);
            break;
        
        case XZY:
            *yaw = atan2f(p_R->p_data[7], p_R->p_data[4]);
            *pitch = asinf(-1.0*p_R->p_data[1]);
            *roll = atan2f(p_R->p_data[2], p_R->p_data[6]);
            break;
        
        case YXZ:
            *yaw = atan2f(p_R->p_data[2], p_R->p_data[8]);
            *pitch = asinf(-1.0*p_R->p_data[5]);
            *roll = atan2f(p_R->p_data[3], p_R->p_data[4]);
            break;
        
        case YZX:
            *yaw = atan2f(-1.0*p_R->p_data[6], p_R->p_data[0]);
            *pitch = asinf(p_R->p_data[3]);
            *roll = atan2f(-1.0*p_R->p_data[5], p_R->p_data[4]);
            break;
        
        case ZXY:
            *yaw = atan2f(-1.0*p_R->p_data[1], p_R->p_data[4]);
            *pitch = asinf(p_R->p_data[7]);
            *roll = atan2f(-1.0*p_R->p_data[6], p_R->p_data[8]);
            break;
        
        case ZYX:
            *yaw = atan2f(p_R->p_data[3], p_R->p_data[0]);
            *pitch = asinf(-1.0*p_R->p_data[6]);
            *roll = atan2f(p_R->p_data[7], p_R->p_data[8]);
            break;
    }

    // TODO: Add optional conversion to degrees

    return ROB_SUCCESS;
}


void
rob_tr2rpy(rob_frame_t* p_F, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw)
{
    rob_rot2rpy(p_F->p_R, angle_units, rot_sequence, roll, pitch, yaw);
}


// Still not fully tested
// Change all rot_sequence paraments into eul_seq, and rot_sequence into rpy_seq
rob_status_t 
rob_rot2eul(matf32_t* p_R, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi)
{
    /**
     * Working from the matrix rotation definitions in Craig - Introduction to Robotics
     * 
     * Craig uses alpha, beta and gamma for the angles, the equivalent in this function is phi, theta and psi, respectively.
     * 
     * Solving for each angle in rotation sequence, using acos, asin and atan2, to avoid divisions.
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    switch(rot_sequence)
    {
        case XYX:
            *phi = atan2f(p_R->p_data[3], -1.0*p_R->p_data[6]);
            *theta = acosf(p_R->p_data[0]);
            *psi = atan2f(p_R->p_data[1], p_R->p_data[2]);
            break;

        case XZX:
            *phi = atan2f(p_R->p_data[6], p_R->p_data[3]);
            *theta = acosf(p_R->p_data[0]); 
            *psi = atan2f(p_R->p_data[2], -1.0*p_R->p_data[1]);
            break;

        case YXY:
            *phi = atan2f(p_R->p_data[1], p_R->p_data[7]);
            *theta = acosf(p_R->p_data[4]);
            *psi = atan2f(p_R->p_data[3], -1.0*p_R->p_data[5]);
            break;

        case YZY:
            *phi = atan2f(p_R->p_data[7], -1.0*p_R->p_data[1]);
            *theta = acosf(p_R->p_data[4]);
            *psi = atan2f(p_R->p_data[5], p_R->p_data[3]);
            break;

        case ZXZ:
            *phi = atan2f(p_R->p_data[2], -1.0*p_R->p_data[5]);
            *theta = acosf(p_R->p_data[8]);
            *psi = atan2f(p_R->p_data[6], p_R->p_data[7]);
            break;

        case ZYZ:
            *phi = atan2f(p_R->p_data[5], p_R->p_data[2]);
            *theta = acosf(p_R->p_data[8]);
            *psi = atan2f(p_R->p_data[7], -1.0*p_R->p_data[6]);
            break;
    }

    // TODO: Add optional conversion to degrees

    return ROB_SUCCESS;
}


// Still not tested
void
rob_tr2eul(rob_frame_t* p_F, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi)
{
    rob_rot2eul(p_F->p_R, angle_units, rot_sequence, phi, theta, psi);
}


// ----------------------------------------------------------------------------------------------------
// 4.7. Quaternions -> Rotation Matrices and Homogeneous Transformations
// ----------------------------------------------------------------------------------------------------

// Tested => Works
rob_status_t
rob_quat2rot(rob_quat_t* p_uq, matf32_t* p_R)
{
    /**
     * Element by element
     * 
     * Use notation xyz to check if I wrote it well from the reference equation, then change to:
     * uq_s, uq_i, uq_j and uq_k
     */

    if (rob_isrot(p_R))
    {
        return ROT_SIZE_MISMATCH;
    }

    float s = *p_uq->p_s;
    float x = *p_uq->p_i;
    float y = *p_uq->p_j;
    float z = *p_uq->p_k;

    // Row 1
    p_R->p_data[0] = 1.0 - (2.0*((y*y) + (z*z)));
    p_R->p_data[1] = 2.0*((x*y) - (s*z));
    p_R->p_data[2] = 2.0*((x*z) + (s*y));
    
    // Row 2
    p_R->p_data[3] = 2.0*((x*y) + (s*z));
    p_R->p_data[4] = 1.0 - (2.0*((x*x) + (z*z)));
    p_R->p_data[5] = 2.0*((y*z) - (s*x));
    
    // Row 3
    p_R->p_data[6] = 2.0*((x*z) - (s*y));
    p_R->p_data[7] = 2.0*((y*z) + (s*x));
    p_R->p_data[8] = 1.0 - (2.0*((x*x) + (y*y)));

    return ROB_SUCCESS;
}


// Tested => Works
void
rob_quat2tr(rob_quat_t* p_uq, rob_frame_t* p_F)
{
    rob_quat2rot(p_uq, p_F->p_R);
    rob_rot2tr(p_F->p_R, p_F);
}


// ----------------------------------------------------------------------------------------------------
// 4.8. Quaternions -> Roll-Pitch-Yaw and Euler Angles
// ----------------------------------------------------------------------------------------------------

// Tested => Works
void
rob_quat2rpy(rob_quat_t* p_uq, bool angle_units, rob_angle_sequences_t rot_sequence, float* roll, float* pitch, float* yaw)
{
    float temp_R_data[MAX_MAT_SIZE];
    matf32_t temp_R;
    matf32_init(&temp_R, 3, 3, temp_R_data);

    // Convert from quaternion to rotation matrix, save in a temporal matrix
    rob_quat2rot(p_uq, &temp_R);

    // Convert the temporal rotation matrix to rpy angles
    rob_rot2rpy(&temp_R, angle_units, rot_sequence, roll, pitch, yaw);
}


// Tested => Works
void
rob_quat2eul(rob_quat_t* p_uq, bool angle_units, rob_angle_sequences_t rot_sequence, float* phi, float* theta, float* psi)
{
    float temp_R_data[MAX_MAT_SIZE];
    matf32_t temp_R;
    matf32_init(&temp_R, 3, 3, temp_R_data);

    // Convert from quaternion to rotation matrix, save in a temporal matrix
    rob_quat2rot(p_uq, &temp_R);

    // Convert the temporal rotation matrix to euler angles
    rob_rot2eul(&temp_R, angle_units, rot_sequence, phi, theta, psi);
}

// ====================================================================================================
// 5. Utility functions (printing, angle conversions, etc.)
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 5.1. Angle units conversions
// ----------------------------------------------------------------------------------------------------

// Tested => Works
float
deg2rad(float* p_theta)
{
    return *p_theta = *p_theta*(float)(M_PI/180);
}


// Tested => Works
float 
rad2deg(float* p_theta)
{
    return *p_theta = *p_theta*(float)(180/M_PI);
}


// ----------------------------------------------------------------------------------------------------
// 5.2. Print functions
// ----------------------------------------------------------------------------------------------------

// Tested => Works
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


// Tested => Works
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


// Tested => Works
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


// Tested => Works
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


// Tested => Works
void
rob_quat_print(const rob_quat_t* p_srcq)
{
    printf("\n\n%.9f + %.9fi + %.9fj + %.9fk\n\n", *p_srcq->p_s, *p_srcq->p_i, *p_srcq->p_j, *p_srcq->p_k);
}


// Tested => Works
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
        
        case TR_SIZE_MISMATCH:
            printf("TR_SIZE_MISMATCH\n");
            break;

        case VEC_SIZE_MISMATCH:
            printf("VEC_SIZE_MISMATCH\n");
            break;

        case TRANSFORM_FRAMES_MISMATCH:
            printf("TRANSFORM_FRAMES_MISMATCH\n");
            break;

        case NULL_QUATERNION_ERR:
            printf("NULL_QUATERNION_ERR\n");
            break;
    }
}


// ----------------------------------------------------------------------------------------------------
// 5.3. Check functions
// ----------------------------------------------------------------------------------------------------

// Tested => Works
bool
rob_isrot(matf32_t* p_R)
{
    if (p_R->num_rows != 3 || p_R->num_cols != 3)
    {
        return true;
    }

    return false;
}


// Tested => Works
bool
rob_istr(matf32_t* p_T)
{
    if (p_T->num_rows != 4 || p_T->num_cols != 4)
    {
        return true;
    }

    return false;
}


// Tested => Works
bool
rob_isvec(matf32_t* p_v)
{
    // Defined as a column vector for convenience for all operations
    if (p_v->num_rows != 3 || p_v->num_cols != 1)
    {
        return true;
    }

    return false;
}

// Tested => Works
rob_status_t
rob_check_transform_frames(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp)
{
    /**
     * Order of operations is: A_p = A_T_B * B_p
     * In notation of this function: p_dstp = p_F.p_T * p_dstp
     * (where p_F.p_T is the transformation matrix of the frame F)
     * 
     * So check:
     *  - If p_dstp.ref_tag == p_F.dst_tag 
     *  - If p_F.ref_tag == p_srcp.ref_tag
     */

    if (p_dstp->ref_tag != p_F->dst_tag)
    {
        return TRANSFORM_FRAMES_MISMATCH;
    }

    if (p_F->ref_tag != p_srcp->ref_tag)
    {
        return TRANSFORM_FRAMES_MISMATCH;
    }

    return ROB_SUCCESS;
}


// Tested => Works
bool
rob_check_null_quaternion(const rob_quat_t* p_q)
{
    if (*p_q->p_s == 0.0 && *p_q->p_i == 0.0 && *p_q->p_j == 0.0 && *p_q->p_k == 0.0)
    {
        return true;
    }

    return false;
}

// Tested => Works
bool
rob_quat_is_equal(rob_quat_t* p_q1, rob_quat_t* p_q2)
{
    if (is_equal_margin(*p_q1->p_s, *p_q2->p_s) && is_equal_margin(*p_q1->p_i, *p_q2->p_i) && is_equal_margin(*p_q1->p_j, *p_q2->p_j) && is_equal_margin(*p_q1->p_k, *p_q2->p_k))
    {
        return true;
    }

    return false;
}