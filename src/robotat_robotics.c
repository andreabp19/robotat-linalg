
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
- inversa de cuaterniones
- aplicacion de rotacion a un vector directamente en cuaterniones
*/

/** DONE :D
* crear función trotx, troty, trotz para afectar toda la matriz de transformacion homogenea
* definir cuaternion
* multiplicacion de cuaterniones
* add rob_quat_print() to print quaternions (formatted to identify the real and imaginary parts)
* 
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
rob_rotx(rob_frame_t* p_F, float* p_theta)
{
    /**
     *  Rotation Matrix X-axis
     *  | 1     0           0      |
     *  | 0 cos(theta) -sin(theta) |
     *  | 0 sin(theta)  cos(theta) |
     */

    matf32_t* p_R = p_F->p_R;
    matf32_t* p_T = p_F->p_T;

    float rads = *p_theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(p_F->angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1: // DEGREES
            rad2deg(&rads); //math.h revisar si hay una
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(*p_theta*M_PI/180);
    float sin_theta = sinf(*p_theta*M_PI/180);
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
rob_roty(rob_frame_t* p_F, float* p_theta)
{
    /**
     *  Rotation Matrix Y-axis
     *  | cos(theta)  0  sin(theta)|
     *  |      0      1      0     |
     *  |-sin(theta)  0  cos(theta)|
     */

    matf32_t* p_R = p_F->p_R;
    matf32_t* p_T = p_F->p_T;

    float rads = *p_theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(p_F->angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1:
            rad2deg(&rads);
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(*p_theta*M_PI/180);
    float sin_theta = sinf(*p_theta*M_PI/180);
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
rob_rotz(rob_frame_t* p_F, float* p_theta)
{
    /**
     *  Rotation Matrix Z-axis
     *  | cos(theta) -sin(theta) 0 |
     *  | sin(theta)  cos(theta) 0 |
     *  |     0           0      1 |
     */

    matf32_t* p_R = p_F->p_R;
    matf32_t* p_T = p_F->p_T;

    float rads = *p_theta*M_PI/180;

    // Check if the input angle is in radians or degrees
    switch(p_F->angle_units)
    {
        case 0: // RADIANS
            break;
        
        case 1:
            rad2deg(&rads);
            break;
    }

    // Calculate sin, cos and -sin
    float cos_theta = cosf(*p_theta*M_PI/180);
    float sin_theta = sinf(*p_theta*M_PI/180);
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
rob_trotx(rob_frame_t* p_F, float* p_theta)
{
    // Generate x rotation matrix
    rob_rotx(p_F, p_theta);

    // Set rotation matrix in the homogeneous transformation matrix
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}


void
rob_troty(rob_frame_t* p_F, float* p_theta)
{
    // Generate x rotation matrix
    rob_roty(p_F, p_theta);

    // Set rotation matrix in the homogeneous transformation matrix
    matf32_submatrix_copy(p_F->p_R, p_F->p_T, 0, 0, 0, 0, p_F->p_R->num_rows, p_F->p_R->num_cols);
}


void
rob_trotz(rob_frame_t* p_F, float* p_theta)
{
    // Generate x rotation matrix
    rob_rotz(p_F, p_theta);

    // Set rotation matrix in the homogeneous transformation matrix
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


void
rob_apply_euler_angles(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_euler_angles_t euler_tag, float* p_phi, float* p_theta, float* p_psi)
{
    /**
     *  Switch-case to execute transformations based onthe euler_tag introduced
     *  
     *  For each case, sequence goes as follows:
     *      - rotate first axis,
     *      - apply transformation
     *      - repeat previous steps for the following axis
     */
   
    switch(euler_tag)
    {
        case XYX:
            // X
            //printf("ROTX:\n");
            rob_rotx(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            // Y
            //printf("ROTY:\n");
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            // X
            //printf("ROTX:\n");
            rob_rotx(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            break;

        case XZX:
            // X
            rob_rotx(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXY:
            // Y
            rob_roty(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXZ:
            // Z
            rob_rotz(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYZ:
            // Z
            rob_rotz(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XYZ:
            // X
            rob_rotx(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XZY:
            // X
            rob_rotx(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YZX:
            // Y
            rob_roty(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXZ:
            // Y
            rob_roty(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXY:
            // Z
            rob_rotz(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_psi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYX:
            // Z
            rob_rotz(p_F, p_phi);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_psi);
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



// ====================================================================================================
// 4. Utility functions (printing, angle conversions, etc.)
// ====================================================================================================

// ----------------------------------------------------------------------------------------------------
// 4.1. Angle units conversions
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
// 4.2. Print functions
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
// 4.3. Check functions
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
rob_ishomog(matf32_t* p_T)
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

