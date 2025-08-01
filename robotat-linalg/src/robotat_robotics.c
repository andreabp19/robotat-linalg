
/**
 * @file robotat_robotics.c
 * @author Andrea Pineda
 * @date created 19 Jul. 2025
 * 
 * Robotics algorithms
 */

#include "robotat_robotics.h"

// OPERATIONS

// crear función trotx, troty, trotz para afectar toda la matriz de transformacion homogenea

/* TO-DO: cuaterniones
- definir cuaternion
- matriz de rotación a cuaternion
- transformacion homogenea a cuaternion y viceversa
- multiplicacion de cuaterniones
- inversa de cuaterniones
- aplicacion de rotacion a un vector directamente en cuaterniones
*/

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


void
rob_transl(rob_frame_t* p_F, matf32_t* p_v)
{
    matf32_t* p_T = p_F->p_T;
    matf32_t* p_R = p_F->p_R;
    matf32_t* p_t = p_F->p_v;

    // Set coordinates vector inside the transformation matrix
    matf32_submatrix_copy(p_v, p_T, 0, 0, 0, p_R->num_cols, p_v->num_rows, p_v->num_cols);
}


void
rob_rotx(rob_frame_t* p_F, float* p_theta) // esto es trotx
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

    // Set rotation matrix in the homogeneous transformation matrix
    matf32_submatrix_copy(p_R, p_T, 0, 0, 0, 0, p_R->num_rows, p_R->num_cols);
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

    // Set rotation matrix in the homogeneous transformation matrix
    matf32_submatrix_copy(p_R, p_T, 0, 0, 0, 0, p_R->num_rows, p_R->num_cols);
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

    // Set rotation matrix in the homogeneous transformation matrix
    matf32_submatrix_copy(p_R, p_T, 0, 0, 0, 0, p_R->num_rows, p_R->num_cols);
}


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
rob_apply_euler_angles(rob_frame_t* p_F, rob_point_t* p_srcp, rob_point_t* p_dstp, rob_euler_angles_t euler_tag, float* p_theta)
{
    /**
     *  Switch-case to execute transformations based onthe euler_tag introduced
     *  
     *  For each case, sequence goes as follows:
     *      - rotate first axis,
     *      - apply transformation
     *      - repeat previous steps for the following axis
     */
    
    // A correction: add two more angle arguments, one for each rotation, otherwise all axis will be rotated exactly the same angle.

    switch(euler_tag)
    {
        case XYX:
            // X
            //printf("ROTX:\n");
            rob_rotx(p_F, p_theta);
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
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            //matf32_print(p_F->p_T);
            //matf32_print(p_dstp->p_v);
            break;

        case XZX:
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXY:
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXZ:
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYZ:
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XYZ:
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case XZY:
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YZX:
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case YXZ:
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZXY:
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;

        case ZYX:
            // Z
            rob_rotz(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // Y
            rob_roty(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            // X
            rob_rotx(p_F, p_theta);
            rob_apply_transform(p_F, p_srcp, p_dstp);
            break;    
    }
}

// UTILITY FUNCTIONS

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
