
/**
 * @author Andrea Pineda
 * @date Created 6 Jan. 2026
 * Last modified 6 Jan. 2026
 * 
 * Robotics demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "robotat_robotics.h"

// ---------------------------------------------------------------------------
// Definitions
// ---------------------------------------------------------------------------

// --------------------------------------------------
// Reference frames and points
// --------------------------------------------------

// Reference frames: A -> B
rob_frame_t F_AB;

// Reference Point in A
float p_A_data[4];
rob_point_t p_A;
matf32_t p_A_v;

// Reference Point in B
float p_B_data[4];
rob_point_t p_B;
matf32_t p_B_v;

// --------------------------------------------------
// Homogeneous transformation matrices and vectors
// --------------------------------------------------

// Frame AB

// Homogeneous transformation matrix AB
float F_AB_T_data[MAX_MAT_SIZE];
matf32_t F_AB_T;

// Rotation matrix AB
float F_AB_R_data[MAX_MAT_SIZE];
matf32_t F_AB_R;

// Coordinates vector AB
float F_AB_v_data[MAX_MAT_SIZE];
matf32_t F_AB_v;

// --------------------------------------------------
// Quaternions
// --------------------------------------------------

float q1_data[4] = {1, 2, 3, 4};
rob_quat_t q1;

float q2_data[4] = {5, 6, 7, 8};
rob_quat_t q2;

float q3_data[4] = {0, 0, 0, 0};
rob_quat_t q3;

// --------------------------------------------------
// Cardan and Euler Angles
// --------------------------------------------------
float roll = 0.2;
float pitch = 0.5;
float yaw = 0.75;
float phi = 0.5;
float theta = 0.4;

bool angle_units = 0; // 0: Radians, 1: Degrees

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // ---------------------------------------------------------------------------
    // Initializations
    // ---------------------------------------------------------------------------

    // Reference Frame AB
    matf32_init(&F_AB_T, 4, 4, F_AB_T_data);  // Homogeneous transformation matrix
    matf32_init(&F_AB_R, 3, 3, F_AB_R_data);  // Rotation matrix
    matf32_init(&F_AB_v, 3, 1, F_AB_v_data);  // Coordinates vector for the transformation

    rob_frame_init(&F_AB, &F_AB_T, &F_AB_R, &F_AB_v, FRAME_A, FRAME_B, angle_units);

    // Initialize reference points in A and B
    rob_refpoint_init(&p_A, &p_A_v, p_A_data, FRAME_A);
    rob_refpoint_init(&p_B, &p_B_v, p_B_data, FRAME_B);

    // Quaternions
    rob_quat_init(&q1, &q1_data[0], &q1_data[1], &q1_data[2], &q1_data[3]);
    rob_quat_init(&q2, &q2_data[0], &q2_data[1], &q2_data[2], &q2_data[3]);
    rob_quat_init(&q3, &q3_data[0], &q3_data[1], &q3_data[2], &q3_data[3]);

    // ---------------------------------------------------------------------------
    // Example operations
    // ---------------------------------------------------------------------------

    // 1. Print data of the reference frame
    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("Original frame (empty matrices)\n");
    printf("----------------------------------------------------------------------------------------------------\n");
    rob_frame_print(&F_AB);

    // 2. Generate rotation matrix and coordinates vector
    // Several examples for generating rotation matrices, comment/uncomment to test them as you need

    matf32_randn(F_AB.p_v, 0, 1); // Coordinates vector with random values

    // 2.1 Rotation around a single axis 
    //rob_rotx(F_AB.p_R, theta, 0); // X-axis, from a single angle, radians

    // 2.2 Rotation from Euler or Cardan angles
    //rob_eul2rot(phi, theta, phi, XYX, 0, F_AB.p_R); // Euler angles, 0=radians
    rob_rpy2rot(roll, pitch, yaw, XYZ, 0, F_AB.p_R); // Cardan angles, 0=radians

    // Print the frame's data again, matrix R is changed with the rotation.
    // Matrix T (the homogeneous transformation) is not updated yet with R.
    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("2. Generated rotation matrix R, still not updated homogeneous transformation matrix T \n");
    printf("----------------------------------------------------------------------------------------------------\n");
    rob_frame_print(&F_AB);

    // 3. Update homogeneous transformation matrix

    rob_update_transform(&F_AB, F_AB.p_R, F_AB.p_v);

    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("3. Updated homogeneous transformation matrix with T and v \n");
    printf("----------------------------------------------------------------------------------------------------\n");
    rob_frame_print(&F_AB);

    // 4. Generate data for reference points

    // Both points with random values, just as an example.
    // The last element must remain 1, as initialized, so it's recommended to modify the other values manually
    p_A.p_v->p_data[0] = 1;
    p_A.p_v->p_data[1] = 2;
    p_A.p_v->p_data[2] = 3;
    // p_data[3] = 1, don't change it

    p_B.p_v->p_data[0] = 4;
    p_B.p_v->p_data[1] = 5;
    p_B.p_v->p_data[2] = 6;
    // p_data[3] = 1, don't change it

    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("4. Reference points for frames A and B \n");
    printf("----------------------------------------------------------------------------------------------------\n");

    rob_refpoint_print(&p_A);
    rob_refpoint_print(&p_B);

    // 5. Apply the homogeneous transformation

    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("5. Apply the homogeneous transformation \n");
    printf("----------------------------------------------------------------------------------------------------\n");

    // 5.1 Using rob_apply_transform: takes the frame instance and the two points
    printf("\nPoint B before the transform is applied:\n");
    rob_refpoint_print(&p_B);
    rob_status_t status;
    status = rob_apply_transform(&F_AB, &p_A, &p_B);
    printf("\nPoint B transformed:\n");
    rob_status_print(status);
    rob_refpoint_print(&p_B); // Point B will be changed as it's the destination

    // 5.2 Using rob_apply_rot_sequence: generates a rotation and applies the transformation immediately
    // rob_apply_rot_sequence works with both Euler and Cardan angles
    status = rob_apply_rot_sequence(&F_AB, &p_A, &p_B, XYZ, phi, theta, phi, 0); // 0=radians
    printf("\nPoint B transformed with Euler or Cardan angles:\n");
    rob_status_print(status);
    rob_refpoint_print(&p_B); // Point B will be changed as it's the destination

    // 5.3 Using a quaternion: converts the quaternion to a rotation matrix, then applies the transform.
    status = rob_quat_apply_transform(&q1, &F_AB, &p_A, &p_B);
    printf("\nPoint B transformed with a quaternion:\n");
    rob_status_print(status);
    rob_refpoint_print(&p_B); // Point B will be changed as it's the destination

    // Before applying the transform, the frame IDs are checked for the right order of operation
    // If the frames don't match the right order, an error message is return and the operation aborted.

    // 6. Basic quaternion operations

    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("6. Basic quaternion operations \n");
    printf("----------------------------------------------------------------------------------------------------\n");

    printf("q1:\n");
    rob_quat_print(&q1);
    printf("q2:\n");
    rob_quat_print(&q2);
    printf("q3:\n");
    rob_quat_print(&q3);

    rob_quat_add(&q1, &q2, &q3);
    printf("q3 = q1 + q2:\n");
    rob_quat_print(&q3);

    rob_quat_sub(&q1, &q2, &q3);
    printf("q3 = q1 - q2:\n");
    rob_quat_print(&q3);

    rob_quat_mul(&q1, &q2, &q3);
    printf("q3 = q1 * q2:\n");
    rob_quat_print(&q3);

    rob_quat_inv(&q1, &q3);
    printf("q3 = inv(q1):\n");
    rob_quat_print(&q3);

    rob_quat_conj(&q1, &q3);
    printf("q3 = conj(q1):\n");
    rob_quat_print(&q3);

    // 7. About conversion routines
    // Use the following legend:
    // - rot = rotation matrix
    // - tr = homogeneous transformation matrix
    // - quat = quaternion
    // - eul = Euler angles
    // - rpy = Cardan angles
    // Example: rob_rot2quat = rotation matrix to quaternion.
    // All conversion routines follow the same pattern
    // Be mindful that some value combinations might not be easily converted, always check them before.

    printf("\n----------------------------------------------------------------------------------------------------\n");
    printf("7. Example of conversion routines \n");
    printf("----------------------------------------------------------------------------------------------------\n");
    
    rob_roty(F_AB.p_R, theta, 0);
    printf("Rotation matrix:\n");
    matf32_print(F_AB.p_R);

    printf("rob_rot2quat: rotation matrix to quaternion\n");
    rob_rot2quat(F_AB.p_R, &q1);
    rob_quat_print(&q1);
}