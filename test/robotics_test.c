
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "linsolve.h"
#include "robotat_robotics.h"
#include "robotics_test_results.h"

// --------------------------------------------------
// Reference frames and points
// --------------------------------------------------

// Frame: A -> B
rob_frame_t F_AB;

// Point A
float p_A_data[4] = {0, 0, 0, 0};
rob_point_t p_A;
matf32_t p_A_v;

// Point B
// p_B_data defined in robotics_test_results.h
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
// F_AB_v_data defined in robotics_test_results.h
matf32_t F_AB_v;




// Temporal matrices
float temp_T_data[MAX_MAT_SIZE];
matf32_t temp_T;

float temp_R_data[MAX_MAT_SIZE];
matf32_t temp_R;

// --------------------------------------------------
// Quaternions
// --------------------------------------------------

float q1_norm = 0;
// q1_data defined in robotics_test_results.h
rob_quat_t q1;

float uq1_norm = 0;
// uq1_data defined in robotics_test_results.h
rob_quat_t uq1;

float q2_data[4] = {0,0,0,0};
rob_quat_t q2;

float temp_q_norm = 0;
float temp_q_data[4] = {0, 0, 0, 0};
rob_quat_t temp_q;

// --------------------------------------------------
// Roll-Pitch-Yaw and Euler Angles
// --------------------------------------------------
// All angles are defined in robotics_test_results.h


// --------------------------------------------------
// Other variables
// --------------------------------------------------

bool angle_units = 0; // 0: Radians, 1: Degrees

// --------------------------------------------------
// Results from Matlab
// --------------------------------------------------

// Data arrays for result matrices defined in robotics_test_results.h

// Homogeneous transformation results

matf32_t R_transl;
matf32_t R_rotx;
matf32_t R_roty;
matf32_t R_rotz;
matf32_t R_trotx;
matf32_t R_troty;
matf32_t R_trotz;
matf32_t R_transform;
matf32_t R_inv_transform;

// Quaternion operation results

rob_quat_t R_quat_add;
rob_quat_t R_quat_sub;
rob_quat_t R_quat_scale;
rob_quat_t R_quat_mul;
rob_quat_t R_quat_conj;
rob_quat_t R_quat_inv;

// Conversions results

matf32_t R_r2t;
matf32_t R_t2r;
matf32_t R_rpy2r;
matf32_t R_rpy2tr;
matf32_t R_rt2tr;
matf32_t R_eul2r;
matf32_t R_eul2tr;
matf32_t R_q2r;
matf32_t R_q2tr;

rob_quat_t R_rpy2q;
rob_quat_t R_eul2q;
rob_quat_t R_r2q;
rob_quat_t R_tr2q;

float temp_roll = 0;
float temp_pitch = 0;
float temp_yaw = 0;

float temp_phi = 0;
float temp_theta = 0;
float temp_psi = 0;

int main(void)
{
    clock_t time;

    rob_status_t rob_status;

    // Initialize matrices for Frame AB
    matf32_init(&F_AB_T, 4, 4, F_AB_T_data);  // Homogeneous transformation matrix
    matf32_init(&F_AB_R, 3, 3, F_AB_R_data);  // Rotation matrix
    matf32_init(&F_AB_v, 3, 1, F_AB_v_data);  // Coordinates vector for the transformation

    matf32_init(&temp_T, 4, 4, temp_T_data);
    matf32_init(&temp_R, 3, 3, temp_R_data);

    // Initialize reference frame
    // Note: last argument 0=radians, 1=degrees
    rob_frame_init(&F_AB, &F_AB_T, &F_AB_R, &F_AB_v, FRAME_A, FRAME_B, angle_units);

    // Initialize vector points
    rob_refpoint_init(&p_A, &p_A_v, p_A_data, FRAME_A);
    rob_refpoint_init(&p_B, &p_B_v, p_B_data, FRAME_B);
    
    // Initialize quaternions
    rob_quat_init(&q1, &q1_data[0], &q1_data[1], &q1_data[2], &q1_data[3]);
    rob_quat_init(&temp_q, &temp_q_data[0], &temp_q_data[1], &temp_q_data[2], &temp_q_data[3]);
    rob_unitquat_init(&uq1, &uq1_data[0], &uq1_data[1], &uq1_data[2], &uq1_data[3]);
    rob_quat_init(&q2, &q2_data[0], &q2_data[1], &q2_data[2], &q2_data[3]);

    // ---------------------------------------------------------------------------
    // Matlab results initializations
    // ---------------------------------------------------------------------------

    // Matrices and vectors
    matf32_init(&R_transl, 4, 4, R_transl_data);
    matf32_init(&R_rotx, 3, 3, R_rotx_data);
    matf32_init(&R_roty, 3, 3, R_roty_data);
    matf32_init(&R_rotz, 3, 3, R_rotz_data);
    matf32_init(&R_trotx, 4, 4, R_trotx_data);
    matf32_init(&R_troty, 4, 4, R_troty_data);
    matf32_init(&R_trotz, 4, 4, R_trotz_data);
    matf32_init(&R_transform, 4, 1, R_transform_data);
    matf32_init(&R_inv_transform, 4, 4, R_inv_transform_data);
    matf32_init(&R_r2t, 4, 4, R_r2t_data);
    matf32_init(&R_t2r, 3, 3, R_t2r_data);
    matf32_init(&R_rpy2r, 3, 3, R_rpy2r_data);
    matf32_init(&R_rpy2tr, 4, 4, R_rpy2tr_data);
    matf32_init(&R_rt2tr, 4, 4, R_rt2tr_data);
    matf32_init(&R_eul2r, 3, 3, R_eul2r_data);
    matf32_init(&R_eul2tr, 4, 4, R_eul2tr_data);
    matf32_init(&R_q2r, 3, 3, R_q2r_data);
    matf32_init(&R_q2tr, 4, 4, R_q2tr_data);

    // Quaternions
    rob_quat_init(&R_quat_add, &R_quat_add_data[0], &R_quat_add_data[1], &R_quat_add_data[2], &R_quat_add_data[3]);
    rob_quat_init(&R_quat_sub, &R_quat_sub_data[0], &R_quat_sub_data[1], &R_quat_sub_data[2], &R_quat_sub_data[3]);
    rob_quat_init(&R_quat_scale, &R_quat_scale_data[0], &R_quat_scale_data[1], &R_quat_scale_data[2], &R_quat_scale_data[3]);
    rob_quat_init(&R_quat_mul, &R_quat_mul_data[0], &R_quat_mul_data[1], &R_quat_mul_data[2], &R_quat_mul_data[3]);
    rob_quat_init(&R_quat_conj, &R_quat_conj_data[0], &R_quat_conj_data[1], &R_quat_conj_data[2], &R_quat_conj_data[3]);
    rob_quat_init(&R_quat_inv, &R_quat_inv_data[0], &R_quat_inv_data[1], &R_quat_inv_data[2], &R_quat_inv_data[3]);
    rob_quat_init(&R_rpy2q, &R_rpy2q_data[0], &R_rpy2q_data[1], &R_rpy2q_data[2], &R_rpy2q_data[3]);
    rob_quat_init(&R_eul2q, &R_eul2q_data[0], &R_eul2q_data[1], &R_eul2q_data[2], &R_eul2q_data[3]);
    rob_quat_init(&R_r2q, &R_r2q_data[0], &R_r2q_data[1], &R_r2q_data[2], &R_r2q_data[3]);
    rob_quat_init(&R_tr2q, &R_tr2q_data[0], &R_tr2q_data[1], &R_tr2q_data[2], &R_tr2q_data[3]);

    printf("\n\n");

    // ---------------------------------------------------------------------------
    // 1. rob_transl
    // ---------------------------------------------------------------------------

    float transl_time[x]; // Raw time data
    float transl_mean_time = 0;

    // Note about these two lines:
    // My implementation of rob_transl just assigns the coordinates vector into the T matrix.
    // On the other hand, Robotics Toolbox's transl creates a 4x4 T matrix, and sets the rotation matrix to an 3x3 identity matrix.
    // For that difference, if I don't reset the rotation matrix as an identity using matf32_t, the comparison won't match.
    // The implementation is different, but the important thing is the coordinastes vector is assigned correctly.
    matf32_eye(F_AB.p_R);   // Set R to an identity matrix
    rob_rot2tr(F_AB.p_R, &F_AB); // Set rotation matrix to the homogeneous transformation matrix

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_transl(F_AB.p_v, &F_AB);
        transl_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_transl\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", transl_time[j]);
    //}
    //printf("\n\n");

    transl_mean_time = mean(transl_time, x);

    bool transl_ans = matf32_is_equal(F_AB.p_T, &R_transl);
    printf("transl          ,%s,mean_time(s): %.9f\n", transl_ans?"success":"failure", transl_mean_time);

    // ---------------------------------------------------------------------------
    // 2. rob_rotx
    // ---------------------------------------------------------------------------

    float rotx_time[x]; // Raw time data
    float rotx_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rotx(F_AB.p_R, theta, 0);
        rotx_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rotx\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rotx_time[j]);
    //}
    //printf("\n\n");

    rotx_mean_time = mean(rotx_time, x);

    bool rotx_ans = matf32_is_equal(F_AB.p_R, &R_rotx);
    printf("rotx            ,%s,mean_time(s): %.9f\n", rotx_ans?"success":"failure", rotx_mean_time);

    // ---------------------------------------------------------------------------
    // 3. rob_roty
    // ---------------------------------------------------------------------------

    float roty_time[x]; // Raw time data
    float roty_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_roty(F_AB.p_R, theta, 0);
        roty_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_roty\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", roty_time[j]);
    //}
    //printf("\n\n");

    roty_mean_time = mean(roty_time, x);

    bool roty_ans = matf32_is_equal(F_AB.p_R, &R_roty);
    printf("roty            ,%s,mean_time(s): %.9f\n", roty_ans?"success":"failure", roty_mean_time);

    // ---------------------------------------------------------------------------
    // 4. rob_rotz
    // ---------------------------------------------------------------------------

    float rotz_time[x]; // Raw time data
    float rotz_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rotz(F_AB.p_R, theta, 0);
        rotz_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rotz\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rotz_time[j]);
    //}
    //printf("\n\n");

    rotz_mean_time = mean(rotz_time, x);

    bool rotz_ans = matf32_is_equal(F_AB.p_R, &R_rotz);
    printf("rotz            ,%s,mean_time(s): %.9f\n", rotz_ans?"success":"failure", rotz_mean_time);

    // ---------------------------------------------------------------------------
    // 5. rob_trotx
    // ---------------------------------------------------------------------------

    float trotx_time[x]; // Raw time data
    float trotx_mean_time = 0;

    // Set coordinates vector to zero because it was modified earlier and trotx only changes the rotation matrix
    // Manual set to zero to avoid messing with the coordinates vector's original values. 
    // In matlab, trotx was tested without changing the translation, so the vector is zeros.
    matf32_set(F_AB.p_T, 1, 4, 0);
    matf32_set(F_AB.p_T, 2, 4, 0);
    matf32_set(F_AB.p_T, 3, 4, 0);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_trotx(&F_AB, theta, angle_units);
        trotx_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_trotx\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", trotx_time[j]);
    //}
    //printf("\n\n");

    trotx_mean_time = mean(trotx_time, x);

    bool trotx_ans = matf32_is_equal(F_AB.p_T, &R_trotx);
    printf("trotx           ,%s,mean_time(s): %.9f\n", trotx_ans?"success":"failure", trotx_mean_time);

    // ---------------------------------------------------------------------------
    // 6. rob_troty
    // ---------------------------------------------------------------------------

    float troty_time[x]; // Raw time data
    float troty_mean_time = 0;

    // The coordinates vector remains unchanged since last operation in F_AB, so there's no need to modify it like before trotx.

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_troty(&F_AB, theta, angle_units);
        troty_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_troty\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", troty_time[j]);
    //}
    //printf("\n\n");

    troty_mean_time = mean(troty_time, x);

    bool troty_ans = matf32_is_equal(F_AB.p_T, &R_troty);
    printf("troty           ,%s,mean_time(s): %.9f\n", troty_ans?"success":"failure", troty_mean_time);

    // ---------------------------------------------------------------------------
    // 7. rob_trotz
    // ---------------------------------------------------------------------------

    float trotz_time[x]; // Raw time data
    float trotz_mean_time = 0;

    // The coordinates vector remains unchanged since last operation in F_AB, so there's no need to modify it like before trotx.

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_trotz(&F_AB, theta, angle_units);
        trotz_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_trotz\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", trotz_time[j]);
    //}
    //printf("\n\n");

    trotz_mean_time = mean(trotz_time, x);

    bool trotz_ans = matf32_is_equal(F_AB.p_T, &R_trotz);
    printf("trotz           ,%s,mean_time(s): %.9f\n", trotz_ans?"success":"failure", trotz_mean_time);

    // ---------------------------------------------------------------------------
    // 8. rob_apply_transform
    // ---------------------------------------------------------------------------

    float apply_transform_time[x]; // Raw time data
    float apply_transform_mean_time = 0;

    // Reset translation because I had turned it to zeros for trotx
    // Update homogeneous transformation matrix with both rotx and transl to match matlab's
    matf32_init(&F_AB_v, 3, 1, F_AB_v_data);
    rob_rotx(F_AB.p_R, theta, angle_units);
    rob_update_transform(&F_AB, F_AB.p_R, F_AB.p_v);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_apply_transform(&F_AB, &p_B, &p_A);
        apply_transform_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_apply_transform\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", apply_transform_time[j]);
    //}
    //printf("\n\n");

    apply_transform_mean_time = mean(apply_transform_time, x);

    bool apply_transform_ans = matf32_is_equal(p_A.p_v, &R_transform);
    printf("apply_transform ,%s,mean_time(s): %.9f\n", apply_transform_ans?"success":"failure", apply_transform_mean_time);

    // ---------------------------------------------------------------------------
    // 9. rob_inv_transform
    // ---------------------------------------------------------------------------

    float inv_transform_time[x]; // Raw time data
    float inv_transform_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_inv_transform(&F_AB, &temp_T);
        inv_transform_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_inv_transform\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", inv_transform_time[j]);
    //}
    //printf("\n\n");

    inv_transform_mean_time = mean(inv_transform_time, x);

    bool inv_transform_ans = matf32_is_equal(&temp_T, &R_inv_transform);
    printf("inv_transform   ,%s,mean_time(s): %.9f\n", inv_transform_ans?"success":"failure", inv_transform_mean_time);

    // ---------------------------------------------------------------------------
    // 10. rob_quat_add
    // ---------------------------------------------------------------------------

    float quat_add_time[x]; // Raw time data
    float quat_add_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_add(&q1, &q1, &temp_q);
        quat_add_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_add\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_add_time[j]);
    //}
    //printf("\n\n");

    quat_add_mean_time = mean(quat_add_time, x);

    bool quat_add_ans = rob_quat_is_equal(&temp_q, &R_quat_add);
    printf("quat_add        ,%s,mean_time(s): %.9f\n", quat_add_ans?"success":"failure", quat_add_mean_time);

    // ---------------------------------------------------------------------------
    // 11. rob_quat_sub
    // ---------------------------------------------------------------------------

    float quat_sub_time[x]; // Raw time data
    float quat_sub_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_sub(&q1, &q1, &temp_q);
        quat_sub_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_sub\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_sub_time[j]);
    //}
    //printf("\n\n");

    quat_sub_mean_time = mean(quat_sub_time, x);

    bool quat_sub_ans = rob_quat_is_equal(&temp_q, &R_quat_sub);
    printf("quat_sub        ,%s,mean_time(s): %.9f\n", quat_sub_ans?"success":"failure", quat_sub_mean_time);

    // ---------------------------------------------------------------------------
    // 12. rob_quat_scale
    // ---------------------------------------------------------------------------

    float quat_scale_time[x]; // Raw time data
    float quat_scale_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_scale(&q1, scalar, &temp_q);
        quat_scale_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_scale\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_scale_time[j]);
    //}
    //printf("\n\n");

    quat_scale_mean_time = mean(quat_scale_time, x);

    bool quat_scale_ans = rob_quat_is_equal(&temp_q, &R_quat_scale);
    printf("quat_scale      ,%s,mean_time(s): %.9f\n", quat_scale_ans?"success":"failure", quat_scale_mean_time);

    // ---------------------------------------------------------------------------
    // 13. rob_quat_mul
    // ---------------------------------------------------------------------------

    float quat_mul_time[x]; // Raw time data
    float quat_mul_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_mul(&q1, &q1, &temp_q);
        quat_mul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_mul\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_mul_time[j]);
    //}
    //printf("\n\n");

    quat_mul_mean_time = mean(quat_mul_time, x);

    bool quat_mul_ans = rob_quat_is_equal(&temp_q, &R_quat_mul);
    printf("quat_mul        ,%s,mean_time(s): %.9f\n", quat_mul_ans?"success":"failure", quat_mul_mean_time);

    // ---------------------------------------------------------------------------
    // 14. rob_quat_conj
    // ---------------------------------------------------------------------------

    float quat_conj_time[x]; // Raw time data
    float quat_conj_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_conj(&q1, &temp_q);
        quat_conj_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_conj\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_conj_time[j]);
    //}
    //printf("\n\n");

    quat_conj_mean_time = mean(quat_conj_time, x);

    bool quat_conj_ans = rob_quat_is_equal(&temp_q, &R_quat_conj);
    printf("quat_conj       ,%s,mean_time(s): %.9f\n", quat_conj_ans?"success":"failure", quat_conj_mean_time);

    // ---------------------------------------------------------------------------
    // 15. rob_quat_norm
    // ---------------------------------------------------------------------------

    float quat_norm_time[x]; // Raw time data
    float quat_norm_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat_norm(&q1, &q1_norm);
        quat_norm_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_norm\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_norm_time[j]);
    //}
    //printf("\n\n");

    quat_norm_mean_time = mean(quat_norm_time, x);
    
    bool quat_norm_ans = q1_norm==R_quat_norm_data?true:false;
    printf("quat_norm       ,%s,mean_time(s): %.9f\n", quat_norm_ans?"success":"failure", quat_norm_mean_time);

    // ---------------------------------------------------------------------------
    // 16. rob_quat_inv
    // ---------------------------------------------------------------------------
    
    // Note: seems to have issues with some combinations of values for the quaternion
    // For example, when q1 = 1 + 2i + 3j + 4k, as the j component ends up different

    float quat_inv_time[x]; // Raw time data
    float quat_inv_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_status = rob_quat_inv(&q2, &temp_q);
        quat_inv_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat_inv\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat_inv_time[j]);
    //}
    //printf("\n\n");

    quat_inv_mean_time = mean(quat_inv_time, x);

    bool quat_inv_ans = rob_quat_is_equal(&temp_q, &R_quat_inv);
    printf("quat_inv        ,%s,mean_time(s): %.9f\n", quat_inv_ans?"success":"failure", quat_inv_mean_time);

    // ---------------------------------------------------------------------------
    // 17. rob_rot2tr
    // ---------------------------------------------------------------------------

    float rot2tr_time[x]; // Raw time data
    float rot2tr_mean_time = 0;

    // Reset homogeneous transformation matrix to start fresh in this operation
    matf32_zeros(F_AB.p_R);
    rob_rotx(F_AB.p_R, theta, angle_units);
    matf32_set(F_AB.p_T, 1, 4, 0);
    matf32_set(F_AB.p_T, 2, 4, 0);
    matf32_set(F_AB.p_T, 3, 4, 0);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rot2tr(F_AB.p_R, &F_AB);
        rot2tr_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rot2tr\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rot2tr_time[j]);
    //}
    //printf("\n\n");

    rot2tr_mean_time = mean(rot2tr_time, x);
    
    bool rot2tr_ans = matf32_is_equal(F_AB.p_T, &R_r2t);
    printf("rot2tr          ,%s,mean_time(s): %.9f\n", rot2tr_ans?"success":"failure", rot2tr_mean_time);

    // ---------------------------------------------------------------------------
    // 18. rob_tr2rot
    // ---------------------------------------------------------------------------

    float tr2rot_time[x]; // Raw time data
    float tr2rot_mean_time = 0;

    // Turn rotation matrix to zero and then copy it again from the transformation matrix.
    // It works because those matrices are managed independently in the struct.
    matf32_zeros(F_AB.p_R);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_tr2rot(&F_AB, F_AB.p_R);
        tr2rot_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_tr2rot\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", tr2rot_time[j]);
    //}
    //printf("\n\n");

    tr2rot_mean_time = mean(tr2rot_time, x);
    
    bool tr2rot_ans = matf32_is_equal(F_AB.p_R, &R_t2r);
    printf("tr2rot          ,%s,mean_time(s): %.9f\n", tr2rot_ans?"success":"failure", tr2rot_mean_time);

    // ---------------------------------------------------------------------------
    // 19. rob_rpy2rot
    // ---------------------------------------------------------------------------

    float rpy2rot_time[x]; // Raw time data
    float rpy2rot_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rpy2rot(roll, pitch, yaw, XYZ, angle_units, F_AB.p_R);
        rpy2rot_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rpy2rot\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rpy2rot_time[j]);
    //}
    //printf("\n\n");

    rpy2rot_mean_time = mean(rpy2rot_time, x);
    
    bool rpy2rot_ans = matf32_is_equal(F_AB.p_R, &R_rpy2r);
    printf("rpy2rot         ,%s,mean_time(s): %.9f\n", rpy2rot_ans?"success":"failure", rpy2rot_mean_time);

    // ---------------------------------------------------------------------------
    // 20. rob_rpy2tr
    // ---------------------------------------------------------------------------

    float rpy2tr_time[x]; // Raw time data
    float rpy2tr_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rpy2tr(roll, pitch, yaw, XYZ, angle_units, &F_AB);
        rpy2tr_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rpy2tr\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rpy2tr_time[j]);
    //}
    //printf("\n\n");

    rpy2tr_mean_time = mean(rpy2tr_time, x);
    
    bool rpy2tr_ans = matf32_is_equal(F_AB.p_R, &R_rpy2r);
    printf("rpy2tr          ,%s,mean_time(s): %.9f\n", rpy2tr_ans?"success":"failure", rpy2tr_mean_time);

    // ---------------------------------------------------------------------------
    // 21. rob_update_transform (matlab equivalent is rt2tr)
    // ---------------------------------------------------------------------------

    float update_transform_time[x]; // Raw time data
    float update_transform_mean_time = 0;

    rob_rotx(F_AB.p_R, theta, angle_units);

    // Reset homogeneous transformation matrix to start fresh in this operation
    matf32_zeros(F_AB.p_R);
    rob_rotx(F_AB.p_R, theta, angle_units);
    matf32_set(F_AB.p_T, 1, 4, 0);
    matf32_set(F_AB.p_T, 2, 4, 0);
    matf32_set(F_AB.p_T, 3, 4, 0);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_update_transform(&F_AB, F_AB.p_R, F_AB.p_v);
        update_transform_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_update_transform\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", update_transform_time[j]);
    //}
    //printf("\n\n");

    update_transform_mean_time = mean(update_transform_time, x);
    
    bool update_transform_ans = matf32_is_equal(F_AB.p_T, &R_rt2tr);
    printf("update_transform,%s,mean_time(s): %.9f\n", update_transform_ans?"success":"failure", update_transform_mean_time);

    // ---------------------------------------------------------------------------
    // 22. rob_eul2rot
    // ---------------------------------------------------------------------------

    float eul2rot_time[x]; // Raw time data
    float eul2rot_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        // In robotics toolbox there seems only ZYZ is implemented for eul2r
        rob_eul2rot(phi, theta, psi, ZYZ, angle_units, F_AB.p_R);
        eul2rot_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_eul2rot\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", eul2rot_time[j]);
    //}
    //printf("\n\n");

    eul2rot_mean_time = mean(eul2rot_time, x);
    
    bool eul2rot_ans = matf32_is_equal(F_AB.p_R, &R_eul2r);
    printf("eul2rot         ,%s,mean_time(s): %.9f\n", eul2rot_ans?"success":"failure", eul2rot_mean_time);

    // ---------------------------------------------------------------------------
    // 23. rob_eul2tr
    // ---------------------------------------------------------------------------

    float eul2tr_time[x]; // Raw time data
    float eul2tr_mean_time = 0;

    // Reset homogeneous transformation matrix to avoid confusion of previous values
    matf32_zeros(F_AB.p_R);
    matf32_set(F_AB.p_T, 1, 4, 0);
    matf32_set(F_AB.p_T, 2, 4, 0);
    matf32_set(F_AB.p_T, 3, 4, 0);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_eul2tr(phi, theta, psi, ZYZ, angle_units, &F_AB);
        eul2tr_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_eul2tr\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", eul2tr_time[j]);
    //}
    //printf("\n\n");

    eul2tr_mean_time = mean(eul2tr_time, x);
    
    bool eul2tr_ans = matf32_is_equal(F_AB.p_T, &R_eul2tr);
    printf("eul2tr          ,%s,mean_time(s): %.9f\n", eul2tr_ans?"success":"failure", eul2tr_mean_time);

    // ---------------------------------------------------------------------------
    // 24. rob_rot2quat
    // ---------------------------------------------------------------------------
    
    float rot2quat_time[x]; // Raw time data
    float rot2quat_mean_time = 0;

    rob_rotx(F_AB.p_R, theta, angle_units);
    matf32_set(F_AB.p_v, 1, 1, 0);
    matf32_set(F_AB.p_v, 2, 1, 0);
    matf32_set(F_AB.p_v, 3, 1, 0);
    rob_update_transform(&F_AB, F_AB.p_R, F_AB.p_v);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rot2quat(F_AB.p_R, &temp_q);
        rot2quat_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rot2quat\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rot2quat_time[j]);
    //}
    //printf("\n\n");

    rot2quat_mean_time = mean(rot2quat_time, x);
    
    bool rot2quat_ans = rob_quat_is_equal(&temp_q, &R_r2q);
    printf("rot2quat        ,%s,mean_time(s): %.9f\n", rot2quat_ans?"success":"failure", rot2quat_mean_time);
    

    // ---------------------------------------------------------------------------
    // 25. rob_tr2quat
    // ---------------------------------------------------------------------------

    float tr2quat_time[x]; // Raw time data
    float tr2quat_mean_time = 0;

    rob_rotx(F_AB.p_R, theta, angle_units);
    rob_update_transform(&F_AB, F_AB.p_R, F_AB.p_v);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_tr2quat(&F_AB, &temp_q);
        tr2quat_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_tr2quat\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", tr2quat_time[j]);
    //}
    //printf("\n\n");

    tr2quat_mean_time = mean(tr2quat_time, x);
    
    bool tr2quat_ans = rob_quat_is_equal(&temp_q, &R_tr2q);
    printf("tr2quat         ,%s,mean_time(s): %.9f\n", tr2quat_ans?"success":"failure", tr2quat_mean_time);

    // ---------------------------------------------------------------------------
    // 26. rob_rpy2quat
    // ---------------------------------------------------------------------------

    float rpy2quat_time[x]; // Raw time data
    float rpy2quat_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_rpy2quat(roll, pitch, yaw, XYZ, angle_units, F_AB.p_R, &temp_q);
        rpy2quat_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_rpy2quat\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", rpy2quat_time[j]);
    //}
    //printf("\n\n");

    rpy2quat_mean_time = mean(rpy2quat_time, x);
    
    bool rpy2quat_ans = rob_quat_is_equal(&temp_q, &R_rpy2q);
    printf("rpy2quat        ,%s,mean_time(s): %.9f\n", rpy2quat_ans?"success":"failure", rpy2quat_mean_time);

    // ---------------------------------------------------------------------------
    // 27. rob_rpy2quat
    // ---------------------------------------------------------------------------

    float eul2quat_time[x]; // Raw time data
    float eul2quat_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_eul2quat(phi, theta, psi, ZYZ, angle_units, F_AB.p_R, &temp_q);
        eul2quat_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_eul2quat\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", eul2quat_time[j]);
    //}
    //printf("\n\n");

    eul2quat_mean_time = mean(eul2quat_time, x);
    
    bool eul2quat_ans = rob_quat_is_equal(&temp_q, &R_eul2q);
    printf("eul2quat        ,%s,mean_time(s): %.9f\n", eul2quat_ans?"success":"failure", eul2quat_mean_time);

    // ---------------------------------------------------------------------------
    // 28. rob_tr2rpy
    // ---------------------------------------------------------------------------

    float tr2rpy_time[x]; // Raw time data
    float tr2rpy_mean_time = 0;

    // Reset homogeneous transformation matrix to avoid confusion of previous values
    matf32_zeros(F_AB.p_R);
    matf32_set(F_AB.p_T, 1, 4, 0);
    matf32_set(F_AB.p_T, 2, 4, 0);
    matf32_set(F_AB.p_T, 3, 4, 0);
    rob_rpy2tr(roll, pitch, yaw, XYZ, angle_units, &F_AB);

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_tr2rpy(&F_AB, angle_units, XYZ, &temp_roll, &temp_pitch, &temp_yaw);
        tr2rpy_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }

    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_tr2rpy\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", tr2rpy_time[j]);
    //}
    //printf("\n\n");

    tr2rpy_mean_time = mean(tr2rpy_time, x);
    
    bool tr2rpy_ans = false;

    if (is_equal_margin(R_tr2rpy_roll, temp_roll) && is_equal_margin(R_tr2rpy_pitch, temp_pitch) && is_equal_margin(R_tr2rpy_yaw, temp_yaw))
    {
        tr2rpy_ans = true;
    }

    printf("tr2rpy          ,%s,mean_time(s): %.9f\n", tr2rpy_ans?"success":"failure", tr2rpy_mean_time);

    // ---------------------------------------------------------------------------
    // 29. rob_tr2eul
    // ---------------------------------------------------------------------------

    float tr2eul_time[x]; // Raw time data
    float tr2eul_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_tr2eul(&F_AB, angle_units, ZYZ, &temp_phi, &temp_theta, &temp_psi);
        tr2eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_tr2eul\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", tr2eul_time[j]);
    //}
    //printf("\n\n");

    tr2eul_mean_time = mean(tr2eul_time, x);
    
    bool tr2eul_ans = false;

    if (is_equal_margin(R_tr2eul_phi, temp_phi) && is_equal_margin(R_tr2eul_theta, temp_theta) && is_equal_margin(R_tr2eul_psi, temp_psi))
    {
        tr2eul_ans = true;
    }

    printf("tr2eul          ,%s,mean_time(s): %.9f\n", tr2eul_ans?"success":"failure", tr2eul_mean_time);

    // ---------------------------------------------------------------------------
    // 30. rob_quat2rot
    // ---------------------------------------------------------------------------

    float quat2rot_time[x]; // Raw time data
    float quat2rot_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat2rot(&uq1, F_AB.p_R);
        quat2rot_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat2rot\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat2rot_time[j]);
    //}
    //printf("\n\n");

    quat2rot_mean_time = mean(quat2rot_time, x);
    
    bool quat2rot_ans = matf32_is_equal(F_AB.p_R, &R_q2r);
    printf("quat2rot        ,%s,mean_time(s): %.9f\n", quat2rot_ans?"success":"failure", quat2rot_mean_time);

    // ---------------------------------------------------------------------------
    // 31. rob_quat2tr
    // ---------------------------------------------------------------------------

    float quat2tr_time[x]; // Raw time data
    float quat2tr_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat2tr(&uq1, &F_AB);
        quat2tr_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat2tr\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat2tr_time[j]);
    //}
    //printf("\n\n");

    quat2tr_mean_time = mean(quat2tr_time, x);
    
    bool quat2tr_ans = matf32_is_equal(&F_AB_T, &R_q2tr);
    printf("quat2tr         ,%s,mean_time(s): %.9f\n", quat2tr_ans?"success":"failure", quat2tr_mean_time);

    // ---------------------------------------------------------------------------
    // 32. rob_quat2rpy
    // ---------------------------------------------------------------------------

    float quat2rpy_time[x]; // Raw time data
    float quat2rpy_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat2rpy(&uq1, angle_units, ZYX, &temp_roll, &temp_pitch, &temp_yaw);
        quat2rpy_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat2rpy\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat2rpy_time[j]);
    //}
    //printf("\n\n");

    quat2rpy_mean_time = mean(quat2rpy_time, x);
    
    bool quat2rpy_ans = false;

    if (is_equal_margin(R_q2rpy_roll, temp_roll) && is_equal_margin(R_q2rpy_pitch, temp_pitch) && is_equal_margin(R_q2rpy_yaw, temp_yaw))
    {
        quat2rpy_ans = true;
    }

    printf("quat2rpy        ,%s,mean_time(s): %.9f\n", quat2rpy_ans?"success":"failure", quat2rpy_mean_time);

    // ---------------------------------------------------------------------------
    // 33. rob_quat2eul
    // ---------------------------------------------------------------------------

    float quat2eul_time[x]; // Raw time data
    float quat2eul_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        time = clock();
        rob_quat2eul(&uq1, angle_units, ZYZ, &temp_phi, &temp_theta, &temp_psi);
        quat2eul_time[i] = (float)(clock()-time)/CLOCKS_PER_SEC;
    }
    
    // Print time values formatted to a .mat for plotting in matlab
    //printf("rob_quat2eul\n");
    //for (uint8_t j = 0; j < x; j++)
    //{
    //    printf("%.9f ", quat2eul_time[j]);
    //}
    //printf("\n\n");

    quat2eul_mean_time = mean(quat2eul_time, x);
    
    bool quat2eul_ans = false;

    if (is_equal_margin(R_q2eul_phi, temp_phi) && is_equal_margin(R_q2eul_theta, temp_theta) && is_equal_margin(R_q2eul_psi, temp_psi))
    {
        quat2eul_ans = true;
    }

    printf("quat2eul        ,%s,mean_time(s): %.9f\n", quat2eul_ans?"success":"failure", quat2eul_mean_time);
}