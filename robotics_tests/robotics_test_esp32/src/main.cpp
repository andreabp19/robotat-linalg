
/**
 * @author Andrea Pineda
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "matf32.h"
#include "linsolve.h"
#include "robotat_robotics.h"
#include "robotics_test_results.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------

// Frame: A -> B
rob_frame_t F_AB;

// Homogeneous transformation matrix
float F_AB_T_data[MAX_MAT_SIZE];
matf32_t F_AB_T;

// Rotation matrix
float F_AB_R_data[MAX_MAT_SIZE];
matf32_t F_AB_R;

// Coordinates vector
matf32_t F_AB_v;

// Point A
float p_A_data[4] = {0, 0, 0, 0};
rob_point_t p_A;
matf32_t p_A_v;

// Point B
rob_point_t p_B;
matf32_t p_B_v;

// Time variables
unsigned long tic = 0;
unsigned long toc = 0;

// Result values
matf32_t R_rotx;
matf32_t R_roty;
matf32_t R_rotz;
matf32_t R_transl;
matf32_t R_homtrans;
matf32_t R_xyx;

void setup()
{
    // ---------------------------------------------------------------------------
    // SERIAL CONFIG
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // FUNCTIONS TESTING
    // ---------------------------------------------------------------------------
    Serial.println("\n\n--------------------------------------------------");
    Serial.println("ROBOTICS");
    Serial.println("--------------------------------------------------\n\n");

    // Follow this pattern of tags, it0s easier to understand.
    // F_AB_T, F_AB_R, F_AB_v
    // p_A, p_A_v, p_A_data

    // Initialize matrices
    matf32_init(&F_AB_T, 4, 4, F_AB_T_data);  // Homogeneous transformation matrix
    matf32_init(&F_AB_R, 3, 3, F_AB_R_data);  // Rotation matrix
    matf32_init(&F_AB_v, 3, 1, F_AB_v_data);  // Coordinates vector for the transformation

    // Matrices for matlab results
    matf32_init(&R_rotx, 3, 3, R_rotx_data);
    matf32_init(&R_roty, 3, 3, R_roty_data);
    matf32_init(&R_rotz, 3, 3, R_rotz_data);
    matf32_init(&R_transl, 4, 4, R_transl_data);
    matf32_init(&R_homtrans, 4, 1, R_homtrans_data);
    matf32_init(&R_xyx, 4, 1, R_xyx_data);

    // Initialize reference frame
    // Note: last argument 0=radians, 1=degrees
    rob_frame_init(&F_AB, &F_AB_T, &F_AB_R, &F_AB_v, FRAME_A, FRAME_B, 0);

    // Initialize vector points
    // Put the first line into the refpoint init
    rob_refpoint_init(&p_A, &p_A_v, p_A_data, FRAME_A);
    rob_refpoint_init(&p_B, &p_B_v, p_B_data, FRAME_B);

    // ---------------------------------------------------------------------------
    // 1. rob_rotx
    // ---------------------------------------------------------------------------

    float rotx_time[x]; // Raw time data
    float rotx_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        tic = micros();
        rob_rotx(&F_AB, &theta);
        toc = micros() - tic;
        rotx_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_rotx\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", rotx_time[j]/1000000.0);
    }

    printf("\n\n");

    rotx_mean_time = mean(rotx_time, x);

    bool rotx_ans = matf32_is_equal(F_AB.p_R, &R_rotx);
    //Serial.printf("rotx,%s, mean time: %.9f\n", rotx_ans?"success":"failure", rotx_mean_time/1000000.0);

    // ---------------------------------------------------------------------------
    // 2. rob_roty
    // ---------------------------------------------------------------------------

    float roty_time[x]; // Raw time data
    float roty_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        tic = micros();
        rob_roty(&F_AB, &theta);
        toc = micros() - tic;
        roty_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_roty\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", roty_time[j]/1000000.0);
    }

    printf("\n\n");

    roty_mean_time = mean(roty_time, x);

    bool roty_ans = matf32_is_equal(F_AB.p_R, &R_roty);
    //Serial.printf("roty,%s, mean time: %.9f\n", roty_ans?"success":"failure", roty_mean_time/1000000.0);

    // ---------------------------------------------------------------------------
    // 3. rob_rotz
    // ---------------------------------------------------------------------------

    float rotz_time[x]; // Raw time data
    float rotz_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        tic = micros();
        rob_rotz(&F_AB, &theta);
        toc = micros() - tic;
        rotz_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_rotz\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", rotz_time[j]/1000000.0);
    }

    printf("\n\n");

    rotz_mean_time = mean(rotz_time, x);

    bool rotz_ans = matf32_is_equal(F_AB.p_R, &R_rotz);
    //Serial.printf("rotz,%s, mean time: %.9f\n", rotz_ans?"success":"failure", rotz_mean_time/1000000.0);

    // ---------------------------------------------------------------------------
    // 4. rob_transl
    // ---------------------------------------------------------------------------

    float transl_time[x]; // Raw time data
    float transl_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        // Note about these two lines:
        // My implementation of rob_transl just assigns the coordinates vector into the T matrix.
        // On the other hand, Robotics Toolbox's transl creates a 4x4 T matrix, and sets the rotation matrix to an 3x3 identity matrix.
        // For that difference, if I don't reset the rotation matrix as an identity using matf32_t, the comparison won't match.
        // The implementation is different, but the important thing is the coordinastes vector is assigned correctly.
        matf32_eye(F_AB.p_R);   // Set R to an identity matrix
        matf32_submatrix_copy(F_AB.p_R, F_AB.p_T, 0, 0, 0, 0, F_AB.p_R->num_rows, F_AB.p_R->num_cols);  // Set the new rotation matrix in T.

        tic = micros();
        rob_transl(&F_AB, &F_AB_v);
        toc = micros() - tic;
        transl_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_transl\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", transl_time[j]/1000000.0);
    }

    printf("\n\n");

    transl_mean_time = mean(transl_time, x);

    bool transl_ans = matf32_is_equal(F_AB.p_T, &R_transl);
    //Serial.printf("transl,%s, mean time: %.9f\n", transl_ans?"success":"failure", transl_mean_time/1000000.0);

    // ---------------------------------------------------------------------------
    // 5. rob_apply_transform
    // ---------------------------------------------------------------------------

    float homtrans_time[x]; // Raw time data
    float homtrans_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        // Rotx because I had previously changed the rotation matrix, so it has to match what was used on matlab.
        rob_rotx(&F_AB, &theta);

        tic = micros();
        rob_apply_transform(&F_AB, &p_B, &p_A);
        toc = micros() - tic;
        homtrans_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_apply_transform\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", homtrans_time[j]/1000000.0);
    }

    printf("\n\n");

    homtrans_mean_time = mean(homtrans_time, x);

    bool homtrans_ans = matf32_is_equal(p_A.p_v, &R_homtrans);
    //Serial.printf("homtrans,%s, mean time: %.9f\n", homtrans_ans?"success":"failure", homtrans_mean_time/1000000.0);

    // ---------------------------------------------------------------------------
    // 6. rob_apply_euler_angles
    // ---------------------------------------------------------------------------

    float xyx_time[x]; // Raw time data
    float xyx_mean_time = 0;

    for (uint8_t i = 0; i < x; i++)
    {
        tic = micros();
        rob_apply_euler_angles(&F_AB, &p_B, &p_A, XYX, &theta);
        toc = micros() - tic;
        xyx_time[i] = (float)toc;
    }

    // Print time values formatted to a .mat for plotting in matlab
    printf("rob_apply_euler_angles\n");
    
    for (uint8_t j = 0; j < x; j++)
    {
        printf("%.9f ", xyx_time[j]/1000000.0);
    }

    printf("\n\n");

    xyx_mean_time = mean(xyx_time, x);

    bool xyx_ans = matf32_is_equal(p_A.p_v, &R_xyx);
    //Serial.printf("xyx,%s, mean time: %.9f\n", xyx_ans?"success":"failure", xyx_mean_time/1000000.0);
}

void loop()
{
    
}