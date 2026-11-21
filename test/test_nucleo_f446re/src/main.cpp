
/*
Test project for the STM32 Nucleo F446RE for branch build-test-perf
Author: Andrea Pineda
Date: 20 Nov. 2026, last modified: 20 Nov. 2026
*/

// ----------------------------------------------------------------------------------------------------
// Imported libraries and files
// ----------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <stdint.h>
#include "robotat_control.h"

#include "control_pid_test_data.h"
#include "control_kalman_filter_test_data.h"

// ----------------------------------------------------------------------------------------------------
// Variables and constants
// ----------------------------------------------------------------------------------------------------

// --------------------------------------------------
// PID
// --------------------------------------------------

ctr_pid_t pid;
ctr_sys_lti_t pid_sys;

uint32_t pid_time[SAMPLES];

float pid_u_history[SAMPLES];
float pid_x_history[SAMPLES];
float pid_y_history[SAMPLES];

float pid_A_data[MAX_MAT_SIZE];
matf32_t pid_A;

float pid_B_data[MAX_MAT_SIZE];
matf32_t pid_B;

float pid_C_data[MAX_MAT_SIZE];
matf32_t pid_C;

float pid_D_data[MAX_MAT_SIZE];
matf32_t pid_D;

float pid_x_data[MAX_MAT_SIZE];
matf32_t pid_x;

float pid_y_data[MAX_MAT_SIZE];
matf32_t pid_y;

float pid_u_data[MAX_MAT_SIZE];
matf32_t pid_u;

float Ax_data[MAX_MAT_SIZE];
matf32_t Ax;

float Bu_data[MAX_MAT_SIZE];
matf32_t Bu;

// --------------------------------------------------
// Kalman filter
// --------------------------------------------------

uint32_t kf_time[SAMPLES];

float kf_A_data[1] = {1};
matf32_t kf_A;

float kf_B_data[1] = {1};
matf32_t kf_B;

float kf_C_data[1] = {1};
matf32_t kf_C;

float kf_D_data[1] = {0};
matf32_t kf_D;

float kf_Qw_data[1] = {0};
matf32_t kf_Qw;

float kf_Qv_data[1] = {1};
matf32_t kf_Qv;

float kf_u_data[MAX_MAT_SIZE];
matf32_t kf_u;

float kf_x_data[MAX_MAT_SIZE];
matf32_t kf_x;

float kf_y_data[MAX_MAT_SIZE];
matf32_t kf_y;

float kf_xhat_data[MAX_MAT_SIZE];
matf32_t kf_xhat;

float kf_P_data[1] = {1};
matf32_t kf_P;

float kf_F_data[1] = {1};
matf32_t kf_F;

float kf_w_data[MAX_MAT_SIZE];
matf32_t kf_w;

float kf_v_data[MAX_MAT_SIZE];
matf32_t kf_v;

float Fw_data[MAX_MAT_SIZE];
matf32_t Fw;

#define K (100)

float sample_time = 0.1;

float xhat_history[K];
float x_history[K];
float u_history[K];
float y_history[K];

ctr_sys_lti_t kf_sys;
ctr_kalman_t kf;

// --------------------------------------------------
// Other Variables
// --------------------------------------------------

uint16_t n = SAMPLES - 1;

uint32_t t0 = 0;                  // Initial time
uint32_t t1 = 0;                  // Final time
uint32_t cycle_counter_dt = 0;    // Time interval
uint32_t overhead = 0;            // Cycle Counter overhead (calibration for time of measurement itself)
uint32_t overhead_meas[SAMPLES];  // To take several measurements of the overhead

uint32_t min_t = UINT32_MAX;      // Start at the max. possible value to reduce it to the minimum
uint32_t med_t = 0;
uint32_t max_t = 0;

void insertion_sort(uint32_t *a, uint16_t n);

void setup()
{
    // -------------------------------------------------------------------------------------
    // Serial config
    // -------------------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // -------------------------------------------------------------------------------------
    // Cycle Counter Config
    // -------------------------------------------------------------------------------------

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // Enable the trace/debug blocks (DWT)
    DWT->CYCCNT = 0;                                  // Reset the counter
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             // Start counting

    // -------------------------------------------------------------------------------------
    // Cycle Counter Test calibration and measurements
    // -------------------------------------------------------------------------------------

    // Cycle Counter Calibration -> Overhead measurement: time of measurement itself
    for (uint16_t i = 0; i < n; i++)
    {
        t0 = DWT->CYCCNT;
        //
        t1 = DWT->CYCCNT;

        overhead_meas[i] = t1 - t0;
    }
        
    insertion_sort(overhead_meas, n);
    overhead = overhead_meas[0];

    // Test measurement against the SystemCoreClock (check the Cycle Counter is enabled)

    t0 = DWT->CYCCNT; // Read Cycle Counter for initial time
    delay(1000); // Delay for test measurement
    t1 = DWT->CYCCNT; // Read Cycle Counter for final time
    cycle_counter_dt = t1 - t0;

    //Serial.println("cycle_counter_dt,overhead,SystemCoreClock");
    //Serial.print(cycle_counter_dt);
    //Serial.print(",");
    //Serial.print(overhead);
    //Serial.print(",");
    //Serial.println(SystemCoreClock);

    // -------------------------------------------------------------------------------------
    // PID Tustin test case
    // -------------------------------------------------------------------------------------

    ctr_sys_lti_init(&pid_sys, &pid_x, &pid_A, &pid_B, &pid_C, &pid_D, dt);

    matf32_init(&pid_A, 1, 1, pid_A_data);
    matf32_init(&pid_B, 1, 1, pid_B_data);
    matf32_init(&pid_C, 1, 1, pid_C_data);
    matf32_init(&pid_x, 1, 1, pid_x_data);
    matf32_init(&pid_y, 1, 1, pid_y_data);
    matf32_init(&pid_u, 1, 1, pid_u_data);

    matf32_init(&Ax, 1, 1, Ax_data);
    matf32_init(&Bu, 1, 1, Bu_data);

    matf32_set(&pid_A, 1, 1, 0.9f);
    matf32_set(&pid_B, 1, 1, 0.1f);
    matf32_set(&pid_C, 1, 1, 1);

    ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 0);
    pid.tau = tau;
    pid.dt = dt;

    // PID warm-up call and reset
    pid_u.p_data[0] = ctr_pid_update(&pid, r_k_data1[0], pid_y_history[0]);
    ctr_pid_init(&pid, kp, ki, kd, TUSTIN, 0);
    pid.tau = tau;
    pid.dt = dt;

    // Set initial values to 0
    pid_u_history[0] = 0;
    pid_x_history[0] = 0;
    pid_y_history[0] = 0;

    Serial.flush();   // Wait until no more data is sent, just in case

    for (uint16_t k = 0; k < n; k++)
    {
        // Discretize signal
        t0 = DWT->CYCCNT;
        pid_u.p_data[0] = ctr_pid_update(&pid, r_k_data1[k], pid_y_history[k]);
        t1 = DWT->CYCCNT;

        cycle_counter_dt = t1 - t0;
        pid_time[k] = (cycle_counter_dt > overhead) ? cycle_counter_dt - overhead : 0;
        
        // Update system
        matf32_mul(&pid_A, &pid_x, &Ax);    // A*x_k
        matf32_mul(&pid_B, &pid_u, &Bu);    // B*u_k
        matf32_add(&Ax, &Bu, &pid_x);       // x_k = A*x_k + B*u_k

        matf32_mul(&pid_C, &pid_x, &pid_y); // y_k = C*x_k

        // Save new data
        pid_u_history[k+1] = pid_u.p_data[0];
        pid_x_history[k+1] = pid_x.p_data[0];
        pid_y_history[k+1] = pid_y.p_data[0];
    }

    insertion_sort(pid_time, n);

    min_t = pid_time[0];
    med_t = (n % 2) ? pid_time[n / 2] : (pid_time[n / 2 - 1] + pid_time[n / 2]) / 2;
    max_t = pid_time[n-1];

    Serial.println("routine,minimum(us),median(us),maximum(us),clock_frequency(Hz),compiler_version");
    Serial.print("PID,");
    Serial.print((float)min_t * 1e6f / (float)SystemCoreClock, 4);   // Convert to microseconds
    Serial.print(",");
    Serial.print((float)med_t * 1e6f / (float)SystemCoreClock, 4);   // Convert to microseconds
    Serial.print(",");
    Serial.print((float)max_t * 1e6f / (float)SystemCoreClock, 4); // Convert to microseconds
    Serial.print(",");
    Serial.print(SystemCoreClock);
    Serial.print(",");
    Serial.println(__VERSION__);

    // -------------------------------------------------------------------------------------
    // Kalman filter test case
    // -------------------------------------------------------------------------------------

    matf32_init(&kf_A, 1, 1, kf_A_data);
    matf32_init(&kf_B, 1, 1, kf_B_data);
    matf32_init(&kf_C, 1, 1, kf_C_data);
    matf32_init(&kf_D, 1, 1, kf_D_data);
    matf32_init(&kf_xhat, 1, 1, kf_xhat_data);
    matf32_init(&kf_x, 1, 1, kf_x_data);
    matf32_init(&kf_u, 1, 1, kf_u_data);
    matf32_init(&kf_y, 1, 1, kf_y_data);
    matf32_init(&kf_F, 1, 1, kf_F_data);
    matf32_init(&kf_P, 1, 1, kf_P_data);
    matf32_init(&kf_Qw, 1, 1, kf_Qw_data);
    matf32_init(&kf_Qv, 1, 1, kf_Qv_data);
    matf32_init(&kf_w, 1, 1, kf_w_data);
    matf32_init(&kf_v, 1, 1, kf_v_data);

    // Initial values
    matf32_set(&kf_x, 1, 1, 1);
    matf32_set(&kf_u, 1, 1, 0);
    matf32_mul(&kf_C, &kf_x, &kf_y);
    matf32_zeros(&kf_xhat);

    ctr_sys_lti_init(&kf_sys, &kf_x, &kf_A, &kf_B, &kf_C, &kf_D, sample_time);
    ctr_kalman_init(&kf, &kf_sys, &kf_F, &kf_Qw, &kf_Qv, &kf_xhat, &kf_P);

    matf32_init(&Ax, kf_A.num_rows, kf_x.num_cols, Ax_data);
    matf32_init(&Bu, kf_B.num_rows, kf_u.num_cols, Bu_data);
    matf32_init(&Fw, kf_F.num_rows, kf_w.num_cols, Fw_data);

    matf32_zeros(&Ax);
    matf32_zeros(&Bu);
    matf32_zeros(&Fw);

    for (uint16_t k = 1; k < K; ++k)
    {
        //matf32_set(&u_k, 1, 1, -1); // u = -1

        // u = -0.1*xhat
        matf32_copy(&kf_xhat, &kf_u); // u = xhat
        matf32_scale(&kf_u, -0.1, &kf_u); // u = -0.1*x_hat

        // Generate noise: in this case, assign the value for this iteration from the .h
        kf_w.p_data[0] = w_data100[k];
        kf_v.p_data[0] = v_data100[k];

        matf32_mul(&kf_A, &kf_x, &Ax); // A*x
        matf32_mul(&kf_B, &kf_u, &Bu); // B*u
        matf32_mul(&kf_F, &kf_w, &Fw);
        matf32_add(&Ax, &Bu, &kf_x); // x = A*x + B*u
        matf32_add(&kf_x, &Fw, &kf_x); // x = A*x + B*u + F*w

        matf32_mul(&kf_C, &kf_x, &kf_y); // y = C*x
        matf32_add(&kf_y, &kf_v, &kf_y); // y = C*x + v

        t0 = DWT->CYCCNT;
        ctr_kalman_predict(&kf, &kf_u); // Prediction: xhat = xhat_prior, P = P_prior
        ctr_kalman_correct(&kf, &kf_y); // Correction: xhat = xhat_post, P = P_post
        t1 = DWT->CYCCNT;

        cycle_counter_dt = t1 - t0;
        kf_time[k] = (cycle_counter_dt > overhead) ? cycle_counter_dt - overhead : 0;

        // Save new values
        xhat_history[k] = kf_xhat.p_data[0];
        x_history[k] = kf_x.p_data[0];
        u_history[k] = kf_u.p_data[0];
        y_history[k] = kf_y.p_data[0];
    }

    insertion_sort(kf_time, n);

    min_t = kf_time[0];
    med_t = (n % 2) ? kf_time[n / 2] : (kf_time[n / 2 - 1] + kf_time[n / 2]) / 2;
    max_t = kf_time[n-1];

    Serial.print("KF,");
    Serial.print((float)min_t * 1e6f / (float)SystemCoreClock, 4);   // Convert to microseconds
    Serial.print(",");
    Serial.print((float)med_t * 1e6f / (float)SystemCoreClock, 4);   // Convert to microseconds
    Serial.print(",");
    Serial.print((float)max_t * 1e6f / (float)SystemCoreClock, 4); // Convert to microseconds
    Serial.print(",");
    Serial.print(SystemCoreClock);
    Serial.print(",");
    Serial.println(__VERSION__);

    // -------------------------------------------------------------------------------------
    // MPC test case
    // -------------------------------------------------------------------------------------
}

void loop()
{

}

void insertion_sort(uint32_t *a, uint16_t n)
{
    for (uint16_t i = 1; i < n; i++)
    {
        uint32_t key = a[i];
        int32_t j = (int32_t)i - 1;

        while (j >= 0 && a[j] > key)
        {
            a[j + 1] = a[j];
            j--;
        }

        a[j + 1] = key;
    }
}