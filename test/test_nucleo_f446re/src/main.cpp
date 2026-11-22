
/*
Test project for the STM32 Nucleo F446RE for branch build-test-perf
Author: Andrea Pineda
Date: 20 Nov. 2026, last modified: 21 Nov. 2026
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

#define STR_(x) #x
#define STR(x) STR_(x)

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

float kf_sample_time = 0.1;

float xhat_history[K];
float x_history[K];
float u_history[K];
float y_history[K];

ctr_sys_lti_t kf_sys;
ctr_kalman_t kf;

// --------------------------------------------------
// MPC
// --------------------------------------------------

#define N (4) // Horizon length
#define ROWS (2)
#define COLS (1)

uint32_t mpc_time[SAMPLES];

static float tmpmat1_data[MAX_MAT_SIZE];
static matf32_t tmpmat1;

float mpc_sys_A_data[ROWS*ROWS] = {1.1, 2.0, 0.0, 0.95};
matf32_t mpc_sys_A;

float mpc_sys_B_data[ROWS*COLS] = {0.0, 0.0787};
matf32_t mpc_sys_B;

float mpc_sys_C_data[COLS*ROWS] = {-1, 1};
matf32_t mpc_sys_C;

float mpc_sys_D_data[MAX_MAT_SIZE];
matf32_t mpc_sys_D;

float mpc_u_data[MAX_MAT_SIZE];
matf32_t mpc_u;

float mpc_x_data[MAX_MAT_SIZE];
matf32_t mpc_x;

float qp_Q_data[MAX_MAT_SIZE];
matf32_t qp_Q;

float qp_c_data[MAX_MAT_SIZE];
matf32_t qp_c;

float qp_Ain_data[MAX_MAT_SIZE];
matf32_t qp_Ain;

float qp_bin_data[MAX_MAT_SIZE];
matf32_t qp_bin;

// Memory stack and pointer arrays for MPC matrix M
float M1_data[MAX_MAT_SIZE];
matf32_t M1;
float M2_data[MAX_MAT_SIZE];
matf32_t M2;
float M3_data[MAX_MAT_SIZE];
matf32_t M3;
float M4_data[MAX_MAT_SIZE];
matf32_t M4;

float* mpc_M_data[N] = {M1_data, M2_data, M3_data, M4_data};
matf32_t* mpc_M[N] = {&M1, &M2, &M3, &M4}; // Fix matrix names inside quadprog to avoid conflict with naming this M.

// Memory stack and pointer arrays for MPC convolution matrix C
float AB_data[MAX_MAT_SIZE];
matf32_t AB;
float A2B_data[MAX_MAT_SIZE];
matf32_t A2B;
float A3B_data[MAX_MAT_SIZE];
matf32_t A3B;
float C_null_mat_data[MAX_MAT_SIZE];
matf32_t C_null_mat;

// Put C matrices in an array to iterate over them
float* mpc_C_stack_data[N+1] = {mpc_sys_B_data, AB_data, A2B_data, A3B_data, C_null_mat_data};
matf32_t* mpc_C_stack[N+1] = {&mpc_sys_B, &AB, &A2B, &A3B, &C_null_mat};
matf32_t* mpc_C[N*N]; // Matrix of matrices C. Don't initialize yet.

float mpc_Q_data[MAX_MAT_SIZE];
matf32_t mpc_Q;

float mpc_R_data[MAX_MAT_SIZE];
matf32_t mpc_R;

float U[2*N];
float cost[2*N];
float X1[2*N];
float X2[2*N];

float utQ_data[MAX_MAT_SIZE];
matf32_t utQ;

float utQu_data[MAX_MAT_SIZE];
matf32_t utQu;

float ut_data[MAX_MAT_SIZE];
matf32_t ut;

float ct_data[MAX_MAT_SIZE];
matf32_t ct;

float ctu_data[MAX_MAT_SIZE];
matf32_t ctu;

bool state_constraints = false;

quadprog_t qp;
ctr_sys_lti_t sys;
ctr_mpc_lti_shooting_t mpc;

float mpc_sample_time = 0.01;

// --------------------------------------------------
// Other Variables
// --------------------------------------------------

uint16_t n = SAMPLES - 1;

uint32_t t0 = 0;                  // Initial time
uint32_t t1 = 0;                  // Final time
uint32_t cycle_counter_dt = 0;    // Time interval
uint32_t overhead = 0;            // Cycle Counter overhead (calibration for time of measurement itself)
uint32_t overhead_meas[SAMPLES];  // To take several measurements of the overhead

uint32_t min_t = 0;
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

    Serial.println("routine,minimum(s),median(s),maximum(s),clock_frequency(Hz),compiler_version,lib_commit");
    Serial.print("PID,");
    Serial.print((float)min_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)med_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)max_t / (float)SystemCoreClock, 9); // Convert to seconds
    Serial.print(",");
    Serial.print(SystemCoreClock);
    Serial.print(",");
    Serial.print(__VERSION__);
    Serial.print(",");
    Serial.println(STR(LIB_COMMIT));

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

    ctr_sys_lti_init(&kf_sys, &kf_x, &kf_A, &kf_B, &kf_C, &kf_D, kf_sample_time);
    ctr_kalman_init(&kf, &kf_sys, &kf_F, &kf_Qw, &kf_Qv, &kf_xhat, &kf_P);

    matf32_init(&Ax, kf_A.num_rows, kf_x.num_cols, Ax_data);
    matf32_init(&Bu, kf_B.num_rows, kf_u.num_cols, Bu_data);
    matf32_init(&Fw, kf_F.num_rows, kf_w.num_cols, Fw_data);

    matf32_zeros(&Ax);
    matf32_zeros(&Bu);
    matf32_zeros(&Fw);

    for (uint16_t k = 1; k < n; ++k)
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
    Serial.print((float)min_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)med_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)max_t / (float)SystemCoreClock, 9); // Convert to seconds
    Serial.print(",");
    Serial.print(SystemCoreClock);
    Serial.print(",");
    Serial.print(__VERSION__);
    Serial.print(",");
    Serial.println(STR(LIB_COMMIT));

    // -------------------------------------------------------------------------------------
    // MPC test case
    // -------------------------------------------------------------------------------------

    matf32_init(&mpc_sys_A, ROWS, ROWS, mpc_sys_A_data);
    matf32_init(&mpc_sys_B, ROWS, COLS, mpc_sys_B_data);
    matf32_init(&mpc_sys_C, COLS, ROWS, mpc_sys_C_data);
    matf32_init(&mpc_sys_D, COLS, COLS, mpc_sys_D_data);
    matf32_init(&mpc_x, ROWS, COLS, mpc_x_data);
    matf32_init(&mpc_u, N, COLS, mpc_u_data);
    matf32_init(&qp_Ain, N, N, qp_Ain_data);
    matf32_init(&qp_bin, N, COLS, qp_bin_data);
    matf32_init(&qp_Q, N*COLS, N*COLS, qp_Q_data);
    matf32_init(&qp_c, N*COLS, COLS, qp_c_data);
    matf32_init(&mpc_R, COLS, COLS, mpc_R_data);
    matf32_init(&mpc_Q, ROWS, ROWS, mpc_Q_data);

    matf32_init(&ut, mpc_u.num_cols, mpc_u.num_rows, ut_data);
    matf32_init(&utQ, ut.num_rows, qp_Q.num_cols, utQ_data);
    matf32_init(&utQu, mpc_u.num_rows, mpc_u.num_cols, utQu_data);
    matf32_init(&ct, qp_c.num_cols, qp_c.num_rows, ct_data);
    matf32_init(&ctu, ct.num_rows, mpc_u.num_cols, ctu_data);

    matf32_set(&mpc_Q, 1, 1, 1);
    matf32_set(&mpc_Q, 1, 2, -1);
    matf32_set(&mpc_Q, 2, 1, -1);
    matf32_set(&mpc_Q, 2, 2, 1);

    matf32_set(&mpc_R, 1, 1, 0.01);

    matf32_ones(&mpc_x);

    ctr_sys_lti_init(&sys, &mpc_x, &mpc_sys_A, &mpc_sys_B, &mpc_sys_C, &mpc_sys_D, mpc_sample_time); // x_k_1 = A*x_k + B*u_k, y_k_1 = C*x_k + D*u_k
    //ctr_mpc_lti_init(&mpc, &qp, &sys, &u_k, &x_k, NULL, NULL, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    ctr_mpc_lti_init(&mpc, &qp, &sys, &mpc_u, &mpc_x, &qp_Ain, &qp_bin, &mpc_Q, &mpc_R, &mpc_Q, mpc_C, mpc_M, N, state_constraints);
    ctr_mpc_set_M(&mpc, mpc_M_data); // Initialize and define the matrices for mpc_M
    ctr_mpc_set_C(&mpc, mpc_C_stack, mpc_C_stack_data); // Initialize and define matrices and pointer array for mpc_C
    ctr_mpc_set_qpQ(&mpc, &qp_Q); // Compute values for qp_Q
    ctr_mpc_set_qpc(&mpc, &qp_c); // Compute vales for qp_c

    mpc.state_constraints = true;
    ctr_mpc_set_constraints(&mpc, 10, -10);

    for (uint16_t i = 0; i < n; i++)
    {
        t0 = DWT->CYCCNT;
        for (uint16_t k = 0; k < 2*N; ++k)
        {
            ctr_mpc_set_qpQ(&mpc, &qp_Q);
            ctr_mpc_set_qpc(&mpc, &qp_c);
            ctr_mpc_set_constraints(&mpc, 10, -10);
            ctr_mpc_update(&mpc, &qp_Q, &qp_c, &mpc_x, &mpc_u);
        }
        t1 = DWT->CYCCNT;

        cycle_counter_dt = t1 - t0;
        mpc_time[i] = (cycle_counter_dt > overhead) ? cycle_counter_dt - overhead : 0;
    }

    insertion_sort(mpc_time, n);

    min_t = mpc_time[0];
    med_t = (n % 2) ? mpc_time[n / 2] : (mpc_time[n / 2 - 1] + mpc_time[n / 2]) / 2;
    max_t = mpc_time[n-1];

    Serial.print("MPC,");
    Serial.print((float)min_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)med_t / (float)SystemCoreClock, 9);   // Convert to seconds
    Serial.print(",");
    Serial.print((float)max_t / (float)SystemCoreClock, 9); // Convert to secons
    Serial.print(",");
    Serial.print(SystemCoreClock);
    Serial.print(",");
    Serial.print(__VERSION__);
    Serial.print(",");
    Serial.println(STR(LIB_COMMIT));
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