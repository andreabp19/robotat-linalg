
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

  Serial.println("cycle_counter_dt,overhead,SystemCoreClock");
  Serial.print(cycle_counter_dt);
  Serial.print(",");
  Serial.print(overhead);
  Serial.print(",");
  Serial.println(SystemCoreClock);

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

  Serial.println("minimum,median,maximum");
  Serial.print(min_t);
  Serial.print(",");
  Serial.print(med_t);
  Serial.print(",");
  Serial.println(max_t);

  // -------------------------------------------------------------------------------------
  // Kalman filter test case
  // -------------------------------------------------------------------------------------

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