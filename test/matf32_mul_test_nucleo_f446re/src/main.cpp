
/*
Test project to measure matf32_mul in the STM32 Nucleo F446RE for branch build-test-perf
Author: Andrea Pineda
Date: 21 Nov. 2026, last modified: 21 Nov. 2026
*/

// ----------------------------------------------------------------------------------------------------
// Imported libraries and files
// ----------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <stdint.h>
#include "matf32.h"
#include "linsolve.h"

// ----------------------------------------------------------------------------------------------------
// Variables and constants
// ----------------------------------------------------------------------------------------------------

#define SAMPLES (100)

#define STR_(x) #x
#define STR(x) STR_(x)

// --------------------------------------------------
// Matrices
// --------------------------------------------------

float A_data[MAX_MAT_SIZE];
matf32_t A;

float At_data[MAX_MAT_SIZE];
matf32_t At;

float B_data[MAX_MAT_SIZE];
matf32_t B;

float R_data[MAX_MAT_SIZE];
matf32_t R;

float C_data[MAX_MAT_SIZE];
matf32_t C;

float b_data[MAX_VEC_SIZE];
matf32_t b;

float x_data[MAX_VEC_SIZE];
matf32_t x;

uint16_t matrix_sizes[3] = {3, 6, 10};

uint32_t matf32_mul_time[SAMPLES];

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
    // matf32_mul tests
    // -------------------------------------------------------------------------------------

    //Serial.println("routine,minimum(s),median(s),maximum(s),clock_frequency(Hz),compiler_version,lib_commit");
    Serial.println("routine,maximum(cycles)");

    // Iterate over every chosen matrix dimension n x n: 3x3, 6x6 and 10x10
    for (uint16_t i = 0; i < 3; i++)
    {
        matf32_init(&A, matrix_sizes[i], matrix_sizes[i], A_data);
        matf32_init(&B, matrix_sizes[i], matrix_sizes[i], B_data);
        matf32_init(&R, matrix_sizes[i], matrix_sizes[i], R_data);

        matf32_randn(&A, 0, 1);
        matf32_randn(&B, 0, 1);
        matf32_randn(&R, 0, 1);

        // Measure execution time 100 times, using the cycle counter 
        for (uint16_t k = 0; k < n; k++)
        {
            t0 = DWT->CYCCNT;
            matf32_mul(&A, &B, &R);
            t1 = DWT->CYCCNT;

            cycle_counter_dt = t1 - t0;
            matf32_mul_time[k] = (cycle_counter_dt > overhead) ? cycle_counter_dt - overhead : 0;
        }

        // Order data from min to max
        insertion_sort(matf32_mul_time, n);

        // Get min, median and max observed times
        min_t = matf32_mul_time[0];
        med_t = (n % 2) ? matf32_mul_time[n / 2] : (matf32_mul_time[n / 2 - 1] + matf32_mul_time[n / 2]) / 2;
        max_t = matf32_mul_time[n-1];

        Serial.print("matf32_mul");
        Serial.print(matrix_sizes[i]);
        Serial.print("x");
        Serial.print(matrix_sizes[i]);
        Serial.print(",");
        //Serial.print(min_t);   // Convert to seconds
        //Serial.print(",");
        //Serial.print(med_t);   // Convert to seconds
        //Serial.print(",");
        Serial.println(max_t); // Convert to second
        //Serial.print(",");
        //Serial.print(SystemCoreClock);
        //Serial.print(",");
        //Serial.print(__VERSION__);
        //Serial.print(",");
        //Serial.println(STR(LIB_COMMIT));
    }

    // -------------------------------------------------------------------------------------
    // Solver tests: Cholesky solve
    // -------------------------------------------------------------------------------------

    // The instructions don't specify dimensions to test, so I'm testing the largest case (10x10)
    matf32_init(&A, 10, 10, A_data);
    matf32_init(&At, 10, 10, At_data);
    matf32_init(&C, 10, 10, C_data);
    matf32_init(&b, 10, 1, b_data);
    matf32_init(&x, 10, 1, x_data);

    matf32_randn(&A, 0, 1);
    matf32_randn(&b, 0, 1);

    matf32_trans(&A, &At);          // A'
    matf32_mul(&At, &A, &C);        // A'A -> Symmetric positive definite matrix for Cholesky

    // Measure execution time 100 times, using the cycle counter 
    for (uint16_t k = 0; k < n; k++)
    {
        t0 = DWT->CYCCNT;
        linsolve_method(&C, &b, &x, CHOLESKY);    // Solve system with Cholesky
        t1 = DWT->CYCCNT;

        cycle_counter_dt = t1 - t0;
        matf32_mul_time[k] = (cycle_counter_dt > overhead) ? cycle_counter_dt - overhead : 0;
    }

    Serial.print("linsolve_cholesky10x10,");
    //Serial.print(min_t);   // Convert to seconds
    //Serial.print(",");
    //Serial.print(med_t);   // Convert to seconds
    //Serial.print(",");
    Serial.println(max_t); // Convert to second
    //Serial.print(",");
    //Serial.print(SystemCoreClock);
    //Serial.print(",");
    //Serial.print(__VERSION__);
    //Serial.print(",");
    //Serial.println(STR(LIB_COMMIT));
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