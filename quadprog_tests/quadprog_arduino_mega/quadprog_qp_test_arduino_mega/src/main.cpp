
/**
 * @author Andrea Pineda
 * @date Created 28 Oct 2025, Last Modified: 4 Nov 2025
 * 
 * To test quadprog equality restricted QP functions in the Arduino MEGA
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

#include "quadprog.h"
#include "quadprog_qp_test_data.h"
#include "quadprog_qp_nullspace_test_data.h"

// --------------------------------------------------
// Definitions
// --------------------------------------------------

float* Q_list[1] = {Q_data5};
matf32_t Q;

float* Aeq_list[1] = {Aeq_data5};
matf32_t Aeq;

float* c_list[1] = {c_data5};
matf32_t c;

float* beq_list[1] = {beq_data5};
matf32_t beq;

//float* Q_nullspace_list[1] = {Q_nullspace_data9};
//matf32_t Q_nullspace;
//
//float* c_nullspace_list[1] = {c_nullspace_data9};
//matf32_t c_nullspace;
//
//float* Aeq_nullspace_list[1] = {Aeq_nullspace_data9};
//matf32_t Aeq_nullspace;
//
//float* beq_nullspace_list[1] = {beq_nullspace_data9};
//matf32_t beq_nullspace;
//
//float x_nullspace_data[MAX_MAT_SIZE];
//matf32_t x_nullspace;

float x_data[MAX_MAT_SIZE];
matf32_t x;

quadprog_t qp;

// Variables for measuring time
unsigned long tic = 0;
float toc = 0;

void setup()
{
    // ---------------------------------------------------------------------------
    // Serial config
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // Testing
    // ---------------------------------------------------------------------------

    for (uint8_t i = 0; i < 1; i++)
    {
        //uint8_t n = i + 1;  // matrix dimension to use

        uint8_t n = 5;

        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&Aeq, n, n, Aeq_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&beq, n, 1, beq_list[i]);

        matf32_init(&x, n, 1, x_data);

        quadprog_init(&qp, &Q, &c, &Aeq, &beq, NULL, NULL, NULL);

        // --------------------------------------------------
        // quadprog_qp_linsolve LU
        // --------------------------------------------------
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("LU");
        //Serial.println("------------------------------");
        //Serial.println();

        /*float qp_lu_time[z];
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_linsolve(&qp, &x, LU);
            qp_lu_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("quadprog_qp_lu");
        Serial.println(n);
        for (uint8_t j = 0; j < z; j++)
        {
            Serial.print(qp_lu_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.println("quadprog solution with lu:\n");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        //Serial.print("quadprog_qp_lu");
        //Serial.println(n);
        //Serial.println(mean_qp_lu_time,9);
        //Serial.print("quadprog_qp_linsolve LU, Mean Time (s): ");
        //Serial.print(mean_qp_lu_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_lu_ans?"True":"False");

        // --------------------------------------------------
        // quadprog_qp_linsolve SVD
        // --------------------------------------------------
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("SVD");
        //Serial.println("------------------------------");
        //Serial.println();

        float qp_svd_time[z];
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_linsolve(&qp, &x, SVD);
            qp_svd_time[j] = (float)((micros() - tic)/1000000.0);
            Serial.println(j+1);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("quadprog_qp_svd");
        Serial.println(n);
        for (uint8_t j = 0; j < z; j++)
        {
            Serial.print(qp_svd_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        //Serial.println("quadprog solution with svd");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        //Serial.print("quadprog_qp_svd");
        //Serial.println(n);
        //Serial.println(mean_qp_svd_time,9);
        //Serial.print("quadprog_qp_linsolve SVD, Mean Time (s): ");
        //Serial.print(mean_qp_svd_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_svd_ans?"True":"False");

        // --------------------------------------------------
        // quadprog_qp_linsolve QR
        // --------------------------------------------------
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("QR");
        //Serial.println("------------------------------");
        //Serial.println();

        /*Serial.println("quadprog_qp_qr");
        //float qp_qr_time[z];
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_linsolve(&qp, &x, QR_RECT);
            //qp_qr_time[j] = (float)((micros() - tic)/1000000.0);
            
            uint32_t elapsed = micros() - tic;

            Serial.print(elapsed / 1000000.0, 9);
            Serial.print(" ");
        }
        Serial.println('\n');*/

        // Print time values formatted to a .mat for plotting in matlab
        //Serial.print("quadprog_qp_qr");
        //Serial.println(n);
        //for (uint8_t j = 0; j < z; j++)
        //{
        //    Serial.print(qp_qr_time[j], 9);
        //    Serial.print(" ");
        //}
        //Serial.println();

        //Serial.println("quadprog solution with qr");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        //Serial.print("quadprog_qp_qr");
        //Serial.println(n);
        //Serial.println(mean_qp_qr_time,9);
        //Serial.print("quadprog_qp_linsolve QR, Mean Time (s): ");
        //Serial.print(mean_qp_qr_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_qr_ans?"True":"False");
    }
}

void loop()
{

}