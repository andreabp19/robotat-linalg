/**
 * @author Andrea Pineda
 * @date Created 6 Nov 2025, Last Modified 17 Oct 2026
 * 
 * To test quadprog equality restricted QP functions in the SMT NUCLEO F446RE
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

float* Q_list[5] = {Q_data1, Q_data2, Q_data3, Q_data4, Q_data5};
matf32_t Q;

float* Aeq_list[5] = {Aeq_data1, Aeq_data2, Aeq_data3, Aeq_data4, Aeq_data5};
matf32_t Aeq;

float* c_list[5] = {c_data1, c_data2, c_data3, c_data4, c_data5};
matf32_t c;

float* beq_list[5] = {beq_data1, beq_data2, beq_data3, beq_data4, beq_data5};
matf32_t beq;

float* Q_nullspace_list[9] = {Q_nullspace_data1, Q_nullspace_data2,
    Q_nullspace_data3, Q_nullspace_data4, Q_nullspace_data5,
    Q_nullspace_data6, Q_nullspace_data7, Q_nullspace_data8,
    Q_nullspace_data9};
matf32_t Q_nullspace;

float* c_nullspace_list[9] = {c_nullspace_data1, c_nullspace_data2,
    c_nullspace_data3, c_nullspace_data4, c_nullspace_data5,
    c_nullspace_data6, c_nullspace_data7, c_nullspace_data8,
    c_nullspace_data9};
matf32_t c_nullspace;

float* Aeq_nullspace_list[9] = {Aeq_nullspace_data1, Aeq_nullspace_data2,
    Aeq_nullspace_data3, Aeq_nullspace_data4, Aeq_nullspace_data5,
    Aeq_nullspace_data6, Aeq_nullspace_data7, Aeq_nullspace_data8,
    Aeq_nullspace_data9};
matf32_t Aeq_nullspace;

float* beq_nullspace_list[9] = {beq_nullspace_data1, beq_nullspace_data2,
    beq_nullspace_data3, beq_nullspace_data4, beq_nullspace_data5,
    beq_nullspace_data6, beq_nullspace_data7, beq_nullspace_data8,
    beq_nullspace_data9};
matf32_t beq_nullspace;

float* R_nullspace_list[9] = {R_nullspace1, R_nullspace2, R_nullspace3,
    R_nullspace4, R_nullspace5, R_nullspace6, R_nullspace7,
    R_nullspace8, R_nullspace9};
matf32_t R_nullspace;

float x_nullspace_data[MAX_MAT_SIZE];
matf32_t x_nullspace;

float* R_x_list[5] = {R_x_data1, R_x_data2, R_x_data3, R_x_data4, R_x_data5};
matf32_t R_x;

float* R_Ax_list[5] = {R_Ax_data1, R_Ax_data2, R_Ax_data3, R_Ax_data4, R_Ax_data5};
matf32_t R_Ax;

float x_data[MAX_MAT_SIZE];
matf32_t x;

float Ax_data[MAX_MAT_SIZE];
matf32_t Ax;

quadprog_t qp;

// Variables for measuring time
unsigned long tic = 0;
float toc = 0;

void print_matrix(float* matrix, uint16_t rows, uint16_t cols);

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

    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t n = i + 1;  // matrix dimension to use

        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&Aeq, n, n, Aeq_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&beq, n, 1, beq_list[i]);

        matf32_init(&R_x, n, 1, R_x_list[i]);
        matf32_init(&R_Ax, n, 1, R_Ax_list[i]);

        matf32_init(&x, n, 1, x_data);
        //matf32_init(&Ax, n, 1, Ax_data);

        quadprog_init(&qp, &Q, &c, &Aeq, &beq, NULL, NULL, NULL);

        Serial.println();
        Serial.println("--------------------------------------------------");
        Serial.print("n = ");
        Serial.println(n);
        Serial.println("--------------------------------------------------");
        Serial.println();

        // ---------------------------------------------------------------------------
        // Equality Restricted QPs
        // ---------------------------------------------------------------------------
        // ---------------------------------------------------------------------------
        // quadprog: Equality Restrictions (Square Aeq Matrices)
        // ---------------------------------------------------------------------------
    
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("MATLAB");
        //Serial.println("------------------------------");
        //Serial.println();

        //Serial.println("QP solution in MATLAB:\n");
        //print_matrix(R_x.p_data, R_x.num_rows, R_x.num_cols);
        //Serial.println("beq:\n");
        //print_matrix(&beq);

        // --------------------------------------------------
        // quadprog_qp_linsolve LDL'
        // --------------------------------------------------
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("LDL'");
        //Serial.println("------------------------------");
        //Serial.println();

        float qp_ldlt_time[z];
        float mean_qp_ldlt_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_ldlt(&qp, &x);
            qp_ldlt_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("quadprog_qp_ldlt");
        Serial.println(n);
        for (uint8_t j = 0; j < z; j++)
        {
            Serial.print(qp_ldlt_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        //Serial.println("quadprog solution with ldlt:");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        matf32_mul(&Aeq, &x, &Ax);
        bool qp_ldlt_ans = matf32_is_equal(&x, &R_x);
        //Serial.print("quadprog_qp_ldlt");
        //Serial.println(n);
        //Serial.println(mean_qp_ldlt_time,9);
        //Serial.print("quadprog_qp_ldlt, Mean Time (s): ");
        //Serial.print(mean_qp_ldlt_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_ldlt_ans?"True":"False");

        // --------------------------------------------------
        // quadprog_qp_linsolve LU
        // --------------------------------------------------
        //Serial.println();
        //Serial.println("------------------------------");
        //Serial.println("LU");
        //Serial.println("------------------------------");
        //Serial.println();

        float qp_lu_time[z];
        float mean_qp_lu_time = 0;
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
        Serial.println();

        //Serial.println("quadprog solution with lu:\n");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        matf32_mul(&Aeq, &x, &Ax);
        bool qp_lu_ans = matf32_is_equal(&x, &R_x);
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
        float mean_qp_svd_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_linsolve(&qp, &x, SVD);
            qp_svd_time[j] = (float)((micros() - tic)/1000000.0);
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

        matf32_mul(&Aeq, &x, &Ax);
        bool qp_svd_ans = matf32_is_equal(&x, &R_x);
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

        float qp_qr_time[z];
        float mean_qp_qr_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_linsolve(&qp, &x, QR);
            qp_qr_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("quadprog_qp_qr");
        Serial.println(n);
        for (uint8_t j = 0; j < z; j++)
        {
            Serial.print(qp_qr_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        //Serial.println("quadprog solution with qr");
        //print_matrix(x.p_data, x.num_rows, x.num_cols);

        matf32_mul(&Aeq, &x, &Ax);
        bool qp_qr_ans = matf32_is_equal(&x, &R_x);
        //Serial.print("quadprog_qp_qr");
        //Serial.println(n);
        //Serial.println(mean_qp_qr_time,9);
        //Serial.print("quadprog_qp_linsolve QR, Mean Time (s): ");
        //Serial.print(mean_qp_qr_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_qr_ans?"True":"False");
    }

    // ---------------------------------------------------------------------------
    // Methods for rectangular matrices (n x m) with m > n
    // ---------------------------------------------------------------------------

    for (uint8_t i = 0; i < 9; i++)
    {
        uint16_t n = i + 1;

        //Serial.println();
        //Serial.println("--------------------------------------------------");
        //Serial.print("n = ");
        //Serial.println(n);
        //Serial.println("--------------------------------------------------");
        //Serial.println();

        matf32_init(&Q_nullspace, n+1, n+1, Q_nullspace_list[i]);
        matf32_init(&c_nullspace, n+1, 1, c_nullspace_list[i]);
        matf32_init(&Aeq_nullspace, n, n+1, Aeq_nullspace_list[i]);
        matf32_init(&beq_nullspace, n+1, 1, beq_nullspace_list[i]);
        matf32_init(&x_nullspace, n+1, 1, x_nullspace_data);
        matf32_init(&R_nullspace, n+1, 1, R_nullspace_list[i]);

        quadprog_init(&qp, &Q_nullspace, &c_nullspace, &Aeq_nullspace, &beq_nullspace, NULL, NULL, NULL);

        float qp_nullspace_time[z];
        float mean_qp_nullspace_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            tic = micros();
            quadprog_qp_nullspace(&qp, &x_nullspace);
            //toc = micros() - tic;
            qp_nullspace_time[j] = (float)((micros() - tic)/1000000.0);
        }
        
        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("quadprog_qp_nullspace");
        Serial.println(n);
        for (uint8_t j = 0; j < z; j++)
        {
            Serial.print(qp_nullspace_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        bool qp_nullspace_ans = matf32_is_equal(&x_nullspace, &R_nullspace);
        //Serial.print("quadprog_qp_nullspace");
        //Serial.println(n);
        //Serial.println(mean_qp_nullspace_time,9);
        //Serial.print("quadprog_qp_nullspace, Mean Time (s): ");
        //Serial.print(mean_qp_nullspace_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qp_nullspace_ans?"True":"False");
    }
}

void loop()
{

}

void print_matrix(float* matrix, uint16_t rows, uint16_t cols)
{
  for (uint16_t i = 0; i < rows; ++i)
  {
    for (uint16_t j = 0; j < cols; ++j)
    {
      Serial.print(matrix[i + j*rows],9);
      Serial.print("  ");
    }
    Serial.println();
  }
  Serial.println();
}
