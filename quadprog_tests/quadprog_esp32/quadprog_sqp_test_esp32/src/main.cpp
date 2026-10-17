#include <Arduino.h>

/**
 * @author Andrea Pineda
 * @date Created 7 Oct 2025
 * Last Modified 2 Jan 2026
 *        By: Andrea Pineda
 * 
 * To test quadprog_sqp (active-set) functions of Robotat Linalg in the ESP32
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "quadprog.h"
#include "quadprog_sqp_test_data.h"

float x_data[MAX_MAT_SIZE];
matf32_t x;

float* Q_list[6] = {Q_data1, Q_data2, Q_data3, Q_data4,
    Q_data5, Q_data6};
matf32_t Q;

float* c_list[6] = {c_data1, c_data2, c_data3, c_data4,
    c_data5, c_data6};
matf32_t c;

float* Aeq_list[6] = {Aeq_data1, Aeq_data2, Aeq_data3,
    Aeq_data4, Aeq_data5, Aeq_data6};
matf32_t Aeq;

float* beq_list[6] = {beq_data1, beq_data2, beq_data3,
    beq_data4, beq_data5, beq_data6};
matf32_t beq;

float* Ain_list[6] = {Ain_data1, Ain_data2, Ain_data3,
    Ain_data4, Ain_data5, Ain_data6};
matf32_t Ain;

float* bin_list[6] = {bin_data1, bin_data2, bin_data3,
    bin_data4, bin_data5, bin_data6};
matf32_t bin;

float* R_x_list[6] = {R_x_data1, R_x_data2, R_x_data3,
    R_x_data4, R_x_data5, R_x_data6};
matf32_t R_x;

float xtQ_data[MAX_MAT_SIZE];
matf32_t xtQ;

float xtQx_data[MAX_MAT_SIZE];
matf32_t xtQx;

float xt_data[MAX_MAT_SIZE];
matf32_t xt;

float ct_data[MAX_MAT_SIZE];
matf32_t ct;

float ctx_data[MAX_MAT_SIZE];
matf32_t ctx;

quadprog_t sqp;

uint16_t samples = 100;

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
  for (uint16_t i = 0; i < 6; ++i)
  {
    uint16_t n = i + 1;

    uint16_t Aeq_rows = 2;
    uint16_t Ain_rows = 2;

    //Serial.println("--------------------------------------------------");
    //Serial.print("n = ");
    //Serial.println(n);
    //Serial.println("--------------------------------------------------");

    matf32_init(&Q, n, n, Q_list[i]);
    matf32_init(&c, n, 1, c_list[i]);
    matf32_init(&Aeq, Aeq_rows, n, Aeq_list[i]);
    matf32_init(&beq, Aeq_rows, 1, beq_list[i]);
    matf32_init(&Ain, Ain_rows, n, Ain_list[i]);
    matf32_init(&bin, Ain_rows, 1, bin_list[i]);
    matf32_init(&x, n, 1, x_data);
    matf32_init(&R_x, n, 1, R_x_list[i]);
    matf32_init(&xt, x.num_cols, x.num_rows, xt_data);
    matf32_init(&xtQ, xt.num_rows, Q.num_cols, xtQ_data);
    matf32_init(&xtQx, xt.num_rows, x.num_cols, xtQx_data);
    matf32_init(&ct, c.num_cols, c.num_rows, ct_data);
    matf32_init(&ctx, ct.num_rows, x.num_cols, ctx_data);

    matf32_zeros(&x);

    quadprog_init(&sqp, &Q, &c, &Aeq, &beq, &Ain, &bin, NULL);
    
    float quadprog_sqp_time[samples];
    float mean_quadprog_sqp_time = 0;
    for (uint8_t j = 0; j < samples; j++)
    {
        tic = micros();
        quadprog_sqp(&sqp, &x);
        quadprog_sqp_time[j] = (float)((micros() - tic)/1000000.0);
    }

    // Print time values formatted to a .mat for plotting in matlab
    Serial.print("quadprog_sqp");
    Serial.println(n);
    for (uint8_t j = 0; j < samples; j++)
    {
        Serial.print(quadprog_sqp_time[j], 9);
        Serial.print(" ");
    }
    Serial.println();   

    //Serial.print("quadprog_sqp,general");
    //Serial.println(n);
    //Serial.println(mean_quadprog_sqp_time,9);

    bool active_set_ans = matf32_is_equal(&x, &R_x);
    //Serial.print("quadprog_sqp_general, Mean Time (s): ");
    //Serial.print(mean_quadprog_sqp_time,9);
    //Serial.print(", 1E-05 Tolerance: ");
    //Serial.println(active_set_ans?"True":"False");

    // q = (1/2)*x'Qx + c'x
    matf32_trans(&x, &xt); // x'
    matf32_mul(&xt, &Q, &xtQ); // x'Q
    matf32_mul(&xtQ, &x, &xtQx); // x'Qx
    matf32_scale(&xtQx, 0.5, &xtQx); // (1/2)*x'Qx

    matf32_trans(&c, &ct); // c'
    matf32_mul(&ct, &x, &ctx); // x'c
    matf32_add(&xtQx, &ctx, &xtQx); // (1/2)*x'Qx + c'
    //Serial.println();
    //Serial.println("x:");
    //print_matrix(x.p_data, x.num_rows, x.num_cols);
    //Serial.println("q = (1/2)*x'Qx + c'x:");
    //print_matrix(xtQx.p_data, xtQx.num_rows, xtQx.num_cols);
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
