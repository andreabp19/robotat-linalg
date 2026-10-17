
/**
 * @author Andrea Pineda
 * @date Created 5 Nov 2025
 * Last Modified 5 Nov 2025
 *        By: Andrea Pineda
 * 
 * To test quadprog_sqp (active-set) functions of Robotat Linalg in the Arduino MEGA
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

float* Q_list[1] = {Q_data6};
matf32_t Q;

float* c_list[1] = {c_data6};
matf32_t c;

float* Aeq_list[1] = {Aeq_data6};
matf32_t Aeq;

float* beq_list[1] = {beq_data6};
matf32_t beq;

float* Ain_list[1] = {Ain_data6};
matf32_t Ain;

float* bin_list[1] = {bin_data6};
matf32_t bin;

/*float* Q_list[6] = {Q_data1, Q_data2, Q_data3, Q_data4,
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
matf32_t bin;*/

quadprog_t sqp;

uint16_t samples = 100;

unsigned long tic = 0;
float toc = 0;

void setup()
{
  // ---------------------------------------------------------------------------
  // Serial config
  // ---------------------------------------------------------------------------
  Serial.begin(115200);
  delay(2000);

  // ---------------------------------------------------------------------------
  // Testing
  // ---------------------------------------------------------------------------
  for (uint16_t i = 0; i < 1; ++i)
{
    Serial.println("Here1");

    //uint16_t n = i + 1;
    uint16_t n = 10;

    uint16_t Aeq_rows = 2;
    uint16_t Ain_rows = 2;

    matf32_init(&Q, n, n, Q_list[i]);
    matf32_init(&c, n, 1, c_list[i]);
    matf32_init(&Aeq, Aeq_rows, n, Aeq_list[i]);
    matf32_init(&beq, Aeq_rows, 1, beq_list[i]);
    matf32_init(&Ain, Ain_rows, n, Ain_list[i]);
    matf32_init(&bin, Ain_rows, 1, bin_list[i]);
    matf32_init(&x, n, 1, x_data);

    matf32_zeros(&x);

    quadprog_init(&sqp, &Q, &c, &Aeq, &beq, &Ain, &bin, NULL);

    Serial.println("quadprog_sqp");
    for (uint8_t j = 0; j < samples; j++)
    {
        uint32_t tic = micros();

        quadprog_sqp(&sqp, &x);

        uint32_t elapsed = micros() - tic;

        Serial.print(elapsed / 1000000.0, 9);
        Serial.print(" ");
    }
    Serial.println();


    //Serial.print("quadprog_sqp,general");
    //Serial.println(n);
    //Serial.println(mean_quadprog_sqp_time,9);

    //bool active_set_ans = matf32_is_equal(&x, &R_x);
    //Serial.print("quadprog_sqp_general, Mean Time (s): ");
    //Serial.print(mean_quadprog_sqp_time,9);
    //Serial.print(", 1E-05 Tolerance: ");
    //Serial.println(active_set_ans?"True":"False");

    // q = (1/2)*x'Qx + c'x
    //matf32_trans(&x, &xt); // x'
    //matf32_mul(&xt, &Q, &xtQ); // x'Q
    //matf32_mul(&xtQ, &x, &xtQx); // x'Qx
    //matf32_scale(&xtQx, 0.5, &xtQx); // (1/2)*x'Qx

    //matf32_trans(&c, &ct); // c'
    //matf32_mul(&ct, &x, &ctx); // x'c
    //matf32_add(&xtQx, &ctx, &xtQx); // (1/2)*x'Qx + c'
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
