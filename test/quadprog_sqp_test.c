
/**
 * @author Andrea Pineda
 * @date Created: 1 Oct 2025, Last Modified: 6 Oct 2025
 * 
 * To test quadprog_sqp (active_set method)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "matf32.h"
#include "quadprog.h"
#include "quadprog_sqp_test.h"

float x_data[MAX_MAT_SIZE];
matf32_t x;

float* Q_list[6] = {Q_data1, Q_data2, Q_data3, Q_data4, Q_data5, Q_data6};
matf32_t Q;

float* c_list[6] = {c_data1, c_data2, c_data3, c_data4, c_data5, c_data6};
matf32_t c;

float* Aeq_list[6] = {Aeq_data1, Aeq_data2, Aeq_data3, Aeq_data4, Aeq_data5, Aeq_data6};
matf32_t Aeq;

float* beq_list[6] = {beq_data1, beq_data2, beq_data3, beq_data4, beq_data5, beq_data6};
matf32_t beq;

float* Ain_list[6] = {Ain_data1, Ain_data2, Ain_data3, Ain_data4, Ain_data5, Ain_data6};
matf32_t Ain;

float* bin_list[6] = {bin_data1, bin_data2, bin_data3, bin_data4, bin_data5, bin_data6};
matf32_t bin;

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

int main(void)
{
    clock_t time;

    for (uint16_t i = 0; i < 6; ++i)
    {
        uint16_t n = i + 1;

        uint16_t Aeq_rows = (n + 2) / 3;  // ceil(n/3)
        if (Aeq_rows < 1) Aeq_rows = 1;

        uint16_t Ain_rows = (n + 1) / 2;  // ceil(n/2)
        if (Ain_rows < 2) Ain_rows = 2;

        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n");

        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&Aeq, Aeq_rows, n, Aeq_list[i]);
        matf32_init(&beq, Aeq_rows, 1, beq_list[i]);
        matf32_init(&Ain, Ain_rows, n, Ain_list[i]);
        matf32_init(&bin, Ain_rows, 1, bin_list[i]);

        matf32_init(&x, n, 1, x_data);
        matf32_zeros(&x);
        
        matf32_init(&xt, x.num_cols, x.num_rows, xt_data);
        matf32_init(&xtQ, xt.num_rows, Q.num_cols, xtQ_data);
        matf32_init(&xtQx, xt.num_rows, x.num_cols, xtQx_data);
        matf32_init(&ct, c.num_cols, c.num_rows, ct_data);
        matf32_init(&ctx, ct.num_rows, x.num_cols, ctx_data);

        quadprog_init(&sqp, &Q, &c, &Aeq, &beq, &Ain, &bin, NULL);
        
        float quadprog_sqp_time[100];
        float mean_quadprog_sqp_time = 0;

        /*for (uint8_t j = 0; j < 100; j++)
        {
            time = clock();
            quadprog_sqp(&sqp, &x);
            quadprog_sqp_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_quadprog_sqp_time = mean(quadprog_sqp_time, 100);*/

        quadprog_sqp(&sqp, &x);

        //printf("quadprog_sqp,general%i\n",n);
        //printf("%.9f\n",mean_quadprog_sqp_time);

        // q = (1/2)*x'Qx + c'x
        matf32_trans(&x, &xt); // x'
        matf32_mul(&xt, &Q, &xtQ); // x'Q
        matf32_mul(&xtQ, &x, &xtQx); // x'Qx
        matf32_scale(&xtQx, 0.5, &xtQx); // (1/2)*x'Qx

        matf32_trans(&c, &ct); // c'
        matf32_mul(&ct, &x, &ctx); // x'c
        matf32_add(&xtQx, &ctx, &xtQx); // (1/2)*x'Qx + c'x

        printf("x:\n");
        matf32_print(&x);

        printf("q = (1/2)*x'Qx + c'x:\n");
        matf32_print(&xtQx);
    }
}