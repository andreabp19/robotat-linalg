
/**
 * @author Andrea Pineda
 * @date Created 3 Sep 2025, last modified: 15 Sep 2025
 * 
 * For testing quadprog in computer
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "quadprog.h"
#include "quadprog_test_results.h"

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp_difference_data[MAX_MAT_SIZE];
matf32_t temp_difference;

float* Q_list[10] = {Q_data1, Q_data2, Q_data3, Q_data4, Q_data5};
matf32_t Q;

float* Aeq_list[10] = {Aeq_data1, Aeq_data2, Aeq_data3, Aeq_data4, Aeq_data5};
matf32_t Aeq;

float* c_list[10] = {c_data1, c_data2, c_data3, c_data4, c_data5};
matf32_t c;

float* beq_list[10] = {beq_data1, beq_data2, beq_data3, beq_data4, beq_data5};
matf32_t beq;

float* R_quadprog_list[10] = {R_quadprog_data1, R_quadprog_data2, R_quadprog_data3, R_quadprog_data4, R_quadprog_data5};
matf32_t R_quadprog;

quadprog_t qp;
quadprog_t sqp;

int main(void)
{
    clock_t time;
    err_status_t err_status;
    linsolve_method_t linsolve_method;
    quadprog_status_t quadprog_status;

    printf("\n\n");

    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&temp, n, 1, temp_data); // To save results
        matf32_init(&temp_difference, n, 1, temp_difference_data);
        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&Aeq, n, n, Aeq_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&beq, n, 1, beq_list[i]);
        matf32_init(&R_quadprog, n, 1, R_quadprog_list[i]);

        quadprog_init(&qp, &Q, &c, &Aeq, &beq, NULL, NULL, NULL);
        quadprog_init(&sqp, &Q, &c, NULL, NULL, &Aeq, &beq, NULL);

        //printf("\n--------------------------------------------------\n");
        //printf("n = %i\n", n);
        //printf("--------------------------------------------------\n");

        // ---------------------------------------------------------------------------
        // Equality Restricted QPs
        // ---------------------------------------------------------------------------
        // ---------------------------------------------------------------------------
        // quadprog: Equality Restrictions (quadprog_qp)
        // ---------------------------------------------------------------------------
    
        float quadprog_qp_time[z];
        float mean_quadprog_qp_time = 0;

        for (uint8_t j = 0; j < z; j++)
        {
            time = clock();
            quadprog(&qp, &temp);
            quadprog_qp_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        //quadprog(&qp, &temp); // It's using LU

        mean_quadprog_qp_time = mean(quadprog_qp_time, z);

        /*printf("ESP32 Result:\n");
        matf32_print(&temp);
        printf("MATLAB Result:\n");
        matf32_print(&R_quadprog);*/

        bool quadprog_qp_ans = matf32_is_equal(&temp, &R_quadprog);
        printf("quadprog_qp%i\n",n);
        printf("%.9f\n", mean_quadprog_qp_time);
        //printf("quadprog_qp,time(s):%.9f,%s\n\n", mean_quadprog_qp_time, quadprog_qp_ans?"success":"failure");
        //printf("quadprog_qp,%s\n\n", quadprog_qp_ans?"success":"failure");

        //printf("Difference ESP32 vs MATLAB Result:\n");
        //matf32_sub(&temp, &R_quadprog, &temp_difference);
        //matf32_print(&temp_difference);

        // ---------------------------------------------------------------------------
        // quadprog: Inequality Restrictions (quadprog_sqp)
        // ---------------------------------------------------------------------------
    
        //float quadprog_sqp_time[z];
        //float mean_quadprog_sqp_time = 0;

        //for (uint8_t j = 0; j < z; j++)
        //{
        //    time = clock();
        //    quadprog_sqp(&qp, &temp);
        //    quadprog_sqp_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        //}

        /*quadprog_sqp(&sqp, &temp); // It's using LU

        //mean_quadprog_sqp_time = mean(quadprog_sqp_time, z);

        printf("ESP32 Result:\n");
        matf32_print(&temp);
        printf("MATLAB Result:\n");
        matf32_print(&R_quadprog);

        bool quadprog_sqp_ans = matf32_is_equal(&temp, &R_quadprog);
        //printf("matf32_quadprog_sqp%i\n",n);
        //printf("%.9f\n", mean_quadprog_sqp_time);
        //printf("quadprog_sqp,time(s):%.9f,%s\n\n", mean_quadprog_sqp_time, quadprog_sqp_ans?"success":"failure");
        printf("quadprog_sqp,%s\n\n", quadprog_sqp_ans?"success":"failure");

        printf("Difference ESP32 vs MATLAB Result:\n");
        matf32_sub(&temp, &R_quadprog, &temp_difference);
        matf32_print(&temp_difference);*/

    }
}


