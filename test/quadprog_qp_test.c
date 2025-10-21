
/**
 * @author Andrea Pineda
 * @date Created 3 Sep 2025, last modified: 21 Oct 2025
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

float* c_list[10] = {c_data1, c_data2, c_data3, c_data4, c_data5};
matf32_t c;

float* Aeq_list[10] = {Aeq_data1, Aeq_data2, Aeq_data3, Aeq_data4, Aeq_data5};
matf32_t Aeq;

float* beq_list[10] = {beq_data1, beq_data2, beq_data3, beq_data4, beq_data5};
matf32_t beq;

float Ain_data[MAX_MAT_SIZE];
matf32_t Ain;

float bin_data[MAX_MAT_SIZE];
matf32_t bin;

float* R_x_list[10] = {R_x_data1, R_x_data2, R_x_data3, R_x_data4, R_x_data5};
matf32_t R_x;

float* R_Ax_list[10] = {R_Ax_data1, R_Ax_data2, R_Ax_data3, R_Ax_data4, R_Ax_data5};
matf32_t R_Ax;

float x_nullspace_data[MAX_MAT_SIZE];
matf32_t x_nullspace;

float x_lu_data[MAX_MAT_SIZE];
matf32_t x_lu;

float x_svd_data[MAX_MAT_SIZE];
matf32_t x_svd;

float x_ldlt_data[MAX_MAT_SIZE];
matf32_t x_ldlt;

float x_qr_data[MAX_MAT_SIZE];
matf32_t x_qr;

float Ax_data[MAX_MAT_SIZE];
matf32_t Ax;

float xt_data[MAX_MAT_SIZE];
matf32_t xt;

float xtc_data[MAX_MAT_SIZE];
matf32_t xtc;

float xtQ_data[MAX_MAT_SIZE];
matf32_t xtQ;

float xtQx_data[MAX_MAT_SIZE];
matf32_t xtQx;

quadprog_t qp;
quadprog_t sqp;

int main(void)
{
    clock_t time;
    err_status_t status;
    linsolve_method_t linsolve_method;
    quadprog_status_t quadprog_status;

    printf("\n\n");

    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&Q, n, n, Q_list[i]);
        matf32_init(&c, n, 1, c_list[i]);
        matf32_init(&Aeq, n, n, Aeq_list[i]);
        matf32_init(&beq, n, 1, beq_list[i]);
        matf32_init(&Ain, n, n, Ain_data);
        matf32_init(&bin, n, 1, bin_data);

        matf32_randn(&Ain, 0, 1);
        matf32_randn(&bin, 0, 1);

        matf32_init(&R_x, n, 1, R_x_list[i]);
        matf32_init(&R_Ax, n, 1, R_Ax_list[i]);

        matf32_init(&x_lu, n, 1, x_lu_data);
        matf32_init(&x_svd, n, 1, x_svd_data);
        matf32_init(&x_ldlt, n, 1, x_ldlt_data);
        matf32_init(&x_qr, n, 1, x_qr_data);

        matf32_init(&Ax, n, 1, Ax_data);
        matf32_init(&xt, 1, n, xt_data);
        matf32_init(&xtQ, xt.num_rows, Q.num_cols, xtQ_data);
        matf32_init(&xtQx, xt.num_rows, x_ldlt.num_cols, xtQx_data);
        matf32_init(&xtc, xt.num_rows, c.num_cols, xtc_data);

        matf32_init(&temp, n, 1, temp_data);
        matf32_init(&temp_difference, n, 1, temp_difference_data);

        quadprog_init(&qp, &Q, &c, &Aeq, &beq, NULL, NULL, NULL);
        quadprog_init(&sqp, &Q, &c, NULL, NULL, &Aeq, &beq, NULL);

        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n");

        // ---------------------------------------------------------------------------
        // Equality Restricted QPs
        // ---------------------------------------------------------------------------
        // ---------------------------------------------------------------------------
        // quadprog: Equality Restrictions (Square Aeq Matrices)
        // ---------------------------------------------------------------------------
    
        //printf("----- QP Solution x -----\n\n");
        //printf("----- Ax = b Reconstruction -----\n\n");

        //printf("MATLAB:\n");
        //matf32_print(&R_x);
        //printf("beq:\n");
        //matf32_print(&beq);

        // --------------------------------------------------
        // quadprog_qp_ldlt'
        // --------------------------------------------------
        printf("\n-------------------------\n");
        printf("LDL' %ix%i\n", n, n);
        printf("-------------------------\n");

        float qp_ldlt_time[z];
        float mean_qp_ldlt_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            time = clock();
            quadprog_qp_ldlt(&qp, &x_ldlt);
            qp_ldlt_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_qp_ldlt_time = mean(qp_ldlt_time, z);
        
        matf32_mul(&Aeq, &x_ldlt, &Ax);
        bool qp_ldlt_ans = matf32_is_equal(&x_ldlt, &R_x);
        //printf("quadprog_qp_ldlt%i\n", n);
        //printf("%.9f\n", mean_qp_ldlt_time);
        //printf("quadprog_qp LDL', mean_time(s): %.9f, 1E-05 Tolerance:%s\n\n", mean_qp_ldlt_time, qp_ldlt_ans?"success":"failure");

        //printf("Ax LDL':\n");
        //matf32_print(&Ax);

        // q = (1/2)x'Qx + x'c
        matf32_trans(&x_ldlt, &xt);
        matf32_mul(&xt, &c, &xtc);
        matf32_mul(&xt, &Q, &xtQ);
        matf32_mul(&xtQ, &x_ldlt, &xtQx);
        matf32_scale(&xtQx, 0.5, &xtQx);

        printf("q = (1/2)x'Qx + x'c:\n");
        matf32_print(&xtQx);

        // --------------------------------------------------
        // quadprog_qp LU
        // --------------------------------------------------
        printf("\n-------------------------\n");
        printf("LU %ix%i\n", n, n);
        printf("-------------------------\n");

        float qp_lu_time[z];
        float mean_qp_lu_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            time = clock();
            quadprog_qp(&qp, &x_lu, LU);
            qp_lu_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_qp_lu_time = mean(qp_lu_time, z);

        matf32_mul(&Aeq, &x_lu, &Ax);
        bool qp_lu_ans = matf32_is_equal(&x_lu, &R_x);
        //printf("quadprog_qp LU%i\n", n);
        //printf("%.9f\n", mean_qp_lu_time);
        //printf("quadprog_qp LU, mean_time(s): %.9f, 1E-05 Tolerance:%s\n\n", mean_qp_lu_time, qp_lu_ans?"success":"failure");

        //printf("Ax LU:\n");
        //matf32_print(&Ax);

        // q = (1/2)x'Qx + x'c
        matf32_trans(&x_lu, &xt);
        matf32_mul(&xt, &c, &xtc);
        matf32_mul(&xt, &Q, &xtQ);
        matf32_mul(&xtQ, &x_lu, &xtQx);
        matf32_scale(&xtQx, 0.5, &xtQx);

        printf("q = (1/2)x'Qx + x'c:\n");
        matf32_print(&xtQx);

        // --------------------------------------------------
        // quadprog_qp SVD
        // --------------------------------------------------
        printf("\n-------------------------\n");
        printf("SVD %ix%i\n", n, n);
        printf("-------------------------\n");

        float qp_svd_time[z];
        float mean_qp_svd_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            time = clock();
            quadprog_qp(&qp, &x_svd, SVD);
            qp_svd_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_qp_svd_time = mean(qp_svd_time, z);

        matf32_mul(&Aeq, &x_svd, &Ax);
        bool qp_svd_ans = matf32_is_equal(&x_svd, &R_x);
        //printf("quadprog_qp SVD%i\n", n);
        //printf("%.9f\n", mean_qp_svd_time);
        //printf("quadprog_qp SVD, mean_time(s): %.9f, 1E-05 Tolerance:%s\n\n", mean_qp_svd_time, qp_svd_ans?"success":"failure");
        
        //printf("Ax SVD:\n");
        //matf32_print(&Ax);

        // q = (1/2)x'Qx + x'c
        matf32_trans(&x_svd, &xt);
        matf32_mul(&xt, &c, &xtc);
        matf32_mul(&xt, &Q, &xtQ);
        matf32_mul(&xtQ, &x_svd, &xtQx);
        matf32_scale(&xtQx, 0.5, &xtQx);

        printf("q = (1/2)x'Qx + x'c:\n");
        matf32_print(&xtQx);

        // --------------------------------------------------
        // quadprog_qp QR
        // --------------------------------------------------
        printf("\n-------------------------\n");
        printf("QR %ix%i\n", n, n);
        printf("-------------------------\n");

        float qp_qr_time[z];
        float mean_qp_qr_time = 0;
        for (uint8_t j = 0; j < z; j++)
        {
            time = clock();
            quadprog_qp(&qp, &x_qr, QR);
            qp_qr_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
        mean_qp_qr_time = mean(qp_qr_time, z);

        matf32_mul(&Aeq, &x_qr, &Ax);
        bool qp_qr_ans = matf32_is_equal(&x_qr, &R_x);
        //printf("quadprog_qp QR%i\n", n);
        //printf("%.9f\n", mean_qp_qr_time);
        //printf("quadprog_qp QR, mean_time(s): %.9f, 1E-05 Tolerance:%s\n\n", mean_qp_qr_time, qp_qr_ans?"success":"failure");

        //printf("Ax QR:\n");
        //matf32_print(&Ax);

        // q = (1/2)x'Qx + x'c
        matf32_trans(&x_qr, &xt);
        matf32_mul(&xt, &c, &xtc);
        matf32_mul(&xt, &Q, &xtQ);
        matf32_mul(&xtQ, &x_qr, &xtQx);
        matf32_scale(&xtQx, 0.5, &xtQx);

        printf("q = (1/2)x'Qx + x'c:\n");
        matf32_print(&xtQx);
    }
}


