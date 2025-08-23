
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "matf32.h"
#include "matf32_test_results.h"

float dot_result = 0;

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp_L_data[MAX_MAT_SIZE];
matf32_t temp_L;

float temp_U_data[MAX_MAT_SIZE];
matf32_t temp_U;

float temp_Q_data[MAX_MAT_SIZE];
matf32_t temp_Q;

float temp_R_data[MAX_MAT_SIZE];
matf32_t temp_R;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* B_list[10] = {B_data1, B_data2, B_data3, B_data4, B_data5, B_data6, B_data7, B_data8, B_data9, B_data10};
matf32_t B;

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

float* D_list[9] = {D_data1, D_data2, D_data3, D_data4, D_data5, D_data6, D_data7, D_data8, D_data9};
matf32_t D;

float* rowvec_list[10] = {rowvec_data1, rowvec_data2, rowvec_data3, rowvec_data4, rowvec_data5, rowvec_data6, rowvec_data7, rowvec_data8, rowvec_data9, rowvec_data10};
matf32_t rowvec;

float* colvec_list[10] = {colvec_data1, colvec_data2, colvec_data3, colvec_data4, colvec_data5, colvec_data6, colvec_data7, colvec_data8, colvec_data9, colvec_data10};
matf32_t colvec;

float* R_add_list[10] = {R_add1, R_add2, R_add3, R_add4, R_add5, R_add6, R_add7, R_add8, R_add9, R_add10};
matf32_t R_add;

float* R_sub_list[10] = {R_sub1, R_sub2, R_sub3, R_sub4, R_sub5, R_sub6, R_sub7, R_sub8, R_sub9, R_sub10};
matf32_t R_sub;

float* R_scale_list[10] = {R_scale1, R_scale2, R_scale3, R_scale4, R_scale5, R_scale6, R_scale7, R_scale8, R_scale9, R_scale10};
matf32_t R_scale;

float* R_trans_list[10] = {R_trans1, R_trans2, R_trans3, R_trans4, R_trans5, R_trans6, R_trans7, R_trans8, R_trans9, R_trans10};
matf32_t R_trans;

float* R_mul_list[10] = {R_mul1, R_mul2, R_mul3, R_mul4, R_mul5, R_mul6, R_mul7, R_mul8, R_mul9, R_mul10};
matf32_t R_mul;

float* R_inv_list[10] = {R_inv1, R_inv2, R_inv3, R_inv4, R_inv5, R_inv6, R_inv7, R_inv8, R_inv9, R_inv10};
matf32_t R_inv;

float* R_dot_list[10] = {R_dot1, R_dot2, R_dot3, R_dot4, R_dot5, R_dot6, R_dot7, R_dot8, R_dot9, R_dot10};
matf32_t R_dot;

float* R_vecpremul_list[10] = {R_vecpremul1, R_vecpremul2, R_vecpremul3, R_vecpremul4, R_vecpremul5, R_vecpremul6, R_vecpremul7, R_vecpremul8, R_vecpremul9, R_vecpremul10};
matf32_t R_vecpremul;

float* R_vecposmul_list[10] = {R_vecposmul1, R_vecposmul2, R_vecposmul3, R_vecposmul4, R_vecposmul5, R_vecposmul6, R_vecposmul7, R_vecposmul8, R_vecposmul9, R_vecposmul10};
matf32_t R_vecposmul;

float* R_vecmul_list[10] = {R_vecmul1, R_vecmul2, R_vecmul3, R_vecmul4, R_vecmul5, R_vecmul6, R_vecmul7, R_vecmul8, R_vecmul9, R_vecmul10};
matf32_t R_vecmul;

float* R_arr_add_list[10] = {R_arr_add1, R_arr_add2, R_arr_add3, R_arr_add4, R_arr_add5, R_arr_add6, R_arr_add7, R_arr_add8, R_arr_add9, R_arr_add10};
matf32_t R_arr_add;

float* R_arr_sub_list[10] = {R_arr_sub1, R_arr_sub2, R_arr_sub3, R_arr_sub4, R_arr_sub5, R_arr_sub6, R_arr_sub7, R_arr_sub8, R_arr_sub9, R_arr_sub10};
matf32_t R_arr_sub;

float* R_arr_mul_list[10] = {R_arr_mul1, R_arr_mul2, R_arr_mul3, R_arr_mul4, R_arr_mul5, R_arr_mul6, R_arr_mul7, R_arr_mul8, R_arr_mul9, R_arr_mul10};
matf32_t R_arr_mul;

float* R_pinv_list[10] = {R_pinv1, R_pinv2, R_pinv3, R_pinv4, R_pinv5, R_pinv6, R_pinv7, R_pinv8, R_pinv9, R_pinv10};
matf32_t R_pinv;

float* R_lu_L_list[10] = {R_lu_L1, R_lu_L2, R_lu_L3, R_lu_L4, R_lu_L5, R_lu_L6, R_lu_L7, R_lu_L8, R_lu_L9, R_lu_L10};
matf32_t R_lu_L;

float* R_lu_U_list[10] = {R_lu_U1, R_lu_U2, R_lu_U3, R_lu_U4, R_lu_U5, R_lu_U6, R_lu_U7,  R_lu_U8, R_lu_U9, R_lu_U10};
matf32_t R_lu_U;

float* R_cholesky_list[10] = {R_cholesky1, R_cholesky2, R_cholesky3, R_cholesky4, R_cholesky5, R_cholesky6, R_cholesky7, R_cholesky8, R_cholesky9, R_cholesky10};
matf32_t R_cholesky;

float* R_qr_Q_list[9] = {R_qr_Q1, R_qr_Q2, R_qr_Q3, R_qr_Q4, R_qr_Q5, R_qr_Q6, R_qr_Q7, R_qr_Q8, R_qr_Q9};
matf32_t R_qr_Q;

float* R_qr_R_list[9] = {R_qr_R1, R_qr_R2, R_qr_R3, R_qr_R4, R_qr_R5, R_qr_R6, R_qr_R7, R_qr_R8, R_qr_R9};
matf32_t R_qr_R;

float cond_A = 0;
float cond_B = 0;
float cond_C = 0;
float cond_D = 0;

void main(void)
{
    clock_t time;

    printf("\n\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&temp, n, n, temp_data);
        matf32_init(&temp_L, n, n, temp_L_data);
        matf32_init(&temp_U, n, n, temp_U_data);
        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&B, n, n, B_list[i]);
        matf32_init(&C, n, n, C_list[i]);
        matf32_init(&rowvec, 1, n, rowvec_list[i]);
        matf32_init(&colvec, n, 1, colvec_list[i]);
        matf32_init(&R_add, n, n, R_add_list[i]);
        matf32_init(&R_sub, n, n, R_sub_list[i]);
        matf32_init(&R_scale, n, n, R_scale_list[i]);
        matf32_init(&R_trans, n, n, R_trans_list[i]);
        matf32_init(&R_mul, n, n, R_mul_list[i]);
        matf32_init(&R_inv, n, n, R_inv_list[i]);
        matf32_init(&R_dot, 1, 1, R_dot_list[i]);
        matf32_init(&R_vecpremul, 1, n, R_vecpremul_list[i]);
        matf32_init(&R_vecposmul, n, 1, R_vecposmul_list[i]);
        matf32_init(&R_vecmul, n, n, R_vecmul_list[i]);
        matf32_init(&R_arr_add, n, n, R_arr_add_list[i]);
        matf32_init(&R_arr_sub, n, n, R_arr_sub_list[i]);
        matf32_init(&R_arr_mul, n, n, R_arr_mul_list[i]);
        matf32_init(&R_pinv, n, n, R_pinv_list[i]);
        matf32_init(&R_lu_L, n, n, R_lu_L_list[i]);
        matf32_init(&R_lu_U, n, n, R_lu_U_list[i]);
        matf32_init(&R_cholesky, n, n, R_cholesky_list[i]);
        
        // D, Q and R have only 9 elements in the list of values (no element for n=1 as D is a rectangular matrix n x (n-1))
        if (n > 1)
        {
            matf32_init(&temp_Q, n, n, temp_Q_data);
            matf32_init(&temp_R, n, n-1, temp_R_data);
            matf32_init(&D, n, n-1, D_list[i-1]);
            matf32_init(&R_qr_Q, n, n, R_qr_Q_list[i-1]);
            matf32_init(&R_qr_R, n, n-1, R_qr_R_list[i-1]);
        }

        const matf32_t* mat_array[3] = {&A, &A, &A};
        
        printf("\n--------------------------------------------------\n");
        printf("n = %i\n", n);
        printf("--------------------------------------------------\n");

        matf32_cond(&A, &cond_A);
        matf32_cond(&B, &cond_B);
        matf32_cond(&C, &cond_C);
        matf32_cond(&D, &cond_D);

        printf("cond_A = %.9f\n", cond_A);
        printf("cond_B = %.9f\n", cond_B);
        printf("cond_C = %.9f\n", cond_C);
        printf("cond D = %.9f\n", cond_D);

        // ---------------------------------------------------------------------------
        // 1. matf32_add
        // ---------------------------------------------------------------------------

        float add_time[x];
        float mean_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_add(&A, &A, &temp);
            add_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_add_time = mean(add_time, x);

        bool add_ans = matf32_is_equal(&temp, &R_add);
        //printf("matf32_add%i\n",n);
        //printf("%.9f\n", mean_add_time);
        printf("add      ,time(s):%.9f,%s\n", mean_add_time, add_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 2. matf32_sub
        // ---------------------------------------------------------------------------

        float sub_time[x];
        float mean_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_sub(&A, &A, &temp);
            sub_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_sub_time = mean(sub_time, x);

        bool sub_ans = matf32_is_equal(&temp, &R_sub);
        //printf("matf32_sub%i\n", n);
        //printf("%.9f\n", mean_sub_time);
        printf("sub      ,time(s):%.9f,%s\n", mean_sub_time, sub_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 3. matf32_scale
        // ---------------------------------------------------------------------------

        float scale_time[x];
        float mean_scale_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_scale(&A, c, &temp);
            scale_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_scale_time = mean(scale_time, x);

        bool scale_ans = matf32_is_equal(&temp, &R_scale);
        //printf("matf32_scale%i\n", n);
        //printf("%.9f\n", mean_scale_time);
        printf("scale    ,time(s):%.9f,%s\n", mean_scale_time, scale_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 4. matf32_trans
        // ---------------------------------------------------------------------------

        float trans_time[x];
        float mean_trans_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_trans(&A, &temp);
            trans_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_trans_time = mean(trans_time, x);

        bool trans_ans = matf32_is_equal(&temp, &R_trans);
        //printf("matf32_trans%i\n", n);
        //printf("%.9f\n", mean_trans_time);
        printf("trans    ,time(s):%.9f,%s\n", mean_trans_time, trans_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 5. matf32_mul
        // ---------------------------------------------------------------------------

        float mul_time[x];
        float mean_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_mul(&A, &A, &temp);
            mul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_mul_time = mean(mul_time, x);

        bool mul_ans = matf32_is_equal(&temp, &R_mul);
        //printf("matf32_mul%i\n", n);
        //printf("%.9f\n", mean_mul_time);
        printf("mul      ,time(s):%.9f,%s\n", mean_mul_time, mul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 6. matf32_inv
        // ---------------------------------------------------------------------------

        float inv_time[x];
        float mean_inv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_inv(&A, &temp);
            inv_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }
    
        mean_inv_time = mean(inv_time, x);

        bool inv_ans = matf32_is_equal(&temp, &R_inv);
        //printf("matf32_inv%i\n", n);
        //printf("%.9f\n", mean_inv_time);
        printf("inv      ,time(s):%.9f,%s\n", mean_inv_time, inv_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 7. matf32_dot
        // ---------------------------------------------------------------------------

        float dot_time[x];
        float mean_dot_time = 0;

        float dot_temp_data[1];
        matf32_t dot_temp;

        matf32_init(&dot_temp, 1, 1, dot_temp_data);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_dot(&rowvec, &colvec, dot_temp_data);
            dot_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        matf32_set(&dot_temp, 1, 1, *dot_temp_data);

        mean_dot_time = mean(dot_time, x);

        bool dot_ans = matf32_is_equal(&dot_temp, &R_dot);
        //printf("matf32_dot%i\n", n);
        //printf("%.9f\n", mean_dot_time);
        printf("dot      ,time(s):%.9f,%s\n", mean_dot_time, dot_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 8. matf32_vecposmul
        // ---------------------------------------------------------------------------
        
        float vecposmul_time[x];
        float mean_vecposmul_time = 0;
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
            vecposmul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecposmul_time = mean(vecposmul_time, x);

        bool vecposmul_ans = matf32_is_equal(&vecposmul_temp, &R_vecposmul);
        //printf("matf32_vecposmul%i\n", n);
        //printf("%.9f\n", mean_vecposmul_time);
        printf("vecposmul,time(s):%.9f,%s\n", mean_vecposmul_time, vecposmul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 9. matf32_vecpremul
        // ---------------------------------------------------------------------------

        float vecpremul_time[x];
        float mean_vecpremul_time = 0;
        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
            vecpremul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecpremul_time = mean(vecpremul_time, x);

        bool vecpremul_ans = matf32_is_equal(&vecpremul_temp, &R_vecpremul);
        //printf("matf32_vecpremul%i\n", n);
        //printf("%.9f\n", mean_vecpremul_time);
        printf("vecpremul,time(s):%.9f,%s\n", mean_vecpremul_time, vecpremul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 10. matf32_vecmul_col_row
        // ---------------------------------------------------------------------------
        
        float vecmul_time[x];
        float mean_vecmul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
            vecmul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_vecmul_time = mean(vecmul_time, x);

        bool vecmul_ans = matf32_is_equal(&temp, &R_vecmul);
        //printf("matf32_vecmul%i\n", n);
        //printf("%.9f\n", mean_vecmul_time);
        printf("vecmul   ,time(s):%.9f,%s\n", mean_vecmul_time, vecmul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 11. matf32_arr_add
        // ---------------------------------------------------------------------------

        float arr_add_time[x];
        float mean_arr_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_add(mat_array, 3, &temp);
            arr_add_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_add_time = mean(arr_add_time, x);

        bool arr_add_ans = matf32_is_equal(&temp, &R_arr_add);
        //printf("matf32_arr_add%i\n", n);
        //printf("%.9f\n", mean_arr_add_time);
        printf("arr_add  ,time(s):%.9f,%s\n", mean_arr_add_time, arr_add_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 12. matf32_arr_sub
        // ---------------------------------------------------------------------------

        float arr_sub_time[x];
        float mean_arr_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_sub(mat_array, 3, &temp);
            arr_sub_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_sub_time = mean(arr_sub_time, x);

        bool arr_sub_ans = matf32_is_equal(&temp, &R_arr_sub);
        //printf("matf32_arr_sub%i\n", n);
        //printf("%.9f\n", mean_arr_sub_time);
        printf("arr_sub  ,time(s):%.9f,%s\n", mean_arr_sub_time, arr_sub_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 13. matf32_arr_mul
        // ---------------------------------------------------------------------------

        float arr_mul_time[x];
        float mean_arr_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_arr_mul(mat_array, 3, &temp);
            arr_mul_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_arr_mul_time = mean(arr_mul_time, x);

        bool arr_mul_ans = matf32_is_equal(&temp, &R_arr_mul);
        //printf("matf32_arr_mul%i\n", n);
        //printf("%.9f\n", mean_arr_mul_time);
        printf("arr_mul  ,time(s):%.9f,%s\n", mean_arr_mul_time, arr_mul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 14. matf32_pinv
        // ---------------------------------------------------------------------------

        float pinv_time[x];
        float mean_pinv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_pinv(&A, &temp);
            pinv_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_pinv_time = mean(pinv_time, x);

        bool pinv_ans = matf32_is_equal(&temp, &R_pinv);
        //printf("matf32_pinv%i\n",n);
        //printf("%.9f\n", mean_pinv_time);
        printf("pinv     ,time(s):%.9f,%s\n", mean_pinv_time, pinv_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 15. matf32_lu
        // ---------------------------------------------------------------------------
        
        float lu_time[x];
        float mean_lu_time = 0;

        matf32_zeros(&temp_L);
        matf32_zeros(&temp_U);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_lu(&B, &temp_L, &temp_U);
            lu_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        mean_lu_time = mean(lu_time, x);

        bool lu_L_ans = matf32_is_equal(&temp_L, &R_lu_L);
        bool lu_U_ans = matf32_is_equal(&temp_U, &R_lu_U);
        bool lu_B_ans = matf32_is_equal(&temp, &B);
        printf("lu       ,time(s):%.9f,L:%s,U:%s,B = L*U:%s\n", mean_lu_time, lu_L_ans?"success":"failure", lu_U_ans?"success":"failure", lu_B_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 16. matf32_cholesky
        // ---------------------------------------------------------------------------

        float cholesky_time[x];
        float mean_cholesky_time = 0;

        matf32_zeros(&temp);

        for (uint8_t j = 0; j < x; j++)
        {
            time = clock();
            matf32_cholesky(&C, &temp);
            cholesky_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
        }

        // problema con inversas: QR o SVD con un método sencillo porque implementar eigenvalores es complicado
        // Si se implementa SVD, revisar si se implementaria operaciones de eigenvalores?
        // Regularizaciones (especialmente tikhonov).
        // Pseudoinversa para resover el sistema lineal, en lugar de la inversa (este sí colocarlo en matf32)
        // Métodos iterativos: revisar si necesitan acondicionamiento (todavía pendiente)

        mean_cholesky_time = mean(cholesky_time, x);

        bool cholesky_ans = matf32_is_equal(&temp, &R_cholesky);
        //printf("matf32_cholesky%i\n", n);
        //printf("%.9f\n", mean_cholesky_time);
        printf("cholesky ,time(s):%.9f,%s\n", mean_cholesky_time, cholesky_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // 17. matf32_qr
        // ---------------------------------------------------------------------------

        if (n > 1)
        {
            float qr_time[x];
            float mean_qr_time = 0;

            matf32_zeros(&temp_Q);
            matf32_zeros(&temp_R);

            for (uint8_t j = 0; j < x; j++)
            {
                time = clock();
                matf32_qr(&D, &temp_Q, &temp_R);
                qr_time[j] = (float)(clock()-time)/CLOCKS_PER_SEC;
            }
    
            mean_qr_time = mean(qr_time, x);

            bool qr_Q_ans = matf32_is_equal(&temp_Q, &R_qr_Q);
            bool qr_R_ans = matf32_is_equal(&temp_R, &R_qr_R);
            //printf("matf32_qr%i\n", n);
            //printf("%.9f\n", mean_qr_time);
            printf("qr       ,time(s):%.9f,Q:%s,R:%s\n", mean_qr_time, qr_Q_ans?"success":"failure", qr_R_ans?"success":"failure");
        }
    }
}