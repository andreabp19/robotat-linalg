
/**
 * @author Andrea Pineda
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "matf32.h"
#include "linsolve.h"
#include "matf32_math_test_results.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------

float dot_result = 0;

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

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

float* R_lu_L_list[10] = {R_lu_L1, R_lu_L2, R_lu_L3, R_lu_L4, R_lu_L5, R_lu_L6, R_lu_L7, R_lu_L8, R_lu_L9, R_lu_L10};
matf32_t R_lu_L;

float* R_lu_U_list[10] = {R_lu_U1, R_lu_U2, R_lu_U3, R_lu_U4, R_lu_U5, R_lu_U6, R_lu_U7, R_lu_U8, R_lu_U9, R_lu_U10};
matf32_t R_lu_U;

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

unsigned long tic = 0;
unsigned long toc = 0;

void setup()
{
    // ---------------------------------------------------------------------------
    // SERIAL CONFIG
    // ---------------------------------------------------------------------------
    Serial.begin(115200);
    delay(1500);

    // ---------------------------------------------------------------------------
    // MATF32 TESTING
    // ---------------------------------------------------------------------------
    
    Serial.println("\n\n--------------------------------------------------");
    Serial.println("MATF32 TESTING");
    Serial.println("--------------------------------------------------\n\n");

    Serial.printf("#ID      ,n ,time       ,result\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&rowvec, 1, n, rowvec_list[i]);
        matf32_init(&colvec, n, 1, colvec_list[i]);
        matf32_init(&R_add, n, n, R_add_list[i]);
        matf32_init(&R_sub, n, n, R_sub_list[i]);
        matf32_init(&R_scale, n, n, R_scale_list[i]);
        matf32_init(&R_trans, n, n, R_trans_list[i]);
        matf32_init(&R_mul, n, n, R_mul_list[i]);
        matf32_init(&R_lu_L, n, n, R_lu_L_list[i]);
        matf32_init(&R_lu_U, n, n, R_lu_U_list[i]);
        matf32_init(&R_inv, n, n, R_inv_list[i]);
        matf32_init(&R_dot, n, n, R_inv_list[i]);
        matf32_init(&R_vecpremul, 1, n, R_vecpremul_list[i]);
        matf32_init(&R_vecposmul, n, 1, R_vecposmul_list[i]);
        matf32_init(&R_vecmul, n, n, R_vecmul_list[i]);
        matf32_init(&R_arr_add, n, n, R_arr_add_list[i]);
        matf32_init(&R_arr_sub, n, n, R_arr_sub_list[i]);
        matf32_init(&R_arr_mul, n, n, R_arr_mul_list[i]);
        matf32_init(&temp, n, n, temp_data);

        const matf32_t* mat_array[3] = {&A, &A, &A};

        // ---------------------------------------------------------------------------
        // TEST ADD
        // ---------------------------------------------------------------------------

        float add_time = 0;
        float mean_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_add(&A, &A, &temp);
            toc = micros() - tic;
            add_time += (float)toc;
        }

        mean_add_time = add_time / x;

        bool add_ans = matf32_is_equal(&temp, &R_add);
        Serial.printf("add      ,%02i,%.9f,%s\n", n, mean_add_time/1000000.0, add_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST SUB
        // ---------------------------------------------------------------------------

        float sub_time = 0;
        float mean_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_sub(&A, &A, &temp);
            toc = micros() - tic;
            sub_time += (float)toc;
        }

        mean_sub_time = sub_time / x;

        bool sub_ans = matf32_is_equal(&temp, &R_sub);
        Serial.printf("sub      ,%02i,%.9f,%s\n", n, mean_sub_time/1000000.0, sub_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST SCALE
        // ---------------------------------------------------------------------------

        float scale_time = 0;
        float mean_scale_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_scale(&A, c, &temp);
            toc = micros() - tic;
            scale_time += (float)toc;
        }

        mean_scale_time = scale_time / x;

        bool scale_ans = matf32_is_equal(&temp, &R_scale);
        Serial.printf("scale    ,%02i,%.9f,%s\n", n, mean_scale_time/1000000.0, scale_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST TRANS
        // ---------------------------------------------------------------------------

        float trans_time = 0;
        float mean_trans_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_trans(&A, &temp);
            toc = micros() - tic;
            trans_time += (float)toc;
        }

        mean_trans_time = trans_time / x;

        bool trans_ans = matf32_is_equal(&temp, &R_trans);
        Serial.printf("trans    ,%02i,%.9f,%s\n", n, mean_trans_time/1000000.0, trans_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST MUL
        // ---------------------------------------------------------------------------

        float mul_time = 0;
        float mean_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_mul(&A, &A, &temp);
            toc = micros() - tic;
            mul_time += (float)toc;
        }

        mean_mul_time = mul_time / x;

        bool mul_ans = matf32_is_equal(&temp, &R_mul);
        Serial.printf("mul      ,%02i,%.9f,%s\n", n, mean_mul_time/1000000.0, mul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST INV
        // ---------------------------------------------------------------------------

        float inv_time = 0;
        float mean_inv_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_inv(&A, &temp);
            toc = micros() - tic;
            inv_time += (float)toc;
        }

        mean_inv_time = inv_time / x;

        bool inv_ans = matf32_is_equal(&temp, &R_inv);
        Serial.printf("inv      ,%02i,%.9f,%s\n", n, mean_inv_time/1000000.0, inv_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST DOT
        // ---------------------------------------------------------------------------

        float dot_time = 0;
        float mean_dot_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_dot(&rowvec, &colvec, &dot_result);
            toc = micros() - tic;
            dot_time += (float)toc;
        }

        mean_dot_time = dot_time / x;

        bool dot_ans = matf32_is_equal(&temp, &R_dot);

        Serial.printf("dot      ,%02i,%.9f,%s\n", n, mean_dot_time/1000000.0, dot_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST VECPOSMUL
        // ---------------------------------------------------------------------------
        
        float vecposmul_time = 0;
        float mean_vecposmul_time = 0;
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
            toc = micros() - tic;
            vecposmul_time += (float)toc;
        }

        mean_vecposmul_time = vecposmul_time / x;
        bool vecposmul_ans = matf32_is_equal(&vecposmul_temp, &R_vecposmul);

        Serial.printf("vecposmul,%02i,%.9f,%s\n", n, mean_vecposmul_time/1000000.0, vecposmul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST VECPREMUL
        // ---------------------------------------------------------------------------

        float vecpremul_time = 0;
        float mean_vecpremul_time = 0;
        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
            toc = micros() - tic;
            vecpremul_time += (float)toc;
        }

        mean_vecpremul_time = vecpremul_time / x;

        bool vecpremul_ans = matf32_is_equal(&vecpremul_temp, &R_vecpremul);
        Serial.printf("vecpremul,%02i,%.9f,%s\n", n, mean_vecpremul_time/1000000.0, vecpremul_ans?"success":"failure");
        
        // ---------------------------------------------------------------------------
        // TEST VECMUL COL ROW
        // ---------------------------------------------------------------------------
        
        float vecmul_time = 0;
        float mean_vecmul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
            toc = micros() - tic;
            vecmul_time += (float)toc;
        }

        mean_vecmul_time = vecmul_time / x;

        bool vecmul_ans = matf32_is_equal(&temp, &R_vecmul);
        Serial.printf("vecmul   ,%02i,%.9f,%s\n", n, mean_vecmul_time/1000000.0, vecmul_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST ARR ADD
        // ---------------------------------------------------------------------------

        float arr_add_time = 0;
        float mean_arr_add_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_add(mat_array, 3, &temp);
            toc = micros() - tic;
            arr_add_time += (float)toc;
        }

        mean_arr_add_time = arr_add_time / x;

        bool arr_add_ans = matf32_is_equal(&temp, &R_arr_add);
        Serial.printf("arr_add  ,%02i,%.9f,%s\n", n, mean_arr_add_time/1000000.0, arr_add_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST ARR SUB
        // ---------------------------------------------------------------------------

        float arr_sub_time = 0;
        float mean_arr_sub_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_sub(mat_array, 3, &temp);
            toc = micros() - tic;
            arr_sub_time += (float)toc;
        }

        mean_arr_sub_time = arr_sub_time / x;

        bool arr_sub_ans = matf32_is_equal(&temp, &R_arr_sub);
        Serial.printf("arr_sub  ,%02i,%.9f,%s\n", n, mean_arr_sub_time/1000000.0, arr_sub_ans?"success":"failure");

        // ---------------------------------------------------------------------------
        // TEST ARR MUL
        // ---------------------------------------------------------------------------

        float arr_mul_time = 0;
        float mean_arr_mul_time = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_mul(mat_array, 3, &temp);
            toc = micros() - tic;
            arr_mul_time += (float)toc;
        }

        mean_arr_mul_time = arr_mul_time / x;

        bool arr_mul_ans = matf32_is_equal(&temp, &R_arr_mul);
        Serial.printf("arr_mul  ,%02i,%.9f,%s\n", n, mean_arr_mul_time/1000000.0, arr_mul_ans?"success":"failure");
    }
}

void loop()
{
    
}