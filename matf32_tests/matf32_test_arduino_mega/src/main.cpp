
/**
 * @author Andrea Pineda
 * @date Created 26 Sep 2025, Last Modified: 17 Oct 2026
 * 
 * To test matf32 functions in the Arduino MEGA
 */

#include <Arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "matf32.h"
#include "matf32_matlab_data.h"

// ---------------------------------------------------------------------------
// DEFINITIONS
// ---------------------------------------------------------------------------

float dot_result = 0;

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

//float temp_L_data[MAX_MAT_SIZE];
//matf32_t temp_L;

//float temp_U_data[MAX_MAT_SIZE];
//matf32_t temp_U;

//float temp_Q_data[MAX_MAT_SIZE];
//matf32_t temp_Q;

//float temp_R_data[MAX_MAT_SIZE];
//matf32_t temp_R;

//float U_data[MAX_MAT_SIZE];
//matf32_t U;

//float S_data[MAX_MAT_SIZE];
//matf32_t S;

//float V_data[MAX_MAT_SIZE];
//matf32_t V;

float* A_list[1] = {A_data10};
matf32_t A;

float* C_list[1] = {C_data10};
matf32_t C;

float* rowvec_list[1] = {rowvec_data10};
matf32_t rowvec;

float* colvec_list[1] = {colvec_data10};
matf32_t colvec;

// Condition numbers for all input matrices used
float cond_A = 0;
float cond_B = 0;
float cond_C = 0;
float cond_D = 0;

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

    Serial.println();
    Serial.println();

    for (uint8_t i = 0; i < 1; i++)
    {
        //uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        uint8_t n = 10;

        matf32_init(&temp, n, n, temp_data);
        //matf32_init(&temp_L, n, n, temp_L_data);
        //matf32_init(&temp_U, n, n, temp_U_data);
        //matf32_init(&temp_Q, n, n, temp_Q_data);
        //matf32_init(&temp_R, n, n, temp_R_data);
        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&C, n, n, C_list[i]);
        matf32_init(&rowvec, 1, n, rowvec_list[i]);
        matf32_init(&colvec, n, 1, colvec_list[i]);
        
        /*if (n > 1)
        {
            // SVD matrices
            matf32_init(&U, A.num_rows, A.num_cols, U_data);
            matf32_init(&S, A.num_rows, A.num_cols, S_data);
            matf32_init(&V, A.num_rows, A.num_cols, V_data);
        }*/

        const matf32_t* mat_array[3] = {&A, &A, &A};

        // ---------------------------------------------------------------------------
        // matf32_add
        // ---------------------------------------------------------------------------

        /*float add_time[x];

        //Serial.println("\nmatf32_add");
        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_add(&A, &A, &temp);
            add_time[j] = (float)((micros() - tic)/1000000.0);

            //Serial.print((micros() - tic) / 1000000.0, 9);
            //Serial.print(" ");

            //delay(10);
        }
        //Serial.println('\n');

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_add");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(add_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_add");
        //Serial.println(n);
        //Serial.println(mean_add_time,9);
        //Serial.print("matf32_add           , Mean Time (s): ");
        //Serial.print(mean_add_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(add_ans?"True":"False");
        

        // ---------------------------------------------------------------------------
        // matf32_sub
        // ---------------------------------------------------------------------------

        /*float sub_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_sub(&A, &A, &temp);
            sub_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_sub");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(sub_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_sub");
        //Serial.println(n);
        //Serial.println(mean_sub_time,9);
        //Serial.print("matf32_sub           , Mean Time (s): ");
        //Serial.print(mean_sub_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(sub_ans?"True":"False");
        

        // ---------------------------------------------------------------------------
        // matf32_scale
        // ---------------------------------------------------------------------------

        /*float scale_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_scale(&A, c, &temp);
            scale_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_scale");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(scale_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_scale");
        //Serial.println(n);
        //Serial.println(mean_scale_time,9);
        //Serial.print("matf32_scale         , Mean Time (s): ");
        //Serial.print(mean_scale_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(scale_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_trans
        // ---------------------------------------------------------------------------

        /*float trans_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_trans(&A, &temp);
            trans_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_trans");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(trans_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_trans");
        //Serial.println(n);
        //Serial.println(mean_trans_time,9);
        //Serial.print("matf32_trans         , Mean Time (s): ");
        //Serial.print(mean_trans_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(trans_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_mul
        // ---------------------------------------------------------------------------

        /*float mul_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_mul(&A, &A, &temp);
            mul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_mul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(mul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_mul");
        //Serial.println(n);
        //Serial.println(mean_mul_time,9);
        //Serial.print("matf32_mul           , Mean Time (s): ");
        //Serial.print(mean_mul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(mul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_inv
        // ---------------------------------------------------------------------------

        /*float inv_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_inv(&A, &temp);
            inv_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_inv");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(inv_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_inv");
        //Serial.println(n);
        //Serial.println(mean_inv_time,9);
        //Serial.print("matf32_inv           , Mean Time (s): ");
        //Serial.print(mean_inv_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(inv_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_dot
        // ---------------------------------------------------------------------------

        /*float dot_time[x];
        float dot_temp_data = 0;

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_dot(&rowvec, &colvec, &dot_temp_data);
            dot_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_dot");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(dot_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_dot");
        //Serial.println(n);
        //Serial.println(mean_dot_time,9);
        //Serial.print("matf32_dot           , Mean Time (s): ");
        //Serial.print(mean_dot_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(dot_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecposmul
        // ---------------------------------------------------------------------------
        
        /*float vecposmul_time[x];
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
            vecposmul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecposmul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecposmul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_vecposmul");
        //Serial.println(n);
        //Serial.println(mean_vecposmul_time,9);
        //Serial.print("matf32_vecposmul     , Mean Time (s): ");
        //Serial.print(mean_vecposmul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecposmul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecpremul
        // ---------------------------------------------------------------------------

        /*float vecpremul_time[x];
        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
            vecpremul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecpremul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecpremul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_vecpremul");
        //Serial.println(n);
        //Serial.println(mean_vecpremul_time,9);
        //Serial.print("matf32_vecpremul     , Mean Time (s): ");
        //Serial.print(mean_vecpremul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecpremul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_vecmul_col_row
        // ---------------------------------------------------------------------------
        
        /*float vecmul_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
            vecmul_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_vecmul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(vecmul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_vecmul");
        //Serial.println(n);
        //Serial.println(mean_vecmul_time,9);
        //Serial.print("matf32_vecmul_col_row, Mean Time (s): ");
        //Serial.print(mean_vecmul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(vecmul_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_add
        // ---------------------------------------------------------------------------

        /*float arr_add_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_add(mat_array, 3, &temp);
            arr_add_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_add");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_add_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_arr_add");
        //Serial.println(n);
        //Serial.println(mean_arr_add_time,9);
        //Serial.print("matf32_arr_add       , Mean Time (s): ");
        //Serial.print(mean_arr_add_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_add_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_sub
        // ---------------------------------------------------------------------------

        /*float arr_sub_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_sub(mat_array, 3, &temp);
            arr_sub_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_sub");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_sub_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_arr_sub");
        //Serial.println(n);
        //Serial.println(mean_arr_sub_time,9);
        //Serial.print("matf32_arr_sub       , Mean Time (s): ");
        //Serial.print(mean_arr_sub_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_sub_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_arr_mul
        // ---------------------------------------------------------------------------

        /*float arr_mul_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_arr_mul(mat_array, 3, &temp);
            arr_mul_time[j] = (float)((micros() - tic)/1000000.0);   
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_arr_mul");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(arr_mul_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_arr_mul");
        //Serial.println(n);
        //Serial.println(mean_arr_mul_time,9);
        //Serial.print("matf32_arr_mul       , Mean Time (s): ");
        //Serial.print(mean_arr_mul_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(arr_mul_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_exp
        // ---------------------------------------------------------------------------

        /*float exp_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_exp(&A, &temp, 4);
            exp_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_exp");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(exp_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_exp");
        //Serial.println(n);
        //Serial.println(mean_exp_time,9);
        //Serial.print("matf32_exp           , Mean Time (s): ");
        //Serial.print(mean_exp_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(exp_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_pinv
        // ---------------------------------------------------------------------------

        // ------------------------- BASIC PINV -------------------------
        /*float pinv_basic_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_pinv(&A, &temp, BASIC_PINV);
            pinv_basic_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_pinv_basic");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(pinv_basic_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_pinv_basic");
        //Serial.println(n);
        //Serial.println(mean_pinv_basic_time,9);
        //Serial.print("matf32_pinv: Basic   , Mean Time (s): ");
        //Serial.print(mean_pinv_basic_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(pinv_basic_ans?"True":"False");

        // ------------------------- SVD PINV -------------------------
        float pinv_svd_time[x];

        Serial.println("Here");
        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_pinv(&A, &temp, SVD_PINV);
            pinv_svd_time[j] = (float)((micros() - tic)/1000000.0);
            Serial.println(j+1);
        }

        //print_matrix(temp.p_data, temp.num_rows, temp.num_cols);

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_pinv_svd");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(pinv_svd_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();

        //Serial.print("matf32_pinv_svd");
        //Serial.println(n);
        //Serial.println(mean_pinv_svd_time,9);
        //Serial.print("matf32_pinv: SVD     , Mean Time (s): ");
        //Serial.print(mean_pinv_svd_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(pinv_svd_ans?"True":"False");
        
        // ---------------------------------------------------------------------------
        // matf32_lu
        // ---------------------------------------------------------------------------
        
        /*float lu_time[x];
        
        uint16_t p_index[n];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_lu(&A, &temp_L, &temp_U, p_index);
            lu_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_lu");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(lu_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_lu");
        //Serial.println(n);
        //Serial.println(mean_lu_time,9);
        //Serial.print("matf32_lu            , Mean Time (s): ");
        //Serial.print(mean_lu_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(lu_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_cholesky
        // ---------------------------------------------------------------------------

        /*float cholesky_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_cholesky(&C, &temp);
            cholesky_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_cholesky");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(cholesky_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_cholesky");
        //Serial.println(n);
        //Serial.println(mean_cholesky_time,9);
        //Serial.print("matf32_cholesky      , Mean Time (s): ");
        //Serial.print(mean_cholesky_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(cholesky_ans?"True":"False");

        // ---------------------------------------------------------------------------
        // matf32_qr
        // ---------------------------------------------------------------------------

        /*float qr_time[x];

        for (uint8_t j = 0; j < x; j++)
        {
            tic = micros();
            matf32_qr(&A, &temp_Q, &temp_R);
            qr_time[j] = (float)((micros() - tic)/1000000.0);
        }

        // Print time values formatted to a .mat for plotting in matlab
        Serial.print("matf32_qr");
        Serial.println(n);
        for (uint8_t j = 0; j < x; j++)
        {
            Serial.print(qr_time[j], 9);
            Serial.print(" ");
        }
        Serial.println();*/

        //Serial.print("matf32_qr");
        //Serial.println(n);
        //Serial.println(mean_qr_time,9);
        //Serial.print("matf32_qr            , Mean Time (s): ");
        //Serial.print(mean_qr_time,9);
        //Serial.print(", 1E-05 Tolerance: ");
        //Serial.println(qr_ans?"True":"False");
        

        // ---------------------------------------------------------------------------
        // matf32_jacobi_svd
        // ---------------------------------------------------------------------------

        /*if (n > 1)
        {
            float jacobi_svd_time[x];
            float mean_jacobi_svd_time = 0;

            float A_copy_data[MAX_MAT_SIZE];
            matf32_t A_copy;
            matf32_init(&A_copy, A.num_rows, A.num_cols, A_copy_data);
            matf32_submatrix_copy(&A, &A_copy, 0, 0, 0, 0, A.num_rows, A.num_cols);

            //float Vt_data[MAX_MAT_SIZE];
            //matf32_t Vt;
            //matf32_init(&Vt, V.num_cols, V.num_rows, Vt_data);

            //float US_data[MAX_MAT_SIZE];
            //matf32_t US;
            //matf32_init(&US, U.num_rows, S.num_cols, US_data);

            //float USVt_data[MAX_MAT_SIZE];
            //matf32_t USVt;
            //matf32_init(&USVt, U.num_rows, Vt.num_cols, USVt_data);

            for (uint8_t j = 0; j < x; j++)
            {
                tic = micros();
                matf32_jacobi_svd(&A_copy, &U, &S, &V);
                matf32_copy(&A, &A_copy); // Reset A_copy = A, because it's modified in matf32_jacobi_svd
                jacobi_svd_time[j] = (float)((micros() - tic)/1000000.0);
                Serial.println(j+1);
            }
            //matf32_trans(&V, &Vt);
            //matf32_mul(&U, &S, &US);
            //matf32_mul(&US, &Vt, &USVt);

            // Print time values formatted to a .mat for plotting in matlab
            Serial.print("matf32_jacobi_svd");
            Serial.println(n);
            for (uint8_t j = 0; j < x; j++)
            {
                Serial.print(jacobi_svd_time[j], 9);
                Serial.print(" ");
            }
            Serial.println();

            //bool jacobi_svd_ans = matf32_is_equal(&USVt, &A);
            //Serial.print("matf32_jacobi_svd");
            //Serial.println(n);
            //Serial.println(mean_jacobi_svd_time,9);
            //Serial.print("matf32_jacobi_svd    , Mean Time (s): ");
            //Serial.print(mean_jacobi_svd_time,9);
            //Serial.print(", 1E-05 Tolerance: ");
            //Serial.println(jacobi_svd_ans?"True":"False");
        }*/
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
