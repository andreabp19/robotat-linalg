
/**
 * @author Andrea Pineda
 * @date Created 23 Aug 2025, last modified: 16 Nov. 2026
 * 
 * For testing matf32 in computer
 */

#include <stdio.h>

#include "matf32.h"
#include "matf32_matlab_data.h"

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

float U_data[MAX_MAT_SIZE];
matf32_t U;

float S_data[MAX_MAT_SIZE];
matf32_t S;

float V_data[MAX_MAT_SIZE];
matf32_t V;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

float* rowvec_list[10] = {rowvec_data1, rowvec_data2, rowvec_data3, rowvec_data4, rowvec_data5, rowvec_data6, rowvec_data7, rowvec_data8, rowvec_data9, rowvec_data10};
matf32_t rowvec;

float* colvec_list[10] = {colvec_data1, colvec_data2, colvec_data3, colvec_data4, colvec_data5, colvec_data6, colvec_data7, colvec_data8, colvec_data9, colvec_data10};
matf32_t colvec;


// Matrix operations Matlab Results
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

float* R_exp_list[10] = {R_exp1, R_exp2, R_exp3, R_exp4, R_exp5, R_exp6, R_exp7, R_exp8, R_exp9, R_exp10};
matf32_t R_exp;

float* R_pinv_basic_list[10] = {R_pinv_basic1, R_pinv_basic2, R_pinv_basic3, R_pinv_basic4, R_pinv_basic5, R_pinv_basic6, R_pinv_basic7, R_pinv_basic8, R_pinv_basic9, R_pinv_basic10};
matf32_t R_pinv_basic;

float* R_pinv_svd_list[10] = {R_pinv_svd1, R_pinv_svd2, R_pinv_svd3, R_pinv_svd4, R_pinv_svd5, R_pinv_svd6, R_pinv_svd7, R_pinv_svd8, R_pinv_svd9, R_pinv_svd10};
matf32_t R_pinv_svd;


// Matrix Factorizations Matlab Results
float* R_lu_list[10] = {R_lu1, R_lu2, R_lu3, R_lu4, R_lu5, R_lu6, R_lu7, R_lu8, R_lu9, R_lu10};
matf32_t R_lu;

float* R_cholesky_list[10] = {R_cholesky1, R_cholesky2, R_cholesky3, R_cholesky4, R_cholesky5, R_cholesky6, R_cholesky7, R_cholesky8, R_cholesky9, R_cholesky10};
matf32_t R_cholesky;

float* R_qr_list[10] = {R_qr1, R_qr2, R_qr3, R_qr4, R_qr5, R_qr6, R_qr7, R_qr8, R_qr9, R_qr10};
matf32_t R_qr;

float* R_jacobi_svd_list[9] = {R_jacobi_svd2, R_jacobi_svd3, R_jacobi_svd4, R_jacobi_svd5, R_jacobi_svd6, R_jacobi_svd7, R_jacobi_svd8, R_jacobi_svd9, R_jacobi_svd10};
matf32_t R_jacobi_svd;

float A_QR_data[MAX_MAT_SIZE];
matf32_t A_QR;

float PA_data[MAX_MAT_SIZE];
matf32_t PA;

float Vt_data[MAX_MAT_SIZE];
matf32_t Vt;
float US_data[MAX_MAT_SIZE];
matf32_t US;
float USVt_data[MAX_MAT_SIZE];
matf32_t USVt;

float cond_A = 0;

uint16_t total_tests_counter = 0;
uint16_t fails_counter = 0;

void check_test_case(const char *routine, int m, int n, bool result, uint16_t *total_tests_counter, uint16_t *fails_counter, float cond_A, float max_error);

float matf32_max_numerical_error(const matf32_t* const A, const matf32_t* const B);

int main(void)
{
    printf("\n\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1; // i is 1 value below the dimension n, for each matrix

        uint16_t p_index[n]; // Permutations vector for LU factorization

        matf32_init(&temp, n, n, temp_data);
        matf32_init(&temp_L, n, n, temp_L_data);
        matf32_init(&temp_U, n, n, temp_U_data);
        matf32_init(&temp_Q, n, n, temp_Q_data);
        matf32_init(&temp_R, n, n, temp_R_data);
        matf32_init(&A, n, n, A_list[i]);
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
        matf32_init(&R_exp, n, n, R_exp_list[i]);
        matf32_init(&R_pinv_basic, n, n, R_pinv_basic_list[i]);
        matf32_init(&R_pinv_svd, n, n, R_pinv_svd_list[i]);
        matf32_init(&R_lu, n, n, R_lu_list[i]);
        matf32_init(&R_cholesky, n, n, R_cholesky_list[i]);
        matf32_init(&R_qr, n, n, R_qr_list[i]);
        matf32_init(&A_QR, A.num_rows, A.num_cols, A_QR_data);
        matf32_init(&PA, A.num_rows, A.num_cols, PA_data);

        matf32_zeros(&temp_L);
        matf32_zeros(&temp_U);
        matf32_zeros(&temp);
        matf32_zeros(&temp_Q);
        matf32_zeros(&temp_R);
        
        if (n > 1)
        {
            // SVD matrices
            matf32_init(&U, A.num_rows, A.num_cols, U_data);
            matf32_init(&S, A.num_rows, A.num_cols, S_data);
            matf32_init(&V, A.num_rows, A.num_cols, V_data);
            matf32_init(&Vt, V.num_cols, V.num_rows, Vt_data);
            matf32_init(&US, U.num_rows, S.num_cols, US_data);
            matf32_init(&USVt, U.num_rows, Vt.num_cols, USVt_data);
            matf32_init(&R_jacobi_svd, n, n, R_jacobi_svd_list[i-1]);
        }

        const matf32_t* mat_array[3] = {&A, &A, &A};

        matf32_cond(&A, &cond_A);

        // ---------------------------------------------------------------------------
        // matf32_add
        // ---------------------------------------------------------------------------

        matf32_add(&A, &A, &temp);
        check_test_case("matf32_add", n, n, matf32_is_equal(&temp, &R_add), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_add));

        // ---------------------------------------------------------------------------
        // matf32_sub
        // ---------------------------------------------------------------------------

        matf32_sub(&A, &A, &temp);
        check_test_case("matf32_sub", n, n, matf32_is_equal(&temp, &R_sub), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_sub));

        // ---------------------------------------------------------------------------
        // matf32_scale
        // ---------------------------------------------------------------------------
        
        matf32_scale(&A, c, &temp);
        check_test_case("matf32_scale", n, n, matf32_is_equal(&temp, &R_scale), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_scale));
        
        // ---------------------------------------------------------------------------
        // matf32_trans
        // ---------------------------------------------------------------------------

        matf32_trans(&A, &temp);
        check_test_case("matf32_trans", n, n, matf32_is_equal(&temp, &R_trans), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_trans));
        
        // ---------------------------------------------------------------------------
        // matf32_mul
        // ---------------------------------------------------------------------------

        matf32_mul(&A, &A, &temp);
        check_test_case("matf32_mul", n, n, matf32_is_equal(&temp, &R_mul), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_mul));

        // ---------------------------------------------------------------------------
        // matf32_inv
        // ---------------------------------------------------------------------------

        matf32_inv(&A, &temp);
        check_test_case("matf32_inv", n, n, matf32_is_equal(&temp, &R_inv), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_inv));

        // ---------------------------------------------------------------------------
        // matf32_dot
        // ---------------------------------------------------------------------------

        float dot_temp_data[1];
        matf32_t dot_temp;

        matf32_init(&dot_temp, 1, 1, dot_temp_data);
        matf32_dot(&rowvec, &colvec, dot_temp_data);
        matf32_set(&dot_temp, 1, 1, *dot_temp_data);

        check_test_case("matf32_dot", n, n, matf32_is_equal(&dot_temp, &R_dot), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_dot));

        // ---------------------------------------------------------------------------
        // matf32_vecposmul
        // ---------------------------------------------------------------------------
        
        float vecposmul_result[n];

        matf32_t vecposmul_temp;
        matf32_init(&vecposmul_temp, n, 1, vecposmul_result);

        matf32_vecposmul(&A, colvec.p_data, vecposmul_temp.p_data);
        check_test_case("matf32_vecposmul", n, n, matf32_is_equal(&vecposmul_temp, &R_vecposmul), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&vecposmul_temp, &R_vecposmul));

        // ---------------------------------------------------------------------------
        // matf32_vecpremul
        // ---------------------------------------------------------------------------

        float vecpremul_result[n];

        matf32_t vecpremul_temp;
        matf32_init(&vecpremul_temp, 1, n, vecpremul_result);

        matf32_vecpremul(&A, rowvec.p_data, vecpremul_temp.p_data);
        check_test_case("matf32_vecpremul", n, n, matf32_is_equal(&vecpremul_temp, &R_vecpremul), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&vecpremul_temp, &R_vecpremul));

        // ---------------------------------------------------------------------------
        // matf32_vecmul_col_row
        // ---------------------------------------------------------------------------

        matf32_vecmul_col_row(colvec_list[i], rowvec_list[i], &temp);
        check_test_case("matf32_vecmul", n, n, matf32_is_equal(&temp, &R_vecmul), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_vecmul));

        // ---------------------------------------------------------------------------
        // matf32_arr_add
        // ---------------------------------------------------------------------------

        matf32_arr_add(mat_array, 3, &temp);
        check_test_case("matf32_arr_add", n, n, matf32_is_equal(&temp, &R_arr_add), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_arr_add));

        // ---------------------------------------------------------------------------
        // matf32_arr_sub
        // ---------------------------------------------------------------------------

        matf32_arr_sub(mat_array, 3, &temp);
        check_test_case("matf32_arr_sub", n, n, matf32_is_equal(&temp, &R_arr_sub), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_arr_sub));

        // ---------------------------------------------------------------------------
        // matf32_arr_mul
        // ---------------------------------------------------------------------------

        matf32_arr_mul(mat_array, 3, &temp);
        check_test_case("matf32_arr_mul", n, n, matf32_is_equal(&temp, &R_arr_mul), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_arr_mul));

        // ---------------------------------------------------------------------------
        // matf32_exp
        // ---------------------------------------------------------------------------

        matf32_exp(&A, &temp, 4);
        check_test_case("matf32_exp", n, n, matf32_is_equal(&temp, &R_exp), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_exp));

        // ---------------------------------------------------------------------------
        // matf32_pinv
        // ---------------------------------------------------------------------------

        // ------------------------- BASIC PINV --------------------------------------

        matf32_pinv(&A, &temp, BASIC_PINV);
        check_test_case("matf32_pinv,basic", n, n, matf32_is_equal(&temp, &R_pinv_basic), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_pinv_basic));

        // ------------------------- SVD PINV ----------------------------------------

        matf32_pinv(&A, &temp, SVD_PINV);
        check_test_case("matf32_pinv,svd", n, n, matf32_is_equal(&temp, &R_pinv_svd), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_pinv_svd));

        // ---------------------------------------------------------------------------
        // matf32_lu
        // ---------------------------------------------------------------------------

        matf32_lu(&A, &temp_L, &temp_U, p_index);
        matf32_mul(&temp_L, &temp_U, &PA);
        check_test_case("matf32_lu", n, n, matf32_is_equal(&PA, &R_lu), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&PA, &R_lu));
        
        // ---------------------------------------------------------------------------
        // matf32_cholesky
        // ---------------------------------------------------------------------------

        matf32_zeros(&temp);
        matf32_cholesky(&C, &temp);
        check_test_case("matf32_cholesky", n, n, matf32_is_equal(&temp, &R_cholesky), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&temp, &R_cholesky));

        // ---------------------------------------------------------------------------
        // matf32_qr
        // ---------------------------------------------------------------------------

        matf32_qr(&A, &temp_Q, &temp_R);
        matf32_mul(&temp_Q, &temp_R, &A_QR);
        check_test_case("matf32_qr", n, n, matf32_is_equal(&A_QR, &R_qr), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&A_QR, &R_qr));

        // ---------------------------------------------------------------------------
        // matf32_jacobi_svd
        // ---------------------------------------------------------------------------

        if (n > 1)
        {
            matf32_jacobi_svd(&A, &U, &S, &V);
            matf32_trans(&V, &Vt); // To-Do: Add functions in matf32 to reconstruct the factorizations, it will be easier
            matf32_mul(&U, &S, &US);
            matf32_mul(&US, &Vt, &USVt);
            check_test_case("matf32_jacobi_svd", n, n, matf32_is_equal(&USVt, &R_jacobi_svd), &total_tests_counter, &fails_counter, cond_A, matf32_max_numerical_error(&USVt, &R_jacobi_svd));
        }
    }

    printf("\nmatf32: %u/%u passed\n", total_tests_counter - fails_counter, total_tests_counter);

    return fails_counter > 0 ? 1 : 0; // If there was at least one fail, return 1
}

void check_test_case(const char *routine, int m, int n, bool result, uint16_t *total_tests_counter, uint16_t *fails_counter, float cond_A, float max_error)
{
    printf("%s, %i x %i: %s, conditioning A: %.4f, max. error: %.9f\n", routine, m, n, result?"PASS":"FAIL", cond_A, max_error);

    *total_tests_counter += 1;

    if (!result)
    {
        *fails_counter += 1;
    }
}

float matf32_max_numerical_error(const matf32_t* const A, const matf32_t* const B) // A and B must have the same dimensions and shape
{
    float max_error = 0;
    float numerical_error = 0;

    uint16_t elements = A->num_rows*A->num_cols;

    if (A->num_rows != B->num_rows || A->num_cols != B->num_cols)
    {
        return MATH_SIZE_MISMATCH;
    }

    for (uint16_t k = 0; k < elements; k++)
    {
        numerical_error = fabsf(A->p_data[k] - B->p_data[k]);

        if (isnan(numerical_error))
        {
            return numerical_error;
        } 

        if (numerical_error > max_error)
        {
            max_error = numerical_error;
        }
    }

    return max_error;
}