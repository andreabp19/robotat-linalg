

/**
 * @author Andrea Pineda
 * @date Created 23 Aug 2025, last modified: 18 Nov. 2026
 * 
 * For testing linsolve in computer
 */

#include <stdio.h>

#include "matf32.h"
#include "linsolve.h"
#include "linsolve_matlab_data.h"

float temp_data[MAX_MAT_SIZE];
matf32_t temp;

float temp2_data[MAX_MAT_SIZE];
matf32_t temp2;

float* b_list[10] = {b_data1, b_data2, b_data3, b_data4, b_data5, b_data6, b_data7, b_data8, b_data9, b_data10};
matf32_t b;

float* A_list[10] = {A_data1, A_data2, A_data3, A_data4, A_data5, A_data6, A_data7, A_data8, A_data9, A_data10};
matf32_t A;

float* A_L_list[10] = {A_L_data1, A_L_data2, A_L_data3, A_L_data4, A_L_data5, A_L_data6, A_L_data7, A_L_data8, A_L_data9, A_L_data10};
matf32_t A_L;

float* A_U_list[10] = {A_U_data1, A_U_data2, A_U_data3, A_U_data4, A_U_data5, A_U_data6, A_U_data7, A_U_data8, A_U_data9, A_U_data10};
matf32_t A_U;

float* B_list[10] = {B_data1, B_data2, B_data3, B_data4, B_data5, B_data6, B_data7, B_data8, B_data9, B_data10};
matf32_t B;

float* C_list[10] = {C_data1, C_data2, C_data3, C_data4, C_data5, C_data6, C_data7, C_data8, C_data9, C_data10};
matf32_t C;

float* D_list[10] = {D_data1, D_data2, D_data3, D_data4, D_data5, D_data6, D_data7, D_data8, D_data9};
matf32_t D;

float* R_fwd_subs_list[10] = {R_fwd_subs_data1, R_fwd_subs_data2, R_fwd_subs_data3, R_fwd_subs_data4, R_fwd_subs_data5, R_fwd_subs_data6, R_fwd_subs_data7, R_fwd_subs_data8, R_fwd_subs_data9, R_fwd_subs_data10};
matf32_t R_fwd_subs;

float* R_bwd_subs_list[10] = {R_bwd_subs_data1, R_bwd_subs_data2, R_bwd_subs_data3, R_bwd_subs_data4, R_bwd_subs_data5, R_bwd_subs_data6, R_bwd_subs_data7, R_bwd_subs_data8, R_bwd_subs_data9, R_bwd_subs_data10};
matf32_t R_bwd_subs;

float* R_cholesky_list[10] = {R_cholesky_data1, R_cholesky_data2, R_cholesky_data3, R_cholesky_data4, R_cholesky_data5, R_cholesky_data6, R_cholesky_data7, R_cholesky_data8, R_cholesky_data9, R_cholesky_data10};
matf32_t R_cholesky;

float* R_qr_list[10] = {R_qr_data1, R_qr_data2, R_qr_data3, R_qr_data4, R_qr_data5, R_qr_data6, R_qr_data7, R_qr_data8, R_qr_data9};
matf32_t R_qr;

float* R_lu_list[10] = {R_lu_data1, R_lu_data2, R_lu_data3, R_lu_data4, R_lu_data5, R_lu_data6, R_lu_data7, R_lu_data8, R_lu_data9, R_lu_data10};
matf32_t R_lu;

float* R_svd_list[10] = {R_svd_data1, R_svd_data2, R_svd_data3, R_svd_data4, R_svd_data5, R_svd_data6, R_svd_data7, R_svd_data8, R_svd_data9, R_svd_data10};
matf32_t R_svd;

// SVD Matrices
float U_data[MAX_MAT_SIZE];
matf32_t U;
float S_data[MAX_MAT_SIZE];
matf32_t S;
float V_data[MAX_MAT_SIZE];
matf32_t V;
float A_copy_data[MAX_MAT_SIZE];
matf32_t A_copy;
float U_trans_data[MAX_MAT_SIZE];
matf32_t U_trans;
float US_data[MAX_MAT_SIZE];
matf32_t US;
float USVt_data[MAX_MAT_SIZE];
matf32_t USVt;
float Ax_data[MAX_MAT_SIZE];
matf32_t Ax;

float cond_A = 0;

float norm_b_Ax = 0;
float norm_b = 0;

uint16_t total_tests_counter = 0;
uint16_t fails_counter = 0;

void check_test_case(const char *routine, int m, int n, const matf32_t* const A, const matf32_t* const b, const matf32_t* const x, uint16_t *total_tests_counter, uint16_t *fails_counter);

float matf32_max_numerical_error(const matf32_t* const A, const matf32_t* const B);

int main(void)
{
    for (uint8_t i = 0; i < 10; i++)
    {
        uint8_t n = i + 1;  // matrix dimension to use

        matf32_init(&temp, n, 1, temp_data);
        matf32_init(&b, n, 1, b_list[i]);
        matf32_init(&A, n, n, A_list[i]);
        matf32_init(&A_L, n, n, A_L_list[i]);
        matf32_init(&A_U, n, n, A_U_list[i]);
        matf32_init(&C, n, n, C_list[i]);
        matf32_init(&B, n, n, B_list[i]);
        matf32_init(&R_fwd_subs, n, 1, R_fwd_subs_list[i]);
        matf32_init(&R_bwd_subs, n, 1, R_bwd_subs_list[i]);
        matf32_init(&R_cholesky, n, 1, R_cholesky_list[i]);
        matf32_init(&R_lu, n, 1, R_lu_list[i]);
        matf32_init(&R_svd, n, 1, R_svd_list[i]);
        matf32_init(&U, A.num_rows, A.num_rows, U_data);
        matf32_init(&S, A.num_rows, A.num_cols, S_data);
        matf32_init(&V, A.num_cols, A.num_cols, V_data);
        matf32_init(&A_copy, A.num_rows, A.num_cols, A_copy_data);
        matf32_init(&U_trans, U.num_cols, U.num_rows, U_trans_data);
        matf32_init(&US, U.num_rows, S.num_cols, US_data);
        matf32_init(&USVt, A.num_rows, A.num_cols, USVt_data);
        
        matf32_submatrix_copy(&A, &A_copy, 0, 0, 0, 0, A.num_rows, A.num_cols);

        matf32_cond(&A, &cond_A);

        if (n > 1)
        {
            matf32_init(&temp2, n-1, 1, temp2_data);
            matf32_init(&D, n, n-1, D_list[i-1]);
            matf32_init(&R_qr, n-1, 1, R_qr_list[i-1]);
        }

        // ---------------------------------------------------------------------------
        // linsolve - Forward Substitution
        // ---------------------------------------------------------------------------

        matf32_zeros(&temp);
        linsolve(&A_L, &b, &temp);
        check_test_case("Forward Substitution", n, n, &A_L, &b, &temp, &total_tests_counter, &fails_counter);

        // ---------------------------------------------------------------------------
        // linsolve - Backward Substitution
        // ---------------------------------------------------------------------------

        matf32_zeros(&temp);
        linsolve(&A_U, &b, &temp);
        check_test_case("Backward Substitution", n, n, &A_U, &b, &temp, &total_tests_counter, &fails_counter);

        // ---------------------------------------------------------------------------
        // linsolve - Cholesky
        // ---------------------------------------------------------------------------
        
        matf32_zeros(&temp);
        linsolve(&C, &b, &temp);
        check_test_case("Cholesky", n, n, &C, &b, &temp, &total_tests_counter, &fails_counter);

        // ---------------------------------------------------------------------------
        // linsolve - QR
        // ---------------------------------------------------------------------------
        
        matf32_zeros(&temp2);
        linsolve(&D, &b, &temp2);
        check_test_case("QR", n, n, &D, &b, &temp2, &total_tests_counter, &fails_counter);

        // ---------------------------------------------------------------------------
        // linsolve - LU
        // ---------------------------------------------------------------------------

        matf32_zeros(&temp);
        linsolve(&B, &b, &temp);
        check_test_case("LU", n, n, &B, &b, &temp, &total_tests_counter, &fails_counter);

        // ---------------------------------------------------------------------------
        // linsolve - SVD Jacobi
        // ---------------------------------------------------------------------------

        matf32_zeros(&temp);
        linsolve(&A, &b, &temp);
        check_test_case("SVD", n, n, &A, &b, &temp, &total_tests_counter, &fails_counter);

        printf("\n\n");
    }

    printf("\nmatf32: %u/%u passed\n", total_tests_counter - fails_counter, total_tests_counter);

    return fails_counter > 0 ? 1 : 0; // If there was at least one fail, return 1
}

void check_test_case(const char *routine, int m, int n, const matf32_t* const A, const matf32_t* const b, const matf32_t* const x, uint16_t *total_tests_counter, uint16_t *fails_counter)
{
    float Ax_data[MAX_MAT_SIZE];
    matf32_t Ax;

    float b_Ax_data[MAX_MAT_SIZE];
    matf32_t b_Ax;

    matf32_init(&Ax, A->num_rows, 1, Ax_data);
    matf32_init(&b_Ax, A->num_rows, 1, b_Ax_data);

    matf32_mul(A, x, &Ax);         // A*x
    matf32_sub(b, &Ax, &b_Ax);     // residual = b - A*x

    float residual;

    if (A->num_rows > A->num_cols)
    {
        // Residual for an overdetermined system
        float At_data[MAX_MAT_SIZE];
        matf32_t At;
        float Atr_data[MAX_MAT_SIZE];
        matf32_t Atr;
        float Atb_data[MAX_MAT_SIZE];
        matf32_t Atb;

        matf32_init(&At,  A->num_cols, A->num_rows, At_data);
        matf32_init(&Atr, A->num_cols, 1, Atr_data);
        matf32_init(&Atb, A->num_cols, 1, Atb_data);

        matf32_trans(A, &At);          // A'
        matf32_mul(&At, &b_Ax, &Atr);  // residual = A' * (b - Ax)
        matf32_mul(&At, b, &Atb);      // overdetermined b = (A' * b)

        residual = matf32_norm(&Atr) / matf32_norm(&Atb);
    }
    else
    {
        // Residual for square matrices
        residual = matf32_norm(&b_Ax) / matf32_norm(b);
    }

    bool result = residual < 1E-05;

    printf("%s, %i x %i: %s, residual-based error: %.9f\n", routine, m, n, result?"PASS":"FAIL", residual);

    *total_tests_counter += 1;

    if (!result)
    {
        *fails_counter += 1;
    }
}