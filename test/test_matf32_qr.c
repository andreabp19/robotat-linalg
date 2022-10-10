
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "robotat_linalg.h"

float A_data[] = {1, 2, 3,
                   4, 5, 6,
                   7, 8, 9};

float Q_data[] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};

float R_data[] = {0, 0, 0,
                  0, 0, 0,
                  0, 0, 0};


float Result_data[] = {0, 0,
                       0, 0,
                       0, 0};

float b_vect[3] = {1, 2, 3};
float x_vect[3] = {0.0f, -0.0f, 0.0f};
const float r_vect[2] = {0.0f, 0.5f};

err_status_t
matf32_qr_solve(matf32_t* const p_q, const matf32_t* const p_r, const float* const p_b, float* const p_x);

int
main(void)
{
    matf32_t A, Q, R, Result_Q, Result_R, Result;

    matf32_init(&A, 2, 3, A_data);
    matf32_init(&Q, 2, 2, Q_data);
    matf32_init(&R, 2, 3, R_data);

    matf32_init(&Result, 3, 3, Result_data);

    printf("Matrix A: \n");
    matf32_print(&A);

    printf("Testing QR: \n");
    matf32_qr(&A, &Q, &R);

    printf("Matrix Q: \n");
    matf32_print(&Q);

    printf("Matrix R: \n");
    matf32_print(&R);


    // fix matf32_mul
    //err_status_print(matf32_mul(&Q, &R, &Result));
    err_status_print(matf32_qr_solve(&Q, &R, b_vect, x_vect));

    printf("b: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", b_vect[i]);
    }


    printf("\nx: \n");
    for (int i = 0; i < 3; ++i)
    {
        printf("%f\n", x_vect[i]);
    }

    printf("\nr: \n");
    for (int i = 0; i < 2; ++i)
    {
        printf("%f\n", r_vect[i]);
    }

    //bool ans = matf32_is_equal(&A, &Result);

    //if (ans)
    //{
    //    printf("matf32_qr sucess.\n");
    //    return 0;
    //}
    //else
    //{
    //    printf("matf32_qr failure.\n");
    //    return 1;
    //}
}

// incomplete for A(m,n) with n>m
err_status_t
matf32_qr_solve(matf32_t* const p_q, const matf32_t* const p_r, const float* const p_b, float* const p_x)
{
    matf32_t temp_R;
    float temp[20];
    matf32_init(&temp_R, 2, 2, p_r->p_data);
    //matf32_eye(&temp_R);

    //matf32_submatrix_copy(p_r, &temp_R, 0, 0, 0, 0, p_r->num_rows, p_r->num_cols);

    matf32_print(&temp_R);

    matf32_trans(p_q, p_q);

    matf32_vecposmul(p_q, p_b, p_x);
    
    // undo transpose
    matf32_trans(p_q, p_q);

    return matf32_backward_substitution(&temp_R, p_x, p_x);
}

