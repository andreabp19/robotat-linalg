#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "robotat_linalg.h"
#include "robotat_control.h"

#define SIGMA_W (0.01f)
#define SIGMA_V (0.5f)
#define SIGMA_E (0.1f)      

float Adata[] = { 0,  1,
                   0,  0 };

float Bdata[] = { 0,
                   1 };

float Cdata[] = { 1,  0 };

float Ddata[1] = { 0 };

float Fdata[] = { 0,
                   1 };

float xhatdata[2] = { 0.5,  0.5 };

float Qwdata[1] = { SIGMA_W * SIGMA_W };
float Qvdata[1] = { SIGMA_V * SIGMA_V };
float Pdata[] = { SIGMA_E * SIGMA_E,  0,
                                   0,  SIGMA_E * SIGMA_E };
float inputs_data[1] = { 5 };
float measurements_data[1] = { 5 };

matf32_t A, B, C, D, F, Qw, Qv, P, xhat, inputs, measurements;
sys_lti_t sys;
kalman_info_t kf;
err_status_t error;

float estimate[2];

float Mdata[] = { 1, 4,
                  2, 5,
                  3, 6 };
float Ndata[] = { -1,  2,
                   0, -1 };
float Odata[] = { -5,
                   2 };
float Sdata[3];
float Tdata[4] = { 3,  4,
                   5,  6 };
float Udata[4] = { -1,  -2,
                    1, -5 };
matf32_t M, N, O, S, T, U;


err_status_t
simple_pendulum(matf32_t* const state_dot, const matf32_t* state, const matf32_t* input)
{
    const float g = 9.81;
    const float m = 0.1;
    const float ell = 0.5;
    float x1, x2, u;
    err_status_t error;

    if (matf32_get(state, 1, 1, &x1) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;
    if (matf32_get(state, 1, 2, &x2) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;
    if (matf32_get(input, 1, 1, &u) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;

    if (matf32_set(state_dot, 1, 1, x2) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;
    if (matf32_set(state_dot, 1, 2, -(g / ell) * sinf(x1) + (1 / (m * ell * ell)) * u) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;

    return MATH_SUCCESS;
}


err_status_t
potentiometer(matf32_t* const output, const matf32_t* state, const matf32_t* input)
{
    float x1, x2, u;
    err_status_t error;

    if (matf32_get(state, 1, 1, &x1) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;
    if (matf32_get(state, 1, 2, &x2) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;
    if (matf32_get(input, 1, 1, &u) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;

    if (matf32_set(output, 1, 1, x1) != MATH_SUCCESS)
        return MATH_LENGTH_ERROR;

    return MATH_SUCCESS;
}


void
main()
{
    matf32_init(&A, 2, 2, Adata);
    matf32_init(&B, 2, 1, Bdata);
    matf32_init(&C, 1, 2, Cdata);
    matf32_init(&D, 1, 1, Ddata);
    matf32_init(&F, 2, 1, Fdata);
    matf32_init(&Qw, 1, 1, Qwdata);
    matf32_init(&Qv, 1, 1, Qvdata);
    matf32_init(&xhat, 2, 1, xhatdata);
    matf32_init(&P, 2, 2, Pdata);
    matf32_init(&inputs, 1, 1, inputs_data);
    matf32_init(&measurements, 1, 1, measurements_data);

    ss(&A, &B, &C, &D, 0, &sys);
    if (sys.is_continuous)
        printf("Continuous time LTI system\n");
    printf("A:\n");
    matf32_print(sys.A);
    printf("B:\n");
    matf32_print(sys.B);
    printf("C:\n");
    matf32_print(sys.C);
    printf("D:\n");
    matf32_print(sys.D);

    c2d(&sys, 0.1, FWD_EULER);
    if (!sys.is_continuous)
        printf("Discrete time LTI system\n");
    printf("A:\n");
    matf32_print(sys.A);
    printf("B:\n");
    matf32_print(sys.B);
    printf("C:\n");
    matf32_print(sys.C);
    printf("D:\n");
    matf32_print(sys.D);

    kalman_init(&kf, &sys, &F, &Qw, &Qv, &xhat, &P);
    kalman_predict(&kf, &inputs);
    printf("x[k|k-1]:\n");
    kalman_get_estimate(&kf, estimate);
    print(estimate, 2, 1);
    printf("P[k|k-1]:\n");
    matf32_print(kf.P);

    error = kalman_correct(&kf, &measurements);
    if (error == MATH_SINGULAR)
        printf("Innovation covariance is singular.\n");
    else
    {
        printf("x[k|k]:\n");
        kalman_get_estimate(&kf, estimate);
        print(estimate, 2, 1);
        printf("P[k|k]:\n");
        matf32_print(kf.P);
    }

    matf32_init(&M, 3, 2, Mdata);
    matf32_init(&N, 2, 2, Ndata);
    matf32_init(&O, 2, 1, Odata);
    matf32_init(&S, 3, 1, Sdata);
    matf32_init(&T, 2, 2, Tdata);
    matf32_init(&U, 2, 2, Udata);
    matf32_t* const mats[3] = { &M, &N, &O };
    matf32_t* const mats2[3] = { &U, &T, &N };

    matf32_print(mats[2]);

    error = matf32_arr_mul(mats, 3, &S);
    //error = matf32_arr_mul((matf32_t*[]) {&M, &N, &O}, 3, & S); // Using C99 compound literals
    if (error != MATH_SUCCESS)
        printf("Math error #: %d\n", (int)error);
    else
        matf32_print(&S);

    error = matf32_arr_sub(mats2, 3, &P);
    if (error != MATH_SUCCESS)
        printf("Math error #: %d\n", (int)error);
    else
        matf32_print(&P);

    matf32_print(&M);

    uint16_t i = 3;
    uint16_t j = 2;
    float val;

    matf32_set(&M, i, j, 5.6321);
    matf32_get(&M, i, j, &val);
    matf32_print(&M);
    printf("M(%d, %d) = %f\n", i, j, val);
}

