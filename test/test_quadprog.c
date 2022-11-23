
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "robotat_linalg.h"

float Q_data[] = { 1, -1,
                  -1,  2};

float c_data[] = {-2, -6};

float Aeq_data[] = {1, 1};

float beq_data[] = {0};

float x_data[] = {0,0};

float Result_data[] = {-0.8, 0.8};


int main(void)
{
    clock_t time;
    float time_data = 0;

    matf32_t Q, c, Aeq, beq,  x, Result;

    matf32_init(&Q, 2, 2, Q_data);
    matf32_init(&c, 2, 1, c_data);
    matf32_init(&Aeq, 1, 2, Aeq_data);
    matf32_init(&beq, 1, 1, beq_data);
    matf32_init(&x, 2, 1, x_data);
    matf32_init(&Result, 2, 1, Result_data);

    quadprog_t problem;

    quadprog_init(&problem, &Q, &c, &Aeq, &beq, NULL, NULL, NULL);


    printf("Q:\n");
    matf32_print(&Q);

    printf("c:\n");
    matf32_print(&c);

    printf("Aeq:\n");
    matf32_print(&Aeq);

    printf("beq:\n");
    matf32_print(&beq);

    printf("Testing quadprog: \n");
    for (int i = 0; i < 100; ++i)
    {
        time = clock();
        quadprog(&problem, &x);
        time_data += ((float)clock()-time)/CLOCKS_PER_SEC;
    }

    printf("x:\n");
    matf32_print(&x);

    printf("Time taken: %.9f seconds.\n", time_data/100);

    bool ans = matf32_is_equal(&x, &Result);

    if (ans)
    {
        printf("quadsolve sucess.\n");
        return 0;
    }
    else
    {
        printf("quadsolve failure.\n");
        return 1;
    }
}