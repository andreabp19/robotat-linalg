
/**
 * @author Andrea Pineda
 * @date Created 5 Oct. 2025
 * Last Modified 16 Jan. 2026
 *          By: Andrea Pineda                         
 * 
 * Linsolve demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include "matf32.h"
#include "linsolve.h"

// ---------------------------------------------------------------------------
// Matrices and variables definitions
// ---------------------------------------------------------------------------

static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float m3data[MAX_MAT_SIZE];
static matf32_t m3;
static float m4data[MAX_MAT_SIZE];
static matf32_t m4;
static float m5data[MAX_MAT_SIZE];
static matf32_t m5;
static float m6data[MAX_MAT_SIZE];
static matf32_t m6;

matf32_t* A = &m1;
matf32_t* x = &m2;
matf32_t* b = &m3;
matf32_t* Ax = &m4;
matf32_t* At = &m5;
matf32_t* AtA = &m6;

int main(void)
{
    linsolve_method_t method;
    bool tolerance_check = false;

    matf32_init(A, 3, 3, m1data);
    matf32_init(x, 3, 1, m2data);
    matf32_init(b, 3, 1, m3data);
    matf32_init(Ax, 3, 1, m4data);
    matf32_init(At, 3, 3, m5data);
    matf32_init(AtA, 3, 3, m6data);

    matf32_randn(b, 0, 1);

    // ------------------------- 1. Forward Substitution -------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("FORWARD SUBSTITUTION");
    printf("\n---------------------------------------------------------------------------\n");

    // Set arbitrary values of A to be an upper triangular matrix
    matf32_zeros(A);
    matf32_set(A, 1, 1, 1);
    matf32_set(A, 2, 1, 2);
    matf32_set(A, 2, 2, 3);
    matf32_set(A, 3, 1, 4);
    matf32_set(A, 3, 2, 5);
    matf32_set(A, 3, 3, 6);
    
    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // Check the answer Ax = b
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

    // ------------------------- 2. Backward Substitution -------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("BACKWARD SUBSTITUTION");
    printf("\n---------------------------------------------------------------------------\n");

    // Set arbitrary values of A to be an upper triangular matrix
    matf32_zeros(A);
    matf32_set(A, 1, 1, 1);
    matf32_set(A, 1, 2, 2);
    matf32_set(A, 1, 3, 3);
    matf32_set(A, 2, 2, 4);
    matf32_set(A, 2, 3, 5);
    matf32_set(A, 3, 3, 6);

    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // Check the answer Ax = b
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

    // ------------------------- 3. Cholesky -------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("CHOLESKY");
    printf("\n---------------------------------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_zeros(A);
    matf32_randn(A, 0, 1);
    matf32_trans(A, At); // At = A'
    matf32_mul(At, A, AtA); // AtA = A'A

    method = linsolve_get_method(AtA);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(AtA, b, x);

    // Check the answer Ax = b
    matf32_mul(AtA, x, Ax);

    printf("A'A:\n");
    matf32_print(AtA);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

    // ------------------------- 4. QR -------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("QR with square matrix");
    printf("\n---------------------------------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 3, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    method = linsolve_get_method(A);

    // Solve the system
    linsolve_method(A, b, x, QR_SQUARE);

    // Check the answer Ax = b
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

    printf("\n---------------------------------------------------------------------------\n");
    printf("QR with rectangular matrix");
    printf("\n---------------------------------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 2, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // Check the answer Ax = b
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

    // ------------------------- 5. SVD -------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("SVD");
    printf("\n---------------------------------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 3, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    float condA = 0;
    matf32_cond(A, &condA);
    printf("Condition number of A: %.9f\n\n", condA);

    // Solve the system
    linsolve_method(A, b, x, SVD);

    // Check the answer Ax = b
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    tolerance_check = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", tolerance_check?"true":"false");

}