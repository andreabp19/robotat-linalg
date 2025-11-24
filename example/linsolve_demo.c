
/**
 * @author Andrea Pineda
 * @date Created 5 Oct 2025
 * Last Modified 13 Nov 2025
 *          By: Andrea Pineda                         
 * 
 * Example for learning to use the linsolve library of Robotat Linalg
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
    /*
        The linsolve library implements different methods for solving
        linear systems of the form Ax = b, where A is a matrix (square or rectangular),
        while x and b are vector. x is the solution of the linear system.

        The linear solver is used with the function: linsolve
        and automatically selects method based on the input matrix:
            Lower triangular              -> Forward Substitution
            Upper triangular              -> Backward Substitution
            Symmetric Positive Definite   -> Cholesky
            Rectangular                   -> QR 
            Ill-conditioned matrices      -> SVD (configurable threshold for ill-conditioning)
            Any other case                -> LU 

        That way, it's intended that the user of the library should only need to call "linsolve".
        If a specific method is to be used because it's better for a given application (or because
        others are not recommended), use linsolve_method instead to choose a method.
        
        In the case of SVD, this method is used when the condition number of a matrix is above
        a certain value, which is not fixed, instead it can be configured by modifying the value
        of the constant ILL_CONDITIONING_THRESHOLD, found in constants.h

        In this file, an example is given for the use of the solver for each method.
    */

    linsolve_method_t method;
    bool linsolve_ans = false;

    matf32_init(A, 3, 3, m1data);
    matf32_init(x, 3, 1, m2data);
    matf32_init(b, 3, 1, m3data);
    matf32_init(Ax, 3, 1, m4data);
    matf32_init(At, 3, 3, m5data);
    matf32_init(AtA, 3, 3, m6data);

    matf32_randn(b, 0, 1);

    // ------------------------- 1. Forward Substitution -------------------------

    printf("\n--------------------------------------------------\n");
    printf("FORWARD SUBSTITUTION\n");
    printf("--------------------------------------------------\n");

    // Set arbitrary values of A to be an upper triangular matrix
    matf32_zeros(A);
    matf32_set(A, 1, 1, 1);
    matf32_set(A, 2, 1, 2);
    matf32_set(A, 2, 2, 3);
    matf32_set(A, 3, 1, 4);
    matf32_set(A, 3, 2, 5);
    matf32_set(A, 3, 3, 6);
    
    /*
        linsolve_get_method can be used to check which method will be used for a given matrix,
        and is also used inside linsolve to determine the method before solving.
    */
    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");



    // ------------------------- 2. Backward Substitution -------------------------

    printf("\n--------------------------------------------------\n");
    printf("BACKWARD SUBSTITUTION\n");
    printf("--------------------------------------------------\n");

    // Set arbitrary values of A to be an upper triangular matrix
    matf32_zeros(A);
    matf32_set(A, 1, 1, 1);
    matf32_set(A, 1, 2, 2);
    matf32_set(A, 1, 3, 3);
    matf32_set(A, 2, 2, 4);
    matf32_set(A, 2, 3, 5);
    matf32_set(A, 3, 3, 6);

    /*
        linsolve_get_method can be used to check which method will be used for a given matrix,
        and is also used inside linsolve to determine the method before solving.
    */
    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");



    // ------------------------- 3. Cholesky -------------------------

    printf("\n--------------------------------------------------\n");
    printf("CHOLESKY\n");
    printf("--------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_zeros(A);
    matf32_randn(A, 0, 1);
    matf32_trans(A, At); // At = A'
    matf32_mul(At, A, AtA); // AtA = A'A

    /*
        linsolve_get_method can be used to check which method will be used for a given matrix,
        and is also used inside linsolve to determine the method before solving.
    */
    method = linsolve_get_method(AtA);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(AtA, b, x);

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(AtA, x, Ax);

    printf("A'A:\n");
    matf32_print(AtA);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");



    // ------------------------- 4. QR -------------------------

    printf("\n--------------------------------------------------\n");
    printf("QR with square matrix\n");
    printf("--------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 3, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    /*
        linsolve uses QR to solve Ax = b when A is rectangular (not square). This was decided
        following MATLAB's own linear solver (also called linsolve) so it matches the structure.

        Nonetheless, QR can also be used for square matrices is used separately from the linear solver.
        Here, two examples are provided: one for square matrices with linsolve and one with rectangular matrices
        with linsolve_method.
    */
    method = linsolve_get_method(A);

    // Solve the system
    linsolve_method(A, b, x, QR, SQUARE);

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");

    printf("\n--------------------------------------------------\n");
    printf("QR with rectangular matrix\n");
    printf("--------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 2, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    /*
        linsolve uses QR to solve Ax = b when A is rectangular (not square). This was decided
        following MATLAB's own linear solver (also called linsolve) so it matches the structure.

        Nonetheless, QR can also be used for square matrices is used separately from the linear solver.
        Here, two examples are provided: one for square matrices with linsolve and one with rectangular matrices
        with linsolve_method.
        
        TODO: Modify linsolve_method to accept matrix shape as input.
    */
    method = linsolve_get_method(A);

    // Print the method
    printf("\nMethod: ");
    linsolve_print_method(method);
    printf("\n");

    // Solve the system
    linsolve(A, b, x);

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");

    // ------------------------- 5. SVD -------------------------

    printf("\n--------------------------------------------------\n");
    printf("SVD\n");
    printf("--------------------------------------------------\n");

    // Create a symmetric positive definite matrix AtA = A'A so Cholesky is applicable
    matf32_init(A, 3, 3, m1data);
    matf32_zeros(A);
    matf32_randn(A, 0, 1);

    float condA = 0;
    matf32_cond(A, &condA);
    printf("Condition number of A: %.9f\n\n", condA);

    /*
        SVD is used when the condition number of a matrix surpasses the specified threshold.
        There is no definite guide for what condition number constitutes an ill-conditioned matrix,
        which (theoretically) is supposed to be around the limit of floating-point precision, but in
        practice seems to be much lower. As a general rule, as it increases, some operations
        tend to have more numerical imprecision in one or more decimals (despite the operaiton being
        done correctly). In case of doubt, compare Ax = b and decide if it has enough precision
        for the intended application.
    */

    // Solve the system
    linsolve_method(A, b, x, SVD, SQUARE); // Change the shape enum for 0 and 1 so I can use NULL when not calling QR

    // As the system is Ax = b, you can check the answer by multiplying A*x
    matf32_mul(A, x, Ax);

    printf("A:\n");
    matf32_print(A);
    printf("b:\n");
    matf32_print(b);
    printf("x:\n");
    matf32_print(x);
    printf("Ax:\n");
    matf32_print(Ax);

    linsolve_ans = matf32_is_equal(Ax, b);
    printf("1E-05 Tolerance: %s\n\n", linsolve_ans?"true":"false");

}