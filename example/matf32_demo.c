
/**
 * @author Andrea Pineda
 * @date Created: 23 Sep 2025
 * Last modified: 30 Dec 2025 by Andrea Pineda
 * 
 * matf32 demo
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matf32.h"

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
static float m7data[MAX_MAT_SIZE];
static matf32_t m7;
static float m8data[MAX_MAT_SIZE];
static matf32_t m8;
static float m9data[MAX_MAT_SIZE];
static matf32_t m9;
static float m10data[MAX_MAT_SIZE];
static matf32_t m10;

matf32_t* A = &m1;
matf32_t* b = &m2;
matf32_t* R = &m3;

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    // ---------------------------------------------------------------------------
    // Basic Operations: Sum, Substraction, Transpose, Multiplications, Inverse
    // ---------------------------------------------------------------------------

    // --------------- Matrix Initialization ---------------
    matf32_init(A, 3, 3, m1data); // 3x3 matrix
    matf32_init(b, 3, 1, m2data); // 3x1 matrix (vector defined as a matrix)
    matf32_init(R, 3, 3, m3data); // 3x3 matrix

    // --------------- Assigning values ---------------

    // A = matrix with random values
    matf32_randn(A,0,1);
    matf32_randn(b,0,1);

    // b = [1, 2, 3]
    //matf32_set(b,1,1,1);
    //matf32_set(b,2,1,2);
    //matf32_set(b,3,1,3);

    printf("\n---------------------------------------------------------------------------\n");
    printf("Initial Values");
    printf("\n---------------------------------------------------------------------------\n");
    printf("A:\n");  // Print the name of the matrix for easier identification
    matf32_print(A); // Print the results, with text already formatted as a matrix.
    printf("b:\n");
    matf32_print(b);

    float condA = 0;
    float condb = 0;
    matf32_cond(A, &condA);
    matf32_cond(b, &condb);

    printf("Condition number A: %.9f\n", condA);
    printf("Condition number b: %.9f\n\n", condb);

    // Try the check function for square matrices such as A
    bool square_matrix = matf32_check_square_matrix(b);
    printf("Is b a square matrix? %s\n", square_matrix?"True":"False");
    square_matrix = matf32_check_square_matrix(A);
    printf("Is A a square matrix? %s\n", square_matrix?"True":"False");

    // --------------- 1. Addition ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_add");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_add(A, A, R); // R = A + A
    printf("R = A + A:\n");
    matf32_print(R);

    // --------------- 2. Substraction ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_sub");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_sub(R, A, R); // R = R - A
    printf("R = R - A:\n");
    matf32_print(R);

    // --------------- 3. Transpose --------------- 
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_trans");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_trans(A, R); // R = A'
    printf("R = A':\n");
    matf32_print(R);

    // --------------- 4. Matrix-Matrix Multiplication = Matrix ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_mul");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_mul(A, R, R); // R = A*R (make sure dimensions match, otherwise it leads to errors)
    printf("R = A*R:\n");
    matf32_print(R);

    // --------------- 5. Matrix-Vector Multiplication = Vector ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_vecposmul");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_t* r = &m4;
    matf32_init(r, 3, 1, m4data); // 3x1 matrix

    matf32_vecposmul(A, b->p_data, r->p_data); // Vectors' data arrays are assigned directly, instead of the matrix.
    printf("r = A*b:\n");
    matf32_print(r);

    // --------------- 6. Vector-Matrix Multiplication = Vector ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_vecpremul");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_vecpremul(A, b->p_data, r->p_data);
    printf("r = b*A:\n");
    matf32_print(r);

    // --------------- 7. Vector-Vector Multiplication = Matrix ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_vecmul_col_row");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_vecmul_col_row(b->p_data, b->p_data, R);
    printf("R = b*b:\n");
    matf32_print(R);

    // --------------- 8. Dot Product between two vectors --------------- 
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_dot");
    printf("\n---------------------------------------------------------------------------\n");

    float dot = 0;
    matf32_dot(b, b, &dot);
    printf("Dot product b*b: %.9f\n", dot);

    // --------------- 9. Adding Arrays of Matrices --------------- 
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_arr_add");
    printf("\n---------------------------------------------------------------------------\n");

    const matf32_t* mat_array[3] = {A, A, A}; // Define matrix array as a pointer

    matf32_arr_add(mat_array, 3, R); // Add matrices in the arras (which are 3), and save the result in R
    printf("R = A + A + A\n");
    matf32_print(R);

    // --------------- 10. Substracting Arrays of Matrices ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_arr_sub");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_arr_sub(mat_array, 3, R);
    printf("R = A - A - A:\n");
    matf32_print(R);

    // --------------- 11. Multiplying Arrays of Matrices ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_arr_mul");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_arr_mul(mat_array, 3, R);
    printf("R = A * A * A:\n");
    matf32_print(R);

    // --------------- 12. Inverse of a Matrix ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_inv");
    printf("\n---------------------------------------------------------------------------\n");
    
    matf32_inv(A, R);
    printf("R = inv(A):\n");
    matf32_print(R);

    // --------------- 13. Pseudoinverse of a Matrix ---------------

    // TODO: matf32_pinv example (for both methods)

    // ---------------------------------------------------------------------------
    // Matrix Factorizations: QR, Cholesky, LU, SVD
    // ---------------------------------------------------------------------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_qr");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_t* Q = &m4;
    matf32_init(Q, A->num_rows, A->num_cols, m4data);
    matf32_zeros(Q); // Reset reused matrices to zero to avoid garbage values
    matf32_init(R, A->num_cols, A->num_cols, m3data);
    matf32_zeros(R);

    matf32_qr(A, Q, R);
    printf("A:\n");
    matf32_print(A);
    printf("Q:\n");
    matf32_print(Q);
    printf("R:\n");
    matf32_print(R);

    matf32_t* reconstructed_A = &m10;
    matf32_init(reconstructed_A, A->num_rows, A->num_cols, m10data);
    matf32_mul(Q, R, reconstructed_A);
    printf("Reconstructed A = Q*R:\n");
    matf32_print(reconstructed_A);

    bool tolerance_check = matf32_is_equal(A, reconstructed_A);
    printf("Difference between A and A=Q*R less than 1E-05? %s\n\n", tolerance_check?"True":"False");

    // try the check function for upper triangular matrices: A is square, R is upper triangular
    bool triu_matrix = matf32_check_triangular_upper(A);
    printf("Is A an upper triangular matrix? %s\n", triu_matrix?"True":"False");
    triu_matrix = matf32_check_triangular_upper(R);
    printf("Is R an upper triangular matrix? %s\n", triu_matrix?"True":"False");

    // --------------- 2. Cholesky Factorization ---------------

    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_cholesky");
    printf("\n---------------------------------------------------------------------------\n");

    matf32_t* C = &m3;
    matf32_init(C, A->num_rows, A->num_cols, m3data);
    matf32_zeros(C);

    // Symmetric positive definite matrices are defined as: A = A'A or AA'
    float At_data[MAX_MAT_SIZE]; // For A'
    matf32_t At;
    matf32_init(&At, A->num_cols, A->num_rows, At_data);
    matf32_trans(A, &At);

    matf32_t* AtA = &m4;
    matf32_init(AtA, A->num_rows, At.num_cols, m4data);
    matf32_zeros(AtA);
    
    printf("A:\n");
    matf32_print(A);
    
    printf("At:\n");
    matf32_print(&At);
    matf32_mul(&At, A, AtA);
    printf("AtA (symmetric positive definite, needed for Cholesky):\n");
    matf32_print(AtA);

    matf32_cholesky(AtA, C);
    printf("C:\n");
    matf32_print(C);

    matf32_t* tmpmat = &m9;
    matf32_init(tmpmat, A->num_rows, A->num_cols, m9data);
    matf32_zeros(reconstructed_A);
    matf32_trans(C, tmpmat);
    printf("C':\n");
    matf32_print(tmpmat);
    matf32_mul(tmpmat, C, reconstructed_A);

    printf("Reconstructed AtA = C*C':\n");
    matf32_print(reconstructed_A);

    tolerance_check = matf32_is_equal(AtA, reconstructed_A);
    printf("Difference between AtA and AtA=CC' less than 1E-05? %s\n\n", tolerance_check?"True":"False");

    // Try the check function for symmetric positive definite matrices such as A'A
    bool symposdef_matrix = matf32_check_symposdef(A);
    printf("Is A a Symmetric Positive Definite Matrix? %s\n", symposdef_matrix?"True":"False");
    symposdef_matrix = matf32_check_symposdef(AtA);
    printf("Is A'A a Symmetric Positive Definite Matrix? %s\n", symposdef_matrix?"True":"False");

    // --------------- 3. LU Factorization ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_lu");
    printf("\n---------------------------------------------------------------------------\n");

    uint16_t permutations[3];
    matf32_t* L = &m3;
    matf32_init(L, A->num_rows, A->num_cols, m3data);
    matf32_zeros(L);
    matf32_t* U = &m4;
    matf32_init(U, A->num_rows, A->num_cols, m4data);
    matf32_zeros(U);

    printf("A:\n");
    matf32_print(A);
    matf32_lu(A, L, U, permutations);
    printf("L:\n");
    matf32_print(L);
    printf("U:\n");
    matf32_print(U);

    for (uint16_t i = 0; i < A->num_rows; ++i)
    {
        printf("Permutted rows: %i <--> %i\n", i, permutations[i]);
    }
    printf("\n");

    matf32_t* temp_row = &m7;
    matf32_init(temp_row, 1, A->num_cols, m7data);
    for (uint16_t j = 0; j < A->num_rows; ++j)
    {
        matf32_submatrix_copy(A, temp_row, j, 0, 0, 0, 1, A->num_cols); // Save row to be permutted
        matf32_submatrix_copy(A, A, permutations[j], 0, j, 0, 1, A->num_cols); // Move row in permutations[j] to position j in A
        matf32_submatrix_copy(temp_row, A, 0, 0, permutations[j], 0, 1, A->num_cols); // Move saved row (j) to index permutation[j] in A
    }

    printf("Permutted matrix A = PA:\n");
    matf32_print(A);

    matf32_zeros(tmpmat);
    matf32_mul(L, U, tmpmat);
    printf("L*U:\n");
    matf32_print(tmpmat);

    tolerance_check = matf32_is_equal(A, tmpmat);
    printf("Difference between P*A and L*U less than 1E-05? %s\n\n", tolerance_check?"True":"False");

    // Try the check functions to determine the shape of a matrix: L is lower triangular and A is square.
    bool tril_matrix = matf32_check_triangular_lower(A);
    printf("Is A a lower triangular matrix? %s\n", tril_matrix?"True":"False");
    tril_matrix = matf32_check_triangular_lower(L);
    printf("Is L a lower triangular matrix? %s\n", tril_matrix?"True":"False");

    // --------------- 4. SVD Factorization ---------------
    printf("\n---------------------------------------------------------------------------\n");
    printf("matf32_jacobi_svd");
    printf("\n---------------------------------------------------------------------------\n");

    U = &m3;
    matf32_init(U, A->num_rows, A->num_rows, m3data);
    matf32_zeros(U);
    matf32_t* S = &m4;
    matf32_init(S, A->num_rows, A->num_cols, m4data);
    matf32_zeros(S);
    matf32_t* V = &m5;
    matf32_init(V, A->num_cols, A->num_cols, m5data);
    matf32_zeros(V);

    // Save a copy of A to compare it with the SVD reconstruction A = USV'
    matf32_t* A_copy = &m9;
    matf32_init(A_copy, A->num_rows, A->num_cols, m9data);
    matf32_copy(A, A_copy);

    // Calculate SVD of A
    matf32_jacobi_svd(A_copy, U, S, V);
    printf("A:\n");
    matf32_print(A);
    printf("U:\n");
    matf32_print(U);
    printf("S:\n");
    matf32_print(S);
    printf("V:\n");
    matf32_print(V);

    matf32_t* Vt = &m6;
    matf32_init(Vt, V->num_cols, V->num_rows, m6data);
    matf32_t* US = &m7;
    matf32_init(US, U->num_rows, S->num_cols, m7data);
    matf32_t* USVt = &m8;
    matf32_init(USVt, U->num_rows, Vt->num_cols, m8data);

    matf32_trans(V, Vt);
    matf32_mul(U, S, US);
    matf32_mul(US, Vt, USVt);
    printf("A = USV':\n");
    matf32_print(USVt);

    tolerance_check = matf32_is_equal(A, USVt);
    printf("Difference between A and A=U*S*V' less than 1E-05? %s\n\n", tolerance_check?"True":"False");


}
