
/**
 * @author Andrea Pineda
 * @date Created: 23 Sep 2025
 * Last modified: 03 Oct 2025 by Andrea Pineda
 * 
 * Example for learning to use the matf32 library of Robotat Linalg
 */

// ---------------------------------------------------------------------------
// Libraries
// ---------------------------------------------------------------------------

// Necessary C libraries for using Robotat Linalg (no external libraries are used)
#include <stdio.h>
#include <stdlib.h>
#include <math.h> // sin, cos, sinf, cosf, sqrt, etc.

// Robotat Linalg's linear algebra library
#include "matf32.h"

// ---------------------------------------------------------------------------
// Matrices and variables definitions
// ---------------------------------------------------------------------------

/*
    To create a matrix, the following is needed:
   
    float array_name[MAX_MAT_SIZE];                                  -> Array of floats to save the matrix's data
    matf32_t matrix_name;                                            -> matf32_t matrix struct instance
    matf32_init(&matrix_name, rows_number, cols_number, array_name); -> Matrix initialization function
   
    MAX_MAT_SIZE is a constant value defined in the file constants.h of Robotat Linalg.
    It's purpose is to limit the size of the matrix (max number of elements)
   
    The functions of matf32 operate through pointers.
    This means the function arguments accept the direction of the matrix (&matrix) instead of the actual matrix (without &)
   
    For example: matf32_init(&matrix,...)
                matf32_add(&matrix1, &matrix2, &matrix3)
   
    Which may become tedious as it's necessary to write "&" each time, so pointers to the matrices can also be created:
    For example: matf32_t matrix1;
                matf32_t* p_matrix1 = &matrix1;
   
                Then: matf32_init(p_matrix1,...)
                      matf32_add(p_matrix1, p_matrix2, p_matrix3)
*/

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

/*
    Another way to create the data arrays would be to explicitly assign the values and then initialize the matrix:
        float m1data[3] = {1,2,3};
        matf32_t m1;
        matf32_init(&m1, 3, 1, m1data);
*/

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
    matf32_randn(A,0,1); // Fill matrix with random values sampled from a normal distribution

    matf32_set(b,1,1,1); // Assigns a value to a specific element: matf32_set(matrix, row, column, value)
    matf32_set(b,2,1,2); // Many matf32 functions start the elements' index in 0, matf32_set is an exception (starts in 1)
    matf32_set(b,3,1,3); // Example, in this line, element b(3,1) = 3

    printf("\n--------------------------------------------------\n");
    printf("Initial Values");
    printf("\n--------------------------------------------------\n");
    printf("A:\n");  // Print the name of the matrix for easier identification
    matf32_print(A); // Print the results, with text already formatted as a matrix.
    printf("b:\n");
    matf32_print(b);

    // Try the check function for square matrices such as A
    bool square_matrix = matf32_check_square_matrix(b);
    printf("Square Matrix Check, with b: %s\n", square_matrix?"True":"False");
    square_matrix = matf32_check_square_matrix(A);
    printf("Square Matrix Check, with A: %s\n", square_matrix?"True":"False");

    // --------------- 1. Addition ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_add");
    printf("\n--------------------------------------------------\n");

    matf32_add(A, A, R); // R = A + A
    printf("R = A + A:\n");
    matf32_print(R);

    // --------------- 2. Substraction ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_sub");
    printf("\n--------------------------------------------------\n");

    matf32_sub(R, A, R); // R = R - A
    printf("R = R - A:\n");
    matf32_print(R);

    // --------------- 3. Transpose --------------- 
    printf("\n--------------------------------------------------\n");
    printf("matf32_trans");
    printf("\n--------------------------------------------------\n");

    matf32_trans(A, R); // R = A'
    printf("R = A':\n");
    matf32_print(R);

    // --------------- 4. Matrix-Matrix Multiplication = Matrix ---------------
    /*
        This operation doesn't allow to save the output into any of the input matrices.
        The reason is because input values are required throughout the operation, and replacing any
        would lead to an incorrect result.

        As such, the result must always be saved in a different variable.

        In contrast, the result of matf32_add, matf32_sub and matf32_scale can be saved into one of
        the input matrices without issue.
    */
    printf("\n--------------------------------------------------\n");
    printf("matf32_mul");
    printf("\n--------------------------------------------------\n");

    matf32_mul(A, R, R); // R = A*R (make sure dimensions match, otherwise it leads to errors)
    printf("R = A*R:\n");
    matf32_print(R);

    // --------------- 5. Matrix-Vector Multiplication = Vector ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_vecposmul");
    printf("\n--------------------------------------------------\n");

    matf32_t* r = &m4;
    matf32_init(r, 3, 1, m4data); // 3x1 matrix

    matf32_vecposmul(A, b->p_data, r->p_data); // Vectors' data arrays are assigned directly, instead of the matrix.
    printf("r = A*b:\n");
    matf32_print(r);

    // --------------- 6. Vector-Matrix Multiplication = Vector ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_vecpremul");
    printf("\n--------------------------------------------------\n");

    matf32_vecpremul(A, b->p_data, r->p_data);
    printf("r = b*A:\n");
    matf32_print(r);

    // --------------- 7. Vector-Vector Multiplication = Matrix ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_vecmul_col_row");
    printf("\n--------------------------------------------------\n");

    matf32_vecmul_col_row(b->p_data, b->p_data, R);
    printf("R = b*b:\n");
    matf32_print(R);

    /*
        In this library, when working with matf32_t vectors, whether it's a column or row vector is irrelevant,
        as the arrays' values are accessed and operated directly along with the other vector or matrix.
      
        For example, b was defined as a 3x1 column vector.
        But it's used in a vector-vector multiplication which by definition is: row-vector x col-vector.
        b.p_data is the array of values, and as the length of the arrays match (3 in both inputs), it's a valid operation.
    */

    // --------------- 8. Dot Product between two vectors --------------- 
    printf("\n--------------------------------------------------\n");
    printf("matf32_dot");
    printf("\n--------------------------------------------------\n");

    float dot = 0;
    matf32_dot(b, b, &dot);
    printf("Dot product b*b: %.9f\n", dot);

    // --------------- 9. Adding Arrays of Matrices --------------- 
    printf("\n--------------------------------------------------\n");
    printf("matf32_arr_add");
    printf("\n--------------------------------------------------\n");

    const matf32_t* mat_array[3] = {A, A, A}; // Define matrix array as a pointer

    matf32_arr_add(mat_array, 3, R); // Add matrices in the arras (which are 3), and save the result in R
    printf("R = A + A + A\n");
    matf32_print(R);

    // --------------- 10. Substracting Arrays of Matrices ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_arr_sub");
    printf("\n--------------------------------------------------\n");

    matf32_arr_sub(mat_array, 3, R);
    printf("R = A - A - A:\n");
    matf32_print(R);

    // --------------- 11. Multiplying Arrays of Matrices ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_arr_mul");
    printf("\n--------------------------------------------------\n");

    matf32_arr_mul(mat_array, 3, R);
    printf("R = A * A * A:\n");
    matf32_print(R);

    // --------------- 12. Inverse of a Matrix ---------------
    printf("\n--------------------------------------------------\n");
    printf("matf32_inv");
    printf("\n--------------------------------------------------\n");
    
    matf32_inv(A, R);
    printf("R = inv(A):\n");
    matf32_print(R);

    // --------------- 13. Pseudoinverse of a Matrix ---------------

    // TODO: matf32_pinv example (for both methods)

    // ---------------------------------------------------------------------------
    // Matrix Factorizations: QR, Cholesky, LU, SVD
    // ---------------------------------------------------------------------------

    /*
        Matrix Factorizations are algorithms that allow to deconstruct a matrix into component matrices.
        This facilitates operations for solving linear systems (Ax = b), quadratic programs (QPs), optimization
        problems, and numerical methods in general.
      
        matf32 has the following factorizations available: QR, Cholesky, LU and SVD.
       
        More of each in the respective example below.
    */

    // --------------- 1. QR Factorization (Specifically, Householder QR Factorization) ---------------
    /* 
        A = QR, where: Q (m x n)
                      R (n x n) is an upper triangular matrix (zeros below the main diagonal)
       
        Add brief explanation***
       
    */
    printf("\n--------------------------------------------------\n");
    printf("matf32_qr");
    printf("\n--------------------------------------------------\n");

    // TODO: Examples for QR for rectangular matrices A

    matf32_t* Q = &m4;
    matf32_init(Q, A->num_rows, A->num_cols, m4data);
    matf32_zeros(Q); // Reset reused matrices to zero to avoid garbage values
    matf32_init(R, A->num_cols, A->num_cols, m3data);
    matf32_zeros(R);

    /*
        A->num_rows and A->num_cols are the number of rows and columns of A,
        but obtained through pointers. Another way to get them would be to create variables to reuse them easily:
       
        float m = A->num_rows;
        float n = A->num_cols;
    */

    matf32_qr(A, Q, R);
    printf("A:\n");
    matf32_print(A);
    printf("Q:\n");
    matf32_print(Q);
    printf("R:\n");
    matf32_print(R);

    // try the check function for upper triangular matrices: A is square, R is upper triangular
    bool triu_matrix = matf32_check_triangular_upper(A);
    printf("Upper Triangular Matrix Check, using A: %s\n", triu_matrix?"True":"False");
    triu_matrix = matf32_check_triangular_upper(R);
    printf("Upper Triangular Matrix Check, using R: %s\n", triu_matrix?"True":"False");

    // --------------- 2. Cholesky Factorization ---------------
    /* 
        A = LL', where: L (m x n) is a lower triangular matrix (zeros above the main diagonal)
       
        Add explanation of the Cholesky algorithm

        This method is only applicable to symmetric positive definite matrices.
    */

    printf("\n--------------------------------------------------\n");
    printf("matf32_cholesky");
    printf("\n--------------------------------------------------\n");

    matf32_t* C = &m3;
    matf32_init(C, A->num_rows, A->num_cols, m3data);
    matf32_zeros(C);

    // Symmetric positive definite matrices are defined as: A = A'A or AA'
    float At_data[MAX_MAT_SIZE]; // For A'
    matf32_t At;
    matf32_init(&At, A->num_cols, A->num_rows, At_data); // Requires pointer &At because of the way it was defined.
    matf32_trans(A, &At);

    matf32_t* AtA = &m4;
    matf32_init(AtA, A->num_rows, At.num_cols, m4data);
    matf32_zeros(AtA);
    
    printf("A:\n");
    matf32_print(A);
    
    printf("At:\n");
    matf32_print(&At);
    matf32_mul(&At, A, AtA); // matf32_mul has a condition to avoid assigning the output in an input matrix
    printf("AtA:\n");
    matf32_print(AtA);

    matf32_cholesky(AtA, C);
    printf("C:\n");
    matf32_print(C);

    // Try the check function for symmetric positive definite matrices such as A'A
    bool symposdef_matrix = matf32_check_symposdef(A);
    printf("Symmetric Positive Definite Matrix Check, with A: %s\n", symposdef_matrix?"True":"False");
    symposdef_matrix = matf32_check_symposdef(AtA);
    printf("Symmetric Positive Definite Matrix Check, with A'A: %s\n", symposdef_matrix?"True":"False");

    // --------------- 3. LU Factorization ---------------
    /* 
        Where L is lower triangular (and with 1s in the diagonal in this specific algorithm),
        and U is upper triangular. Also, an array is created to save the row permutations.
       
        The permutations array stores the indices of the rows to be permutted:
        For example:
       
        permutations[0] = 1      -> Switch rows 0 and 1
        permutations[1] = 4      -> Switch rows 1 and 4
        and so on...
       
        If both indices are the same: permutations[1] = 1; that means there was no switch
       
        Permutations are applied to place the largest numbers from each column as pivots,
        in order to avoid divisions by small numbers that can lead to extremely large values,
        as the algorithm for LU used divisions to generate U.
       
        It's worth saying though, that the permutations start from the top row to the bottom, and once
        a row is permutted, it'x fixed in place and no long moved. So it can happen that the lower rows have a smaller value in the pivot,
        while the greates value in their column is in row 1 coincidentally, but it's normal and expected behavior, something like this:
       
                | 5 3 4 | , where the first pivot to be set was 5 in row 1, the next one was decided between rows 2 and 3 (which means pivot 2 is set), and so on.
                | 1 2 5 |   
                | 0 0 1 |
       
        The specific algorithm in matf32_lu is doolittle with partial pivoting.
        There are other algorithms which implement full pivoting, but not currently used in this library.
    */
    printf("\n--------------------------------------------------\n");
    printf("matf32_lu");
    printf("\n--------------------------------------------------\n");

    uint16_t permutations[3];
    matf32_t* L = &m3;
    matf32_init(L, A->num_rows, A->num_cols, m3data);
    matf32_zeros(L);
    matf32_t* U = &m4;
    matf32_init(U, A->num_rows, A->num_cols, m4data);
    matf32_zeros(U);

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

    // Try the check functions to determine the shape of a matrix: L is lower triangular and A is square.
    bool tril_matrix = matf32_check_triangular_lower(A);
    printf("Lower Triangular Matrix Check, with A: %s\n", tril_matrix?"True":"False");
    tril_matrix = matf32_check_triangular_lower(L);
    printf("Lower Triangular Matrix Check, with L: %s\n", tril_matrix?"True":"False");

    /*
        Other available checks are for:
            matf32_check_symmetric          -> Symmetric matrices (not positive definite, just checking for symmetry in general)
            matf32_check_upper_hessenberg   -> Upper Hessenberg matrices (not currently implemented anywhere)
            matf32_check_lower_hessenberg   -> Lower Hessenberg matrices (not currenlty implemented anywhere)
    */

    // --------------- 4. SVD Factorization ---------------
    /* 
        SVD stands for: Singular Values Decomposition, which is a matrix factorizacion defined as:
            A = USV', where: U (m x m) is an orthonormal unitary matrix
                             S (m x n) is a diagonal matrix whose entries are the singular values of A
                             V (n x n) is an orthonormal unitary matrix
       
        The SVD is related to the eigenvalues and eigenvectors of A, as follows:
            The colums of V and U are eigenvectors of A'A and AA', respectively.
            The singular values are the square roots of the eigenvalues of A'A or AA'
       
        Graphically, applying U and V correspond to rotations to A, as both are built by accumulating the rotations
        applied to A to generate the decomposition, while S corresponds to stretching A.
       
        (Note: In books, the notation is actually: U, Sigma and V, but for computations S is used instead of Sigma)
        
        There are several methods for calculating the SVD, some of which include eigenvalues and others that do not.
        In this library, the One-Sided Jacobi Algorithm is implemented, which doesn't use eigenvalues, and is implemented as follows:
            
            1. Iteratively apply Jacobi rotations (J) to A. As reference, a Jacobi rotation matrix looks like this:
                    | 1  0  0  0 |
                    | 0  c  s  0 |  In other words: an identity matrix with a 2x2 rotation matrix embedded in it.
                    | 0 -s  c  0 |  c = cos, s = sin. The sign conventions may vary and row-major/column-major may infuence how to operate it.
                    | 0  0  0  1 |  The indices for the rotation matrix vary, it can be anywhere in the matrix as long as it covers the diagonal.
       
                In One-Sided Jacobi, the rotations are applied like this: A = AJ, to turn A into AV = US,
                and V = VJ, where V starts off as the identity matrix, in order to accumulate or save the rotations used.
       
            2. After getting AV = US and V, the singular values are calculated: Each singular value is the norm of each column of AV, as follows:
                    sigma_i = norm(AV(:,i)), which are used to generate S.
       
            3. Now U is calculated: each column of U is computed by normalizing the columns of AV using the singular values:
                    U(:,i) = AV(:,i) / sigma_i
       
            4. The SVD is matrices are complete now, you can reconstruct A = USV' to check the answer.
               (But as A was overwritten, it's important to save a copy of A before executing the SVD routine, otherwise the answer won't match)
                You can also check if U and V are correct by calculating: U'U and V'V, respectively, as those operations should be almost equal to the identity matrix.
    */
    printf("\n--------------------------------------------------\n");
    printf("matf32_jacobi_svd");
    printf("\n--------------------------------------------------\n");

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
    matf32_jacobi_svd(A, U, S, V);
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
    printf("A_copy:\n");
    matf32_print(A_copy);

    // ---------------------------------------------------------------------------
    // Auxiliary Functions
    // ---------------------------------------------------------------------------

    // Add matf32_norm, matf32_cond, and so on.

    
}
