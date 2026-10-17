
% @author Andrea Pineda
% Last modified: 6 Jan 2026
%
% For generating variables and time measures to compare matf32 functions.

% List of matlab functions/operators to use based on matf32 functions:
% -------------------------------------------------------------------------
% matf32 (robotat-linalg)   | matlab function/operator          | Added?
% -------------------------------------------------------------------------
% matf32_add                | +                                 | DONE
% matf32_sub                | -                                 | DONE
% matf32_scale              | *                                 | DONE
% matf32_trans              | ' or transpose()                  | DONE
% matf32_mul                | *                                 | DONE
% matf32_inv                | inv                               | DONE
% matf32_dot                | dot                               | DONE
% matf32_vecposmul          | *                                 | DONE
% matf32_vecpremul          | *                                 | DONE
% matf32_vecmul_col_row     | *                                 | DONE
% matf32_arr_add            | +                                 | DONE
% matf32_arr_sub            | -                                 | DONE
% matf32_arr_mul            | *                                 | DONE
% matf32_pinv               | pinv                              | DONE
% matf32_lu                 | lu                                | DONE
% matf32_cholesky           | chol                              | DONE
% matf32_qr                 | qr                                | DONE
% matf32_jacobi_svd         | jacobi_svd (custom)               | DONE

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc

%% ------------------------------------------------------------------------
%  Linear Algebra Operations Testing
% -------------------------------------------------------------------------

% ------------------------- General variables -----------------------------

c = 5;      % Scale integer
x = 1000;    % Number of tests per operation

% ------------------------- Data Arrays -----------------------------------

matlab_add_time = 0;
matlab_mean_add_time = zeros(10,1);
matlab_raw_math_add_time = zeros(100,10);

matlab_sub_time = 0;
matlab_mean_sub_time = zeros(10,1);
matlab_raw_math_sub_time = zeros(x,10);

matlab_mul_time = 0;
matlab_mean_mul_time = zeros(10,1);
matlab_raw_math_mul_time = zeros(x,10);

matlab_scale_time = 0;
matlab_mean_scale_time = zeros(10,1);
matlab_raw_math_scale_time = zeros(x,10);

matlab_inv_time = 0;
matlab_mean_inv_time = zeros(10,1);
matlab_raw_math_inv_time = zeros(x,10);

matlab_pinv_basic_time = 0;
matlab_mean_pinv_basic_time = zeros(10,1);
matlab_raw_math_pinv_basic_time = zeros(x,10);

matlab_pinv_svd_time = 0;
matlab_mean_pinv_svd_time = zeros(10,1);
matlab_raw_math_pinv_svd_time = zeros(x,10);

matlab_trans_time = 0;
matlab_mean_trans_time = zeros(10,1);
matlab_raw_math_trans_time = zeros(x,10);

matlab_dot_time = 0;
matlab_mean_dot_time = zeros(10,1);
matlab_raw_math_dot_time = zeros(x,10);

matlab_vecpremul_time = 0;
matlab_mean_vecpremul_time = zeros(10,1);
matlab_raw_math_vecpremul_time = zeros(x,10);

matlab_vecposmul_time = 0;
matlab_mean_vecposmul_time = zeros(10,1);
matlab_raw_math_vecposmul_time = zeros(x,10);

matlab_vecmul_col_row_time = 0;
matlab_mean_vecmul_col_row_time = zeros(10,1);
matlab_raw_math_vecmul_col_row_time = zeros(x,10);

matlab_arr_add_time = 0;
matlab_mean_arr_add_time = zeros(10,1);
matlab_raw_math_arr_add_time = zeros(x,10);

matlab_arr_sub_time = 0;
matlab_mean_arr_sub_time = zeros(10,1);
matlab_raw_math_arr_sub_time = zeros(x,10);

matlab_arr_mul_time = 0;
matlab_mean_arr_mul_time = zeros(10,1);
matlab_raw_math_arr_mul_time = zeros(x,10);

matlab_exp_time = 0;
matlab_mean_exp_time = zeros(10,1);
matlab_raw_math_exp_time = zeros(x,10);

matlab_lu_time = 0;
matlab_mean_lu_time = zeros(10,1);
matlab_raw_math_lu_time = zeros(x,10);

matlab_cholesky_time = 0;
matlab_mean_cholesky_time = zeros(10,1);
matlab_raw_math_cholesky_time = zeros(x,10);

matlab_qr_time = 0;
matlab_mean_qr_time = zeros(10,1);
matlab_raw_math_qr_time = zeros(x,10);

matlab_jacobi_svd_time = 0;
matlab_mean_jacobi_svd_time = zeros(10,1);
matlab_raw_math_jacobi_svd_time = zeros(x,10);

% -------------------------------------------------------------------------
% Print data to .h to use as input in PlatformIO
% -------------------------------------------------------------------------

results_file = fopen('matf32_matlab_data.h','w');

fprintf(results_file, 'float c = %.9f;\n', c);
fprintf(results_file, 'uint8_t x = %.9f;\n', x);

% -------------------------------------------------------------------------
% Main Loop
% -------------------------------------------------------------------------

for n = 1:1:10

    % ---------------------------------------------------------------------
    % Variables definition
    % ---------------------------------------------------------------------

    % Matrices, vectors and arrays for general matrix operations
    while true
        A = randn(n);

        if (det(A) ~= 0 && cond(A,'fro') < 1000)
            break;
        end
    end

    C = A'*A;

    rowvec = randn(1,n);
    colvec = randn(n,1);

    matArray = cat(3,A,A,A);

    % ---------------------------------------------------------------------
    % Print initial data to .h
    % ---------------------------------------------------------------------

    fprintf(results_file, '\n\n//n=%i\n', n);

    print_matrix_to_file(results_file, A, 'float A_data', n);
    print_matrix_to_file(results_file, C, 'float C_data', n);
    print_matrix_to_file(results_file, rowvec, 'float rowvec_data', n);
    print_matrix_to_file(results_file, colvec, 'float colvec_data', n);

    % ---------------------------------------------------------------------
    % Test add
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        add_result = A+A;
        matlab_raw_math_add_time(i, n) = toc;
    end
    %matlab_add_time = toc;
    %matlab_mean_add_time(n) = matlab_add_time/x;

    print_matrix_to_file(results_file, add_result, 'float R_add', n);
    
    % ---------------------------------------------------------------------
    % Test sub
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        sub_result = A-A;
        matlab_raw_math_sub_time(i, n) = toc;
    end
    %matlab_sub_time = toc;
    %matlab_mean_sub_time(n) = matlab_sub_time/x;

    print_matrix_to_file(results_file, sub_result, 'float R_sub', n);

    % ---------------------------------------------------------------------
    % Test scale
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        scale_result = A*c;
        matlab_raw_math_scale_time(i, n) = toc;
    end
    %matlab_scale_time = toc;
    %matlab_mean_scale_time(n) = matlab_scale_time/x;

    print_matrix_to_file(results_file, scale_result, 'float R_scale', n);
    
    % ---------------------------------------------------------------------
    % Test transpose
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        trans_result = transpose(A);
        matlab_raw_math_trans_time(i, n) = toc;
    end
    %matlab_trans_time = toc;
    %matlab_mean_trans_time(n) = matlab_trans_time/x;

    print_matrix_to_file(results_file, trans_result, 'float R_trans', n);

    % ---------------------------------------------------------------------
    % Test mul
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        mul_result = A*A;
        matlab_raw_math_mul_time(i, n) = toc;
    end
    %matlab_mul_time = toc;
    %matlab_mean_mul_time(n) = matlab_mul_time/x;

    print_matrix_to_file(results_file, mul_result, 'float R_mul', n);
    
    % ---------------------------------------------------------------------
    % Test inv
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        inv_result = inv(A);
        matlab_raw_math_inv_time(i, n) = toc;
    end
    %matlab_inv_time = toc;
    %matlab_mean_inv_time(n) = matlab_inv_time/x;

    print_matrix_to_file(results_file, inv_result, 'float R_inv', n);

    % ---------------------------------------------------------------------
    % Test pinv: (A'A)^-1 * A'
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        pinv_result = (A'*A)^-1 * A';
        matlab_raw_math_pinv_basic_time(i, n) = toc;
    end
    %matlab_pinv_basic_time = toc;
    %matlab_mean_pinv_basic_time(n) = matlab_pinv_basic_time/x;

    print_matrix_to_file(results_file,pinv_result,'float R_pinv_basic',n);

    % ---------------------------------------------------------------------
    % Test pinv (SVD)
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        pinv_result = pinv(A);
        matlab_raw_math_pinv_svd_time(i, n) = toc;
    end
    %matlab_pinv_svd_time = toc;
    %matlab_mean_pinv_svd_time(n) = matlab_pinv_svd_time/x;

    print_matrix_to_file(results_file, pinv_result, 'float R_pinv_svd', n);
    
    % ---------------------------------------------------------------------
    % Test dot
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        dot_result = dot(rowvec,colvec);
        matlab_raw_math_dot_time(i, n) = toc;
    end
    %matlab_dot_time = toc;
    %matlab_mean_dot_time(n) = matlab_dot_time/x;

    print_matrix_to_file(results_file, dot_result, 'float R_dot', n);

    % ---------------------------------------------------------------------
    % Test vecpremul
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        vecpremul_result = rowvec*A;
        matlab_raw_math_vecpremul_time(i, n) = toc;
    end
    %matlab_vecpremul_time = toc;
    %matlab_mean_vecpremul_time(n) = matlab_vecpremul_time/x;

    print_matrix_to_file(results_file,vecpremul_result,'float R_vecpremul',n);

    % ---------------------------------------------------------------------
    % Test vecposmul
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        vecposmul_result = A*colvec;
        matlab_raw_math_vecposmul_time(i, n) = toc;
    end
    %matlab_vecposmul_time = toc;
    %matlab_mean_vecposmul_time(n) = matlab_vecposmul_time/x;

    print_matrix_to_file(results_file,vecposmul_result,'float R_vecposmul',n);

    % ---------------------------------------------------------------------
    % Test vecmul col row
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        vecmul_result = colvec*rowvec;
        matlab_raw_math_vecmul_col_row_time(i, n) = toc;
    end
    %matlab_vecmul_col_row_time = toc;
    %matlab_mean_vecmul_col_row_time(n) = matlab_vecmul_col_row_time/x;

    print_matrix_to_file(results_file, vecmul_result, 'float R_vecmul', n);

    % ---------------------------------------------------------------------
    % Test arr add
    % ---------------------------------------------------------------------
   
    %tic;
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp + matArray(:,:,2);
        arr_add_result = temp2 + matArray(:,:,3);
        matlab_raw_math_arr_add_time(i, n) = toc;
    end
    %matlab_arr_add_time = toc;
    %matlab_mean_arr_add_time(n) = matlab_arr_add_time/x;

    print_matrix_to_file(results_file,arr_add_result,'float R_arr_add',n);

    % ---------------------------------------------------------------------
    % Test arr sub
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp - matArray(:,:,2);
        arr_sub_result = temp2 - matArray(:,:,3);
        matlab_raw_math_arr_sub_time(i, n) = toc;
    end
    %matlab_arr_sub_time = toc;
    %matlab_mean_arr_sub_time(n) = matlab_arr_sub_time/x;

    print_matrix_to_file(results_file,arr_sub_result,'float R_arr_sub',n);

    % ---------------------------------------------------------------------
    % Test arr mul
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp*matArray(:,:,2);
        arr_mul_result = temp2*matArray(:,:,3);
        matlab_raw_math_arr_mul_time(i, n) = toc;
    end
    %matlab_arr_mul_time = toc;
    %matlab_mean_arr_mul_time(n) = matlab_arr_mul_time/x;

    print_matrix_to_file(results_file,arr_mul_result,'float R_arr_mul',n);

    % ---------------------------------------------------------------------
    % Test exponent
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        A_exp = A*A*A*A;
        matlab_raw_math_exp_time(i, n) = toc;
    end
    %matlab_exp_time = toc;
    %matlab_mean_exp_time(n) = matlab_exp_time/x;

    print_matrix_to_file(results_file,A_exp,'float R_exp',n);

    % ---------------------------------------------------------------------
    % Test LU decomposition
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        [L,U,P] = lu(A);
        matlab_raw_math_lu_time(i, n) = toc;
    end
    PA = L*U;
    %matlab_lu_time = toc;
    %matlab_mean_lu_time(n) = matlab_lu_time/x;

    print_matrix_to_file(results_file, PA, 'float R_lu', n);

    % ---------------------------------------------------------------------
    % Test Cholesky decomposition
    % ---------------------------------------------------------------------

    %tic;
    for i = 1:1:x
        tic;
        cholesky_result = chol(C);
        matlab_raw_math_cholesky_time(i, n) = toc;
    end
    %matlab_cholesky_time = toc;
    %matlab_mean_cholesky_time(n) = matlab_cholesky_time/x;

    print_matrix_to_file(results_file,cholesky_result,'float R_cholesky',n);

    % ---------------------------------------------------------------------
    % Test QR decomposition
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        [Q,R] = qr(A);
        matlab_raw_math_qr_time(i, n) = toc;
    end
    A_QR = Q*R;
    %matlab_qr_time = toc;
    %matlab_mean_qr_time(n) = matlab_qr_time/x;

    % Print Q and R
    print_matrix_to_file(results_file, A_QR, 'float R_qr', n);

    % ---------------------------------------------------------------------
    % Test custom function SVD with One-Sided Jacobi
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        [U,S,V] = one_sided_jacobi_svd(A);
        matlab_raw_math_jacobi_svd_time(i, n) = toc;
    end
    A_SVD = U*S*V';
    %matlab_jacobi_svd_time = toc;
    %matlab_mean_jacobi_svd_time(n) = matlab_jacobi_svd_time/x;

    print_matrix_to_file(results_file, A_SVD, 'float R_jacobi_svd', n);
end

fclose(results_file);

% ------------------------- Save Data to a .mat ---------------------------

% Mean Times
%save matlab_matf32_time_results.mat matlab_mean_add_time...
%    matlab_mean_sub_time matlab_mean_scale_time matlab_mean_trans_time...
%    matlab_mean_mul_time matlab_mean_inv_time matlab_mean_dot_time...
%    matlab_mean_vecposmul_time matlab_mean_vecpremul_time...
%    matlab_mean_vecmul_col_row_time matlab_mean_arr_add_time...
%    matlab_mean_arr_sub_time matlab_mean_arr_mul_time...
%    matlab_mean_lu_time matlab_mean_cholesky_time matlab_mean_qr_time...
%    matlab_mean_pinv_basic_time matlab_mean_pinv_svd_time...
%    matlab_mean_lu_time matlab_mean_jacobi_svd_time matlab_mean_exp_time

% Raw Times
save matlab_raw_math_time_results.mat matlab_raw_math_add_time...
    matlab_raw_math_sub_time matlab_raw_math_scale_time matlab_raw_math_trans_time...
    matlab_raw_math_mul_time matlab_raw_math_inv_time matlab_raw_math_dot_time...
    matlab_raw_math_vecposmul_time matlab_raw_math_vecpremul_time...
    matlab_raw_math_vecmul_col_row_time matlab_raw_math_arr_add_time...
    matlab_raw_math_arr_sub_time matlab_raw_math_arr_mul_time...
    matlab_raw_math_lu_time matlab_raw_math_cholesky_time matlab_raw_math_qr_time...
    matlab_raw_math_pinv_basic_time matlab_raw_math_pinv_svd_time...
    matlab_raw_math_lu_time matlab_raw_math_jacobi_svd_time matlab_raw_math_exp_time