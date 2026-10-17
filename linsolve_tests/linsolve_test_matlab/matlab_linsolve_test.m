
% Author: Andrea Pineda
% Date created: 2 Aug 2025
% Last modified: 6 Jan 2026
%
% Script for testing linsolve functions from Robotat Linalg

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear 
clc

%% ------------------------------------------------------------------------
%  Testing methods for solving Ax = b linear systems
% -------------------------------------------------------------------------

x = 100;

% -------------------------------------------------------------------------
% Data arrays for each operation
% -------------------------------------------------------------------------

% col 1: iteration, col 2: time
fwd_subs_time = 0;
matlab_fwd_subs_time = zeros(10,1);
matlab_raw_linsolve_fwd_subs_time = zeros(100,2);

bwd_subs_time = 0;
matlab_bwd_subs_time = zeros(10,1);
matlab_raw_linsolve_bwd_subs_time = zeros(100,2);

cholesky_time = 0;
matlab_cholesky_time = zeros(10,1);
matlab_raw_linsolve_cholesky_time = zeros(100,2);

qr_time = 0;
matlab_qr_time = zeros(10,1);
matlab_raw_linsolve_qr_time = zeros(100,2);

lu_time = 0;
matlab_lu_time = zeros(10,1);
matlab_raw_linsolve_lu_time = zeros(100,2);

svd_time = 0;
matlab_svd_time = zeros(10,1);
matlab_raw_linsolve_svd_time = zeros(100,2);

% -------------------------------------------------------------------------
% Print data to a .h to use as input in PlatformIO
% -------------------------------------------------------------------------

results_file = fopen('linsolve_test_results.h','w');

fprintf(results_file, 'uint8_t x = %.9f;\n', x);

% -------------------------------------------------------------------------
% Testing
% -------------------------------------------------------------------------
    
for n = 1:1:10

    % ---------------------------------------------------------------------
    % Generate matrices to use
    % ---------------------------------------------------------------------
    
    while true
        A = randn(n);       % Squarematrix
        A_L = tril(A);      % Lower triangular matrix
        A_U = triu(A);      % Upper triangular matrix

        if (cond(A_L,'fro') < 100 && cond(A_U,'fro') < 100)
            break;
        end
    end

    while true
        B = randn(n);

        if (cond(B,'fro') < 40 && det(B) ~= 0)
            break;
        end
    end

    while true
        temp = randn(n);
        C = transpose(temp)*temp;   % Symmetric Definite Matrix

        if (cond(C,'fro') < 50)
            break;
        end
    end

    while true
        D = randn(n,n-1);
        cond_D = cond(D);

        if (cond(D) < 100)
            break;
        end
    end

    %b = randn(n,1);
    b = linspace(1,3,n)';

    cond_A = cond(A,'fro');
    cond_A_L = cond(A_L,'fro');
    cond_A_U = cond(A_U,'fro');
    cond_C = cond(C,'fro');
    cond_B = cond(B,'fro');

    print_matrix_to_file(results_file, A, 'float A_data', n);
    print_matrix_to_file(results_file, A_L, 'float A_L_data', n);
    print_matrix_to_file(results_file, A_U, 'float A_U_data', n);
    print_matrix_to_file(results_file, C, 'float C_data', n);
    print_matrix_to_file(results_file, B, 'float B_data', n);
    print_matrix_to_file(results_file, b, 'float b_data', n);
    
    if n > 1
        print_matrix_to_file(results_file, D, 'float D_data', n-1);
    end

    % ---------------------------------------------------------------------
    % 1. Forward Substitution - Lower Triangular Matrices
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        fwd_subs_result = A_L\b;
        matlab_raw_linsolve_fwd_subs_time(i, n) = toc;
    end
    %fwd_subs_time = toc;
    %matlab_fwd_subs_time(n) = fwd_subs_time/x;
    
    print_matrix_to_file(results_file, fwd_subs_result, 'float R_fwd_subs_data', n);
    
    % ---------------------------------------------------------------------
    % 2. Backward Substitution - Upper Triangular Matrices
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        bwd_subs_result = A_U\b;
        matlab_raw_linsolve_bwd_subs_time(i, n) = toc;
    end
    %bwd_subs_time = toc;
    %matlab_bwd_subs_time(n) = bwd_subs_time/x;
    
    print_matrix_to_file(results_file, bwd_subs_result, 'float R_bwd_subs_data', n);
    
    % ---------------------------------------------------------------------
    % 3. Cholesky - Symmetric Positive Definite Matrices
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:x
        tic;
        cholesky_result = C\b;
        matlab_raw_linsolve_cholesky_time(i, n) = toc;
    end
    %cholesky_time = toc;
    %matlab_cholesky_time(n) = cholesky_time/x;

    print_matrix_to_file(results_file, cholesky_result, 'float R_cholesky_data', n);
    
    % ---------------------------------------------------------------------
    % 4. QR - Rectangular Matrices (non squared)
    % ---------------------------------------------------------------------

    if n > 1
        %tic;
        for i = 1:1:x
            tic;
            qr_result = D\b;
            matlab_raw_linsolve_qr_time(i, n) = toc;
        end
        %qr_time = toc;
        %matlab_qr_time(n) = qr_time/x;

        print_matrix_to_file(results_file, qr_result, 'float R_qr_data', n-1);
    end

    % ---------------------------------------------------------------------
    % 5. LU
    % ---------------------------------------------------------------------

    %tic;
    for i = 1:1:x
        tic;
        lu_result = linsolve(B,b);
        matlab_raw_linsolve_lu_time(i, n) = toc;
    end
    %lu_time = toc;
    %matlab_lu_time(n) = lu_time/x;

    print_matrix_to_file(results_file, lu_result, 'float R_lu_data', n);
 
    % ---------------------------------------------------------------------
    % 6. Jacobi SVD
    % ---------------------------------------------------------------------

    [U,S,V] = one_sided_jacobi_svd(A);

    %tic;
    for i = 1:1:x
        tic;
        svd_result = V*pinv(S)*U'*b;
        matlab_raw_linsolve_svd_time(i, n) = toc;
    end
    %svd_time = toc;
    %matlab_svd_time(n) = svd_time/x;

    % Print results to .h
    print_matrix_to_file(results_file, svd_result, 'float R_svd_data',n);

end

fclose(results_file);

%save matlab_linsolve_time_results.mat matlab_fwd_subs_time...
%    matlab_bwd_subs_time matlab_cholesky_time matlab_qr_time...
%    matlab_lu_time matlab_svd_time

save matlab_raw_linsolve_time_results.mat matlab_raw_linsolve_fwd_subs_time...
    matlab_raw_linsolve_bwd_subs_time matlab_raw_linsolve_cholesky_time...
    matlab_raw_linsolve_qr_time matlab_raw_linsolve_lu_time...
    matlab_raw_linsolve_svd_time