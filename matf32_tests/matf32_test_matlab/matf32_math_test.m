
% @author Andrea Pineda
% 
% file for comparing with matf32_math functions
% executes operations and exports the data to the following files:
% - matf32_math_test_results.h -> values used and results for each n.
% - matf32_math_test_stats.csv -> time (mean, max, min, stddev) for each n.

% List of functions in matf32_math
% matf32_add - DONE
% matf32_sub - DONE
% matf32_scale - DONE
% matf32_trans - DONE
% matf32_mul - DONE
% matf32_lup - DONE
% matf32_inv - DONE
% matf32_dot - DONE
% matf32_vecposmul - DONE
% matf32_vecpremul - DONE
% matf32_vecmul_col_row - DONE
% matf32_arr_add
% matf32_arr_sub
% matf32_arr_mul

%% CLEAN EVERYTHING

clear
clc

%% MAIN

c = 5;      % Scale integer
x = 100;   % Number of tests per operation

file_labels = ["n","mean_t(s)","max_t(s)","min_s(t)","std_dev_t(s)"];

% -------------------------------------------------------------------------
% DATA ARRAYS FOR EACH OPERATION TO TEST
% -------------------------------------------------------------------------

% Data Arrays:
% 1. Arrays for saving the measured time per operation
% 2. Statistics arrays: [matrix size, mean, max, min, std. deviation]

add_time = zeros(1,x);
add_stats = zeros(10,5);

sub_time = zeros(1,x);
sub_stats = zeros(10,5);

mul_time = zeros(1,x);
mul_stats = zeros(10,5);

lu_time = zeros(1,x);
lu_stats = zeros(10,5);

scale_time = zeros(1,x);
scale_stats = zeros(10,5);

inv_time = zeros(1,x);
inv_stats = zeros(10,5);

trans_time = zeros(1,x);
trans_stats = zeros(10,5);

dot_time = zeros(1,x);
dot_stats = zeros(10,5);

vecpremul_time = zeros(1,x);
vecpremul_stats = zeros(10,5);

vecposmul_time = zeros(1,x);
vecposmul_stats = zeros(10,5);

vecmul_col_row_time = zeros(1,x);
vecmul_col_row_stats = zeros(10,5);

arr_add_time = zeros(1,x);
arr_add_stats = zeros(10,5);

arr_sub_time = zeros(1,x);
arr_sub_stats = zeros(10,5);

arr_mul_time = zeros(1,x);
arr_mul_stats = zeros(10,5);

% -------------------------------------------------------------------------
% PRINT DATA TO A .h TO USE AS INPUT DATA IN PLATFORMIO
% -------------------------------------------------------------------------
results_file = fopen('matf32_math_test_results.h','w');

fprintf(results_file, 'float c = %.9f;\n', c);
fprintf(results_file, 'uint8_t x = %.9f;\n', x);

% -------------------------------------------------------------------------
% OPERATIONS TESTING
% -------------------------------------------------------------------------

for n = 1:1:10

    A = randn(n);
    rowvec = randn(1,n);
    colvec = randn(n,1);

    matArray = cat(3,A,A,A);

    % ---------------------------------------------------------------------
    % PRINT DATA TO A .h TO USE AS INPUT DATA IN PLATFORMIO
    % ---------------------------------------------------------------------

    fprintf(results_file, '\n\n//n=%i\n', n);

    % PRINT A
    fprintf(results_file, 'float A_data%i[%i] = {', n, n*n);

    temp_A = transpose(A);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_A(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % PRINT rowvec
    fprintf(results_file, 'float rowvec_data%i[%i] = {', n, n);

    for z = 1:1:length(rowvec)
        fprintf(results_file, '%.9f', rowvec(z));

        if (z < length(rowvec))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % PRINT colvec
    fprintf(results_file, 'float colvec_data%i[%i] = {', n, n);

    for z = 1:1:length(colvec)
        fprintf(results_file, '%.9f', colvec(z));

        if (z < length(colvec))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST ADD
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        add_result = A+A;
        add_time(i) = toc;
    end
    
    add_stats(n,1) = n;
    add_stats(n,2) = mean(add_time);
    add_stats(n,3) = max(add_time);
    add_stats(n,4) = min(add_time);
    add_stats(n,5) = std(add_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_add%i[%i] = {', n, n*n);
    
    temp_add_result = transpose(add_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_add_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');
    
    % ---------------------------------------------------------------------
    % TEST SUB
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        sub_result = A-A;
        sub_time(i) = toc;
    end
    
    sub_stats(n,1) = n;
    sub_stats(n,2) = mean(sub_time);
    sub_stats(n,3) = max(sub_time);
    sub_stats(n,4) = min(sub_time);
    sub_stats(n,5) = std(sub_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_sub%i[%i] = {', n, n*n);
    
    temp_sub_result = transpose(sub_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_sub_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST SCALE
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        scale_result = A*c;
        scale_time(i) = toc;
    end

    scale_stats(n,1) = n;
    scale_stats(n,2) = mean(scale_time);
    scale_stats(n,3) = max(scale_time);
    scale_stats(n,4) = min(scale_time);
    scale_stats(n,5) = std(scale_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_scale%i[%i] = {', n, n*n);
    
    temp_scale_result = transpose(scale_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_scale_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');
    
    % ---------------------------------------------------------------------
    % TEST TRANSPOSE
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        trans_result = transpose(A);
        trans_time(i) = toc;
    end
    
    trans_stats(n,1) = n;
    trans_stats(n,2) = mean(trans_time);
    trans_stats(n,3) = max(trans_time);
    trans_stats(n,4) = min(trans_time);
    trans_stats(n,5) = std(trans_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_trans%i[%i] = {', n, n*n);
    
    temp_trans_result = transpose(trans_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_trans_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST MUL
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        mul_result = A*A;
        mul_time(i) = toc;
    end

    mul_stats(n,1) = n;
    mul_stats(n,2) = mean(mul_time);
    mul_stats(n,3) = max(mul_time);
    mul_stats(n,4) = min(mul_time);
    mul_stats(n,5) = std(mul_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_mul%i[%i] = {', n, n*n);
    
    temp_mul_result = transpose(mul_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_mul_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST LU
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        [L,U] = lu(A);
        lu_time(i) = toc;
    end
    
    lu_stats(n,1) = n;
    lu_stats(n,2) = mean(lu_time);
    lu_stats(n,3) = max(lu_time);
    lu_stats(n,4) = min(lu_time);
    lu_stats(n,5) = std(lu_time);

    % PRINT RESULT L
    fprintf(results_file, 'float R_lu_L%i[%i] = {', n, n*n);
    
    temp_L = transpose(L);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_L(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % PRINT RESULT U
    fprintf(results_file, 'float R_lu_U%i[%i] = {', n, n*n);

    temp_U = transpose(U);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_U(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');
    
    % ---------------------------------------------------------------------
    % TEST INV
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        inv_result = inv(A);
        inv_time(i) = toc;
    end

    inv_stats(n,1) = n;
    inv_stats(n,2) = mean(inv_time);
    inv_stats(n,3) = max(inv_time);
    inv_stats(n,4) = min(inv_time);
    inv_stats(n,5) = std(inv_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_inv%i[%i] = {', n, n*n);
    
    temp_inv_result = transpose(inv_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_inv_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');
    
    % ---------------------------------------------------------------------
    % TEST DOT
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        dot_result = dot(rowvec,colvec);
        dot_time(i) = toc;
    end

    dot_stats(n,1) = n;
    dot_stats(n,2) = mean(dot_time);
    dot_stats(n,3) = max(dot_time);
    dot_stats(n,4) = min(dot_time);
    dot_stats(n,5) = std(dot_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_dot%i[%i] = {', n, n*n);
    
    temp_dot_result = transpose(dot_result);
    for z = 1:1:length(dot_result)
        fprintf(results_file, '%.9f', temp_dot_result(z));

        if (z < length(dot_result))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST VECPREMUL
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        vecpremul_result = rowvec*A;
        vecpremul_time(i) = toc;
    end
    
    vecpremul_stats(n,1) = n;
    vecpremul_stats(n,2) = mean(vecpremul_time);
    vecpremul_stats(n,3) = max(vecpremul_time);
    vecpremul_stats(n,4) = min(vecpremul_time);
    vecpremul_stats(n,5) = std(vecpremul_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_vecpremul%i[%i] = {', n, n);
    
    temp_vecpremul_result = transpose(vecpremul_result);
    for z = 1:1:length(vecpremul_result)
        fprintf(results_file, '%.9f', temp_vecpremul_result(z));

        if (z < length(vecpremul_result))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST VECPOSMUL
    % ---------------------------------------------------------------------
    for i = 1:1:x
        tic;
        vecposmul_result = A*colvec;
        vecposmul_time(i) = toc;
    end
    
    vecposmul_stats(n,1) = n;
    vecposmul_stats(n,2) = mean(vecposmul_time);
    vecposmul_stats(n,3) = max(vecposmul_time);
    vecposmul_stats(n,4) = min(vecposmul_time);
    vecposmul_stats(n,5) = std(vecposmul_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_vecposmul%i[%i] = {', n, n);
    
    temp_vecposmul_result = transpose(vecposmul_result);
    for z = 1:1:length(vecposmul_result)
        fprintf(results_file, '%.9f', temp_vecposmul_result(z));

        if (z < length(vecposmul_result))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST VECMUL COL ROW
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        vecmul_result = colvec*rowvec;
        vecmul_col_row_time(i) = toc;
    end
    
    vecmul_col_row_stats(n,1) = n;
    vecmul_col_row_stats(n,2) = mean(vecmul_col_row_time);
    vecmul_col_row_stats(n,3) = max(vecmul_col_row_time);
    vecmul_col_row_stats(n,4) = min(vecmul_col_row_time);
    vecmul_col_row_stats(n,5) = std(vecmul_col_row_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_vecmul%i[%i] = {', n, n*n);
    
    temp_vecmul_result = transpose(vecmul_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_vecmul_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST ARR ADD
    % ---------------------------------------------------------------------
   
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp + matArray(:,:,2);
        arr_add_result = temp2 + matArray(:,:,3);
        arr_add_time(i) = toc;
    end

    arr_add_stats(n,1) = n;
    arr_add_stats(n,2) = mean(arr_add_time);
    arr_add_stats(n,3) = max(arr_add_time);
    arr_add_stats(n,4) = min(arr_add_time);
    arr_add_stats(n,5) = std(arr_add_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_arr_add%i[%i] = {', n, n*n);
    
    temp_arr_add_result = transpose(arr_add_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_arr_add_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST ARR SUB
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp - matArray(:,:,2);
        arr_sub_result = temp2 - matArray(:,:,3);
        arr_sub_time(i) = toc;
    end
    
    arr_sub_stats(n,1) = n;
    arr_sub_stats(n,2) = mean(arr_sub_time);
    arr_sub_stats(n,3) = max(arr_sub_time);
    arr_sub_stats(n,4) = min(arr_sub_time);
    arr_sub_stats(n,5) = std(arr_sub_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_arr_sub%i[%i] = {', n, n*n);
    
    temp_arr_sub_result = transpose(arr_sub_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_arr_sub_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

    % ---------------------------------------------------------------------
    % TEST ARR MUL
    % ---------------------------------------------------------------------
    
    for i = 1:1:x
        tic;
        temp = matArray(:,:,1);
        temp2 = temp*matArray(:,:,2);
        arr_mul_result = temp2*matArray(:,:,3);
        arr_mul_time(i) = toc;
    end
    
    arr_mul_stats(n,1) = n;
    arr_mul_stats(n,2) = mean(arr_mul_time);
    arr_mul_stats(n,3) = max(arr_mul_time);
    arr_mul_stats(n,4) = min(arr_mul_time);
    arr_mul_stats(n,5) = std(arr_mul_time);

    % PRINT RESULT
    fprintf(results_file, 'float R_arr_mul%i[%i] = {', n, n*n);
    
    temp_arr_mul_result = transpose(arr_mul_result);
    for z = 1:1:(n*n)
        fprintf(results_file, '%.9f', temp_arr_mul_result(z));

        if (z < (n*n))
            fprintf(results_file, ',');
        end
    end

    fprintf(results_file, '};\n');

end

fclose(results_file);

% -------------------------------------------------------------------------
% EXPORT STATS TO A .csv TO USE IN EXCEL
% -------------------------------------------------------------------------

% ADD
writematrix("ADD", 'matf32_math_test_stats.csv');
writematrix(file_labels, 'matf32_math_test_stats.csv','WriteMode', 'append');
writematrix(add_stats,'matf32_math_test_stats.csv','WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% SUB
writematrix("SUB", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(sub_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% SCALE
writematrix("SCALE", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(scale_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% TRANS
writematrix("TRANS", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(trans_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% MUL
writematrix("MUL", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(mul_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% INV
writematrix("INV", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(inv_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% DOT
writematrix("DOT", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(dot_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% VECPREMUL
writematrix("VECPREMUL", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(vecpremul_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% VECPOSMUL
writematrix("VECPOSMUL", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(vecposmul_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% VECMUL COL ROW
writematrix("VECMUL COL ROW", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(vecmul_col_row_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% ARR ADD
writematrix("ARR ADD", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(arr_add_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% ARR SUB
writematrix("ARR SUB", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(arr_sub_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');

% ADD MUL
writematrix("ARR MUL", 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(file_labels, 'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix(arr_mul_stats,'matf32_math_test_stats.csv', 'WriteMode', 'append');
writematrix("", 'matf32_math_test_stats.csv','WriteMode', 'append');