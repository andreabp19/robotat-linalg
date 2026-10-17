
% Author: Andrea Pineda
% Date created: 5 Nov 2025
% Last modified: 6 Jan 2026
%
% Script for testing active-set functions based on Algorithm 5.4 Martins,
% Engineering Design Optimization, implemented here and in Robotat Linalg.

%% ------------------------------------------------------------------------
% Clean Workspace
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
% Testing Active-Set for Inequality Constraints
% -------------------------------------------------------------------------

clear
clc
results_file = fopen('quadprog_sqp_test_data.h','w');

matlab_raw_quadprog_sqp_ineq_time = zeros(100,5)
matlab_raw_quadprog_sqp_mixed_constraints_time = zeros(100, 5)

for n = 1:1:5

    x0 = zeros(n,1);
    Q = randn(n);
    Q = Q'*Q;
    c = randn(n,1);
    Aeq = randn(n);
    beq = randn(n,1);
    Ain = randn(n);
    bin = randn(n,1);

    opts = optimoptions('quadprog','Algorithm','active-set','MaxIterations',30);
    
    for i = 1:1:100
        tic;
        active_set_result = quadprog_active_set_ineq(Q,c,Ain,bin,x0)
        matlab_raw_quadprog_sqp_ineq_time(i, n) = toc;
    end

    matlab_result = quadprog(Q,c,Ain,bin,[],[],[],[],x0,opts)

    q_active_set = ((1/2)*active_set_result'*Q*active_set_result) + (c'*active_set_result)
    q_matlab = ((1/2)*matlab_result'*Q*matlab_result) + (c'*matlab_result)

    print_matrix_to_file(results_file, x0, 'float x0_data', n);
    print_matrix_to_file(results_file, Q, 'float Q_data', n);
    print_matrix_to_file(results_file, c, 'float c_data', n);
    print_matrix_to_file(results_file, Aeq, 'float Aeq_data', n);
    print_matrix_to_file(results_file, beq, 'float beq_data', n);
    print_matrix_to_file(results_file, Ain, 'float Ain_data', n);
    print_matrix_to_file(results_file, bin, 'float bin_data', n);
    print_matrix_to_file(results_file, matlab_result, 'float R_x_data', n);
    print_matrix_to_file(results_file, q_matlab, 'float R_q_data', n);

end

%% ------------------------------------------------------------------------
% Testing Active-Set for Both Equality and Inequality Constraints
% -------------------------------------------------------------------------

%clear
%clc
results_file = fopen('quadprog_sqp_test_data.h','w');

n_tests = 6;

matlab_active_set_time = 0;
matlab_mean_active_set_time = zeros(6,1);

for n = 1:n_tests

    fprintf("\n--------------------------------------------------\n");
    fprintf("n = %i\n", n);
    fprintf("--------------------------------------------------\n\n");

    x0 = eye(n,1);

    % QP structure generated in chatgpt to ensure it's feasible and
    % scalable for different matrix dimensions (and avoid singularities,
    % because randomly generating matrices is giving too many matrices
    % close to singularities).
    
    % Positive definite Q
    Q = randn(n); 
    Q = Q'*Q + 1e-3*eye(n);
    c = randn(n,1);
    
    Aeq_rows = 2;
    %Aeq_rows = (n + 2) / 3;
    %Aeq_rows = floor(Aeq_rows);
    %if Aeq_rows < 1
    %    Aeq_rows = 1;
    %end
    
    Aeq = randn(Aeq_rows, n);
    beq = Aeq * x0;   % makes x0 feasible
    
    Ain_rows = 2;
    %Ain_rows = (n + 1) / 2;  
    %Ain_rows = floor(Ain_rows);
    %if Ain_rows < 2
    %    Ain_rows = 2;
    %end

    % Inequality constraints
    %Ain_rows = max(2, round(n/2));
    Ain = randn(Ain_rows, n);
    slack = abs(randn(Ain_rows,1)) + 0.5;
    bin = Ain * x0 + slack;

    % Custom Solver
    for i = 1:1:100
        tic;
        x_active_set = quadprog_active_set_general(Q, c, Aeq, beq, Ain, bin, x0)
        matlab_raw_quadprog_sqp_mixed_constraints_time(i, n) = toc;
    end
    
    q_active_set = ((1/2)*x_active_set'*Q*x_active_set) + (c'*x_active_set)

    % MATLAB Solver
    opts = optimoptions('quadprog','Algorithm','active-set','Display','off','MaxIterations',50);
    x_matlab = quadprog(Q, c, Ain, bin, Aeq, beq, [], [], x0, opts)
    q_matlab = ((1/2)*x_matlab'*Q*x_matlab) + (c'*x_matlab)

    % Save results to a .h
    print_matrix_to_file(results_file, x0, 'float x0_data', n);
    print_matrix_to_file(results_file, Q, 'float Q_data', n);
    print_matrix_to_file(results_file, c, 'float c_data', n);
    print_matrix_to_file(results_file, Aeq, 'float Aeq_data', n);
    print_matrix_to_file(results_file, beq, 'float beq_data', n);
    print_matrix_to_file(results_file, Ain, 'float Ain_data', n);
    print_matrix_to_file(results_file, bin, 'float bin_data', n);
    print_matrix_to_file(results_file, x_matlab, 'float R_x_data', n);
    print_matrix_to_file(results_file, q_matlab, 'float R_q_data', n);

end

%save matlab_quadprog_sqp_time_results matlab_mean_active_set_time

save matlab_raw_quadprog_sqp_time matlab_raw_quadprog_sqp_ineq_time...
    matlab_raw_quadprog_sqp_mixed_constraints_time