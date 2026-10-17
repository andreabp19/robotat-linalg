
% Author: Andrea Pineda
% Date created: 6 Aug 2025
% Last modified: 6 Jan 2025
%
% Script for testing quadprog functions and compare with quadprog from
% Robotat Linalg

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
%  Testing methods for solving quadratic programs with equality
%  restrictions
% -------------------------------------------------------------------------

z = 100;

% -------------------------------------------------------------------------
% Print data to a .h to use as input in PlatformIO
% -------------------------------------------------------------------------

results_file = fopen('quadprog_qp_test_data.h','w');
fprintf(results_file, 'uint8_t z = %.9f;\n', z);

% -------------------------------------------------------------------------
% Testing
% -------------------------------------------------------------------------

quadprog_result = zeros(z,1);
quadprog_time = zeros(z,2);
matlab_quadprog_qp_time = zeros(5,1);

matlab_raw_quadprog_qp_time = zeros(100,5);
matlab_raw_quadprog_nullspace_time = zeros(100,9);

for n = 1:1:5

    % Uncomment to generate random matrices 
    c = randn(n,1);
    beq = randn(n,1);

    while true
        Aeq = randn(n);
        cond_Aeq = cond(Aeq,'fro');

        if (cond_Aeq < 1000)
            break;
        end
    end

    while true
        temp_Q = randn(n);
        Q = temp_Q'*temp_Q;
        cond_Q = cond(Q,'fro');

        if (cond_Q < 1000)
            break;
        end
    end

    % -------------------------------------------------------------
    % Example of an extremelly well-conditioned problem
    % -------------------------------------------------------------
    % c   = linspace(1, 5, n)';    
    % beq = linspace(1, 5, n)';
    % Q = eye(n);                
    % Aeq = eye(n);

    % ---------------------------------------------------------------------
    % Test quadprog
    % ---------------------------------------------------------------------
    for i = 1:1:100
        tic;
        x = quadprog(Q,c,[],[],Aeq,beq,[],[],[])
        %quadprog_time(i,2) = toc;
        %quadprog_time(i,1) = i;
        matlab_raw_quadprog_qp_time(i, n) = toc;
    end

    Ax = Aeq*x
    q = (1/2)*x'*Q*x + x'*c

    %matlab_quadprog_qp_time(n) = mean(quadprog_time(:,2));

    % Print matrices to .h
    print_matrix_to_file(results_file, Q, 'float Q_data', n);
    print_matrix_to_file(results_file, Aeq, 'float Aeq_data', n);
    print_matrix_to_file(results_file, c, 'float c_data',n);
    print_matrix_to_file(results_file, beq, 'float beq_data',n);
    print_matrix_to_file(results_file, x, 'float R_x_data', n);
    print_matrix_to_file(results_file, Ax, 'float R_Ax_data', n);

end

%save matlab_quadprog_time_results matlab_quadprog_qp_time

%% ------------------------------------------------------------------------
%  Nullspace method for solving the KKT system
% -------------------------------------------------------------------------

%clear
%clc

quadprog_qp_time_matlab = zeros(9,1);

results_file = fopen('quadprog_qp_nullspace_test_data.h','w');

for n = 1:1:9

    while true
        Aeq = randn(n,n+1);
        cond_Aeq = cond(Aeq);
    
        if (cond_Aeq < 100)
            break;
        end
    end
    
    while true
        temp_Q = randn(n+1);
        Q = temp_Q'*temp_Q;
        cond_Q = cond(Q,'fro');
    
        if (cond_Q < 100)
            break;
        end
    end
    
    c = randn(n+1,1);
    beq = randn(n,1);
    
    % Time measurement
    for i = 1:1:100
        tic;
        matlab_quadprog_result = quadprog(Q,c,[],[],Aeq,beq,[],[],[])
        %quadprog_qp_time_matlab(n) = toc;
        matlab_raw_quadprog_nullspace_time(i, n) = toc;
    end

    qp_nullspace_result = quadprog_qp_nullspace(Q,c,Aeq,beq)
    
    % if (abs(matlab_quadprog_result - qp_nullspace_result) < 1E-05)
    %     disp("1E-05 Tolerance: True");
    % else if
    %     disp("1E-05 Tolerance: False");
    % end
    
    print_matrix_to_file(results_file, Aeq, 'float Aeq_nullspace_data',n);
    print_matrix_to_file(results_file, Q, 'float Q_nullspace_data', n);
    print_matrix_to_file(results_file, c, 'float c_nullspace_data', n);
    print_matrix_to_file(results_file, beq, 'float beq_nullspace_data', n);
    print_matrix_to_file(results_file, qp_nullspace_result, 'float R_nullspace', n);

end

%save quadprog_qp_nullspace_results_matlab quadprog_qp_time_matlab

save matlab_raw_quadprog_qp_time_results matlab_raw_quadprog_qp_time...
    matlab_raw_quadprog_nullspace_time

%% ------------------------------------------------------------------------
%  Testing the LDL' factorization with Bunch and Parlett pivoting
% -------------------------------------------------------------------------

% clear
% clc
% format long
% 
% results_file = fopen('quadprog_test_results.h','w');
% 
% Q = randn(5);
% Q = Q'*Q;
% A = randn(5);
% %b = [1 2 3 4 5]'
% %c = [1 2 3 4 5]'
% b = randn(5,1)
% c = randn(5,1);
% n = [c; b];
% 
% print_matrix_to_file(results_file, Q, 'float Q_data',1);
% print_matrix_to_file(results_file, A, 'float Aeq_data',1);
% print_matrix_to_file(results_file, b, 'float beq_data',1);
% print_matrix_to_file(results_file, c, 'float c_data',1);
% 
% 
% M = [Q A'; A zeros(5)];
% 
% % TODO: Modify the function to accept Q, A and b, and solve the system
% % inside of it.
% % Also add the construction of vector n to solve it as My = n as it is done
% % in quadprog_qp
% [P,L,D] = quadprog_qp_ldlt(M);
% 
% PMPt = P*M*P';
% LDLt = L*D*L';
% 
% PMPt - LDLt;
% 
% % Now to solve Ax = b with Bunch and Parlett LDL'
% 
% z1 = L\(P*n);
% z2 = D\z1;
% z3 = (L')\z2;
% x = P'*z3;
% Mx = M*x;
% 
% matlab_quadprog_result = quadprog(Q,c,[],[],A,b,[],[],[])
% x(1:5,1)
% Ax = A*x(1:5,1)
% b
% 
% print_matrix_to_file(results_file, x(1:5,1), 'float R_x_data',1);
% print_matrix_to_file(results_file, Ax, 'float R_Ax_data',1);


% Save input data to operate in quadprog and check for errors.

%save matlab_quadprog_sqp_time_results