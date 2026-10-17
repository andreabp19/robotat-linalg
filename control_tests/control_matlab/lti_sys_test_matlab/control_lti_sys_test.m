
% Author: Andrea Pineda
% Date created: 28 Oct2025
% Last modified: 28 Oct 2025
% 
% File for testing LTI system control algorithms to compare them with those
% in robotat_control.

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
% LTI System Discretizations
% -------------------------------------------------------------------------

K = 300; % Number of iterations for measuring time of each function

% Arrays for saving the mean time of each function for each matrix size
matlab_c2d_fwd_eul_time = zeros(10,9);
matlab_c2d_bwd_eul_time = zeros(10,9);
matlab_c2d_tustin_time = zeros(10,9);

results_file = fopen('control_lti_sys_test_data.h','w'); % File for data
fprintf(results_file, 'uint8_t K = %.9f;\n', K); % Save K

matlab_raw_control_c2d_fwd_eul_time = zeros(K, 10);
matlab_raw_control_c2d_bwd_eul_time = zeros(K, 10);
matlab_raw_control_c2d_tustin_time = zeros(K, 10);

for n = 2:1:10

    A_fwd_eul = randn(n);
    B_fwd_eul = randn(n,1);

    A_bwd_eul = randn(n);
    B_bwd_eul = randn(n,1);

    A_tustin = randn(n);
    B_tustin = randn(n,1);

    C = randn(1,n);
    D = 0;
    Ts = 0.001;
    
    print_matrix_to_file(results_file,A_fwd_eul,'float A_fwd_eul',n);
    print_matrix_to_file(results_file,A_bwd_eul,'float A_bwd_eul',n);
    print_matrix_to_file(results_file,A_tustin,'float A_tustin',n);
    print_matrix_to_file(results_file,B_fwd_eul,'float B_fwd_eul',n);
    print_matrix_to_file(results_file,B_bwd_eul,'float B_bwd_eul',n);
    print_matrix_to_file(results_file,B_tustin,'float B_tustin',n);
    print_matrix_to_file(results_file,C,'float sys_lti_C',n);
    print_matrix_to_file(results_file,D,'float sys_lti_D',n);
    
    % ---------------------------------------------------------------------
    % State Spaces - c2d - Forward Euler
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:K
        tic;
        [Ad,Bd] = ctr_c2d(A_fwd_eul,B_fwd_eul,Ts,'ForwardEuler');
        matlab_raw_control_c2d_fwd_eul_time(i, n) = toc;
    end
    %matlab_c2d_fwd_eul_time(n) = toc/K
    
    % Save Ad and Bd to .h
    print_matrix_to_file(results_file,Ad,'float R_c2d_fwd_eul_Ad',n);
    print_matrix_to_file(results_file,Bd,'float R_c2d_fwd_eul_Bd',n);
    
    % ---------------------------------------------------------------------
    % State Spaces - c2d - Backward Euler
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:K
        tic;
        [Ad,Bd] = ctr_c2d(A_bwd_eul,B_bwd_eul,Ts,'BackwardEuler');
        matlab_raw_control_c2d_bwd_eul_time(i, n) = toc;
    end
    %matlab_c2d_bwd_eul_time(n) = toc/K
    
    % Save Ad and Bd to .h
    print_matrix_to_file(results_file,Ad,'float R_c2d_bwd_eul_Ad',n);
    print_matrix_to_file(results_file,Bd,'float R_c2d_bwd_eul_Bd',n);
    
    % ---------------------------------------------------------------------
    % State Spaces - c2d - Tustin
    % ---------------------------------------------------------------------
    
    %tic;
    for i = 1:1:K
        tic;
        [Ad,Bd] = ctr_c2d(A_tustin,B_tustin,Ts,'Tustin');
        matlab_raw_control_c2d_tustin_time(i, n) = toc;
    end
    %matlab_c2d_tustin_time(n) = toc/K
    
    % Save Ad and Bd to .h
    print_matrix_to_file(results_file,Ad,'float R_c2d_tustin_Ad',n);
    print_matrix_to_file(results_file,Bd,'float R_c2d_tustin_Bd',n);

end

save matlab_control_lti_sys_results matlab_c2d_fwd_eul_time...
    matlab_c2d_bwd_eul_time matlab_c2d_tustin_time

save matlab_raw_control_lti_sys_time_results matlab_raw_control_c2d_fwd_eul_time...
    matlab_raw_control_c2d_bwd_eul_time matlab_raw_control_c2d_tustin_time