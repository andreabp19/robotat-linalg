
% Author: Andrea Pineda
% Date created: 28 Oct2025
% Last modified: 9 Nov 2025
% 
% File for testing Nonlinear system control algorithms to compare them with
% those in robotat_control.

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
% Variables and Nonlinear System Definition
% -------------------------------------------------------------------------

samples = 100; % Number of iterations for measuring time of each function

% Simple pendulum variables
m = 2;      % kg
l = 1;      % m 
g = 9.81;   % m/s^2
delta = 2;

% Time arrays
matlab_raw_control_linear_state_feedback_time = zeros(samples, 1);
matlab_raw_control_linloc_time = zeros(samples, 1);
matlab_raw_control_nonlin_simulate_fwd_eul_time = zeros(samples, 1);
matlab_raw_control_nonlin_simulate_rk4_time = zeros(samples, 1);

% Simple Pendulum model: f = system dynamics, h = system outputs
f = @(x,u) [x(2);
            -(g/l)*sin(x(1)) + (1/m(l*l)*u)];
h = @(x,u) atan(x(1));
xss = [-1; 5];
uss = 2;

results_file = fopen('control_nonlin_sys_test_data.h','w'); % File for data
fprintf(results_file, 'uint8_t samples = %.9f;\n', samples);

%% ------------------------------------------------------------------------
% Nonlinear System Dynamics and Outputs
% -------------------------------------------------------------------------

% Time measurement for f(x,u)
tic;
for i = 1:1:samples
    fss = f(xss,uss);
end
mean_sys_dynamics_time = toc/samples;

% Time measurement for h(x,u)
tic;
for i = 1:1:samples
    hss = h(xss,uss);
end
mean_sys_outputs_time = toc/samples;

%% ------------------------------------------------------------------------
% Nonlinear System Linearization (loclin_fast)
% -------------------------------------------------------------------------
matlab_linloc_time = zeros(samples,1);
% Time measurement for loclin_fast
for i = 1:1:samples
    tic;
    [A,B,C,D] = loclin_fast(f,h,xss,uss,delta);
    %matlab_linloc_time(i) = toc;
    matlab_raw_control_linloc_time(i) = toc;
end
%mean_linloc_time = toc/samples;

%% ------------------------------------------------------------------------
% Linear State Feedback
% -------------------------------------------------------------------------

matlab_linear_state_feedback_time = zeros(10,1);
for n = 1:1:10
    x = randn(n,1);
    u = 0;
    K = randn(1,n);
    xss = randn(n,1);
    uss = 2;
    
    %tic;
    for i = 1:1:samples
        tic;
        u = -K * (x - xss) + uss;
        matlab_linear_state_feedback_time(i) = toc;
    end
    matlab_linear_state_feedback_time(n) = toc/samples;
    print_matrix_to_file(results_file, x, 'float x_data', n);
    print_matrix_to_file(results_file, K, 'float K_data', n);
    print_matrix_to_file(results_file, xss, 'float xss_data', n);
end

%% ------------------------------------------------------------------------
% Nonlinear Simulate (Step Forward)
% -------------------------------------------------------------------------

x_k = [4; 5];
u_k = 2;

% --------------- Step Forward with Forward Euler -------------------------
matlab_nonlin_simulate_fwd_eul_time = zeros(samples,1);
for i = 1:1:samples
    tic;
    [x_k_1_fwd_eul] = nonlin_sys_simulate(f,x_k,u_k,delta,'ForwardEuler');
    %matlab_nonlin_simulate_fwd_eul_time(i) = toc;
    matlab_raw_control_nonlin_simulate_fwd_eul_time(i) = toc;
end
%mean_nonlin_simulate_fwd_eul_time = toc/samples;
print_matrix_to_file(results_file,x_k_1_fwd_eul,'float R_x_k_1_fwd_eul',1);
    

% --------------- Step Forward with Runge-Kutta4 --------------------------
matlab_nonlin_simulate_rk4_time = zeros(samples,1);
for i = 1:1:samples
    tic;
    [x_k_1_rk4] = nonlin_sys_simulate(f,x_k,u_k,delta,'RK4');
    %matlab_nonlin_simulate_rk4_time(i) = toc;
    matlab_raw_control_nonlin_simulate_rk4_time(i) = toc;
end
%mean_nonlin_simulate_rk4_time = toc/samples;
print_matrix_to_file(results_file, x_k_1_rk4, 'float R_x_k_1_rk4', 1);

%% ------------------------------------------------------------------------
% Save Data to a .mat
% -------------------------------------------------------------------------

save control_nonlin_sys_results_matlab matlab_linloc_time...
    matlab_nonlin_simulate_fwd_eul_time matlab_nonlin_simulate_rk4_time...
    matlab_linear_state_feedback_time

save matlab_raw_control_nonlin_sys_time_results matlab_raw_control_linear_state_feedback_time...
    matlab_raw_control_linloc_time matlab_raw_control_nonlin_simulate_fwd_eul_time...
    matlab_raw_control_nonlin_simulate_rk4_time







