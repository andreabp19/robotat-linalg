
% Author: Andrea Pineda
% Date created: 28 Oct 2025
% Last modified: 6 Jan 2026
% 
% File for testing PID control algorithms and compare them with those in
% robotat_control.

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
% Variables That Apply to all Discretizations.
% 
% Exception: kp, ki and kd, as those are adjusted individually to
% each discretization, and the variables' history data.
% -------------------------------------------------------------------------

e_k_1 = 0;
e_k_2 = 0;
u_k_1 = 0;
u_k_2 = 0;
x_k = 0;
y_k = 0;

A = 0.9;
B = 0.1;
C = 1;

tau = 0.2;
Ts = 0.001;
samples = 300;

% In case an impulse response is needed: r_k = zeros, then r_k(i) = 1
r_k = ones(samples,1);  % step input
r_k(1:50) = 0;

t = (0:samples-1)*Ts; % Time array for plotting

results_file = fopen('control_pid_test_data.h','w');

matlab_raw_control_pid_pure_discrete_time = zeros(samples, 1);
matlab_raw_control_pid_fwd_euler_time = zeros(samples, 1);
matlab_raw_control_pid_bwd_euler_time = zeros(samples, 1);
matlab_raw_control_pid_tustin_time = zeros(samples, 1);

%% ------------------------------------------------------------------------
% PID Discretizations: Pure Discrete
% -------------------------------------------------------------------------

kp = 2.5;
ki = 0.95;
kd = 0.5;

u_k_history_pure_discrete = zeros(samples,1);
x_k_history_pure_discrete = zeros(samples,1);
y_k_history_pure_discrete = zeros(samples,1);
pid_pure_discrete_mean_time = 0;

for k = 1:1:samples
    tic
    [u_k,e_k] = pid_update(kp,ki,kd,...
        u_k_1,u_k_2,e_k_1,e_k_2,r_k(k),y_k,tau,Ts,'PureDiscrete');
    matlab_raw_control_pid_pure_discrete_time(k) = toc;
    %pid_pure_discrete_mean_time = pid_pure_discrete_mean_time + toc;
    
    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_2 = u_k_1;
    u_k_1 = u_k;

    x_k = A*x_k + B*u_k;
    y_k = C*x_k;
    
    u_k_history_pure_discrete(k) = u_k;
    x_k_history_pure_discrete(k) = x_k;
    y_k_history_pure_discrete(k) = y_k;
end
pid_pure_discrete_mean_time = pid_pure_discrete_mean_time/samples

% figure(1);
% subplot(3,1,1);
% plot(t, r_k, '--', t, y_k_history_pure_discrete, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System Output (y_k)');
% legend('Reference','Output');
% grid on;
% 
% subplot(3,1,2);
% plot(t, r_k, '--', t, u_k_history_pure_discrete, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('Control Signal (u_k)');
% grid on;
% 
% subplot(3,1,3);
% plot(t, x_k_history_pure_discrete, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System State (x_k)');
% grid on;
% 
% exportgraphics(gcf,'pid_pure_discrete_test.eps');
% 
% save control_pid_results_matlab x_k_history_pure_discrete...
%     u_k_history_pure_discrete y_k_history_pure_discrete r_k

%% ------------------------------------------------------------------------
% PID Discretizations: Forward Euler
% -------------------------------------------------------------------------

kp = 1;
ki = 0.2;
kd = 0.5;

u_k_history_fwd_eul = zeros(samples,1);
x_k_history_fwd_eul = zeros(samples,1);
y_k_history_fwd_eul = zeros(samples,1);
pid_fwd_eul_mean_time = 0;

for k = 1:1:samples
    tic
    [u_k,e_k] = pid_update(kp,ki,kd,...
        u_k_1,u_k_2,e_k_1,e_k_2,r_k(k),y_k,tau,Ts,'ForwardEuler');
    matlab_raw_control_pid_fwd_euler_time(k) = toc;
    %pid_fwd_eul_mean_time = pid_fwd_eul_mean_time + toc;

    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_2 = u_k_1;
    u_k_1 = u_k;

    x_k = A*x_k + B*u_k;
    y_k = C*x_k;

    u_k_history_fwd_eul(k) = u_k;
    x_k_history_fwd_eul(k) = x_k;
    y_k_history_fwd_eul(k) = y_k;
end
pid_fwd_eul_mean_time = pid_fwd_eul_mean_time/samples

% figure(2);
% subplot(3,1,1);
% plot(t, r_k, '--', t, y_k_history_fwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System Output (y_k)');
% legend('Reference','Output');
% grid on;
% 
% subplot(3,1,2);
% plot(t, r_k, '--', t, u_k_history_fwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('Control Signal (u_k)');
% grid on;
% 
% subplot(3,1,3);
% plot(t, x_k_history_fwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System State (x_k)');
% grid on;
% 
% exportgraphics(gcf,'pid_fwd_eul_test.eps');

%% ------------------------------------------------------------------------
% PID Discretizations: Backward Euler
% -------------------------------------------------------------------------

kp = 1;
ki = 0.2;
kd = 0.5;

u_k_history_bwd_eul = zeros(samples,1);
x_k_history_bwd_eul = zeros(samples,1);
y_k_history_bwd_eul = zeros(samples,1);
pid_bwd_eul_mean_time = 0;

for k = 1:1:samples
    tic
    [u_k,e_k] = pid_update(kp,ki,kd,...
        u_k_1,u_k_2,e_k_1,e_k_2,r_k(k),y_k,tau,Ts,'BackwardEuler');
    matlab_raw_control_pid_bwd_euler_time(k) = toc;
    %pid_bwd_eul_mean_time = pid_bwd_eul_mean_time + toc;

    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_2 = u_k_1;
    u_k_1 = u_k;

    x_k = A*x_k + B*u_k;
    y_k = C*x_k;

    u_k_history_bwd_eul(k) = u_k;
    x_k_history_bwd_eul(k) = x_k;
    y_k_history_bwd_eul(k) = y_k;
end
pid_bwd_eul_mean_time = pid_bwd_eul_mean_time/samples

% figure(3);
% subplot(3,1,1);
% plot(t, r_k, '--', t, y_k_history_bwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System Output (y_k)');
% legend('Reference','Output');
% grid on;
% 
% subplot(3,1,2);
% plot(t, r_k, '--', t, u_k_history_bwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('Control Signal (u_k)');
% grid on;
% 
% subplot(3,1,3);
% plot(t, x_k_history_bwd_eul, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System State (x_k)');
% grid on;
% 
% exportgraphics(gcf,'pid_bwd_eul_test.eps');

%% ------------------------------------------------------------------------
% PID Discretizations: Tustin
% -------------------------------------------------------------------------

kp = 1;
ki = 0.2;
kd = 0.5;

u_k_history_tustin = zeros(samples,1);
x_k_history_tustin = zeros(samples,1);
y_k_history_tustin = zeros(samples,1);
pid_tustin_mean_time = 0; 

for k = 1:1:samples
    tic
    [u_k,e_k] = pid_update(kp,ki,kd,...
        u_k_1,u_k_2,e_k_1,e_k_2,r_k(k),y_k,tau,Ts,'Tustin');
    matlab_raw_control_pid_tustin_time(k) = toc;
    %pid_tustin_mean_time = pid_tustin_mean_time + toc;

    e_k_2 = e_k_1;
    e_k_1 = e_k;
    u_k_2 = u_k_1;
    u_k_1 = u_k;

    x_k = A*x_k + B*u_k;
    y_k = C*x_k;

    u_k_history_tustin(k) = u_k;
    x_k_history_tustin(k) = x_k;
    y_k_history_tustin(k) = y_k;
end
pid_tustin_mean_time = pid_tustin_mean_time/samples

% figure(4);
% subplot(3,1,1);
% plot(t, r_k, '--', t, y_k_history_tustin, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System Output (y_k)');
% legend('Reference','Output');
% grid on;
% 
% subplot(3,1,2);
% plot(t, r_k, '--', t, u_k_history_tustin, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('Control Signal (u_k)');
% grid on;
% 
% subplot(3,1,3);
% plot(t, x_k_history_tustin, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Amplitude');
% title('System State (x_k)');
% grid on;
% 
% exportgraphics(gcf,'pid_tustin_test.eps');

%% ------------------------------------------------------------------------
% PID Discretizations: Save Data
% -------------------------------------------------------------------------

print_matrix_to_file(results_file,A,'float A_data',1);
print_matrix_to_file(results_file,B,'float B_data',1);
print_matrix_to_file(results_file,C,'float C_data',1);
print_matrix_to_file(results_file,r_k,'float r_k_data',1);
fprintf(results_file, 'float kp = %.9f;\n', kp);
fprintf(results_file, 'float ki = %.9f;\n', ki);
fprintf(results_file, 'float kd = %.9f;\n', kd);
fprintf(results_file, 'float tau = %.9f;\n', tau);
fprintf(results_file, 'float dt = %.9f;\n', Ts);
fprintf(results_file, '#define SAMPLES (%i)\n', samples);

save control_pid_results_matlab x_k_history_pure_discrete...
    x_k_history_fwd_eul x_k_history_bwd_eul x_k_history_tustin...
    u_k_history_pure_discrete u_k_history_fwd_eul u_k_history_bwd_eul...
    u_k_history_tustin y_k_history_pure_discrete y_k_history_fwd_eul...
    y_k_history_bwd_eul y_k_history_tustin r_k

save matlab_raw_control_pid_time_results matlab_raw_control_pid_pure_discrete_time...
    matlab_raw_control_pid_fwd_euler_time matlab_raw_control_pid_bwd_euler_time...
    matlab_raw_control_pid_tustin_time