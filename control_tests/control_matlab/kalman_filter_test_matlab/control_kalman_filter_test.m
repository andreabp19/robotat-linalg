
% Author: Andrea Pineda
% Date created: 28 Oct2025
% Last modified: 6 Jan 2026
% 
% File for testing Kalman Filter algorithms to compare them with those
% in robotat_control.

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc
format long

%% ------------------------------------------------------------------------
% Kalman Filter Test Loop
% -------------------------------------------------------------------------

results_file = fopen('control_kalman_filter_test_data.h','w');

rows = 1;
A = 1;
B = 1;
C = 1;
D = 0;
F = B;

% Condiciones iniciales
samples = 300;
P_prior = 1;
P_post = 1;
x0 = 1;
u0 = 0;
y0 = C*x0;
xhat0 = 0;
rng = 10;

xhat_prior = xhat0;
xhat_post = 0;
x = x0;
u = u0;
y = y0;

xhat_history = zeros(samples,1);
x_history = zeros(samples,1);
u_history = zeros(samples,1);
y_history = zeros(samples,1);
w_history = zeros(samples,1);
v_history = zeros(samples,1);

xhat_history(1) = xhat0;
x_k_history(1) = x;
u_k_history(1) = u;
y_k_history(1) = y;
w_history(1) = 0;
v_history(1) = 0;

desv_est_proceso = 0; % Desviacion estandar del proceso/actuador
desv_est_medicion = 1; % Desviacion estandar del sensor

% Parametros de los procesos estocasticos
sw = desv_est_proceso;
sv = desv_est_medicion;

% "Matrices" de covarianza del ruido de proceso y observacion
Qw = sw^2;
Qv = sv^2;

mean_kalman_update_time = 0; % For measuring time: prediction + correction
matlab_raw_control_kf_time = zeros(samples, 1);

% Kalman Filter loop
for k = 1:samples-1

    u = -0.1*xhat_post;
    %kf_u = -1;
    w = sw*randn(size(u));
    v = sv*randn(size(y));

    x = A*x + B*u + F*w;
    y = C*x + v;

    tic;
    [xhat_prior,P_prior] = kalman_predict(A,B,F,P_post,Qw,xhat_post,u);
    [xhat_post,P_post] = kalman_correct(C,Qv,xhat_prior,P_prior,y);
    matlab_raw_control_kf_time(k) = toc;
    %mean_kalman_update_time = mean_kalman_update_time + toc;

    xhat_history(k+1) = xhat_post;
    x_history(k+1) = x;
    u_history(k+1) = u;
    y_history(k+1) = y;
    w_history(k+1) = w;
    v_history(k+1) = v;
end
%mean_kalman_update_time

fprintf(results_file, 'uint8_t K = %.9f;\n', samples);
fprintf(results_file, 'uint8_t x0 = %.9f;\n', x0);
fprintf(results_file, 'uint8_t y0 = %.9f;\n', y0);
fprintf(results_file, 'uint8_t u0 = %.9f;\n', u0);
fprintf(results_file, 'uint8_t sw = %.9f;\n', sw);
fprintf(results_file, 'uint8_t sv = %.9f;\n', sv);
print_matrix_to_file(results_file, A, 'float A_data', rows);
print_matrix_to_file(results_file, B, 'float B_data', rows);
print_matrix_to_file(results_file, C, 'float C_data', rows);
print_matrix_to_file(results_file, D, 'float D_data', rows);
print_matrix_to_file(results_file, F, 'float F_data', rows);
print_matrix_to_file(results_file, w_history, 'float w_data', samples);
print_matrix_to_file(results_file, v_history, 'float v_data', samples);

%% ------------------------------------------------------------------------
% Kalman Filter Plotting (Optional as everything is plotted in
% matlab_plotting -> control_plotting)
% -------------------------------------------------------------------------

% n = 1:samples;
% figure(1);
% hold on
% plot(n, xhat_history', 'LineWidth', 2.5);
% plot(n, x_history', 'LineWidth', 2.5);
% xlabel('k');
% ylabel('xhat');
% legend('verdadero','estimado');
% grid on
% hold off
% exportgraphics(gcf,'kalman_filter_test_states.eps');
% 
% figure(2);
% hold on
% plot(n, u_history', 'LineWidth', 2.5);
% xlabel('k');
% ylabel('u_k');
% grid on
% hold off
% exportgraphics(gcf,'kalman_filter_test_input.eps');
% 
% figure(3);
% hold on
% plot(n, y_history', 'LineWidth', 2.5);
% xlabel('k');
% ylabel('y_k');
% grid on
% hold off
% exportgraphics(gcf,'kalman_filter_test_output.eps');

%% ------------------------------------------------------------------------
% Save Data to a .mat
% -------------------------------------------------------------------------

%save control_kalman_filter_time_results_matlab mean_kalman_update_time...
%    x_history y_history xhat_history u_history

save matlab_raw_control_kf_time_results matlab_raw_control_kf_time