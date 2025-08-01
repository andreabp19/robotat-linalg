
% Author: Andrea Pineda
% Date created: 1 Aug 2025
%
% Script for plotting time results of the validated functions

% Most variables include in their name the device in which they were
% created (i.e. that they belong to that device's test), except for the
% matlab variables, which have no additional name appended: rotx_time,
% roty_time, etc.

% Additionally, all variable names have the corresponding name of the
% function so, for example, if you want to compare both matlab's and the
% esp32's results of rotx and rob_rotx, respectively, then the variables
% names are rotx_time for matlab and rob_rotx_time_esp32 for robotat
% linalg's equivalent of rotx in the  esp32. The same applies with
% homstrans for matlab and the equivalent of rob_apply_transform_esp32 in robotat
% linal tested in the esp32.

% May be confusing at first, so refer to the documentation or follow the
% comments in this document to make sure you compare the right functions
% with each other.

% -------------------------------------------------------------------------
% Clear workspace and terminal
% -------------------------------------------------------------------------
clear
clc

% -------------------------------------------------------------------------
% Importing all variables from .mat and .txt files
% -------------------------------------------------------------------------

load matlab_robotics_test_stats.mat

fid = fopen("esp32_robotics_test_stats.txt");
lines = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
lines = lines{1};

temp_lines = find(contains(lines, 'rob_rotx'));
rob_rotx_time_esp32 = str2num(lines{temp_lines + 1});

temp_lines = find(contains(lines, 'rob_roty'));
rob_roty_time_esp32 = str2num(lines{temp_lines + 1});

temp_lines = find(contains(lines, 'rob_rotz'));
rob_rotz_time_esp32 = str2num(lines{temp_lines + 1});

temp_lines = find(contains(lines, 'rob_transl'));
rob_transl_time_esp32 = str2num(lines{temp_lines + 1});

temp_lines = find(contains(lines, 'rob_apply_transform'));
rob_apply_transform_time_esp32 = str2num(lines{temp_lines + 1});

temp_lines = find(contains(lines, 'rob_apply_euler_angles'));
rob_apply_euler_angles_time_esp32 = str2num(lines{temp_lines + 1});

% Horizontal axis: iterations
x = linspace(1, 100, 100);

% Colors
violet = [178,128,224]/255;
yellow = [255,204,102]/255;

% -------------------------------------------------------------------------
% Plotting results
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% 1. rotx and rob_rotx
% -------------------------------------------------------------------------

figure(1);
hold on

semilogy(x, rotx_time, 'Color',yellow,'LineWidth',2);
semilogy(x, rob_rotx_time_esp32, 'Color',violet,'LineWidth',2);

legend('MATLAB', 'ESP32', 'Location', 'best');

xlim([1,100]);
xlabel('Iteracion');
ylabel('Time (s)');

grid on
set(gca, 'YScale', 'log');

exportgraphics(gcf,'validacion_rob_rotx.eps');

% -------------------------------------------------------------------------
% 2. roty vs rob_roty
% -------------------------------------------------------------------------

figure(2);
hold on

semilogy(x, roty_time, 'Color',yellow,'LineWidth',2);
semilogy(x, rob_roty_time_esp32, 'Color',violet,'LineWidth',2);

legend('MATLAB', 'ESP32', 'Location', 'best');

xlim([1,100]);
xlabel('Iteracion');
ylabel('Time (s)');

grid on
set(gca, 'YScale', 'log');

exportgraphics(gcf,'validacion_rob_roty.eps');

% -------------------------------------------------------------------------
% 3. rotz vs rob_rotz
% -------------------------------------------------------------------------

figure(3);
hold on

semilogy(x, rotz_time, 'Color',yellow,'LineWidth',2);
semilogy(x, rob_rotz_time_esp32, 'Color',violet,'LineWidth',2);

legend('MATLAB', 'ESP32', 'Location', 'best');

xlim([1,100]);
xlabel('Iteracion');
ylabel('Time (s)');

grid on
set(gca, 'YScale', 'log');

exportgraphics(gcf,'validacion_rob_rotz.eps');

% -------------------------------------------------------------------------
% 4. transl vs rob_transl
% -------------------------------------------------------------------------

figure(4);
hold on

semilogy(x, transl_time, 'Color',yellow,'LineWidth',2);
semilogy(x, rob_transl_time_esp32, 'Color',violet,'LineWidth',2);

legend('MATLAB', 'ESP32', 'Location', 'best');

xlim([1,100]);
xlabel('Iteracion');
ylabel('Time (s)');

grid on
set(gca, 'YScale', 'log');

exportgraphics(gcf,'validacion_rob_transl.eps');

% -------------------------------------------------------------------------
% 5. homtrans vs rob_apply_transform
% -------------------------------------------------------------------------

figure(5);
hold on

semilogy(x, homtrans_time, 'Color',yellow,'LineWidth',2);
semilogy(x, rob_apply_transform_time_esp32, 'Color',violet,'LineWidth',2);

legend('MATLAB', 'ESP32', 'Location', 'best');

xlim([1,100]);
xlabel('Iteracion');
ylabel('Time (s)');

grid on
set(gca, 'YScale', 'log');

exportgraphics(gcf,'validacion_rob_apply_transform.eps');

% -------------------------------------------------------------------------
% 6. successive implementation of rotx and homtrans vs
% rob_apply_euler_angles
% -------------------------------------------------------------------------

