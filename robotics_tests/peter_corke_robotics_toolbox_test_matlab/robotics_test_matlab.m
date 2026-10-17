

% Author Andrea Pineda
% Last modified: 6 Jan 2026
% File for comparing with robotat_robotics functions

% list of operations to implement:
% - rot matrix 2D and 3D
% - homogeneous transformations 2D and 3D
% - 2D pose
% - 3D pose
% - conversions between homogeneous transformations, rotations matrices,
% quaternions and angles
% - quaternion operations

% Some differences to note:
% transl creates a 4x4 matrix with the coordinates vector and a I matrix
% where the rotation matrix should go.
% On the other hand, rob_transl just assigns the coordinates vector to the
% 4x4 matrix T, which was created beforehand.
%
% rotx, roty, rotz, trotx, troty and trotz work the same as their matlab
% equivalents.
% 
% rob_ print functions have no equivalent because matlab makes it easy to
% print matrices and values. Print functions will not be measured for time
% as those are auxiliary functions.

%% ------------------------------------------------------------------------
%  Clean Workspace and Console, Configure Output Format
% -------------------------------------------------------------------------

clear
clc

%% ------------------------------------------------------------------------
%  Testing operations from the Robotics Toolbox of Peter Corke
% -------------------------------------------------------------------------

x = 100;    % Number of tests per operation

% A_p is calculated as transform_result = homtrans(...)
% T and R matrices are also calculated in differente parts of the code
B_p = [randn(3,1); 1];    % Point (x,y,z,1) with reference to FRAME_B

v = randn(3,1);     % Coord. vector for the hom. transformation matrix

% Angles in radians
phi = 0.25;     
theta = 0.5;
psi = 0.75;

roll = 1.0;
pitch = 2.0;
yaw = 3.0;

% Quaternions
q1 = Quaternion([5,6,7,8]);
uq1 = UnitQuaternion([5,6,7,8]);
scalar = 2;

% -------------------------------------------------------------------------
% Data arrays for each operation
% -------------------------------------------------------------------------

matlab_raw_robotics_transl_time = zeros(x,1);

matlab_raw_robotics_rotx_time = zeros(x,1);
matlab_raw_robotics_roty_time = zeros(x,1);
matlab_raw_robotics_rotz_time = zeros(x,1);

matlab_raw_robotics_trotx_time = zeros(x,1);
matlab_raw_robotics_troty_time = zeros(x,1);
matlab_raw_robotics_trotz_time = zeros(x,1);

matlab_raw_robotics_transform_time = zeros(x,1);
matlab_raw_robotics_inv_transform_time = zeros(x,1);

matlab_raw_robotics_quat_add_time = zeros(x,1);
matlab_raw_robotics_quat_sub_time = zeros(x,1);
matlab_raw_robotics_quat_scale_time = zeros(x,1);
matlab_raw_robotics_quat_mul_time = zeros(x,1);
matlab_raw_robotics_quat_conj_time = zeros(x,1);
matlab_raw_robotics_quat_norm_time = zeros(x,1);
matlab_raw_robotics_quat_inv_time = zeros(x,1);

matlab_raw_robotics_r2t_time = zeros(x,1);
matlab_raw_robotics_t2r_time = zeros(x,1);
matlab_raw_robotics_rpy2r_time = zeros(x,1);
matlab_raw_robotics_rpy2tr_time = zeros(x,1);
matlab_raw_robotics_rt2tr_time = zeros(x,1);
matlab_raw_robotics_eul2r_time = zeros(x,1);
matlab_raw_robotics_eul2tr_time = zeros(x,1);
matlab_raw_robotics_tr2q_time = zeros(x,1);
matlab_raw_robotics_rpy2q_time = zeros(x,1);
matlab_raw_robotics_eul2q_time = zeros(x,1);
matlab_raw_robotics_r2rpy_time = zeros(x,1);
matlab_raw_robotics_tr2rpy_time = zeros(x,1);
matlab_raw_robotics_tr2eul_time = zeros(x,1);
matlab_raw_robotics_q2r_time = zeros(x,1);
matlab_raw_robotics_q2tr_time = zeros(x,1);
matlab_raw_robotics_q2rpy_time = zeros(x,1);
matlab_raw_robotics_q2eul_time = zeros(x,1);
matlab_raw_robotics_r2q_time = zeros(x,1);

% -------------------------------------------------------------------------
% Print data to a .h to use as input in PlatformIO
% -------------------------------------------------------------------------

results_file = fopen('robotics_test_results.h','w');
 
fprintf(results_file, 'uint8_t x = %.9f;\n', x);
fprintf(results_file, 'float phi = %.9f;\n', phi);
fprintf(results_file, 'float theta = %.9f;\n', theta);
fprintf(results_file, 'float psi = %.9f;\n', psi);
fprintf(results_file, 'float roll = %.9f;\n', roll);
fprintf(results_file, 'float pitch = %.9f;\n', pitch);
fprintf(results_file, 'float yaw = %.9f;\n', yaw);
fprintf(results_file, 'float F_AB_v_data[3] = {%.9f, %.9f, %.9f};\n',...
    v(1), v(2), v(3));
fprintf(results_file, 'float p_B_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    B_p(1), B_p(2), B_p(3), B_p(4));
fprintf(results_file, 'float q1_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    q1.s, q1.v(1), q1.v(2), q1.v(3));
fprintf(results_file, 'float uq1_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    uq1.s, uq1.v(1), uq1.v(2), uq1.v(3));
fprintf(results_file, 'float scalar = %.9f;\n', scalar);


% -------------------------------------------------------------------------
% Functions testing
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% 1. transl
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    transl_result = transl(v);
    matlab_raw_robotics_transl_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_transl_data[16] = {');

temp_transl = transpose(transl_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_transl(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 2. rotx
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rotx_result = rotx(theta);
    matlab_raw_robotics_rotx_time(i,1) = toc;
end

% Print results to .h
fprintf(results_file, 'float R_rotx_data[9] = {');

temp_rotx = transpose(rotx_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_rotx(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 3. roty
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    roty_result = roty(theta);
    matlab_raw_robotics_roty_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_roty_data[9] = {');

temp_roty = transpose(roty_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_roty(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 4. rotz
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rotz_result = rotz(theta);
    matlab_raw_robotics_rotz_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_rotz_data[9] = {');

temp_rotz = transpose(rotz_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_rotz(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 5. trotx
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    trotx_result = trotx(theta);
    matlab_raw_robotics_trotx_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_trotx_data[16] = {');

temp_trotx = transpose(trotx_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_trotx(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');


% -------------------------------------------------------------------------
% 6. troty
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    troty_result = troty(theta);
    matlab_raw_robotics_troty_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_troty_data[16] = {');

temp_troty = transpose(troty_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_troty(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 7. trotz
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    trotz_result = trotz(theta);
    matlab_raw_robotics_trotz_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_trotz_data[16] = {');

temp_trotz = transpose(trotz_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_trotz(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 8. homtrans (apply homogeneous transformation)
% -------------------------------------------------------------------------

for i = 1:1:x
    transl_result(1:3, 1:3) = rotx_result;
    tic;
    transform_result = homtrans(transl_result, B_p);
    matlab_raw_robotics_transform_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_transform_data[16] = {');

temp_transform = transpose(transform_result);
for z = 1:1:4

    fprintf(results_file, '%.9f', temp_transform(z));

    if (z < 4)
            fprintf(results_file, ',');
    end
end

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 9. inv(T) - Inverse of an homogeneous transformation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    inv_transform_result = inv(transl_result);
    matlab_raw_robotics_inv_transform_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_inv_transform_data[16] = {');

temp_inv_transform = transpose(inv_transform_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_inv_transform(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 10. quaternion addition
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_add_result = q1+q1;
    matlab_raw_robotics_quat_add_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_add_data[4] = {%.9f, %.9f, %.9f, %.9f};\n'...
    ,quat_add_result.s, quat_add_result.v(1), quat_add_result.v(2), quat_add_result.v(3));

% -------------------------------------------------------------------------
% 11. quaternion substraction
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_sub_result = q1-q1;
    matlab_raw_robotics_quat_sub_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_sub_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    quat_sub_result.s, quat_sub_result.v(1), quat_sub_result.v(2), quat_sub_result.v(3));

% -------------------------------------------------------------------------
% 12. quaternion*scalar multiplication
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_scale_result = q1*scalar;
    matlab_raw_robotics_quat_scale_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_scale_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    quat_scale_result.s, quat_scale_result.v(1), quat_scale_result.v(2), quat_scale_result.v(3));

% -------------------------------------------------------------------------
% 13. quaternion*quaternion multiplication
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_mul_result = q1*q1;
    matlab_raw_robotics_quat_mul_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_mul_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    quat_mul_result.s, quat_mul_result.v(1), quat_mul_result.v(2), quat_mul_result.v(3));

% -------------------------------------------------------------------------
% 14. quaternion conjugate
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_conj_result = q1.conj();
    matlab_raw_robotics_quat_conj_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_conj_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    quat_conj_result.s, quat_conj_result.v(1), quat_conj_result.v(2), quat_conj_result.v(3));

% -------------------------------------------------------------------------
% 15. quaternion norm
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_norm_result = q1.norm;
    matlab_raw_robotics_quat_norm_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_norm_data = %.9f;\n', quat_norm_result);
% -------------------------------------------------------------------------
% 16. unit quaternion inverse
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    quat_inv_result = q1.inv;
    matlab_raw_robotics_quat_inv_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_quat_inv_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    quat_inv_result.s, quat_inv_result.v(1), quat_inv_result.v(2), quat_inv_result.v(3));

% -------------------------------------------------------------------------
% 17. r2t - rotation matrix to homogeneous transformation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    r2t_result = r2t(rotx_result);
    matlab_raw_robotics_r2t_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_r2t_data[16] = {');

temp_r2t = transpose(r2t_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_r2t(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 18. t2r - homogeneous transformation matrix to rotation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    t2r_result = t2r(trotx(theta));
    matlab_raw_robotics_t2r_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_t2r_data[9] = {');

temp_t2r = transpose(t2r_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_t2r(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 19. rpy2r - roll-pitch-yaw angles to rotation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rpy2r_result = rpy2r(roll, pitch, yaw, 'xyz');
    matlab_raw_robotics_rpy2r_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_rpy2r_data[9] = {');

temp_rpy2r = transpose(rpy2r_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_rpy2r(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 20. rpy2tr - roll-pitch-yaw to homogeneous transformation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rpy2tr_result = rpy2tr(roll, pitch, yaw, 'xyz');
    matlab_raw_robotics_rpy2tr_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_rpy2tr_data[16] = {');

temp_rpy2tr = transpose(rpy2tr_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_rpy2tr(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 21. rt2tr - rotation matrix and translation vector to homogeneous trans.
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rt2tr_result = rt2tr(rotx_result, v);
    matlab_raw_robotics_rt2tr_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_rt2tr_data[16] = {');

temp_rt2tr = transpose(rt2tr_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_rt2tr(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 22. eul2r - euler angles to rotation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    eul2r_result = eul2r(phi, theta, psi);
    matlab_raw_robotics_eul2r_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_eul2r_data[9] = {');

temp_eul2r = transpose(eul2r_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_eul2r(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 23. eul2tr - euler angles to homogeneous transformation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    eul2tr_result = eul2tr(phi, theta, psi, 'xyz');
    matlab_raw_robotics_eul2tr_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_eul2tr_data[16] = {');

temp_eul2tr = transpose(eul2tr_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_eul2tr(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 24. r2q - homogeneous transformation matrix to quaternion
% -------------------------------------------------------------------------

    % UnitQuaternion.tr2q accepts both a rotation matrix and an homogenous
    % transformation matrix, so it's the same function for this test (r2q)
    % and for the next one (25. tr2q)

for i = 1:1:x
    tic;
    [r2q_s, r2q_v] = uq1.tr2q(rotx_result);
    matlab_raw_robotics_r2q_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_r2q_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    r2q_s, r2q_v(1), r2q_v(2), r2q_v(3));

% -------------------------------------------------------------------------
% 25. tr2q - homogeneous transformation matrix to quaternion
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    [tr2q_s, tr2q_v] = uq1.tr2q(transl_result);
    matlab_raw_robotics_tr2q_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_tr2q_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    tr2q_s, tr2q_v(1), tr2q_v(2), tr2q_v(3));

% -------------------------------------------------------------------------
% 26. rpy2q - roll-pitch-yaw to quaternion
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    rpy2q_result = uq1.rpy(roll,pitch,yaw,'xyz');
    matlab_raw_robotics_rpy2q_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_rpy2q_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    rpy2q_result.s, rpy2q_result.v(1), rpy2q_result.v(2), rpy2q_result.v(3));

% -------------------------------------------------------------------------
% 27. eul2q - euler angles to quaternion
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    eul2q_result = uq1.eul(phi,theta,psi);
    matlab_raw_robotics_eul2q_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_eul2q_data[4] = {%.9f, %.9f, %.9f, %.9f};\n',...
    eul2q_result.s, eul2q_result.v(1), eul2q_result.v(2), eul2q_result.v(3));

% -------------------------------------------------------------------------
% 28. tr2rpy - homogeneous transformation matrix to roll-pitch-yaw angles
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    tr2rpy_result = tr2rpy(rpy2tr_result, 'xyz');
    matlab_raw_robotics_tr2rpy_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_tr2rpy_roll = %.9f;\n', tr2rpy_result(1));
fprintf(results_file, 'float R_tr2rpy_pitch = %.9f;\n', tr2rpy_result(2));
fprintf(results_file, 'float R_tr2rpy_yaw = %.9f;\n', tr2rpy_result(3));

% -------------------------------------------------------------------------
% 29. tr2eul - homogeneous transformation matrix to euler angles
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    tr2eul_result = tr2eul(rpy2tr_result);
    matlab_raw_robotics_tr2eul_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_tr2eul_phi = %.9f;\n', tr2eul_result(1));
fprintf(results_file, 'float R_tr2eul_theta = %.9f;\n', tr2eul_result(2));
fprintf(results_file, 'float R_tr2eul_psi = %.9f;\n', tr2eul_result(3));

% -------------------------------------------------------------------------
% 30. q2r - quaternion to rotation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    q2r_result = UnitQuaternion.q2r([uq1.s, uq1.v(1), uq1.v(2), uq1.v(3)]');
    matlab_raw_robotics_q2r_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_q2r_data[9] = {');

temp_q2r = transpose(q2r_result);
for z = 1:1:9

    fprintf(results_file, '%.9f', temp_q2r(z));

    if (z < 9)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 31. q2tr - quaternion to homogeneous transformation matrix
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    q2tr_result = uq1.T();
    matlab_raw_robotics_q2tr_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_q2tr_data[16] = {');

temp_q2tr = transpose(q2tr_result);
for z = 1:1:16

    fprintf(results_file, '%.9f', temp_q2tr(z));

    if (z < 16)
            fprintf(results_file, ',');
    end
end    

fprintf(results_file, '};\n');

% -------------------------------------------------------------------------
% 32. q2rpy - quaternion to roll-pitch-yaw angles
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    q2rpy_result = uq1.torpy();
    matlab_raw_robotics_q2rpy_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_q2rpy_roll = %.9f;\n', q2rpy_result(1));
fprintf(results_file, 'float R_q2rpy_pitch = %.9f;\n', q2rpy_result(2));
fprintf(results_file, 'float R_q2rpy_yaw = %.9f;\n', q2rpy_result(3));

% -------------------------------------------------------------------------
% 33. q2eul - quaternion to euler angles
% -------------------------------------------------------------------------

for i = 1:1:x
    tic;
    q2eul_result = uq1.toeul();
    matlab_raw_robotics_q2eul_time(i,1) = toc;
end
 
% Print results to .h
fprintf(results_file, 'float R_q2eul_phi = %.9f;\n', q2eul_result(1));
fprintf(results_file, 'float R_q2eul_theta = %.9f;\n', q2eul_result(2));
fprintf(results_file, 'float R_q2eul_psi = %.9f;\n', q2eul_result(3));

% -------------------------------------------------------------------------
% Save time results into a .mat file for plotting in a new file
% -------------------------------------------------------------------------

save matlab_raw_robotics_time_results.mat...
    matlab_raw_robotics_transl_time matlab_raw_robotics_rotx_time matlab_raw_robotics_roty_time matlab_raw_robotics_rotz_time...
    matlab_raw_robotics_trotx_time matlab_raw_robotics_troty_time matlab_raw_robotics_trotz_time matlab_raw_robotics_transform_time...
    matlab_raw_robotics_inv_transform_time matlab_raw_robotics_quat_add_time matlab_raw_robotics_quat_sub_time...
    matlab_raw_robotics_quat_scale_time matlab_raw_robotics_quat_mul_time matlab_raw_robotics_quat_conj_time...
    matlab_raw_robotics_quat_norm_time matlab_raw_robotics_quat_inv_time matlab_raw_robotics_r2t_time matlab_raw_robotics_t2r_time...
    matlab_raw_robotics_rpy2r_time matlab_raw_robotics_rpy2tr_time matlab_raw_robotics_rt2tr_time matlab_raw_robotics_eul2r_time...
    matlab_raw_robotics_eul2tr_time matlab_raw_robotics_r2q_time matlab_raw_robotics_tr2q_time matlab_raw_robotics_rpy2q_time matlab_raw_robotics_eul2q_time...
    matlab_raw_robotics_tr2rpy_time matlab_raw_robotics_tr2eul_time matlab_raw_robotics_q2r_time...
    matlab_raw_robotics_q2tr_time matlab_raw_robotics_q2rpy_time matlab_raw_robotics_q2eul_time