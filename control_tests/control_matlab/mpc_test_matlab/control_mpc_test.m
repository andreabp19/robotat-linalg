
% author: Andrea Pineda
% date: Created: 12 Oct 2025
%       Last Modified: 6 Jan 2026
%
% To test the custom mpc algorithm to be implemented in robotat_control

clear
clc
format long

A = [1.1, 2; 0, 0.95];
B = [0, 0.0787]';
C = [-1, 1];

Q = C'*C;
R = 0.01;
x_k = [1,1]';
N = 4;

% Input constraints
ub = 10;
lb = -10;

% Input constraints
%Ain = [eye(N/2) zeros(N/2); zeros(N/2) -eye(N/2)]
bin = [ub; ub; lb; lb]

% State constraints (bin is updated inside the main loop)
Ain = eye(N)*[A, zeros(size(A)); zeros(size(A)) -A]

u_k = zeros(N,1);
x_k_history = zeros(2*N,2);
u_k_history = zeros(2*N,1);
cost_history = zeros(2*N,1);

mpc_mean_time = 0;

matlab_raw_control_mpc_time = zeros(100, 1);

for i = 1:1:100
    tic;
    % Iterate over the receding horizon
    for k = 1:1:2*N
    
        %fprintf("\n-------------------------\n");
        %fprintf("k = %i\n", k);
        %fprintf("-------------------------\n");
        
        %tic;
        [Qqp,c] = mpc_lti_unconstrained_shooting(A,B,Q,R,x_k,N)
        
        % Use this one for an unrestricted MPC
        %[u_k,x_k] = mpc_lti_update(Qqp,c,x_k,u_k,A,B,0,0)
    
        % Use this one for a restricted MPC
        bin = [ub - B*u_k(1); lb - B*u_k(1)] % state restriction bin
        [u_k,x_k] = mpc_lti_update(Qqp,c,x_k,u_k,A,B,Ain,bin)
    
        %mpc_mean_time = mpc_mean_time + toc;
        
        %x_k_history(k,:) = x_k';
        %u_k_history(k) = u_k(1);
        %cost_history(k) = ((1/2)*u_k'*Qqp*u_k) + (c'*u_k);
    end
    matlab_raw_control_mpc_time(i) = toc;
end



%mpc_mean_time % Print mean time measurement


fprintf("\n-------------------------\n");
u_k_history

fprintf("\n-------------------------\n");
cost_history

save matlab_control_mpc_results x_k_history u_k_history cost_history

save matlab_raw_control_mpc_time_results matlab_raw_control_mpc_time