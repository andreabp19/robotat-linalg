
function [u_k_1,x_k_1] = mpc_lti_update(Qqp,c,x_k,u_k,A,B,Ain,bin)
    
    % Solve the QP
    %u_k_1 = active_set_general(Qqp,c,0,0,0,0,u_k);
    
    if (Ain == 0) % Unconstrained problem
        u_k_1 = Qqp\(-c);
    else
        u_k_1 = quadprog_active_set(Qqp,c,Ain,bin,u_k);
    end

    % Compute new trajectory
    u0 = u_k_1(1)
    x_k_1 = A*x_k + B*u_k_1(1);

end