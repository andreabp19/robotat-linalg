
function [u_k,e_k] = pid_update(kp,ki,kd,u_k_1,u_k_2,e_k_1,e_k_2,r_k,y_k,tau,T,pid_alg)

    e_k = r_k - y_k;

    switch pid_alg

        case 'PureDiscrete'
            u_k = kp*e_k + ki*(e_k_1 + e_k) + kd*(e_k - e_k_1);

        case 'ForwardEuler'
            a0 = tau;
            u_k = (kp*tau + ki*tau + kd)*e_k...
                + (kp*(T - 2*tau) + ki*(T - tau) - 2*kd)*e_k_1...
                + (kp*(tau - T) + kd)*e_k_2...
                - (T - 2*tau)*u_k_1...
                - (tau - T)*u_k_2;
            u_k = u_k / a0;
            

        case 'BackwardEuler'
            a0 = T + tau;
            u_k = (kp*(T + tau) + ki*(T + tau) + kd)*e_k...
                + (-kp*(2*tau + T) - ki*tau - 2*kd)*e_k_1...
                + (kp*tau + kd)*e_k_2...
                + (2*tau + T)*u_k_1...
                - (tau)*u_k_2;
            u_k = u_k / a0;

        case 'Tustin'
            a0 = 2*tau + T;
            u_k = (kp*(2*tau + T) + 0.5*ki*(2*tau + T) + 2*kd)*e_k...
                + (-kp*4*tau + 0.5*ki*(-2*tau + T) - 4*kd)*e_k_1...
                + (kp*(2*tau - T) + 2*kd)*e_k_2...
                + (4*tau)*u_k_1...
                - (2*tau - T)*u_k_2;
            u_k = u_k / a0;

    end

end