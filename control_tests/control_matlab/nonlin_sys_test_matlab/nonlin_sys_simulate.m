
function [x_k_1] = nonlin_sys_simulate(f_handler, x_k, u_k, delta, method)

    switch method

        case 'ForwardEuler'
            fss = f_handler(x_k, u_k);
            x_k_1 = x_k + (fss*delta);

        case 'RK4'
            k1 = f_handler(x_k, u_k);
            k2 = f_handler(x_k + k1*delta/2, u_k);
            k3 = f_handler(x_k + k2*delta/2, u_k);
            k4 = f_handler(x_k + delta*k3, u_k);

            x_k_1 = x_k + (delta/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end