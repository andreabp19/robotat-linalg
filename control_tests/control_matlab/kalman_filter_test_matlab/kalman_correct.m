
function [xhat_post,P_post] = kalman_correct(C,Qv,xhat_prior,P_prior,y)

    z_k = y - C*xhat_prior; % Innovation
    S_k = C*P_prior*C' + Qv; % Covarianza de la innovacion
    L_k = P_prior*C' * S_k^(-1);

    % These equations are different from what's used in ctr_kalman_predict
    xhat_post = xhat_prior + L_k*z_k;
    P_post = P_prior - L_k*C*P_prior;
end