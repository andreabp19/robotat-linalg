
function [xhat_prior, P_prior] = kalman_predict(A,B,F,P,Qw,xhat,u)

    % Prediccion
    xhat_prior = A*xhat + B*u;
    P_prior = A*P*A' + F*Qw*F';
    
end