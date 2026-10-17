
% Based on python algorithm by wlav-svennnik (in Github): bunchparlett
% which is also based on Golub, Matrix Computations for Bunch-Parlett LDL'
function [P,L,D] = quadprog_qp_ldlt(A)

    n = length(A); % m == n as A is always square

    alpha = (1 + sqrt(17))/8 % As Bunch and Parlett say

    mu0 = 0;
    mu1 = 0;
    P = eye(n);

    % Identify indices
    for i = 1:n
        for j = 1:n
            if abs(A(i,j)) > mu0
                mu0 = abs(A(i,j));
                exi = i;
                exj = j;
            end
        end
    end

    for i = 1:n
        if abs(A(i,i)) > mu1
            mu1 = abs(A(i,i));
            exii = i;
        end
    end

    % Generate permutation matrix
    if mu1 >= alpha*mu0
        s = 1;
        Pii_row = P(exii,:); % Save row exii
        P(exii,:) = P(1,:);
        P(1,:) = Pii_row;

    else
        s = 2;
        Pi_row = P(exi,:); % Save row exi
        Pj_row = P(exj,:); % Save row exj
        P(exi,:) = P(1,:);
        P(exj,:) = P(2,:);
        P(1,:) = Pi_row;
        P(2,:) = Pj_row;
    end

    % Calculate PAP'
    Aprime = P*A*P';

    % Extract submatrices of PAP' = [E C'; C B]
    E = Aprime(1:s,1:s);
    C = Aprime(s+1:end,1:s);
    B = Aprime(s+1:end,s+1:end);

    % Generate lower triangle matrix L and diagonal matrix D
    L = [eye(s) zeros(s,n-s); C*(E^-1) eye(n-s)];
    D = [E zeros(s,n-s); zeros(n-s,s) (B-(C*(E^-1))*C')];
end
