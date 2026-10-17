
function [A,V] = one_sided_jacobi(A,V,j,k)

    % Important elements of AtA
    a_jj = A(:,j)'*A(:,j);
    a_jk = A(:,j)'*A(:,k);
    a_kk = A(:,k)'*A(:,k);

    if abs(a_jk) > 1E-7

        % Calculate coefficients
        tau = (a_kk - a_jj)/(2*a_jk);
        if tau == 0
            t = 1;
        else
            t = sign(tau)/(abs(tau) + sqrt(1 + tau^2));
        end
        
        % Calculate sine and cosine
        c = 1/sqrt(1 + t^2);
        s = c*t;

        % Apply rotations
        Aj = A(:,j);
        Ak = A(:,k);
        Vj = V(:,j);
        Vk = V(:,k);

        A(:,j) = c*Aj - s*Ak;
        A(:,k) = s*Aj + c*Ak;
        
        V(:,j) = c*Vj - s*Vk;
        V(:,k) = s*Vj + c*Vk;

    end
end