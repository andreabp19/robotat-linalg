
function [U,S,V] = one_sided_jacobi_svd(A)

    [m,n] = size(A);

    [AV,V] = one_sided_jacobi(A);
    
    U = zeros(n);
    S = zeros(n);

    for j = 1:n
        S(j,j) = norm(AV(:,j));
        if S(j,j) < 1E-05
            S(j,j) = 0;
        else
            U(:,j) = AV(:,j)./S(j,j);
        end    
    end
end