
function [U,S,V] = one_sided_jacobi_svd(A)

    [m,n] = size(A); % Number of cols

    V = eye(n); % Initialize V

    % Execute one_sided_jacobi for to get AV = US and V
    for i = 1:10
        for j = 1:n-1
            for k = j+1:n
            [A,V] = one_sided_jacobi(A,V,j,k);
            end
        end
    end

    % Extract S and U
    U = zeros(m,n);
    S = zeros(n);

    for j = 1:n
        sigma = norm(A(:,j));
        S(j,j) = sigma;

        if S(j,j) < 1E-7
            U(:,j) = zeros(m,1);
        else
            U(:,j) = A(:,j)/sigma;
        end  
    end
end