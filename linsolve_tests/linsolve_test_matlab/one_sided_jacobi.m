
function [AV,V] = one_sided_jacobi(A)

    [rows,cols] = size(A);
    n = cols;

    % V = I (n x n)
    V = eye(n);
    
    delta = 1E-05; % Tolerance

    % Values off diagonal
    AtA = A' * A; % Generate orthonormal matrix A'A
    offDiagA = AtA - diag(diag(AtA)); % Remove diagonal values
    norm_offDiagA2 = norm(offDiagA,'fro')^2; % Calculate frobenius norm

    iterations = 0;

    while sqrt(norm_offDiagA2) > delta
        iterations = iterations + 1;
        for p = 1:n-1
            for q = p + 1:n
                J = generate_rotation_matrix(A,p,q,'symschur2',0,0);
                A = A*J; % One sided Jacobi rotation
                V = V*J; % Save orthogonal vector V

                % offDiagA^2 = offDiagA^2 - a_pq^2 - a_qp^2
                a_pq = A(p,q);
                a_qp = A(q,p);
                norm_offDiagA2 = norm_offDiagA2 - a_pq^2 - a_qp^2;
            end    
        end
    end

    AV = A;
end