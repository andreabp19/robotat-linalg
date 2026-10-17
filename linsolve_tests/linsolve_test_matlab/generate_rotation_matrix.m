
% Add argument to indicate if it's left or right side rotation, so as to
% adjust the dimensions of the rotation matrix to be generated. For
% example: for right side jacobi rotation J: A*J (m x n)(n x n), and for
% left side it would be J*A (m x m)(m x n).
function [rotation_matrix] = generate_rotation_matrix(matrix, p, q, method, a, b)

    [rows,cols] = size(matrix);
   
    n = cols;
    
    % Generate cosine-sine pair {c,s} based on the selected method
    switch method
        case 'givens'
            [c,s] = givens_pair(a,b);
    
        case 'symschur2'
            [c,s] = symschur2_pair(matrix,p,q);
    end
    
    % Generate identity matrix
    rotation_matrix = eye(n);
    
    % Set cosine and sine values in the rotation matrix
    rotation_matrix(p,p) = c;
    rotation_matrix(p,q) = s;
    rotation_matrix(q,p) = -s;
    rotation_matrix(q,q) = c;
        
end