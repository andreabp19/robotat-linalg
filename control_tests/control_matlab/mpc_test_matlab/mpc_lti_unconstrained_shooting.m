
function [Qqp,c] = mpc_lti_unconstrained_shooting(A, B, Q, R, x_k, N)

    % LTI System: A*x_k + B*u_k
    % (only inputs to be controlled, so no output expression)
    %
    % u* = argmin(u) (1/2)*u'*Q*u + c'*u
    % 
    % Q = [C'*Qhat*C + Rhat]*2
    % c = C'*Qhat*x_k
    %
    % C     -> Convolution Matrix
    % Qhat  -> Penalization matrix (the last submatrix is Shat)
    % Rhat  -> Penalization matrix
    % M     -> Horizon length matrix
    %
    % x_k   -> Current state
    % u_k   -> Current input

    n = size(A,1); % Number of rows in A
    m = size(B,2); % Number of columns in B

    % Step 1. Generate C

    C = zeros(N*n,N*m);
    [C_rows,C_cols] = size(C);

    % Assign B to the diagonals of C
    col = 1;
    for row = 1:n:C_rows
        C(row:row+n-1,col) = B;
        col = col + 1;
    end

    % Assign the terms A^i * B to the lower diagonal of C
    pivot = 1;
    for col = 1:1:C_cols-1 % Last column omitted because it's the last diagonal
        exp = 0; % Counter for the exponents
        for row = (pivot+n):n:C_rows % Start below the first B, steps of n rows
            exp = exp + 1;
            C(row:row+n-1,col) = (A^exp) * B;
        end
        pivot = pivot + n;
    end

    % Step 2. Generate Qhat and Rhat
    
    % Get dimensions for Q and Qhat
    Qhat = zeros(N*n,N*n);
    Qhat_size = size(Qhat,1); % It's always square
    [Q_rows,Q_cols] = size(Q);

    % Assign Qhat to diagonals of Qhat, in this case Qbar=Q (last Q in Qhat)
    col = 1;
    for row = 1:Q_rows:Qhat_size
        Qhat(row:row+Q_rows-1,col:col+Q_cols-1) = Q;
        col = col + Q_cols;
    end

    % Get dimensions for R and Rhat
    Rhat = zeros(N*m,N*m);
    Rhat_size = size(Rhat,1); % It's always square

    % Assign Rhat to diagonals of Rhat
    col = 1;
    for row = 1:m:Rhat_size
        Rhat(row:row+m-1,col:row+m-1) = R;
        col = col + m;
    end

    % Step 3. Generate M

    M = zeros(N*n,n);
    exp = 0;
    for row = 1:n:N*n
        exp = exp + 1;
        M(row:row+n-1,1:n) = A^exp;
    end

    % Step 4. Define QP to be solved: (1/2)*u'*2*Q*u + (2*F*x)*u
    Qqp = 2*(C'*Qhat*C + Rhat);
    F = C'*Qhat*M;
    c = 2*F*x_k;
end