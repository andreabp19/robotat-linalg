
% Based on Algorithm 5.4 from Martins, Engineering Design Optimization
% but modified to solve problems with both equality and inequality
% constraints.
function [xhat] = active_set_general(Q,c,Aeq,beq,Ain,bin,x0)

    % Not confuse: c (linear term in the cost function) with Ain (inequality
    % constraints matrix).

    n = size(Ain,1); % Number of rows in the inequality constraints matrix
    x_k = x0;
    epsilon = 1E-06; % Float tolerance? Book doesn't specify value
    
    active_rows = 0; % To count rows if W_k has less than n active rows.
    W_k = zeros(n,1);

    % Initial set W_k = i, for all i where (c_i'*x_k + d_i) < epsilon
    active_index = 1;
    for i = 1:1:n
        % Rows inside the Working Set (active rows)
        if abs(Ain(i,:)*x_k - bin(i,:)) < epsilon
            W_k(active_index) = i; % Save indices of the active rows
            active_rows = active_rows + 1; % Save number of active rows
            active_index = active_index + 1;
        end
    end

    iterations = 0;


    % Solving the Working Set withing a finite number of iterations
    for i = 1:1:30
        iterations = iterations + 1;

        A_w = zeros(active_rows,size(Q,2));   % Matrix
        b_w = zeros(active_rows,1); % Vector

        % Select working set: A_w = Ain_i, b_w = d_i, for all i in W_k
        for j = 1:1:active_rows
            A_w(j,:) = Ain(W_k(j),:);
            b_w(j,:) = bin(W_k(j),:);
        end

        %A_w
        %b_w

        % Solve the KKT system with the Working Set
        [A_w_rows,A_w_cols] = size(A_w);
        [Aeq_rows, Aeq_cols] = size(Aeq);
        [Q_rows, Q_cols] = size(Q);

        M = [ Q,     Aeq',    A_w';
            Aeq,   zeros(Aeq_rows, Aeq_rows),  zeros(Aeq_rows, A_w_rows);
            A_w,   zeros(A_w_rows, Aeq_rows),  zeros(A_w_rows, A_w_rows) ];
        l = [-Q*x_k - c; -Aeq*x_k + beq; -A_w*x_k + b_w];
        p = M\l;
        % Separate p_k and lambda from p
        p_k = p(1:Q_rows);
        lambda_eq = p(Q_rows+1 : Q_rows+Aeq_rows);
        lambda  = p(Q_rows+Aeq_rows+1 : end);
        
        if size(lambda,1) > 0 % If lambda's not empty
            [sigma, sigma_index] = min(lambda);
        else
            sigma = 1.0/0.0; % Similar to quadprog_sqp original code
        end

        %sigma

        %norm(p_k)
        if norm(p_k) < epsilon

            if sigma >= epsilon
                xhat = x_k;
                break;
            else 
                % Remove sigma_row from the Working Set W_k
                if sigma_index < active_rows
                    % Replace sigma_row with the other active constraints
                    W_k(sigma_index:active_rows-1) = W_k(sigma_index+1:active_rows);
                end
                W_k(active_rows) = 0; % Erase new inactive constraint
                active_rows = active_rows - 1; % Reduce active rows count
            end
        else
            alpha = 1; % Initialize with optimum step
            B = 0; % Initialize blocking index with arbitrary value
            
            % Ainheck inactive rows (outside the Working Set)
            for j = 1:n
                if all(W_k(1:active_rows) ~= j)  % Inactive constraints
                    Ajp = Ain(j,:)*p_k;
                    if Ain(j,:)*p_k > epsilon
                        Ajx = bin(j) - Ain(j,:)*x_k;
                        alpha_b = Ajx / Ajp;
                        
                        if alpha_b < alpha
                            alpha = alpha_b;
                            B = j; % Save blocking index
                        end
                    end
                end
            end

            if B > 0
                active_rows = active_rows + 1;
                W_k(active_rows) = B;
            end
            
            x_k = x_k + alpha*p_k;
        end
    end

    xhat = x_k;

end