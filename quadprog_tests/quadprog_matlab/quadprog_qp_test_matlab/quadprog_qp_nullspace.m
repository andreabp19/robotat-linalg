
function [x] = quadprog_qp_nullspace(G, c, Aeq, beq)

    [m,n] = size(Aeq);

    [Q,~] = qr(Aeq');
    Y = Q(:,1:m);

    if (m == n)
        x = (Aeq*Y)\beq;

    else

        Z = Q(:,m+1:end);
    
        AY = Aeq*Y;
    
        py = AY\beq;
        pz = (Z'*G*Z)\(-Z'*G*Y*py - Z'*c);
    
        x = Y*py + Z*pz;

    end

end