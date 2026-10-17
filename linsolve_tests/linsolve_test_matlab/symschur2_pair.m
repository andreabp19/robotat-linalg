
function [c,s] = symschur2_pair(A,p,q)

    if A(p,q) ~= 0

        tau = (A(q,q) - A(p,p))/(2*A(p,q));

        if tau >= 0
            t = 1/(tau + sqrt(1 + (tau*tau)));

        else
            t = 1/(tau - sqrt(1 + (tau*tau)));

        end
        
        c = 1/sqrt(1 + (t*t));
        s = t*c;
    
    else
        c = 1;
        s = 0;

    end
end