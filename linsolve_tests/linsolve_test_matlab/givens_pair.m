
function [c,s] = givens_pair(a,b)

    if b == 0
        c = 1;
        s = 0;
    else
        if abs(b) > abs(a)
            tau = -a/b;
            s = 1/sqrt(1 + tau*tau);
            c = s*tau;
        else
            tau = -b/a;
            c = 1/sqrt(1 + tau*tau);
            s = c*tau;
        end
    end
end