
function [Ad,Bd] = ctr_c2d(Ac,Bc,sample_time,method)

    n = length(Ac); % A is square
    I = eye(n);

    switch (method)
        case 'ForwardEuler'
            Ad = I + (Ac*sample_time);
            Bd = Bc*sample_time;

        case 'BackwardEuler'
            Ad = (I - (Ac*sample_time))^-1;
            Bd = Ad * Bc*sample_time;

        case 'Tustin'
            I_Asample_time = (I - (Ac*sample_time/2))^-1;
            Ad = I_Asample_time * (I + Ac*sample_time/2);
            Bd = I_Asample_time * (Bc*sample_time/2);
    end
end