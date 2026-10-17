
% Removes zero values and returns 100 nonzero samples
function data = clean_array(data)

    data = data(data ~= 0);
    data = data(1:100);

end