

function print_matrix_to_file(results_file, matrix, var_name, var_id)

[rows,cols] = size(matrix);
n = rows;

% This is to save as row-major indexing. Matlab assumens column-major
% indexing, so comment this line if you need column-major.
temp_matrix = transpose(matrix);

fprintf(results_file, '%s%i[%i] = {', var_name, var_id, rows*cols);

for z = 1:(rows*cols)
    fprintf(results_file, '%.9f', temp_matrix(z));
    if z < (rows*cols)
        fprintf(results_file, ',');
    end
end

fprintf(results_file, '};\n');

end