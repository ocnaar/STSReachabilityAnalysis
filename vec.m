function column_vector = vec(matrix)
% Reshape matrix into column vector. 
column_vector = reshape(matrix,numel(matrix),1);