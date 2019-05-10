%function out = invert_matrix(matrix)
%inverts Mx2 matrix, [x1 y1; x2 y2;...;xn yn]
% -> [xn yn; xn-1 yn-1;...;x1 y1]
function out = invert_matrix(matrix)
i = 1;
for ind = length(matrix(:,1)):-1:1
    out(i,1) = matrix(ind,1);
    out(i,2) = matrix(ind,2);
    i = i+1;
end
end