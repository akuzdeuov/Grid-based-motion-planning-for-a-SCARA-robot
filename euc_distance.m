%function euc_d = euc_distance(x1, y1, x2, y2)
%returns euclidean distance between two nodes
function euc_d = euc_distance(x1, y1, x2, y2)
euc_d = sqrt((x2-x1)^2+(y2-y1)^2);
end