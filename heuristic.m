%function dist = heuristic(x1, y1, x2, y2)
%returns euclidean distance between two nodes
%http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
%x and y - row and column of a node
function dist = heuristic(x1, y1, x2, y2)
% D = 1; D2 = sqrt(2);
% dx = abs(x2-x1);
% dy = abs(y2 - y1);
% dist = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
dist = sqrt((x2-x1)^2+(y2-y1)^2);
end