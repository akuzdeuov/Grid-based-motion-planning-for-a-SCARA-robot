%function out = new_node(x, y, xp, yp, xg, yg, ph)
%x and y - row and column of a given node
%xp and yp - row and column of a parent node of the given node
%xg and yg - row and column of a goal node
%ph - cost of the parent node
function out = new_node(x, y, xp, yp, xg, yg, ph)
%cost of travelling to the given node
g = ph + euc_distance(x,y,xp,yp);
%heuristic between the given node and the goal node
h = heuristic(x,y,xg,yg);
f = g + h;
out = [x, y, xp, yp, g, h, f];
end