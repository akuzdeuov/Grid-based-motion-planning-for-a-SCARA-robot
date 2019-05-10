% function out = node_successor(x, y, xg, yg, r, c, g, map, close)
% returns neighbour nodes of a given node
% x and y - row and column of a given node
% xg and yg - row and column of a goal node
% r and c - total number of rows and columns of the grid
% map - grid matrix
% close list
function out = node_successor(x, y, xg, yg, r, c, g, map, close)
%set counter
count=1;
%empty neighbours list
neighbours = [];
for ind1= 1:-1:-1
    for ind2= 1:-1:-1
        if (ind1~=ind2 || ind1~=0)  %The node itself is not its successor, skip it
            %if neighbour node is in the close list then skip it
            if ~isempty(close) && get_index(close,x+ind1,y+ind2)
                continue;
            end
            % if neighbours are not outside of the grid and not in obstacle
            % space
            if ((x+ind1 >0 && x+ind1 <= r) && (y+ind2 >0 && y+ind2 <=c) && map(x+ind1,y+ind2)==0)
                neighbours(count,1) = x + ind1; % row of the neighbour 
                neighbours(count,2) = y + ind2; % column of the neighbour
                neighbours(count,3) = x; % row of the parent
                neighbours(count,4) = y; % column of the parent
                neighbours(count,5) = g + euc_distance(x,y,neighbours(count,1),neighbours(count,2));%cost of travelling to node
                neighbours(count,6) = heuristic(xg,yg,neighbours(count,1),neighbours(count,2));%distance between node and goal
                neighbours(count,7) = neighbours(count,5)+neighbours(count,6);%fn
                count=count+1;
            end
        end
    end
end
%if neighbours are empty then return 0 else return neighbours matrix
if isempty(neighbours)
    out = 0;
else
    out = neighbours;
end
end