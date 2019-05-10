%function path = aStar(qs, qe, map)
% qs - start configuration 
% qe - goal configuration
% map - free and obstacle space grid
function path = aStar(qs, qe, map)
%% check whether start and goal position are chosen appropriately
if map(qs(1),qs(2))==1 || map(qe(1),qe(2))==1
    disp('Start or Goal inside of an Obstacle');
    path = Inf;
    return;
elseif qs == qe
    disp('We are already at the goal position');
    return
end
%% initialization
% rows and columns of the map
[r,c] = size(map);
xs = qs(1); ys = qs(2); %start index
xg = qe(1); yg = qe(2); %goal index
%#-----------------------------------
%OPEN_LIST|x|y|xp|yp|g|h|f|
%contains node which successors are not visited
%x and y are row and column of current node  
%xp and yp are row and column of the parent node 
%g - cost of the travelling to the current node
%h - heuristic from the current node to goal node
%f = g + h
open = [];
%#-----------------------------------
%CLOSE_LIST|x|y|xp|yp|g|h|f|
%contains node which successors are visited
close = [];
%#add starting point to the open list
open(1,:) = new_node(xs, ys,  xs,  ys, xg, yg, 0);
%#obstacles
% obs = obstacles(map);
%% start of the algorithm
flag = 0;
while(~isempty(open) && flag == 0)
    disp('Search...');
    [value, index] = min(open(:,7)); %pick a node with the lowest f in the open list
    % get neighbour nodes of the current node
    node_suc = node_successor(open(index,1), open(index,2), xg, yg, r, c, open(index,5), map, close);
    if node_suc == 0
        close = [close; open(index,:)];
        open(index,:) = [];
        continue
    end
    % for each neighbour node
    for ind = 1:length(node_suc(:,1))
        % if the neighbour is the goal then path is found
        if node_suc(ind, 1)==xg && node_suc(ind,2) == yg
            disp('Path is found!!!');
            flag = 1;
            break;
        end
        % get cost of travelling from the start to the neighbour node
        current_cost = node_suc(ind,5);
        % if the neighbour in the open list and its cost is 
        % more than existing node then skip it else replace the existing
        % node with this neighbour node
        % else if neighbor node not in the open list then add it
        if get_index(open, node_suc(ind,1), node_suc(ind,2))
            iind = get_index(open, node_suc(ind,1), node_suc(ind,2));
            if open(iind,5) < current_cost
                continue;
            end
            open(iind,:) = node_suc(ind,:);
        else
            open = [open; node_suc(ind,:)];            
        end
    end
    % add current node to close node
    close = [close; open(index,:)];
    % if goal position is found then add it to the close list
    if flag==1
        close = [close; new_node(xg,  yg, open(index,1), open(index,2),  xg, yg, open(index,5))];
    end
    % delete current node from the open list
    open(index,:) = [];
end
% if path is found then reconstruct the shortest path
path = close;%reconstruct_path(close, qe, qs);
end