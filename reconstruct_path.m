%function total_path = reconstruct_path(close, current, qs)
%reconstructs the shortest path using parent nodes.
%current - current node
%qs - start node
%close - close list
function total_path = reconstruct_path(close, current, qs)
i = 1;
while (current(1)~=qs(1) || current(2) ~= qs(2))
    ind = get_index(close, current(1),current(2));
    current = close(ind, 3:4);
    total_path(i,:) = current;
    i = i + 1;
end
end