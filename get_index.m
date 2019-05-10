%function index = get_index(list, xi, yi)
%returns index of a row which contains both xi and yi from the list
function index = get_index(list, xi, yi)
ind = 0;
for i=1:length(list(:,1))
    if(list(i,1) == xi && list(i,2) == yi)
        ind = i;
        break;
    end
end
index = ind;
end