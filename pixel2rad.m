%function out =pixel2rad(path,theta1, theta2)
%converts row and column values to radians
%theta1 - a row vector with theta 1 values
%theta2 - a row vector with theta 2 values
%path - obtained shortest path which contains 
%theta 1 and theta 2 coordinates 
function out = pixel2rad(path,theta1, theta2)
for i=1:length(path(:,1))
    out(i,1) = theta1(path(i,2));
    out(i,2) = theta2(path(i,1));
end
end