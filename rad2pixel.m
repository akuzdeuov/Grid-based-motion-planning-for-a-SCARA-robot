%function out = rad2pixel(theta, rad)
% theta - row vector with theta values (theta1/theta2)
% rad - radian which will be converted
function out = rad2pixel(theta, rad)
i = 1;
while rad>=theta(i)
    i = i+1;
end
out = i;
end
