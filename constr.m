% function out = constr(l1, l2, th1, th2, xc, yc, B, k)
% l1 - length of link 1
% l2 - length of link 2
% th1 - theta 1
% th2 - theta 2
% xc, yc - coordinates of an spherical obstacle
% B - radius of the obstacle
% k - coefficient which divides the link 2 to segment
function out = constr(l1, l2, th1, th2, xc, yc, B, k)
out = false;
if ((l1*cos(th1)+k*l2*cos(th1+th2)-xc)^2+(l1*sin(th1)+k*l2*sin(th1+th2)-yc)^2)>=(1/8*l2+B)^2
    out = true;
end
end