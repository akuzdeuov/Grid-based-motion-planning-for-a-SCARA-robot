% Course: Optimal Control and Planning (ROBT615)
% Project: Grid-based motion planning for a SCARA robot
% Author: Kuzdeuov Askat
% email: askat.kuzdeuov@nu.edu.kz
% Date: 3rd of September, 2018
%% Functions (a more detailed explanation is given inside of each function)
% 1.constr(l1, l2, th1, th2, xc, yc, B, k) - this function returns boolean
% value. I have chosen four spheres to cover link 2. So, instead of 
% writing long constraints in this main function I call this function. 
% 2.rad2pixel(theta, rad) - this function converts given configuration in
% radians to pixel (row and column of the grid). The returned value is used for A*.
% 3.aStar(qs, qe, map) - A* algorithm which calculates the shortest between start
% position and goal position
% 4.node_successor(x, y, xg, yg, r, c, g, map, close) - returns information
% about neighbours of a given node.
% 5.new_node(x, y, xp, yp, xg, yg, ph) - inserts new node to a list. 
% 6.euc_distance(x1, y1, x2, y2) - returns euclidean distance between two
% nodes
% 7.heuristic(x1, y1, x2, y2) - returns euclidean distance between two
% nodes. 
% 8.get_index(list, xi, yi) - returns index of a row which contains both xi and yi in a list
% 9.pixel2rad(path,theta1, theta2) - converts row and column values to
% radians
% 10.invert_matrix(matrix) - inverts Mx2 matrix, [x1 y1; x2 y2;...;xn yn]
% -> [xn yn; xn-1 yn-1;...;x1 y1]
% 11.reconstruct_path(close, current, qs) - reconstructs the shortest path
% using parent nodes.

%% prepare the workspace
clear; clc; close all;
%% 1.Initialization 
l1 = 0.5; l2 = 0.4; % length of each links (m)
d_wall = -0.1; % distance from the base of the robot to the wall (m)
qs = [0.5 -0.5]'; % q_start (rad) -pi/8<theta1<9pi/8, -pi/2<theta2<pi/2
qe = [2 -1]'; % q_end (rad)
B = 0.2;  % radius of the circle around obstacles (m)
xc1 = -0.6; yc1 = 0.7; xc2 = 0.6; yc2 = 0.7; % coordinates of the centers of the obstacles (m)
Ti = 2; % motion time (sec)
%% 2. Definition and representation of free space and obstacle space
theta1 = -pi/8:0.0125:9*pi/8; % range for theta 1, -pi/8<theta1<9pi/8, sampling rate 0.0125 rad
theta2 = -pi/2:0.01:pi/2; % range for theta 2, -pi/2<theta2<pi/2, sampling rate 0.01 rad
[X,Y] = meshgrid(theta1,theta2);
map = ones(size(X)); % create initial grid with ones
% check for obstacles, 
% if a point is in free space then it is equal to 0 else to 1
for ind1=1:length(theta1)
    for ind2=1:length(theta2)
        if l1*sin(X(ind1,ind2))>=d_wall && l1*sin(X(ind1,ind2))+l2*sin(X(ind1,ind2)+Y(ind1,ind2))>=d_wall ... 
                && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc1,yc1,B,1/8) && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc1,yc1,B,3/8)... 
                && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc1,yc1,B,5/8) && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc1,yc1,B,7/8)...
                && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc2,yc2,B,1/8) && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc2,yc2,B,3/8)...
                && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc2,yc2,B,5/8) && constr(l1,l2,X(ind1,ind2),Y(ind1,ind2),xc2,yc2,B,7/8)
            map(ind1,ind2) = 0;
        end
    end
end
% plot free and obstacle space
figure(1)
contourf(X,Y,map); 
xlabel('\theta1 (rad)'); ylabel('\theta2 (rad)'); 
title('Free (blue) and obstacle (yellow) space');
grid on
%% 3. Graph definition and graph search
% convert radian values in qs and qe to pixel values
% in order to give them as an input to A* algorithm
qss = [rad2pixel(theta2, qs(2)) rad2pixel(theta1, qs(1))]; 
qee = [rad2pixel(theta2, qe(2)) rad2pixel(theta1, qe(1))];
% run A* algorithm and return visited nodes
path = aStar(qss, qee, map);
% reconstruct the shortest path from visited nodes
short_path = reconstruct_path(path, qee, qss);
% plot the grid with drawn shortest path
figure(2);
contourf(X,Y,map);
text(qs(1), qs(2), 1, 'START', 'Color', 'r', 'FontSize',12);
text(qe(1), qe(2), 1, 'END', 'Color', 'r', 'FontSize',12);
xlabel('\theta1 (rad)'); ylabel('\theta2 (rad)');
title('The shortest path (green), visited nodes (white)');
grid on
hold on
%convert path values to radians
p = pixel2rad(path, theta1, theta2);
plot(p(:,1), p(:,2),'w*');
hold on
%convert short path values to radians
sh_p = pixel2rad(short_path, theta1, theta2);
plot(sh_p(:,1), sh_p(:,2),'-g','LineWidth',2);
hold off
%% 4. Trajectory generation
% convert row and column values to radians 
rads = pixel2rad(short_path,theta1,theta2);
% path returned values from goal position to start position
% so we invert matrix in order to have from start to goal
q = invert_matrix(rads);
%time intervals between two nodes
t = 0:Ti/(length(q(:,1))-1):Ti;
ind = 1;
while ind<=length(q(:,1))  
    if ind == 1 || ind==length(q(:,1))
        qt(ind,:) = q(ind,:); % angular positions (rad) at the beginning and at the end
        qt_d(ind,:) = [0 0]; % angular velocities (rad/sec) at the beginning and at the end
    else 
        %cubic interpolation        
        qt(ind,:) = (qt(ind-1,:)'+(3*t(ind)^2/Ti^2 - 2*t(ind)^3/Ti^3)*(q(ind,:)'-qt(ind-1,:)')); % angular positions (rad)
        qt_d(ind,:) = ((6*t(ind)/Ti^2 - 6*t(ind)^2/Ti^3))*(qt(ind,:)-qt(ind-1,:));  % angular velocities (rad/sec)
    end
    ind = ind+1;
end

% plot angular positions with respect to time
figure(3);
subplot 121, 
plot(t,qt(:,1),'r');
hold on
plot(t,qt(:,2),'b');
xlabel('time (sec)');
ylabel('\theta (rad)');
legend('\theta1 (rad)','\theta2 (rad)');
title('Angular position (rad)');
grid on
hold off
% plot angular velocities with respect to time
subplot 122, 
plot(t,qt_d(:,1),'r');
hold on
plot(t,qt_d(:,2),'b');
xlabel('time (sec)');
ylabel('\theta''');
hLeg = legend('$$\dot{\theta1}$$', '$$\dot{\theta2}$$');
set(hLeg,'Interpreter','latex');
title('Angular velocity (rad/sec)');
grid on
hold off

%% visualization
% angle vector for drawing circles  around obstacles
ang=0:0.01:2*pi; 
% x and y coordinates for drawing 4 circles with radius l2/8 at link2 
xl2=l2*cos(ang)/8; 
yl2=l2*sin(ang)/8;
%x and y cooridnates for drawing obstacles represented as a circle
xp=B*cos(ang);
yp=B*sin(ang);
figure(4)
for tt=1:length(qt(:,1))
    %set axis of the plot
    axis([-1 1 -1 1]);
    %plot the first circle obstacle
    plot(xc1+xp,yc1+yp,'Color', 'm', 'LineWidth',2);
    hold on
    %plot the second circle obstacle
    plot(xc2+xp,yc2+yp,'Color', 'm', 'LineWidth',2);
    hold on
    %plot the wall obstacle
    plot([-1, 1], [-0.1, -0.1], 'Color', 'm','LineWidth',2);
    hold on
    axis([-1 1 -1 1]);
    %plot link 1, if it is in free space then its color is green, else red 
    if l1*sin(qt(tt,1))>=d_wall
        plot([0, l1*cos(qt(tt,1))], [0, l1*sin(qt(tt,1))], 'Color', 'g','LineWidth',4);
    else
        plot([0, l1*cos(qt(tt,1))], [0, l1*sin(qt(tt,1))], 'Color', 'r','LineWidth',4);
    end
    hold on
    axis([-1 1 -1 1]);
    %plot link 2 and four circles at link 2 
    %if they are in free space then their color is blue, else red 
    if l1*sin(qt(tt,1))+l2*sin(qt(tt,1)+qt(tt,2))>=d_wall ... 
                && constr(l1,l2,qt(tt,1),qt(tt,2),xc1,yc1,B,1/8) && constr(l1,l2,qt(tt,1),qt(tt,2),xc1,yc1,B,3/8)... 
                && constr(l1,l2,qt(tt,1),qt(tt,2),xc1,yc1,B,5/8) && constr(l1,l2,qt(tt,1),qt(tt,2),xc1,yc1,B,7/8)...
                && constr(l1,l2,qt(tt,1),qt(tt,2),xc2,yc2,B,1/8) && constr(l1,l2,qt(tt,1),qt(tt,2),xc2,yc2,B,3/8)...
                && constr(l1,l2,qt(tt,1),qt(tt,2),xc2,yc2,B,5/8) && constr(l1,l2,qt(tt,1),qt(tt,2),xc2,yc2,B,7/8)
        plot([l1*cos(qt(tt,1)), l1*cos(qt(tt,1))+l2*cos(qt(tt,1)+qt(tt,2))], [l1*sin(qt(tt,1)), l1*sin(qt(tt,1))+l2*sin(qt(tt,1)+qt(tt,2))], 'Color', 'b','LineWidth',4);
        hold on
        plot(((l1*cos(qt(tt,1)))+1/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+1/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'b');
        hold on
        plot(((l1*cos(qt(tt,1)))+3/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+3/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'b');
        hold on
        plot(((l1*cos(qt(tt,1)))+5/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+5/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'b');
        hold on
        plot(((l1*cos(qt(tt,1)))+7/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+7/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'b');
    else
        plot([l1*cos(qt(tt,1)), l1*cos(qt(tt,1))+l2*cos(qt(tt,1)+qt(tt,2))], [l1*sin(qt(tt,1)), l1*sin(qt(tt,1))+l2*sin(qt(tt,1)+qt(tt,2))], 'Color', 'r','LineWidth',4);
        hold on
        plot(((l1*cos(qt(tt,1)))+1/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+1/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'r');
        hold on
        plot(((l1*cos(qt(tt,1)))+3/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+3/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'r');
        hold on
        plot(((l1*cos(qt(tt,1)))+5/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+5/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'r');
        hold on
        plot(((l1*cos(qt(tt,1)))+7/8*l2*cos(qt(tt,1)+qt(tt,2)))+xl2,(l1*sin(qt(tt,1)))+7/8*l2*sin(qt(tt,1)+qt(tt,2))+yl2, 'Color', 'r');
    end
    xlabel('x'); ylabel('y');
    grid on
    pause(0.1)
    hold off
end

