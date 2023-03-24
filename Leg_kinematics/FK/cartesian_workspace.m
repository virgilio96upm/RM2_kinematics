% Define the range of joint angles
num_point = 250;
q1_range = linspace(deg2rad(-160), deg2rad(-230), num_point);
q2_range = linspace(deg2rad(-25.4), deg2rad(15), num_point);
q3_range = linspace(deg2rad(-160), deg2rad(-230), num_point);

% Initialize the end effector positions
x = zeros(length(q1_range)*length(q2_range)*length(q3_range), 1);
y = zeros(length(q1_range)*length(q2_range)*length(q3_range), 1);
x1 = zeros(length(q1_range)*length(q2_range)*length(q3_range), 1);
y1 = zeros(length(q1_range)*length(q2_range)*length(q3_range), 1);
phi = zeros(length(q1_range)*length(q2_range)*length(q3_range), 1);
R = 78.8;

% Calculate the end effector positions for all combinations of joint angles
k = 1;
for i = 1:length(q1_range)
    for j = 1:length(q2_range)
        for l = 1:length(q3_range)
            [x(k), y(k), phi(k)] = dir_kin(q1_range(i), q2_range(j), q3_range(l));
            x1(k) = x(k)- R*cosd(phi(k));
            y1(k) = y(k)- R*sind(phi(k));
            k = k + 1;
        end
    end
end

% Plot the end effector positions
figure;
scatter(x, y, '.', 'MarkerEdgeColor', 'b');
% hold on
% scatter(x1, y1, '.', 'MarkerEdgeColor', 'r');
xlabel('x (mm)');
ylabel('y (mm)');
title('Workspace of the Mechanism');
hold off

%% Function
function [x,y,phi] = dir_kin(q1,q2,q3)
    %Constants
    D_1 = 33.8;
    D_2 = 45;
    L1 = 38.5;
    L2 = 55;
    L3 = L1;
    l1 = 42;
    l2 = 60;
    l3 = l1;
    R = 78.8;
    
    L1x = L1*cos(q1);
    L1y = L1*sin(q1);
    L2x = L2*cos(q2);
    L2y = L2*sin(q2);
    L3x = L3*cos(q3);
    L3y = L3*sin(q3);
    
    d2 = -2*(D_1 + L2x);
    d3 = -2*(D_1+D_2 + L3x);
    e2 = -2*L2y;%POSIBLEMENTE SE PUEDA ELIMINAR VARIABLE INTERMEDIA
    e3 = -2*L3y;%POSIBLEMENTE SE PUEDA ELIMINAR VARIABLE INTERMEDIA
    f2 = (D_1 + L2x)^2 + (L2y)^2 - l2^2;
    f3 = (D_1+D_2 + L3x)^2 + (L3y)^2 - l3^2;

    u1 = (d3-d2)^2 + (e3-e2)^2;
    v1 = 2*(f3-f2)*(e3-e2) - (d2*(d3-d2)*(e3-e2)) + e2*(d3-d2)^2;
    w1 = (f3-f2)^2 - (d2*(f3-f2)*(d3-d2)) + f2*(d3-d2)^2;

    if (4*u1*w1)>v1^2
        x = 0;
        y = 0;
        phi = 0;
    else
        B_y = (-v1 + sqrt(v1^2 - 4*u1*w1))/(2*u1); % y coordinate of point P
        B_x = -((e3-e2)*B_y + (f3-f2))/((d3-d2)); % x coordinate of point P

        u2 = -2*R*(B_x-L1x);
        v2 = -2*R*(B_y-L1y);
        w2 = (B_x-L1x)^2 + (B_y-L1y)^2 - l1^2 + R^2;
        if (v2^2 + u2^2)<= w2^2
            x = 0;
            y = 0;
            phi = 0;
        else
        t = (-v2 - sqrt(v2^2 - w2^2 + u2^2))/(w2-u2);
        phi_rad = 2*atan(t); % orientation of the end effector

        % coordinates of point B
        x = B_x;
        y = B_y; 
        phi = rad2deg(phi_rad);
        end
    end
   
    % coordinates of the end effector
%     x = B_x - R*cos(phi_rad)/2;
%     y = B_y - R*sin(phi_rad)/2;
%     phi = rad2deg(phi_rad);
end
