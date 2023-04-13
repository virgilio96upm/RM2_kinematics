%% FORWARD KINEMATICS %%
% INPUT: Motor input in deg
% OUTPUT: position and orientation in mm and deg , respectively

%test direct kinematics
q1_d = deg2rad(-175);
q2_d = deg2rad(9.6);
q3_d = deg2rad(-195);
[x_1,y_1,phi_1] = dir_kin(q1_d,q2_d,q3_d);

function [x,y,phi] = dir_kin(q1,q2,q3)
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

    B_y = (-v1 + sqrt(v1^2 - 4*u1*w1))/(2*u1); % y coordinate of point P
    if (4*u1*w1)>v1^2
        error("Not feasable pose can be obtained because the given values result in a singularity")
    else
    B_x = -((e3-e2)*B_y + (f3-f2))/((d3-d2)); % x coordinate of point P


    u2 = -2*R*(B_x-L1x);
    v2 = -2*R*(B_y-L1y);
    w2 = (B_x-L1x)^2 + (B_y-L1y)^2 - l1^2 + R^2;

    t = (-v2 - sqrt(v2^2 - w2^2 + u2^2))/(w2-u2);
    if (w2^2 + u2^2)<= v2^2
        error("Not feasable pose can be obtained because the given values result in a singularity")
    else
    phi_rad = 2*atan(t); % orientation of the end effector
    
    % coordinates of the end effector
    x = B_x;
    y = B_y;
    phi = rad2deg(phi_rad);
    % coordinates of the end effector CG
%     x = B_x - R*cos(phi_rad)/2;
%     y = B_y - R*sin(phi_rad)/2;
%     phi = rad2deg(phi_rad);
end
