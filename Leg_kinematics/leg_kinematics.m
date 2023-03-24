
%test direct kinematics
q1_d = deg2rad(-175);
q2_d = deg2rad(9.6);
q3_d = deg2rad(-195);
[x_1,y_1,phi_1] = dir_kin(q1_d,q2_d,q3_d);

% test inverse kinematics (Check if numbers are correct)
P_X = x_1; % X position of the end side of the end effector coupling
P_Y = y_1; % Y position of the end side of the end effector coupling
Phi = deg2rad(phi_1); % Tilt angle of the end effector with respect to the horizontal axis

[q1,q2,q3] = inv_kin(P_X,P_Y,Phi);

%test direct kinematics 2
q1_i = deg2rad(q1);
q2_i = deg2rad(q2);
q3_i = deg2rad(q3);
[x_2,y_2,phi_2] = dir_kin(q1_i,q2_i,q3_i);
%Inverse kinematics
%Direct kinematics
%     O_1 = [0, 0];
%     O_2 = [D_1, 0];
%     O_3 = [D_1+D_2, 0];

%% Leg Kinematics
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

    B_y = (-v1 + sqrt(v1^2 - 4*u1*w1))/(2*u1); % y coordinate of point P
    B_x = -((e3-e2)*B_y + (f3-f2))/((d3-d2)); % x coordinate of point P


    u2 = -2*R*(B_x-L1x);
    v2 = -2*R*(B_y-L1y);
    w2 = (B_x-L1x)^2 + (B_y-L1y)^2 - l1^2 + R^2;

    t = (-v2 - sqrt(v2^2 - w2^2 + u2^2))/(w2-u2);
    phi_rad = 2*atan(t); % orientation of the end effector
    
    % coordinates of the end effector
    x = B_x;
    y = B_y;
    phi = rad2deg(phi_rad);
    % coordinates of the end effector
%     x = B_x - R*cos(phi_rad)/2;
%     y = B_y - R*sin(phi_rad)/2;
%     phi = rad2deg(phi_rad);
end

function [q1,q2,q3] = inv_kin(P_X,P_Y,Phi)
    D_1 = 33.8;
    D_2 = 45;
    L1 = 38.5;
    L2 = 55;
    L3 = L1;
    l1 = 42;
    l2 = 60;
    l3 = l1;
    R = 78.8;

    a1 = -2*(P_X - R.*cos(Phi)).*L1;
    b1 = -2*(P_Y - R.*sin(Phi)).*L1;
    c1 = (a1./(-2*L1)).^2 + (b1./(-2*L1)).^2 + L1^2 - l1^2;
    
    
    t1 = (-b1 - sqrt(b1.^2 - c1.^2 + a1.^2))./(c1 - a1);
    q1_rad = 2.*atan(t1); % orientation of the end effector
    q1 = rad2deg(q1_rad);
    %disp(c1);
    
    a2 = -2.*(P_X - D_1).*L2;
    b2 = -2.*(P_Y).*L2;
    c2 = (a2./(-2*L2)).^2 + (b2./(-2*L2)).^2 + L2^2 - l2^2;
    t2 = (-b2 - sqrt(b2.^2 - c2.^2 + a2.^2))./(c2 - a2);
    q2_rad = 2.*atan(t2); % orientation of the end effector
    q2 = rad2deg(q2_rad);
    
    a3 = -2.*(P_X-(D_1+D_2)).*L3;
    b3 = -2.*(P_Y).*L3;
    c3 = (a3./(-2*L3)).^2 + (b3./(-2*L3)).^2 + L3^2 - l3^2;
    t3 = (-b3 + sqrt(b3.^2 - c3.^2 + a3.^2))./(c3 - a3);
    q3_rad = 2.*atan(t3); % orientation of the end effector
    q3 = rad2deg(q3_rad);
    

end

