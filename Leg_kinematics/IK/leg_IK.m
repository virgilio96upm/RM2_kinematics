%% INVERSE KINEMATICS %%
% INPUT: position and orientation in mm and deg , respectively 
% OUTPUT: Motor input in deg

% test inverse kinematics
P_X = 45.7641; % X position of the end side of the end effector coupling
P_Y = 51.7588; % Y position of the end side of the end effector coupling
Phi = deg2rad(9.9538); % Tilt angle of the end effector with respect to the horizontal axis

[q1,q2,q3] = inv_kin(P_X,P_Y,Phi);

function [q1,q2,q3] = inv_kin(P_X,P_Y,Phi)
% Constants
D_1 = 33.8;
D_2 = 45;
L1 = 38.5;
L2 = 55;
L3 = L1;
l1 = 42;
l2 = 60;
l3 = l1;
R = 78.8;

a1 = -2*(P_X - R*cos(Phi))*L1;
b1 = -2*(P_Y - R*sin(Phi))*L1;
c1 = (a1/(-2*L1))^2 + (b1/(-2*L1))^2 + L1^2 - l1^2;
t1 = (-b1 - sqrt(b1^2 - c1^2 + a1^2))/(c1 - a1);
if (b1^2 + a1^2)<= c1^2
  error("No real value can be obtained")
else
q1_rad = 2*atan(t1);

q1 = rad2deg(q1_rad);% M1 deg

a2 = -2*(P_X - D_1)*L2;
b2 = -2*(P_Y)*L2;
c2 = (a2/(-2*L2))^2 + (b2/(-2*L2))^2 + L2^2 - l2^2;
t2 = (-b2 - sqrt(b2^2 - c2^2 + a2^2))/(c2 - a2);
if (b2^2 + a2^2)<= c2^2
  error("No real value can be obtained")
else
q2_rad = 2*atan(t2);

q2 = rad2deg(q2_rad); % M2 deg

a3 = -2*(P_X-(D_1+D_2))*L3;
b3 = -2*(P_Y)*L3;
c3 = (a3/(-2*L3))^2 + (b3/(-2*L3))^2 + L3^2 - l3^2;
t3 = (-b3 + sqrt(b3^2 - c3^2 + a3^2))/(c3 - a3);
if (b3^2 + a3^2)<= c3^2
  error("No real value can be obtained")
else
q3_rad = 2*atan(t3);

q3 = rad2deg(q3_rad); % M3 deg

end
