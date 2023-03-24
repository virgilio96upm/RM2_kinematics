
%%% FORWARD KINEMATICS FUNCTION %%%
% INPUT 
% l11,l12,l21,l22,l31,l32 en milimetros
% OUTPUT
% (Q(e,v),X,Y,Z)-> cuaternio (escalar,X,Y,Z) y posición en milimetros (X,Y,Z)
l = [60 75.01 95 60.01 75 60.01];
pose = Fk_RM2CM(l);

function RM2CM_Pose = Fk_RM2CM(l)
% substract first section rack lenghts. the negative value for the second
% racks is due to the sense of rotation in the motor. check
l(:,2) = l(:,2)- l(:,1);
l(:,4) = l(:,4)- l(:,3);
l(:,6) = l(:,6) -l(:,5);
% verify that there are not negative rack lenghts
if any(l<0)
    error("Not feasable path can be obtained because the lenght of a section is negative")
end
% rotations needed to maintain base orientation in TCP frame
Rot_z0 = [0 1 0 0;
         -1 0 0 0;
         0 0 1 0;
         0 0 0 1];
Rot_z1 = [0 -1 0 0;
         1 0 0 0;
         0 0 1 0;
         0 0 0 1];
% first section
T1 = FK(l(1,1), l(1,3), l(1,5));
% Second section
T2 = FK(l(1,2), l(1,4), l(1,6));
% HTM from DK
DK_pose = Rot_z0*T1*T2*Rot_z1;
% Orientation in quaternion form
DK_q = tform2quat(DK_pose);
% Position of the arm
DK_xyz = (DK_pose * [0;0;0;1]);
DK_xyz(4,:)=[];
RM2CM_Pose = [DK_q DK_xyz.'];
end

function T = FK(l1,l2,l3) % Homogeneous transformation Matrix
if (l1==l2) && (l2==l3) && (l1==0)
    T=[1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
else
    d = 61; % disc diameter
    g = l1*l1 + l2*l2 + l3*l3 -l1*l2 -l1*l3 -l2*l3;
    % parameterization of the arm by (s,k,phi)
    k = 2*sqrt(g)/(d*(l1+l2+l3));
    if k <= 0.000001
        T=[1 0 0 0;
            0 1 0 0;
            0 0 1 l1;
            0 0 0 1];
    else
        r = 1/k;
        phi = atan2(sqrt(3)*(l2-l3),(l2+l3-2*l1));
        theta = 2*sqrt(g)/(3*d);
        % Auxiliar variables
        c_t = cos(theta);
        s_t = sin(theta);
        c_p = cos(phi);
        s_p = sin(phi);
        % Homogeneus Matrix
        T = [c_p*c_p*c_t + s_p*s_p, c_p*s_p*(c_t-1),       c_p*s_t, r*c_p*(1-c_t);
            c_p*s_p*(c_t-1),       s_p*s_p*c_t + c_p*c_p, s_p*s_t, r*s_p*(1-c_t);
            -c_p*s_t,              -s_p*s_t,               c_t,     r*s_t;
            0,                     0,                     0,       1;];
    end
end
end
