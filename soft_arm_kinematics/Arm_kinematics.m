% INPUT Cinematica Directa
% l11,l12,l21,l22,l31,l32 en metros
% OUTPUT
% (Q(e,v),X,Y,Z)-> cuaternio (escalar,X,Y,Z) y posición en metros (X,Y,Z)

% INPUT Cinematica Inversa
% (Q(e,v),X,Y,Z)-> cuaternio (escalar,X,Y,Z) y posición en metros (X,Y,Z)
% OUTPUT
% l11,l12,l21,l22,l31,l32 en metros

x_1 = table2array(struct2array(load("Evaluation_sequences\EndPositionSec1.mat")));
% Sequences predictions
test = [0.996112059403955	0.0868697163209791	0.0130843604874492	0.00657396432594635	0.000111641000000000	-0.00248878000000000	0.0192355000000000];
seq_1_pred = predict(test); % prediccion IK NN
y_1 = Fk_RM2CM(seq_1_pred); % Calculo analitico DK basado en la predicción
seq_1_pred_2 = predict(y_1); % predicción IK NN del calculo analitico
y_2 = Fk_RM2CM(seq_1_pred_2);
seq_1_pred_3 = predict(y_2);

%% Functions
% Inverse_Kinematics
function out = predict(X)
%ELM Parameters
input_weights = table2array(struct2array(load("FAST_SIGMOID\ELM_iweights_fastsigmoid_50HN.mat")));
biases = table2array(struct2array(load("FAST_SIGMOID\ELM_bias_fastsigmoid_50HN.mat")));
output_weights = table2array(struct2array(load("FAST_SIGMOID\ELM_oweights_fastsigmoid_50HN.mat")));
G = X * input_weights + biases.';
H = G ./ (1 + abs(G));
out = H * output_weights;
end

function RM2CM_Pose = Fk_RM2CM(l)
% substract first section rack lenghts. the negative value for the second
% racks is due to the sense of rotation in the motor. check
l(:,2) = l(:,2)- l(:,1);
l(:,4) = l(:,4)- l(:,3);
l(:,6) = l(:,6) -l(:,5);
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
DK_xyz = DK_pose * [0;0;0;1];
DK_xyz(4,:)=[];
RM2CM_Pose = [DK_q DK_xyz.'];
end

function T = FK(l1,l2,l3) % Homogeneous transformation Matrix
d = 0.061; % disc diameter
g = l1*l1 + l2*l2 + l3*l3 -l1*l2 -l1*l3 -l2*l3;
% parameterization of the arm by (s,k,phi)
k = 2*sqrt(g)/(d*(l1+l2+l3));
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