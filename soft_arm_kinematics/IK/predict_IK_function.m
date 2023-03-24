% sequences
x_1 = table2array(struct2array(load("Evaluation_sequences\EndPositionSec1.mat")));
x_2 = table2array(struct2array(load("Evaluation_sequences\EndPositionSec2.mat")));
x_3 = table2array(struct2array(load("Evaluation_sequences\EndPositionSec3.mat")));
x_4 = table2array(struct2array(load("Evaluation_sequences\EndPositionSec4.mat")));
% Sequences predictions
seq_1_pred = predict(x_1);
seq_2_pred = predict(x_2);
seq_3_pred = predict(x_3);
seq_4_pred = predict(x_4);

% INPUT
% (Q(e,v),X,Y,Z)-> Quaternio (escalar,X,Y,Z) y posición en metros (X,Y,Z)
% Functions
function out = predict(X)
%ELM Parameters
input_weights = table2array(struct2array(load("FAST_SIGMOID\ELM_iweights_fastsigmoid_50HN.mat")));
biases = table2array(struct2array(load("FAST_SIGMOID\ELM_bias_fastsigmoid_50HN.mat")));
output_weights = table2array(struct2array(load("FAST_SIGMOID\ELM_oweights_fastsigmoid_50HN.mat")));
G = X * input_weights + biases.';
H = G ./ (1 + abs(G));
out = H * output_weights * 1000;
end
