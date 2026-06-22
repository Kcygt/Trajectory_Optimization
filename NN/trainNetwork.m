clear;
clc;
close all;

load trajectoryDataset.mat

%% NORMALIZATION
[Xn,psX] = mapminmax(X');
[Yn,psY] = mapminmax(Y');

%% NETWORK
net = fitnet([128 128 64]);

net.trainParam.epochs = 1000;
net.trainParam.goal = 1e-6;

net.divideParam.trainRatio = 0.8;
net.divideParam.valRatio = 0.1;
net.divideParam.testRatio = 0.1;

%% TRAIN
net = train(net,Xn,Yn);

%% SAVE
save('trainedTrajectoryNet.mat','net','psX','psY');

fprintf('Neural network training completed.\n');