clear; clc;
close all;
% Get parameters
params

% see the initial results
initPlotting

% Start the optimization
optimization

% Final plotting
plotting

% t = 0:0.1:5;
% v = sin(t);  % example velocity profile
% 
% a = diff(v)./diff(t);
% neg_indices = find(a < 0);
% 
% t_neg = t(neg_indices + 1);