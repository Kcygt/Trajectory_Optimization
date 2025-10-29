%% plot_cartesian_all_simple.m
% Plot Cartesian trajectories (3D) for all datasets
% No time, just positions from FK

clear; clc; close all

%% ---------------- Load data ----------------
S  = load('Sdata2.mat');
P2 = load('Pdata2.mat');
P3 = load('Pdata3.mat');
P4 = load('Pdata4.mat');
P5 = load('Pdata5.mat');

% ---------- Extract actual joint angles ----------
qS  = S.yOpt(:,7:9);
qP2 = P2.Pdata2(:,7:9);
qP3 = P3.Pdata3(:,7:9);
qP4 = P4.Pdata4(:,7:9);
qP5 = P5.Pdata5(:,7:9);

xTarget = S.xTarget;

%% ---------------- Forward Kinematics ----------------
l1 = 0.208;
l2 = 0.168;
FK = @(q) [ ...
    sin(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))), ...
    l2 - l2*cos(q(:,3)) + l1*sin(q(:,2)), ...
    -l1 + cos(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))) ];

XYZ_S  = FK(qS);
XYZ_P2 = FK(qP2);
XYZ_P3 = FK(qP3);
XYZ_P4 = FK(qP4);
XYZ_P5 = FK(qP5);

%% ---------------- 3D Plot ----------------
figure('Name','Cartesian Trajectories (All)');
hold on; grid on; axis equal; view(3);

% Colors
col_sim = [0.20 0.60 1.00];  % blue
col_p2  = [0.00 0.80 0.00];  % green
col_p3  = [1.00 0.50 0.00];  % orange
col_p4  = [0.70 0.00 0.70];  % purple
col_p5  = [0.10 0.10 0.10];  % black

% Plot trajectories (no time)
plot3(XYZ_S(:,1),  XYZ_S(:,2),  XYZ_S(:,3),  '-', 'Color', col_sim, 'LineWidth', 2.0, 'DisplayName','Simulation (Sdata2)');
plot3(XYZ_P2(:,1), XYZ_P2(:,2), XYZ_P2(:,3), '-', 'Color', col_p2,  'LineWidth', 1.8, 'DisplayName','Real (Pdata2)');
plot3(XYZ_P3(:,1), XYZ_P3(:,2), XYZ_P3(:,3), '-', 'Color', col_p3,  'LineWidth', 1.8, 'DisplayName','1.5x (Pdata3)');
plot3(XYZ_P4(:,1), XYZ_P4(:,2), XYZ_P4(:,3), '-', 'Color', col_p4,  'LineWidth', 1.8, 'DisplayName','2x (Pdata4)');
plot3(XYZ_P5(:,1), XYZ_P5(:,2), XYZ_P5(:,3), '-', 'Color', col_p5,  'LineWidth', 1.8, 'DisplayName','4x (Pdata5)');

% Mark start and target
plot3(0, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
      'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
plot3(xTarget(1,1), xTarget(1,2), xTarget(1,3), 'p', 'MarkerSize', 13, ...
      'MarkerFaceColor', [0.95 0.7 0.2], 'MarkerEdgeColor','k', 'DisplayName', 'Target');
plot3(xTarget(2,1), xTarget(2,2), xTarget(2,3), 'p', 'MarkerSize', 13, ...
      'MarkerFaceColor', [0.95 0.7 0.2], 'MarkerEdgeColor','k', 'DisplayName', 'Target');

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Cartesian Space Trajectories (All)');
legend('Location','bestoutside');
