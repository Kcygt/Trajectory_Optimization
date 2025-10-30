clear; clc; close all;

% ===== Load Data =====
load('Sdata8.mat', 'tOpt', 'wnOpt', 'xTarget', 'yOpt', 'ctrlPoints');

% ===== Forward Kinematics function =====
FK = @(q1, q2, q3) deal( ...
    sin(q1).*(0.208*cos(q2)+0.168*sin(q3)), ... % X
    0.168 - 0.168*cos(q3) + 0.208*sin(q2), ... % Y
    -0.208 + cos(q1).*(0.208*cos(q2)+0.168*sin(q3)) ); % Z

% ===== Compute Cartesian Trajectories =====
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9)); % actual
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3)); % desired

% ===== Colors and Style =====
col_opt  = [0.20 0.60 1.00];
col_tgt  = [1.00 0.55 0.10];
col_ctrl = [0.10 0.70 0.10];
col_end  = [0.85 0.10 0.10];
col_start= [0.10 0.70 0.70];

% ===== Plot Cartesian Space Trajectory =====
figure('Name','Cartesian Space Trajectory (3D)');
hold on; grid on; axis equal;
view(45,25);

% Plot optimized trajectory
plot3(CxOpt, CyOpt, CzOpt, '-', 'LineWidth', 2.0, 'Color', col_opt, 'DisplayName', 'Optimized Trajectory');

% Plot target points
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', col_tgt, 'MarkerEdgeColor','k', 'DisplayName', 'Target Points');

% Plot control points
for i = 1:size(ctrlPoints,1)
    plot3(ctrlPoints(i,1), ctrlPoints(i,2), ctrlPoints(i,3), ...
        'd', 'MarkerSize', 9, 'MarkerFaceColor', col_ctrl, 'MarkerEdgeColor','k', ...
        'DisplayName', sprintf('Control Point %d', i));
end

% Start and End points
startXYZ = [0, 0, 0];
endXYZ   = [CxDes(end), CyDes(end), CzDes(end)];
plot3(startXYZ(1), startXYZ(2), startXYZ(3), 'o', 'MarkerSize', 9, ...
      'MarkerFaceColor', col_start, 'MarkerEdgeColor','k', 'DisplayName', 'Start');
plot3(endXYZ(1), endXYZ(2), endXYZ(3), 's', 'MarkerSize', 10, ...
      'MarkerFaceColor', col_end, 'MarkerEdgeColor','k', 'DisplayName', 'End');

% Connect control points visually
if size(ctrlPoints,1) >= 2
    plot3(ctrlPoints(:,1), ctrlPoints(:,2), ctrlPoints(:,3), ...
          ':', 'LineWidth', 1.0, 'Color', [0.3 0.8 0.3], 'HandleVisibility','off');
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Cartesian Space Trajectory');
legend('Location','bestoutside');

% Fixed axis limits
xlim([-0.3 0.2]);
ylim([-0.3 0.2]);
zlim([-0.3 0.2]);
