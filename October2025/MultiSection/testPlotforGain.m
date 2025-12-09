%% =========================
% Full Cartesian & Joint Analysis (1000 samples)
%% =========================
clear; clc; close all

%% ---------------- Load data ----------------
S  = load('Sdata2.mat');
P2 = load('Pdata2.mat');
P3 = load('Pdata3.mat');
P4 = load('Pdata4.mat');
P5 = load('Pdata5.mat');

tOpt   = S.tOpt;
tfinal = tOpt(end);

% Actual joints
qS  = S.yOpt(:,1:3);
qP2 = P2.Pdata(:,7:9);
qP3 = P3.Pdata(:,7:9);
qP4 = P4.Pdata4(:,7:9);
qP5 = P5.Pdata5(:,7:9);

qdS  = S.yOpt(:,4:6);
qdP2 = P2.Pdata(:,10:12);
qdP3 = P3.Pdata(:,10:12);
qdP4 = P4.Pdata4(:,10:12);
qdP5 = P5.Pdata5(:,10:12);

%% ---------------- Targets ----------------
xTarget = [];
if isfield(S,'xTarget'), xTarget = S.xTarget; end
if isempty(xTarget), xTarget = [0.02 0.06 0.06; 0.03 0.05 0.07]; end
if size(xTarget,2) ~= 3, xTarget = xTarget(:).'; end

startPt = [0 0 0];

%% ---------------- Forward Kinematics ----------------
l1 = 0.208; l2 = 0.168;
FK = @(q) [ ...
    sin(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))), ...
    l2 - l2*cos(q(:,3)) + l1*sin(q(:,2)), ...
    -l1 + cos(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))) ];

XYZ_P2 = FK(qP2);
XYZ_P3 = FK(qP3);
XYZ_P4 = FK(qP4);
XYZ_P5 = FK(qP5);

if isfield(S,'xFinal') && numel(S.xFinal)>=3
    xFinal = S.xFinal(1,1:3);
else
    xFinal = XYZ_P2(end,:); % use P2 as reference
end

%% ---------------- Colors & bundles ----------------
col_p2  = [0.2 0.6 1.0];  % black
col_p3  = [0.00 0.80 0.00];  % green
col_p4  = [1.00 0.50 0.00];  % orange
col_p5  = [0.70 0.00 0.70];  % purple

names  = {'Hardware','1.5x','2x','4x'};
colors = {col_p2, col_p3, col_p4, col_p5};
trajs  = {XYZ_P2, XYZ_P3, XYZ_P4, XYZ_P5};

%% ---------------- 3D: trajectories + planes + shadows ----------------
fig = figure('Name','Cartesian Trajectories + Planes + Targets');
set(fig, 'defaultTextInterpreter','latex');
set(fig, 'defaultAxesTickLabelInterpreter','latex');
set(fig, 'defaultLegendInterpreter','latex');

hold on; grid on; axis equal; view(45,25);

% Plot 3D trajectories
for i = 1:numel(trajs)
    plot3(trajs{i}(:,1),trajs{i}(:,2),trajs{i}(:,3),'-','Color',colors{i},'LineWidth',2.0,'DisplayName',names{i});
end

% Start and Final points
plot3(startPt(1),startPt(2),startPt(3),'o','MarkerSize',10,'MarkerFaceColor','b','MarkerEdgeColor','k','DisplayName','Start');
plot3(xFinal(1),xFinal(2),xFinal(3),'s','MarkerSize',10,'MarkerFaceColor','r','MarkerEdgeColor','k','DisplayName','Final');

% Limits (+pad)
allXYZ = vertcat(XYZ_P2,XYZ_P3,XYZ_P4,XYZ_P5, startPt, xFinal, xTarget);
xlim_ = [min(allXYZ(:,1)), max(allXYZ(:,1))];
ylim_ = [min(allXYZ(:,2)), max(allXYZ(:,2))];
zlim_ = [min(allXYZ(:,3)), max(allXYZ(:,3))];
pad = 0.02 * max([range(xlim_), range(ylim_), range(zlim_)]);
xlim_ = xlim_ + [-pad pad];
ylim_ = ylim_ + [-pad pad];
zlim_ = zlim_ + [-pad pad];

% Shifted XZ plane location
y_xz = ylim_(1) + 0.1;
ylim_ = [min(ylim_(1), y_xz), max(ylim_(2), y_xz)];

% 3D planes without face color
[Xp, Yp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(ylim_(1),ylim_(2),2));
Zp = zlim_(1)*ones(size(Xp));
surf(Xp, Yp, Zp, 'FaceAlpha',0, 'EdgeColor','none', 'HandleVisibility','off'); % XY
[Xp, Zp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(zlim_(1),zlim_(2),2));
Yp = y_xz*ones(size(Xp));
surf(Xp, Yp, Zp, 'FaceAlpha',0, 'EdgeColor','none', 'HandleVisibility','off'); % XZ
[Yp, Zp] = meshgrid(linspace(ylim_(1),ylim_(2),2), linspace(zlim_(1),zlim_(2),2));
Xp = xlim_(1)*ones(size(Yp));
surf(Xp, Yp, Zp, 'FaceAlpha',0, 'EdgeColor','none', 'HandleVisibility','off'); % YZ

% Shadows
light = 0.55;
sh_cols = cellfun(@(c) lighten(c, light), colors, 'UniformOutput', false);
for i = 1:numel(trajs)
    XYZ = trajs{i};
    plot3(XYZ(:,1), XYZ(:,2), zlim_(1)*ones(size(XYZ,1),1), '-', 'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');
    plot3(XYZ(:,1), y_xz*ones(size(XYZ,1),1), XYZ(:,3), '-', 'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');
    plot3(xlim_(1)*ones(size(XYZ,1),1), XYZ(:,2), XYZ(:,3), '-', 'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');
end

% Targets and projections
for k = 1:size(xTarget,1)
    if k == 2
        faceColor = [0.5 0.5 0.5]; projColor = [0.5 0.5 0.5];
    else
        faceColor = [0 0 0]; projColor = [0 0 0];
    end
    t = xTarget(k,:);
    plot3(t(1), t(2), t(3), 'p','MarkerFaceColor',faceColor,'MarkerEdgeColor','k','MarkerSize',13,'DisplayName',sprintf('Target %d',k));
    plot3(t(1), t(2), zlim_(1), 'p','MarkerFaceColor',projColor,'MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');
    plot3(t(1), y_xz, t(3), 'p','MarkerFaceColor',projColor,'MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');
    plot3(xlim_(1), t(2), t(3), 'p','MarkerFaceColor',projColor,'MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');
end

% Start point projection onto XZ plane (y = y_xz)  <-- ADDED
plot3(startPt(1), y_xz, startPt(3), 'o', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'HandleVisibility','off');

% Final point projections
plot3(xFinal(1), xFinal(2), zlim_(1), 's','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');
plot3(xFinal(1), y_xz, xFinal(3), 's','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');
plot3(xlim_(1), xFinal(2), xFinal(3), 's','MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',9,'HandleVisibility','off');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('\textbf{Cartesian Trajectories with Projection Planes}');
legend('Location','bestoutside'); xlim(xlim_); ylim(ylim_); zlim(zlim_);

%% ---------------- Joint Positions (3x1 subplot, gain style) ----------------
fig_q = figure('Name','Joint Positions');
set(fig_q, 'defaultTextInterpreter','latex');
set(fig_q, 'defaultAxesTickLabelInterpreter','latex');
set(fig_q, 'defaultLegendInterpreter','latex');

jointNames = {'q_1','q_2','q_3'};
traj_data  = {qP2, qP3, qP4, qP5};
time_scales = [1, 1.5, 2, 4];  % time scaling factors

% Resample all trajectories to 1000 points
for i = 1:numel(traj_data)
    n = size(traj_data{i},1);
    traj_data{i} = interp1(1:n, traj_data{i}, linspace(1,n,1000)', 'linear');
end

tUni = linspace(0, tOpt(end), 1000)';

for j = 1:3
    subplot(3,1,j); hold on; grid on;
    for i = 1:numel(traj_data)
        t_plot = tUni / time_scales(i);
        plot(t_plot, traj_data{i}(:,j), '-', 'Color', colors{i}, 'LineWidth', 1.5);
    end
    ylabel(sprintf('$\\mathbf{%s (rad)}$', jointNames{j}),'Interpreter','latex');
    if j==1
        title('\textbf{Joint Positions vs Time}');
        legend(names,'Location','bestoutside');
    end
    if j==3
        xlabel('Time (s)');
    end
end

%% ---------------- Joint Velocities (3x1 subplot, gain style) ----------------
fig_qd = figure('Name','Joint Velocities');
set(fig_qd, 'defaultTextInterpreter','latex');
set(fig_qd, 'defaultAxesTickLabelInterpreter','latex');
set(fig_qd, 'defaultLegendInterpreter','latex');

traj_vels = {qdP2, qdP3, qdP4, qdP5};

% Resample all velocities to 1000 points
for i = 1:numel(traj_vels)
    n = size(traj_vels{i},1);
    traj_vels{i} = interp1(1:n, traj_vels{i}, linspace(1,n,1000)', 'linear');
end

for j = 1:3
    subplot(3,1,j); hold on; grid on;
    for i = 1:numel(traj_vels)
        t_plot = tUni / time_scales(i);
        plot(t_plot, traj_vels{i}(:,j), '-', 'Color', colors{i}, 'LineWidth', 1.5);
    end
    ylabel(sprintf('$\\mathbf{\\dot{q}_{%d} (rad/s)}$', j),'Interpreter','latex');
    if j==1
        title('\textbf{Joint Velocities vs Time}');
        legend(names,'Location','bestoutside');
    end
    if j==3
        xlabel('Time (s)');
    end
end

%% ---------------- Helper functions ----------------

function c2 = lighten(c, amt)
    c2 = c + amt*(1 - c);
end
