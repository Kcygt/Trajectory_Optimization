%% plot_cartesian_all_planes_minDist_rmse_final_withTargetPlaneColors_shiftedXZ.m
% - 3D Cartesian trajectories (Sdata2, Pdata2, Pdata3, Pdata4, Pdata5)
% - XY (z = zmin), shifted XZ (y = ymin + 0.1), YZ (x = xmin) transparent planes
% - Shadow projections (hidden from legend)
% - Target projections with plane-specific colors
% - Start + Final + Target(s) in 3D (legend)
% - Prints min distance, Cartesian RMSE, and Joint-space RMSE (pos & vel)
% - Uses LaTeX for \dot{q} labels; no Unicode diacritics

clear; clc; close all

%% ---------------- Load data ----------------
S  = load('Sdata2.mat');
P2 = load('Pdata2.mat');
P3 = load('Pdata3.mat');
P4 = load('Pdata4.mat');
P5 = load('Pdata5.mat');

ttimes = S.optimalTimes;
tfinal = S.optimalTimes(3);
tOpt   = S.tOpt;

% Actual joints
qS  = S.yOpt(:,1:3);
qP2 = P2.Pdata2(:,7:9);
qP3 = P3.Pdata3(:,7:9);
qP4 = P4.Pdata4(:,7:9);
qP5 = P5.Pdata5(:,7:9);

qdS  = S.yOpt(:,4:6);
qdP2 = P2.Pdata2(:,10:12);
qdP3 = P3.Pdata3(:,10:12);
qdP4 = P4.Pdata4(:,10:12);
qdP5 = P5.Pdata5(:,10:12);

%% ---------------- Targets ----------------
xTarget = [];
if isfield(S,'xTarget'), xTarget = S.xTarget; end
if isempty(xTarget), xTarget = [0.02 0.06 0.06]; end
if size(xTarget,2) ~= 3, xTarget = xTarget(:).'; end

startPt = [0 0 0];

%% ---------------- Forward Kinematics ----------------
l1 = 0.208; l2 = 0.168;
FK = @(q) [ ...
    sin(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))), ...
    l2 - l2*cos(q(:,3)) + l1*sin(q(:,2)), ...
    -l1 + cos(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))) ];

XYZ_S  = FK(qS);
XYZ_P2 = FK(qP2);
XYZ_P3 = FK(qP3);
XYZ_P4 = FK(qP4);
XYZ_P5 = FK(qP5);

% Final point
if isfield(S,'xFinal') && numel(S.xFinal)>=3
    xFinal = S.xFinal(1,1:3);
else
    xFinal = XYZ_S(end,:);
end

%% ---------------- Colors & bundles ----------------
col_sim = [0.20 0.60 1.00];  % blue
col_p2  = [0.00 0.80 0.00];  % green
col_p3  = [1.00 0.50 0.00];  % orange
col_p4  = [0.70 0.00 0.70];  % purple
col_p5  = [0.10 0.10 0.10];  % black

names  = {'Simulation','Real','1.5x','2x','4x'};
colors = {col_sim, col_p2, col_p3, col_p4, col_p5};
trajs  = {XYZ_S,   XYZ_P2,   XYZ_P3,   XYZ_P4,   XYZ_P5};

% Plane target colors (per plane)
col_xy_proj = [1.00 0.85 0.20];   % yellow (XY @ zmin)
col_xz_proj = [1.00 0.20 0.80];   % magenta (XZ @ y = y_xz)
col_yz_proj = [0.20 1.00 1.00];   % cyan   (YZ @ xmin)

%% ---------------- 3D: trajectories + planes + shadows ----------------
fig = figure('Name','Cartesian Trajectories + Planes + Targets + RMSE');
% Make LaTeX the default for text in this figure (safe for \dot labels later)
set(fig, 'defaultTextInterpreter','latex');
set(fig, 'defaultAxesTickLabelInterpreter','latex');
set(fig, 'defaultLegendInterpreter','latex');

hold on; grid on; axis equal; view(45,25);

% Plot 3D trajectories
for i = 1:numel(trajs)
    plot3(trajs{i}(:,1),trajs{i}(:,2),trajs{i}(:,3),'-','Color',colors{i},'LineWidth',2.0,'DisplayName',names{i});
end

% Start, Final, Targets in 3D
plot3(startPt(1),startPt(2),startPt(3),'o','MarkerSize',10,'MarkerFaceColor','g','MarkerEdgeColor','k','DisplayName','Start');
plot3(xFinal(1),xFinal(2),xFinal(3),'s','MarkerSize',10,'MarkerFaceColor','r','MarkerEdgeColor','k','DisplayName','Final');
for k = 1:size(xTarget,1)
    plot3(xTarget(k,1),xTarget(k,2),xTarget(k,3),'p','MarkerSize',13,...
          'MarkerFaceColor',[0.95 0.7 0.2],'MarkerEdgeColor','k','DisplayName',sprintf('Target %d',k));
end

% Limits (+pad)
allXYZ = vertcat(XYZ_S,XYZ_P2,XYZ_P3,XYZ_P4,XYZ_P5, startPt, xFinal, xTarget);
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

% XY plane @ zmin
[Xp, Yp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(ylim_(1),ylim_(2),2));
Zp = zlim_(1)*ones(size(Xp));
surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[0.6 0.6 1.0], 'HandleVisibility','off');

% XZ plane @ y = y_xz (SHIFTED)
[Xp, Zp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(zlim_(1),zlim_(2),2));
Yp = y_xz*ones(size(Xp));
surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[0.6 1.0 0.6], 'HandleVisibility','off');

% YZ plane @ xmin
[Yp, Zp] = meshgrid(linspace(ylim_(1),ylim_(2),2), linspace(zlim_(1),zlim_(2),2));
Xp = xlim_(1)*ones(size(Yp));
surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[1.0 0.6 0.6], 'HandleVisibility','off');

% Shadows (hidden from legend)
light = 0.55;
sh_cols = cellfun(@(c) lighten(c, light), colors, 'UniformOutput', false);
for i = 1:numel(trajs)
    XYZ = trajs{i};
    plot3(XYZ(:,1), XYZ(:,2), zlim_(1)*ones(size(XYZ,1),1), '-', ...
        'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');            % XY
    plot3(XYZ(:,1), y_xz*ones(size(XYZ,1),1), XYZ(:,3), '-', ...
        'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');            % shifted XZ
    plot3(xlim_(1)*ones(size(XYZ,1),1), XYZ(:,2), XYZ(:,3), '-', ...
        'Color', sh_cols{i}, 'LineWidth', 1.4, 'HandleVisibility','off');            % YZ
end

% Target projections (plane-specific colors)
for k = 1:size(xTarget,1)
    t = xTarget(k,:);
    plot3(t(1), t(2), zlim_(1), 'p', 'MarkerSize', 9, ...
        'MarkerFaceColor', col_xy_proj, 'MarkerEdgeColor','k', 'HandleVisibility','off');          % XY
    plot3(t(1), y_xz,   t(3), 'p', 'MarkerSize', 9, ...
        'MarkerFaceColor', col_xz_proj, 'MarkerEdgeColor','k', 'HandleVisibility','off');          % shifted XZ
    plot3(xlim_(1), t(2), t(3), 'p', 'MarkerSize', 9, ...
        'MarkerFaceColor', col_yz_proj, 'MarkerEdgeColor','k', 'HandleVisibility','off');          % YZ
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Cartesian Trajectories with Projection Planes');
legend('Location','bestoutside');
xlim(xlim_); ylim(ylim_); zlim(zlim_);

%% ---------------- Minimum Distance (printed) ----------------
datasets = { ...
    'Simulation', XYZ_S; ...
    'Real',       XYZ_P2; ...
    '1.5x',       XYZ_P3; ...
    '2x',         XYZ_P4; ...
    '4x',         XYZ_P5  ...
};

fprintf('\n=== Minimum Cartesian distance to TARGET(s) ===\n');
for k = 1:size(xTarget,1)
    tgt = xTarget(k,:);
    fprintf('Target %d at [%.4f, %.4f, %.4f]:\n', k, tgt(1), tgt(2), tgt(3));
    for i = 1:size(datasets,1)
        name = datasets{i,1};
        XYZ  = datasets{i,2};
        d = vecnorm(XYZ - tgt, 2, 2);
        dmin = min(d);
        fprintf('  %-12s  d_min = %.6f\n', name, dmin);
    end
    fprintf('\n');
end

%% ---------------- Cartesian RMSE vs Simulation ----------------
Nxyz = 1000;
XYZ_ref = resample_equal(XYZ_S, Nxyz);
fprintf('=== Cartesian RMSE vs Simulation ===\n');
fprintf('Name         RMSE_total    RMSE_x      RMSE_y      RMSE_z\n');

for i = 2:numel(trajs)
    XYZ_i   = resample_equal(trajs{i}, Nxyz);
    diff    = XYZ_i - XYZ_ref;
    rmse_x  = sqrt(mean(diff(:,1).^2));
    rmse_y  = sqrt(mean(diff(:,2).^2));
    rmse_z  = sqrt(mean(diff(:,3).^2));
    rmseTot = sqrt(mean(sum(diff.^2,2)));
    fprintf('%-12s  %10.6f  %9.6f  %9.6f  %9.6f\n', names{i}, rmseTot, rmse_x, rmse_y, rmse_z);
end

%% ---------------- Joint-space RMSE (positions & velocities) ----------------
Nq = 1000;
q_ref  = resample_equal(qS,  Nq);
qd_ref = resample_equal(qdS, Nq);

Qsets  = {qP2, qP3, qP4, qP5};
QDsets = {qdP2, qdP3, qdP4, qdP5};
Jnames = names(2:end);           % {'Real','1.5x','2x','4x'}

fprintf('\n=== Joint Position RMSE vs Simulation (radians) ===\n');
fprintf('Name         RMSE_q1     RMSE_q2     RMSE_q3     RMSE_total\n');
for i = 1:numel(Qsets)
    qi   = resample_equal(Qsets{i}, Nq);
    dqi  = qi - q_ref;
    rmse = sqrt(mean(dqi.^2, 1));
    rmseTot = sqrt(mean(sum(dqi.^2, 2)));
    fprintf('%-12s  %9.6f  %9.6f  %9.6f  %12.6f\n', Jnames{i}, rmse(1), rmse(2), rmse(3), rmseTot);
end

fprintf('\n=== Joint Velocity RMSE vs Simulation (rad/s) ===\n');
fprintf('Name         RMSE_dq1    RMSE_dq2    RMSE_dq3    RMSE_total\n');
for i = 1:numel(QDsets)
    qdi   = resample_equal(QDsets{i}, Nq);
    dqdi  = qdi - qd_ref;
    rmse  = sqrt(mean(dqdi.^2, 1));
    rmseTot = sqrt(mean(sum(dqdi.^2, 2)));
    fprintf('%-12s  %9.6f  %9.6f  %9.6f  %12.6f\n', Jnames{i}, rmse(1), rmse(2), rmse(3), rmseTot);
end

%% ---------------- Time vectors for joint plots ----------------
tPs2 = linspace(0,size(qP2,1)/1000,size(qP2,1));
tPs3 = linspace(0,size(qP3,1)/1000,size(qP3,1));
tPs4 = linspace(0,size(qP4,1)/1000,size(qP4,1));
tPs5 = linspace(0,size(qP5,1)/1000,size(qP5,1));

%% ---------------- Joint position subplots (3x1) ----------------
fig2 = figure('Name','Joint Positions Comparison');
set(fig2, 'defaultTextInterpreter','latex');
set(fig2, 'defaultAxesTickLabelInterpreter','latex');
set(fig2, 'defaultLegendInterpreter','latex');

for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(tOpt, qS(:,i), 'LineWidth', 2.0, 'Color', col_sim);
    plot(tPs2, qP2(:,i),'LineWidth', 1.2, 'Color', col_p2);
    plot(tPs3, qP3(:,i),'LineWidth', 1.2, 'Color', col_p3);
    plot(tPs4, qP4(:,i),'LineWidth', 1.2, 'Color', col_p4);
    plot(tPs5, qP5(:,i),'LineWidth', 1.2, 'Color', col_p5);
    ylabel(sprintf('$q_{%d}$ (rad)', i), 'Interpreter','latex');
    title(sprintf('Joint %d Position', i), 'Interpreter','none');
    if i == 3, xlabel('Time (s)'); end
end
legend({'Simulation','Real','1.5x','2x','4x'}, 'Location','bestoutside');
sgtitle('Joint Positions vs Time', 'Interpreter','none');

%% ---------------- Joint velocity subplots (3x1) ----------------
fig3 = figure('Name','Joint Velocities Comparison');
set(fig3, 'defaultTextInterpreter','latex');
set(fig3, 'defaultAxesTickLabelInterpreter','latex');
set(fig3, 'defaultLegendInterpreter','latex');

for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(tOpt, qdS(:,i), 'LineWidth', 2.0, 'Color', col_sim);
    plot(tPs2, qdP2(:,i),'LineWidth', 1.2, 'Color', col_p2);
    plot(tPs3, qdP3(:,i),'LineWidth', 1.2, 'Color', col_p3);
    plot(tPs4, qdP4(:,i),'LineWidth', 1.2, 'Color', col_p4);
    plot(tPs5, qdP5(:,i),'LineWidth', 1.2, 'Color', col_p5);
    % IMPORTANT: LaTeX interpreter + single backslash
    ylabel(sprintf('$\\dot{q}_{%d}$ (rad/s)', i), 'Interpreter','latex');
    title(sprintf('Joint %d Velocity', i), 'Interpreter','none');
    if i == 3, xlabel('Time (s)'); end
end
legend({'Simulation','Real','1.5x','2x','4x'}, 'Location','bestoutside');
sgtitle('Joint Velocities vs Time', 'Interpreter','none');

%% ---------------- Helpers ----------------
function c2 = lighten(c, amt)
% Blend color c toward white by fraction amt in [0,1].
    c2 = c + amt*(1 - c);
end

function XYZr = resample_equal(XYZ, N)
% Resample an NxM array to exactly N rows using index-based interpolation.
    n = size(XYZ,1);
    if n == N
        XYZr = XYZ; 
        return
    end
    s  = linspace(1, n, n).';
    sr = linspace(1, n, N).';
    XYZr = interp1(s, XYZ, sr, 'linear');
end
