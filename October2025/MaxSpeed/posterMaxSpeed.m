% process_and_plot_3D_only_cleaned_YZproj_opposite_with_joint_plots_and_measured_vels.m
clear; clc; close all;

%% --- dataset index ---
i = 8;
Pfile = sprintf('Pdata%d.mat', i);
Sfile = sprintf('Sdata%d.mat', i);
Pfile4x = sprintf('Pdata10.mat');

%% --- load data ---
Pdata = load(Pfile); % expects field Pdata.Pdata
Sdata = load(Sfile); % expects fields yOpt, tOpt, xTarget, Opt
Pdata4x = load(Pfile4x); % expects fields yOpt, tOpt, xTarget, Opt

%% --- extract data (positions + measured velocities) ---
PqAct   = Pdata.Pdata(:,7:9);    % hardware joint positions
PqdAct  = Pdata.Pdata(:,10:12);  % hardware joint velocities (added)

PqAct4x   = Pdata4x.Pdata(:,7:9);    % hardware joint positions
PqdAct4x  = Pdata4x.Pdata(:,10:12);  % hardware joint velocities (added)

SqDes   = Sdata.yOpt(:,1:3);     % reference joint positions
SqdDes  = Sdata.yOpt(:,4:6);     % reference joint velocities (added)

SqAct   = Sdata.yOpt(:,7:9);     % simulation joint positions
SqdAct  = Sdata.yOpt(:,10:12);   % simulation joint velocities (added)

Stime = Sdata.tOpt;
Ptime = linspace(0, Stime(end), size(PqAct,1));

xTarget = Sdata.xTarget(1:3,:); % first 3 targets (you used 3 in plotting)

%% --- forward kinematics ---
[SxAct,SyAct,SzAct] = FK(SqAct(:,1), SqAct(:,2), SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1), PqAct(:,2), PqAct(:,3));
[PxAct4x,PyAct4x,PzAct4x] = FK(PqAct4x(:,1), PqAct4x(:,2), PqAct4x(:,3));

[SxDes, SyDes, SzDes] = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));

%% --- colors ---
colorSim     = [0 1 0]; % blue
colorPhantom = [1 0 0]; % red
colorPhantom4x = [0 0 1]; % red
colorRef     = [0 0 0]; % black
homeColor    = [0 0 0]; % black
targetColor  = [0 1 1]; % brownish/cyan
ctrlColor    = [1 .5 0]; % green

%% --- marker sizes ---
markerSizeDefault = 10;
markerSizeProj    = markerSizeDefault + 2; % +2 for projections

%% --- Figure 1: 3D Cartesian ---
figure('Name',sprintf('DataSet%d - 3D Cartesian',i),'Position',[100 100 1200 800]);
hold on; grid on; view(3); axis equal;
xlabel('$X$ (m)','Interpreter','latex');
ylabel('$Y$ (m)','Interpreter','latex');
zlabel('$Z$ (m)','Interpreter','latex');
title('Cartesian Space Trajectories');

%% --- fixed axes limits ---
xlim([-0.1 0.02]);
ylim([-0.1 0.02]);
zlim([-0.05 0.05]);

%% --- define planes for projections ---
xLimits = xlim;
yLimits = ylim;
zLimits = zlim;
planes.x = xLimits(2)+0.07;   % YZ plane position set to MAX X (opposite side + offset)
planes.z = zLimits(1);        % XY plane position (at min Z)
planes.y = yLimits(2);        % XZ plane position (at max Y)

%% --- plot Home position ---
h = gobjects(0); labels = {};
homePos = [0 0 0];
h(end+1) = plot3(homePos(1), homePos(2), homePos(3), 'o', ...
    'MarkerFaceColor',homeColor,'MarkerEdgeColor','k','MarkerSize',markerSizeDefault);
labels{end+1} = 'Home';

% projections of Home (XY, XZ, YZ at opposite side)
plot3(homePos(1), homePos(2), planes.z, 'o', 'MarkerSize', markerSizeProj, ...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XY
plot3(homePos(1), planes.y, homePos(3), 'o', 'MarkerSize', markerSizeProj, ...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XZ
plot3(planes.x, homePos(2), homePos(3), 'o', 'MarkerSize', markerSizeProj, ...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % YZ (opposite)

%% --- plot trajectories ---
h(end+1) = plot3(SxAct,SyAct,SzAct,'-','LineWidth',2,'Color',colorSim);    labels{end+1} = 'Simulation';
h(end+1) = plot3(PxAct,PyAct,PzAct,'-','LineWidth',2,'Color',colorPhantom); labels{end+1} = 'Pyhsical Robot (1x)';
h(end+1) = plot3(PxAct4x,PyAct4x,PzAct4x,'-','LineWidth',2,'Color',colorPhantom4x); labels{end+1} = 'Pyhsical Robot (4x)';
h(end+1) = plot3(SxDes,SyDes,SzDes,'--','LineWidth',2,'Color',colorRef); labels{end+1} = 'Reference';

%% --- plot target points (first 3 as before) ---
for k = 1:3
    hTarget(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
        'p','MarkerSize',12,'MarkerFaceColor',targetColor,'MarkerEdgeColor',targetColor);
end
h(end+1) = hTarget(1);
labels{end+1} = 'Targets';

%% --- plot control points (spheres removed) ---
ctrlPts = Sdata.ctrlPoints;
ctrlPts1 = ctrlPts(1,:);
ctrlPts2 = ctrlPts(2,:);

ctrlCenters = {ctrlPts1, ctrlPts2};
radii = [0.0132, 0.005];  % kept for reference (not used)

hCtrl = []; % handle for legend
for k = 1:2
    % control point diamond (kept)
    hTmp = plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), ctrlCenters{k}(3), ...
        'd','MarkerFaceColor',ctrlColor,'MarkerEdgeColor',ctrlColor,'MarkerSize',markerSizeProj);
    if k == 1
        hCtrl = hTmp;
    end

    % projections of control points (XY, XZ, YZ at opposite) - keep but hidden from legend
    plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), planes.z, 'd', ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'none', ...
        'MarkerSize', markerSizeProj, 'HandleVisibility','off'); % XY
    plot3(ctrlCenters{k}(1), planes.y, ctrlCenters{k}(3), 'd', ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'none', ...
        'MarkerSize', markerSizeProj, 'HandleVisibility','off'); % XZ
    plot3(planes.x, ctrlCenters{k}(2), ctrlCenters{k}(3), 'd', ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'none', ...
        'MarkerSize', markerSizeProj, 'HandleVisibility','off'); % YZ (opposite)
end
h(end+1) = hCtrl;
labels{end+1} = 'Control Points';

%% --- projections (XY, XZ and YZ at opposite) ---
% Simulation
plot3(SxAct, SyAct, planes.z*ones(size(SzAct)), ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off'); % XY
plot3(SxAct, planes.y*ones(size(SyAct)), SzAct, ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off'); % XZ
plot3(planes.x*ones(size(SxAct)), SyAct, SzAct, ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off'); % YZ (opposite)

% Phantom
plot3(PxAct, PyAct, planes.z*ones(size(PzAct)), ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % XY
plot3(PxAct, planes.y*ones(size(PyAct)), PzAct, ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % XZ
plot3(planes.x*ones(size(PxAct)), PyAct, PzAct, ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % YZ (opposite)

% Reference
plot3(SxDes, SyDes, planes.z*ones(size(SzDes)), ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off'); % XY
plot3(SxDes, planes.y*ones(size(SyDes)), SzDes, ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off'); % XZ
plot3(planes.x*ones(size(SxDes)), SyDes, SzDes, ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off'); % YZ (opposite)

% 4x
plot3(PxAct4x, PyAct4x, planes.z*ones(size(PzAct4x)), ':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off'); % XY
plot3(PxAct4x, planes.y*ones(size(PyAct4x)), PzAct4x, ':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off'); % XZ
plot3(planes.x*ones(size(PxAct4x)), PyAct4x, PzAct4x, ':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off'); % YZ (opposite)


% Targets projections (+2 size) for XY, XZ, YZ (opposite)
for k = 1:3
    xt = xTarget(k,:);
    plot3(xt(1), xt(2), planes.z,'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','HandleVisibility','off'); % XY
    plot3(xt(1), planes.y, xt(3),'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','HandleVisibility','off'); % XZ
    plot3(planes.x, xt(2), xt(3),'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','HandleVisibility','off'); % YZ (opposite)
end

%% --- legend ---
legend(h, labels, 'Location', 'eastoutside');
hold off;

% rotate3d on;
% disp('Adjust the view using the mouse. Press Enter in the Command Window when ready to save.');
% pause; % wait for user
% exportgraphics(gcf, sprintf('DataSet%d_3D_plot.png', i), 'Resolution', 1000);
% disp('Figure saved as PNG.');

%% --- Align trajectories and truncate positions & velocities ---
N = min([size(SqDes,1), size(SqAct,1), size(PqAct,1), size(SqdDes,1), size(SqdAct,1), size(PqdAct,1)]); % minimum number of points

% truncate all trajectories to N points (positions)
SqDes_trunc  = SqDes;
SqAct_trunc  = SqAct(1:N, :);
PqAct_trunc  = PqAct(1:N, :);

SxDes_trunc = SxDes(1:N);
SyDes_trunc = SyDes(1:N);
SzDes_trunc = SzDes(1:N);

SxAct_trunc = SxAct(1:N);
SyAct_trunc = SyAct(1:N);
SzAct_trunc = SzAct(1:N);

PxAct_trunc = PxAct(1:N);
PyAct_trunc = PyAct(1:N);
PzAct_trunc = PzAct(1:N);

% truncate measured/provided velocities to N points
vel_ref = SqdDes(1:N, :);
vel_sim = SqdAct(1:N, :);
vel_hw  = PqdAct(1:N, :);

%% --- Cartesian RMSE ---
sim_cart_RMSE = sqrt(mean((SxDes_trunc-SxAct_trunc).^2 + ...
                          (SyDes_trunc-SyAct_trunc).^2 + ...
                          (SzDes_trunc-SzAct_trunc).^2));

hw_cart_RMSE  = sqrt(mean((SxDes_trunc-PxAct_trunc).^2 + ...
                          (SyDes_trunc-PyAct_trunc).^2 + ...
                          (SzDes_trunc-PzAct_trunc).^2));

fprintf('Cartesian RMSE (Reference vs Simulation): %.5f m\n', sim_cart_RMSE);
fprintf('Cartesian RMSE (Reference vs Hardware)  : %.5f m\n', hw_cart_RMSE);

%% --- Joint RMSE (positions) ---
sim_joint_RMSE = sqrt(mean(sum((SqDes_trunc - SqAct_trunc).^2, 2)));
hw_joint_RMSE  = sqrt(mean(sum((SqDes_trunc - PqAct_trunc).^2, 2)));

fprintf('Joint RMSE (Reference vs Simulation): %.5f rad\n', sim_joint_RMSE);
fprintf('Joint RMSE (Reference vs Hardware)  : %.5f rad\n', hw_joint_RMSE);

%% --- Joint RMSE (velocities) using provided/measured velocities ---
sim_joint_vel_RMSE = sqrt(mean(sum((vel_ref - vel_sim).^2, 2)));
hw_joint_vel_RMSE  = sqrt(mean(sum((vel_ref - vel_hw).^2, 2)));

fprintf('Joint velocity RMSE (Reference vs Simulation): %.5f rad/s\n', sim_joint_vel_RMSE);
fprintf('Joint velocity RMSE (Reference vs Hardware)  : %.5f rad/s\n', hw_joint_vel_RMSE);

%% --- Plot joint positions and velocities ---

% time vectors (use same length N for positions and velocities since velocities provided)
t = Stime(1:N);        

% Colors and labels
cols = {colorRef, colorSim, colorPhantom};
leg_names = {'Reference','Simulation','Hardware'};

% --- Joint positions (3 subplots, stacked) ---
figPos = figure('Name',sprintf('DataSet%d - Joint Positions',i),'Position',[150 150 900 800]);
for j = 1:3
    subplot(3,1,j); hold on; grid on;
    plot(t, SqDes_trunc(:,j), '--','LineWidth',1.6,'Color',cols{1});
    plot(t, SqAct_trunc(:,j), '-','LineWidth',1.2,'Color',cols{2});
    plot(Ptime, PqAct(:,j), '-','LineWidth',1.2,'Color',cols{3});
    ylabel(sprintf('q_{%d} (rad)', j),'Interpreter','tex');
    if j == 1
        title('Joint Positions');
        legend(leg_names,'Location','best');
    end
    if j == 3
        xlabel('Time (s)');
    end
end
hold off;

leg_names1 = {'Hardware','Simulation','Reference'};

%% --- Joint velocities (3 subplots, stacked) using provided velocities ---
figVel = figure('Name',sprintf('DataSet%d - Joint Velocities',i),...
                'Position',[200 200 900 800]);

cols = {colorRef, colorSim, colorPhantom};
leg_names1 = {'Hardware','Reference','Simulation'};

% Total simulation end time
t_end = t(end);

% Use tiledlayout for reliable spacing
tl = tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

% --- Loop over joints ---
for j = 1:3
    wn_values = cellfun(@(x) x(j), Sdata.wnOpt); % extract joint j wn for all sections
    t_switch1 = 1 / wn_values(1);
    t_switch2 = t_switch1 + 1 / wn_values(2);
    t_switch3 = t_end; % end of simulation

    % Section midpoints for labeling
    mid_sec1 = t_switch1 / 2;
    mid_sec2 = t_switch1 + (1 / wn_values(2))/2;
    mid_sec3 = t_switch2 + (t_end - t_switch2)/2;

    ax = nexttile; hold(ax,'on'); grid(ax,'on');

    % Plot velocities
    plot(ax, Ptime, PqdAct(:,j), '-','LineWidth',1.2,'Color',cols{3});
    plot(ax, t, vel_sim(:,j), '-','LineWidth',2,'Color',[0 1 0]);
    plot(ax, t, vel_ref(:,j), '--','LineWidth',1.6,'Color',cols{1});

    % --- Correct ylabel using LaTeX interpreter ---
    % Example produced label:  \dot{q}_1 (rad/s)
    ylabel(ax, sprintf('$\\dot{q}_{%d}\\,\\mathrm{(rad/s)}$', j), 'Interpreter', 'latex', 'FontSize', 11,'FontWeight','bold');

    % Vertical dashed lines at switching times
    xline(ax, t_switch1,'--k','HandleVisibility','off');
    xline(ax, t_switch2,'--k','HandleVisibility','off');

    % Section labels in the middle of each section
    ylims = ylim(ax);
    ytext = ylims(2) * 0.9; % slightly below top
    text(ax, mid_sec1, ytext, 'Section 1','HorizontalAlignment','center','FontWeight','bold');
    text(ax, mid_sec2, ytext, 'Section 2','HorizontalAlignment','center','FontWeight','bold');
    text(ax, mid_sec3, ytext, 'Section 3','HorizontalAlignment','center','FontWeight','bold');

    % Add per-subplot small title (this is your "subtitle" for each subplot)
    title(ax, sprintf('Joint %d', j), 'FontWeight','normal');

    % Put legend only on the first subplot
    if j == 1
        legend(ax, leg_names1, 'Location', 'best');
    end

    % X label only on bottom subplot
    if j == 3
        xlabel(ax, 'Time (s)');
    end

    hold(ax,'off');
end

% === Figure-wide main title ===
sgtitle(tl, 'Joint Velocities', 'FontWeight', 'bold', 'FontSize', 14);


%% --- Minimum distance from targets to reference trajectory ---
minDist = zeros(5,1);
for k = 1:3
    xt = xTarget(k,:);
    d = sqrt((SxDes_trunc-xt(1)).^2 + (SyDes_trunc-xt(2)).^2 + (SzDes_trunc-xt(3)).^2);
    minDist(k) = min(d);
end

for k = 1:3
    fprintf('Minimum distance from target %d to reference trajectory: %.5f m\n', k, minDist(k));
end
