% process_and_plot_3D_only.m
clear; clc; close all;

%% --- dataset index ---
i = 5;
Pfile = sprintf('Pdata%d.mat', i);
Sfile = sprintf('Sdata%d.mat', i);

Pfile4x = sprintf('Pdata7.mat');


%% --- load data ---
Pdata = load(Pfile); % expects field Pdata.Pdata
Sdata = load(Sfile); % expects fields yOpt, tOpt, xTarget, Opt
Pdata4x = load(Pfile4x); % expects fields yOpt, tOpt, xTarget, Opt

%% --- extract data ---
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

xTarget = Sdata.xTarget(1:5,:); % first 5 targets

%% --- forward kinematics ---
[SxAct,SyAct,SzAct] = FK(SqAct(:,1), SqAct(:,2), SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1), PqAct(:,2), PqAct(:,3));
[SxDes, SyDes, SzDes] = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));
[PxAct4x,PyAct4x,PzAct4x] = FK(PqAct4x(:,1), PqAct4x(:,2), PqAct4x(:,3));

%% --- colors ---
colorSim     = [0 1 0]; % blue
colorPhantom = [1 0 0]; % red
colorPhantom4x = [0 0 1]; % red
colorRef     = [0 0 0]; % black
homeColor    = [0 0 0]; % black
targetColor  = [0 1 1]; % brownish/cyan
ctrlColor    = [1 .5 0]; % green
sphereColor  = [0 0 1]; % blue

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
xlim([-0.03 0.16]);
ylim([-0.06 0.03]);
zlim([-0.03 0.05]);

%% --- define planes for projections ---
zLimits = zlim;       
yLimits = ylim;       
planes.z = zLimits(1);  
planes.y = yLimits(2);  

%% --- plot Home position ---
h = gobjects(0); labels = {};
homePos = [0 0 0];
h(end+1) = plot3(homePos(1), homePos(2), homePos(3), 'o', ...
    'MarkerFaceColor',homeColor,'MarkerEdgeColor','k','MarkerSize',markerSizeDefault);
labels{end+1} = 'Home';

% projections of Home
plot3(homePos(1), homePos(2), planes.z, 'o', 'MarkerSize', markerSizeProj, ...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XY
plot3(homePos(1), planes.y, homePos(3), 'o', 'MarkerSize', markerSizeProj, ...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XZ

%% --- plot trajectories ---
h(end+1) = plot3(SxDes,SyDes,SzDes,'--','LineWidth',2,'Color',colorRef); labels{end+1} = 'Reference trajectory followed by optimization';
h(end+1) = plot3(SxAct,SyAct,SzAct,'-','LineWidth',2,'Color',colorSim);    labels{end+1} = 'Simulation on digital twin';
h(end+1) = plot3(PxAct,PyAct,PzAct,'-','LineWidth',2,'Color',colorPhantom); labels{end+1} = 'Physical Robot at normal speed';
h(end+1) = plot3(PxAct4x,PyAct4x,PzAct4x,'-','LineWidth',2,'Color',colorPhantom4x); labels{end+1} = 'Physical Robot at 4x speed';

%% --- plot target points (all 5) ---
for k = 1:5
    hTarget(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
        'p','MarkerSize',12,'MarkerFaceColor',targetColor,'MarkerEdgeColor',targetColor);
end
h(end+1) = hTarget(1); 
labels{end+1} = 'Targets';

%% --- plot control points and spheres ---
ctrlPts = Sdata.Opt(end-8:end);  
ctrlPts1 = ctrlPts(1:3);
ctrlPts2 = ctrlPts(4:6);
ctrlPts3 = ctrlPts(7:9);

ctrlCenters = {ctrlPts1, ctrlPts2, ctrlPts3};
radii = [0.0132, 0.005, 0.0112];  

hCtrl = []; % handle for legend
for k = 1:3
    % control point diamond
    hTmp = plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), ctrlCenters{k}(3), ...
        'd','MarkerFaceColor',ctrlColor,'MarkerEdgeColor',ctrlColor,'MarkerSize',markerSizeProj);
    if k == 1
        hCtrl = hTmp;
    end
    
    % sphere
    [sx, sy, sz] = sphere(30);
    xSphere = sx * radii(k) + ctrlCenters{k}(1);
    ySphere = sy * radii(k) + ctrlCenters{k}(2);
    zSphere = sz * radii(k) + ctrlCenters{k}(3);
    surf(xSphere, ySphere, zSphere, ...
        'FaceColor', sphereColor, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % projections of control points
    plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), planes.z, 'd', ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'none', ...
        'MarkerSize', markerSizeProj, 'HandleVisibility','off'); % XY
    plot3(ctrlCenters{k}(1), planes.y, ctrlCenters{k}(3), 'd', ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerEdgeColor', 'none', ...
        'MarkerSize', markerSizeProj, 'HandleVisibility','off'); % XZ
    
    % projections of spheres
    surf(sx*radii(k)+ctrlCenters{k}(1), ...
         sy*radii(k)+ctrlCenters{k}(2), ...
         planes.z*ones(size(sz)), ...
         'FaceColor', sphereColor, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility','off'); % XY
    surf(sx*radii(k)+ctrlCenters{k}(1), ...
         planes.y*ones(size(sy)), ...
         sz*radii(k)+ctrlCenters{k}(3), ...
         'FaceColor', sphereColor, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility','off'); % XZ
end
h(end+1) = hCtrl;
labels{end+1} = 'Control Points';

%% --- projections (XY and XZ only) ---
% Simulation
plot3(SxAct, SyAct, planes.z*ones(size(SzAct)), ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off'); % XY
plot3(SxAct, planes.y*ones(size(SyAct)), SzAct, ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off'); % XZ

% Phantom
plot3(PxAct, PyAct, planes.z*ones(size(PzAct)), ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % XY
plot3(PxAct, planes.y*ones(size(PyAct)), PzAct, ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % XZ

% Phantom 4x
plot3(PxAct4x, PyAct4x, planes.z*ones(size(PzAct4x)), ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off'); % XY
plot3(PxAct4x, planes.y*ones(size(PyAct4x)), PzAct4x, ':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off'); % XZ


% Reference
plot3(SxDes, SyDes, planes.z*ones(size(SzDes)), ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off'); % XY
plot3(SxDes, planes.y*ones(size(SyDes)), SzDes, ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off'); % XZ

% Targets projections (+2 size)
for k = 1:5
    xt = xTarget(k,:);
    plot3(xt(1), xt(2), planes.z,'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','HandleVisibility','off'); % XY
    plot3(xt(1), planes.y, xt(3),'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','HandleVisibility','off'); % XZ
end

%% --- legend ---
legend(h, labels, 'Location', 'eastoutside');
hold off;



% rotate3d on;
% disp('Adjust the view using the mouse. Press Enter in the Command Window when ready to save.');
% pause; % wa
% exportgraphics(gcf, sprintf('DataSet%d_3D_plot.png', i), 'Resolution', 1000);
% disp('Figure saved as PNG.');
%% --- Align trajectories ---
N = min([size(SqDes,1), size(SqAct,1), size(PqAct,1)]); % minimum number of points

% truncate all trajectories to N points
SqDes_trunc  = SqDes(1:N, :);
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

%% --- Joint RMSE (velocities) ---
dt_sim = mean(diff(Stime(1:N))); % aligned time step

vel_ref  = diff(SqDes_trunc) ./ dt_sim;
vel_sim  = diff(SqAct_trunc) ./ dt_sim;
vel_hw   = diff(PqAct_trunc) ./ dt_sim;

sim_joint_vel_RMSE = sqrt(mean(sum((vel_ref - vel_sim).^2, 2)));
hw_joint_vel_RMSE  = sqrt(mean(sum((vel_ref - vel_hw).^2, 2)));

fprintf('Joint velocity RMSE (Reference vs Simulation): %.5f rad/s\n', sim_joint_vel_RMSE);
fprintf('Joint velocity RMSE (Reference vs Hardware)  : %.5f rad/s\n', hw_joint_vel_RMSE);

%% --- Minimum distance from targets to reference trajectory ---
minDist = zeros(5,1);
for k = 1:5
    xt = xTarget(k,:);
    d = sqrt((SxDes_trunc-xt(1)).^2 + (SyDes_trunc-xt(2)).^2 + (SzDes_trunc-xt(3)).^2);
    minDist(k) = min(d);
end

for k = 1:5
    fprintf('Minimum distance from target %d to reference trajectory: %.5f m\n', k, minDist(k));
end



%% --- Figure 2: Joint Velocities with Sections ---
%% --- Figure 2: Joint Velocities with Sections ---
fig = figure('Name',sprintf('DataSet%d - Joint Velocities', i), ...
             'Position',[100 100 1200 800]);

jointNames = {'q1', 'q2', 'q3'};
colors = [0 1 0; 1 0 0; 0 0 0]; % simulation (green), hardware (red), reference (black)

% Define section boundaries and labels
sections = [0 0.82; 0.82 2.12; 2.12 3.45; 3.45 5];
sectionLabels = {'Section 1','Section 2','Section 3','Section 4'};
sectionCenters = mean(sections,2);

% Use tiledlayout for proper main title + subplot subtitles
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    ax = nexttile; 
    hold(ax,'on'); grid(ax,'on');

    % --- Plot velocities ---
    plot(ax, Ptime, PqdAct(:,j), '-', 'LineWidth', 2, 'Color', colors(2,:));  % Hardware
    plot(ax, Stime, SqdAct(:,j), '-', 'LineWidth', 2, 'Color', colors(1,:));  % Simulation
    plot(ax, Stime, SqdDes(:,j), '--', 'LineWidth', 2, 'Color', colors(3,:)); % Reference

    % --- Vertical dashed lines for section boundaries ---
    for s = 2:size(sections,1)
        xline(ax, sections(s,1), '--k', 'LineWidth', 1.2);
    end

    % --- Section labels at middle of each section ---
    yL = ylim(ax);
    for s = 1:size(sections,1)
        text(ax, sectionCenters(s), yL(2)*0.9, sectionLabels{s}, ...
            'HorizontalAlignment','center','FontWeight','bold');
    end

    % --- Y-label with LaTeX derivative and units ---
    ylabel(ax, sprintf('$\\dot{%s}\\,\\mathrm{(rad/s)}$', jointNames{j}), ...
           'Interpreter','latex', 'FontSize', 12);

    % --- Subplot title / subtitle ---
    title(ax, sprintf('Joint %d', j), 'FontWeight','bold');

    % --- Legend only on first subplot ---
    if j == 1
        legend(ax, {'Physical robot at normal speed','Simulation on digital twin','Reference trajectory followed by optimization'}, 'Location','best');
    end

    % --- X label only on bottom subplot ---
    if j == 3
        xlabel(ax, 'Time (s)');
    end

    hold(ax,'off');
end

% --- Main figure title ---
sgtitle(tl, '', 'FontWeight','bold', 'FontSize', 16);
