%% process_and_plot_3D_only.m
clear; clc; close all;

%% --- Dataset index ---
i = 5;
Pfile    = sprintf('Pdata%d.mat', i);
Sfile    = sprintf('Sdata%d.mat', i);
Pfile2x  = 'Pdata6.mat';  % Hardware 2x
Pfile4x  = 'Pdata7.mat';  % Hardware 4x

%% --- Load data ---
Pdata     = load(Pfile);    % expects field Pdata.Pdata
Sdata     = load(Sfile);    % expects fields yOpt, tOpt, xTarget, Opt
Pdata2x   = load(Pfile2x);
Pdata4x   = load(Pfile4x);

%% --- Extract data ---
PqAct       = Pdata.Pdata(:,7:9);    % Hardware 1x joint positions
PqdAct      = Pdata.Pdata(:,10:12);  % Hardware 1x joint velocities
PqAct_2x    = Pdata2x.Pdata(:,7:9);
PqdAct_2x   = Pdata2x.Pdata(:,10:12);
PqAct_4x    = Pdata4x.Pdata(:,7:9);
PqdAct_4x   = Pdata4x.Pdata(:,10:12);

SqDes  = Sdata.yOpt(:,1:3);   % Reference joint positions
SqdDes = Sdata.yOpt(:,4:6);   % Reference joint velocities
SqAct  = Sdata.yOpt(:,7:9);   % Simulation joint positions
SqdAct = Sdata.yOpt(:,10:12); % Simulation joint velocities

Stime    = Sdata.tOpt;                                  % Reference time
Ptime    = linspace(0, Stime(end), size(PqAct,1));      % Hardware 1x time
Ptime2x  = linspace(0, Ptime(end)/2, size(PqAct_2x,1));   % Hardware 2x time
Ptime4x  = linspace(0, Ptime(end)/4, size(PqAct_4x,1));   % Hardware 4x time

xTarget = Sdata.xTarget(1:5,:);  % First 5 targets

%% --- Forward kinematics ---
[SxAct,SyAct,SzAct]       = FK(SqAct(:,1), SqAct(:,2), SqAct(:,3));
[PxAct,PyAct,PzAct]       = FK(PqAct(:,1), PqAct(:,2), PqAct(:,3));
[SxDes,SyDes,SzDes]       = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));
[PxAct2x,PyAct2x,PzAct2x] = FK(PqAct_2x(:,1), PqAct_2x(:,2), PqAct_2x(:,3));
[PxAct4x,PyAct4x,PzAct4x] = FK(PqAct_4x(:,1), PqAct_4x(:,2), PqAct_4x(:,3));

%% --- Colors ---
palette         = lines(6);
colorSim        = [0 1 0];        % Green
colorPhantom    = palette(6,:);    % Red
colorPhantom2x  = palette(4,:);
colorPhantom4x  = palette(5,:);
colorRef        = [0 0 0];        % Black
homeColor       = [0 0 0];
targetColor     = [0 1 1];
ctrlColor       = [1 0.5 0];
sphereColor     = [0 0 1];

%% --- Marker sizes ---
markerSizeDefault = 10;
markerSizeProj    = markerSizeDefault + 2;

%% --- Figure 1: 3D Cartesian Trajectories ---
figure('Name',sprintf('DataSet%d - 3D Cartesian',i),'Position',[100 100 1200 800]);
hold on; grid on; view(3); axis equal;

xlabel('$X$ (m)','Interpreter','latex');
ylabel('$Y$ (m)','Interpreter','latex');
zlabel('$Z$ (m)','Interpreter','latex');
title('Cartesian Space Trajectories');

% Fixed axes limits
xlim([-0.03 0.16]); ylim([-0.06 0.03]); zlim([-0.03 0.05]);
zLimits = zlim; yLimits = ylim; 
planes.z = zLimits(1); planes.y = yLimits(2);

%% --- Plot Home position ---
h = gobjects(0); labels = {};
homePos = [0 0 0];

% 3D Home
h(end+1) = plot3(homePos(1),homePos(2),homePos(3),'o', ...
    'MarkerFaceColor',homeColor,'MarkerEdgeColor','k','MarkerSize',markerSizeDefault);
labels{end+1} = 'Home';

% Projections
plot3(homePos(1), homePos(2), planes.z, 'o','MarkerSize',markerSizeProj,...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XY
plot3(homePos(1), planes.y, homePos(3), 'o','MarkerSize',markerSizeProj,...
    'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XZ

%% --- Plot trajectories ---
h(end+1) = plot3(SxDes,SyDes,SzDes,'--','LineWidth',2,'Color',colorRef); labels{end+1} = 'Reference trajectory';
h(end+1) = plot3(SxAct,SyAct,SzAct,'-','LineWidth',2,'Color',colorSim); labels{end+1} = 'Simulation';
h(end+1) = plot3(PxAct,PyAct,PzAct,'-','LineWidth',2,'Color',colorPhantom); labels{end+1} = 'Hardware(1x)';
h(end+1) = plot3(PxAct2x,PyAct2x,PzAct2x,'-','LineWidth',2,'Color',colorPhantom2x); labels{end+1} = 'Hardware(2x)';
h(end+1) = plot3(PxAct4x,PyAct4x,PzAct4x,'-','LineWidth',2,'Color',colorPhantom4x); labels{end+1} = 'Hardware(4x)';

%% --- Plot target points ---
for k = 1:5
    hTarget(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
        'p','MarkerSize',12,'MarkerFaceColor',targetColor,'MarkerEdgeColor',targetColor);
end
h(end+1) = hTarget(1); labels{end+1} = 'Targets';

%% --- Plot control points and spheres ---
ctrlPts = Sdata.Opt(end-8:end);
ctrlCenters = {ctrlPts(1:3), ctrlPts(4:6), ctrlPts(7:9)};
radii = [0.0132, 0.005, 0.0112];
hCtrl = [];

for k = 1:3
    % Diamond marker
    hTmp = plot3(ctrlCenters{k}(1),ctrlCenters{k}(2),ctrlCenters{k}(3),'d', ...
        'MarkerFaceColor',ctrlColor,'MarkerEdgeColor',ctrlColor,'MarkerSize',markerSizeProj);
    if k == 1, hCtrl = hTmp; end
    
    % Sphere
    [sx,sy,sz] = sphere(30);
    surf(sx*radii(k)+ctrlCenters{k}(1), sy*radii(k)+ctrlCenters{k}(2), sz*radii(k)+ctrlCenters{k}(3), ...
        'FaceColor',sphereColor,'FaceAlpha',0.3,'EdgeColor','none');
    
    % Projections
    plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), planes.z, 'd','MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','MarkerSize',markerSizeProj,'HandleVisibility','off'); % XY
    plot3(ctrlCenters{k}(1), planes.y, ctrlCenters{k}(3), 'd','MarkerFaceColor',[0.6 0.6 0.6], ...
        'MarkerEdgeColor','none','MarkerSize',markerSizeProj,'HandleVisibility','off'); % XZ
    
    % Sphere projections
    surf(sx*radii(k)+ctrlCenters{k}(1), sy*radii(k)+ctrlCenters{k}(2), planes.z*ones(size(sz)), ...
        'FaceColor',sphereColor,'FaceAlpha',0.1,'EdgeColor','none','HandleVisibility','off'); % XY
    surf(sx*radii(k)+ctrlCenters{k}(1), planes.y*ones(size(sy)), sz*radii(k)+ctrlCenters{k}(3), ...
        'FaceColor',sphereColor,'FaceAlpha',0.1,'EdgeColor','none','HandleVisibility','off'); % XZ
end

h(end+1) = hCtrl; labels{end+1} = 'Control Points';

% Legend for sphere only
hLegendSphere = plot3(NaN,NaN,NaN,'o','MarkerFaceColor',sphereColor,'MarkerEdgeColor','none','MarkerSize',10);
h(end+1) = hLegendSphere; labels{end+1} = 'Control Point Sphere';

%% --- Projections of trajectories ---
% Simulation
plot3(SxAct,SyAct,planes.z*ones(size(SzAct)),':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off');
plot3(SxAct,planes.y*ones(size(SyAct)),SzAct,':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off');

% Hardware 1x
plot3(PxAct,PyAct,planes.z*ones(size(PzAct)),':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off');
plot3(PxAct,planes.y*ones(size(PyAct)),PzAct,':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off');

% Hardware 2x
plot3(PxAct2x,PyAct2x,planes.z*ones(size(PzAct2x)),':','LineWidth',1.2,'Color',colorPhantom2x,'HandleVisibility','off');
plot3(PxAct2x,planes.y*ones(size(PyAct2x)),PzAct2x,':','LineWidth',1.2,'Color',colorPhantom2x,'HandleVisibility','off');

% Hardware 4x
plot3(PxAct4x,PyAct4x,planes.z*ones(size(PzAct4x)),':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off');
plot3(PxAct4x,planes.y*ones(size(PyAct4x)),PzAct4x,':','LineWidth',1.2,'Color',colorPhantom4x,'HandleVisibility','off');

% Reference
plot3(SxDes,SyDes,planes.z*ones(size(SzDes)),':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off');
plot3(SxDes,planes.y*ones(size(SyDes)),SzDes,':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off');

% Target projections
for k = 1:5
    xt = xTarget(k,:);
    plot3(xt(1), xt(2), planes.z,'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XY
    plot3(xt(1), planes.y, xt(3),'p','MarkerSize',markerSizeProj,'MarkerFaceColor',[0.6 0.6 0.6],'MarkerEdgeColor','none','HandleVisibility','off'); % XZ
end

%% --- Legend ---
legend(h, labels, 'Location','eastoutside');
hold off;





%% ------------------ Joint-space: positions ------------------
figure('Name','Joint Positions: Reference vs Simulation vs Phantom','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    
    % Reference joint positions
    plot(Stime, SqDes(:,j), 'k-', 'LineWidth', 1.6);
    
    % Simulation joint positions
    plot(Stime, SqAct(:,j), 'b--', 'LineWidth', 1.4);
    
    % Hardware 1x joint positions
    plot(Ptime, PqAct(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom);
    
    % Hardware 2x joint positions
    plot(Ptime2x, PqAct_2x(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom2x);
    
    % Hardware 4x joint positions
    plot(Ptime4x, PqAct_4x(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom4x);

    title(sprintf('Joint %d Position', j));
    ylabel(['$q_{' num2str(j) '}$ [rad]'], 'Interpreter','latex');
    legend({'Reference','Simulation','Hardware(1x)','Hardware(2x)','Hardware(4x)'}, 'Location','best');
end
xlabel('Time [s]');

%% ------------------ Joint-space: velocities ------------------
figure('Name','Joint Velocities: Reference vs Simulation vs Phantom','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    
    % Reference joint velocities
    plot(Stime, SqdDes(:,j), 'k-', 'LineWidth', 1.6);
    
    % Simulation joint velocities
    plot(Stime, SqdAct(:,j), 'b--', 'LineWidth', 1.4);
    
    % Hardware 1x joint velocities
    plot(Ptime, PqdAct(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom);
    
    % Hardware 2x joint velocities
    plot(Ptime2x, PqdAct_2x(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom2x);
    
    % Hardware 4x joint velocities
    plot(Ptime4x, PqdAct_4x(:,j), '-', 'LineWidth', 1.8, 'Color', colorPhantom4x);

    title(sprintf('Joint %d Velocity', j));
    ylabel(['$\dot{q}_{' num2str(j) '}$ [rad/s]'], 'Interpreter','latex');
    legend({'Reference','Simulation','Hardware(1x)','Hardware(2x)','Hardware(4x)'}, 'Location','best');
end
xlabel('Time [s]');
