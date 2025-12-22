% process_and_plot_3D_only.m
clear; clc; close all;

%% --- dataset index ---
i = 8;
Pfile = sprintf('Pdata%d.mat', i);
Sfile = sprintf('Sdata%d.mat', i);

%% --- load data ---
Pdata = load(Pfile); % expects field Pdata.Pdata
Sdata = load(Sfile); % expects fields yOpt, tOpt, xTarget, Opt

%% --- extract data ---
SqDes  = Sdata.yOpt(:,1:3);
SqAct  = Sdata.yOpt(:,7:9);
PqAct  = Pdata.Pdata(:,7:9);

SqdDes  = Sdata.yOpt(:,4:6);
SqdAct  = Sdata.yOpt(:,10:12);
PqdAct  = Pdata.Pdata(:,10:12);

Stime = Sdata.tOpt;
Ptime = linspace(0, Stime(end), size(PqAct,1));

xTarget = Sdata.xTarget(1:3,:); % first 3 targets

%% --- forward kinematics ---
[SxAct,SyAct,SzAct] = FK(SqAct(:,1), SqAct(:,2), SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1), PqAct(:,2), PqAct(:,3));
[SxDes, SyDes, SzDes] = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));

%% --- colors ---
colorSim     = [0 1 0]; 
colorPhantom = [1 .5 0];
colorRef     = [0 0 0];
homeColor    = [0 0 0];
targetColor  = [83 234 253]/255;
ctrlColor    = [1 0 1];
sphereColor  = [0 0 1];

%% --- Figure 1: 3D Cartesian ---
figure('Name',sprintf('DataSet%d - 3D Cartesian',i),'Position',[100 100 1200 800]);
hold on; grid on; view(3); axis equal;
xlabel('$X$ (m)','Interpreter','latex');
ylabel('$Y$ (m)','Interpreter','latex');
zlabel('$Z$ (m)','Interpreter','latex');
title('Cartesian Space Position');

%% --- fixed axes limits ---
xlim([-0.03 0.1]);
ylim([-0.06 0.03]);
zlim([-0.03 0.05]);

%% --- define planes for projections ---
zLimits = zlim;
yLimits = ylim;
xLimits = xlim;

planes.z = zLimits(1); % z-plane for projection
planes.y = yLimits(2); % y-plane for projection
planes.x = 0.1;    % x-plane for projection (for y-z plane)

%% --- legend containers ---
h = gobjects(0); 
labels = {};

%% --- plot Home position ---
h(end+1) = plot3(0,0,0,'o','MarkerFaceColor',homeColor,...
    'MarkerEdgeColor','k','MarkerSize',10); 
labels{end+1} = 'Home';

%% --- plot trajectories ---
h(end+1) = plot3(SxAct,SyAct,SzAct,'-','LineWidth',2,'Color',colorSim);
labels{end+1} = 'Simulation';

h(end+1) = plot3(PxAct,PyAct,PzAct,'-','LineWidth',2,'Color',colorPhantom);
labels{end+1} = 'Hardware';

h(end+1) = plot3(SxDes,SyDes,SzDes,'--','LineWidth',1.5,'Color',colorRef);
labels{end+1} = 'Reference';

%% --- plot target points ---
for k = 1:3
    hTarget(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
        'p','MarkerSize',10,'MarkerFaceColor',targetColor,...
        'MarkerEdgeColor',targetColor);
end
h(end+1) = hTarget(1);
labels{end+1} = 'Targets';

%% --- control points ---
ctrlPts = Sdata.ctrlPoints;
ctrlCenters = {ctrlPts(1,:), ctrlPts(2,:)};
radii = [0.0132, 0.005, 0.0112];

hCtrl   = gobjects(1);

for k = 1:2
    % control point
    hTmp = plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), ctrlCenters{k}(3), ...
        'd','MarkerFaceColor',ctrlColor,'MarkerEdgeColor',ctrlColor,...
        'MarkerSize',10);
    
    if k == 1
        hCtrl = hTmp;
    end
       
    % projections (x-y plane)
    plot3(ctrlCenters{k}(1), ctrlCenters{k}(2), planes.z,'d', ...
        'MarkerFaceColor',[.6 .6 .6],'MarkerEdgeColor','none','HandleVisibility','off');
    % projections (x-z plane)
    plot3(ctrlCenters{k}(1), planes.y, ctrlCenters{k}(3),'d', ...
        'MarkerFaceColor',[.6 .6 .6],'MarkerEdgeColor','none','HandleVisibility','off');
    % projections (y-z plane)
    plot3(planes.x, ctrlCenters{k}(2), ctrlCenters{k}(3),'d', ...
        'MarkerFaceColor',[.6 .6 .6],'MarkerEdgeColor','none','HandleVisibility','off');
end

h(end+1) = hCtrl;
labels{end+1} = 'Control Points';



%% --- trajectory projections ---
% x-y plane (z = min)
plot3(SxAct,SyAct,planes.z*ones(size(SzAct)),':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off');
plot3(PxAct,PyAct,planes.z*ones(size(PzAct)),':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off');
plot3(SxDes,SyDes,planes.z*ones(size(SzDes)),':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off');

% x-z plane (y = max)
plot3(SxAct,planes.y*ones(size(SyAct)),SzAct,':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off');
plot3(PxAct,planes.y*ones(size(PyAct)),PzAct,':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off');
plot3(SxDes,planes.y*ones(size(SyDes)),SzDes,':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off');

% y-z plane (x = min)
plot3(planes.x*ones(size(SxAct)), SyAct, SzAct, ':','LineWidth',1.2,'Color',colorSim,'HandleVisibility','off');
plot3(planes.x*ones(size(PxAct)), PyAct, PzAct, ':','LineWidth',1.2,'Color',colorPhantom,'HandleVisibility','off');
plot3(planes.x*ones(size(SxDes)), SyDes, SzDes, ':','LineWidth',1.2,'Color',colorRef,'HandleVisibility','off');

%% --- target projections ---
for k = 1:3
    xt = xTarget(k,:);
    % x-y plane
    plot3(xt(1), xt(2), planes.z, 'p', 'MarkerFaceColor',[.6 .6 .6], 'MarkerEdgeColor','none','HandleVisibility','off');
    % x-z plane
    plot3(xt(1), planes.y, xt(3), 'p', 'MarkerFaceColor',[.6 .6 .6], 'MarkerEdgeColor','none','HandleVisibility','off');
    % y-z plane
    plot3(planes.x, xt(2), xt(3), 'p', 'MarkerFaceColor',[.6 .6 .6], 'MarkerEdgeColor','none','HandleVisibility','off');
end

%% --- legend ---
legend(h,labels,'Location','eastoutside');
hold off;


%% --- Joint positions ---
figure('Name',sprintf('DataSet%d - Joint Positions',i),'Position',[150 150 800 700]);
jointNames = {'Joint 1','Joint 2','Joint 3'};

for j = 1:3
    subplot(3,1,j); hold on; grid on;
    
    % --- Plot with DisplayName ---
    plot(Ptime,PqAct(:,j),'Color',colorPhantom,'LineWidth',1.8,'DisplayName','Hardware');
    plot(Stime,SqAct(:,j),'Color',colorSim,'LineWidth',1.8,'DisplayName','Simulation');
    plot(Stime,SqDes(:,j),'--','Color',colorRef,'LineWidth',1.8,'DisplayName','Reference');
    
    ylabel(sprintf('$q_%d\\,[\\mathrm{rad}]$', j), 'Interpreter', 'latex');
    if j==1
        title('Joint Positions');
        legend('Location','best');
    end
    if j==3, xlabel('Time (s)'); end
    
    % % --- Compute section times ---
    % wn_j = cell2mat(cellfun(@(c) c(j), Sdata.wnOpt, 'UniformOutput', false)); 
    % t_section1 = 1 / wn_j(1);
    % t_section2 = t_section1 + 1 / wn_j(2);
    % 
    % % --- Vertical lines (no legend entries) ---
    % xline(t_section1,'--k','HandleVisibility','off');
    % xline(t_section2,'--k','HandleVisibility','off');
    % 
    % % --- Section labels ---
    % y_top = max([PqAct(:,j); SqAct(:,j); SqDes(:,j)]) * .85;
    % text(t_section1/2, y_top, 'Section 1', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
    % text(t_section1 + (t_section2 - t_section1)/2, y_top, 'Section 2', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
    % text((t_section2 + max(Stime))/2, y_top, 'Section 3', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
end

%% --- Joint velocities ---
figure('Name',sprintf('DataSet%d - Joint Velocities',i),'Position',[150 150 800 700]);

for j = 1:3
    subplot(3,1,j); hold on; grid on;
    
    % --- Plot with DisplayName ---
    plot(Ptime,PqdAct(:,j),'Color',colorPhantom,'LineWidth',1.8,'DisplayName','Hardware');
    plot(Stime,SqdAct(:,j),'Color',colorSim,'LineWidth',1.8,'DisplayName','Simulation');
    plot(Stime,SqdDes(:,j),'--','Color',colorRef,'LineWidth',1.8,'DisplayName','Reference');
    
    ylabel(sprintf('$\\dot{q}_%d\\,[\\mathrm{rad/s}]$', j), 'Interpreter', 'latex');
    if j==1
        title('Joint Velocities');
        legend('Location','best');
    end
    if j==3, xlabel('Time (s)'); end
    
    % --- Vertical lines (no legend entries) ---
    wn_j = cell2mat(cellfun(@(c) c(j), Sdata.wnOpt, 'UniformOutput', false)); 
    t_section1 = 1 / wn_j(1);
    t_section2 = t_section1 + 1 / wn_j(2);
    xline(t_section1,'--k','HandleVisibility','off');
    xline(t_section2,'--k','HandleVisibility','off');
    
    % --- Section labels ---
    y_top = max([PqdAct(:,j); SqdAct(:,j); SqdDes(:,j)]) * .85;
    text(t_section1/2, y_top, 'Section 1', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
    text(t_section1 + (t_section2 - t_section1)/2, y_top, 'Section 2', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
    text((t_section2 + max(Stime))/2, y_top, 'Section 3', 'HorizontalAlignment','center','VerticalAlignment','bottom','Rotation',0,'Color','k');
end

%% --- MINIMUM DISTANCE: TARGET ↔ TRAJECTORIES (mm) ---

nTargets = size(xTarget,1);

dRef = zeros(nTargets,1);   % mm
dSim = zeros(nTargets,1);   % mm
dHw  = zeros(nTargets,1);   % mm

for k = 1:nTargets
    xt = xTarget(k,:);   % target [x y z] in meters

    % Reference trajectory
    dRef(k) = 1000 * min( sqrt( ...
        (SxDes - xt(1)).^2 + ...
        (SyDes - xt(2)).^2 + ...
        (SzDes - xt(3)).^2 ) );

    % Simulation trajectory
    dSim(k) = 1000 * min( sqrt( ...
        (SxAct - xt(1)).^2 + ...
        (SyAct - xt(2)).^2 + ...
        (SzAct - xt(3)).^2 ) );

    % Hardware trajectory
    dHw(k) = 1000 * min( sqrt( ...
        (PxAct - xt(1)).^2 + ...
        (PyAct - xt(2)).^2 + ...
        (PzAct - xt(3)).^2 ) );
end

%% --- print results (mm) ---
fprintf('\nMinimum distance to each target (mm)\n');
fprintf('Target   Reference     Simulation     Hardware\n');
fprintf('-----------------------------------------------\n');
for k = 1:nTargets
    fprintf('%3d     %10.3f     %10.3f     %10.3f\n', ...
        k, dRef(k), dSim(k), dHw(k));
end
