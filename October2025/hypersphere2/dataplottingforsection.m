% process_and_plot_3D_only.m
clear; clc; close all;

%% --- dataset index ---
i = 5;
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

xTarget = Sdata.xTarget(1:5,:); % first 5 targets

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

%% ============================================================
%% --- SECTION DEFINITIONS ---
%% ============================================================
sectionTimes  = [0.91 2.14 3.47];
sectionLabels = {'Section 1','Section 2','Section 3','Section 4'};
sectionColor  = [0.3 0.3 0.3];

%% --- Figure 1: 3D Cartesian ---
figure('Name',sprintf('DataSet%d - 3D Cartesian',i),'Position',[100 100 1200 800]);
hold on; grid on; view(3); axis equal;
xlabel('$X$ (m)','Interpreter','latex');
ylabel('$Y$ (m)','Interpreter','latex');
zlabel('$Z$ (m)','Interpreter','latex');
title('Cartesian Space Position');

xlim([-0.03 0.16]);
ylim([-0.06 0.03]);
zlim([-0.03 0.05]);

zLimits = zlim;
yLimits = ylim;
planes.z = zLimits(1);
planes.y = yLimits(2);

h = gobjects(0); 
labels = {};

h(end+1) = plot3(0,0,0,'o','MarkerFaceColor',homeColor,...
    'MarkerEdgeColor','k','MarkerSize',10); 
labels{end+1} = 'Home';

h(end+1) = plot3(SxAct,SyAct,SzAct,'-','LineWidth',2,'Color',colorSim);
labels{end+1} = 'Simulation';

h(end+1) = plot3(PxAct,PyAct,PzAct,'-','LineWidth',2,'Color',colorPhantom);
labels{end+1} = 'Hardware';

h(end+1) = plot3(SxDes,SyDes,SzDes,'--','LineWidth',1.5,'Color',colorRef);
labels{end+1} = 'Reference';

for k = 1:5
    hTarget(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
        'p','MarkerSize',10,'MarkerFaceColor',targetColor,...
        'MarkerEdgeColor',targetColor);
end
h(end+1) = hTarget(1);
labels{end+1} = 'Targets';

legend(h,labels,'Location','eastoutside');
hold off;

%% ============================================================
%% --- Joint Positions ---
%% ============================================================
figure('Name',sprintf('DataSet%d - Joint Positions',i),'Position',[150 150 800 700]);
jointNames = {'Joint 1','Joint 2','Joint 3'};

for j = 1:3
    subplot(3,1,j); hold on; grid on;

    plot(Ptime,PqAct(:,j),'Color',colorPhantom,'LineWidth',1.8);
    plot(Stime,SqAct(:,j),'Color',colorSim,'LineWidth',1.8);
    plot(Stime,SqDes(:,j),'--','Color',colorRef,'LineWidth',1.8);

    yl = ylim;

    % --- section lines ---
    for s = 1:length(sectionTimes)
        xline(sectionTimes(s),'--','Color',sectionColor,...
            'LineWidth',1.2,'HandleVisibility','off');
    end

    % --- section labels ---
    text(sectionTimes(1)/2, yl(2)*0.95, sectionLabels{1}, ...
        'HorizontalAlignment','center','FontWeight','bold');

    for s = 1:length(sectionTimes)
        if s < length(sectionTimes)
            xm = mean(sectionTimes(s:s+1));
        else
            xm = mean([sectionTimes(s), Stime(end)]);
        end
        text(xm, yl(2)*0.95, sectionLabels{s+1}, ...
            'HorizontalAlignment','center','FontWeight','bold');
    end

    ylabel(sprintf('%s (rad)',jointNames{j}));

    if j==1
        title('Joint Positions');
        legend('Hardware','Simulation','Reference','Location','best');
    end
    if j==3
        xlabel('Time (s)');
    end
end

%% ============================================================
%% --- Joint Velocities ---
%% ============================================================
figure('Name',sprintf('DataSet%d - Joint Velocities',i),'Position',[150 150 800 700]);

for j = 1:3
    subplot(3,1,j); hold on; grid on;

    plot(Ptime,PqdAct(:,j),'Color',colorPhantom,'LineWidth',1.8);
    plot(Stime,SqdAct(:,j),'Color',colorSim,'LineWidth',1.8);
    plot(Stime,SqdDes(:,j),'--','Color',colorRef,'LineWidth',1.8);

    yl = ylim;

    % --- section lines ---
    for s = 1:length(sectionTimes)
        xline(sectionTimes(s),'--','Color',sectionColor,...
            'LineWidth',1.2,'HandleVisibility','off');
    end

    % --- section labels ---
    text(sectionTimes(1)/2, yl(2)*0.95, sectionLabels{1}, ...
        'HorizontalAlignment','center','FontWeight','bold');

    for s = 1:length(sectionTimes)
        if s < length(sectionTimes)
            xm = mean(sectionTimes(s:s+1));
        else
            xm = mean([sectionTimes(s), Stime(end)]);
        end
        text(xm, yl(2)*0.95, sectionLabels{s+1}, ...
            'HorizontalAlignment','center','FontWeight','bold');
    end

    ylabel(sprintf('%s (rad/s)',jointNames{j}));

    if j==1
        title('Joint Velocities');
        legend('Hardware','Simulation','Reference','Location','best');
    end
    if j==3
        xlabel('Time (s)');
    end
end
