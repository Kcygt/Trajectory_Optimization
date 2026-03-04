% process_and_plot_3D_only.m
clear; clc; close all;

%% --- dataset index ---
i = 5;
Pfile = sprintf('Pdata%d.mat', i);
Sfile = sprintf('Sdata%d.mat', i);

%% --- load data ---
Pdata = load(Pfile);
Sdata = load(Sfile);

%% --- extract data ---
SqDes  = Sdata.yOpt(:,1:3);
SqAct  = Sdata.yOpt(:,7:9);
PqAct  = Pdata.Pdata(:,7:9);

SqdDes  = Sdata.yOpt(:,4:6);
SqdAct  = Sdata.yOpt(:,10:12);
PqdAct  = Pdata.Pdata(:,10:12);

Stime = Sdata.tOpt;
Ptime = linspace(0, Stime(end), size(PqAct,1));

xTarget = Sdata.xTarget(1:5,:);

%% --- forward kinematics ---
[SxAct,SyAct,SzAct] = FK(SqAct(:,1), SqAct(:,2), SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1), PqAct(:,2), PqAct(:,3));
[SxDes,SyDes,SzDes] = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));

%% --- colors ---
colorSim     = [0 1 0]; 
colorPhantom = [1 .5 0];
colorRef     = [0 0 0];

%% ============================================================
%% --- Figure 2: Joint Positions (WITH INSET ZOOM) ---
%% ============================================================

figure('Name',sprintf('DataSet%d - Joint Positions',i),...
       'Position',[150 150 900 750]);

jointNames = {'Joint 1','Joint 2','Joint 3'};

% ===== Zoom window (EDIT HERE) =====
zoomStart = 2;     % seconds
zoomEnd   = 3;     % seconds
% ====================================

for j = 1:3
    
    axMain = subplot(3,1,j);
    hold on; grid on;
    
    plot(Ptime,PqAct(:,j),'Color',colorPhantom,'LineWidth',1.8);
    plot(Stime,SqAct(:,j),'Color',colorSim,'LineWidth',1.8);
    plot(Stime,SqDes(:,j),'--','Color',colorRef,'LineWidth',1.8);
    
    ylabel(sprintf('%s (rad)',jointNames{j}));
    
    if j==1
        title('Joint Positions');
        legend('Hardware','Simulation','Reference','Location','best');
    end
    if j==3
        xlabel('Time (s)');
    end
    
    % ---- Draw zoom rectangle on main plot ----
    yLimits = ylim;
    rectangle('Position',[zoomStart, yLimits(1), ...
              zoomEnd-zoomStart, range(yLimits)],...
              'EdgeColor','k','LineStyle','--');
    
    %% ===== Inset axes (robust positioning) =====
    pos = axMain.Position;
    
    axInset = axes('Position',[pos(1)+0.60*pos(3), ...
                               pos(2)+0.55*pos(4), ...
                               0.30*pos(3), ...
                               0.35*pos(4)]);
    box on; hold on;
    
    plot(axInset,Ptime,PqAct(:,j),'Color',colorPhantom,'LineWidth',1);
    plot(axInset,Stime,SqAct(:,j),'Color',colorSim,'LineWidth',1);
    plot(axInset,Stime,SqDes(:,j),'--','Color',colorRef,'LineWidth',1);
    
    xlim(axInset,[zoomStart zoomEnd]);
    ylim(axInset,'auto');
    
    axInset.XTickLabel = [];
    axInset.YTickLabel = [];
end


%% ============================================================
%% --- Figure 3: Joint Velocities (WITH INSET ZOOM) ---
%% ============================================================

figure('Name',sprintf('DataSet%d - Joint Velocities',i),...
       'Position',[150 150 900 750]);

for j = 1:3
    
    axMain = subplot(3,1,j);
    hold on; grid on;
    
    plot(Ptime,PqdAct(:,j),'Color',colorPhantom,'LineWidth',1.8);
    plot(Stime,SqdAct(:,j),'Color',colorSim,'LineWidth',1.8);
    plot(Stime,SqdDes(:,j),'--','Color',colorRef,'LineWidth',1.8);
    
    ylabel(sprintf('%s (rad/s)',jointNames{j}));
    
    if j==1
        title('Joint Velocities');
        legend('Hardware','Simulation','Reference','Location','best');
    end
    if j==3
        xlabel('Time (s)');
    end
    
    % ---- Draw zoom rectangle ----
    yLimits = ylim;
    rectangle('Position',[zoomStart, yLimits(1), ...
              zoomEnd-zoomStart, range(yLimits)],...
              'EdgeColor','k','LineStyle','--');
    
    %% ===== Inset axes =====
    pos = axMain.Position;
    
    axInset = axes('Position',[pos(1)+0.60*pos(3), ...
                               pos(2)+0.55*pos(4), ...
                               0.30*pos(3), ...
                               0.35*pos(4)]);
    box on; hold on;
    
    plot(axInset,Ptime,PqdAct(:,j),'Color',colorPhantom,'LineWidth',1);
    plot(axInset,Stime,SqdAct(:,j),'Color',colorSim,'LineWidth',1);
    plot(axInset,Stime,SqdDes(:,j),'--','Color',colorRef,'LineWidth',1);
    
    xlim(axInset,[zoomStart zoomEnd]);
    ylim(axInset,'auto');
    
    axInset.XTickLabel = [];
    axInset.YTickLabel = [];
end