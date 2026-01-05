close all
clear
clc

%% ================= HIGH-VISIBILITY COLOR DEFINITIONS =================
colorSim     = [0.0 0.8 0.0];     % Desired trajectory
colorPhantom = [1.0 0.4 0.0];     % Actual trajectory
colorRef     = [0 0 0];
homeColor    = [0 0 0];
ctrlColor    = [0.9 0.3 0.5];
sphereColor  = [0.2 0.2 1.0];

initTargetColor = [1.0 0.0 0.0];  % Initial target
updTargetColor  = [0.0 1.0 1.0];  % Updated target

speedLabels = {'1× Speed','1.5× Speed','2× Speed','4× Speed'};
speedColors = [
    0.0 0.8 0.0
    0.0 0.6 1.0
    1.0 0.6 0.0
    0.8 0.0 0.8 ];

radius = [0.009 0.005 0.0085];

%% ================= DATA RANGE =================
startData = 1;
endData   = 4;

%% ================= CONTROL PARAMETERS =================
k    = 222.9073;
Fdes = -1;

diffPos = zeros(5,endData);

%% ================= FIGURE 1: FORCE vs TIME =================
figure(1); clf
tiledlayout(2,2,'Padding','compact','TileSpacing','compact')

%% ================= FIGURE 2: 3D TRAJECTORIES =================
figure(2); clf
tiledlayout(2,2,'Padding','compact','TileSpacing','compact')

%% ================= FIGURE 3: Y vs X (OVERLAY) =================
figure(3); clf; hold on; grid on

for dataNum = startData:endData

    %% ---------------- LOAD DATA ----------------
    load(['Sdata' num2str(dataNum) '.mat'],'xTarget','Opt')
    tmp = load(['Pdata' num2str(dataNum) '.mat']);
    Pdata = tmp.(['Pdata' num2str(dataNum)]);

    time = linspace(0,Opt(1),length(Pdata));

    [xAct,yAct,zAct] = FK(Pdata(:,1),Pdata(:,2),Pdata(:,3));
    [xDes,yDes,zDes] = FK(Pdata(:,10),Pdata(:,11),Pdata(:,12));
    Fz = Pdata(:,6);

    xCtrl = [Opt(14:16); Opt(17:19); Opt(20:22)];

    %% ================= FORCE TARGET COMPUTATION =================
    indexing = zeros(size(xTarget,1),1);
    Fact     = zeros(size(xTarget,1),1);
    newTarget = zeros(size(xTarget,1),1);

    for i = 1:size(xTarget,1)
        dist = sqrt((xAct-xTarget(i,1)).^2 + ...
                    (yAct-xTarget(i,2)).^2 + ...
                    (zAct-xTarget(i,3)).^2);
        [~,idx] = min(dist);
        indexing(i) = idx;
        Fact(i) = Fz(idx);
        newTarget(i) = (Fdes - Fact(i))/k + yDes(idx);
        diffPos(i,dataNum) = yDes(idx) - yAct(idx);
    end

    %% ================= FIGURE 1 =================
    figure(1)
    nexttile; hold on; grid on

    plot(time,Fz,'Color',speedColors(dataNum,:), 'LineWidth',1.8)
    plot(time(indexing),Fact,'o','MarkerFaceColor',initTargetColor,...
        'MarkerEdgeColor','k','MarkerSize',7)

    title(speedLabels{dataNum},'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('F_z [N]')
    set(gca,'FontSize',11,'LineWidth',1.1)

    %% ================= FIGURE 2 =================
    figure(2)
    nexttile; hold on; grid on

    plot3(xDes,yDes,zDes,'k--','LineWidth',1.8)
    plot3(xAct,yAct,zAct,'Color',speedColors(dataNum,:), 'LineWidth',2)
    plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',5)
    plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'d',...
        'MarkerFaceColor',ctrlColor,'MarkerEdgeColor','k')

    [sx,sy,sz] = sphere(25);
    for i = 1:size(xCtrl,1)
        surf(radius(i)*sx+xCtrl(i,1),...
             radius(i)*sy+xCtrl(i,2),...
             radius(i)*sz+xCtrl(i,3),...
             'FaceColor',sphereColor,'FaceAlpha',0.25,...
             'EdgeColor','none')
    end

    axis equal
    view(135,25)
    xlabel('X'); ylabel('Y'); zlabel('Z')
    title(speedLabels{dataNum},'FontWeight','bold')
    set(gca,'FontSize',11,'LineWidth',1.1)

    %% ================= FIGURE 3 =================
    figure(3)
    plot(xAct,yAct,'Color',speedColors(dataNum,:), 'LineWidth',2)

end

%% ================= FINALIZE FIGURES =================
figure(1)
sgtitle('Measured Force vs Time for Different Execution Speeds','FontSize',14,'FontWeight','bold')

figure(2)
sgtitle('3D End-Effector Trajectories at Different Speeds','FontSize',14,'FontWeight','bold')

figure(3)
plot(xDes,yDes,'k--','LineWidth',2)
plot(xCtrl(:,1),xCtrl(:,2),'kd','MarkerFaceColor',ctrlColor,'MarkerSize',8)
legend(['Desired',speedLabels,'Control Points'],'Location','best')
axis equal
xlabel('X'); ylabel('Y')
title('Y–X Trajectory Comparison at Different Speeds','FontWeight','bold')
set(gca,'FontSize',12,'LineWidth',1.2)
