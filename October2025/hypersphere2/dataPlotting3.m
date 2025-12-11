%% ================= Full 3D + Plane Projections + 2D Views =================
clear; clc; close all;

%% ------------------ Load data ------------------
Sd = load('Sdata5.mat');
Pd = load('Pdata5.mat');

SqDes = Sd.yOpt(:,1:3);
SqdDes = Sd.yOpt(:,4:6);
SqAct = Sd.yOpt(:,7:9);
SqdAct = Sd.yOpt(:,10:12);

PqDes  = Pd.Pdata(:,1:3);
PqdDes = Pd.Pdata(:,4:6);
PqAct  = Pd.Pdata(:,7:9);
PqdAct = Pd.Pdata(:,10:12);

%% ------------------ Forward Kinematics ------------------
[SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
[SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));

%% ------------------ Control Points + Spheres ------------------
ctrlPts = [Sd.Opt(14:16); Sd.Opt(17:19); Sd.Opt(20:22)];
radii  = [0.0132, 0.005, 0.0112];

%% ------------------ Visual Settings ------------------
colRef = [0 0 0];
colSim = [0 0.6 0];
colPhn = [1 0.4 0];
colCtrl = [1 0 0];
colBlue = [0 0 1]; % uniform blue

markerSizeHome = 8;
markerSizeTgt  = 12;
markerSizeCtrl = 8;
lineW = 1.6;

%% ------------------ Figure and layout ------------------
figure('Name','3D + Projections','Color','w', 'Units','normalized','Position',[0.05 0.05 0.8 0.8]);
set(gcf,'Renderer','opengl'); % ensure transparency works

% 3 rows x 2 columns: left column = 3D plot spanning 3 rows
tl = tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

%% =================== 3D Plot with Plane Projections ===================
ax3 = nexttile(tl,[3 1]);
hold on; grid on; axis equal; box on;

% Offsets for plane projections
offsetXY = min([SzDes; SzAct; PzAct]) - 0.2;
offsetXZ = min([SyDes; SyAct; PyAct]) + 0.2;
offsetYZ = min([SxDes; SxAct; PxAct]) - 0.2;

% --- Main 3D trajectories ---
plot3(SxDes,SyDes,SzDes,'--','Color',colRef,'LineWidth',2);
plot3(SxAct,SyAct,SzAct,'-','Color',colSim,'LineWidth',2);
plot3(PxAct,PyAct,PzAct,'-','Color',colPhn,'LineWidth',2);

% --- Home ---
plot3(0,0,0,'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeHome);

% --- Targets (main) ---
for i = 1:size(Sd.xTarget,1)
    if i==1
        hTgt = plot3(Sd.xTarget(i,1),Sd.xTarget(i,2),Sd.xTarget(i,3), 'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
    else
        plot3(Sd.xTarget(i,1),Sd.xTarget(i,2),Sd.xTarget(i,3), 'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt,'HandleVisibility','off');
    end
end

% --- Control Points (main) ---
for i = 1:3
    if i==1
        hCtrl = plot3(ctrlPts(i,1),ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
    else
        plot3(ctrlPts(i,1),ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl,'HandleVisibility','off');
    end
end

% --- 3D spheres (main) ---
[xs,ys,zs] = sphere(60);
for i = 1:3
    surf(ctrlPts(i,1)+radii(i)*xs, ctrlPts(i,2)+radii(i)*ys, ctrlPts(i,3)+radii(i)*zs, ...
         'FaceColor', colBlue,'EdgeColor','none','FaceAlpha',0.15);
end

%% ----------------- Trajectory projections on planes -----------------
% --- XY plane (Z = offsetXY) ---
plot3(SxDes,SyDes,offsetXY*ones(size(SxDes)),'--','Color',colRef,'LineWidth',1.2);
plot3(SxAct,SyAct,offsetXY*ones(size(SxAct)),'-','Color',colSim,'LineWidth',1.2);
plot3(PxAct,PyAct,offsetXY*ones(size(PxAct)),'-','Color',colPhn,'LineWidth',1.2);
for i = 1:size(Sd.xTarget,1)
    plot3(Sd.xTarget(i,1),Sd.xTarget(i,2),offsetXY,'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
end
for i = 1:3
    plot3(ctrlPts(i,1),ctrlPts(i,2),offsetXY,'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
    surf(ctrlPts(i,1)+radii(i)*xs, ctrlPts(i,2)+radii(i)*ys, offsetXY + 0*zs, ...
         'FaceColor', colBlue,'EdgeColor','none','FaceAlpha',0.15);
end

% --- XZ plane (Y = offsetXZ) ---
plot3(SxDes,offsetXZ*ones(size(SxDes)),SzDes,'--','Color',colRef,'LineWidth',1.2);
plot3(SxAct,offsetXZ*ones(size(SxAct)),SzAct,'-','Color',colSim,'LineWidth',1.2);
plot3(PxAct,offsetXZ*ones(size(PxAct)),PzAct,'-','Color',colPhn,'LineWidth',1.2);
for i = 1:size(Sd.xTarget,1)
    plot3(Sd.xTarget(i,1),offsetXZ,Sd.xTarget(i,3),'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
end
for i = 1:3
    plot3(ctrlPts(i,1),offsetXZ,ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
    surf(ctrlPts(i,1)+radii(i)*xs, offsetXZ + 0*ys, ctrlPts(i,3)+radii(i)*zs, ...
         'FaceColor', colBlue,'EdgeColor','none','FaceAlpha',0.15);
end

% --- YZ plane (X = offsetYZ) ---
plot3(offsetYZ*ones(size(SyDes)),SyDes,SzDes,'--','Color',colRef,'LineWidth',1.2);
plot3(offsetYZ*ones(size(SyAct)),SyAct,SzAct,'-','Color',colSim,'LineWidth',1.2);
plot3(offsetYZ*ones(size(PyAct)),PyAct,PzAct,'-','Color',colPhn,'LineWidth',1.2);
for i = 1:size(Sd.xTarget,1)
    plot3(offsetYZ,Sd.xTarget(i,2),Sd.xTarget(i,3),'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
end
for i = 1:3
    plot3(offsetYZ,ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
    surf(offsetYZ + 0*xs, ctrlPts(i,2)+radii(i)*ys, ctrlPts(i,3)+radii(i)*zs, ...
         'FaceColor', colBlue,'EdgeColor','none','FaceAlpha',0.15);
end

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectories with XY/XZ/YZ Plane Projections','FontWeight','bold');
view(35,25);

legend('Reference','Simulation','Phantom','Home','Target','Control Points','Location','eastoutside');

%% =================== 2D Projections on Right Column ===================
% XY projection
axXY = nexttile(tl,2);
hold on; grid on; axis equal; box on;
plot(SxDes,SyDes,'--','Color',colRef,'LineWidth',lineW);
plot(SxAct,SyAct,'-','Color',colSim,'LineWidth',lineW);
plot(PxAct,PyAct,'-','Color',colPhn,'LineWidth',lineW);
plot(0,0,'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeHome);
plot(Sd.xTarget(:,1),Sd.xTarget(:,2),'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
plot(ctrlPts(:,1),ctrlPts(:,2),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
for i = 1:3
    hRect = rectangle('Position',[ctrlPts(i,1)-radii(i),ctrlPts(i,2)-radii(i),2*radii(i),2*radii(i)], ...
                      'Curvature',[1 1],'FaceColor',colBlue,'EdgeColor',colBlue,'LineWidth',1.2);
    hRect.FaceAlpha = 0.15;
end
xlabel('X [m]'); ylabel('Y [m]'); title('XY Projection','FontWeight','bold');

% XZ projection
axXZ = nexttile(tl,4);
hold on; grid on; axis equal; box on;
plot(SxDes,SzDes,'--','Color',colRef,'LineWidth',lineW);
plot(SxAct,SzAct,'-','Color',colSim,'LineWidth',lineW);
plot(PxAct,PzAct,'-','Color',colPhn,'LineWidth',lineW);
plot(0,0,'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeHome);
plot(Sd.xTarget(:,1),Sd.xTarget(:,3),'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
plot(ctrlPts(:,1),ctrlPts(:,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
for i = 1:3
    hRect = rectangle('Position',[ctrlPts(i,1)-radii(i),ctrlPts(i,3)-radii(i),2*radii(i),2*radii(i)], ...
                      'Curvature',[1 1],'FaceColor',colBlue,'EdgeColor',colBlue,'LineWidth',1.2);
    hRect.FaceAlpha = 0.15;
end
xlabel('X [m]'); ylabel('Z [m]'); title('XZ Projection','FontWeight','bold');

% YZ projection
axYZ = nexttile(tl,6);
hold on; grid on; axis equal; box on;
plot(SyDes,SzDes,'--','Color',colRef,'LineWidth',lineW);
plot(SyAct,SzAct,'-','Color',colSim,'LineWidth',lineW);
plot(PyAct,PzAct,'-','Color',colPhn,'LineWidth',lineW);
plot(0,0,'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeHome);
plot(Sd.xTarget(:,2),Sd.xTarget(:,3),'p','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',markerSizeTgt);
plot(ctrlPts(:,2),ctrlPts(:,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',markerSizeCtrl);
for i = 1:3
    hRect = rectangle('Position',[ctrlPts(i,2)-radii(i),ctrlPts(i,3)-radii(i),2*radii(i),2*radii(i)], ...
                      'Curvature',[1 1],'FaceColor',colBlue,'EdgeColor',colBlue,'LineWidth',1.2);
    hRect.FaceAlpha = 0.15;
end
xlabel('Y [m]'); ylabel('Z [m]'); title('YZ Projection','FontWeight','bold');

%% ------------------ Adjust limits and title ------------------
xlims = xlim(ax3); set([axXY axXZ], 'XLim', xlims);
sgtitle('Robot Trajectories: 3D + Plane Projections + 2D Views','FontSize',14,'FontWeight','bold');
drawnow;
