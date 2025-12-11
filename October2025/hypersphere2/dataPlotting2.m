%% dataPlotting_withProjections.m
% Enhanced: 3D trajectories, 2D projections, joint-space plots, RMSE, min distances
clear; clc; close all;

%% ------------------ Load data ------------------
% Sdata5.mat: Sd.yOpt, Sd.xTarget, Sd.Opt
% Pdata5.mat: Pd.Pdata
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
try
    [SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
    [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));
catch ME
    error('FK function call failed. Ensure FK.m is on path. Original error:\n%s', ME.message);
end

%% ------------------ Control Points + Spheres ------------------
ctrlPts = [Sd.Opt(14:16); Sd.Opt(17:19); Sd.Opt(20:22)];
radii  = [0.0132, 0.005, 0.0112];

%% ------------------ 3D Cartesian Trajectories ------------------
figure('Name','3D Cartesian Trajectories','Color','w'); 
hold on; grid on; axis equal;
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% Colors
colRef  = [0 0 0];        % black dashed
colSim  = [0 1 0];        % green
colPhn  = [1 0.4 0];      % orange
colHome = [0 0 0];        % black
colCtrl = [1 0 0];        % red
sphereAlpha = 0.35;

% Trajectories
hRef = plot3(SxDes,SyDes,SzDes,'--','Color',colRef,'LineWidth',1.5);
hSim = plot3(SxAct,SyAct,SzAct,'-','Color',colSim,'LineWidth',1.5);
hPhn = plot3(PxAct,PyAct,PzAct,'-','Color',colPhn,'LineWidth',1.5);

% Origin (Home)
hHome = plot3(0,0,0,'ks','MarkerFaceColor',colHome,'MarkerSize',8);

% Targets
for i = 1:size(Sd.xTarget,1)
    if i==1
        hTgt = plot3(Sd.xTarget(i,1),Sd.xTarget(i,2),Sd.xTarget(i,3),'p',...
                     'MarkerFaceColor',colHome,'MarkerEdgeColor','k','MarkerSize',12);
    else
        plot3(Sd.xTarget(i,1),Sd.xTarget(i,2),Sd.xTarget(i,3),'p',...
              'MarkerFaceColor',colHome,'MarkerEdgeColor','k','MarkerSize',12,'HandleVisibility','off');
    end
end

% Control points + spheres
[xs,ys,zs] = sphere(50);
for i=1:3
    if i==1
        hCtrl = plot3(ctrlPts(i,1),ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',6);
    else
        plot3(ctrlPts(i,1),ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',6,'HandleVisibility','off');
    end
    surf(ctrlPts(i,1)+radii(i)*xs, ctrlPts(i,2)+radii(i)*ys, ctrlPts(i,3)+radii(i)*zs, ...
        'FaceAlpha',sphereAlpha,'EdgeColor','none','FaceColor',[1 1 0],'HandleVisibility','off');
end

xlabel('X [m]','FontWeight','bold'); ylabel('Y [m]','FontWeight','bold'); zlabel('Z [m]','FontWeight','bold');
title('3D Cartesian Trajectories','FontSize',14,'FontWeight','bold');
view(35,25); rotate3d on;

legend([hRef,hSim,hPhn,hHome,hTgt,hCtrl],{'Reference','Simulation','Phantom','Home Position','Targets','Control Points'}, ...
       'Location','bestoutside','FontSize',11,'Box','on');

%% ------------------ 2D Projections: XY, XZ, YZ ------------------
theta = linspace(0,2*pi,120);
figure('Name','2D Projections','Color','w','Units','normalized','Position',[0.05 0.05 0.9 0.35]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

% --- XY ---
nexttile; hold on; grid on; box on; axis equal;
plot(SxDes,SyDes,'--','LineWidth',1.6,'Color',colRef);
plot(SxAct,SyAct,'-','LineWidth',1.6,'Color',colSim);
plot(PxAct,PyAct,'-','LineWidth',1.4,'Color',colPhn);
plot(0,0,'ks','MarkerFaceColor',colHome,'MarkerSize',8);

for i=1:size(Sd.xTarget,1)
    plot(Sd.xTarget(i,1),Sd.xTarget(i,2),'p','MarkerFaceColor',colHome,'MarkerEdgeColor','k','MarkerSize',10);
end
for i=1:3
    plot(ctrlPts(i,1),ctrlPts(i,2),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',7);
    fill(ctrlPts(i,1)+radii(i)*cos(theta), ctrlPts(i,2)+radii(i)*sin(theta), colCtrl,'FaceAlpha',0.25,'EdgeColor','none');
end
xlabel('X [m]'); ylabel('Y [m]'); title('XY Projection','FontWeight','bold');
legend({'Reference','Simulation','Phantom','Home','Targets','Control Points'},'Location','bestoutside');

% --- XZ ---
nexttile; hold on; grid on; box on; axis equal;
plot(SxDes,SzDes,'--','LineWidth',1.6,'Color',colRef);
plot(SxAct,SzAct,'-','LineWidth',1.6,'Color',colSim);
plot(PxAct,PzAct,'-','LineWidth',1.4,'Color',colPhn);
plot(0,0,'ks','MarkerFaceColor',colHome,'MarkerSize',8);

for i=1:size(Sd.xTarget,1)
    plot(Sd.xTarget(i,1),Sd.xTarget(i,3),'p','MarkerFaceColor',colHome,'MarkerEdgeColor','k','MarkerSize',10);
end
for i=1:3
    plot(ctrlPts(i,1),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',7);
    fill(ctrlPts(i,1)+radii(i)*cos(theta), ctrlPts(i,3)+radii(i)*sin(theta), colCtrl,'FaceAlpha',0.25,'EdgeColor','none');
end
xlabel('X [m]'); ylabel('Z [m]'); title('XZ Projection','FontWeight','bold');

% --- YZ ---
nexttile; hold on; grid on; box on; axis equal;
plot(SyDes,SzDes,'--','LineWidth',1.6,'Color',colRef);
plot(SyAct,SzAct,'-','LineWidth',1.6,'Color',colSim);
plot(PyAct,PzAct,'-','LineWidth',1.4,'Color',colPhn);
plot(0,0,'ks','MarkerFaceColor',colHome,'MarkerSize',8);

for i=1:size(Sd.xTarget,1)
    plot(Sd.xTarget(i,2),Sd.xTarget(i,3),'p','MarkerFaceColor',colHome,'MarkerEdgeColor','k','MarkerSize',10);
end
for i=1:3
    plot(ctrlPts(i,2),ctrlPts(i,3),'d','MarkerFaceColor',colCtrl,'MarkerEdgeColor','k','MarkerSize',7);
    fill(ctrlPts(i,2)+radii(i)*cos(theta), ctrlPts(i,3)+radii(i)*sin(theta), colCtrl,'FaceAlpha',0.25,'EdgeColor','none');
end
xlabel('Y [m]'); ylabel('Z [m]'); title('YZ Projection','FontWeight','bold');

%% ------------------ Joint-space Plots ------------------
tS = (1:size(SqDes,1))/1000;
tP = (1:size(PqDes,1))/1000;

% Positions
figure('Name','Joint Positions','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for j=1:3
    nexttile; hold on; grid on;
    plot(tS,SqDes(:,j),'k--','LineWidth',2.6);
    plot(tS,SqAct(:,j),'-','LineWidth',1.8,'Color',colSim);
    plot(tP,PqAct(:,j),'-','LineWidth',1.6,'Color',colPhn);
    title(sprintf('Joint %d Position',j));
    ylabel('q [rad]');
    if j==1, legend({'Reference','Simulation','Phantom'},'Location','best'); end
end
xlabel('Sample');

% Velocities
figure('Name','Joint Velocities','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for j=1:3
    nexttile; hold on; grid on;
    plot(tS,SqdDes(:,j),'k--','LineWidth',2.6);
    plot(tS,SqdAct(:,j),'-','LineWidth',1.8,'Color',colSim);
    plot(tP,PqdAct(:,j),'-','LineWidth',1.6,'Color',colPhn);
    title(sprintf('Joint %d Velocity',j));
    ylabel('dq/dt [rad/s]');
    if j==1, legend({'Reference','Simulation','Phantom'},'Location','best'); end
end
xlabel('Sample');

%% ------------------ Metrics ------------------
% RMSE
pos_err_S = SqAct - SqDes; vel_err_S = SqdAct - SqdDes;
rmseS_pos_j = sqrt(mean(pos_err_S.^2,1,'omitnan'));
rmseS_vel_j = sqrt(mean(vel_err_S.^2,1,'omitnan'));
rmseS_pos_all = sqrt(mean(pos_err_S(:).^2,'omitnan'));
rmseS_vel_all = sqrt(mean(vel_err_S(:).^2,'omitnan'));

pos_err_P = PqAct - PqDes; vel_err_P = PqdAct - PqdDes;
rmseP_pos_j = sqrt(mean(pos_err_P.^2,1,'omitnan'));
rmseP_vel_j = sqrt(mean(vel_err_P.^2,1,'omitnan'));
rmseP_pos_all = sqrt(mean(pos_err_P(:).^2,'omitnan'));
rmseP_vel_all = sqrt(mean(vel_err_P(:).^2,'omitnan'));

fprintf('\n==================== RMSE (Joint Space) ====================\n');
fprintf('Simulation vs Reference: Position RMSE [rad]: J1=%.6f, J2=%.6f, J3=%.6f\n',rmseS_pos_j);
fprintf('Velocity RMSE [rad/s]: J1=%.6f, J2=%.6f, J3=%.6f\n',rmseS_vel_j);
fprintf('Overall Position RMSE: %.6f, Velocity RMSE: %.6f\n',rmseS_pos_all,rmseS_vel_all);
fprintf('Phantom vs Reference: Position RMSE [rad]: J1=%.6f, J2=%.6f, J3=%.6f\n',rmseP_pos_j);
fprintf('Velocity RMSE [rad/s]: J1=%.6f, J2=%.6f, J3=%.6f\n',rmseP_vel_j);
fprintf('Overall Position RMSE: %.6f, Velocity RMSE: %.6f\n',rmseP_pos_all,rmseP_vel_all);

% Minimum distances to targets
XYZ_ref = [SxDes,SyDes,SzDes];
XYZ_sim = [SxAct,SyAct,SzAct];
XYZ_phn = [PxAct,PyAct,PzAct];

nT = size(Sd.xTarget,1);
minDist_ref = zeros(nT,1); minDist_sim = zeros(nT,1); minDist_phn = zeros(nT,1);

for i=1:nT
    tgt = Sd.xTarget(i,:);
    minDist_ref(i) = min(sqrt(sum((XYZ_ref - tgt).^2,2)),'omitnan');
    minDist_sim(i) = min(sqrt(sum((XYZ_sim - tgt).^2,2)),'omitnan');
    minDist_phn(i) = min(sqrt(sum((XYZ_phn - tgt).^2,2)),'omitnan');
end

fprintf('\n==================== Minimum Distance to Targets ====================\n');
fprintf('%6s  %12s  %12s  %12s\n','Target','Reference','Simulation','Phantom');
for i=1:nT
    fprintf('%6d  %12.6f  %12.6f  %12.6f\n',i,minDist_ref(i),minDist_sim(i),minDist_phn(i));
end
fprintf('Overall min distance: Reference=%.6f m, Simulation=%.6f m, Phantom=%.6f m\n', ...
        min(minDist_ref), min(minDist_sim), min(minDist_phn));

%% ------------------ End of Script ------------------
