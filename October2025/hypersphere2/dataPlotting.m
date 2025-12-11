%% dataPlotting_3Donly.m
% Standalone script for 3D Cartesian Trajectories with targets and control points
clear; clc; close all;

%% ------------------ Load data ------------------
% Expects Sdata5.mat containing Sd.yOpt, Sd.xTarget, Sd.Opt
%         Pdata5.mat containing Pd.Pdata
Sd = load('Sdata5.mat');  % -> Sd.yOpt, Sd.xTarget, Sd.Opt
Pd = load('Pdata5.mat');  % -> Pd.Pdata

% Extract joint positions
SqDes = Sd.yOpt(:,1:3);
SqAct = Sd.yOpt(:,7:9);
PqAct = Pd.Pdata(:,7:9);

%% ------------------ Forward Kinematics ------------------
% Replace with your real FK.m if available
try
    [SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
    [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));
catch
    warning('FK.m not found. Using dummy FK for testing.');
    [SxDes,SyDes,SzDes] = dummyFK(SqDes);
    [SxAct,SyAct,SzAct] = dummyFK(SqAct);
    [PxAct,PyAct,PzAct] = dummyFK(PqAct);
end

%% ------------------ Control Points + Spheres ------------------
ctrlPts = [Sd.Opt(14:16); Sd.Opt(17:19); Sd.Opt(20:22)];
radii  = [0.0132, 0.005, 0.0112];

%% ------------------ 3D Cartesian Trajectories ------------------
figure('Name','3D Cartesian Trajectories','Color','w'); 
hold on; grid on; axis equal;

% --- Trajectories ---
plot3(SxDes,SyDes,SzDes,'--','Color',[0 0 0],'LineWidth',1.5); % Reference
plot3(SxAct,SyAct,SzAct,'-','Color',[0 1 0],'LineWidth',1.5);   % Simulation
plot3(PxAct,PyAct,PzAct,'-','Color',[1 0.4 0],'LineWidth',1.5); % Phantom

% --- Home Position ---
plot3(0,0,0,'ks','MarkerFaceColor','k','MarkerSize',8);

% --- Target Points ---
for i = 1:size(Sd.xTarget,1)
    plot3(Sd.xTarget(i,1), Sd.xTarget(i,2), Sd.xTarget(i,3), ...
          'p','MarkerFaceColor',[0 0 0],'MarkerEdgeColor','k','MarkerSize',12);
end

% --- Control Points & Spheres ---
[xs, ys, zs] = sphere(50); % precompute sphere for all
for i = 1:3
    % Control point marker
    plot3(ctrlPts(i,1), ctrlPts(i,2), ctrlPts(i,3), 'd', ...
          'MarkerFaceColor',[1 0 0],'MarkerEdgeColor','k','MarkerSize',6);
    % Sphere
    surf(ctrlPts(i,1) + radii(i)*xs, ...
         ctrlPts(i,2) + radii(i)*ys, ...
         ctrlPts(i,3) + radii(i)*zs, ...
         'FaceAlpha',0.35,'EdgeColor','none','FaceColor',[1 1 0]);
end

% --- Labels and Title ---
xlabel('X [m]','FontWeight','bold'); 
ylabel('Y [m]','FontWeight','bold'); 
zlabel('Z [m]','FontWeight','bold');
title('3D Cartesian Trajectories and Points','FontSize',14,'FontWeight','bold');

% --- View and interactivity ---
view(35,25); 
rotate3d on;

% --- Legend ---
legend({'Reference','Simulation','Phantom','Home Position','Targets','Control Points'}, ...
       'Location','bestoutside');

%% ------------------ Dummy FK (for testing only) ------------------
function [x,y,z] = dummyFK(q)
% Simple planar 3-DOF forward kinematics for testing
L1 = 0.1; L2 = 0.08; L3 = 0.06;
q1 = q(:,1); q2 = q(:,2); q3 = q(:,3);
n = size(q,1);
x = zeros(n,1); y = zeros(n,1); z = zeros(n,1);
for k = 1:n
    th1 = q1(k); th2 = q2(k); th3 = q3(k);
    x(k) = L1*cos(th1) + L2*cos(th1+th2) + L3*cos(th1+th2+th3);
    y(k) = L1*sin(th1) + L2*sin(th1+th2) + L3*sin(th1+th2+th3);
    z(k) = 0; % planar motion
end
end
