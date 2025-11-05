%% dataPlotting.m
clear; clc; close all;

%% ------------------ Load data (FIXED) ------------------
% Expects Sdata5.mat containing Sd.yOpt, and Pdata5.mat containing Pd.Pdata
Sd = load('Sdata5.mat');             % -> Sd.yOpt, Sd.xTarget, Sd.Opt
Pd = load('Pdata5.mat');             % -> Pd.Pdata

SqDes = Sd.yOpt(:,1:3);
SqdDes = Sd.yOpt(:,4:6);
SqAct = Sd.yOpt(:,7:9);
SqdAct = Sd.yOpt(:,10:12);

PqDes  = Pd.Pdata(:,1:3);
PqdDes = Pd.Pdata(:,4:6);
PqAct  = Pd.Pdata(:,7:9);
PqdAct = Pd.Pdata(:,10:12);

%% ------------------ Forward Kinematics ------------------
% FK should accept 3 column-vectors of joint angles and return x,y,z
[SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
[SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));

%% ------------------ Control Points + Spheres (data) ------------------
ctrlPts = [Sd.Opt(14:16); Sd.Opt(17:19); Sd.Opt(20:22)];
radii  = [0.0132, 0.005, 0.0112];

%% ------------------ 3D Cartesian Trajectories ------------------
figure('Name','3D Cartesian Trajectories','Color','w'); 
hold on; grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% --- Trajectories (make thicker and with color distinction) ---
hRef = plot3(SxDes,SyDes,SzDes,'Color',[0 0.6 1],'LineWidth',3);         % Reference (light blue)
hSim = plot3(SxAct,SyAct,SzAct,'-','Color',[0 0 0.8],'LineWidth',3);    % Simulation (dark blue)
hPhn = plot3(PxAct,PyAct,PzAct,'-','Color',[1 0.8 0],'LineWidth',3);     % Phantom (gold/yellow)

% --- Origin (Home Position) ---
hHome = plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',8);

% --- Target points (bright green pentagrams) ---
for i = 1:size(Sd.xTarget,1)
    if i == 1
        hTgt = plot3(Sd.xTarget(i,1), Sd.xTarget(i,2), Sd.xTarget(i,3), ...
                     'p', 'MarkerFaceColor',[0 1 0], 'MarkerEdgeColor','k', 'MarkerSize',12);
    else
        plot3(Sd.xTarget(i,1), Sd.xTarget(i,2), Sd.xTarget(i,3), ...
              'p', 'MarkerFaceColor',[0 1 0], 'MarkerEdgeColor','k', 'MarkerSize',12, ...
              'HandleVisibility','off');
    end
end

% --- Control point markers (solid red diamonds) ---
for i = 1:3
    if i == 1
        hCtrl = plot3(ctrlPts(i,1), ctrlPts(i,2), ctrlPts(i,3), ...
                      'd', 'MarkerFaceColor',[1 0 0], 'MarkerEdgeColor','k', 'MarkerSize',6);
    else
        plot3(ctrlPts(i,1), ctrlPts(i,2), ctrlPts(i,3), ...
              'd', 'MarkerFaceColor',[1 0 0], 'MarkerEdgeColor','k', 'MarkerSize',6, ...
              'HandleVisibility','off');
    end
end

% --- Transparent spheres around control points (red translucent) ---
[xs, ys, zs] = sphere(50);
for i = 1:3
    surf(ctrlPts(i,1) + radii(i)*xs, ...
         ctrlPts(i,2) + radii(i)*ys, ...
         ctrlPts(i,3) + radii(i)*zs, ...
         'FaceAlpha',0.35, 'EdgeColor','none', 'FaceColor',[1 0 0], ...
         'HandleVisibility','off');
end

% --- Plot style and visuals ---
xlabel('X [m]','FontWeight','bold');
ylabel('Y [m]','FontWeight','bold');
zlabel('Z [m]','FontWeight','bold');
title('3D Cartesian Trajectories and Points','FontSize',14,'FontWeight','bold');

axis equal;
view(35,25);                % angled perspective for clarity
camproj('perspective');
rotate3d on;
box on;
lighting gouraud;           % smooth lighting for spheres
camlight('headlight');      % adds depth perception

% --- Legend with clear labels ---
legend([hRef, hSim, hPhn, hHome, hTgt, hCtrl], ...
       {'Reference','Simulation','Phantom','Home Position','Targets','Control Pts'}, ...
       'Location','bestoutside','FontSize',11,'Box','on');

%% ------------------ Joint-space Plots (Separate Figures) ------------------
% Use sample indices by default; replace with Sd.t / Pd.t if available.
tS = 1:size(SqDes,1);
tP = 1:size(PqDes,1);
tS = tS/1000;
tP = tP/1000;
% -------- Positions (Reference, Simulation, Phantom) --------
figure('Name','Joint Positions: Reference vs Simulation vs Phantom','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    plot(tS, SqDes(:,j), 'k-',  'LineWidth', 1.6);   % Reference (desired)
    plot(tS, SqAct(:,j), 'b--', 'LineWidth', 1.4);   % Simulation actual
    plot(tP, PqAct(:,j),  '-','LineWidth', 2.2, 'Color', [1 0.8 0 0.6]);  % Phantom actual
    title(sprintf('Joint %d Position', j));
    ylabel('q [rad]');
    if j == 1
        legend({'Reference','Simulation','Phantom'}, 'Location','best');
    end
end
xlabel('Sample');  % change to 'Time [s]' if using time vectors

% -------- Velocities (Reference, Simulation, Phantom) --------
figure('Name','Joint Velocities: Reference vs Simulation vs Phantom','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    plot(tP, PqdAct(:,j),  '-','LineWidth', 2.2, 'Color', [1 0.8 0 0.6]);  % Phantom actual
    plot(tS, SqdDes(:,j), 'k-',  'LineWidth', 1.6);  % Reference (desired)
    plot(tS, SqdAct(:,j), 'b--', 'LineWidth', 1.4);  % Simulation actual
    title(sprintf('Joint %d Velocity', j));
    ylabel('dq/dt [rad/s]');
    if j == 1
        legend({'Phantom','Reference','Simulation'}, 'Location','best');
    end
end
xlabel('Sample');  % change to 'Time [s]' if using time vectors


%% ------------------ Metrics: RMSE and Min Distances (printed to Command Window) ------------------

% ---- RMSE (Simulation vs Reference / Phantom vs Reference of P's own desired) ----
% Errors (Simulation)
pos_err_S = SqAct - SqDes;           % [rad]
vel_err_S = SqdAct - SqdDes;         % [rad/s]
rmseS_pos_j = sqrt(mean(pos_err_S.^2, 1, 'omitnan'));   % per joint
rmseS_vel_j = sqrt(mean(vel_err_S.^2, 1, 'omitnan'));   % per joint
rmseS_pos_all = sqrt(mean(pos_err_S(:).^2, 'omitnan')); % overall
rmseS_vel_all = sqrt(mean(vel_err_S(:).^2, 'omitnan')); % overall

% Errors (Phantom)
pos_err_P = PqAct - PqDes;           % [rad]
vel_err_P = PqdAct - PqdDes;         % [rad/s]
rmseP_pos_j = sqrt(mean(pos_err_P.^2, 1, 'omitnan'));   % per joint
rmseP_vel_j = sqrt(mean(vel_err_P.^2, 1, 'omitnan'));   % per joint
rmseP_pos_all = sqrt(mean(pos_err_P(:).^2, 'omitnan')); % overall
rmseP_vel_all = sqrt(mean(vel_err_P(:).^2, 'omitnan')); % overall

% ---- Print RMSE to Command Window ----
fprintf('\n==================== RMSE (Joint Space) ====================\n');
fprintf('Units: position in rad, velocity in rad/s\n\n');

fprintf('Simulation vs Reference (Desired):\n');
fprintf('  Position  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad]\n', rmseS_pos_j(1), rmseS_pos_j(2), rmseS_pos_j(3));
fprintf('  Velocity  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad/s]\n', rmseS_vel_j(1), rmseS_vel_j(2), rmseS_vel_j(3));
fprintf('  Overall Position RMSE: %.6f [rad]\n', rmseS_pos_all);
fprintf('  Overall Velocity RMSE: %.6f [rad/s]\n\n', rmseS_vel_all);

fprintf('Phantom vs Reference (Desired for Phantom):\n');
fprintf('  Position  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad]\n', rmseP_pos_j(1), rmseP_pos_j(2), rmseP_pos_j(3));
fprintf('  Velocity  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad/s]\n', rmseP_vel_j(1), rmseP_vel_j(2), rmseP_vel_j(3));
fprintf('  Overall Position RMSE: %.6f [rad]\n', rmseP_pos_all);
fprintf('  Overall Velocity RMSE: %.6f [rad/s]\n', rmseP_vel_all);

% ---- Minimum distance from each target to each trajectory (Reference/Simulation/Phantom) ----
% Trajectory points as Nx3 matrices
XYZ_ref = [SxDes, SyDes, SzDes];   % Reference (desired)
XYZ_sim = [SxAct, SyAct, SzAct];   % Simulation (actual)
XYZ_phn = [PxAct, PyAct, PzAct];   % Phantom (actual)

nT = size(Sd.xTarget,1);
minDist_ref = zeros(nT,1);
minDist_sim = zeros(nT,1);
minDist_phn = zeros(nT,1);

for i = 1:nT
    tgt = Sd.xTarget(i, :);                % 1x3
    d_ref = sqrt(sum( (XYZ_ref - tgt).^2, 2 ));
    d_sim = sqrt(sum( (XYZ_sim - tgt).^2, 2 ));
    d_phn = sqrt(sum( (XYZ_phn - tgt).^2, 2 ));
    minDist_ref(i) = min(d_ref, [], 'omitnan');
    minDist_sim(i) = min(d_sim, [], 'omitnan');
    minDist_phn(i) = min(d_phn, [], 'omitnan');
end

% ---- Print min distances ----
fprintf('\n==================== Minimum Distance to Targets ====================\n');
fprintf('Units: meters (m)\n\n');
fprintf('%6s  %12s  %12s  %12s\n', 'Target', 'Reference', 'Simulation', 'Phantom');
for i = 1:nT
    fprintf('%6d  %12.6f  %12.6f  %12.6f\n', i, minDist_ref(i), minDist_sim(i), minDist_phn(i));
end
fprintf('\nOverall min distance (best across targets):\n');
fprintf('  Reference: %.6f m\n', min(minDist_ref));
fprintf('  Simulation: %.6f m\n', min(minDist_sim));
fprintf('  Phantom: %.6f m\n\n', min(minDist_phn));
