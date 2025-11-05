%% dataPlotting_multiPhantom_timeSec.m
clear; clc; close all;

%% ------------------ Load data ------------------
% Expects:
%   Sdata5.mat  -> Sd.yOpt (12 cols), Sd.xTarget (Nx3), Sd.Opt, optional Sd.t (ms)
%   Pdata5.mat  -> Pd.Pdata (12 cols), optional Pd.t (ms)   [Phantom 1× speed]
%   Pdata6.mat  -> Pd.Pdata (12 cols), optional Pd.t (ms)   [Phantom 2× speed]
%   Pdata7.mat  -> Pd.Pdata (12 cols), optional Pd.t (ms)   [Phantom 4× speed]
Sd  = load('Sdata5.mat');
Pd1 = load('Pdata5.mat');   % 1× speed
Pd2 = load('Pdata6.mat');   % 2× speed
Pd3 = load('Pdata7.mat');   % 4× speed

%% ------------------ Unpack Simulation (reference & actual) ------------------
SqDes  = Sd.yOpt(:,1:3);
SqdDes = Sd.yOpt(:,4:6);
SqAct  = Sd.yOpt(:,7:9);
SqdAct = Sd.yOpt(:,10:12);

%% ------------------ Unpack Phantom runs ------------------
unpackP = @(Pd) deal(Pd.Pdata(:,1:3), Pd.Pdata(:,4:6), Pd.Pdata(:,7:9), Pd.Pdata(:,10:12));
[P1_qDes, P1_qdDes, P1_qAct, P1_qdAct] = unpackP(Pd1);
[P2_qDes, P2_qdDes, P2_qAct, P2_qdAct] = unpackP(Pd2);
[P3_qDes, P3_qdDes, P3_qAct, P3_qdAct] = unpackP(Pd3);

% Collect for convenient looping
PqDes_all  = {P1_qDes,  P2_qDes,  P3_qDes};
PqdDes_all = {P1_qdDes, P2_qdDes, P3_qdDes};
PqAct_all  = {P1_qAct,  P2_qAct,  P3_qAct};
PqdAct_all = {P1_qdAct, P2_qdAct, P3_qdAct};
Pds_all    = {Pd1, Pd2, Pd3};

phLabels   = {'Phantom (1×)','Phantom (2×)','Phantom (4×)'};
phColors3D = [ ...
    1.00 0.80 0.00;   % gold  (1×)
    0.00 0.65 0.00;   % green (2×)
    0.60 0.00 0.80];  % purple(4×)

%% ------------------ Time vectors in seconds ------------------
% If Sd.t / Pd*.t exist (ms), convert to seconds; otherwise fall back to sample index /1000
if isfield(Sd, 't') && ~isempty(Sd.t)
    tS = Sd.t(:) / 1000;         % [s]
else
    tS = (1:size(SqDes,1))' / 1000;
end

tP = cell(1,3);
for k = 1:3
    PdK = Pds_all{k};
    if isfield(PdK, 't') && ~isempty(PdK.t)
        tP{k} = PdK.t(:) / 1000; % [s]
    else
        tP{k} = (1:size(PqDes_all{k},1))' / 1000;
    end
end

%% ------------------ Forward Kinematics ------------------
% FK should accept 3 column-vectors of joint angles and return x,y,z
[SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
[SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));

PxAct_all = cell(1,3); PyAct_all = cell(1,3); PzAct_all = cell(1,3);
for k = 1:3
    qA = PqAct_all{k};
    [PxAct_all{k}, PyAct_all{k}, PzAct_all{k}] = FK(qA(:,1), qA(:,2), qA(:,3));
end

%% ------------------ Control Points + Spheres (data) ------------------
ctrlPts = [Sd.Opt(14:16); Sd.Opt(17:19); Sd.Opt(20:22)];
radii  = [0.0132, 0.005, 0.0112];

%% ------------------ 3D Cartesian Trajectories ------------------
figure('Name','3D Cartesian Trajectories','Color','w'); 
hold on; grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% Trajectories
hRef = plot3(SxDes,SyDes,SzDes,'Color',[0 0.6 1],'LineWidth',3);       % Reference
hSim = plot3(SxAct,SyAct,SzAct,'-','Color',[0 0 0.8],'LineWidth',3);   % Simulation

% Phantom (three speeds)
hPh = gobjects(1,3);
for k = 1:3
    hPh(k) = plot3(PxAct_all{k}, PyAct_all{k}, PzAct_all{k}, '-', ...
        'Color', [phColors3D(k,:) 0.75], 'LineWidth', 3);
end

% Origin (Home Position)
hHome = plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',8);

% Target points (bright green pentagrams)
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

% Control point markers (solid red diamonds)
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

% Transparent spheres around control points (red translucent)
[xs, ys, zs] = sphere(50);
for i = 1:3
    surf(ctrlPts(i,1) + radii(i)*xs, ...
         ctrlPts(i,2) + radii(i)*ys, ...
         ctrlPts(i,3) + radii(i)*zs, ...
         'FaceAlpha',0.35, 'EdgeColor','none', 'FaceColor',[1 0 0], ...
         'HandleVisibility','off');
end

% Style
xlabel('X [m]','FontWeight','bold');
ylabel('Y [m]','FontWeight','bold');
zlabel('Z [m]','FontWeight','bold');
title('3D Cartesian Trajectories and Points','FontSize',14,'FontWeight','bold');

axis equal;
view(35,25);
camproj('perspective');
rotate3d on; box on;
lighting gouraud; camlight('headlight');

% Legend
legend([hRef, hSim, hPh(1), hPh(2), hPh(3), hHome, hTgt, hCtrl], ...
       [{'Reference','Simulation'}, phLabels, {'Home Position','Targets','Control Pts'}], ...
       'Location','bestoutside','FontSize',11,'Box','on');

%% ------------------ Joint-space: Positions ------------------
figure('Name','Joint Positions: Reference vs Simulation vs Phantom Speeds','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    plot(tS, SqDes(:,j), 'k-',  'LineWidth', 1.6);   % Reference (desired)
    plot(tS, SqAct(:,j), 'b--', 'LineWidth', 1.4);   % Simulation (actual)
    for k = 1:3
        plot(tP{k}, PqAct_all{k}(:,j),  '-', 'LineWidth', 1.8, ...
            'Color', [phColors3D(k,:) 0.60]);        % Phantom actuals
    end
    title(sprintf('Joint %d Position', j));
    ylabel('q [rad]');
    if j == 1
        legend([{'Reference','Simulation'}, phLabels], 'Location','best');
    end
end
xlabel('Time [s]');

%% ------------------ Joint-space: Velocities ------------------
figure('Name','Joint Velocities: Reference vs Simulation vs Phantom Speeds','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for j = 1:3
    nexttile; hold on; grid on;
    plot(tS, SqdDes(:,j), 'k-',  'LineWidth', 1.6);  % Reference (desired)
    plot(tS, SqdAct(:,j), 'b--', 'LineWidth', 1.4);  % Simulation (actual)
    for k = 1:3
        plot(tP{k}, PqdAct_all{k}(:,j),  '-', 'LineWidth', 1.8, ...
            'Color', [phColors3D(k,:) 0.35]);        % Phantom actuals
    end
    title(sprintf('Joint %d Velocity', j));
    ylabel('dq/dt [rad/s]');
    if j == 1
        legend([{'Reference','Simulation'}, phLabels], 'Location','best');
    end
end
xlabel('Time [s]');

%% ------------------ Metrics: RMSE (Joint Space) ------------------
fprintf('\n==================== RMSE (Joint Space) ====================\n');
fprintf('Units: position in rad, velocity in rad/s\n\n');

% Simulation vs Reference
pos_err_S = SqAct - SqDes;
vel_err_S = SqdAct - SqdDes;
rmseS_pos_j   = sqrt(mean(pos_err_S.^2, 1, 'omitnan'));
rmseS_vel_j   = sqrt(mean(vel_err_S.^2, 1, 'omitnan'));
rmseS_pos_all = sqrt(mean(pos_err_S(:).^2, 'omitnan'));
rmseS_vel_all = sqrt(mean(vel_err_S(:).^2, 'omitnan'));

fprintf('Simulation vs Reference (Desired):\n');
fprintf('  Position  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad]\n', rmseS_pos_j(1), rmseS_pos_j(2), rmseS_pos_j(3));
fprintf('  Velocity  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad/s]\n', rmseS_vel_j(1), rmseS_vel_j(2), rmseS_vel_j(3));
fprintf('  Overall Position RMSE: %.6f [rad]\n', rmseS_pos_all);
fprintf('  Overall Velocity RMSE: %.6f [rad/s]\n\n', rmseS_vel_all);

% Each Phantom vs its own desired
for k = 1:3
    pos_err_P = PqAct_all{k} - PqDes_all{k};
    vel_err_P = PqdAct_all{k} - PqdDes_all{k};
    rmseP_pos_j   = sqrt(mean(pos_err_P.^2, 1, 'omitnan'));
    rmseP_vel_j   = sqrt(mean(vel_err_P.^2, 1, 'omitnan'));
    rmseP_pos_all = sqrt(mean(pos_err_P(:).^2, 'omitnan'));
    rmseP_vel_all = sqrt(mean(vel_err_P(:).^2, 'omitnan'));

    fprintf('%s vs Reference (Desired for that run):\n', phLabels{k});
    fprintf('  Position  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad]\n', rmseP_pos_j(1), rmseP_pos_j(2), rmseP_pos_j(3));
    fprintf('  Velocity  RMSE per joint:  J1 = %.6f,  J2 = %.6f,  J3 = %.6f  [rad/s]\n', rmseP_vel_j(1), rmseP_vel_j(2), rmseP_vel_j(3));
    fprintf('  Overall Position RMSE: %.6f [rad]\n', rmseP_pos_all);
    fprintf('  Overall Velocity RMSE: %.6f [rad/s]\n\n', rmseP_vel_all);
end

%% ------------------ Minimum distance to targets ------------------
XYZ_ref = [SxDes, SyDes, SzDes];     % Reference (desired)
XYZ_sim = [SxAct, SyAct, SzAct];     % Simulation (actual)

nT = size(Sd.xTarget,1);
minDist_ref = zeros(nT,1);
minDist_sim = zeros(nT,1);
minDist_ph  = zeros(nT,3);

% Precompute Phantom XYZs
XYZ_ph_all = cell(1,3);
for k = 1:3
    XYZ_ph_all{k} = [PxAct_all{k}, PyAct_all{k}, PzAct_all{k}];
end

for i = 1:nT
    tgt = Sd.xTarget(i, :);                % 1x3
    d_ref = sqrt(sum( (XYZ_ref - tgt).^2, 2 ));
    d_sim = sqrt(sum( (XYZ_sim - tgt).^2, 2 ));
    minDist_ref(i) = min(d_ref, [], 'omitnan');
    minDist_sim(i) = min(d_sim, [], 'omitnan');
    for k = 1:3
        d_ph = sqrt(sum( (XYZ_ph_all{k} - tgt).^2, 2 ));
        minDist_ph(i,k) = min(d_ph, [], 'omitnan');
    end
end

% Print min distances
fprintf('\n==================== Minimum Distance to Targets ====================\n');
fprintf('Units: meters (m)\n\n');
fprintf('%6s  %12s  %12s  %12s  %12s  %12s\n', 'Target', 'Reference', 'Simulation', 'Ph 1×', 'Ph 2×', 'Ph 4×');
for i = 1:nT
    fprintf('%6d  %12.6f  %12.6f  %12.6f  %12.6f  %12.6f\n', i, ...
        minDist_ref(i), minDist_sim(i), minDist_ph(i,1), minDist_ph(i,2), minDist_ph(i,3));
end
fprintf('\nOverall min distance (best across targets):\n');
fprintf('  Reference: %.6f m\n',  min(minDist_ref));
fprintf('  Simulation: %.6f m\n', min(minDist_sim));
fprintf('  Phantom 1×: %.6f m\n', min(minDist_ph(:,1)));
fprintf('  Phantom 2×: %.6f m\n', min(minDist_ph(:,2)));
fprintf('  Phantom 4×: %.6f m\n\n', min(minDist_ph(:,3)));
