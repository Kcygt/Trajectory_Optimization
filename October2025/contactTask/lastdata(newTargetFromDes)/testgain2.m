close all
clear
clc

%% ================= COLORS =================
colorSim      = [0.0 0.8 0.0];
colorRef      = [0 0 0];
homeColor     = [0 0 0];
ctrlColor     = [0.9 0.3 0.5];
sphereColor   = [0.2 0.2 1.0];

initTargetColor = [1.0 0.0 0.0];
updTargetColor  = [0.0 1.0 1.0];

radius = [0.0098, 0.006, 0.012];

%% ================= PALETTE FOR HARDWARE TRAJECTORIES =================
palette         = lines(6);
colorPhantom    = palette(6,:);   % Initial
colorPhantom2x  = palette(4,:);   % Optimized
colorPhantom4x  = palette(5,:);   % 2x speed
colorPhantom8x  = palette(2,:);   % 4x speed

trajColors = {colorPhantom, colorPhantom2x, colorPhantom4x, colorPhantom8x};

%% ================= DATA RANGE & PARAMETERS =================
startData = 1;
endData   = 4;
numData   = endData - startData + 1;

k    = 222.9073;
Fdes = -1;

%% ================= LEGEND LABELS =================
trajLabels = {
    'Initial Trajectory'
    'Optimized Trajectory'
    '2x Speed Trajectory'
    '4x Speed Trajectory'
};

forceLegendLabels = {
    'Initial Measured Force'
    'Optimized Measured Force'
    '2x Speed Measured Force'
    '4x Speed Measured Force'
};

targetForceLegendLabels = {
    'Initial Target Force'
    'Optimized Target Force'
    '2x Speed Target Force'
    '4x Speed Target Force'
};

%% ================= HANDLE STORAGE =================
hForceAll  = gobjects(numData,1);
hTargetAll = gobjects(numData,1);
hAct3D     = gobjects(numData,1);
hAct2D     = gobjects(numData,1);

%% ================= MAIN LOOP =================
for dataNum = startData:endData

    idx = dataNum - startData + 1;

    %% -------- Load Data --------
    dataStr = num2str(dataNum);
    load(['Sdata' dataStr '.mat'], 'xTarget', 'Opt');
    tmp = load(['Pdata' dataStr '.mat']);
    Pdata = tmp.(['Pdata' dataStr]);

    time = linspace(0, Opt(1), length(Pdata));

    %% -------- Forward Kinematics --------
    [xAct, yAct, zAct] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));
    [xDes, yDes, zDes] = FK(Pdata(:,10), Pdata(:,11), Pdata(:,12));
    Fz = Pdata(:,6);

    xCtrl = [Opt(14:16); Opt(17:19); Opt(20:22)];

    numTargets = size(xTarget,1);
    if dataNum == startData
        targetForces = zeros(numTargets, numData);
    end

    %% ================= FIGURE 1: FORCE =================
    figure(1); hold on; grid on;

    hForceAll(idx) = plot(time, Fz, ...
        'Color', trajColors{idx}, 'LineWidth', 1.5);

    newTarget = zeros(numTargets,1);

    for i = 1:numTargets
        distance = sqrt((xAct - xTarget(i,1)).^2 + ...
                        (yAct - xTarget(i,2)).^2 + ...
                        (zAct - xTarget(i,3)).^2);
        [~, idxNearest] = min(distance);

        newTarget(i) = (Fdes - Fz(idxNearest)) / k + yDes(idxNearest);
        targetForces(i, idx) = Fz(idxNearest);

        % Target force dot on plot (hidden from legend individually)
        plot(time(idxNearest), Fz(idxNearest), 'o', ...
             'MarkerFaceColor', trajColors{idx}, ...
             'MarkerEdgeColor', colorRef, ...
             'MarkerSize', 8, 'LineWidth', 1.5, ...
             'HandleVisibility', 'off');
    end

    % Proxy handle for target force legend entry
    hTargetAll(idx) = plot(NaN, NaN, 'o', ...
        'MarkerFaceColor', trajColors{idx}, ...
        'MarkerEdgeColor', colorRef, ...
        'MarkerSize', 8, 'LineWidth', 1.5);

    %% ================= FIGURE 2: 3D =================
    figure(2); hold on; grid on;

    if dataNum == startData

        % Reference trajectory: black dashed
        hDes  = plot3(xDes, yDes, zDes, ...
            'Color', colorRef, 'LineStyle', '--', 'LineWidth', 2);
        hHome = plot3(0, 0, 0, 'o', ...
            'MarkerFaceColor', homeColor, 'MarkerEdgeColor', homeColor);
        hCtrl = plot3(xCtrl(:,1), xCtrl(:,2), xCtrl(:,3), 'd', ...
            'MarkerFaceColor', ctrlColor, 'MarkerEdgeColor', colorRef);
        hInit = plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'o', ...
            'MarkerFaceColor', initTargetColor, 'MarkerEdgeColor', colorRef);

        % Spheres: only once (first cycle)
        [sx, sy, sz] = sphere(30);
        for i = 1:3
            surf(radius(i)*sx + xCtrl(i,1), ...
                 radius(i)*sy + xCtrl(i,2), ...
                 radius(i)*sz + xCtrl(i,3), ...
                 'FaceColor', sphereColor, 'FaceAlpha', 0.35, ...
                 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end

        hRegion   = plot3(NaN, NaN, NaN, 'o', ...
            'MarkerFaceColor', sphereColor, 'MarkerEdgeColor', 'none');
        hUpdTgt3D = plot3(NaN, NaN, NaN, 'o', ...
            'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef);

    end

    hAct3D(idx) = plot3(xAct, yAct, zAct, ...
        'Color', trajColors{idx}, 'LineWidth', 2);

    % Updated targets (hidden from legend)
    plot3(xTarget(:,1), newTarget, xTarget(:,3), 'o', ...
        'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef, ...
        'HandleVisibility', 'off');

    axis equal
    view(135, 25)
    camlight headlight
    lighting gouraud
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Cartesian Space Trajectories')

    %% ================= FIGURE 3: 2D =================
    figure(3); hold on; grid on;

    if dataNum == startData

        hHome2 = plot(0, 0, 'o', ...
            'MarkerFaceColor', homeColor, 'MarkerEdgeColor', homeColor);

        % Reference trajectory: black dashed
        hDes2  = plot(xDes, yDes, ...
            'Color', colorRef, 'LineStyle', '--', 'LineWidth', 2);
        hInit2 = plot(xTarget(:,1), xTarget(:,2), 'o', ...
            'MarkerFaceColor', initTargetColor, 'MarkerEdgeColor', colorRef);
        hCtrl2 = plot(xCtrl(:,1), xCtrl(:,2), 'd', ...
            'MarkerFaceColor', ctrlColor, 'MarkerEdgeColor', colorRef);

        % Filled circles: only once (first cycle)
        for i = 1:3
            th = linspace(0, 2*pi, 200);
            fill(xCtrl(i,1) + radius(i)*cos(th), ...
                 xCtrl(i,2) + radius(i)*sin(th), ...
                 sphereColor, 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
                 'HandleVisibility', 'off');
        end

        hRegion2  = plot(NaN, NaN, 'o', ...
            'MarkerFaceColor', sphereColor, 'MarkerEdgeColor', 'none');
        hUpdTgt2D = plot(NaN, NaN, 'o', ...
            'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef);

    end

    hAct2D(idx) = plot(xAct, yAct, 'Color', trajColors{idx}, 'LineWidth', 2);

    % Updated targets (hidden from legend)
    plot(xTarget(:,1), newTarget, 'o', ...
        'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef, ...
        'HandleVisibility', 'off');

    axis equal
    xlabel('X [m]'); ylabel('Y [m]');
    title('2D Cartesian Space Trajectories')

end

%% ================= FINAL LEGENDS =================

% ---- Figure 1 --------
figure(1)
title('Measured Force Along the Z-Axis')
xlabel('Time [s]')
ylabel('Fz [N]')

legendHandles1 = gobjects(numData*2, 1);
legendLabels1  = cell(numData*2, 1);
for i = 1:numData
    legendHandles1(2*i-1) = hForceAll(i);
    legendHandles1(2*i)   = hTargetAll(i);
    legendLabels1{2*i-1}  = forceLegendLabels{i};
    legendLabels1{2*i}    = targetForceLegendLabels{i};
end
legend(legendHandles1, legendLabels1, 'Location', 'best')

% ---- Figure 2: 3D --------
figure(2)
legend( ...
    [hDes; hAct3D; hHome; hCtrl; hInit; hUpdTgt3D; hRegion], ...
    [{'Reference Trajectory'}; trajLabels(:); ...
     {'Home'; 'Control Points'; 'Initial Targets'; ...
      'Updated Targets'; 'Control Point Sphere'}], ...
    'Location', 'best')

% ---- Figure 3: 2D --------
figure(3)
legend( ...
    [hDes2; hAct2D; hHome2; hCtrl2; hInit2; hUpdTgt2D; hRegion2], ...
    [{'Reference Trajectory'}; trajLabels(:); ...
     {'Home'; 'Control Points'; 'Initial Targets'; ...
      'Updated Targets'; 'Control Point Sphere'}], ...
    'Location', 'best')

%% ================= DISPLAY FORCES AT TARGETS =================
disp('Force at target points for each trajectory:');
for i = 1:numTargets
    fprintf('Target %d:  %.3f N (initial),  %.3f N (optimized),  %.3f N (2x),  %.3f N (4x)\n', ...
        i, targetForces(i,1), targetForces(i,2), targetForces(i,3), targetForces(i,4));
end