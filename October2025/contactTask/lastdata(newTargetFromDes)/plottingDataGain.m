

%% --- Load data ---
Pdata     = load(Pfile);    % expects field Pdata.Pdata
Sdata     = load(Sfile);    % expects fields yOpt, tOpt, xTarget, Opt
Pdata2x   = load(Pfile2x);
Pdata4x   = load(Pfile4x);


close all
clear
clc

%% ================= HIGH-VISIBILITY COLOR DEFINITIONS =================
colorSim     = [0.0 0.8 0.0];     % bright green (desired trajectory)
colorPhantom = [1.0 0.4 0.0];     % strong orange (actual trajectory)
colorRef     = [0 0 0];           % black
homeColor    = [0 0 0];           % home
ctrlColor    = [0.9 0.3 0.5];     % purple (control points)
sphereColor  = [0.2 0.2 1.0];     % vivid blue (control regions)

initTargetColor = [1.0 0.0 0.0];  % RED (initial targets)
updTargetColor  = [0.0 1.0 1.0];  % CYAN (updated targets)

% Radii of spheres around control points
radius = [0.009, 0.005, 0.0085];
radius = [0.009, 0.005, 0.0085];
radius = [0.0098, 0.006, 0.012];

% Dataset range
startData = 3;
endData   = 4;

% Spring constant and desired force
k = 222.9073;
Fdes = -1;

diffPos = zeros(5, endData);

for dataNum = startData:endData

    %% ---------------- Load Data ----------------
    dataStr = num2str(dataNum);
    load(['Sdata' dataStr '.mat'], 'xTarget', 'Opt');
    tmp = load(['Pdata' dataStr '.mat']);
    Pdata = tmp.(['Pdata' dataStr]);

    % Time vector
    time = linspace(0, Opt(1), length(Pdata));

    % Forward kinematics
    [xAct, yAct, zAct] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));
    [xDes, yDes, zDes] = FK(Pdata(:,10), Pdata(:,11), Pdata(:,12));
    Fz = Pdata(:,6);

    % Control points
    xCtrl = [Opt(14:16); Opt(17:19); Opt(20:22)];

    %% ---------------- PLOT 1: Force vs Time ----------------
    figure; hold on; grid on;

    hForce = plot(time, Fz, 'Color', sphereColor, 'LineWidth', 1.5);

    indexing  = zeros(size(xTarget,1), 1);
    Fact      = zeros(size(xTarget,1), 1);
    newTarget = zeros(size(xTarget,1), 1);

    for i = 1:size(xTarget,1)
        distance = sqrt((xAct - xTarget(i,1)).^2 + ...
                        (yAct - xTarget(i,2)).^2 + ...
                        (zAct - xTarget(i,3)).^2);
        [~, idx] = min(distance);

        indexing(i) = idx;
        Fact(i) = Fz(idx);
        newTarget(i) = (Fdes - Fact(i)) / k + yDes(idx);

        plot(time(idx), Fact(i), 'o', ...
            'MarkerFaceColor', initTargetColor, ...
            'MarkerEdgeColor', colorRef, ...
            'MarkerSize', 8, 'LineWidth', 1.5);

        diffPos(i,dataNum) = yDes(idx) - yAct(idx);
    end

    hTarget = plot(nan, nan, 'o', ...
        'MarkerFaceColor', initTargetColor, ...
        'MarkerEdgeColor', colorRef, ...
        'MarkerSize', 8, 'LineWidth', 1.5);

    legend([hForce hTarget], {'Measured Force','Force at Target Points'}, 'Location','best')
    title('Measured Force Along the Z-Axis')
    xlabel('Time [s]')
    ylabel('Fz [N]')

    %% ---------------- PLOT 2: 3D Trajectories ----------------
    figure; hold on; grid on;

    hDes = plot3(xDes, yDes, zDes, 'Color', colorSim, 'LineWidth', 2);
    hAct = plot3(xAct, yAct, zAct, 'Color', colorPhantom, 'LineWidth', 2);
    hHome = plot3(0,0,0,'o','MarkerFaceColor',homeColor,'MarkerEdgeColor',homeColor,'MarkerSize',6);
    hCtrl = plot3(xCtrl(:,1), xCtrl(:,2), xCtrl(:,3), 'd', ...
        'MarkerFaceColor', ctrlColor, 'MarkerEdgeColor', colorRef, 'MarkerSize', 8);
    hInit = plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'o', ...
        'MarkerFaceColor', initTargetColor, 'MarkerEdgeColor', colorRef, 'MarkerSize', 6, 'LineWidth',1.5);
    hUpd  = plot3(xTarget(:,1), newTarget, xTarget(:,3), 'o', ...
        'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef, 'MarkerSize', 6, 'LineWidth',1.5);

    % Control spheres
    [sx, sy, sz] = sphere(30);
    for i = 1:size(xCtrl,1)
        surf(radius(i)*sx + xCtrl(i,1), ...
             radius(i)*sy + xCtrl(i,2), ...
             radius(i)*sz + xCtrl(i,3), ...
             'FaceColor', sphereColor, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'HandleVisibility','off');
    end

    % Projection planes (hidden)
    planes.z = -0.04; % x-y plane
    planes.y = 0.04;  % x-z plane
    plot3(xDes, yDes, planes.z*ones(size(zDes)), ':', 'Color', colorSim, 'LineWidth', 1.2, 'HandleVisibility','off');
    plot3(xAct, yAct, planes.z*ones(size(zAct)), ':', 'Color', colorPhantom, 'LineWidth', 1.2, 'HandleVisibility','off');
    plot3(xDes, planes.y*ones(size(yDes)), zDes, ':', 'Color', colorSim, 'LineWidth', 1.2, 'HandleVisibility','off');
    plot3(xAct, planes.y*ones(size(yAct)), zAct, ':', 'Color', colorPhantom, 'LineWidth', 1.2, 'HandleVisibility','off');

    % Target projections (hidden)
    for i = 1:size(xTarget,1)
        xt = xTarget(i,:);
        plot3(xt(1), xt(2), planes.z, 'o', 'MarkerFaceColor',[.6 .6 .6], 'MarkerEdgeColor','none','HandleVisibility','off')
        plot3(xt(1), planes.y, xt(3), 'o', 'MarkerFaceColor',[.6 .6 .6], 'MarkerEdgeColor','none','HandleVisibility','off')
    end

    % Control point projections (hidden)
    for i = 1:size(xCtrl,1)
        plot3(xCtrl(i,1), xCtrl(i,2), planes.z, 'd', 'MarkerFaceColor', ctrlColor, 'MarkerEdgeColor', colorRef, 'HandleVisibility','off')
        surf(radius(i)*sx + xCtrl(i,1), radius(i)*sy + xCtrl(i,2), planes.z+0*sz, ...
            'FaceColor', sphereColor, 'FaceAlpha', 0.2, 'EdgeColor','none','HandleVisibility','off')
        plot3(xCtrl(i,1), planes.y, xCtrl(i,3), 'd', 'MarkerFaceColor', ctrlColor, 'MarkerEdgeColor', colorRef, 'HandleVisibility','off')
        surf(radius(i)*sx + xCtrl(i,1), 0*sy + planes.y, radius(i)*sz + xCtrl(i,3), ...
            'FaceColor', sphereColor, 'FaceAlpha', 0.2, 'EdgeColor','none','HandleVisibility','off')
    end

    % Dummy handle for Control Region legend
    hRegion = plot3(NaN, NaN, NaN, 'o', 'MarkerFaceColor', sphereColor, ...
                    'MarkerEdgeColor', 'none', 'MarkerSize', 10, 'LineWidth', 1.5);

    % Legend
    legend([hDes hAct hHome hCtrl hInit hUpd hRegion], ...
           {'Reference Trajectory','Hardware Trajectory','Home','Control Points', ...
            'Initial Targets','Updated Targets','Control Point Sphere'}, 'Location','best')

    % Axis & view
    axis equal
    view(135,25)
    lighting gouraud
    camlight headlight
    set(gca,'FontSize',12,'LineWidth',1.2)
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(' Cartesian Space Trajectories ')

    %% ---------------- PLOT 3: Y vs X Trajectories ----------------
    figure; hold on; grid on;
    hHome = plot(0,0,'o', 'MarkerFaceColor', [0 0 0], ...
                  'MarkerEdgeColor', colorRef, 'MarkerSize',6,'LineWidth',1.5);
    hDes2 = plot(xDes, yDes, 'Color', colorSim, 'LineWidth', 2);
    hAct2 = plot(xAct, yAct, 'Color', colorPhantom, 'LineWidth', 2);
    hInit2 = plot(xTarget(:,1), xTarget(:,2), 'o', 'MarkerFaceColor', initTargetColor, ...
                  'MarkerEdgeColor', colorRef, 'MarkerSize',6,'LineWidth',1.5);
    hUpd2  = plot(xTarget(:,1), newTarget, 'o', 'MarkerFaceColor', updTargetColor, ...
                  'MarkerEdgeColor', colorRef, 'MarkerSize',6,'LineWidth',1.5);
    hCtrl2 = plot(xCtrl(:,1), xCtrl(:,2), 'd', 'MarkerFaceColor', ctrlColor, ...
                  'MarkerEdgeColor', colorRef, 'MarkerSize',8);

    % Projected control regions
    theta = linspace(0,2*pi,200);
    for i = 1:size(xCtrl,1)
        fill(xCtrl(i,1) + radius(i)*cos(theta), xCtrl(i,2) + radius(i)*sin(theta), ...
             sphereColor, 'FaceAlpha',0.25, 'EdgeColor','none')
    end

    % Dummy handle for Control Region legend
    hRegion2 = plot(NaN, NaN, 'o', 'MarkerEdgeColor', sphereColor, ...
                    'MarkerFaceColor', sphereColor, 'MarkerSize',10, 'LineWidth',2);

    % Legend
    legend([hHome hDes2 hAct2 hInit2 hUpd2 hCtrl2 hRegion2], ...
           {'Home position','Reference Trajectory','Hardware Trajectory','Initial Targets','Updated Targets','Control Points','Control Point Sphere'}, ...
           'Location','best')

    axis equal
    set(gca,'FontSize',12,'LineWidth',1.2)
    title('2D Cartesian Space Trajectories')
    xlabel('X [m]'); ylabel('Y [m]')

end

newTarget
