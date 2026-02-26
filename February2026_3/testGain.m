close all
clear
clc

%% ================= COLORS =================
colorSim      = [0.0 0.8 0.0];  % Reference trajectory
colorRef      = [0 0 0];
homeColor     = [0 0 0];
ctrlColor     = [0.9 0.3 0.5];
sphereColor   = [0.2 0.2 1.0];

initTargetColor = [1.0 0.0 0.0];
updTargetColor  = [0.0 1.0 1.0];

radius = [0.0098, 0.006, 0.012];

%% ================= PALETTE FOR HARDWARE TRAJECTORIES =================
palette         = lines(6);
colorPhantom    = palette(6,:); % Normal speed
colorPhantom2x  = palette(4,:); % 2x speed
colorPhantom4x  = palette(5,:); % 4x speed

trajColors = {colorPhantom, colorPhantom2x, colorPhantom4x};

%% ================= DATA RANGE & PARAMETERS =================
startData = 7;
endData   = 9;
numData   = endData - startData + 1;

k    = 222.9073;
Fdes = -1;

%% ================= LEGEND LABELS =================
trajLabels = {
    'Optimized trajectory'
    '2x Speed trajectory'
    '4x Speed Trajectory'
};

forceLegendLabels = {
    'Measured Force (normal speed)'
    'Measured Force (2x speed)'
    'Measured Force (4x speed)'
};

targetLegendLabels = {
    'Target points (normal speed)'
    'Target points (2x speed)'
    'Target points (4x speed)'
};

%% ================= HANDLE STORAGE =================
hForceAll  = gobjects(numData,1);
hTargetAll = gobjects(numData,1);
hAct3D     = gobjects(numData,1);
hAct2D     = gobjects(numData,1);

%% ================= MAIN LOOP =================
for dataNum = startData:endData

    idx = dataNum - startData + 1;

    %% -------- tLoad Data --------
    dataStr = num2str(dataNum);
    load(['Sdata' dataStr '.mat'], 'xTarget', 'Opt');
    tmp = load(['Pdata' dataStr '.mat']);
    Pdata = tmp.(['Pdata' dataStr]);
    
    time = linspace(0, Opt(1), length(Pdata));

    %% -------- Forward Kinematics --------
    % Forward kinematics
    [xAct, yAct, zAct] = FK(Pdata(:,7), Pdata(:,8), Pdata(:,9));
    [xDes, yDes, zDes] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));
    Fz = Pdata(:,15);

    % xCtrl = [Opt(14:16); Opt(17:19); Opt(20:22)];

    numTargets = size(xTarget,1);
    if dataNum == startData
        targetForces = zeros(numTargets, numData); % store forces at targets
    end

    %% ================= FIGURE 1: FORCE =================
    figure(1); hold on; grid on;
    
    
    

    hForceAll(idx) = plot(time, Fz, ...
        'Color', trajColors{idx}, 'LineWidth',1.5);

    newTarget = zeros(numTargets,1);

    for i = 1:numTargets
        distance = sqrt((xAct - xTarget(i,1)).^2 + ...
                        (yAct - xTarget(i,2)).^2 + ...
                        (zAct - xTarget(i,3)).^2);
        [~, idxNearest] = min(distance);

        % Compute updated target (if needed)
        newTarget(i) = (Fdes - Fz(idxNearest)) / k + yDes(idxNearest);

        % Store the force at this target point
        targetForces(i, idx) = Fz(idxNearest);

        % Plot target point
        plot(time(idxNearest), Fz(idxNearest), 'o', ...
             'MarkerFaceColor', trajColors{idx}, ...
             'MarkerEdgeColor', colorRef, ...
             'MarkerSize', 8, 'LineWidth', 1.5);
    end

    % Dummy handle for target legend
    hTargetAll(idx) = plot(NaN,NaN,'o', ...
        'MarkerFaceColor', trajColors{idx}, ...
        'MarkerEdgeColor', colorRef, ...
        'MarkerSize',8,'LineWidth',1.5);

    %% ================= FIGURE 2: 3D =================
    figure(2); hold on; grid on;

    if dataNum == startData
        hDes = plot3(xDes,yDes,zDes,'Color',colorSim,'LineWidth',2);
        hHome = plot3(0,0,0,'o','MarkerFaceColor',homeColor);
        % hCtrl = plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'d',...
            % 'MarkerFaceColor',ctrlColor,'MarkerEdgeColor',colorRef);
        hInit = plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'o',...
            'MarkerFaceColor',initTargetColor);
    end

    hAct3D(idx) = plot3(xAct,yAct,zAct,'Color',trajColors{idx},'LineWidth',2);

    % Updated targets plotted but not in legend
    plot3(xTarget(:,1), newTarget, xTarget(:,3), 'o', ...
        'MarkerFaceColor', updTargetColor, 'MarkerEdgeColor', colorRef);

    % [sx,sy,sz] = sphere(30);
    % for i = 1:3
    %     surf(radius(i)*sx+xCtrl(i,1), ...
    %          radius(i)*sy+xCtrl(i,2), ...
    %          radius(i)*sz+xCtrl(i,3), ...
    %          'FaceColor',sphereColor,'FaceAlpha',0.35, ...
    %          'EdgeColor','none','HandleVisibility','off');
    % end
    % 
    % hRegion = plot3(NaN,NaN,NaN,'o','MarkerFaceColor',sphereColor,'MarkerEdgeColor','none');

    axis equal
    view(135,25)
    camlight headlight
    lighting gouraud
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Cartesian Space Trajectories')

    %% ================= FIGURE 3: 2D =================
    figure(3); hold on; grid on;

    if dataNum == startData
        hHome2 = plot(0,0,'o','MarkerFaceColor',homeColor);
        hDes2  = plot(xDes,yDes,'Color',colorSim,'LineWidth',2);
        hInit2 = plot(xTarget(:,1),xTarget(:,2),'o','MarkerFaceColor',initTargetColor);
        % hCtrl2 = plot(xCtrl(:,1),xCtrl(:,2),'d','MarkerFaceColor',ctrlColor);
    end

    hAct2D(idx) = plot(xAct,yAct,'Color',trajColors{idx},'LineWidth',2);

    % Updated targets plotted but not in legend
    plot(xTarget(:,1), newTarget,'o','MarkerFaceColor',updTargetColor);
    % 
    % for i = 1:3
    %     th = linspace(0,2*pi,200);
    %     fill(xCtrl(i,1)+radius(i)*cos(th), ...
    %          xCtrl(i,2)+radius(i)*sin(th), ...
    %          sphereColor,'FaceAlpha',0.25,'EdgeColor','none');
    % end

    % hRegion2 = plot(NaN,NaN,'o','MarkerFaceColor',sphereColor,'MarkerEdgeColor','none');

    axis equal
    xlabel('X [m]'); ylabel('Y [m]');
    title('2D Cartesian Space Trajectories')

end

%% ================= FINAL LEGENDS =================

% ---- Figure 1
figure(1)
legend([hForceAll; hTargetAll], ...
       [forceLegendLabels(:); targetLegendLabels(:)], ...
       'Location','best')
title('Measured Force Along the Z-Axis')
xlabel('Time [s]')
ylabel('Fz [N]')

% ---- Figure 2
figure(2)
legend([hDes; hAct3D; hHome;  hInit], ...
       [{'Reference Trajectory'}; trajLabels(:); ...
        {'Home';'Control Points';'Initial Targets';'Control Point Sphere'}], ...
       'Location','best')

% ---- Figure 3
figure(3)
legend([hHome2; hDes2; hAct2D; hInit2], ...
       [{'Home';'Reference Trajectory'}; trajLabels(:); ...
        {'Initial Targets';'Control Points';'Control Point Sphere'}], ...
       'Location','best')

%% ================= DISPLAY FORCES AT TARGETS =================
disp('Force at target points for each speed:');
for i = 1:numTargets
    fprintf('Target %d: ', i);
    fprintf('%.3f N (normal), %.3f N (2x), %.3f N (4x)\n', targetForces(i,:));
end


% axinsert=axes;
% axinsert.Position=[.5 .12 .2 .15];
% plot(axinsert,hDes2.XData,hDes2.YData)
% axinsert.XTickLabel='';
% axinsert.YTickLabel='';