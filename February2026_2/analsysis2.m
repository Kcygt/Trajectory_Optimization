close all
clear 
clc

% Set the range of dataset numbers you want to analyze
startData = 7;
endData = 9;

% Spring constant and desired force
k = 222.9073;
Fdes = -1;
avg_diff = zeros(7,1);

diffPos = zeros(5, endData);

for dataNum = startData:endData
    % Build file and variable names
    dataStr = num2str(dataNum);
    sdataFile = ['Sdata' dataStr '.mat'];
    pdataFile = ['Pdata' dataStr '.mat'];

    % Load Sdata (xTarget and Opt are consistent, so this is safe)
    load(sdataFile, 'xTarget', 'Opt');

    % Load Pdata and extract variable (e.g., Pdata5, Pdata6, etc.)
    pdataVarName = ['Pdata' dataStr];
    tmp = load(pdataFile);  % Load entire .mat file into struct
    Pdata = tmp.(pdataVarName);  % Access the correct field dynamically

    % Time vector
    time = linspace(0, Opt(1), length(Pdata));

    % Forward kinematics
    [xAct, yAct, zAct] = FK(Pdata(:,7), Pdata(:,8), Pdata(:,9));
    [xDes, yDes, zDes] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));
    Fz = Pdata(:,15);

    % Control points
    xCtrl(1,:) = [ Opt(14) Opt(15) Opt(16) ];
    xCtrl(2,:) = [ Opt(17) Opt(18) Opt(19) ];
    xCtrl(3,:) = [ Opt(20) Opt(21) Opt(22) ];

    % --------- PLOT 1: Force vs Time ---------
    figure; grid on; hold on;
    h1 = plot(time, Fz, 'b', 'LineWidth', 1.5); % Force vs Time line

    % Process targets
    indexing = zeros(size(xTarget,1), 1);
    Fact = zeros(size(xTarget,1), 1);
    newTarget = zeros(size(xTarget,1), 1);

    % Marker color for target points
    markerColor = [0.85 0.1 0.1]; % reddish color
    
    for i = 1:size(xTarget, 1)
        distance = sqrt((xAct - xTarget(i,1)).^2 + ...
                        (yAct - xTarget(i,2)).^2 + ...
                        (zAct - xTarget(i,3)).^2);
        [~, idx] = min(distance);
        indexing(i) = idx;
        Fact(i) = Fz(idx);
        newTarget(i) = (Fdes - Fact(i)) / k + yDes(idx);

        % Plot target point with consistent color
        plot(time(idx), Fact(i), '*', 'Color', markerColor, ...
             'MarkerSize', 10, 'LineWidth', 2);
        
        diffPos(i,dataNum) = yDes(indexing(i),1) - yAct(indexing(i),1);
    end

    % Add legend (one entry for all markers)
    h2 = plot(nan, nan, '*', 'Color', markerColor, 'MarkerSize', 10, 'LineWidth', 2); 
    legend([h1, h2], {'Measured Force', 'Force at Target Points'}, 'Location', 'best');

    title('Force vs Time')
    xlabel('Time [s]')
    ylabel('Fz [N]')

    % --------- PLOT 2: 3D Trajectories ---------
    figure; hold on; grid on;
    plot3(xDes, yDes, zDes, 'b', 'LineWidth', 1.5)
    plot3(xAct, yAct, zAct, 'r', 'LineWidth', 1.5)
    plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k')
    plot3(xCtrl(:,1), xCtrl(:,2), xCtrl(:,3), 'gd', 'MarkerFaceColor', 'g')
    plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'k*')

    % Highlight measurement points — bright yellow with black edges
    plot3(xAct(indexing), yAct(indexing), zAct(indexing), 'o', ...
          'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1 0.8 0]);

    % Updated target points
    plot3(xTarget(:,1), newTarget, xTarget(:,3), 'co', 'MarkerFaceColor', 'c')

    title(['3D Trajectory for Dataset ', num2str(dataNum)])
    legend('Desired Trajectory','Actual Trajectory','Home','Control Points', ...
           'Initial Target','Measurement Point','Updated Target', 'Location', 'best')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

end

% Display final newTarget array
newTarget
