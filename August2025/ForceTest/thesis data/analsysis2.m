close all
clear 
clc

% Set the range of dataset numbers you want to analyze
startData = 1;
endData = 3;

% Spring constant and desired force
k = 532.3389;
Fdes = -1;

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

    % Now use Pdata normally
    time = linspace(0, Opt(1), length(Pdata));

    % Continue using Pdata as before
    [xAct, yAct, zAct] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));
    [xDes, yDes, zDes] = FK(Pdata(:,4), Pdata(:,5), Pdata(:,6));
    Fz = Pdata(:,9);

    % Control points (static)
    xCtrl(1,:) = [ Opt(14) Opt(15) Opt(16) ];
    xCtrl(2,:) = [Opt(17) Opt(18) Opt(19)];
    xCtrl(3,:) = [ Opt(20) Opt(21) Opt(22) ];

    % Plot force vs time
    figure; grid on; hold on;
    plot(time, Fz)
    title(['Force vs Time for Dataset ', num2str(dataNum)])
    xlabel('Time [s]')
    ylabel('Fz [N]')

    % Process targets
    indexing = zeros(size(xTarget,1), 1);
    Fact = zeros(size(xTarget,1), 1);
    newTarget = zeros(size(xTarget,1), 1);

    for i = 1:size(xTarget, 1)
        distance = sqrt((xAct - xTarget(i,1)).^2 + (yAct - xTarget(i,2)).^2 + (zAct - xTarget(i,3)).^2);
        [~, idx] = min(distance);
        indexing(i) = idx;
        Fact(i) = Fz(idx);
        newTarget(i) = (Fdes - Fact(i)) / k + xTarget(i,2);

        plot(time(idx), Fact(i), '*', 'MarkerSize', 10, 'LineWidth', 2)
    end

    % Plot 3D trajectories
    figure; hold on; grid on;
    plot3(xDes, yDes, zDes, 'b')
    plot3(xAct, yAct, zAct, 'r')
    plot3(0, 0, 0, 'ko')
    plot3(xCtrl(:,1), xCtrl(:,2), xCtrl(:,3), 'gd')
    plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'k*')
    plot3(xAct(indexing), yAct(indexing), zAct(indexing), 'm.', 'MarkerSize', 15)
    plot3(xTarget(:,1), newTarget, xTarget(:,3), 'co')

    title(['3D Trajectory for Dataset ', num2str(dataNum)])
    legend('Desired Trajectory','Actual Trajectory','Home','Control Points','Initial Target','Min Distance','Updated Target')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
end
