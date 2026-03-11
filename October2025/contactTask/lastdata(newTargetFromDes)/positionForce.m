close all
clear
clc

% Dataset range
startData = 2;
endData   = 4;

for dataNum = startData:endData

    %% Load Data
    dataStr = num2str(dataNum);

    load(['Sdata' dataStr '.mat'], 'Opt');
    tmp = load(['Pdata' dataStr '.mat']);
    Pdata = tmp.(['Pdata' dataStr]);

    %% Forward Kinematics
    [xAct, yAct, zAct] = FK(Pdata(:,1), Pdata(:,2), Pdata(:,3));

    %% Force Data
    Fz = Pdata(:,6);

    %% Select ii based on dataset
    if dataNum == 2
        ii = 3000;
    elseif dataNum == 3
        ii = 1450;
    elseif dataNum == 4
        ii = 950;
    end

    %% Plot Force vs Position
    figure(1)
    hold on
    grid on
    
    plot(xAct, Fz, 'LineWidth', 1.5)

end

xlabel('Position X [m]')
ylabel('Fz [N]')
title('Force vs Position')
legend('Normal Speed','2x speed-up','4x speed-up')