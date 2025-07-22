close all
% publish('plottingData.m', 'html');
for i = 16:16
    % Construct filenames
    PdataFile = sprintf('Pdata%d.mat', i);
    SdataFile = sprintf('data%d.mat', i);
    
    % Load data
    Pdata = load(PdataFile);
    Sdata = load(SdataFile);
    
    % Call plotting function with dataset label
    plotPhantomSimulation(Pdata, Sdata, sprintf('DataSet%d', i));
end

function plotPhantomSimulation(Pdata, Sdata, figPrefix)
    
    % Phantom Desired and Actual 
    PqDes = Pdata.Pdata(:,1:3);
    PqdDes = Pdata.Pdata(:,4:6);
    PqAct = Pdata.Pdata(:,7:9);
    PqdAct = Pdata.Pdata(:,10:12);

    % Extract data
    Stime = Sdata.tOpt;
    Ptime = linspace(0, Stime(end), length(PqAct));

    % Simulation Desired and Actual 
    SqDes = Sdata.yOpt(:,1:3);
    SqdDes = Sdata.yOpt(:,4:6);
    SqAct = Sdata.yOpt(:,7:9);
    SqdAct = Sdata.yOpt(:,10:12);

    xTarget = Sdata.xTarget;
    [SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
    [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));

    % Cartesian Space Position Figure
    fig1 = figure; hold on; grid on;
    plot(0, 0, 'o', 'LineWidth', 2, 'MarkerSize', 5)       % Start Point
    plot(0.05, 0.05, 'o', 'LineWidth', 2, 'MarkerSize', 5) % End Point
    plot(SyDes, SzDes, 'b', 'LineWidth', 2)               % Desired Position
    plot(SyAct, SzAct, 'r', 'LineWidth', 2)               % Simulation Trajectory
    plot(PyAct, PzAct, 'g', 'LineWidth', 2)               % Phantom Trajectory

    targetHandles = gobjects(size(xTarget,1),1);
    for k = 1:size(xTarget,1)
        targetHandles(k) = plot(xTarget(k,2), xTarget(k,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end
    targetLabels = arrayfun(@(i) sprintf('Target Point %d', i), 1:size(xTarget,1), 'UniformOutput', false);
    legendLabels = [{'Start Point', 'End Point', 'Desired Position', 'Simulation Trajectory', 'Phantom Trajectory'}, targetLabels];
    legend(legendLabels, 'Location', 'eastoutside')
    title('Cartesian Space Position')
    xlabel('Y axis')
    ylabel('Z axis')
    
    % Save Cartesian figure as PDF
    % saveas(fig1, sprintf('%s_Cartesian_Position.pdf', figPrefix));

    % Joint Position Figure
    fig2 = figure;
    for i = 1:3
        subplot(3,1,i); grid on; hold on;
        plot(Ptime, PqAct(:,i), 'r-', 'LineWidth', 1.2);    % Phantom Actual
        plot(Stime, SqDes(:,i), 'g-', 'LineWidth', 1.2);    % Simulation Desired
        plot(Stime, SqAct(:,i), 'b-', 'LineWidth', 1.2);    % Simulation Actual

        title(sprintf('Joint %d Position', i));
        if i == 3, xlabel('Time'); end
        ylabel('Position(rad)');
    end
    legend('Phantom Actual', 'Simulation Desired', 'Simulation Actual');
    
    % Save Joint Position figure as PDF
    % saveas(fig2, sprintf('%s_Joint_Position.pdf', figPrefix));

    % Joint Velocity Figure
    fig3 = figure;
    for i = 1:3
        subplot(3,1,i); grid on; hold on;
        plot(Ptime, PqdAct(:,i), 'r-', 'LineWidth', 1.2);    % Phantom Actual
        plot(Stime, SqdDes(:,i), 'g-', 'LineWidth', 1.2);    % Simulation Desired
        plot(Stime, SqdAct(:,i), 'b-', 'LineWidth', 1.2);    % Simulation Actual

        title(sprintf('Joint %d Velocity', i));
        if i == 3, xlabel('Time'); end
        ylabel('Velocity rad/s');
    end
    legend('Phantom Actual', 'Simulation Desired', 'Simulation Actual');

    % Save Joint Velocity figure as PDF
    % saveas(fig3, sprintf('%s_Joint_Velocity.pdf', figPrefix));
end

