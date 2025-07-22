close all
% publish('plottingData.m', 'html');
for i = 14:14
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
    


    % --- Calculate number of target points ---
    nTargets = size(xTarget, 1);  % Number of target points
    
    % --- Initialize distance arrays ---
    TargetMinSim = zeros(nTargets, 1);
    TargetMinPthm = zeros(nTargets, 1);
    
    % --- Compute minimum distances to each target point ---
    for i = 1:nTargets
        % Simulation trajectory vs target
        diffsSim = [SxAct, SyAct, SzAct] - xTarget(i, :);
        TargetMinSim(i) = min(sqrt(sum(diffsSim.^2, 2)));
    
        % Phantom trajectory vs target
        diffsPthm = [PxAct, PyAct, PzAct] - xTarget(i, :);
        TargetMinPthm(i) = min(sqrt(sum(diffsPthm.^2, 2)));
    end
    
    % --- Begin Plotting ---
    fig1 = figure; hold on; grid on;
    
    % Start and End Points
    hStart = plot(0, 0, 'o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Start Point');
    hEnd   = plot(0.05, 0.05, 'o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'End Point');
    
    % Desired Position and Trajectories
    hDesired = plot(SyDes, SzDes, 'b', 'LineWidth', 2, 'DisplayName', 'Desired Position');
    hSim     = plot(SyAct, SzAct, 'r', 'LineWidth', 2, 'DisplayName', 'Simulation Trajectory');
    hPhantom = plot(PyAct, PzAct, 'g', 'LineWidth', 2, 'DisplayName', 'Phantom Trajectory');
    
    % Target Points
    targetHandles = gobjects(nTargets,1);
    targetLabels = cell(nTargets,1);
    for k = 1:nTargets
        % Plot target point
        targetHandles(k) = plot(xTarget(k,2), xTarget(k,3), 'p', ...
            'MarkerSize', 12, 'MarkerFaceColor', 'k', ...
            'DisplayName', sprintf('Target Point %d', k));
        
        % Create target labels
        targetLabels{k} = sprintf('Target Point %d', k);
    end
    
    % Optional: Plot Control Points if they exist
    if isfield(Sdata, 'xCtrl')
        xCtrl = Sdata.xCtrl;
        hCtrl = plot(xCtrl(:,1), xCtrl(:,2), 'd', 'LineWidth', 1.5, ...
            'MarkerSize', 6, 'DisplayName', 'Control Points');
        ctrlLabel = {'Control Points'};
        ctrlHandles = hCtrl;
    else
        ctrlLabel = {};
        ctrlHandles = [];
    end
    
    % --- Create Dummy Handles for Error Info (for legend) ---
    errorHandles = gobjects(nTargets*2, 1); % Two per target
    errorLabels = cell(nTargets*2, 1);
    
    for k = 1:nTargets
        errorHandles(2*k-1) = plot(nan, nan, 'w');  % Invisible handle for Sim error
        errorLabels{2*k-1} = sprintf('Target %d Sim Err: %.6f', k, TargetMinSim(k));
    
        errorHandles(2*k) = plot(nan, nan, 'w');    % Invisible handle for Pthm error
        errorLabels{2*k} = sprintf('Target %d Pthm Err: %.6f', k, TargetMinPthm(k));
    end
    
    % --- Build Full Legend ---
    legendHandles = [hStart, hEnd, hDesired, hSim, hPhantom, ...
                     targetHandles(:)', ctrlHandles(:)', errorHandles(:)'];
    
    legendLabels = [{'Start Point', 'End Point', 'Desired Position', ...
                     'Simulation Trajectory', 'Phantom Trajectory'}, ...
                     targetLabels(:)', ctrlLabel(:)', errorLabels(:)'];
    
    legend(legendHandles, legendLabels, 'Location', 'eastoutside');
    
    % --- Axis Labels and Title ---
    xlabel('Y axis');
    ylabel('Z axis');
    title('Cartesian Space Position');


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

