close all
% publish('plottingData.m', 'html');
for i = 1:3
    % Construct filenames
    PdataFile = sprintf('Pdata%d.mat', i);
    SdataFile = sprintf('Sdata%d.mat', i);
    GdataFile = sprintf('Gdata%d.mat', i);
    
    % Check which files exist
    hasPdata = exist(PdataFile, 'file');
    hasSdata = exist(SdataFile, 'file');
    hasGdata = exist(GdataFile, 'file');
    
    % Only proceed if at least one file exists
    if hasPdata || hasSdata || hasGdata
        % Load available data
        if hasPdata
            Pdata = load(PdataFile);
            fprintf('Loaded: %s\n', PdataFile);
        else
            Pdata = [];
            fprintf('Missing: %s\n', PdataFile);
        end
        
        if hasSdata
            Sdata = load(SdataFile);
            fprintf('Loaded: %s\n', SdataFile);
        else
            Sdata = [];
            fprintf('Missing: %s\n', SdataFile);
        end
        
        if hasGdata
            Gdata = load(GdataFile);
            fprintf('Loaded: %s\n', GdataFile);
        else
            Gdata = [];
            fprintf('Missing: %s\n', GdataFile);
        end
        
        % Call plotting function with available data
        plotPhantomSimulation(Pdata, Sdata, Gdata, sprintf('DataSet%d', i));
    else
        fprintf('Skipping dataset %d - no data files exist\n', i);
    end
end

function plotPhantomSimulation(Pdata, Sdata, Gdata, figPrefix)
    
    % Initialize flags for available data
    hasPdata = ~isempty(Pdata);
    hasSdata = ~isempty(Sdata);
    hasGdata = ~isempty(Gdata);
    
    % Phantom Desired and Actual 
    if hasPdata
        PqDes = Pdata.Pdata(:,1:3);
        PqdDes = Pdata.Pdata(:,4:6);
        PqAct = Pdata.Pdata(:,7:9);
        PqdAct = Pdata.Pdata(:,10:12);
    end

    % Extract data
    if hasSdata
        Stime = Sdata.tOpt;
        if hasPdata
            Ptime = linspace(0, Stime(end), length(PqAct));
        end
    end
    
    if hasGdata
        Gtime = Gdata.Hys;
    end

    % Simulation Desired and Actual 
    if hasSdata
        SqDes = Sdata.yOpt(:,1:3);
        SqdDes = Sdata.yOpt(:,4:6);
        SqAct = Sdata.yOpt(:,7:9);
        SqdAct = Sdata.yOpt(:,10:12);
    end
    
    % Gain Data
    if hasGdata
        GqDes = Gdata.yOut1(:,1:3);
        GqdDes = Gdata.yOut1(:,4:6);
        GqAct = Gdata.yOut1(:,7:9);
        GqdAct = Gdata.yOut1(:,10:12);
    end



    % Get target points if available
    if hasSdata
        xTarget = Sdata.xTarget;
    else
        xTarget = [];
    end
    
    % Calculate FK for available data
    if hasSdata
        [SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
        [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    end
    
    if hasPdata
        [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));
    end
    
    if hasGdata
        [GxDes,GyDes,GzDes] = FK(GqDes(:,1),GqDes(:,2),GqDes(:,3));
        [GxAct,GyAct,GzAct] = FK(GqAct(:,1),GqAct(:,2),GqAct(:,3));
    end



    % --- Calculate number of target points ---
    if ~isempty(xTarget)
        nTargets = size(xTarget, 1);  % Number of target points
        
        % --- Initialize distance arrays ---
        TargetMinSim = zeros(nTargets, 1);
        TargetMinPthm = zeros(nTargets, 1);
        TargetMinGain = zeros(nTargets, 1);
        
        % --- Compute minimum distances to each target point ---
        for i = 1:nTargets
            % Simulation trajectory vs target
            if hasSdata
                diffsSim = [SxAct, SyAct, SzAct] - xTarget(i, :);
                TargetMinSim(i) = min(sqrt(sum(diffsSim.^2, 2)));
            end
        
            % Phantom trajectory vs target
            if hasPdata
                diffsPthm = [PxAct, PyAct, PzAct] - xTarget(i, :);
                TargetMinPthm(i) = min(sqrt(sum(diffsPthm.^2, 2)));
            end

            % Gain trajectory vs target
            if hasGdata
                diffsGain = [GxAct, GyAct, GzAct] - xTarget(i, :);
                TargetMinGain(i) = min(sqrt(sum(diffsGain.^2, 2)));
            end
        end
    else
        nTargets = 0;
        TargetMinSim = [];
        TargetMinPthm = [];
        TargetMinGain = [];
    end
    
    % --- Begin Plotting ---
    fig1 = figure; hold on; grid on;
    
    % Initialize plot handles
    plotHandles = [];
    plotLabels = {};
    colors = lines(7);  % 7 default colors

    % Start and End Points
      % Start and End Points
    hStart = plot(0, 0, 's', 'LineWidth', 2, 'MarkerSize', 6, ...
                  'MarkerEdgeColor', colors(2,:), 'MarkerFaceColor', colors(2,:), ...
                  'DisplayName', 'Start Point');
    text(0 + 0.002, 0 + 0.006, 'Start', 'FontSize', 9, 'FontWeight', 'bold', 'Color', colors(2,:));
    
    hEnd = plot(Sdata.xFinal(1), Sdata.xFinal(3), 's', 'LineWidth', 2, 'MarkerSize', 6, ...
                'MarkerEdgeColor', colors(2,:), 'MarkerFaceColor', colors(2,:), ...
                'DisplayName', 'End Point');
    text(Sdata.xFinal(1) + 0.002, Sdata.xFinal(3) + 0.002, 'End', 'FontSize', 9, 'FontWeight', 'bold', 'Color', colors(2,:));
    plotHandles = [plotHandles, hStart, hEnd];
    plotLabels = [plotLabels, {'Start Point', 'End Point'}];
    
    % Desired Position and Trajectories
    if hasSdata
        % hDesired = plot(SxDes, SzDes, 'b', 'LineWidth', 1.5, 'DisplayName', 'Desired Position');
        hSim     = plot(SxAct, SzAct, 'Color', colors(5,:),'LineWidth', 1.5, 'DisplayName', 'Simulation Trajectory');
        plotHandles = [plotHandles,  hSim];
        plotLabels = [plotLabels, {'Simulation Trajectory'}];
    end
    
    if hasPdata
        hPhantom = plot(PxAct, PzAct, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Phantom Trajectory');
        plotHandles = [plotHandles, hPhantom];
        plotLabels = [plotLabels, {'Phantom Trajectory'}];
    end
    
    if hasGdata
        hGain = plot(GxAct, GzAct, 'm', 'LineWidth', 2, 'DisplayName', 'Gain Trajectory');
        plotHandles = [plotHandles, hGain];
        plotLabels = [plotLabels, {'Gain Trajectory'}];
    end
    
    % Target Points
    if ~isempty(xTarget)
        targetHandles = gobjects(nTargets,1);
        targetLabels = cell(nTargets,1);
        for k = 1:nTargets
            % Plot target point
            targetHandles(k) = plot(xTarget(k,1), xTarget(k,3), 'o', ...
                'MarkerSize', 6, 'MarkerEdgeColor', colors(4,:), 'MarkerFaceColor', colors(4,:), ...
                'DisplayName', sprintf('Target Point %d', k));
            
            % Add text label next to the target
            text(xTarget(k,1)+0.002, xTarget(k,3)+0.002, ...   % slight offset
                 sprintf('Target %d', k), ...                        % label text
                 'FontSize', 9, ...
                 'FontWeight', 'bold', ...
                 'Color', colors(4,:));
    
            targetLabels{k} = sprintf('Target Point %d', k);
        end
        plotHandles = [plotHandles, targetHandles(:)'];
        plotLabels = [plotLabels, targetLabels(:)'];
    end
    
    % Optional: Plot Control Points if they exist
    if hasSdata && isfield(Sdata, 'xCtrl')
        xCtrl = Sdata.xCtrl;
        hCtrl = plot(xCtrl(:,1), xCtrl(:,3), 's', 'LineWidth', 1.5, ...
            'MarkerSize', 6, 'DisplayName', 'Control Points');
        ctrlLabel = {'Control Points'};
        ctrlHandles = hCtrl;
        plotHandles = [plotHandles, ctrlHandles];
        plotLabels = [plotLabels, ctrlLabel];
    else
        ctrlLabel = {};
        ctrlHandles = [];
    end
    
    % --- Create Dummy Handles for Error Info (for legend) ---
    errorHandles = [];
    errorLabels = {};
    
    if ~isempty(xTarget)
        for k = 1:nTargets
            if hasSdata
                errorHandles(end+1) = plot(nan, nan, 'w');  % Invisible handle for Sim error
                errorLabels{end+1} = sprintf('Target %d Sim Err: %.6f', k, TargetMinSim(k));
            end
        
            if hasPdata
                errorHandles(end+1) = plot(nan, nan, 'w');    % Invisible handle for Pthm error
                errorLabels{end+1} = sprintf('Target %d Pthm Err: %.6f', k, TargetMinPthm(k));
            end

            if hasGdata
                errorHandles(end+1) = plot(nan, nan, 'w');    % Invisible handle for Gain error
                errorLabels{end+1} = sprintf('Target %d Gain Err: %.6f', k, TargetMinGain(k));
            end
        end
    end
    
    % --- Build Full Legend ---
    allHandles = [plotHandles, errorHandles];
    allLabels = [plotLabels, errorLabels];
    
    legend(allHandles, allLabels, 'Location', 'eastoutside');
    
    % --- Axis Labels and Title ---
    xlabel('X axis (m)');
    ylabel('Y axis (m)');
    title('3D Cartesian Space Position');


    % Joint Position Figure
    if hasPdata || hasSdata || hasGdata
        fig2 = figure;
        for i = 1:3
            subplot(3,1,i); grid on; hold on;
            
            % Initialize legend entries for this subplot
            subplotHandles = [];
            subplotLabels = {};
            
            if hasPdata
                plot(Ptime, PqAct(:,i), 'r-', 'LineWidth', 1.2);    % Phantom Actual
                subplotHandles(end+1) = plot(Ptime, PqAct(:,i), 'r-', 'LineWidth', 1.2);
                subplotLabels{end+1} = 'Phantom Actual';
            end
            
            if hasSdata
                plot(Stime, SqDes(:,i), 'g-', 'LineWidth', 1.2);    % Simulation Desired
                plot(Stime, SqAct(:,i), 'b-', 'LineWidth', 1.2);    % Simulation Actual
                subplotHandles(end+1) = plot(Stime, SqDes(:,i), 'g-', 'LineWidth', 1.2);
                subplotHandles(end+1) = plot(Stime, SqAct(:,i), 'b-', 'LineWidth', 1.2);
                % subplotLabels{end+1} = 'Simulation Desired';
                subplotLabels{end+1} = 'Simulation Actual';
            end
            
            if hasGdata
                plot(Gtime, GqAct(:,i), 'm-', 'LineWidth', 1.2);    % Gain Actual
                subplotHandles(end+1) = plot(Gtime, GqAct(:,i), 'm-', 'LineWidth', 1.2);
                subplotLabels{end+1} = 'Gain Actual';
            end

            title(sprintf('Joint %d Position', i));
            if i == 3, xlabel('Time'); end
            ylabel('Position(rad)');
            legend(subplotHandles, subplotLabels);
        end
    end
    
    % Save Joint Position figure as PDF
    % saveas(fig2, sprintf('%s_Joint_Position.pdf', figPrefix));

    % Joint Velocity Figure
    if hasPdata || hasSdata || hasGdata
        fig3 = figure;
        for i = 1:3
            subplot(3,1,i); grid on; hold on;
            
            % Initialize legend entries for this subplot
            subplotHandles = [];
            subplotLabels = {};
            
            if hasPdata
                plot(Ptime, PqdAct(:,i), 'r-', 'LineWidth', 1.2);    % Phantom Actual
                subplotHandles(end+1) = plot(Ptime, PqdAct(:,i), 'r-', 'LineWidth', 1.2);
                subplotLabels{end+1} = 'Phantom Actual';
            end
            
            if hasSdata
                plot(Stime, SqdDes(:,i), 'g-', 'LineWidth', 1.2);    % Simulation Desired
                plot(Stime, SqdAct(:,i), 'b-', 'LineWidth', 1.2);    % Simulation Actual
                subplotHandles(end+1) = plot(Stime, SqdDes(:,i), 'g-', 'LineWidth', 1.2);
                subplotHandles(end+1) = plot(Stime, SqdAct(:,i), 'b-', 'LineWidth', 1.2);
                % subplotLabels{end+1} = 'Simulation Desired';
                subplotLabels{end+1} = 'Simulation Actual';
            end
            
            if hasGdata
                plot(Gtime, GqdAct(:,i), 'm-', 'LineWidth', 1.2);    % Gain Actual
                subplotHandles(end+1) = plot(Gtime, GqdAct(:,i), 'm-', 'LineWidth', 1.2);
                subplotLabels{end+1} = 'Gain Actual';
            end

            title(sprintf('Joint %d Velocity', i));
            if i == 3, xlabel('Time'); end
            ylabel('Velocity rad/s');
            legend(subplotHandles, subplotLabels);
        end
    end

    % Save Joint Velocity figure as PDF
    % saveas(fig3, sprintf('%s_Joint_Velocity.pdf', figPrefix));
end

