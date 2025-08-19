function easyPlot(dataNumber, plotType)
% EASYPLOT - Easy plotting function for force data analysis
% 
% Usage:
%   easyPlot(dataNumber, plotType)
%   easyPlot([1,3,5], plotType)  % Plot datasets 1, 3, and 5
%   easyPlot(1:7, plotType)      % Plot all datasets 1-7
%
% Inputs:
%   dataNumber - Integer from 1 to 7 OR array of integers OR range
%   plotType   - String specifying what to plot:
%                'force' - Plot force data
%                'trajectory' - Plot 3D trajectory
%                'position' - Plot position vs time
%                'all' - Plot all available plots
%
% Examples:
%   easyPlot(7, 'force')         % Plot force data from dataset 7
%   easyPlot([1,3,5], 'all')     % Plot all from datasets 1, 3, 5
%   easyPlot(1:7, 'trajectory')  % Plot trajectory from all datasets

    % Handle multiple datasets
    if length(dataNumber) > 1
        for i = 1:length(dataNumber)
            fprintf('\n--- Plotting Dataset %d ---\n', dataNumber(i));
            easyPlot(dataNumber(i), plotType);
        end
        return;
    end

    % Validate inputs
    if ~ismember(dataNumber, 1:7)
        error('Data number must be between 1 and 7');
    end
    
    if nargin < 2
        plotType = 'all';
    end
    
    % Load data
    sDataFile = sprintf('Sdata%d.mat', dataNumber);
    pDataFile = sprintf('Pdata%d.mat', dataNumber);
    
    fprintf('Loading %s and %s...\n', sDataFile, pDataFile);
    
    % Load the data
    sData = load(sDataFile);
    pData = load(pDataFile);
    
    % Get the correct field name for Pdata
    pDataField = sprintf('Pdata%d', dataNumber);
    if ~isfield(pData, pDataField)
        error('Field %s not found in %s', pDataField, pDataFile);
    end
    
    % Extract data
    if isfield(sData, 'Opt')
        Opt = sData.Opt;
    else
        Opt = 7; % Default time duration
    end
    
    if isfield(sData, 'xTarget')
        xTarget = sData.xTarget;
    else
        xTarget = [];
    end
    
    % Create time vector
    time = linspace(0, Opt(1), length(pData.(pDataField)));
    
    % Calculate forward kinematics
    [xAct, yAct, zAct] = FK(pData.(pDataField)(:,1), pData.(pDataField)(:,2), pData.(pDataField)(:,3));
    [xDes, yDes, zDes] = FK(pData.(pDataField)(:,4), pData.(pDataField)(:,5), pData.(pDataField)(:,6));
    
    % Extract force data
    Fz = pData.(pDataField)(:,9);
    
    % Plot based on type
    switch lower(plotType)
        case 'force'
            plotForce(time, Fz, dataNumber);
        case 'trajectory'
            plotTrajectory(xDes, yDes, zDes, xAct, yAct, zAct, xTarget, dataNumber);
        case 'position'
            plotPosition(time, yAct, dataNumber);
        case 'all'
            plotForce(time, Fz, dataNumber);
            plotTrajectory(xDes, yDes, zDes, xAct, yAct, zAct, xTarget, dataNumber);
            plotPosition(time, yAct, dataNumber);
        otherwise
            error('Unknown plot type. Use: force, trajectory, position, or all');
    end
end

function plotForce(time, Fz, dataNumber)
    figure('Name', sprintf('Force Data - Dataset %d', dataNumber));
    plot(time, Fz, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Force (N)');
    title(sprintf('Force vs Time - Dataset %d', dataNumber));
    grid on;
end

function plotTrajectory(xDes, yDes, zDes, xAct, yAct, zAct, xTarget, dataNumber)
    figure('Name', sprintf('Trajectory - Dataset %d', dataNumber));
    hold on;
    plot3(xDes, yDes, zDes, 'b-', 'LineWidth', 2, 'DisplayName', 'Desired');
    plot3(xAct, yAct, zAct, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual');
    
    if ~isempty(xTarget)
        plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'g*', 'MarkerSize', 10, 'DisplayName', 'Targets');
    end
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('3D Trajectory - Dataset %d', dataNumber));
    legend('show');
    grid on;
    axis equal;
end

function plotPosition(time, yAct, dataNumber)
    figure('Name', sprintf('Position - Dataset %d', dataNumber));
    plot(time, yAct - yAct(1), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title(sprintf('Position vs Time - Dataset %d', dataNumber));
    grid on;
end
