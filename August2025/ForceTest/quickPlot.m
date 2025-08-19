% QUICKPLOT - Quick plotting script for force data analysis
% 
% This script makes it easy to plot your data without typing long commands.
% Just run this script and follow the prompts.

fprintf('=== Quick Plot Tool ===\n');
fprintf('Available datasets: 1-7\n');
fprintf('Available plot types: force, trajectory, position, all\n');
fprintf('You can specify:\n');
fprintf('  - Single dataset: 3\n');
fprintf('  - Multiple datasets: [1,3,5]\n');
fprintf('  - Range: 1:7 (all datasets)\n\n');

% Get user input
dataInput = input('Enter dataset(s) (e.g., 3, [1,3,5], or 1:7): ');
plotType = input('Enter plot type (force/trajectory/position/all): ', 's');

% Validate input
if isempty(dataInput)
    fprintf('No dataset specified. Using dataset 7.\n');
    dataInput = 7;
end

if isempty(plotType)
    fprintf('No plot type specified. Plotting all.\n');
    plotType = 'all';
end

% Convert to array if single number
if length(dataInput) == 1
    dataNum = dataInput;
else
    dataNum = dataInput;
end

% Validate dataset numbers
if any(dataNum < 1) || any(dataNum > 7)
    fprintf('Invalid dataset number(s). Using dataset 7.\n');
    dataNum = 7;
end

% Call the easy plotting function
try
    easyPlot(dataNum, plotType);
    fprintf('Plotting completed successfully!\n');
catch ME
    fprintf('Error: %s\n', ME.message);
end
