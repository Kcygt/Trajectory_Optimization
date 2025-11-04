close all
% publish('plottingData.m', 'html');
for i = 3:5
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

    % Simulation Desired and Actual 
    if hasSdata
        SqDes = Sdata.yOpt(:,1:3);
        SqdDes = Sdata.yOpt(:,4:6);
        SqAct = Sdata.yOpt(:,7:9);
        SqdAct = Sdata.yOpt(:,10:12);
    end

    % --- Calculate RMSE for Joint Position and Velocity ---
    rmsePos = zeros(3,1);  % RMSE for joint position
    rmseVel = zeros(3,1);  % RMSE for joint velocity

    % For each joint (1 to 3)
    for i = 1:3
        % Position RMSE calculation
        if hasSdata
            % Joint Position RMSE (Simulation)
            rmsePos(i) = sqrt(mean((SqAct(:,i) - SqDes(:,i)).^2));  % Simulation Actual vs Desired
        end
        if hasPdata
            % Joint Position RMSE (Phantom)
            rmsePos(i) = sqrt(mean((PqAct(:,i) - PqDes(:,i)).^2));  % Phantom Actual vs Desired
        end
        
        % Velocity RMSE calculation
        if hasSdata
            % Joint Velocity RMSE (Simulation)
            rmseVel(i) = sqrt(mean((SqdAct(:,i) - SqdDes(:,i)).^2));  % Simulation Actual vs Desired
        end
        if hasPdata
            % Joint Velocity RMSE (Phantom)
            rmseVel(i) = sqrt(mean((PqdAct(:,i) - PqdDes(:,i)).^2));  % Phantom Actual vs Desired
        end
    end

    % Display RMSE for joint position and velocity
    disp('RMSE for Joint Position (in rad):');
    for i = 1:3
        fprintf('Joint %d Position RMSE: %.4f\n', i, rmsePos(i));
    end

    disp('RMSE for Joint Velocity (in rad/s):');
    for i = 1:3
        fprintf('Joint %d Velocity RMSE: %.4f\n', i, rmseVel(i));
    end
end
