close all
% publish('plottingData.m', 'html');
for i = 1:2
    % Construct filenames
    PdataFile = sprintf('Pdata%d.mat', i);
    SdataFile = sprintf('Sdata%d.mat', i);
    GdataFile = sprintf('Gdata%d.mat', i);
    
    % Check which files exist (2 => file exists)
    hasPdata = exist(PdataFile, 'file') == 2;
    hasSdata = exist(SdataFile, 'file') == 2;
    hasGdata = exist(GdataFile, 'file') == 2;
    
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
    
    % ---------------- Phantom Actual ----------------
    if hasPdata
        PqAct  = Pdata.Pdata(:,7:9);
        PqdAct = Pdata.Pdata(:,10:12);
    else
        PqAct = []; PqdAct = [];
    end

    % ---------------- Extract times ----------------
    if hasSdata
        Stime = Sdata.tOpt;
        if hasPdata
            Ptime = linspace(0, Stime(end), length(PqAct));
        else
            Ptime = [];
        end
    else
        Stime = [];
        Ptime = [];
    end
    
    if hasGdata
        Gtime = Gdata.Hys;
    else
        Gtime = [];
    end

    % ---------------- Simulation Desired and Actual ----------------
    if hasSdata
        SqDes  = Sdata.yOpt(:,1:3);
        SqdDes = Sdata.yOpt(:,4:6);
        SqAct  = Sdata.yOpt(:,7:9);
        SqdAct = Sdata.yOpt(:,10:12);
    else
        SqDes = []; SqdDes = []; SqAct = []; SqdAct = [];
    end
    
    % ---------------- Gain Data (Actual) ----------------
    if hasGdata
        GqAct  = Gdata.yOut1(:,7:9);
        GqdAct = Gdata.yOut1(:,10:12);
    else
        GqAct = []; GqdAct = [];
    end

    % ---------------- Target points ----------------
    if hasSdata && isfield(Sdata,'xTarget')
        xTarget = Sdata.xTarget;
    else
        xTarget = [];
    end
    
    % ---------------- Forward kinematics ----------------
    if hasSdata && ~isempty(SqAct)
        [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    else
        SxAct=[];SyAct=[];SzAct=[];
    end
    
    if hasPdata && ~isempty(PqAct)
        [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));
    else
        PxAct=[];PyAct=[];PzAct=[];
    end
    
    if hasGdata && ~isempty(GqAct)
        [GxAct,GyAct,GzAct] = FK(GqAct(:,1),GqAct(:,2),GqAct(:,3));
    else
        GxAct=[];GyAct=[];GzAct=[];
    end

    % ---------------- Distances to targets (optional legend text) ----------------
    if ~isempty(xTarget)
        nTargets = size(xTarget, 1);
        TargetMinSim  = zeros(nTargets, 1);
        TargetMinPthm = zeros(nTargets, 1);
        TargetMinGain = zeros(nTargets, 1);
        
        for k = 1:nTargets
            if ~isempty(SxAct)
                diffsSim = [SxAct, SyAct, SzAct] - xTarget(k, :);
                TargetMinSim(k) = min(sqrt(sum(diffsSim.^2, 2)));
            end
            if ~isempty(PxAct)
                diffsPthm = [PxAct, PyAct, PzAct] - xTarget(k, :);
                TargetMinPthm(k) = min(sqrt(sum(diffsPthm.^2, 2)));
            end
            if ~isempty(GxAct)
                diffsGain = [GxAct, GyAct, GzAct] - xTarget(k, :);
                TargetMinGain(k) = min(sqrt(sum(diffsGain.^2, 2)));
            end
        end
    else
        nTargets = 0;
        TargetMinSim  = [];
        TargetMinPthm = [];
        TargetMinGain = [];
    end
    
    % ==================== 3D Cartesian Plot (FIGURE 1) ====================
    fig1 = figure('Name',[figPrefix ' - 3D Cartesian']); %#ok<NASGU>
    hold on; grid on; view(3);
    axis equal
    xlabel('$X$ (m)','Interpreter','latex');
    ylabel('$Y$ (m)','Interpreter','latex');
    zlabel('$Z$ (m)','Interpreter','latex');
    title('3D Cartesian Space Position');

    % Collect legend handles/labels
    plotHandles = gobjects(0);
    plotLabels  = {};

    % Start point (origin) — choose a base color
    palette = parula(10);      % get 10 colors from the parula colormap

    startColor = palette(8, :);  % use first palette color instead of black
    hStart = plot3(0, 0, 0, 'o', 'LineWidth', 1.5, 'MarkerSize', 7, ...
        'MarkerFaceColor', startColor, 'MarkerEdgeColor', startColor, ...
        'DisplayName', 'Start Point');
    plotHandles(end+1) = hStart;
    plotLabels{end+1}  = 'Start Point';

    % End/Final point (only if Sdata available)
    if hasSdata && isfield(Sdata,'xFinal') && numel(Sdata.xFinal) >= 3
        palette = parula(10);      % get 10 colors from the parula colormap

        finalColor = palette(5, :);  % use first palette color instead of black
        hEnd = plot3(Sdata.xFinal(1),Sdata.xFinal(2),Sdata.xFinal(3), ...
            'o', 'LineWidth', 1.5, 'MarkerSize', 7, ...
            'MarkerFaceColor', finalColor, 'MarkerEdgeColor', finalColor, ...
            'DisplayName', 'Final Point');
        plotHandles(end+1) = hEnd;
        plotLabels{end+1}  = 'Final Point';
    else
        hEnd = [];
        finalColor = [];
    end
    
    % Trajectories (no reference projection required)
    if ~isempty(SxAct)
        hSim = plot3(SxAct, SyAct, SzAct, 'r-', 'LineWidth', 2, ...
            'DisplayName', 'Simulation ($q_{\mathrm{sim}}$)');
        plotHandles(end+1) = hSim;
        plotLabels{end+1}  = 'Simulation ($q_{\mathrm{sim}}$)';
    else
        hSim = [];
    end
    if ~isempty(PxAct)
        hPhantom = plot3(PxAct, PyAct, PzAct, 'g-', 'LineWidth', 2, ...
            'DisplayName', 'Phantom ($q_{\mathrm{ph}}$)');
        plotHandles(end+1) = hPhantom;
        plotLabels{end+1}  = 'Phantom ($q_{\mathrm{ph}}$)';
    else
        hPhantom = [];
    end
    if ~isempty(GxAct)
        hGain = plot3(GxAct, GyAct, GzAct, 'm-', 'LineWidth', 2, ...
            'DisplayName', 'Gain');
        plotHandles(end+1) = hGain;
        plotLabels{end+1}  = 'Gain';
    else
        hGain = [];
    end
    
    % Target points (plotted as filled pentagrams)
    if ~isempty(xTarget)
        targetHandles = gobjects(nTargets,1);
        targetLabels  = cell(nTargets,1);
        for k = 1:nTargets
            targetHandles(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
                'p', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'MarkerEdgeColor','k', ...
                'DisplayName', sprintf('Target %d', k));
            targetLabels{k} = sprintf('Target %d', k);
        end
        plotHandles = [plotHandles, targetHandles(:)'];
        plotLabels  = [plotLabels,  targetLabels(:)'];
    end

    % Dummy handles for target error lines (legend only)
    errorHandles = gobjects(0);
    errorLabels  = {};
    if ~isempty(xTarget)
        for k = 1:nTargets
            if ~isempty(SxAct)
                errorHandles(end+1) = plot(nan, nan, 'w');
                errorLabels{end+1}  = sprintf('Target %d Sim Err: %.6f', k, TargetMinSim(k));
            end
            if ~isempty(PxAct)
                errorHandles(end+1) = plot(nan, nan, 'w');
                errorLabels{end+1}  = sprintf('Target %d Pthm Err: %.6f', k, TargetMinPthm(k));
            end
            if ~isempty(GxAct)
                errorHandles(end+1) = plot(nan, nan, 'w');
                errorLabels{end+1}  = sprintf('Target %d Gain Err: %.6f', k, TargetMinGain(k));
            end
        end
    end
    
    % ---------- Projections on XY / XZ / YZ planes (same axes) ----------
    ax = gca;

    % Capture limits before adding projections
    xl = xlim(ax); yl = ylim(ax); zl = zlim(ax);

    % Offset distance: 1 cm (meters)
    offset = 0.01;

    % Keep XY at z = zmin, keep XZ at y = ymin (UNCHANGED),
    % move YZ to the "other side": x = xmax + 1 cm
    planes.z = zl(1);          % XY @ zmin
    planes.y = yl(1);          % XZ @ ymin
    planes.x = xl(2) + offset; % YZ shifted

    % Expand axes to include shifted YZ plane
    xlim([xl(1), max(xl(2), planes.x)]);
    ylim(yl);
    zlim(zl);

    % Trajectory projections (reference path not projected)
    if ~isempty(SxAct)
        add3DProjections(ax, SxAct, SyAct, SzAct, planes, get(plotHandles(strcmp(plotLabels,'Simulation ($q_{\mathrm{sim}}$)')),'Color'));
    end
    if ~isempty(PxAct)
        add3DProjections(ax, PxAct, PyAct, PzAct, planes, get(plotHandles(strcmp(plotLabels,'Phantom ($q_{\mathrm{ph}}$)')),'Color'));
    end
    if ~isempty(GxAct)
        add3DProjections(ax, GxAct, GyAct, GzAct, planes, get(plotHandles(strcmp(plotLabels,'Gain')),'Color'));
    end

    % Project TARGET POINTS too (no legend entries to avoid clutter)
    if ~isempty(xTarget)
        addTargetProjections(ax, xTarget, planes);
    end

    % Project START and FINAL points; projections use the SAME color as the points
    addPointProjections(ax, [0 0 0], planes, get(hStart,'MarkerFaceColor'));
    if ~isempty(hEnd)
        addPointProjections(ax, Sdata.xFinal(1:3), planes, get(hEnd,'MarkerFaceColor'));
    end
    % --------------------------------------------------------------------

    % Legend with LaTeX interpreter (includes trajectories + targets + errors)
    allHandles = [plotHandles, errorHandles];
    allLabels  = [plotLabels,  errorLabels];
    legend(allHandles, allLabels, 'Location', 'eastoutside', 'Interpreter', 'latex');

    % ==================== Joint Position Figure (FIGURE 2) ====================
    if hasPdata || hasSdata || hasGdata
        fig2 = figure('Name',[figPrefix ' - Joint Position']); %#ok<NASGU>
        for j = 1:3
            subplot(3,1,j); grid on; hold on;
            
            subplotHandles = gobjects(0);
            subplotLabels  = {};
            
            if hasPdata && ~isempty(PqAct)
                hP = plot(Ptime, PqAct(:,j), 'r-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hP;
                subplotLabels{end+1}  = 'Phantom Actual';
            end
            
            if hasSdata && ~isempty(SqDes)
                hSd = plot(Stime, SqDes(:,j), 'k--', 'LineWidth', 1.2);
                subplotHandles(end+1) = hSd;
                subplotLabels{end+1}  = sprintf('Reference ($q_{d,%d}$)', j);
            end
            if hasSdata && ~isempty(SqAct)
                hSa = plot(Stime, SqAct(:,j), 'b-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hSa;
                subplotLabels{end+1}  = sprintf('Simulation ($q_{%d}$)', j);
            end
            
            if hasGdata && ~isempty(GqAct)
                hG = plot(Gtime, GqAct(:,j), 'm-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hG;
                subplotLabels{end+1}  = 'Gain Actual';
            end

            title(sprintf('Joint %d Position', j));
            if j == 3, xlabel('Time (s)'); end
            ylabel('Position (rad)');
            legend(subplotHandles, subplotLabels, 'Interpreter', 'latex', 'Location', 'best');
        end
    end
    
    % ==================== Joint Velocity Figure (FIGURE 3) ====================
    if hasPdata || hasSdata || hasGdata
        fig3 = figure('Name',[figPrefix ' - Joint Velocity']); %#ok<NASGU>
        for j = 1:3
            subplot(3,1,j); grid on; hold on;
            
            subplotHandles = gobjects(0);
            subplotLabels  = {};
            
            if hasPdata && ~isempty(PqdAct)
                hP = plot(Ptime, PqdAct(:,j), 'r-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hP;
                subplotLabels{end+1}  = 'Phantom Actual';
            end
            
            if hasSdata && ~isempty(SqdDes)
                hSd = plot(Stime, SqdDes(:,j), 'k--', 'LineWidth', 1.2);
                subplotHandles(end+1) = hSd;
                subplotLabels{end+1}  = sprintf('Reference ($\\dot{q}_{d,%d}$)', j);
            end
            if hasSdata && ~isempty(SqdAct)
                hSa = plot(Stime, SqdAct(:,j), 'b-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hSa;
                subplotLabels{end+1}  = sprintf('Simulation ($\\dot{q}_{%d}$)', j);
            end
            
            if hasGdata && ~isempty(GqdAct)
                hG = plot(Gtime, GqdAct(:,j), 'm-', 'LineWidth', 1.2);
                subplotHandles(end+1) = hG;
                subplotLabels{end+1}  = 'Gain Actual';
            end

            title(sprintf('Joint %d Velocity', j));
            if j == 3, xlabel('Time (s)'); end
            ylabel('Velocity (rad/s)');
            legend(subplotHandles, subplotLabels, 'Interpreter', 'latex', 'Location', 'best');
        end
    end

    % Optionally save figures:
    % saveas(fig1, sprintf('%s_3D.pdf', figPrefix));
    % saveas(fig2, sprintf('%s_Joint_Position.pdf', figPrefix));
    % saveas(fig3, sprintf('%s_Joint_Velocity.pdf', figPrefix));
end


% ---------- Helper: overlay trajectory projections (same axes) ----------
function add3DProjections(ax, X, Y, Z, planes, color)
    % XY projection
    plot3(ax, X, Y, planes.z*ones(size(Z)), ':', 'Color', color, 'LineWidth', 1.2, ...
        'HandleVisibility','off');
    % XZ projection (unchanged plane)
    plot3(ax, X, planes.y*ones(size(Y)), Z, ':', 'Color', color, 'LineWidth', 1.2, ...
        'HandleVisibility','off');
    % YZ projection (shifted plane)
    plot3(ax, planes.x*ones(size(X)), Y, Z, ':', 'Color', color, 'LineWidth', 1.2, ...
        'HandleVisibility','off');
end

% ---------- Helper: project target points onto planes (no legend clutter) ----------
function addTargetProjections(ax, xTarget, planes)
    for k = 1:size(xTarget,1)
        xt = xTarget(k,1); yt = xTarget(k,2); zt = xTarget(k,3);
        % XY
        plot3(ax, xt, yt, planes.z, 'p', 'MarkerSize', 9, ...
            'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor','none','HandleVisibility','off');
        % XZ
        plot3(ax, xt, planes.y, zt, 'p', 'MarkerSize', 9, ...
            'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor','none','HandleVisibility','off');
        % YZ
        plot3(ax, planes.x, yt, zt, 'p', 'MarkerSize', 9, ...
            'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor','none','HandleVisibility','off');
    end
end

% ---------- Helper: project single points (Start/Final) ----------
function addPointProjections(ax, pt, planes, color)
    % Projects a single 3D point onto XY (z=planes.z), XZ (y=planes.y),
    % and YZ (x=planes.x) using the SAME color as the original point.
    x = pt(1); y = pt(2); z = pt(3);
    % XY projection
    plot3(ax, x, y, planes.z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'HandleVisibility','off');
    % XZ projection (unchanged)
    plot3(ax, x, planes.y, z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'HandleVisibility','off');
    % YZ projection (shifted +1 cm)
    plot3(ax, planes.x, y, z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'HandleVisibility','off');
end