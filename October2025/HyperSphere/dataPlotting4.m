% process_and_plot.m
% Loads datasets 4:5 and plots according to your preferred palette & markers.
clear; clc; close all;

for i = 4:4
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

    % ------------------- Color scheme (GLOBAL for this function) -------------------
    gem = [
        0.9961    0.5469         0;  % Emerald
        0.00 0.35 0.70;              % Sapphire
        0.55 0.00 0.55;              % Amethyst
        0.80 0.00 0.20;              % Ruby
        0.20 0.60 0.90;              % Blue topaz
        0.90 0.75 0.10;              % Citrine
        0.8594    0.0781    0.2344;  % Turquoise
        0.85 0.40 0.40;              % Garnet
        0    0.8047    0.8164;       % Tanzanite
        0.5430         0    0.5430;  % Rose quartz
        ];

    palette      = gem;
    colorSim     = [0 1 0];   % Simulation -> turquoise
    colorPhantom = [1.0 0.4 0.0] ; % Phantom    -> tanzanite
    colorRef     = [0 0 0];         % Reference  -> black
    colorGain    = [1 0 1];         % Gain       -> magenta
    startColor   = [0 0 1];    % Start point -> emerald
    finalColor   = [1 0 0];   % Final point -> rose quartz

    % Use 'color' variable for target plotting exactly as you requested:
    color = [0 0 0];  % bright green for target markers (kept as 'color')

    % ---------------- Phantom Actual ----------------
    if hasPdata && isfield(Pdata,'Pdata')
        PqAct  = Pdata.Pdata(:,7:9);
        PqdAct = Pdata.Pdata(:,10:12);
    else
        PqAct = []; PqdAct = [];
    end

    % ---------------- Extract times ----------------
    if hasSdata && isfield(Sdata,'tOpt')
        Stime = Sdata.tOpt;
        if ~isempty(PqAct)
            Ptime = linspace(0, Stime(end), size(PqAct,1));
        else
            Ptime = [];
        end
    else
        Stime = [];
        Ptime = [];
    end
    
    if hasGdata && isfield(Gdata,'Hys')
        Gtime = Gdata.Hys;
    else
        Gtime = [];
    end

    % ---------------- Section split times ----------------
    hasSections = hasSdata && isfield(Sdata,'optimalTimes') && numel(Sdata.optimalTimes) >= 3;
    if hasSections
        t1 = Sdata.optimalTimes(1);
        t2 = Sdata.optimalTimes(2);
        t3 = Sdata.optimalTimes(3);
        % Sections: 1) [0, t1], 2) [t1, t2], 3) [t2, t3]
        sectionEdges  = [0, t1; t1, t2; t2, t3];
        sectionLabels = {'Section 1','Section 2','Section 3'};
    else
        sectionEdges  = [];
        sectionLabels = {};
    end

    % ---------------- Simulation Desired and Actual ----------------
    if hasSdata && isfield(Sdata,'yOpt')
        SqDes  = Sdata.yOpt(:,1:3);
        SqdDes = Sdata.yOpt(:,4:6);
        SqAct  = Sdata.yOpt(:,7:9);
        SqdAct = Sdata.yOpt(:,10:12);
    else
        SqDes = []; SqdDes = []; SqAct = []; SqdAct = [];
    end
    
    % ---------------- Gain Data (Actual) ----------------
    if hasGdata && isfield(Gdata,'yOut1')
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
    % NOTE: Requires FK(q1,q2,q3) on path.
    if ~isempty(SqAct)
        [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
    else
        SxAct=[];SyAct=[];SzAct=[];
    end
    
    if ~isempty(PqAct)
        [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));
    else
        PxAct=[];PyAct=[];PzAct=[];
    end
    
    if ~isempty(GqAct)
        [GxAct,GyAct,GzAct] = FK(GqAct(:,1),GqAct(:,2),GqAct(:,3));
    else
        GxAct=[];GyAct=[];GzAct=[];
    end

    % --------- Forward kinematics for Reference (SqDes) ----------
    if ~isempty(SqDes)
        [SxDes, SyDes, SzDes] = FK(SqDes(:,1), SqDes(:,2), SqDes(:,3));
    else
        SxDes = []; SyDes = []; SzDes = [];
    end

    % ---------------- Per-target minimum distance errors ----------------
    if ~isempty(xTarget)
        nTargets = size(xTarget, 1);

        TargetMinRef   = nan(nTargets,1);
        TargetMinSim   = nan(nTargets,1);
        TargetMinPthm  = nan(nTargets,1);
        TargetMinGain  = nan(nTargets,1);

        for k = 1:nTargets
            xk = xTarget(k,:); % 1x3

            if ~isempty(SxDes)
                diffs = [SxDes, SyDes, SzDes] - xk;
                TargetMinRef(k) = min(sqrt(sum(diffs.^2,2)));
            end
            if ~isempty(SxAct)
                diffs = [SxAct, SyAct, SzAct] - xk;
                TargetMinSim(k) = min(sqrt(sum(diffs.^2,2)));
            end
            if ~isempty(PxAct)
                diffs = [PxAct, PyAct, PzAct] - xk;
                TargetMinPthm(k) = min(sqrt(sum(diffs.^2,2)));
            end
            if ~isempty(GxAct)
                diffs = [GxAct, GyAct, GzAct] - xk;
                TargetMinGain(k) = min(sqrt(sum(diffs.^2,2)));
            end
        end

        % -------- Print tidy table (one row per target) --------
        fprintf('\n===== Minimum Distance to Each Target =====\n');
        fprintf('Units: meters (and millimeters in parentheses)\n\n');

        hdr = '%-10s | %-22s | %-22s | %-22s | %-22s\n';
        sep = [repmat('-',1,108) '\n'];
        fprintf(hdr, 'Target', 'Reference', 'Simulation', 'Phantom', 'Gain');
        fprintf(sep);

        for k = 1:nTargets
            if isnan(TargetMinRef(k)),  rStr = 'N/A'; else, rStr = sprintf('%.6f m (%.1f mm)', TargetMinRef(k), 1e3*TargetMinRef(k)); end
            if isnan(TargetMinSim(k)),  sStr = 'N/A'; else, sStr = sprintf('%.6f m (%.1f mm)', TargetMinSim(k), 1e3*TargetMinSim(k)); end
            if isnan(TargetMinPthm(k)), pStr = 'N/A'; else, pStr = sprintf('%.6f m (%.1f mm)', TargetMinPthm(k), 1e3*TargetMinPthm(k)); end
            if isnan(TargetMinGain(k)), gStr = 'N/A'; else, gStr = sprintf('%.6f m (%.1f mm)', TargetMinGain(k), 1e3*TargetMinGain(k)); end

            fprintf('%-10s | %-22s | %-22s | %-22s | %-22s\n', ...
                sprintf('Target %d', k), rStr, sStr, pStr, gStr);
        end

        fprintf('%s\n', repmat('=',1,108));
    else
        fprintf('\n(No target points found in Sdata.xTarget — skipping per-target errors.)\n\n');
    end

    % ==================== RMSE (Position & Velocity) ====================
    fprintf('===== RMSE vs Reference =====\n');
    fprintf('(Units: position in rad, velocity in rad/s)\n\n');

    series(1).name = 'Simulation'; series(1).t = Stime; series(1).q = SqAct; series(1).qd = SqdAct;
    series(2).name = 'Phantom';    series(2).t = Ptime; series(2).q = PqAct; series(2).qd = PqdAct;
    series(3).name = 'Gain';       series(3).t = Gtime; series(3).q = GqAct; series(3).qd = GqdAct;

    haveRefPos = ~isempty(SqDes);
    haveRefVel = ~isempty(SqdDes);

    fprintf('%-10s | %-24s | %-24s\n', 'Series', 'RMSE Position [q1 q2 q3]', 'RMSE Velocity [q1 q2 q3]');
    fprintf('%s\n', repmat('-',1,66));

    for s = 1:numel(series)
        qRMSE  = [NaN NaN NaN];
        qdRMSE = [NaN NaN NaN];

        if ~isempty(series(s).t) && ~isempty(series(s).q) && haveRefPos
            for j = 1:3
                qRMSE(j) = rmse_vs_ref(series(s).t, series(s).q(:,j), Stime, SqDes(:,j));
            end
        end
        if ~isempty(series(s).t) && ~isempty(series(s).qd) && haveRefVel
            for j = 1:3
                qdRMSE(j) = rmse_vs_ref(series(s).t, series(s).qd(:,j), Stime, SqdDes(:,j));
            end
        end

        posStr = sprintf('[%.4g %.4g %.4g]', qRMSE(1), qRMSE(2), qRMSE(3));
        velStr = sprintf('[%.4g %.4g %.4g]', qdRMSE(1), qdRMSE(2), qdRMSE(3));
        fprintf('%-10s | %-24s | %-24s\n', series(s).name, posStr, velStr);
    end
    fprintf('%s\n\n', repmat('=',1,66));

    % ==================== 3D Cartesian Plot (FIGURE 1) ====================
    fig1 = figure('Name',[figPrefix ' - 3D Cartesian']); %#ok<NASGU>
    hold on; grid on; view(3);
    axis equal
    xlabel('$X$ (m)','Interpreter','latex');
    ylabel('$Y$ (m)','Interpreter','latex');
    zlabel('$Z$ (m)','Interpreter','latex');
    title('Cartesian Space Position');

    % Collect legend handles/labels
    plotHandles = gobjects(0);
    plotLabels  = {};

    % Start point (origin) - face color startColor, black edge
    hStart = plot3(0, 0, 0, 'o', 'LineWidth', 1.5, 'MarkerSize', 7, ...
        'MarkerFaceColor', startColor, 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Start Point');
    plotHandles(end+1) = hStart;
    plotLabels{end+1}  = 'Start Point';

    % End/Final point (only if Sdata available) - face finalColor, black edge
    if hasSdata && isfield(Sdata,'xFinal') && numel(Sdata.xFinal) >= 3
        hEnd = plot3(Sdata.xFinal(1),Sdata.xFinal(2),Sdata.xFinal(3), ...
            'o', 'LineWidth', 1.5, 'MarkerSize', 7, ...
            'MarkerFaceColor', finalColor, 'MarkerEdgeColor', 'k', ...
            'DisplayName', 'Final Point');
        plotHandles(end+1) = hEnd;
        plotLabels{end+1}  = 'Final Point';
    else
        hEnd = [];
    end
    
    % Trajectories with standardized colors
    if ~isempty(SxAct)
        hSim = plot3(SxAct, SyAct, SzAct, '-', 'LineWidth', 2, ...
            'Color', colorSim, 'DisplayName', 'Simulation ($X_{\mathrm{sim}}$)');
        plotHandles(end+1) = hSim;
        plotLabels{end+1}  = 'Simulation ($X_{\mathrm{sim}}$)';
    else
        hSim = [];
    end
    if ~isempty(PxAct)
        hPhantom = plot3(PxAct, PyAct, PzAct, '-', 'LineWidth', 2, ...
            'Color', colorPhantom, 'DisplayName', 'Phantom ($X_{\mathrm{ph}}$)');
        plotHandles(end+1) = hPhantom;
        plotLabels{end+1}  = 'Phantom ($X_{\mathrm{ph}}$)';
    else
        hPhantom = [];
    end
    if ~isempty(GxAct)
        hGain = plot3(GxAct, GyAct, GzAct, '-', 'LineWidth', 2, ...
            'Color', colorGain, 'DisplayName', 'Gain');
        plotHandles(end+1) = hGain;
        plotLabels{end+1}  = 'Gain';
    else
        hGain = [];
    end

    % Reference (desired) trajectory
    if ~isempty(SxDes)
        hRef = plot3(SxDes, SyDes, SzDes, '--', 'LineWidth', 1.8, ...
            'Color', colorRef, 'DisplayName', 'Reference ($X_{ref}$)');
        plotHandles(end+1) = hRef;
        plotLabels{end+1}  = 'Reference ($q_{ref}$)';
    else
        hRef = [];
    end
    
    % Target points (filled pentagrams) - use your requested 'color' variable
    if ~isempty(xTarget)
        nTargets = size(xTarget, 1);
        targetHandles = gobjects(nTargets,1);
        targetLabels  = cell(nTargets,1);
        for k = 1:nTargets
            targetHandles(k) = plot3(xTarget(k,1), xTarget(k,2), xTarget(k,3), ...
                'p', 'MarkerSize', 12, 'MarkerFaceColor', color, 'MarkerEdgeColor', color, ...
                'DisplayName', sprintf('Target %d', k));
            targetLabels{k} = sprintf('Target %d', k);
        end
        plotHandles = [plotHandles, targetHandles(:)'];
        plotLabels  = [plotLabels,  targetLabels(:)'];
    end

    % Dummy handles for target error lines (legend only) - optional
    errorHandles = gobjects(0);
    errorLabels  = {};

    % ---------- Projections on XY / XZ / YZ planes (same axes) ----------
    ax = gca;

    % Capture limits before adding projections
    xl = xlim(ax); yl = ylim(ax); zl = zlim(ax);

    % Offset distance: 1 cm (meters)
    offset = 0.01;

    % Keep XY at z = zmin, keep XZ at y = ymin, move YZ to x = xmax + 1 cm
    planes.z = zl(1);          % XY @ zmin
    planes.y = yl (1);          % XZ @ ymin
    planes.x = xl(2) + offset; % YZ shifted

    % Expand axes to include shifted YZ plane
    xlim([xl(1), max(xl(2), planes.x)]);
    ylim(yl);
    zlim(zl);

    % Trajectory projections (use the same standardized colors)
    if ~isempty(SxAct)
        add3DProjections(ax, SxAct, SyAct, SzAct, planes, colorSim);
    end
    if ~isempty(PxAct)
        add3DProjections(ax, PxAct, PyAct, PzAct, planes, colorPhantom);
    end
    if ~isempty(GxAct)
        add3DProjections(ax, GxAct, GyAct, GzAct, planes, colorGain);
    end
    % (Optional) also project the reference path:
    % if ~isempty(SxDes)
    %     add3DProjections(ax, SxDes, SyDes, SzDes, planes, colorRef);
    % end

    % Project TARGET POINTS too (no legend entries to avoid clutter)
    if ~isempty(xTarget)
        addTargetProjections(ax, xTarget, planes);
    end

    % Project START and FINAL points; projections use the SAME face color but black edges
    addPointProjections(ax, [0 0 0], planes, startColor);
    if ~isempty(hEnd)
        addPointProjections(ax, Sdata.xFinal(1:3), planes, finalColor);
    end
    % --------------------------------------------------------------------

    % Legend with LaTeX interpreter (includes trajectories + targets + errors)
    allHandles = [plotHandles, errorHandles];
    allLabels  = [plotLabels,  errorLabels];
    [allHandles, allLabels] = orderLegend(allHandles, allLabels); % enforce order
    legend(allHandles, allLabels, 'Location', 'eastoutside', 'Interpreter', 'latex');

    % ==================== Joint Position Figure (FIGURE 2) ====================
    if hasPdata || hasSdata || hasGdata
        fig2 = figure('Name',[figPrefix ' - Joint Position']); %#ok<NASGU>
        for j = 1:3
            subplot(3,1,j); grid on; hold on;
            
            subplotHandles = gobjects(0);
            subplotLabels  = {};
            
            if ~isempty(PqAct)
                hP = plot(Ptime, PqAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorPhantom);
                subplotHandles(end+1) = hP;
                subplotLabels{end+1}  = 'Phantom ($q_{ph}$)';
            end
            
            if ~isempty(SqDes)
                hSd = plot(Stime, SqDes(:,j), '--', 'LineWidth', 1.2, 'Color', colorRef);
                subplotHandles(end+1) = hSd;
                subplotLabels{end+1}  = 'Reference ($q_{ref}$)';
            end
            if ~isempty(SqAct)
                hSa = plot(Stime, SqAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorSim);
                subplotHandles(end+1) = hSa;
                subplotLabels{end+1}  = 'Simulation ($q_{sim}$)';
            end
            
            if ~isempty(GqAct)
                hG = plot(Gtime, GqAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorGain);
                subplotHandles(end+1) = hG;
                subplotLabels{end+1}  = 'Gain Actual';
            end

            title(sprintf('Joint %d Position', j));
            if j == 3, xlabel('Time (s)'); end
            ylabel('Position (rad)');

            % Shade and label sections
            if hasSections
                shadeTimeSections(gca, sectionEdges, sectionLabels);
            end

            [subplotHandles, subplotLabels] = orderLegend(subplotHandles, subplotLabels);
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
            
            if ~isempty(PqdAct)
                hP = plot(Ptime, PqdAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorPhantom);
                subplotHandles(end+1) = hP;
                subplotLabels{end+1}  = 'Phantom ($\dot{q}_{ph}$)';
            end
            
            if ~isempty(SqdDes)
                hSd = plot(Stime, SqdDes(:,j), '--', 'LineWidth', 1.2, 'Color', colorRef);
                subplotHandles(end+1) = hSd;
                subplotLabels{end+1}  = 'Reference ($\dot{q}_{ref}$)';
            end
            if ~isempty(SqdAct)
                hSa = plot(Stime, SqdAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorSim);
                subplotHandles(end+1) = hSa;
                subplotLabels{end+1}  = 'Simulation ($\dot{q}_{sim}$)';
            end
            
            if ~isempty(GqdAct)
                hG = plot(Gtime, GqdAct(:,j), '-', 'LineWidth', 1.2, 'Color', colorGain);
                subplotHandles(end+1) = hG;
                subplotLabels{end+1}  = 'Gain Actual';
            end

            title(sprintf('Joint %d Velocity', j));
            if j == 3, xlabel('Time (s)'); end
            ylabel('Velocity (rad/s)');

            % Shade and label sections
            if hasSections
                shadeTimeSections(gca, sectionEdges, sectionLabels);
            end

            [subplotHandles, subplotLabels] = orderLegend(subplotHandles, subplotLabels);
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
    % XZ projection
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
    % and YZ (x=planes.x) using the SAME face color but black edge.
    x = pt(1); y = pt(2); z = pt(3);
    % XY projection
    plot3(ax, x, y, planes.z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'HandleVisibility','off');
    % XZ projection
    plot3(ax, x, planes.y, z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'HandleVisibility','off');
    % YZ projection (shifted +1 cm)
    plot3(ax, planes.x, y, z, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'HandleVisibility','off');
end

% ---------- Helper: order legend entries (Simulation, Phantom, Reference first) ----------
function [hOrdered, lOrdered] = orderLegend(h, l)
    desired = {'Simulation','Phantom','Reference'};
    idx = [];
    for k = 1:numel(desired)
        m = find(contains(l, desired{k}), 1, 'first');
        if ~isempty(m), idx(end+1) = m; end %#ok<AGROW>
    end
    rest = setdiff(1:numel(l), idx, 'stable');
    idx = [idx, rest];
    hOrdered = h(idx);
    lOrdered = l(idx);
end

% ---------- Helper: shade time sections on current axes ----------
function shadeTimeSections(ax, sectionEdges, sectionLabels)
    hold(ax, 'on');

    baseAlpha = 0.08;
    yL = ylim(ax);
    ySpan = [yL(1) yL(2)];

    if ~isempty(sectionEdges)
        allBounds = unique(sectionEdges(:));
        for k = 1:numel(allBounds)
            xline(ax, allBounds(k), '-', 'Color', [0 0 0 0.15], 'LineWidth', 1, ...
                'HandleVisibility','off', 'HitTest','off');
        end
    end

    for i = 1:size(sectionEdges,1)
        x0 = sectionEdges(i,1);
        x1 = sectionEdges(i,2);
        if ~isfinite(x0) || ~isfinite(x1) || x1 <= x0
            continue
        end

        col = 0.5 + 0.1*mod(i,2); % alternating gray
        ph = patch(ax, [x0 x1 x1 x0], [ySpan(1) ySpan(1) ySpan(2) ySpan(2)], col*[1 1 1], ...
            'FaceAlpha', baseAlpha, 'EdgeColor','none', ...
            'HandleVisibility','off', 'HitTest','off');

        xC = (x0 + x1)/2;
        yC = ySpan(1) + 0.92*(ySpan(2)-ySpan(1));
        if i <= numel(sectionLabels)
            text(ax, xC, yC, sectionLabels{i}, ...
                'HorizontalAlignment','center', 'VerticalAlignment','top', ...
                'FontSize', 9, 'Color', [0 0 0], 'Interpreter','none', ...
                'HitTest','off');
        end
        uistack(ph, 'bottom');
    end
end

% ---------- Helper: RMSE between a series and a reference ----------
function e = rmse_vs_ref(tA, A, tRef, Aref)
    e = NaN;
    if isempty(tA) || isempty(A) || isempty(tRef) || isempty(Aref)
        return
    end
    if all(isnan(A)) || all(isnan(Aref))
        return
    end

    tA   = tA(:);
    A    = A(:);
    tRef = tRef(:);
    Aref = Aref(:);

    ArefOnA = interp1(tRef, Aref, tA, 'pchip', NaN);

    mask = isfinite(A) & isfinite(ArefOnA);
    if nnz(mask) < 2
        return
    end

    diff = A(mask) - ArefOnA(mask);
    e = sqrt(mean(diff.^2));
end
