%% ======================= Publication-Quality 3D Trajectory =======================
% Expects these in workspace:
% CxOpt, CyOpt, CzOpt, tUni, optimalTimes, xTarget [Nx3], xFinal [1x3],
% numTargets, numPhases, optimalWn

%% ----------------------- Figure & axes styling -----------------------------------
fig = figure('Color','w','Units','centimeters','Position',[2 2 18 14]); %#ok<NASGU>
tiledlayout(1,1,'Padding','compact','TileSpacing','compact');
ax = nexttile; hold(ax,'on'); box(ax,'on'); grid(ax,'on'); ax.GridAlpha = 0.15;
axis(ax,'equal'); view(ax, 42, 26);
ax.LineWidth = 1.1;
ax.FontName = 'Times New Roman'; ax.FontSize = 10;

xlabel(ax, '$X$ (m)','Interpreter','latex','FontSize',12,'FontWeight','bold');
ylabel(ax, '$Y$ (m)','Interpreter','latex','FontSize',12,'FontWeight','bold');
zlabel(ax, '$Z$ (m)','Interpreter','latex','FontSize',12,'FontWeight','bold');

title(ax, sprintf('Optimized Cartesian Trajectory (%d Targets, %d Phases)', ...
    numTargets, numPhases), 'FontSize', 13, 'FontWeight', 'bold','Interpreter','none');

%% ----------------------- Data prep ----------------------------------------------
C = [CxOpt(:), CyOpt(:), CzOpt(:)];
span = max(range(C,1));

% Robust switching indices
switchIndices = zeros(1, numel(optimalTimes));
for i = 1:numel(optimalTimes)
    [~, switchIndices(i)] = min(abs(tUni - optimalTimes(i)));
end
switchIndices = unique(max(1, min(switchIndices, numel(tUni))));
segStarts = [1, switchIndices + 1];
segEnds   = [switchIndices, numel(tUni)];
nSegs = numel(segStarts);

%% ----------------------- Trajectory rendering -----------------------------------
baseColors = lines(max(numPhases,6));
trajLW = 2.2; finalLW = 1.8;

for s = 1:nSegs
    thisC = C(segStarts(s):segEnds(s), :);
    if s < nSegs
        plot3(ax, thisC(:,1), thisC(:,2), thisC(:,3), ...
            'Color', baseColors(mod(s-1,size(baseColors,1))+1,:), ...
            'LineWidth', trajLW, 'HandleVisibility','off');
    else
        plot3(ax, thisC(:,1), thisC(:,2), thisC(:,3), ...
            '--', 'Color', [0.25 0.25 0.25], ...
            'LineWidth', finalLW, 'HandleVisibility','off');
    end
end

% Phase boundary markers
if ~isempty(switchIndices)
    swC = C(switchIndices, :);
    scatter3(ax, swC(:,1), swC(:,2), swC(:,3), ...
        38, 0.15+[0.85 0.85 0.85], 'filled', 'MarkerEdgeColor',[0.2 0.2 0.2], ...
        'HandleVisibility','off');
end

% Direction arrows
Ndir = max(12, round(numel(tUni)/150));
idxDir = unique(round(linspace(1, numel(tUni)-1, Ndir)));
dirVec = C(idxDir+1,:) - C(idxDir,:);
normDir = dirVec ./ max(1e-9, vecnorm(dirVec,2,2));
arrowLen = 0.02 * span;
quiver3(ax, C(idxDir,1), C(idxDir,2), C(idxDir,3), ...
    normDir(:,1)*arrowLen, normDir(:,2)*arrowLen, normDir(:,3)*arrowLen, ...
    0, 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9, 'MaxHeadSize', 0.8, ...
    'HandleVisibility','off');

% Proxy line for legend
hTrajLegend = plot3(ax, nan, nan, nan, '-', 'Color', [0 0.45 0.74], ...
    'LineWidth', 2.2, 'DisplayName', 'Trajectory');

%% ----------------------- Start, Targets, Final ----------------------------------
% Start Point
startPt = C(1,:);
hStart = scatter3(ax, startPt(1), startPt(2), startPt(3), 55, 's', 'filled', ...
    'MarkerFaceColor', [0.20 0.70 0.20], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Start Point');

% Label start point
text(startPt(1), startPt(2), startPt(3) + 0.02*span, 'Start', ...
    'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1], ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom');

% Target points (stars)
hTarget = plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', ...
    'MarkerSize', 8, 'LineWidth', 1.2, 'Color', [0 0.45 0.74], ...
    'MarkerEdgeColor','k', 'DisplayName', 'Target');

% Final Point (orange circle)
hFinal = scatter3(ax, xFinal(1), xFinal(2), xFinal(3), 65, ...
    'o', 'filled', 'MarkerFaceColor', [1 0.50 0.05], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Final Point');

%% ----------------------- Distance errors & labels (text BELOW target) ------------
distanceErrors = zeros(size(xTarget,1),1);
closestIdx = zeros(size(xTarget,1),1);

for i = 1:size(xTarget,1)
    diffs = C - xTarget(i,:);
    dists = vecnorm(diffs, 2, 2);
    [minDist, idxMin] = min(dists);
    distanceErrors(i) = minDist; closestIdx(i) = idxMin;

    closestPoint = C(idxMin,:);
    arrowVec = xTarget(i,:) - closestPoint;
    quiver3(ax, closestPoint(1), closestPoint(2), closestPoint(3), ...
            arrowVec(1), arrowVec(2), arrowVec(3), ...
            0, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2, 'MaxHeadSize', 0.5, ...
            'HandleVisibility','off');
    
    % --- label now UNDER each target ---
    text(xTarget(i,1), xTarget(i,2), xTarget(i,3) - 0.02*span, ...
        sprintf('%.3f m', minDist), ...
        'FontSize', 9, 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold', ...
        'HorizontalAlignment','center', 'VerticalAlignment','top');
end

%% ----------------------- Legend --------------------------------------------------
legend([hTrajLegend, hStart, hFinal, hTarget], ...
       {'Trajectory','Start Point','Final Point','Target'}, ...
       'Location','northeastoutside','FontSize',10);

%% ----------------------- Stats annotation ---------------------------------------
meanErr = mean(distanceErrors);
maxErr  = max(distanceErrors);
minErr  = min(distanceErrors);
[~, worstIdx] = max(distanceErrors);

wnLines = strings(numPhases,1);
for i = 1:numPhases
    idx = (i-1)*3 + 1;
    wn = optimalWn(idx:idx+2);
    wnLines(i) = sprintf('Phase %d: [%.4f, %.4f, %.4f]', i, wn(1), wn(2), wn(3));
end
swLines = strings(numel(optimalTimes),1);
for i = 1:numel(optimalTimes)
    swLines(i) = sprintf('t_{%d} = %.4f', i, optimalTimes(i));
end

annText = sprintf([ ...
    '\\bfDistance Errors\\rm\n' ...
    'min = %.4f m, mean = %.4f m, max = %.4f m (Target %d)\n\n' ...
    '\\bfSwitching Times\\rm\n%s\n\n' ...
    '\\bfNatural Frequencies (\\omega_n)\\rm\n%s' ...
    ], minErr, meanErr, maxErr, worstIdx, ...
       strjoin(cellstr(swLines), '\n'), strjoin(cellstr(wnLines), '\n') );

annotation('textbox',[0.72 0.05 0.26 0.42], 'Interpreter','tex', ...
    'String', annText, 'FontSize',9, 'EdgeColor',[0.7 0.7 0.7], ...
    'BackgroundColor',[1 1 1], 'Margin',8, 'LineWidth',0.8);

%% ----------------------- Console summary ----------------------------------------
fprintf('\n=== Optimized Parameters ===\n');
fprintf('Switching Times:\n  %s\n', strjoin(cellstr(swLines), ', '));
fprintf('\nNatural Frequencies (ω_n):\n  %s\n', strjoin(cellstr(wnLines), '\n  '));
fprintf('\nDistance Errors (m): min=%.4f, mean=%.4f, max=%.4f (Target %d)\n', ...
    minErr, meanErr, maxErr, worstIdx);

%% ----------------------- Limits & export ----------------------------------------
pad = 0.04 * span;
xlim(ax, [min(C(:,1))-pad, max(C(:,1))+pad]);
ylim(ax, [min(C(:,2))-pad, max(C(:,2))+pad]);
zlim(ax, [min(C(:,3))-pad, max(C(:,3))+pad]);
ax.TickDir = 'out';
ax.MinorGridAlpha = 0.07;
ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off'; ax.ZMinorGrid = 'off';

outBase = 'optimized_trajectory';
exportgraphics(gcf, outBase + ".pdf", 'ContentType','vector');           % vector for thesis
exportgraphics(gcf, outBase + ".png", 'Resolution', 600, 'BackgroundColor','white'); % high-DPI PNG
