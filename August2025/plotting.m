%% === Optimized 3D Trajectory Visualization ===
figure('Color', 'w'); hold on; grid on; axis equal; view(45, 25);
xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Optimized Cartesian Trajectory (%d Targets, %d Phases)', ...
    numTargets, numPhases), 'FontSize', 13, 'FontWeight', 'bold');

%% === Find switching time indices ===
switchIndices = zeros(1, length(optimalTimes));
for i = 1:length(optimalTimes)
    [~, switchIndices(i)] = min(abs(tUni - optimalTimes(i)));
end

%% === Define distinct colormap for segments ===
numColors = max(numPhases, 6);
segmentColors = lines(numColors);  % visually distinct colors

%% === Plot trajectory segments ===
for i = 1:length(switchIndices)
    if i == 1
        startIdx = 1;
    else
        startIdx = switchIndices(i-1) + 1;
    end
    endIdx = switchIndices(i);

    colorIdx = mod(i-1, numColors) + 1;
    plot3(CxOpt(startIdx:endIdx), CyOpt(startIdx:endIdx), CzOpt(startIdx:endIdx), ...
          'Color', segmentColors(colorIdx,:), 'LineWidth', 2.2);
end

%% === Plot final segment (if any) ===
if ~isempty(switchIndices)
    plot3(CxOpt(switchIndices(end)+1:end), CyOpt(switchIndices(end)+1:end), CzOpt(switchIndices(end)+1:end), ...
          '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.8);
end

%% === Plot target and final points ===
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'p', 'MarkerSize', 10, ...
      'MarkerFaceColor', [0 0.45 0.74], 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Target Points');
plot3(xFinal(1), xFinal(2), xFinal(3), 'o', 'MarkerSize', 10, ...
      'MarkerFaceColor', [1 0.5 0.05], 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Final Point');

%% === Compute and plot distance errors ===
distanceErrors = zeros(size(xTarget,1), 1);
for i = 1:size(xTarget,1)
    diffs = [CxOpt, CyOpt, CzOpt] - xTarget(i,:);
    dists = vecnorm(diffs, 2, 2);
    [minDist, idxMin] = min(dists);
    distanceErrors(i) = minDist;
    closestPoint = [CxOpt(idxMin), CyOpt(idxMin), CzOpt(idxMin)];
    arrowVec = xTarget(i,:) - closestPoint;
    quiver3(closestPoint(1), closestPoint(2), closestPoint(3), ...
            arrowVec(1), arrowVec(2), arrowVec(3), ...
            0, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.4, 'MaxHeadSize', 0.5);
end

%% === Build legend dynamically ===
legendEntries = arrayfun(@(i) sprintf('Phase %d', i), 1:numPhases, 'UniformOutput', false);
legendEntries{end+1} = 'Final Phase';
legendEntries{end+1} = 'Target Points';
legendEntries{end+1} = 'Final Point';
legend(legendEntries, 'Location', 'northeastoutside', 'FontSize', 10);

%% === Annotate distance errors on plot ===
for i = 1:size(xTarget,1)
    text(xTarget(i,1), xTarget(i,2), xTarget(i,3) + 0.015, ...
        sprintf('%.3f m', distanceErrors(i)), ...
        'FontSize', 9, 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
end

%% === Display optimized parameters in console ===
fprintf('\n=== Optimized Parameters ===\n');
fprintf('Switching Times:\n');
for i = 1:length(optimalTimes)
    fprintf('  t%d = %.4f\n', i, optimalTimes(i));
end

fprintf('\nNatural Frequencies (ω_n):\n');
for i = 1:numPhases
    idx = (i-1)*3 + 1;
    fprintf('  Phase %d: [%.4f, %.4f, %.4f]\n', i, optimalWn(idx:idx+2));
end

%% === Final formatting ===
box on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, 'LineWidth', 1.1);


% % Plot segmented trajectory
% figure; hold on; grid on; view(3);
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% title(sprintf('Optimized Cartesian Trajectory with %d Targets, %d Phases', numTargets, numPhases));
% 
% 
% % === Find switching time indices ===
% switchIndices = zeros(1, length(optimalTimes));
% for i = 1:length(optimalTimes)
%     [~, switchIndices(i)] = min(abs(tUni - optimalTimes(i)));
% end
% 
% % === Plot trajectory segments in different colors ===
% colors = {'r', 'g', 'b', 'm', 'c', 'y'};
% for i = 1:length(switchIndices)
%     if i == 1
%         startIdx = 1;
%     else
%         startIdx = switchIndices(i-1) + 1;
%     end
%     endIdx = switchIndices(i);
% 
%     colorIdx = mod(i-1, length(colors)) + 1;
%     plot3(CxOpt(startIdx:endIdx), CyOpt(startIdx:endIdx), CzOpt(startIdx:endIdx), ...
%           'Color', colors{colorIdx}, 'LineWidth', 2);
% end
% 
% % === Plot final segment ===
% if length(switchIndices) > 0
%     plot3(CxOpt(switchIndices(end)+1:end), CyOpt(switchIndices(end)+1:end), CzOpt(switchIndices(end)+1:end), ...
%           'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
% end
% 
% % === Plot targets and final point ===
% plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', 'MarkerSize', 10, 'Color', [0,0.4,0.8]);
% plot3(xFinal(1), xFinal(2), xFinal(3), 'o', 'MarkerSize', 10, 'Color', [1,0.5,0.05]);
% 
% % === Build legend with distance errors ===
% % === Build legend with distance errors ===
% legendEntries = {};
% 
% % Add phase names only if more than one phase
% if length(switchIndices) ~= 1
%     for i = 1:length(switchIndices)
%         legendEntries{end+1} = sprintf('Phase %d', i);
%     end
%     legendEntries{end+1} = 'Final Phase';  % Only add if multiple phases
% else
%     legendEntries{end+1} = 'Phase 1';  % Single phase
% end
% 
% legendEntries{end+1} = 'Target Points';
% legendEntries{end+1} = 'Final Point';
% 
% distanceErrors = zeros(size(xTarget,1), 1);
% % Add distance error information to legend
% for i = 1:length(distanceErrors)
%     legendEntries{end+1} = sprintf('Distance Error %d: %.4f m', i, distanceErrors(i));
% end
% % === Calculate minimum distances to targets first ===
% for i = 1:size(xTarget,1)
%     % Compute distances from trajectory to this target
%     diffs = [CxOpt, CyOpt, CzOpt] - xTarget(i,:);
%     dists = vecnorm(diffs, 2, 2);  % Euclidean distance
% 
%     % Find minimum distance and index
%     [minDist, idxMin] = min(dists);
%     distanceErrors(i) = minDist;
% 
%     % Closest trajectory point
%     closestPoint = [CxOpt(idxMin), CyOpt(idxMin), CzOpt(idxMin)];
% 
%     % Vector from closest point to target
%     arrowVec = xTarget(i,:) - closestPoint;
% 
%     % Plot the arrow
%     quiver3(closestPoint(1), closestPoint(2), closestPoint(3), ...
%             arrowVec(1), arrowVec(2), arrowVec(3), ...
%             0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
% end
% legend(legendEntries, 'Location', 'northeastoutside');
% 
% 
% % === Display optimized parameters in console ===
% fprintf('\nOptimized Parameters:\n');
% fprintf('Switching Times:\n');
% for i = 1:length(optimalTimes)
%     fprintf('  t%d = %.4f\n', i, optimalTimes(i));
% end
% 
% fprintf('\nWn Parameters:\n');
% for i = 1:numPhases
%     idx = (i-1)*3 + 1;
%     fprintf('  wn%d = [%.4f, %.4f, %.4f]\n', i, optimalWn(idx:idx+2));
% end
