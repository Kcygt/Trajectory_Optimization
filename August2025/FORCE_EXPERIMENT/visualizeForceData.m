clear; clc; close all;

%% Load force data
dataNumber = 5;
load(sprintf('Fdata%d.mat', dataNumber));
load(sprintf('Rdata%d.mat', dataNumber));

%% Extract data
tOpt = forceData.tOpt;
forceProfile = forceData.forceProfile;
forceMagnitude = forceData.forceMagnitude;
contactStart = forceData.contactStart;
contactEnd = forceData.contactEnd;
targetForce = forceData.targetForce;
xTarget = forceData.xTarget;
xCtrl = forceData.xCtrl;
cartesianTraj = forceData.cartesianTraj;

referenceForce = referenceForceData.referenceForce;

%% Create comprehensive visualization
figure('Position', [50, 50, 1400, 1000]);

% Subplot 1: Force components over time
subplot(3,3,1);
plot(tOpt, forceProfile(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Fx');
hold on;
plot(tOpt, forceProfile(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Fy');
plot(tOpt, forceProfile(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Fz');
plot(tOpt, forceMagnitude, 'k--', 'LineWidth', 2, 'DisplayName', '|F|');
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Force Components vs Time');
legend('Location', 'best');

% Subplot 2: Force magnitude vs reference
subplot(3,3,2);
plot(tOpt, forceMagnitude, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Force');
hold on;
plot(tOpt, referenceForce, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference Force');
% Mark contact region
patch([tOpt(contactStart), tOpt(contactEnd), tOpt(contactEnd), tOpt(contactStart)], ...
      [0, 0, max(forceMagnitude)*1.1, max(forceMagnitude)*1.1], ...
      'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Contact Region');
grid on;
xlabel('Time (s)');
ylabel('Force Magnitude (N)');
title('Force Magnitude vs Reference');
legend('Location', 'best');

% Subplot 3: Force error
subplot(3,3,3);
forceError = forceMagnitude - referenceForce;
plot(tOpt, forceError, 'm-', 'LineWidth', 2);
hold on;
plot([tOpt(1), tOpt(end)], [0, 0], 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Force Error (N)');
title('Force Tracking Error');
legend('Force Error', 'Zero Reference', 'Location', 'best');

% Subplot 4: 3D trajectory with force visualization and surface
subplot(3,3,4);

% Create rectangular surface with specified dimensions
% Dimensions: 15cm X, 4cm Y, 4cm Z, centered at (0,0,0)
% X: [-0.075, 0.075], Y: [-0.02, 0.02], Z: [-0.02, 0.02]
x_surface = linspace(-0.075, 0.075, 50);  % 15cm = 0.15m, so ±0.075m
y_surface = linspace(-0.02, 0.02, 20);    % 4cm = 0.04m, so ±0.02m
[X_surface, Y_surface] = meshgrid(x_surface, y_surface);
Z_surface = zeros(size(X_surface)); % Flat rectangular surface at Z = 0

% Plot the rectangular surface
surf(X_surface, Y_surface, Z_surface, 'FaceAlpha', 0.2, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 0.5);
hold on;

scatter3(cartesianTraj(:,1), cartesianTraj(:,2), cartesianTraj(:,3), 20, forceMagnitude, 'filled');
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'k*', 'MarkerSize', 10, 'DisplayName', 'Targets');
plot3(xCtrl(:,1), xCtrl(:,2), xCtrl(:,3), 'rd', 'MarkerSize', 10, 'DisplayName', 'Control Points');
colorbar;
colormap(jet);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory with Force Magnitude and Surface');
legend('3D Surface', 'Trajectory', 'Targets', 'Control Points', 'Location', 'best');

% Set axis limits to include the surface
xlim([-0.08, 0.08]);
ylim([-0.03, 0.03]);
zlim([-0.03, 0.03]);

% Subplot 5: Force vs position
subplot(3,3,5);
trajectoryDistance = cumsum([0; sqrt(diff(cartesianTraj(:,1)).^2 + diff(cartesianTraj(:,2)).^2 + diff(cartesianTraj(:,3)).^2)]);
plot(trajectoryDistance, forceMagnitude, 'b-', 'LineWidth', 2);
hold on;
patch([trajectoryDistance(contactStart), trajectoryDistance(contactEnd), ...
       trajectoryDistance(contactEnd), trajectoryDistance(contactStart)], ...
      [0, 0, max(forceMagnitude)*1.1, max(forceMagnitude)*1.1], ...
      'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Contact Region');
grid on;
xlabel('Distance along trajectory (m)');
ylabel('Force Magnitude (N)');
title('Force vs Position');
legend('Force Magnitude', 'Contact Region', 'Location', 'best');

% Subplot 6: Force statistics
subplot(3,3,6);
contactForces = forceMagnitude(contactStart:contactEnd);
histogram(contactForces, 20, 'FaceColor', 'blue', 'FaceAlpha', 0.7);
hold on;
plot([targetForce, targetForce], [0, max(histcounts(contactForces))], 'r--', 'LineWidth', 2, 'DisplayName', 'Target Force');
grid on;
xlabel('Force Magnitude (N)');
ylabel('Frequency');
title('Force Distribution in Contact Region');
legend('Force Distribution', 'Target Force', 'Location', 'best');

% Subplot 7: Contact region zoom
subplot(3,3,7);
contactTime = tOpt(contactStart:contactEnd);
contactForceData = forceMagnitude(contactStart:contactEnd);
plot(contactTime, contactForceData, 'b-', 'LineWidth', 2);
hold on;
plot(contactTime, ones(size(contactTime))*targetForce, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Force Magnitude (N)');
title('Contact Region Detail');
legend('Actual Force', 'Target Force', 'Location', 'best');

% Subplot 8: Force components in contact region
subplot(3,3,8);
contactForceProfile = forceProfile(contactStart:contactEnd, :);
plot(contactTime, contactForceProfile(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Fx');
hold on;
plot(contactTime, contactForceProfile(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Fy');
plot(contactTime, contactForceProfile(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Fz');
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Force Components in Contact Region');
legend('Location', 'best');

% Subplot 9: Performance metrics
subplot(3,3,9);
% Calculate performance metrics
rmse = sqrt(mean(forceError(contactStart:contactEnd).^2));
mae = mean(abs(forceError(contactStart:contactEnd)));
maxError = max(abs(forceError(contactStart:contactEnd)));
avgForce = mean(contactForces);

% Create text display
text(0.1, 0.8, sprintf('Performance Metrics:'), 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.7, sprintf('RMSE: %.4f N', rmse), 'FontSize', 10);
text(0.1, 0.6, sprintf('MAE: %.4f N', mae), 'FontSize', 10);
text(0.1, 0.5, sprintf('Max Error: %.4f N', maxError), 'FontSize', 10);
text(0.1, 0.4, sprintf('Avg Force: %.4f N', avgForce), 'FontSize', 10);
text(0.1, 0.3, sprintf('Target Force: %.1f N', targetForce), 'FontSize', 10);
text(0.1, 0.2, sprintf('Contact Duration: %.3f s', tOpt(contactEnd) - tOpt(contactStart)), 'FontSize', 10);

axis([0 1 0 1]);
axis off;
title('Performance Summary');

%% Display detailed statistics
fprintf('\n=== Detailed Force Analysis ===\n');
fprintf('Contact Region:\n');
fprintf('  Start time: %.3f s\n', tOpt(contactStart));
fprintf('  End time: %.3f s\n', tOpt(contactEnd));
fprintf('  Duration: %.3f s\n', tOpt(contactEnd) - tOpt(contactStart));

fprintf('\nForce Statistics:\n');
fprintf('  Target force: %.1f N\n', targetForce);
fprintf('  Average force: %.4f N\n', avgForce);
fprintf('  Min force: %.4f N\n', min(contactForces));
fprintf('  Max force: %.4f N\n', max(contactForces));
fprintf('  Std deviation: %.4f N\n', std(contactForces));

fprintf('\nTracking Performance:\n');
fprintf('  RMSE: %.4f N\n', rmse);
fprintf('  MAE: %.4f N\n', mae);
fprintf('  Max error: %.4f N\n', maxError);
fprintf('  Mean error: %.4f N\n', mean(forceError(contactStart:contactEnd)));

%% Save the figure
saveas(gcf, sprintf('ForceAnalysis_Data%d.png', dataNumber));
saveas(gcf, sprintf('ForceAnalysis_Data%d.fig', dataNumber));
fprintf('\nForce analysis figure saved as ForceAnalysis_Data%d.png/.fig\n', dataNumber);
