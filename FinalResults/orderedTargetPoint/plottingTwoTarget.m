dataNum = 20;  % Change this to 2, 3, etc. for other runs


r = 0.01; % Radius
theta = linspace(0, 2*pi, 100); % Angle for circle

x_circle1 = Opt(8) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle1 = Opt(9) + r*sin(theta); % y = center_y + r*sin(θ)
x_circle2 = Opt(10) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle2 = Opt(11) + r*sin(theta); % y = center_y + r*sin(θ)
% Recompute distances to control points
ctrl1 = [0 Opt(8:9)];
ctrl2 = [0 Opt(10:11)];
dist1 = sqrt((x_opt - ctrl1(1)).^2 + (y_opt - ctrl1(2)).^2 + (z_opt - ctrl1(3)).^2);
dist2 = sqrt((x_opt - ctrl2(1)).^2 + (y_opt - ctrl2(2)).^2 + (z_opt - ctrl2(3)).^2);

% Find the first time where distance to control point 1 is less than threshold (phase 2 start)
tol = 0.01; % same as in your phase logic
idx_phase2 = find(dist1 <= tol, 1, 'first');
t_phase2 = tt(idx_phase2);

% Find the first time after phase 2 where distance to control point 2 is less than threshold (phase 3 start)
idx_phase3 = find(dist2 <= tol & tt <= t_phase2, 1, 'first');
t_phase3 = tt(idx_phase3);

% Plot vertical lines at phase change times
figure; hold on; grid on;
plot(y_opt, z_opt, '.-')
plot(xTarget(1,2), xTarget(1,3), '*')
plot(xTarget(2,2), xTarget(2,3), '*')
plot(xEnd(2), xEnd(3), 'o')
plot(Opt(8), Opt(9), 'd')
plot(Opt(10), Opt(11), 'd')
plot(x_circle1, y_circle1, 'b--', 'LineWidth', 1.5);
plot(x_circle2, y_circle2, 'b--', 'LineWidth', 1.5);

legend('Optimized Trajectory','Target Point 1','Target Point 2','End Point','Control Point 1','Control Point 2')
xlabel('Y axis (m)')
ylabel('Z axis (m)')
title('Cartesian Space Trajectory Results with Phase Changes')
disp('Optimal Parameter:')
disp(['tspan = [ ', num2str(Opt(1)), ' ];'])
disp(['wn1 =  [ ', num2str(Opt(2:3)), ' ];'])
disp(['wn2 =  [ ', num2str(Opt(4:5)), ' ];'])
disp(['wn3 =  [ ', num2str(Opt(6:7)), ' ];'])
disp(['CtrlPnt 1= [   ', num2str(Opt(8:9)), ' ];'])
disp(['CtrlPnt 2= [   ', num2str(Opt(10:11)), ' ];'])




% Velocity Plot
figure; hold on; grid on;
plot(tt, yy(:,10:12))
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
title('Velocity')


% Joint position 
figure;
for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(tt, yy(:,i), '--') % desired
    plot(tt, yy(:,i+6))     % actual
    ylabel(['Joint ', num2str(i), ' Position (rad)'])
    if i == 3
        xlabel('Time (s)')
    end
    legend('Desired', 'Actual')
end
% saveas(gcf, sprintf('data%dJointPosition.fig', dataNum))  % Dynamic name



% Joint Velocity 
figure;
for i = 4:6
    subplot(3,1,i-3); hold on; grid on;
    plot(tt, yy(:,i), '--') % desired
    plot(tt, yy(:,i+6))     % actual
    ylabel(['Joint ', num2str(i), ' Position (rad)'])
    if i == 3
        xlabel('Time (s)')
    end
    legend('Desired', 'Actual')
end
% saveas(gcf, sprintf('data%dJointVelocity.fig', dataNum))  % Dynamic name

% Calculate Position error
PosErr = yy(:,7:9) - yy(:,1:3); % assuming yy(:,6:9) = actual, yy(:,1:3) = desired

% Calculate Velocity error
VelErr = yy(:,10:12) - yy(:,4:6); % assuming yy(:,6:9) = actual, yy(:,1:3) = desired

% Compute RMSE
PosRMSE= sum(rmse(yy(:,7:9), yy(:,1:3)),2);
VelRMSE= sum(rmse(yy(:,10:12), yy(:,4:6)),2);

% Compute max absolute error
PosMaxErr = max(abs(PosErr));
VelMaxErr = max(abs(VelErr));
TargetMin1 = min(sqrt(sum(([x_opt, y_opt, z_opt] - xTarget(1,:)).^2,2)));
TargetMin2 = min(sqrt(sum(([x_opt, y_opt, z_opt] - xTarget(2,:)).^2,2)));


% Print results
fprintf('Minimum Target Error:    %.6f\n', TargetMin1);
fprintf('Minimum Target Error:    %.6f\n', TargetMin2);

fprintf('Position RMSE (rad):      %.6f\n', PosRMSE);
fprintf('Velocity RMSE (rad/s):    %.6f\n', VelRMSE);

desc = 'I used the time and wn in optimization , Opt = [time zeta wn]';
save(sprintf('data%d.mat', dataNum), ...
    'Opt','PosRMSE','VelRMSE','TargetMin1','TargetMin2', ...
    'tt','yy','tInit','yInit','xTarget','wt');