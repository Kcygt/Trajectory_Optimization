dataNum = 20;  % Change this to 2, 3, etc. for other runs

% Cartesian Space Trajectory 
figure; hold on; grid on;
plot(yi, zi,'--')
plot(yOpt,zOpt,'.-')
plot(xTarget(1,2),xTarget(1,3),'*')
plot(xTarget(2,2),xTarget(2,3),'*')
plot(xTarget(3,2),xTarget(3,3),'*')
plot(Opt(9),Opt(10),'d')

plot(xDes(2),xDes(3),'o')
legend('Initial Trjectory','Optimized Trajectory','Target Point 1','Target Point 2','Target Point 3','Control Point 1','Control Point 2','End Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
disp(['Optimal Parameter:', num2str(Opt)])
% saveas(gcf, sprintf('data%dCartesianPosition.fig', dataNum))  % Dynamic name


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
TargetMin1 = min(sqrt(sum(([xOpt, yOpt, zOpt] - xTarget(1,:)).^2,2)));
TargetMin2 = min(sqrt(sum(([xOpt, yOpt, zOpt] - xTarget(2,:)).^2,2)));
TargetMin3 = min(sqrt(sum(([xOpt, yOpt, zOpt] - xTarget(3,:)).^2,2)));


% Print results
fprintf('Minimum Target Error:    %.6f\n', TargetMin1);
fprintf('Minimum Target Error:    %.6f\n', TargetMin2);
fprintf('Minimum Target Error:    %.6f\n', TargetMin3);

fprintf('Position RMSE (rad):      %.6f\n', PosRMSE);
fprintf('Velocity RMSE (rad/s):    %.6f\n', VelRMSE);

desc = 'I used the time and wn in optimization , Opt = [time zeta wn]';
save(sprintf('data%d.mat', dataNum), ...
    'Opt','PosRMSE','VelRMSE','TargetMin1','TargetMin1', ...
    'tt','yy','tInit','yInit','xTarget','wt','desc');