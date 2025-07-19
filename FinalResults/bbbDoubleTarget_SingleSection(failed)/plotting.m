dataNum = 13;  % Change this to 2, 3, etc. for other runs

% Cartesian Space Trajectory 
figure; hold on; grid on;
plot(yi, zi,'--')
plot(y_Des,z_Des,'o',  'LineWidth',1., 'Color', [0.1725, 0.6275, 0.1725],'MarkerSize',7)
plot(y_opt,z_opt,'-.', 'LineWidth',1.2, 'Color',[ 0.7969 0  0.7969])
plot(xTarget(1,2),xTarget(1,3),'*','LineWidth',1.2,'MarkerSize',8,'Color',[0, 0.3984, 0.8])
plot(xTarget(2,2),xTarget(2,3),'*','LineWidth',1.2,'MarkerSize',8, 'Color',[0, 0.3984, 0.8])

plot(xDes(2),xDes(3),'p','LineWidth',1.5,'MarkerSize',14,'Color',[1.0000, 0.4980, 0.0549])
legend('Initial Trajectory','Desired Trajectory','Optimized Trajectory','Target Point 1','Target Point 2','End Point')
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
TargetMin1 = min(sqrt(sum(([x_opt, y_opt, z_opt] - xTarget(1,:)).^2,2)));
TargetMin2 = min(sqrt(sum(([x_opt, y_opt, z_opt] - xTarget(2,:)).^2,2)));

% % Print results
fprintf('Minimum Target 1 Error:    %.6f\n', TargetMin1);
fprintf('Minimum Target 2 Error:    %.6f\n', TargetMin2);
fprintf('Position RMSE (rad):      %.6f\n', PosRMSE);
fprintf('Velocity RMSE (rad/s):    %.6f\n', VelRMSE);
% 
desc = 'I used the time and wn in optimization , Opt = [time zeta wn]';
save(sprintf('data%d.mat', dataNum), ...
    'Opt','PosRMSE','VelRMSE','TargetMin1', ...
    'tt','yy','tInit','yInit','xTarget','wt','desc');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

