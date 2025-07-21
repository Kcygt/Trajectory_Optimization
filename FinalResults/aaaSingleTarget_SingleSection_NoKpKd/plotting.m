dataNum = 1;  % Change this to 2, 3, etc. for other runs

% Cartesian Space Trajectory 
figure; hold on; grid on;
plot(yi, zi,'--')
plot(y_Des,z_Des,'o')
plot(y_opt,z_opt,'.')
plot(xTarget(2),xTarget(3),'*')
plot(xDes(2),xDes(3),'o')
legend('Initial Trajectory','Desired Trajectory','Optimized Trajectory','Target Point','End Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
disp(['Optimal Parameter:', num2str(Opt)])


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


save(sprintf('data%d.mat', dataNum), ...
    'Opt','PosRMSE','VelRMSE', ...
    'tt','yy','tInit','yInit','xTarget');