%%% Plotting
[xi, yi_plot, zi] = FK(yi(:,7), yi(:,8), yi(:,9));     % Initial Trajectory
[x_opt, y_opt, z_opt] = FK(yy(:,7), yy(:,8), yy(:,9)); % Optimized Trajectory

figure; hold on; grid on;
plot(yi_plot, zi,'--')
plot(y_opt,z_opt,'.-')
plot(xMid(1,2),xMid(1,3),'*')
plot(xMid(2,2),xMid(2,3),'*')
plot(xMid(3,2),xMid(3,3),'*')
plot(xDes(2),xDes(3),'o')
plot(Opt(10),Opt(11),'d')
legend('Initial Trajectory','Optimized Trajectory','Target Point 1','Target Point 2','End Point','Control Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
disp('Optimal Parameter:')
disp(['tspan = [ ', num2str(Opt(1:2)), ' ];'])
disp(['wn1 =  [ ', num2str(Opt(3:5)), ' ];'])
disp(['wn2 =  [ ', num2str(Opt(6:8)), ' ];'])
disp(['CtrlPnt = [   ', num2str(Opt(9:11)), ' ];'])