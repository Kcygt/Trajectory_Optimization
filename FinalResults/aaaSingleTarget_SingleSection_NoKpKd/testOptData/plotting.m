% Cartesian Space Trajectory 
figure; hold on; grid on;
plot(y_Des,z_Des,'o')
plot(y_opt,z_opt,'.')
plot(xTarget(2),xTarget(3),'*')
plot(xDes(2),xDes(3),'o')
legend('Desired Trajectory','Optimized Trajectory','Target Point','End Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
