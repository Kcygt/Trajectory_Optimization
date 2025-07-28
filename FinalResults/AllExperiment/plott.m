clear
close all

load("data21.mat")
[x,y,z] = FK(yOpt(:,7),yOpt(:,8),yOpt(:,9));

figure; hold on; grid on;
plot(y,z)
plot(0 ,0,'o')
plot(xTarget(1,2),xTarget(1,3),'*')
plot(xTarget(2,2),xTarget(2,3),'*')

plot(Opt(8),Opt(9),'d')
plot(Opt(10),Opt(11),'d')

legend('Actual Trajecotry','Home Position','Target 1','Target 2','Control Point 1','Control Point 2')
xlabel('Y axis')
ylabel('Z axis')
title('Cartesian Space Trajectory')