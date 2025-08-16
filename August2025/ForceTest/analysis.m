% close all
% clear 
% clc
load('P3data.mat')

time = 0:0.001:5;  % 1x5001 vector
[x,y,z] = FK(P3data(:,1),P3data(:,2),P3data(:,3));
Fz = P3data(:,6);

xTarget =[0.0100   -0.0300         0;
          0.0400   -0.0300         0;
          0.0700   -0.0300         0;
          0.1000   -0.0300         0;
          0.1300   -0.0300         0];

xCtrl(1,:) = [ -0.0125  ,  -0.0336   -0.0021 ];
xCtrl(2,:) = [  0.0560 ,  -0.0280  , -0.0010 ];
xCtrl(3,:) = [  0.1538 ,  -0.0305  ,  0.0071 ];

figure; grid on; hold on;
plot(time,Fz)



% Find the index of minimum distance between trajectory and target point
TargetAndForce = zeros(length(xTarget),2);
indexx = zeros(5,1);
for i = 1:size(xTarget, 1)
    target_point_i = xTarget(i,:);
    distances_i = sqrt((x - target_point_i(1)).^2 + (y - target_point_i(2)).^2 + (z - target_point_i(3)).^2);
    [min_dist_i, min_idx_i] = min(distances_i);
    

    indexx(i,1) = min_idx_i;
    TargetAndForce(i,:) = [ y(min_idx_i) Fz(min_idx_i)];
    plot(time(min_idx_i),Fz(min_idx_i),'*','MarkerSize',10,'LineWidth',2)

end

k = 532.3389;
constant =  0.4897;
Fdes = -1;


Xdes = zeros(5,1);
for i = 1:5
    Xdes(i) = (Fdes - TargetAndForce(i,2) - constant) / k + TargetAndForce(i,1);
end



figure; hold on; grid on;
plot3(x,y,z)
plot3(0,0,0,'o')
plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'*')
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*')
plot3(xTarget(:,1), TargetAndForce(:,1),xTarget(:,3),'.',MarkerSize=15)
plot3(xTarget(:,1), Xdes,xTarget(:,3),'o')

legend('Trajectory','Home','Control points','Initial Target','Minimum dist','Updated Target Points')