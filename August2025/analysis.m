close all
clear 
clc
load('Pdata.mat')

time = 0:0.001:5;  % 1x5001 vector
[x,y,z] = FK(Pdata(:,1),Pdata(:,2),Pdata(:,3));
Fz = Pdata(:,6);

xTarget =[0.0100   -0.0300         0;
          0.0400   -0.0300         0;
          0.0700   -0.0300         0;
          0.1000   -0.0300         0;
          0.1300   -0.0300         0];

xCtrl(1,:) = [ -0.0125  ,  -0.0336   -0.0021 ];
xCtrl(2,:) = [ 0.0560 ,  -0.0280  , -0.0010 ];
xCtrl(3,:) = [  0.1538 ,  -0.0305  ,  0.0071 ];

figure; hold on; grid on;
plot3(x,y,z)
plot3(0,0,0,'o')
plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'*')
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*')

figure; grid on; hold on;
plot(time,Fz)



% Find the index of minimum distance between trajectory and target point
FatTP = zeros(length(xTarget),1);
for i = 1:size(xTarget, 1)
    target_point_i = xTarget(i,:);
    distances_i = sqrt((x - target_point_i(1)).^2 + (y - target_point_i(2)).^2 + (z - target_point_i(3)).^2);
    [min_dist_i, min_idx_i] = min(distances_i);
    
    fprintf('Target %d [%.4f, %.4f, %.4f]: min distance = %.6f at index %d (t = %.3f s)\n', ...
        i, target_point_i(1), target_point_i(2), target_point_i(3), min_dist_i, min_idx_i, time(min_idx_i));

    fprintf('Actual robot position %d [%.7f]: \n', i,  y(min_idx_i));
    
    FatTP(i,1) = Fz(min_idx_i);
    plot(time(min_idx_i),Fz(min_idx_i),'*','MarkerSize',10,'LineWidth',2)
end

