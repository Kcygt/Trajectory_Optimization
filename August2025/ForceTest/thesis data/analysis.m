% close all
% clear 
% clc
load('Sdata1.mat','xTarget','Opt')
load('Pdata1.mat')

time = linspace(0,Opt(1),length(Pdata1));  % 1x5001 vector
[xAct,yAct,zAct] = FK(Pdata1(:,1),Pdata1(:,2),Pdata1(:,3));
[xDes,yDes,zDes] = FK(Pdata1(:,4),Pdata1(:,5),Pdata1(:,6));

Fz = Pdata1(:,9);

xCtrl(1,:) = [ -0.0185, -0.03886, -0.0021 ];
xCtrl(2,:) = [ 0.0560, -0.0380, -0.0010 ];
xCtrl(3,:) = [ 0.1538, -0.0375, 0.0071 ];

figure; grid on; hold on;
plot(time,Fz)

% Find the index of minimum distance between trajectory and target point
minDist = zeros(length(xTarget),1);
Fact = zeros(length(xTarget),1);

indexing = zeros(5,1);
for i = 1:size(xTarget, 1)
    distance = sqrt((xAct - xTarget(i,1)).^2 + (yAct - xTarget(i,2)).^2 + (zAct - xTarget(i,3)).^2);
    [~, idx] = min(distance);
    
    
    indexing(i,1) = idx;
    minDist(i) = yAct(idx);
    Fact(i) = Fz(idx);
    plot(time(idx),Fact(i),'*','MarkerSize',10,'LineWidth',2)

end
% Force features
k = 532.3389;

Fdes = -1;


newTarget = zeros(5,1);
for i = 1:5
    newTarget(i) = (Fdes - Fact(i) ) / k + xTarget(i,2);
end

PosAvg = mean(yDes(indexing(1):indexing(5)) - yAct(indexing(1):indexing(5)));


figure; hold on; grid on;
plot3(xDes,yDes,zDes)
plot3(xAct,yAct,zAct)
plot3(0,0,0,'o')
plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'d')
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*')
plot3(xAct(indexing),yAct(indexing),zAct(indexing),'.',MarkerSize=15)
plot3(xTarget(:,1), newTarget,xTarget(:,3),'o')


legend('Desired Trajectory','Actual Trajectory','Home','Control points','Initial Target','Minimum dist','Updated Target Points')

newTarget
% % Estimate force at new target positions
% ForceEstimation = k * (yAct(indexing) - newTarget);
% 
% % Display
% disp('Estimated Force at Updated Target Positions:')
% disp(ForceEstimation)