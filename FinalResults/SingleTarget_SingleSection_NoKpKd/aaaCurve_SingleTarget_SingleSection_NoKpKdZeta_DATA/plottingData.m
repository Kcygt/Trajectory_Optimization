Pdata = load('Pdata5.mat');
Sdata = load('data5.mat');

PqDes = Pdata.Pdata(:,1:3);
PqdDes = Pdata.Pdata(:,4:6);

PqAct = Pdata.Pdata(:,7:9);
PqdAct = Pdata.Pdata(:,10:12);


SqDes = Sdata.yy(:,1:3);
SqdDes = Sdata.yy(:,4:6);

SqAct = Sdata.yy(:,7:9);
SqdAct = Sdata.yy(:,10:12);

xTarget = Sdata.xTarget;
[SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3));
[SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3));
[PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3));


% Cartesian Space Position
figure; hold on; grid on;
plot(SyDes,SzDes,SyAct,SzAct,PyAct,PzAct )
plot(xTarget(2),xTarget(3),'*')
title('Cartesian Space Position')
xlabel('Y axis')
ylabel('Z axis')
legend('Desired Position','Simulation Trajectory','Phantom Trajectory')

