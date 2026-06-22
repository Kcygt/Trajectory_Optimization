clear;

for i = 1:(numControlPoints+1)
    lb = [lb 0.01*ones(1,3)];
    ub = [ub 10*ones(1,3)];
end

for i = 1:numControlPoints
    lb = [lb xCtrl(i,:) - 0.025];
    ub = [ub xCtrl(i,:) + 0.025];
end

%% OPTIMIZATION
wt = [1000 10 0.0001];

objectiveFunc = @(params)objectiveFunction(...
    params,qDes,wt,xTarget,numControlPoints);

options = optimoptions('fmincon',...
    'Display','iter',...
    'Algorithm','sqp',...
    'MaxIterations',30);

[Opt,fval] = fmincon(...
    objectiveFunc,...
    OptNN,...
    [],[],[],[],...
    lb,ub,[],options);

fprintf('Optimization completed.\n');

%% SIMULATION
qCtrlOptimized = extractQctrl(Opt,numControlPoints);

wnParams = extractWn(Opt,numControlPoints);

qCtrlIK = zeros(numControlPoints,3);

for i = 1:numControlPoints
    qCtrlIK(i,:) = IK(...
        qCtrlOptimized(i,1),...
        qCtrlOptimized(i,2),...
        qCtrlOptimized(i,3));
end

qDesSimulation = [qCtrlIK; qFinal];

odeParams.qDes = qDesSimulation;
odeParams.qCtrl = qCtrlIK;
odeParams.wn = wnParams;

[t,y] = ode15s(@(t,x)robotDynamics(t,x,odeParams),...
    0:0.005:Opt(1),...
    zeros(12,1));

[xTraj,yTraj,zTraj] = FK(y(:,7),y(:,8),y(:,9));

%% PLOT
figure;
hold on;
grid on;
view(3);

plot3(xTraj,yTraj,zTraj,'LineWidth',2);
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'r*','MarkerSize',10);

xlabel('X');
ylabel('Y');
zlabel('Z');

title('Hybrid Neural Network + fmincon Optimization');