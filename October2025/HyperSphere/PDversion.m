clear; clc; close all;
dataNumber = 77;

%% ===== CONFIGURATION SECTION =====
xTarget = [  0.01, -0.03, 0;
             0.04, -0.03, 0;
             0.07, -0.03, 0;
             0.1,  -0.03, 0;
             0.13, -0.03, 0];

controlPointIndices = [1,3,5];   % indices of control points
qDesTail = [0,0,0];
wt = [100, 1, 0.0001]; % [Target, End, Time]

tspan = 5;
wnValues = [2,2,2];
tolRadEps = 1e-10;
maxIterations = 60;
numStarts = 5;
rLower = 0.008; rUpper = 0.012; r0val = 0.01;
cpTol = 0.015;

%% ===== AUTOMATIC SETUP =====
numTargets = size(xTarget,1);
numControlPoints = length(controlPointIndices);
if any(controlPointIndices < 1) || any(controlPointIndices > numTargets)
    error('Control point indices must be valid');
end

xCtrl = xTarget(controlPointIndices,:);
qCtrl = zeros(numControlPoints,3);
for i = 1:numControlPoints
    qCtrl(i,:) = IK(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3));
end
qDes = [qCtrl; qDesTail];

[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];

%% ===== INITIAL PARAMETER VECTOR =====
initPrms = tspan;
% wn parameters
for i = 1:(numControlPoints+1)
    initPrms = [initPrms, wnValues];
end
% control points
for i = 1:numControlPoints
    initPrms = [initPrms, xCtrl(i,:)];
end
% radii
initPrms = [initPrms, r0val*ones(1,numControlPoints)];

% ===== Add initial Kp/Kd for optimization =====
Kp0 = 70*ones(1,3); Kd0 = 120*ones(1,3);
for i = 1:(numControlPoints+1)
    initPrms = [initPrms, Kp0, Kd0];
end

%% ===== INITIAL SIMULATION =====
t_uniform = 0:0.001:tspan;
[KpParamsInit,KdParamsInit] = extractGainParameters(initPrms,numControlPoints);

[tInit, yInit] = ode45(@(t,x) myTwolinkwithprefilter( ...
    t,x,qDes,tspan,extractWnParameters(initPrms,numControlPoints),...
    xCtrl, extractRadii(initPrms,numControlPoints),KpParamsInit,KdParamsInit), ...
    t_uniform, zeros(12,1));

%% ===== BOUNDS =====
lb = 0; ub = 5;
for i = 1:(numControlPoints+1)
    lb = [lb, 0.01*ones(1,3)]; ub = [ub, 10*ones(1,3)];
end
for i = 1:numControlPoints
    lb = [lb, xCtrl(i,:)-cpTol]; ub = [ub, xCtrl(i,:)+cpTol];
end
lb = [lb, rLower*ones(1,numControlPoints)]; ub = [ub, rUpper*ones(1,numControlPoints)];

% Kp/Kd bounds
for i = 1:(numControlPoints+1)
    lb = [lb, 60*ones(1,3), 110*ones(1,3)];
    ub = [ub, 80*ones(1,3), 130*ones(1,3)];
end

%% ===== OPTIMIZATION =====
objectiveFunc = @(prms) objectiveFunction(prms,qDes,wt,xTarget,xFinal,numControlPoints,controlPointIndices,tolRadEps);
nonlconFunc = @(prms) trajConstraint(prms,qDes,xTarget,xFinal,numControlPoints,controlPointIndices,tolRadEps);

options = optimoptions('fmincon','Display','none','Algorithm','sqp',...
    'TolCon',1e-8,'OptimalityTolerance',1e-8,'StepTolerance',1e-8,'MaxIterations',maxIterations);

problem = createOptimProblem('fmincon','objective',objectiveFunc,...
    'x0',initPrms,'lb',lb,'ub',ub,'options',options,'nonlcon',nonlconFunc);

ms = MultiStart('UseParallel',true,'Display','iter');
fprintf('\n=== Starting MultiStart Optimization ===\n');
[Opt, fval] = run(ms,problem,numStarts); %#ok<ASGLU>

%% ===== SIMULATE WITH OPTIMAL PARAMETERS =====
tOpt = 0:0.001:Opt(1);
[KpOpt,KdOpt] = extractGainParameters(Opt,numControlPoints);

[tOpt, yOpt] = ode45(@(t,x) myTwolinkwithprefilter( ...
    t,x,qDes,Opt(1),extractWnParameters(Opt,numControlPoints),...
    extractControlPoints(Opt,numControlPoints),extractRadii(Opt,numControlPoints),KpOpt,KdOpt), ...
    tOpt, zeros(12,1));

[CxOpt,CyOpt,CzOpt] = FK(yOpt(:,7),yOpt(:,8),yOpt(:,9));

%% ===== PLOTTING =====
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Optimized Cartesian Trajectory with %d Targets and %d Control Points',numTargets,numControlPoints));
plot3(CxOpt,CyOpt,CzOpt,'.-','LineWidth',2);
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*','MarkerSize',10);
plot3(xFinal(1),xFinal(2),xFinal(3),'o','MarkerSize',10);

ctrlPoints = extractControlPoints(Opt,numControlPoints);
for i = 1:numControlPoints
    plot3(ctrlPoints(i,1),ctrlPoints(i,2),ctrlPoints(i,3),'d','MarkerSize',10);
end

rOpt = extractRadii(Opt,numControlPoints);
[sx,sy,sz] = sphere(20);
for i = 1:numControlPoints
    surf(ctrlPoints(i,1)+rOpt(i)*sx,ctrlPoints(i,2)+rOpt(i)*sy,ctrlPoints(i,3)+rOpt(i)*sz,...
        'EdgeColor','none','FaceAlpha',0.3,'FaceColor','b');
end

legendEntries = {'Trajectory','Target Points','Final Point'};
for i = 1:numControlPoints
    legendEntries{end+1} = sprintf('Control Pt %d',i);
end
legend(legendEntries);

%% ===== DISPLAY RESULTS =====
fprintf('\nOptimized Parameters:\n');
fprintf('tspan = %.4f\n',Opt(1));

wnParams = extractWnParameters(Opt,numControlPoints);
for i = 1:length(wnParams)
    fprintf('wn%d = [%.4f %.4f %.4f]\n',i,wnParams{i});
end

for i = 1:numControlPoints
    fprintf('xCtrl(%d,:) = [%.4f %.4f %.4f]\n',i,ctrlPoints(i,:));
end

for i = 1:numControlPoints
    fprintf('rCtrl(%d) = %.4f\n',i,rOpt(i));
end

for i = 1:(numControlPoints+1)
    fprintf('Kp%d = [%.3f %.3f %.3f]\n',i,KpOpt{i});
    fprintf('Kd%d = [%.3f %.3f %.3f]\n',i,KdOpt{i});
end

save(sprintf('Sdata%d.mat',dataNumber),...
    'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal',...
    'numTargets','numControlPoints','controlPointIndices');

%% ===== HELPER FUNCTIONS =====

function wnParams = extractWnParameters(Opt,numControlPoints)
    wnParams = cell(numControlPoints+1,1);
    startIdx = 2;
    for i = 1:(numControlPoints+1)
        idx = startIdx + (i-1)*3;
        wnParams{i} = Opt(idx:idx+2);
    end
end

function ctrlPoints = extractControlPoints(Opt,numControlPoints)
    ctrlPoints = zeros(numControlPoints,3);
    startIdx = 2 + (numControlPoints+1)*3;
    for i = 1:numControlPoints
        idx = startIdx + (i-1)*3;
        ctrlPoints(i,:) = Opt(idx:idx+2);
    end
end

function rCtrl = extractRadii(Opt,numControlPoints)
    startIdx = 2 + (numControlPoints+1)*3 + numControlPoints*3;
    rCtrl = Opt(startIdx:startIdx+numControlPoints-1);
end

function [KpParams,KdParams] = extractGainParameters(Opt,numControlPoints)
    numSections = numControlPoints+1;
    startIdx = 2 + numSections*3 + numControlPoints*3 + numControlPoints;
    KpParams = cell(numSections,1);
    KdParams = cell(numSections,1);
    for i = 1:numSections
        idx = startIdx + (i-1)*6;
        KpParams{i} = Opt(idx:idx+2);
        KdParams{i} = Opt(idx+3:idx+5);
    end
end

function error = objectiveFunction(prms,qDes,wt,xTarget,xFinal,numControlPoints,controlPointIndices,tolRadEps)
    tUni = 0:0.001:prms(1);
    wnParams = extractWnParameters(prms,numControlPoints);
    ctrlPoints = extractControlPoints(prms,numControlPoints);
    rCtrl = extractRadii(prms,numControlPoints);
    [KpParams,KdParams] = extractGainParameters(prms,numControlPoints);

    [~,y] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),wnParams,ctrlPoints,rCtrl,KpParams,KdParams),...
        tUni,zeros(12,1));
    [x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x0,y0,z0];

    distMidF = 0;
    for i = 1:size(xTarget,1)
        di2 = min(sum((xOut - xTarget(i,:)).^2,2));
        cpIdx = find(controlPointIndices==i,1);
        if ~isempty(cpIdx)
            ri = rCtrl(cpIdx);
            over2 = max(0,di2-ri^2);
        else
            over2 = max(0,di2-tolRadEps);
        end
        distMidF = distMidF + over2;
    end

    distEndErr = sum((xOut(end,:) - xFinal).^2,2);
    timePenalty = prms(1);
    radiusReg = 1e-3 * sum(rCtrl.^2);
    error = wt(1)*distMidF + wt(2)*distEndErr + wt(3)*timePenalty + radiusReg;
end

function [c, ceq] = trajConstraint(prms,qDes,xTarget,xFinal,numControlPoints,controlPointIndices,tolRadEps)
    ceq = [];
    tUni = 0:0.001:prms(1);
    wnParams = extractWnParameters(prms,numControlPoints);
    ctrlPoints = extractControlPoints(prms,numControlPoints);
    rCtrl = extractRadii(prms,numControlPoints);
    [KpParams,KdParams] = extractGainParameters(prms,numControlPoints);

    [~,yy] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),wnParams,ctrlPoints,rCtrl,KpParams,KdParams),...
        tUni,zeros(12,1));
    [x0,y0,z0] = FK(yy(:,7),yy(:,8),yy(:,9));
    x = [x0,y0,z0];

    idxs = zeros(size(xTarget,1),1);
    for i = 1:size(xTarget,1)
        [~,idxs(i)] = min(sum((x - xTarget(i,:)).^2,2));
    end

    c = [];
    for i = 1:length(idxs)-1
        c(end+1) = idxs(i) - idxs(i+1);
    end

    for i = 1:size(xTarget,1)
        di2 = min(sum((x - xTarget(i,:)).^2,2));
        cpIdx = find(controlPointIndices==i,1);
        if ~isempty(cpIdx)
            ri = rCtrl(cpIdx);
            c(end+1) = di2 - ri^2;
        else
            c(end+1) = di2 - tolRadEps;
        end
    end

    c(end+1) = sum((x(end,:) - xFinal).^2) - tolRadEps;
end

function dxdt = myTwolinkwithprefilter(t,x,qDes,tspan,wnParams,xCtrl,rCtrl,KpParams,KdParams)
    persistent phase
    if t==0 || isempty(phase), phase = 1; end

    zeta = [1 1 1];
    q = x(7:9); qd = x(10:12);
    [x_now,y_now,z_now] = FK(q(1),q(2),q(3));
    x_curr = [x_now,y_now,z_now];

    numControlPoints = size(xCtrl,1);
    dists = zeros(numControlPoints,1);
    for i = 1:numControlPoints, dists(i) = norm(x_curr - xCtrl(i,:)); end

    qCtrl = zeros(numControlPoints,3);
    for i = 1:numControlPoints, qCtrl(i,:) = IK(xCtrl(i,1),xCtrl(i,2),xCtrl(i,3)); end

    if phase<=numControlPoints && dists(phase)<=rCtrl(phase)
        phase = phase + 1;
    end

    if phase<=numControlPoints
        wn = wnParams{phase};
        qControl = qCtrl(phase,:);
    else
        wn = wnParams{end};
        qControl = qDes(end,:);
    end

    A = [zeros(3),eye(3); -diag(wn).^2, -2*diag(zeta).*diag(wn)];
    B = [zeros(3); diag(wn).^2];

    Kp = diag(KpParams{phase});
    Kd = diag(KdParams{phase});

    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);

    [M,C,G] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    tau = M*controller + C*qd;
    qdd = M\(tau - C*qd);

    dxdt = [A*x(1:6)+B*qControl'; qd; qdd];
end

function [x,y,z]=FK(q1,q2,q3)
    l1=0.208; l2=0.168;
    x=sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y=l2-l2*cos(q3)+l1*sin(q2);
    z=-l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end

function Q=IK(x,y,z)
    l1=0.208; l2=0.168;
    q1=atan2(x,z+l1);
    R=sqrt(x^2+(z+l1)^2);
    r=sqrt(x^2+(y-l2)^2+(z+l1)^2);
    Beta=atan2(y-l2,R);
    Gamma=acos((l1^2+r^2-l2^2)/(2*l1*r));
    q2=Gamma+Beta;
    Alpha=acos((l1^2+l2^2-r^2)/(2*l1*l2));
    q3=q2+Alpha-pi/2;
    Q=[q1,q2,q3];
end

function [M,C,G]=compute_M_C_G(theta1,theta2,theta3,dtheta1,dtheta2,dtheta3)
    % Mass, Coriolis, Gravity (same as original)
    l1=0.208; l2=0.168; l3=0.0325;
    l5=-0.0368; l6=0.0527; g=-9.80665;
    m_a=0.0202; m_c=0.0249; m_be=0.2359; m_df=0.1906;
    Ia_xx=0.4864e-4; Ia_yy=0.001843e-4; Ia_zz=0.4864e-4;
    Ic_xx=0.959e-4; Ic_yy=0.959e-4; Ic_zz=0.0051e-4;
    Ibe_xx=11.09e-4; Ibe_yy=10.06e-4; Ibe_zz=0.591e-4;
    Idf_xx=7.11e-4; Idf_yy=0.629e-4; Idf_zz=6.246e-4;
    Ibaseyy = 11.87e-4;
    M11 = ( 1/8*( 4*Ia_yy  + 4*Ia_zz  + 8*Ibaseyy + 4*Ibe_yy + 4*Ibe_zz + 4*Ic_yy + 4*Ic_zz + 4*Idf_zz + 4*l1^2*m_a + l2^2*m_a + l1^2*m_c + 4*l3^2*m_c  ) + ...
            1/8*( 4*Ibe_yy - 4*Ibe_zz + 4*Ic_zz   + l1^2*(4*m_a + m_c)) * cos(2*theta2) + ...
            1/8*( 4*Ia_yy  - 4*Ia_zz  + 4*Idf_yy  - 4*Idf_zz - l2^2*m_a - 4*l3^2*m_c) * cos(2*theta3) + l1*(l2*m_a + l3*m_c)*cos(theta2)*sin(theta3)  );
    M22 = 1/4*(4*(Ibe_xx + Ic_xx + l1^2*m_a) + l1^2*m_c);
    M23 = -1/2*l1*(l2*m_a + l3*m_c) * sin(theta2-theta3);
    M32 = M23;
    M33 = 1/4 * (4*Ia_xx + 4*Idf_xx + l2^2*m_a + 4*l3^2*m_c);
    M = [M11 0 0;0 M22 M23;0 M32 M33];
    C = zeros(3);
    N2 = 1/2*g*(2*l1*m_a + 2*l5*m_be + l1*m_c)*cos(theta2);
    N3 = 1/2*g*(l2*m_a + 2*l3*m_c - 2*l6*m_df)*sin(theta3);
    G = [0 N2 N3]';
end