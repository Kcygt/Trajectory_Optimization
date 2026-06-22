%% Phase1_GenerateTrainingData.m
% Runs your existing fmincon optimizer across many random target configurations
% and saves (input, optimal_output) pairs for NN training.
%
% OUTPUT: TrainingData.mat containing:
%   inputData  - [N x inputDim]  each row = flattened target positions
%   outputData - [N x outputDim] each row = optimal fmincon parameters

clear; clc; close all;

%% ===== DATASET SETTINGS =====
numSamples       = 500;    % Number of random configurations to generate
                           % Increase to 2000+ for better generalisation
numTargets       = 5;      % Must match your real problem
numControlPoints = 3;      % Must match your real problem
controlPointIndices = [1, 3, 5];

% Workspace bounds — targets are sampled uniformly inside this box
xBounds = [0.01, 0.13];   % x range (metres)
yBounds = [-0.05, -0.01]; % y range
zBounds = [0.0,   0.0];   % z range (planar; keep fixed if your robot is planar)

% Optimiser settings (same as your main script)
wt        = [1000, 10, 0.0001];
tspan     = 5;
wnValues  = [2, 2, 2];
tolRad    = 0.025;
maxIter   = 60;
numStarts = 3;   % Fewer starts per sample to keep generation time reasonable
qDes_end  = [0, 0, 0];

%% ===== DIMENSION SIZES =====
% Input:  numTargets * 3  (x,y,z of every target)
% Output: 1 (tspan) + 3*(numControlPoints+1) (wn) + 3*numControlPoints (ctrl pts)
inputDim  = numTargets * 3;
outputDim = 1 + 3*(numControlPoints + 1) + 3*numControlPoints;

inputData  = zeros(numSamples, inputDim);
outputData = zeros(numSamples, outputDim);
validMask  = false(numSamples, 1);  % Track which samples converged

fprintf('Generating %d training samples...\n', numSamples);
fprintf('Input dim: %d   Output dim: %d\n\n', inputDim, outputDim);

%% ===== GENERATION LOOP =====
for s = 1:numSamples
    fprintf('Sample %d / %d ... ', s, numSamples);

    % --- Sample random target positions ---
    xTarget = zeros(numTargets, 3);
    xTarget(:,1) = sort(xBounds(1) + diff(xBounds) * rand(numTargets,1)); % sorted in x
    xTarget(:,2) = yBounds(1)   + diff(yBounds)   * rand(numTargets,1);
    xTarget(:,3) = zBounds(1)   + diff(zBounds)   * rand(numTargets,1);

    % --- IK for control points ---
    xCtrl  = xTarget(controlPointIndices, :);
    qCtrl  = zeros(numControlPoints, 3);
    valid  = true;
    for i = 1:numControlPoints
        try
            qCtrl(i,:) = IK(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3));
        catch
            valid = false; break;
        end
    end
    if ~valid
        fprintf('IK failed — skipping\n'); continue;
    end

    qDes = [qCtrl; qDes_end];
    [Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
    xFinal = [Px, Py, Pz];

    % --- Build initial parameter vector ---
    initPrms = tspan;
    for i = 1:(numControlPoints + 1)
        initPrms = [initPrms, wnValues]; %#ok<AGROW>
    end
    for i = 1:numControlPoints
        initPrms = [initPrms, xCtrl(i,:)]; %#ok<AGROW>
    end

    % --- Bounds ---
    lb = 0;
    ub = 5;
    for i = 1:(numControlPoints + 1)
        lb = [lb, 0.01*ones(1,3)]; %#ok<AGROW>
        ub = [ub, 10*ones(1,3)];   %#ok<AGROW>
    end
    for i = 1:numControlPoints
        lb = [lb, xCtrl(i,:) - tolRad]; %#ok<AGROW>
        ub = [ub, xCtrl(i,:) + tolRad]; %#ok<AGROW>
    end

    % --- Run optimiser ---
    try
        objFcn  = @(p) objectiveFunction(p, qDes, wt, xTarget, xFinal, numControlPoints);
        conFcn  = @(p) trajConstraint(p, qDes, xTarget, xFinal, numControlPoints);
        opts    = optimoptions('fmincon','Display','none','Algorithm','sqp',...
                               'TolCon',1e-8,'OptimalityTolerance',1e-8,...
                               'StepTolerance',1e-8,'MaxIterations',maxIter);
        problem = createOptimProblem('fmincon','objective',objFcn,'x0',initPrms,...
                                     'lb',lb,'ub',ub,'options',opts,'nonlcon',conFcn);
        ms      = MultiStart('UseParallel',false,'Display','off');
        [Opt, fval] = run(ms, problem, numStarts);

        inputData(s,:)  = xTarget(:)';    % flatten column-major
        outputData(s,:) = Opt(1:outputDim);
        validMask(s)    = true;
        fprintf('OK  fval=%.4e\n', fval);
    catch ME
        fprintf('FAILED: %s\n', ME.message);
    end
end

%% ===== TRIM AND SAVE =====
inputData  = inputData(validMask,:);
outputData = outputData(validMask,:);
numValid   = sum(validMask);

fprintf('\n%d / %d samples valid.\n', numValid, numSamples);
save('TrainingData.mat', 'inputData','outputData','numTargets','numControlPoints',...
     'controlPointIndices','inputDim','outputDim');
fprintf('Saved TrainingData.mat\n');

%% ===== EMBEDDED HELPER FUNCTIONS (copies from main script) =====

function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal, numControlPoints)
    tUni = 0:0.001:prms(1);
    wnParams   = extractWnParameters(prms, numControlPoints);
    ctrlPoints = extractControlPoints(prms, numControlPoints);
    [~, y] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),wnParams,ctrlPoints),...
                   tUni, zeros(12,1));
    [x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x0,y0,z0];
    distMidF = 0;
    for i = 1:size(xTarget,1)
        distMidF = distMidF + min(sum((xOut - xTarget(i,:)).^2, 2));
    end
    distEndErr  = sum((xOut(end,:) - xFinal).^2,2);
    error = wt(1)*distMidF + wt(2)*distEndErr + wt(3)*prms(1);
end

function [c, ceq] = trajConstraint(prms, qDes, xTarget, xFinal, numControlPoints)
    ceq  = [];
    tUni = 0:0.001:prms(1);
    wnParams   = extractWnParameters(prms, numControlPoints);
    ctrlPoints = extractControlPoints(prms, numControlPoints);
    [~, yy] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),wnParams,ctrlPoints),...
                    tUni, zeros(12,1));
    [x0,y0,z0] = FK(yy(:,7),yy(:,8),yy(:,9));
    x = [x0,y0,z0];
    idxs = zeros(size(xTarget,1),1);
    for i = 1:size(xTarget,1)
        [~, idxs(i)] = min(sum((x - xTarget(i,:)).^2, 2));
    end
    c = [];
    for i = 1:length(idxs)-1
        c(end+1) = idxs(i) - idxs(i+1); %#ok<AGROW>
    end
    for i = 1:size(xTarget,1)
        c(end+1) = min(sum((x - xTarget(i,:)).^2, 2)) - 1e-10; %#ok<AGROW>
    end
    c(end+1) = sum((x(end,:) - xFinal).^2) - 1e-10;
end

function wnParams = extractWnParameters(Opt, numControlPoints)
    wnParams = cell(numControlPoints+1, 1);
    startIdx = 2;
    for i = 1:(numControlPoints+1)
        idx = startIdx + (i-1)*3;
        wnParams{i} = Opt(idx:idx+2);
    end
end

function ctrlPoints = extractControlPoints(Opt, numControlPoints)
    ctrlPoints = zeros(numControlPoints, 3);
    startIdx   = 2 + (numControlPoints+1)*3;
    for i = 1:numControlPoints
        idx = startIdx + (i-1)*3;
        ctrlPoints(i,:) = Opt(idx:idx+2);
    end
end

function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wnParams, xCtrl)
    persistent phase
    if t == 0 || isempty(phase), phase = 1; end
    zeta = [1 1 1];
    q  = x(7:9);
    qd = x(10:12);
    [x_now, y_now, z_now] = FK(q(1),q(2),q(3));
    x_curr = [x_now, y_now, z_now];
    numControlPoints = size(xCtrl,1);
    dists = zeros(numControlPoints,1);
    for i = 1:numControlPoints
        dists(i) = norm(x_curr - xCtrl(i,:));
    end
    qCtrl = zeros(numControlPoints,3);
    for i = 1:numControlPoints
        qCtrl(i,:) = IK(xCtrl(i,1),xCtrl(i,2),xCtrl(i,3));
    end
    if phase <= numControlPoints && dists(phase) <= 0.01
        phase = phase + 1;
    end
    if phase <= numControlPoints
        wn = wnParams{phase};
        qControl = qCtrl(phase,:);
    else
        wn = wnParams{end};
        qControl = qDes(end,:);
    end
    A = [zeros(3), eye(3); -diag(wn).^2, -2*diag([1 1 1])*diag(wn)];
    B = [zeros(3); diag(wn).^2];
    Kp = diag([70 70 70]);
    Kd = diag([120 120 120]);
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    [M, C, ~] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    tau  = M*controller + C*qd;
    qdd  = M\(tau - C*qd);
    dxdt = [A*x(1:6) + B*qControl'; qd; qdd];
end

function [x,y,z] = FK(q1,q2,q3)
    l1=0.208; l2=0.168;
    x = sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y = l2-l2*cos(q3)+l1*sin(q2);
    z = -l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end

function Q = IK(x,y,z)
    l1=0.208; l2=0.168;
    q1    = atan2(x,z+l1);
    R     = sqrt(x^2+(z+l1)^2);
    r     = sqrt(x^2+(y-l2)^2+(z+l1)^2);
    Beta  = atan2(y-l2,R);
    Gamma = acos((l1^2+r^2-l2^2)/(2*l1*r));
    q2    = Gamma+Beta;
    Alpha = acos((l1^2+l2^2-r^2)/(2*l1*l2));
    q3    = q2+Alpha-pi/2;
    Q     = [q1,q2,q3];
end

function [M, C, G] = compute_M_C_G(theta1,theta2,theta3,dtheta1,dtheta2,dtheta3)
    l1=0.208; l2=0.168; l3=0.0325; l5=-0.0368; l6=0.0527;
    g=-9.80665;
    m_a=0.0202; Ia_xx=0.4864e-4; Ia_yy=0.001843e-4; Ia_zz=0.4864e-4;
    m_c=0.0249; Ic_xx=0.959e-4;  Ic_yy=0.959e-4;    Ic_zz=0.0051e-4;
    m_be=0.2359; Ibe_xx=11.09e-4; Ibe_yy=10.06e-4;  Ibe_zz=0.591e-4;
    m_df=0.1906; Idf_xx=7.11e-4;  Idf_yy=0.629e-4;  Idf_zz=6.246e-4;
    Ibaseyy=11.87e-4;
    M11=(1/8*(4*Ia_yy+4*Ia_zz+8*Ibaseyy+4*Ibe_yy+4*Ibe_zz+4*Ic_yy+4*Ic_zz+...
         4*Idf_zz+4*l1^2*m_a+l2^2*m_a+l1^2*m_c+4*l3^2*m_c)+...
         1/8*(4*Ibe_yy-4*Ibe_zz+4*Ic_zz+l1^2*(4*m_a+m_c))*cos(2*theta2)+...
         1/8*(4*Ia_yy-4*Ia_zz+4*Idf_yy-4*Idf_zz-l2^2*m_a-4*l3^2*m_c)*cos(2*theta3)+...
         l1*(l2*m_a+l3*m_c)*cos(theta2)*sin(theta3));
    M22=1/4*(4*(Ibe_xx+Ic_xx+l1^2*m_a)+l1^2*m_c);
    M23=-1/2*l1*(l2*m_a+l3*m_c)*sin(theta2-theta3);
    M=[M11 0 0; 0 M22 M23; 0 M23 1/4*(4*Ia_xx+4*Idf_xx+l2^2*m_a+4*l3^2*m_c)];
    C11=1/8*(-2*sin(theta2)*((4*Ibe_yy-4*Ibe_zz+4*Ic_yy-4*Ic_zz+4*l1^2*m_a+l1^2*m_c)*cos(theta2)+...
         2*l1*(l2*m_a+l3*m_c)*sin(theta3)))*dtheta2+...
         2*cos(theta3)*(2*l1*(l2*m_a+l3*m_c)*cos(theta2)+...
         (-4*Ia_yy+4*Ia_zz-4*Idf_yy+4*Idf_zz+l2^2*m_a+4*l3^2*m_c)*sin(theta3))*dtheta3;
    C12=-1/8*((4*Ibe_yy-4*Ibe_zz+4*Ic_yy-4*Ic_zz+l1^2*(4*m_a+m_c))*sin(2*theta2)+...
         4*l1*(l2*m_a+l3*m_c)*sin(theta2)*sin(theta3))*dtheta1;
    C13=-1/8*(-4*l1*(l2*m_a+l3*m_c)*cos(theta2)*cos(theta3)-...
         (-4*Ia_yy+4*Ia_zz-4*Idf_yy+4*Idf_zz+l2^2*m_a+4*l3^2*m_c)*sin(2*theta3))*dtheta1;
    C23=1/2*l1*(l2*m_a+l3*m_c)*cos(theta2-theta3)*dtheta3;
    C32=-1/2*l1*(l2*m_a+l3*m_c)*cos(theta2-theta3)*dtheta2;
    C=[C11 C12 C13; -C12 0 C23; -C13 C32 0];
    N2=1/2*g*(2*l1*m_a+2*l5*m_be+l1*m_c)*cos(theta2);
    N3=1/2*g*(l2*m_a+2*l3*m_c-2*l6*m_df)*sin(theta3);
    G=[0 N2 N3]';
end
