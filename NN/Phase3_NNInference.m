%% Phase3_NNInference.m
% Replaces fmincon entirely at runtime.
% The trained NN predicts optimal parameters instantly from target positions.
%
% USAGE:
%   1. Run Phase1_GenerateTrainingData.m   -> TrainingData.mat
%   2. Run Phase2_TrainNeuralNetwork.m     -> TrainedPolicy.mat
%   3. Run this script                     -> trajectory + plots
%
% You can also call predictOptimalParams() from any other script.

clear; clc; close all;

%% ===== LOAD TRAINED POLICY =====
policyFile = 'TrainedPolicy.mat';
if ~isfile(policyFile)
    error('TrainedPolicy.mat not found. Run Phase2_TrainNeuralNetwork.m first.');
end
load(policyFile);   % net, inputMean/Std, outputMean/Std, inputDim, outputDim, numControlPoints
fprintf('Loaded trained policy (val MSE=%.6f)\n', bestValLoss);

%% ===== DEFINE YOUR TARGET CONFIGURATION =====
controlPointIndices = [1, 3, 5];

xTarget = [ 0.01, -0.03, 0;
            0.04, -0.03, 0;
            0.07, -0.03, 0;
            0.10, -0.03, 0;
            0.13, -0.03, 0];

numTargets = size(xTarget, 1);
qDes_end   = [0, 0, 0];

%% ===== NN INFERENCE (replaces fmincon) =====
fprintf('\nRunning NN inference... ');
tic;
Opt = predictOptimalParams(net, xTarget, inputMean, inputStd, outputMean, outputStd);
elapsed = toc;
fprintf('done in %.4f s\n', elapsed);

%% ===== RECONSTRUCT qDes FROM OPT PARAMS =====
xCtrl  = extractControlPoints(Opt, numControlPoints);
qCtrl  = zeros(numControlPoints, 3);
for i = 1:numControlPoints
    qCtrl(i,:) = IK(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3));
end
qDes = [qCtrl; qDes_end];

[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];

%% ===== SIMULATE WITH NN PARAMETERS =====
fprintf('Simulating trajectory with NN parameters...\n');
tOpt = 0:0.001:Opt(1);
wnParams = extractWnParameters(Opt, numControlPoints);

[tSim, ySim] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, Opt(1), wnParams, xCtrl), ...
                     tOpt, zeros(12,1));

[CxNN, CyNN, CzNN] = FK(ySim(:,7), ySim(:,8), ySim(:,9));

%% ===== DISPLAY PREDICTED PARAMETERS =====
fprintf('\n--- NN Predicted Parameters ---\n');
fprintf('tspan = %.4f s\n', Opt(1));
for i = 1:length(wnParams)
    fprintf('wn%d   = [%.4f, %.4f, %.4f]\n', i, wnParams{i});
end
for i = 1:numControlPoints
    fprintf('ctrlPt%d = [%.4f, %.4f, %.4f]\n', i, xCtrl(i,:));
end

%% ===== COMPUTE TRAJECTORY QUALITY METRICS =====
xTraj = [CxNN, CyNN, CzNN];
distToTargets = zeros(numTargets, 1);
for i = 1:numTargets
    distToTargets(i) = min(sqrt(sum((xTraj - xTarget(i,:)).^2, 2)));
end
distToEnd = norm(xTraj(end,:) - xFinal);

fprintf('\n--- Trajectory Quality ---\n');
for i = 1:numTargets
    fprintf('  Target %d: min distance = %.5f m\n', i, distToTargets(i));
end
fprintf('  End point error: %.5f m\n', distToEnd);
fprintf('  Total time: %.3f s\n', Opt(1));

%% ===== PLOT =====
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('NN-Predicted Trajectory (no fmincon)');

plot3(CxNN, CyNN, CzNN, '.-', 'LineWidth', 2, 'DisplayName','Trajectory');
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'r*', 'MarkerSize',10, 'DisplayName','Targets');
plot3(xFinal(1), xFinal(2), xFinal(3), 'go', 'MarkerSize',10, 'MarkerFaceColor','g', 'DisplayName','Final');

r = 0.01; [sx,sy,sz] = sphere(20);
for i = 1:numControlPoints
    surf(xCtrl(i,1)+r*sx, xCtrl(i,2)+r*sy, xCtrl(i,3)+r*sz, ...
         'EdgeColor','none','FaceAlpha',0.25,'FaceColor','b','HandleVisibility','off');
    plot3(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3), 'bd', 'MarkerSize',10, ...
          'DisplayName', sprintf('CtrlPt %d',i));
end
legend('Location','best');

%% ===== OPTIONAL: COMPARE NN vs FMINCON =====
% Uncomment the block below to run fmincon on the same target set and compare.
%
% fprintf('\nRunning fmincon for comparison...\n');
% tic;
% [OptFmin, ~] = runFmincon(xTarget, numControlPoints, controlPointIndices, qDes_end);
% tFmin = toc;
% fprintf('fmincon took %.2f s\n', tFmin);
%
% tFminSim = 0:0.001:OptFmin(1);
% wnFmin = extractWnParameters(OptFmin, numControlPoints);
% xCtrlFmin = extractControlPoints(OptFmin, numControlPoints);
% qCtrlFmin = zeros(numControlPoints,3);
% for i=1:numControlPoints; qCtrlFmin(i,:)=IK(xCtrlFmin(i,1),xCtrlFmin(i,2),xCtrlFmin(i,3)); end
% qDesFmin = [qCtrlFmin; qDes_end];
% [~,yFmin] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDesFmin,OptFmin(1),wnFmin,xCtrlFmin),...
%                   tFminSim, zeros(12,1));
% [CxF,CyF,CzF] = FK(yFmin(:,7),yFmin(:,8),yFmin(:,9));
% plot3(CxF,CyF,CzF,'--','LineWidth',2,'DisplayName','fmincon');
% legend;
% fprintf('Speed-up: %.1fx\n', tFmin/elapsed);

%% ===== HELPER: CORE INFERENCE FUNCTION =====

function Opt = predictOptimalParams(net, xTarget, inMean, inStd, outMean, outStd)
    % Takes raw target positions, returns denormalised parameter vector.
    inputVec  = xTarget(:)';                         % [1 x inputDim]
    inputNorm = (inputVec - inMean) ./ inStd;        % normalise
    X         = dlarray(inputNorm', 'CB');            % [inputDim x 1]
    YNorm     = extractdata(predict(net, X))';        % [1 x outputDim]
    Opt       = (YNorm .* outStd) + outMean;          % denormalise
end

%% ===== ROBOT FUNCTIONS (must match Phase 1 exactly) =====

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
    q  = x(7:9); qd = x(10:12);
    [x_now, y_now, z_now] = FK(q(1),q(2),q(3));
    x_curr = [x_now, y_now, z_now];
    numCP  = size(xCtrl,1);
    dists  = zeros(numCP,1);
    for i = 1:numCP, dists(i) = norm(x_curr - xCtrl(i,:)); end
    qCtrl  = zeros(numCP,3);
    for i = 1:numCP, qCtrl(i,:) = IK(xCtrl(i,1),xCtrl(i,2),xCtrl(i,3)); end
    if phase <= numCP && dists(phase) <= 0.01, phase = phase + 1; end
    if phase <= numCP
        wn = wnParams{phase}; qControl = qCtrl(phase,:);
    else
        wn = wnParams{end};   qControl = qDes(end,:);
    end
    A = [zeros(3),eye(3); -diag(wn).^2, -2*diag([1 1 1])*diag(wn)];
    B = [zeros(3); diag(wn).^2];
    Kp = diag([70 70 70]); Kd = diag([120 120 120]);
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    [M,C,~] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    tau = M*controller + C*qd;
    qdd = M\(tau - C*qd);
    dxdt = [A*x(1:6) + B*qControl'; qd; qdd];
end

function [x,y,z] = FK(q1,q2,q3)
    l1=0.208; l2=0.168;
    x=sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y=l2-l2*cos(q3)+l1*sin(q2);
    z=-l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end

function Q = IK(x,y,z)
    l1=0.208; l2=0.168;
    q1=atan2(x,z+l1);
    R=sqrt(x^2+(z+l1)^2); r=sqrt(x^2+(y-l2)^2+(z+l1)^2);
    Beta=atan2(y-l2,R); Gamma=acos((l1^2+r^2-l2^2)/(2*l1*r));
    q2=Gamma+Beta; Alpha=acos((l1^2+l2^2-r^2)/(2*l1*l2));
    q3=q2+Alpha-pi/2; Q=[q1,q2,q3];
end

function [M,C,G] = compute_M_C_G(theta1,theta2,theta3,dtheta1,dtheta2,dtheta3)
    l1=0.208; l2=0.168; l3=0.0325; l5=-0.0368; l6=0.0527; g=-9.80665;
    m_a=0.0202; Ia_xx=0.4864e-4; Ia_yy=0.001843e-4; Ia_zz=0.4864e-4;
    m_c=0.0249; Ic_xx=0.959e-4; Ic_yy=0.959e-4; Ic_zz=0.0051e-4;
    m_be=0.2359; Ibe_xx=11.09e-4; Ibe_yy=10.06e-4; Ibe_zz=0.591e-4;
    m_df=0.1906; Idf_xx=7.11e-4; Idf_yy=0.629e-4; Idf_zz=6.246e-4;
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
