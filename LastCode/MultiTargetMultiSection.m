clear; clc;
close all;

%% ===== CONFIGURATION SECTION =====
% ONLY CHANGE THESE TWO PARAMETERS - everything else is automatic!

% Define target points (N x 3 matrix where N is number of targets)
% xTarget(1,:) = [0.005, 0.02, 0.01];
xTarget(1,:) = [0.040, 0.035, 0.01];
% xTarget(3,:) = [0.035, 0.03, 0.04];
% Add or remove targets as needed:
% xTarget = zeros(3,3);
% xTarget(1,:) = [0.005, 0.02, 0.01];
% xTarget(2,:) = [0.02, 0.025, 0.025];
% xTarget(3,:) = [0.035, 0.03, 0.04];

% Define number of phases (this determines tspan and wn)
numPhases =1;  % Change this to 2, 3, 4, 5, etc.

%% ===== AUTOMATIC SETUP (Don't modify below this line) =====

% Get number of targets
numTargets = size(xTarget, 1);

% Define desired final configuration
qDes = [0.191425481525343, 0.190854007095390, 0.356154760943820];

% Weights for optimization
wt = [500, 1, 0.001]; % [Target, End, Time]

% Base parameters (will be automatically adjusted)
baseTspan = [0.4154, 0.9925, 1.5251];
baseWn = [1.3195, 1.6735, 1.8446, 2.86879, 3.9139, 4.00, 16.6248, 0.1000, 0.6028];

% Automatically generate tspan and wn based on numPhases
if numPhases <= length(baseTspan)
    % Use existing base parameters if we have enough
    tspan = baseTspan(1:numPhases);
    wn = baseWn(1:(3*numPhases));
else
    % Extend parameters if we need more phases
    tspan = [baseTspan, baseTspan(end) + (1:(numPhases-length(baseTspan)))];
    additionalWn = repmat([2.0, 2.0, 2.0], 1, numPhases - length(baseWn)/3);
    wn = [baseWn, additionalWn];
end

fprintf('=== Automatic Configuration ===\n');
fprintf('Number of targets: %d\n', numTargets);
fprintf('Number of phases: %d\n', numPhases);
fprintf('Number of switching times: %d\n', length(tspan));
fprintf('Number of wn parameters: %d\n', length(wn));
fprintf('Total optimization parameters: %d\n\n', length(tspan) + length(wn));

% Compute final position
[Px, Py, Pz] = FK(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

% Build initial parameter vector
initPrms = [tspan, wn];

% Initial simulation
tUni = 0:0.01:tspan(end);
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn), tUni, zeros(12, 1));

% Extract initial trajectory
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9));

% Build bounds
lb = zeros(1, length(initPrms));
ub = zeros(1, length(initPrms));

% Time bounds (switching times)
lb(1:length(tspan)) = 0;
ub(1:length(tspan)) = 3;

% Wn bounds
startIdx = length(tspan) + 1;
for i = 1:numPhases
    idx = startIdx + (i-1)*3;
    lb(idx:idx+2) = 0.1;
    ub(idx:idx+2) = 15;
end

% Optimization
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal, numTargets);

options = optimoptions('fmincon', ...
    'Display', 'none', ...
    'TolCon', 1e-4, ...
    'OptimalityTolerance', 1e-4, ...
    'StepTolerance', 1e-4, ...
    'MaxIterations', 60);

problem = createOptimProblem('fmincon', ...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xTarget, xFinal, numTargets));

% MultiStart setup
ms = MultiStart('UseParallel', false, 'Display', 'iter');
numStarts = 5;

fprintf('=== Starting MultiStart Optimization ===\n');
fprintf('Number of targets: %d\n', numTargets);
fprintf('Number of phases: %d\n', numPhases);
fprintf('Number of switching times: %d\n', length(tspan));
fprintf('Number of wn parameters: %d\n', length(wn));
fprintf('Number of optimization runs: %d\n\n', numStarts);

[Opt, fval] = run(ms, problem, numStarts);

% Extract optimal parameters
optimalTimes = Opt(1:length(tspan));
optimalWn = Opt(length(tspan)+1:end);

% Simulate with optimal parameters
tUni = 0:0.01:optimalTimes(end);
[tOpt, yOpt] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, optimalTimes, optimalWn), ...
                     tUni, zeros(12, 1));

% Extract optimized trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9));

%% ===== PLOTTING =====

% Plot segmented trajectory
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Optimized Cartesian Trajectory with %d Targets, %d Phases', numTargets, numPhases));

% Find switching time indices
switchIndices = zeros(1, length(optimalTimes));
for i = 1:length(optimalTimes)
    [~, switchIndices(i)] = min(abs(tUni - optimalTimes(i)));
end

% Plot trajectory segments in different colors
colors = {'r', 'g', 'b', 'm', 'c', 'y'};
for i = 1:length(switchIndices)
    if i == 1
        startIdx = 1;
    else
        startIdx = switchIndices(i-1) + 1;
    end
    endIdx = switchIndices(i);
    
    colorIdx = mod(i-1, length(colors)) + 1;
    plot3(CxOpt(startIdx:endIdx), CyOpt(startIdx:endIdx), CzOpt(startIdx:endIdx), ...
          'Color', colors{colorIdx}, 'LineWidth', 2);
end

% Plot final segment
if length(switchIndices) > 0
    plot3(CxOpt(switchIndices(end)+1:end), CyOpt(switchIndices(end)+1:end), CzOpt(switchIndices(end)+1:end), ...
          'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
end

% Plot targets and final point
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', 'MarkerSize', 10, 'Color', [0,0.4,0.8]);
plot3(xFinal(1), xFinal(2), xFinal(3), 'p', 'MarkerSize', 14, 'Color', [1,0.5,0.05]);

% Build legend
legendEntries = {};
for i = 1:length(switchIndices)
    legendEntries{end+1} = sprintf('Phase %d', i);
end
legendEntries{end+1} = 'Final Phase';
legendEntries{end+1} = 'Target Points';
legendEntries{end+1} = 'Final Point';
legend(legendEntries);

% Display optimized parameters
fprintf('\nOptimized Parameters:\n');
fprintf('Switching Times:\n');
for i = 1:length(optimalTimes)
    fprintf('  t%d = %.4f\n', i, optimalTimes(i));
end

fprintf('\nWn Parameters:\n');
for i = 1:numPhases
    idx = (i-1)*3 + 1;
    fprintf('  wn%d = [%.4f, %.4f, %.4f]\n', i, optimalWn(idx:idx+2));
end

% Save results
save(sprintf('flexible_test5_data_%d_targets_%d_phases.mat', numTargets, numPhases), ...
    'Opt', 'tOpt', 'yOpt', 'tInit', 'yInit', 'xTarget', 'xFinal', 'numTargets', 'numPhases', 'optimalTimes', 'optimalWn');

%% ===== HELPER FUNCTIONS =====

% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal, numTargets)
    % Extract parameters dynamically
    numPhases = length(prms) / 4; % Total parameters / 4 (3 wn per phase + 1 time per phase)
    numTimes = numPhases;
    tspan = prms(1:numTimes);
    wn = prms(numTimes+1:end);
    
    x0 = zeros(12, 1);
    x0(1:3) = qDes;
    tUni = 0:0.01:tspan(end);
    
    % Simulate the system
    [~, y] = ode45(@(t,x) myTwolinkwithprefilter(t,x,qDes,tspan,wn), tUni, x0);
    
    [x,y,z] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x,y,z];
    
    % Calculate minimum distance to each target
    distMid = 0;
    for i = 1:size(xTarget,1)
        distMid = distMid + min(sqrt(sum((xOut - xTarget(i,:)).^2,2)));
    end
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - xFinal).^2,2)));
    
    % Time penalty
    timePenalty = tspan(end);
    
    % Composite error
    error = wt(1)*distMid + wt(2)*distEndErr + wt(3)*timePenalty;
end

% Constraint Function
function [c, ceq] = trajConstraint(prms, qDes, xTarget, xFinal, numTargets)
    ceq = [];
    % Extract parameters dynamically
    numPhases = length(prms) / 4; % Total parameters / 4
    numTimes = numPhases;
    tspan = prms(1:numTimes);
    wn = prms(numTimes+1:end);
    
    tUni = 0:0.01:tspan(end);
    
    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,tspan,wn), tUni, zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));
    xOut = [x,y,z];
    
    % Calculate distances to each target
    dists = zeros(size(xTarget,1), 1);
    for i = 1:size(xTarget,1)
        dists(i) = min(sqrt(sum((xOut - xTarget(i,:)).^2,2)));
    end
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - xFinal).^2,2)));
    
    % Nonlinear inequality constraints
    c = [];
    
    % Target proximity constraints
    for i = 1:length(dists)
        c(end+1) = dists(i) - 1e-10;
    end
    
    % End point constraint
    c(end+1) = distEndErr - 1e-10;
    
    % Time ordering constraints
    for i = 1:length(tspan)-1
        c(end+1) = tspan(i) - tspan(i+1);
    end
end

% Dynamics Function with Prefilter
function dxdt = myTwolinkwithprefilter(t, x, qDes, t_st, wn)
    zeta = [1 1 1];
    
    % Determine current phase based on time
    phase = 1;
    for i = 1:length(t_st)
        if t > t_st(i)
            phase = i + 1;
        end
    end
    
    % Select wn parameters for current phase
    wnPhase = wn((phase-1)*3+1:phase*3);
    
    A = [zeros(3), eye(3); -diag(wnPhase).^2, -2*diag(zeta)*diag(wnPhase)];
    B = [zeros(3); diag(wnPhase).^2];
    
    q = x(7:9);
    qd = x(10:12);
    
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  
    
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    
    [M,C,G] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau = M*(controller) + C*qd;
    qdd = M\(tau-C*qd);
    
    dxdt = [A*x(1:6) + B*qDes(:); qd; qdd];
end

% Forward Kinematics (FK)
function [x,y,z] = FK(q1,q2,q3)
    l1 = 0.208; 
    l2 = 0.168;  
    x = sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y = l2-l2*cos(q3)+l1*sin(q2);
    z = -l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end

% Inverse Kinematics (IK)
function Q = IK(x,y,z)
    l1 = 0.208; 
    l2 = 0.168;  
    q1 = atan2(x,z+l1);
    
    R = sqrt(x^2+(z+l1)^2);
    r = sqrt(x^2+(y-l2)^2+(z+l1)^2);
    
    Beta = atan2(y-l2,R);
    Gamma = acos((l1^2+r^2-l2^2)/(2*l1*r));
    
    q2 = Gamma+Beta;
    
    Alpha = acos((l1^2+l2^2-r^2)/(2*l1*l2));
    
    q3 = q2+Alpha-pi/2;
    
    Q = [q1,q2,q3];
end

function [M, C, G] = compute_M_C_G(theta1, theta2,theta3, dtheta1, dtheta2,dtheta3)
    % link lenghts
    l1 = 0.208;   l2 = 0.168;   l3 = 0.0325;
    l5 = -0.0368; l6 = 0.0527;
    
    % Gravity
    g = -9.80665; % m/s^2
       
    % segment A
    m_a = 0.0202;
    Ia_xx = 0.4864*1e-4;  Ia_yy = 0.001843*1e-4;  Ia_zz = 0.4864*1e-4;
    Ia = [Ia_xx 0 0;  0  Ia_yy 0;  0 0 Ia_zz];
    
    % segment C
    m_c = 0.0249;
    Ic_xx = 0.959*1e-4;  Ic_yy = 0.959*1e-4;  Ic_zz = 0.0051*1e-4;
    Ic = [Ic_xx 0 0;  0  Ic_yy 0;  0 0 Ic_zz];
    
    % segment BE
    m_be = 0.2359;
    Ibe_xx = 11.09*1e-4;  Ibe_yy = 10.06*1e-4;  Ibe_zz = 0.591*1e-4;
    Ibe = [Ibe_xx 0 0;  0  Ibe_yy 0;  0 0 Ibe_zz];
    
    % segment DF
    m_df = 0.1906;
    Idf_xx = 7.11*1e-4;  Idf_yy = 0.629*1e-4;  Idf_zz = 6.246*1e-4;
    Idf = [Idf_xx 0 0;  0  Idf_yy 0;  0 0 Idf_zz];
    
    % BASE
    Ibaseyy = 11.87e-4;
    
    % MASS 
    M11 = ( 1/8*( 4*Ia_yy  + 4*Ia_zz  + 8*Ibaseyy + 4*Ibe_yy + 4*Ibe_zz + 4*Ic_yy + 4*Ic_zz + 4*Idf_zz + 4*l1^2*m_a + l2^2*m_a + l1^2*m_c + 4*l3^2*m_c  ) + ...
            1/8*( 4*Ibe_yy - 4*Ibe_zz + 4*Ic_zz   + l1^2*(4*m_a + m_c)) * cos(2*theta2) + ...
            1/8*( 4*Ia_yy  - 4*Ia_zz  + 4*Idf_yy  - 4*Idf_zz - l2^2*m_a - 4*l3^2*m_c) * cos(2*theta3) + l1*(l2*m_a + l3*m_c)*cos(theta2)*sin(theta3)  );
    
    M22 = 1/4*(4*(Ibe_xx + Ic_xx + l1^2*m_a) + l1^2*m_c);
    M23 = -1/2*l1*(l2*m_a + l3*m_c) * sin(theta2-theta3);
    M32 = M23;
    M33 = 1/4 * (4*Ia_xx + 4*Idf_xx + l2^2*m_a + 4*l3^2*m_c);
    
    M = [M11 0 0; 0 M22 M23; 0 M32 M33];
    
    % CORIOLIS
    C11 = 1/8*( -2*sin(theta2) * ((4*Ibe_yy - 4*Ibe_zz * 4*Ic_yy - 4*Ic_zz + 4*l1^2*m_a +l1^2*m_c )*cos(theta2) + 2*l1*(l2*m_a + l3*m_c)*sin(theta3) ) )*dtheta2 + ...
           2*cos(theta3)*(2*l1*(l2*m_a + l3*m_c)*cos(theta2) + (-4*Ia_yy + 4*Ia_zz -4*Idf_yy + 4*Idf_zz + l2^2*m_a + 4*l3^2*m_c)*sin(theta3)) * dtheta3 ;
    
    C12 = -1/8 * ((4*Ibe_yy - 4*Ibe_zz + 4*Ic_yy - 4*Ic_zz + l1^2*(4*m_a + m_c))*sin(2*theta2) + 4*l1*(l2*m_a+l3*m_c)*sin(theta2)*sin(theta3))*dtheta1; 
    C13 = -1/8*(-4*l1*(l2*m_a + l3*m_c)*cos(theta2)*cos(theta3) - (-4*Ia_yy + 4*Ia_zz - 4*Idf_yy + 4*Idf_zz + l2^2*m_a + 4*l3*m_c)*sin(2*theta3))*dtheta1;
    C21 = -C12;
    C23 = 1/2*l1*(l2*m_a + l3*m_c)*cos(theta2*theta3)*dtheta3;
    C31 = -C13;
    C32  = -1/2*l1*(l2*m_a + l3*m_c)*cos(theta2 - theta3)*dtheta2;
    
    C = [C11 C12 C13; C21 0 C23; C31 C32 0];
    
    % GRAVITY
    
    N2 = 1/2*g*(2*l1*m_a + 2*l5*m_be + l1*m_c)*cos(theta2);
    N3 = 1/2*g*(l2*m_a + 2*l3*m_c - 2*l6*m_df)*sin(theta3);
    
    G = [0 N2 N3]';
end 