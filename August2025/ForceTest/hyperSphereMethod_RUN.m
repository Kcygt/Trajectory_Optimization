clear; clc;
close all;
dataNumber = 9;

%% ===== CONFIGURATION SECTION =====
% Change these parameters to modify the number of targets and control points

% Define target points (N x 3 matrix where N is number of targets)
xTarget = [  0.01,  -0.03, 0;
             0.04,  -0.03, 0;
              0.07, -0.03, 0;
              0.1,  -0.03, 0;
              0.13, -0.03, 0];

wn1   = [ 2.2173, 3.3822, 1.5977 ];
wn2   = [ 2.5729, 1.8243, 2.2975 ];
wn3   = [ 3.5356, 1.6036, 2.2264 ];
wn4   = [ 5.4805, 5.3693, 5.5996 ];
% p9
xCtrl(1,:) = [ -0.0185, -0.03886, -0.0021 ];
xCtrl(2,:) = [ 0.0560, -0.0380, -0.0010 ];
xCtrl(3,:) = [ 0.1538, -0.0375, 0.0071 ];
% 10
xCtrl(1,:) = [ -0.0185, -0.03886, -0.0021 ];
xCtrl(2,:) = [ 0.0560, -0.0380, -0.0010 ];
xCtrl(3,:) = [ 0.1538, -0.0425, 0.0071 ];
% 11
xCtrl(1,:) = [ -0.0185, -0.03886, -0.0021 ];
xCtrl(2,:) = [ 0.0560, -0.0380, -0.0010 ];
xCtrl(3,:) = [ 0.1538, -0.0465, 0.0071 ];

% 12
xCtrl(1,:) = [ -0.0185, -0.03886, -0.0021 ];
xCtrl(2,:) = [ 0.0560, -0.0380, -0.0010 ];
xCtrl(3,:) = [ 0.1538, -0.0495, 0.0071 ];
% Specify which targets to use as control points (indices)
controlPointIndices = [1,3, 5]; % Use first and last targets as control points

% Final desired configuration
qDes = [0, 0, 0];

% Weights for optimization
wt = [1000, 10, 0.0001]; % [Target, End, Time]

% Initial parameters
tspan = 3.4877;
wn1   = [ 2.5913, 5.7849, 2.9558 ];
wn2   = [ 9.1684, 8.6364, 6.2232 ];
wn3   = [ 8.1015, 9.7757, 8.9940 ];
wn4   = [ 4.3301, 9.6705, 8.4473 ];
xCtrl(1,:) = [ 0.0204, -0.0330, 0.0008 ];
xCtrl(2,:) = [ 0.0550, -0.0296, -0.0006 ];
xCtrl(3,:) = [ 0.0750, -0.0260, 0.0250 ];% Define wn parameters for each phase (one for each control point + one for final phase)
wnValues = [2, 2, 2]; % Base wn values for each phase

% Optimization parameters
tolRad = 0.025;
maxIterations = 60;
numStarts = 5;

%% ===== AUTOMATIC SETUP (Don't modify below this line) =====

% Get dimensions
numTargets = size(xTarget, 1);
numControlPoints = length(controlPointIndices);

% Validate control point indices
if any(controlPointIndices < 1) || any(controlPointIndices > numTargets)
    error('Control point indices must be between 1 and %d', numTargets);
end

% Extract control points from targets
% xCtrl = xTarget(controlPointIndices, :);

% Compute inverse kinematics for control points
qCtrl = zeros(numControlPoints, 3);
for i = 1:numControlPoints
    qCtrl(i,:) = IK(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3));
end

% Build desired configuration
qDes = [qCtrl; qDes];

% Compute final position
[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];

% Build initial parameter vector

initPrms = [tspan, wn1,wn2,wn3,wn4,xCtrl(1,:),xCtrl(2,:),xCtrl(3,:)];

% Simulation
t_uniform = 0:0.001:tspan;
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, extractWnParameters(initPrms, numControlPoints), xCtrl), ...
                      t_uniform, zeros(12, 1));
[CxOpt, CyOpt, CzOpt] = FK(yInit(:,7), yInit(:,8), yInit(:,9));

%% ===== PLOTTING =====
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Optimized Cartesian Trajectory with %d Targets and %d Control Points', numTargets, numControlPoints));

% Trajectory
plot3(CxOpt, CyOpt, CzOpt, '.-', 'LineWidth', 2);

% Target Points
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', 'MarkerSize', 10);

% Final Point
plot3(xFinal(1), xFinal(2), xFinal(3), 'o', 'MarkerSize', 10);

% Control Points
for i = 1:numControlPoints
    plot3(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3), 'd', 'MarkerSize', 10);
end

% Sphere radius and mesh
r = 0.01;
[sx, sy, sz] = sphere(20);
for i = 1:numControlPoints
    surf(xCtrl(i,1)+r*sx, xCtrl(i,2)+r*sy, xCtrl(i,3)+r*sz, ...
         'EdgeColor','none', 'FaceAlpha',0.3, 'FaceColor','b');
end

% Build legend
legendEntries = {'Trajectory', 'Target Points', 'Final Point'};
for i = 1:numControlPoints
    legendEntries{end+1} = sprintf('Control Pt %d', i);
end
legend(legendEntries);






% Save results
% save(sprintf('Sdata%d.mat', dataNumber ), ...
%     'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal','numTargets','numControlPoints');

%% ===== HELPER FUNCTIONS =====

% Extract wn parameters from optimization parameters
function wnParams = extractWnParameters(Opt, numControlPoints)
    wnParams = cell(numControlPoints + 1, 1);
    startIdx = 2; % Start after tspan
    for i = 1:(numControlPoints + 1)
        idx = startIdx + (i-1)*3;
        wnParams{i} = Opt(idx:idx+2);
    end
end

% Extract control points from optimization parameters
function ctrlPoints = extractControlPoints(Opt, numControlPoints)
    ctrlPoints = zeros(numControlPoints, 3);
    startIdx = 2 + (numControlPoints + 1)*3; % Start after tspan and wn parameters
    for i = 1:numControlPoints
        idx = startIdx + (i-1)*3;
        ctrlPoints(i,:) = Opt(idx:idx+2);
    end
end


function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wnParams, xCtrl)
    persistent phase

    % Automatically reset phase at start of new simulation
    if t == 0
        phase = 1;
    elseif isempty(phase)
        phase = 1;
    end

    zeta = [1 1 1];  % Damping ratio

    % Extract joint state
    q  = x(7:9);     % Joint angles
    qd = x(10:12);   % Joint velocities

    % Current Cartesian position
    [x_now, y_now, z_now] = FK(q(1), q(2), q(3));
    x_curr = [x_now, y_now, z_now];

    % Distance to control points
    numControlPoints = size(xCtrl, 1);
    dists = zeros(numControlPoints, 1);
    for i = 1:numControlPoints
        dists(i) = norm(x_curr - xCtrl(i,:));
    end

    % Compute joint-space control targets from Cartesian control points
    qCtrl = zeros(numControlPoints, 3);
    for i = 1:numControlPoints
        qCtrl(i,:) = IK(xCtrl(i,1), xCtrl(i,2), xCtrl(i,3));
    end

    % Phase transition logic
    if phase <= numControlPoints && dists(phase) <= 0.01
        phase = phase + 1;
    end

    % Select controller based on phase
    if phase <= numControlPoints
        wn = wnParams{phase}; % Use wn for current phase
        qControl = qCtrl(phase,:);
    else
        wn = wnParams{end}; % Use final wn for end phase
        qControl = qDes(end,:);
    end

    % Prefilter dynamics
    A = [zeros(3), eye(3); -diag(wn).^2, -2 * diag(zeta) * diag(wn)];
    B = [zeros(3); diag(wn).^2];

    % PD control gains
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  

    % Control law
    controller = Kp * (x(1:3) - q) + Kd * (x(4:6) - qd);

    % Dynamics
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
    tau = M * controller + C * qd;
    qdd = M \ (tau - C * qd);

    % Final state derivative
    dxdt = [A * x(1:6) + B * qControl'; qd; qdd];
end

% Forward Kinematics (FK)
function [x,y,z]=FK(q1,q2,q3)
    l1=0.208; 
    l2=0.168;  
    x=sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y=l2-l2*cos(q3)+l1*sin(q2);
    z=-l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end

% Inverse Kinematics (IK)
function Q=IK(x,y,z)
    l1=0.208; 
    l2=0.168;  
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

% Custom output function to show run progress
function stop = outputFunction(x, optimValues, state, numTargets, numControlPoints)
    persistent runCount iterationCount
    
    if isempty(runCount)
        runCount = 0;
        iterationCount = 0;
    end
    
    stop = false;
    
    switch state
        case 'init'
            runCount = runCount + 1;
            iterationCount = 0;
            fprintf('\n--- Starting Run %d ---\n', runCount);
            fprintf('Initial function value: %.6e\n', optimValues.fval);
            
        case 'iter'
            iterationCount = iterationCount + 1;
            if mod(iterationCount, 5) == 0  % Show every 5th iteration
                fprintf('Run %d, Iter %d: f(x) = %.6e\n', runCount, iterationCount, optimValues.fval);
            end
            
        case 'done'
            fprintf('Run %d completed: Final f(x) = %.6e\n', runCount, optimValues.fval);
            fprintf('--- End Run %d ---\n\n', runCount);
    end
end 