clear; clc;
close all;

%% ===== DIRECT TRAJECTORY PLANNING FOR STRAIGHT-LINE MOTION =====

% Define target points in a straight line
xTarget = [...
    -0.02, -0.03, 0.0;
    0.0,   -0.03, 0.0;
    0.02,  -0.03, 0.0];

% Final desired configuration
qDes = [0, 0, 0];

% Weights for optimization
wt = [1000, 10, 0.0001, 1000]; % [Target, End, Time, Smoothness]

% Initial parameters
tspan = 5;
% Define waypoints for direct trajectory planning
numWaypoints = 20;
initWaypoints = zeros(numWaypoints, 3);

% Initialize waypoints along straight line
for i = 1:numWaypoints
    t = (i-1)/(numWaypoints-1);
    initWaypoints(i,:) = xTarget(1,:) + t * (xTarget(end,:) - xTarget(1,:));
end

% Build initial parameter vector: [tspan, waypoints]
initPrms = [tspan, reshape(initWaypoints, 1, [])];

% Simulation
t_uniform = 0:0.001:tspan;
[tInit, yInit] = ode45(@(t, x) directTrajectoryController(t, x, qDes, tspan, initWaypoints), ...
                      t_uniform, zeros(12, 1));
[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];

% Bounds
lb = [0]; % Time
ub = [6]; % Time

% Add bounds for waypoints (allow small deviation from straight line)
for i = 1:numWaypoints
    idealPoint = xTarget(1,:) + (i-1)/(numWaypoints-1) * (xTarget(end,:) - xTarget(1,:));
    lb = [lb, idealPoint - 0.005]; % Small deviation allowed
    ub = [ub, idealPoint + 0.005];
end

% Optimization
objectiveFunc = @(params) directObjectiveFunction(params, qDes, wt, xTarget, xFinal);
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
    'TolCon', 1e-6, 'OptimalityTolerance', 1e-7, 'StepTolerance', 1e-3, 'MaxIterations', 100);
problem = createOptimProblem('fmincon', ...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) directTrajConstraint(prms, qDes, xTarget, xFinal));
ms = MultiStart('UseParallel', true, 'Display', 'iter');
[Opt, fval] = run(ms, problem, 10);

% Extract optimized waypoints
tspan_opt = Opt(1);
waypoints_opt = reshape(Opt(2:end), numWaypoints, 3);

% Simulate with optimal parameters
tOpt = 0:0.001:tspan_opt;
[tOpt, yOpt] = ode45(@(t, x) directTrajectoryController(t, x, qDes, tspan_opt, waypoints_opt), ...
                     tOpt, zeros(12, 1));
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9));

%% ===== PLOTTING =====
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Direct Trajectory Planning - Straight Line Motion');

% Trajectory
plot3(CxOpt, CyOpt, CzOpt, '.-', 'LineWidth', 2);

% Target Points
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', 'MarkerSize', 10);

% Final Point
plot3(xFinal(1), xFinal(2), xFinal(3), 'o', 'MarkerSize', 10);

% Waypoints
plot3(waypoints_opt(:,1), waypoints_opt(:,2), waypoints_opt(:,3), 'd', 'MarkerSize', 5);

% Draw ideal straight line
plot3([xTarget(1,1), xTarget(end,1)], [xTarget(1,2), xTarget(end,2)], [xTarget(1,3), xTarget(end,3)], '--r', 'LineWidth', 1);

legend('Trajectory', 'Target Points', 'Final Point', 'Waypoints', 'Ideal Straight Line');

% Display optimized parameters
fprintf('Optimized Parameters:\n');
fprintf('tspan = %.4f;\n', tspan_opt);

% Calculate trajectory straightness
trajectoryPoints = [CxOpt, CyOpt, CzOpt];
straightnessError = calculateStraightnessError(trajectoryPoints, xTarget(1,:), xTarget(end,:));
fprintf('Trajectory straightness error: %.6f\n', straightnessError);

save('directTrajectory_data.mat', 'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal','waypoints_opt');

%% ===== DIRECT TRAJECTORY FUNCTIONS =====

% Direct trajectory controller without prefilter
function dxdt = directTrajectoryController(t, x, qDes, tspan, waypoints)
    % Extract joint state
    q  = x(7:9);     % Joint angles
    qd = x(10:12);   % Joint velocities

    % Current Cartesian position
    [x_now, y_now, z_now] = FK(q(1), q(2), q(3));
    x_curr = [x_now, y_now, z_now];

    % Interpolate desired position from waypoints
    t_normalized = t / tspan;
    desired_pos = interpolateWaypoints(waypoints, t_normalized);
    
    % Convert to joint space
    q_desired = IK(desired_pos(1), desired_pos(2), desired_pos(3));

    % Direct PD control (no prefilter)
    Kp = diag([150 150 150]);  % High proportional gain for direct motion
    Kd = diag([200 200 200]);  % High derivative gain for stability

    % Control law
    controller = Kp * (q_desired' - q) + Kd * (-qd);

    % Dynamics
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
    tau = M * controller + C * qd + G;
    qdd = M \ (tau - C * qd - G);

    % State derivative (no prefilter states)
    dxdt = [qd; qdd];
end

% Interpolate between waypoints
function pos = interpolateWaypoints(waypoints, t)
    numWaypoints = size(waypoints, 1);
    
    if t <= 0
        pos = waypoints(1, :);
    elseif t >= 1
        pos = waypoints(end, :);
    else
        % Linear interpolation between waypoints
        idx = t * (numWaypoints - 1) + 1;
        idx_low = floor(idx);
        idx_high = min(idx_low + 1, numWaypoints);
        alpha = idx - idx_low;
        
        pos = (1 - alpha) * waypoints(idx_low, :) + alpha * waypoints(idx_high, :);
    end
end

% Direct objective function
function error = directObjectiveFunction(prms, qDes, wt, xTarget, xFinal)
    tspan = prms(1);
    waypoints = reshape(prms(2:end), [], 3);
    
    tUni = 0:0.001:tspan;
    [~, y] = ode45(@(t,x) directTrajectoryController(t, x, qDes, tspan, waypoints), ...
                   tUni, zeros(12, 1));
    [x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x0,y0,z0];
    
    % Target distance error
    distMidF = 0;
    for i = 1:size(xTarget,1)
        distMidF = distMidF + min(sum((xOut - xTarget(i,:)).^2, 2));
    end
    
    % End point error
    distEndErr = sum((xOut(end,:) - xFinal).^2,2);
    
    % Time penalty
    timePenalty = tspan;
    
    % Smoothness penalty (minimize acceleration)
    smoothnessPenalty = 0;
    for i = 2:length(tUni)-1
        dt = tUni(i+1) - tUni(i);
        accel = (xOut(i+1,:) - 2*xOut(i,:) + xOut(i-1,:)) / (dt^2);
        smoothnessPenalty = smoothnessPenalty + sum(accel.^2);
    end
    
    error = wt(1)*distMidF + wt(2)*distEndErr + wt(3)*timePenalty + wt(4)*smoothnessPenalty;
end

% Direct constraint function
function [c, ceq] = directTrajConstraint(prms, qDes, xTarget, xFinal)
    ceq = [];
    tspan = prms(1);
    waypoints = reshape(prms(2:end), [], 3);
    
    tUni = 0:0.001:tspan;
    [~, yy] = ode45(@(t,x) directTrajectoryController(t, x, qDes, tspan, waypoints), ...
                    tUni, zeros(12, 1));
    [x0,y0,z0] = FK(yy(:,7),yy(:,8),yy(:,9));
    x = [x0,y0,z0];
    
    idxs = zeros(size(xTarget,1),1);
    for i = 1:size(xTarget,1)
        [~, idxs(i)] = min(sum((x - xTarget(i,:)).^2, 2));
    end
    
    c = [];
    % Ensure targets are visited in order
    for i = 1:length(idxs)-1
        c(end+1) = idxs(i) - idxs(i+1);
    end
    
    % Ensure all targets are visited
    for i = 1:size(xTarget,1)
        c(end+1) = min(sum((x - xTarget(i,:)).^2, 2)) - 1e-10;
    end
    
    % Ensure final position is reached
    c(end+1) = sum((x(end,:) - xFinal).^2) - 1e-10;
    
    % Add straight-line constraint
    straightLineError = calculateStraightnessError(x, xTarget(1,:), xTarget(end,:));
    c(end+1) = straightLineError - 0.005; % Maximum deviation from straight line
end

% Function to calculate straightness error
function error = calculateStraightnessError(trajectoryPoints, startPoint, endPoint)
    % Calculate the ideal straight line vector
    lineVector = endPoint - startPoint;
    lineLength = norm(lineVector);
    
    if lineLength == 0
        error = 0;
        return;
    end
    
    % Normalize the line vector
    lineUnitVector = lineVector / lineLength;
    
    % Calculate perpendicular distance from each trajectory point to the line
    totalError = 0;
    for i = 1:size(trajectoryPoints, 1)
        % Vector from start point to current trajectory point
        pointVector = trajectoryPoints(i,:) - startPoint;
        
        % Projection onto the line
        projection = dot(pointVector, lineUnitVector);
        
        % Closest point on the line
        closestPoint = startPoint + projection * lineUnitVector;
        
        % Perpendicular distance
        distance = norm(trajectoryPoints(i,:) - closestPoint);
        totalError = totalError + distance^2;
    end
    
    error = totalError / size(trajectoryPoints, 1);
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