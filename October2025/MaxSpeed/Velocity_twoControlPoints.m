clear; clc; close all;
dataNumber = 8;
%%%%% Configuration - Choose control point indices %%%%%
indexControlPoint = [1, 3];   % choose directly from xTarget rows

%%%%% Optimal Parameters %%%%%
qDes = [ 0   0.198678167676855   0.327814256075948 ];
qDes = [0, 0, 0];
[x, y, z] = FK(qDes(1), qDes(2), qDes(3));
xDes = [x, y, z];

% Example target points (can be larger)
% Case 6
% xTarget = [ 0,     0.02,  0.005;
%             0,     0.03,  0.03;
%             0,     0.04, 0.045];

% % Case 7
% xTarget = [
%     0,     0.02,  0.005;
%     0,     0.03,  0.015;
%     0,     0.04, 0.03];
xTarget = [ -0.03, -0.05, 0.02;
            0.03, -0.08,  0.02;
            0.025, -0.025,  0.02 ];
% Number of control points is based on chosen indices
numControlPoints = numel(indexControlPoint);

% Parameters
tspan = 20;

% Initialize wn parameters for each control point + final point
numWnSets = numControlPoints + 1;
wn = cell(numWnSets, 1);
for i = 1:numWnSets
    wn{i} = [1 1 1];  % Default values
end

% Select control points based on chosen indices
ctrlPoints = xTarget(indexControlPoint, :);

% Convert control points to joint angles
qCtrl = zeros(numControlPoints, 3);
for i = 1:numControlPoints
    qCtrl(i,:) = IK(ctrlPoints(i,1), ctrlPoints(i,2), ctrlPoints(i,3));
end

% Combine all desired joint angles (control points + final)
qDes = [qCtrl; qDes];

% Weights
wt = [1800, 20, 0.001];   % [Target, End, Time]

% Create initial parameters vector
initPrms = [tspan];
for i = 1:numWnSets
    initPrms = [initPrms, wn{i}];
end
for i = 1:numControlPoints
    initPrms = [initPrms, ctrlPoints(i,:)];
end

t_uniform = 0:0.01:tspan;

% Initial Condition
[tInit, yInit] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn, ctrlPoints), ...
                       t_uniform, zeros(12, 1));

% Bounds for optimization
tb = [.15 0.15 0.15];

% Lower and Upper Limits
lb = [0];  % Time
ub = [7];  % Time

% Add bounds for wn parameters
for i = 1:numWnSets
    lb = [lb, 0.8 0.8 0.8];
    ub = [ub, 4 4 4];
end

% Add bounds for control points
for i = 1:numControlPoints
    lb = [lb, ctrlPoints(i,:) - tb];
    ub = [ub, ctrlPoints(i,:) + tb];
end

% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xDes, numControlPoints);

% Run optimization
options = optimoptions('fmincon', 'Display', 'off', ... 
                        'TolCon', 1e-10,'Maxiterations',50);

% Create optimization problem
problem = createOptimProblem('fmincon',...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xTarget, numControlPoints));

% MultiStart setup
ms = MultiStart('UseParallel', true, 'Display', 'iter');
numStarts = 5;

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);
tOpt = 0:0.01:Opt(1);

% Extract optimized parameters
tspan_opt = Opt(1);
wn_opt = cell(numWnSets, 1);
ctrlPoints_opt = zeros(numControlPoints, 3);

paramIdx = 2;
for i = 1:numWnSets
    wn_opt{i} = Opt(paramIdx:paramIdx+2);
    paramIdx = paramIdx + 3;
end
for i = 1:numControlPoints
    ctrlPoints_opt(i,:) = Opt(paramIdx:paramIdx+2);
    paramIdx = paramIdx + 3;
end

% Simulate with optimal parameters
[tOpt, yOpt] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan_opt, wn_opt, ctrlPoints_opt), ...
                  tOpt, zeros(12, 1));

% Calculate switching times
t_Vmax = cell(numWnSets-1, 1);
for i = 1:numWnSets-1
    t_Vmax{i} = 1./wn_opt{i};
end

% Forward Kinematics
[xi, yi, zi] = FK(yInit(:,7), yInit(:,8), yInit(:,9));     % Initial Trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9)); % Optimized Trajectory
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3)); % Optimized Trajectory

%% ------------ 3D Cartesian plotting ------------
% Colors
col_init = [0.70 0.70 0.70];  % gray (initial)
col_opt  = [0.20 0.60 1.00];  % blue (optimized)
col_tgt  = [1.00 0.55 0.10];  % orange (targets)
col_ctrl = [0.10 0.70 0.10];  % green (control points)
col_end  = [0.85 0.10 0.10];  % red (end)
col_start= [0.10 0.70 0.70];  % teal (start)

% Start/end in Cartesian
startXYZ = [xi(1), yi(1), zi(1)];
endXYZ   = xDes;  % your chosen final Cartesian point

% 3D plot
figure('Name','Cartesian Space Trajectory (3D)'); hold on; grid on; axis equal
view(45,25);

% Initial trajectory (3D)
plot3(xi, yi, zi, '--', 'LineWidth', 1.5, 'Color', col_init, 'DisplayName','Initial Trajectory');

% Optimized trajectory (3D)
plot3(CxOpt, CyOpt, CzOpt, '-',  'LineWidth', 2.0, 'Color', col_opt,  'DisplayName','Optimized Trajectory');

% Targets
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', col_tgt, 'MarkerEdgeColor','k', 'DisplayName','Target Points');

% Optimized control points (in Cartesian as given/optimized)
for i = 1:numControlPoints
    plot3(ctrlPoints_opt(i,1), ctrlPoints_opt(i,2), ctrlPoints_opt(i,3), ...
        'd', 'MarkerSize', 9, 'MarkerFaceColor', col_ctrl, 'MarkerEdgeColor','k', ...
        'DisplayName', sprintf('Control Point %d', i));
end

% Start & End markers
plot3(startXYZ(1), startXYZ(2), startXYZ(3), 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', col_start, 'MarkerEdgeColor','k', 'DisplayName','Start');
plot3(endXYZ(1),   endXYZ(2),   endXYZ(3),   's', 'MarkerSize', 10, ...
    'MarkerFaceColor', col_end,   'MarkerEdgeColor','k', 'DisplayName','End');

% Optional: connect control points in order to visualize waypoints chain
if numControlPoints >= 2
    plot3(ctrlPoints_opt(:,1), ctrlPoints_opt(:,2), ctrlPoints_opt(:,3), ...
        ':', 'LineWidth', 1.0, 'Color', [0.3 0.8 0.3], 'HandleVisibility','off');
end

% Axes labeling
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('Cartesian Space Trajectory Results (data %d)', dataNumber));
legend('Location','bestoutside');

% Nice bounds with small pad
allXYZ = [ [xi yi zi]; [CxOpt CyOpt CzOpt]; xTarget; ctrlPoints_opt; startXYZ; endXYZ ];
pad = 0.03 * max(range(allXYZ));
xlim([min(allXYZ(:,1))-pad, max(allXYZ(:,1))+pad]);
ylim([min(allXYZ(:,2))-pad, max(allXYZ(:,2))+pad]);
zlim([min(allXYZ(:,3))-pad, max(allXYZ(:,3))+pad]);

% Optional: translucent reference planes (comment out if not needed)
drawPlane = false;
if drawPlane
    xlim_ = xlim; ylim_ = ylim; zlim_ = zlim;
    % XY @ zmin
    [Xp, Yp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(ylim_(1),ylim_(2),2));
    Zp = zlim_(1)*ones(size(Xp));
    surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[0.6 0.6 1.0], 'HandleVisibility','off');
    % XZ @ y = ymin + 0.1
    [Xp, Zp] = meshgrid(linspace(xlim_(1),xlim_(2),2), linspace(zlim_(1),zlim_(2),2));
    Yp = (ylim_(1)+0.1)*ones(size(Xp));
    surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[0.6 1.0 0.6], 'HandleVisibility','off');
    % YZ @ xmin
    [Yp, Zp] = meshgrid(linspace(ylim_(1),ylim_(2),2), linspace(zlim_(1),zlim_(2),2));
    Xp = xlim_(1)*ones(size(Yp));
    surf(Xp, Yp, Zp, 'FaceAlpha',0.07, 'EdgeColor','none', 'FaceColor',[1.0 0.6 0.6], 'HandleVisibility','off');
end

% Print optimized parameter vector if you like
disp(['Optimal Parameter: ', mat2str(Opt,4)]);


%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%

% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xDes, numControlPoints)
    
    x0 = zeros(12, 1);
    
    % Extract parameters
    T_total = prms(1);
    t_uniform = 0:0.01:T_total;
    
    % Extract wn and control points from parameters
    [wn, ctrlPoints] = extractParameters(prms, numControlPoints);
    
    % Simulate the system
    [tt, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1), wn, ctrlPoints), ...
                    t_uniform, x0);
    
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));
    xOut = [x,y,z];
    
    % Calculate minimum distance to target points
    distMid = 0;
    for i = 1:size(xTarget, 1)
        distMid = distMid + min(sqrt(sum((xOut - xTarget(i,:)).^2,2)));
    end
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - xDes).^2,2)));
    
    % Time penalty
    timePenalty = prms(1);

    % Composite error (normalized)
    error = wt(1) * distMid + wt(2) * distEndErr + wt(3) * timePenalty;
end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms, qDes, xTarget, numControlPoints)
    ceq = []; % No equality constraints
    T_total = prms(1);
    t_uniform = 0:0.01:T_total;

    % Extract wn and control points from parameters
    [wn, ctrlPoints] = extractParameters(prms, numControlPoints);

    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1), wn, ctrlPoints), ...
                    t_uniform, zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));
    xOut = [x,y,z];

    % Calculate distances to target points
    c = [];
    for i = 1:size(xTarget, 1)
        distMid = min(sqrt(sum((xOut - xTarget(i,:)).^2,2)));
        c = [c; distMid - 1e-10];
    end
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - [0, 0.05 0.05]).^2,2)));
    c = [c; distEndErr - 1e-10];
end

% Helper function to extract parameters
function [wn, ctrlPoints] = extractParameters(prms, numControlPoints)
    numWnSets = numControlPoints + 1;
    
    wn = cell(numWnSets, 1);
    ctrlPoints = zeros(numControlPoints, 3);
    
    paramIdx = 2;
    for i = 1:numWnSets
        wn{i} = prms(paramIdx:paramIdx+2);
        paramIdx = paramIdx + 3;
    end
    for i = 1:numControlPoints
        ctrlPoints(i,:) = prms(paramIdx:paramIdx+2);
        paramIdx = paramIdx + 3;
    end
end

function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wn, ctrlPoints)
    zeta = [1 1 1];  % Damping ratio
    numControlPoints = size(ctrlPoints, 1);
    numWnSets = numControlPoints + 1;

    % Compute per-joint switching times
    t_Vmax = cell(numWnSets-1, 1);
    t_Vmax{1} = 1 ./ wn{1};
    for i = 2:numWnSets-1
        t_Vmax{i} = t_Vmax{i-1} + 1 ./ wn{i};
    end

    % Get control point joint angles via IK
    qCtrl = zeros(numControlPoints, 3);
    for i = 1:numControlPoints
        qCtrl(i,:) = IK(ctrlPoints(i,1), ctrlPoints(i,2), ctrlPoints(i,3));
    end

    % Initialize natural frequency and control target for each joint
    wn_current = zeros(1, 3);
    qControl = zeros(1, 3);

    % Per-joint switching logic based on t_Vmax
    for i = 1:3
        if t <= t_Vmax{1}(i)
            wn_current(i) = wn{1}(i);
            qControl(i) = qCtrl(1, i);
        else
            % Find which stage we're in
            stage = 1;
            for j = 1:numWnSets-1
                if t > t_Vmax{j}(i)
                    stage = j + 1;
                end
            end
            
            wn_current(i) = wn{stage}(i);
            if stage <= numControlPoints
                qControl(i) = qCtrl(stage, i);
            else
                qControl(i) = qDes(end, i);  % Final desired joint angle
            end
        end
    end

    % Construct system matrices for prefilter dynamics
    A = [zeros(3), eye(3); -diag(wn_current).^2, -2 * diag(zeta) * diag(wn_current)];
    B = [zeros(3); diag(wn_current).^2];

    % Extract actual joint state
    q  = x(7:9);     % Joint angles
    qd = x(10:12);   % Joint velocities

    % PD control gains
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  

    % Control law using prefiltered desired joint values
    controller = Kp * (x(1:3) - q) + Kd * (x(4:6) - qd);

    % Dynamics matrices
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));

    % Compute torque and acceleration
    tau = M * controller + C * qd;
    qdd = M \ (tau - C * qd);

    % Return state derivative
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


