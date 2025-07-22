clear; clc;
close all;

qDes = [ 0   0.198678167676855   0.327814256075948 ];
[Px, Py, Pz] = FK(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

xTarget = zeros(2,3);
xCtrl = zeros(2,3);
qCtrl = zeros(2,3);
xTarget(1,:) = [0, 0.035, 0.025];
xTarget(2,:) = [0, 0.015, 0.005];

% Parameters
tspan =  5;
wn1 = [ 2 2 ];
wn2 = [ 2 2 ];
wn3 = [ 2 2 ];
xCtrl(1,:) = xTarget(1,:) + [0 0.03 0.015];
xCtrl(2,:) = xTarget(2,:) - [0 0.03 0.015];

qCtrl(1,:) = IK(xCtrl(1,1), xCtrl(1,2), xCtrl(1,3));
qCtrl(2,:) = IK(xCtrl(2,1), xCtrl(2,2), xCtrl(2,3));

qDes =[qCtrl; qDes];

% Weights
wt = [650, 10, 0.001];   % [Target, End, Time]

initPrms = [tspan, wn1, wn2,wn3, xCtrl(1,2:3),xCtrl(2,2:3)];

t_uniform = 0:0.001:tspan;


% Initial Condition
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  wn1,wn2,wn3,xCtrl(1,2:3),xCtrl(2,2:3)), t_uniform, zeros(12, 1));

% [xi, yi, zi] = FK(yInit(:,7), yInit(:,8), yInit(:,9));     % Initial Trajectory

% %%% Plotting
% figure; hold on; grid on;
% plot(yInit, zInit,'--')
% plot(xTarget(1,2),xTarget(1,3),'*')
% plot(xTarget(2,2),xTarget(2,3),'*')
% plot(xCtrl(1,2),xCtrl(1,3),'d')
% plot(xCtrl(2,2),xCtrl(2,3),'d')
% plot(xFinal(2),xFinal(3),'o')

tolRad = 0.02;

% Lower and Upper Limits
lb = [0      ...  % Time
      0.5 0.5 ...  % wn1
      0.5 0.5 ...  % Wn2
      0.5 0.5 ...  % Wn3
      xTarget(1,2)-tolRad xTarget(1,3)-tolRad       ...  % Control Point1
      xTarget(2,2)-tolRad xTarget(2,3)-tolRad ];         % Control Point2
ub = [3   ...   % Time
      15 15 ...  % wn1
      15 15 ...  % Wn2
      15 15 ... % Wn3
      xTarget(1,2)+tolRad xTarget(1,3)+tolRad ... % Control Point1
      xTarget(2,2)+tolRad xTarget(2,3)+tolRad ];  % Control Point2


% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal);

% Run optimization
options = optimoptions('fmincon','PlotFcns', 'optimplot', 'Display', 'off', ... 
                        'TolCon', 1e-10,'MaxIterations',50); % Added constraint tolerance

% Create optimization problem
problem = createOptimProblem('fmincon',...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xTarget));

% MultiStart setup
ms = MultiStart('UseParallel', true, 'Display', 'iter');
numStarts = 5; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);
tUni = 0:0.001:Opt(1);

% Simulate with optimal parameters
[tOpt, yOpt] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1),  Opt(2:3), Opt(4:5),Opt(6:7),Opt(8:9),Opt(10:11)), ...
                  tUni, zeros(12, 1));

% Forward Kinematics
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9)); % Initial Trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9)); % Optimized Trajectory
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3)); % Optimized Trajectory



dataNum = 19;  % Change this to 2, 3, etc. for other runs


r = 0.01; % Radius
theta = linspace(0, 2*pi, 100); % Angle for circle

x_circle1 = Opt(8) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle1 = Opt(9) + r*sin(theta); % y = center_y + r*sin(θ)
x_circle2 = Opt(10) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle2 = Opt(11) + r*sin(theta); % y = center_y + r*sin(θ)
% Recompute distances to control points
ctrl1 = [0 Opt(8:9)];
ctrl2 = [0 Opt(10:11)];
dist1 = sqrt((CxOpt - ctrl1(1)).^2 + (CyOpt - ctrl1(2)).^2 + (CzOpt - ctrl1(3)).^2);
dist2 = sqrt((CxOpt - ctrl2(1)).^2 + (CyOpt - ctrl2(2)).^2 + (CzOpt - ctrl2(3)).^2);

% % Find the first time where distance to control point 1 is less than threshold (phase 2 start)
% tol = 0.01; % same as in your phase logic
% idx_phase2 = find(dist1 <= tol, 1, 'first');
% t_phase2 = tOpt(idx_phase2);
% 
% % Find the first time after phase 2 where distance to control point 2 is less than threshold (phase 3 start)
% idx_phase3 = find(dist2 <= tol & tOpt <= t_phase2, 1, 'first');
% t_phase3 = tOpt(idx_phase3);

% Plot vertical lines at phase change times
figure; hold on; grid on;
plot(CyOpt, CzOpt, '.-')
plot(xTarget(1,2), xTarget(1,3), '*')
plot(xTarget(2,2), xTarget(2,3), '*')
plot(xFinal(2), xFinal(3), 'o')
plot(Opt(8), Opt(9), 'd')
plot(Opt(10), Opt(11), 'd')
plot(x_circle1, y_circle1, 'b--', 'LineWidth', 1.5);
plot(x_circle2, y_circle2, 'b--', 'LineWidth', 1.5);

legend('Optimized Trajectory','Target Point 1','Target Point 2','End Point','Control Point 1','Control Point 2')
xlabel('Y axis (m)')
ylabel('Z axis (m)')
title('Cartesian Space Trajectory Results with Phase Changes')
disp('Optimal Parameter:')
disp(['tspan = [ ', num2str(Opt(1)), ' ];'])
disp(['wn1 =  [ ', num2str(Opt(2:3)), ' ];'])
disp(['wn2 =  [ ', num2str(Opt(4:5)), ' ];'])
disp(['wn3 =  [ ', num2str(Opt(6:7)), ' ];'])
disp(['CtrlPnt 1= [   ', num2str(Opt(8:9)), ' ];'])
disp(['CtrlPnt 2= [   ', num2str(Opt(10:11)), ' ];'])


   
% Velocity Plot
figure; hold on; grid on;
plot(tOpt, yOpt(:,10:12))
xlabel('Time (s)')
ylabel('Velocity (rad/s)')
title('Velocity')


% Joint position 
figure;
for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(tOpt, yOpt(:,i), '--') % desired
    plot(tOpt, yOpt(:,i+6))     % actual
    ylabel(['Joint ', num2str(i), ' Position (rad)'])
    if i == 3
        xlabel('Time (s)')
    end
    legend('Desired', 'Actual')
end
% saveas(gcf, sprintf('data%dJointPosition.fig', dataNum))  % Dynamic name



% Joint Velocity 
figure;
for i = 4:6
    subplot(3,1,i-3); hold on; grid on;
    plot(tOpt, yOpt(:,i), '--') % desired
    plot(tOpt, yOpt(:,i+6))     % actual
    ylabel(['Joint ', num2str(i), ' Position (rad)'])
    if i == 3
        xlabel('Time (s)')
    end
    legend('Desired', 'Actual')
end
save(sprintf('data%d.mat', dataNum), ...
    'Opt', 'tOpt','yOpt','tInit','yInit','xTarget');
% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal)
    tUni =  0:0.001:prms(1);
    % Simulate the system
    [t, y] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, prms(1),  prms(2:3), prms(4:5),prms(6:7),prms(8:9),prms(10:11)), ...
                    tUni, zeros(12, 1));
    % y_uniform = interp1(t,y,t_uniform);
    [x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x0,y0,z0];
    


    [tVal1,tIdx1] =  min( sum((xOut -  xTarget(1,:)).^2,2) );
    [tVal2,tIdx2] =  min( sum((xOut -  xTarget(2,:)).^2,2) );
    
    
    % Calculate minimum distance to middle point
    distMid1 = sum(min(sum((xOut(1:tIdx1,:) - permute(xTarget(1,:), [3 2 1])).^2, 2), [], 1));
    distMid2 = sum(min(sum((xOut(tIdx1:end,:) - permute( xTarget(2,:), [3 2 1])).^2, 2), [], 1));

    distMidF = distMid1 + distMid2;

    % End point error
    distEndErr = sum((xOut(end,:) - xFinal).^2,2);

    
    % Time penalty
    timePenalty = prms(1);

    % Composite error (normalized)
    error = wt(1) * distMidF    + ...
            wt(2) * distEndErr + ...
            wt(3) * timePenalty;
end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xTarget)
    ceq = []; % No equality constraints
    tUni = 0:0.001:prms(1);
    % Simulate trajectory
    [tt, yy] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, prms(1),  prms(2:3), prms(4:5),prms(6:7),prms(8:9),prms(10:11)), ...
                   tUni , zeros(12, 1));
    
    [x0,y0,z0] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    x = [x0,y0,z0];
    
    % Order Section
    [tVal1,tIdx1] = min(  sum((x -  xTarget(1,:)).^2,2)  );
    [tVal2,tIdx2] = min(  sum((x -  xTarget(2,:)).^2,2)  );
    
    % Calculate distances to midpoint in 3D space
    distanceMid1  = sum((x(1:tIdx1,:) -  xTarget(1,:)).^2,2);
    distanceMid2  = sum((x(tIdx1:end,:) -  xTarget(2,:)).^2,2);
     

    % End point error
    distEndErr = sum((x(end,:) - [0.0, 0.05, 0.05]).^2,2);
    
    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distanceMid1) - 1e-10;
         min(distanceMid2) - 1e-10;
         distEndErr    - 1e-10;
         tIdx1 - tIdx2];

end
 

function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wn1, wn2, wn3, xCtrl1, xCtrl2)
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
    dist1 = norm(x_curr - [0 xCtrl1]);
    dist2 = norm(x_curr - [0 xCtrl2]);

    % Compute joint-space control targets
    qCtrl(1,:) = IK(0, xCtrl1(1), xCtrl1(2));
    qCtrl(2,:) = IK(0, xCtrl2(1), xCtrl2(2));

    % Phase transition logic
    if phase == 1 && dist1 <= 0.01
        phase = 2;
    end
    if phase == 2 && dist2 <= 0.01
        phase = 3;
    end

    % Select controller based on phase
    switch phase
        case 1
            wn = [1 wn1];
            qControl = qCtrl(1,:);
        case 2
            wn = [1 wn2];
            qControl = qCtrl(2,:);
        case 3
            wn = [1 wn3];
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

