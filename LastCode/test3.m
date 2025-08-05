clear; clc;
close all;

% Define 5 target points
% xTarget = [...
%     0.00, -0.03, -0.03;
%     0.03, -0.01, -0.02;
%     0.06,  0.01, -0.01;
%     0.09,  0.03, 0.0;
%     0.12,  0.05, 0.01];
xTarget = [...
    0.005, -0.03, 0.0;
    0.01, -0.03, 0.0;
    0.015,  -0.03,0.0;
    0.02,  -0.03, 0.0;
    0.025,  -0.03, 0.0];

xTarget = [    0.005, -0.03, 0.0;
];

% Use first and last as control points
xCtrl(1,:) = xTarget(1,:);
xCtrl(2,:) = xTarget(end,:);

qCtrl(1,:) = IK(xCtrl(1,1), xCtrl(1,2), xCtrl(1,3));
qCtrl(2,:) = IK(xCtrl(2,1), xCtrl(2,2), xCtrl(2,3));

% Final desired configuration
qDes = [qCtrl; 0 0 0];

% Weights and Initial Parameters
wt = [1000, 10, 0.0001];
tspan = 5;
wn1 = [2 2 2]; wn2 = [ 2 2 2]; wn3 = [2 2 2 ];
initPrms = [tspan, wn1, wn2, wn3, xCtrl(1,:), xCtrl(2,:)];

% Simulation
t_uniform = 0:0.001:tspan;
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn1, wn2, wn3, xCtrl(1,:), xCtrl(2,:)), ...
                      t_uniform, zeros(12, 1));
[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];


% Bounds
tolRad = 0.02;
lb = [0, 0.5*ones(1,9), xCtrl(1,:)-tolRad, xCtrl(2,:)-tolRad];
ub = [6, 15*ones(1,9), xCtrl(1,:)+tolRad, xCtrl(2,:)+tolRad];

% Optimization
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal);
options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp', ...
    'TolCon', 1e-6, 'OptimalityTolerance', 1e-7, 'StepTolerance', 1e-3, 'MaxIterations', 50);
problem = createOptimProblem('fmincon', ...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xTarget,xFinal));
ms = MultiStart('UseParallel', true, 'Display', 'iter');
[Opt, fval] = run(ms, problem, 5);

% Simulate with optimal parameters
tOpt = 0:0.001:Opt(1);
[tOpt, yOpt] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1), Opt(2:4), Opt(5:7), Opt(8:10), Opt(11:13), Opt(14:16)), ...
                     tOpt, zeros(12, 1));
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9));


% Plotting
figure; hold on; grid on; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Optimized Cartesian Trajectory with Phases');

% Trajectory
plot3(CxOpt, CyOpt, CzOpt, '.-');

% Target Points
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*');

% Final & Control Points
plot3(xFinal(1), xFinal(2), xFinal(3), 'o');
plot3(Opt(11), Opt(12), Opt(13), 'd');  % Ctrl 1
plot3(Opt(14), Opt(15), Opt(16), 'd');  % Ctrl 2

% Sphere radius and mesh
r = 0.01;
[sx, sy, sz] = sphere(20);
surf(Opt(11)+r*sx, Opt(12)+r*sy, Opt(13)+r*sz, 'EdgeColor','none', 'FaceAlpha',0.3, 'FaceColor','b');
surf(Opt(14)+r*sx, Opt(15)+r*sy, Opt(16)+r*sz, 'EdgeColor','none', 'FaceAlpha',0.3, 'FaceColor','b');

legend('Trajectory', 'Target Points', 'Final Point', 'Control Pt 1', 'Control Pt 2');

% Display optimized parameters
fprintf('Optimized Parameters:\n');
fprintf('tspan = %.4f;\n', Opt(1));
fprintf('wn1   = [ %.4f, %.4f, %.4f ];\n', Opt(2:4));
fprintf('wn2   = [ %.4f, %.4f, %.4f ];\n', Opt(5:7));
fprintf('wn3   = [ %.4f, %.4f, %.4f ];\n', Opt(8:10));
fprintf('xCtrl(1,:) = [ %.4f, %.4f, %.4f ];\n', Opt(11:13));
fprintf('xCtrl(2,:) = [ %.4f, %.4f, %.4f ];\n', Opt(14:16));


% % Plot
% figure; hold on; grid on; view(3);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Trajectory with 5 Targets and 2 Control Points');
% plot3(CxOpt, CyOpt, CzOpt, '.-');
% for i = 1:size(xTarget,1)
%     plot3(xTarget(i,1), xTarget(i,2), xTarget(i,3), '*');
% end
% plot3(xFinal(1), xFinal(2), xFinal(3), 'o');
% plot3(Opt(11), Opt(12), Opt(13), 'd');
% plot3(Opt(14), Opt(15), Opt(16), 'd');
% 
% legend('Trajectory', 'Target Points', 'Final', 'Ctrl 1', 'Ctrl 2');
save(sprintf('data%d.mat', 4), ...
    'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal');


% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal)
    tUni = 0:0.001:prms(1);
    [~, y] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, prms(1), prms(2:4), prms(5:7), prms(8:10), prms(11:13), prms(14:16)), ...
                   tUni, zeros(12, 1));
    [x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x0,y0,z0];
    
    distMidF = 0;
    for i = 1:size(xTarget,1)
        distMidF = distMidF + min(sum((xOut - xTarget(i,:)).^2, 2));
    end
    distEndErr = sum((xOut(end,:) - xFinal).^2,2);
    timePenalty = prms(1);
    error = wt(1)*distMidF + wt(2)*distEndErr + wt(3)*timePenalty;
end

% Constraint Function
function [c, ceq] = trajConstraint(prms, qDes, xTarget,xFinal)
    ceq = [];
    tUni = 0:0.001:prms(1);
    [~, yy] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, prms(1), prms(2:4), prms(5:7), prms(8:10), prms(11:13), prms(14:16)), ...
                    tUni, zeros(12, 1));
    [x0,y0,z0] = FK(yy(:,7),yy(:,8),yy(:,9));
    x = [x0,y0,z0];
    idxs = zeros(size(xTarget,1),1);
    for i = 1:size(xTarget,1)
        [~, idxs(i)] = min(sum((x - xTarget(i,:)).^2, 2));
    end
    c = [];
    for i = 1:length(idxs)-1
        c(end+1) = idxs(i) - idxs(i+1);
    end
    for i = 1:size(xTarget,1)
        c(end+1) = min(sum((x - xTarget(i,:)).^2, 2)) - 1e-10;
    end
    c(end+1) = sum((x(end,:) - xFinal).^2) - 1e-10;
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
    dist1 = norm(x_curr - xCtrl1);
    dist2 = norm(x_curr - xCtrl2);

    % Compute joint-space control targets from Cartesian control points
    qCtrl(1,:) = IK(xCtrl1(1), xCtrl1(2),xCtrl1(3));
    qCtrl(2,:) = IK(xCtrl2(1), xCtrl2(2),xCtrl2(3));

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
            wn = wn1;
            qControl = qCtrl(1,:);
        case 2
            wn = wn2;
            qControl = qCtrl(2,:);
        case 3
            wn =  wn3;
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

% function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wn1, wn2, wn3, xCtrl1, xCtrl2)
%     persistent phase
% 
%     % Automatically reset phase at start of new simulation
%     if t == 0
%         phase = 1;
%     elseif isempty(phase)
%         phase = 1;
%     end
% 
%     zeta = [1 1 1];  % Damping ratio
% 
%     % Extract joint state
%     q  = x(7:9);     % Joint angles
%     qd = x(10:12);   % Joint velocities
% 
%     % Current Cartesian position
%     [x_now, y_now, z_now] = FK(q(1), q(2), q(3));
%     x_curr = [x_now, y_now, z_now];
% 
%     % Distance to control points
%     dist1 = norm(x_curr - xCtrl1);
%     dist2 = norm(x_curr - xCtrl2);
% 
%     % Compute joint-space control targets
%     qCtrl(1,:) = IK(xCtrl1(1), xCtrl1(2),xCtrl1(3));
%     qCtrl(2,:) = IK(xCtrl2(1), xCtrl2(2),xCtrl2(3));
% 
%     % Phase transition logic
%     if phase == 1 && dist1 <= 0.01
%         phase = 2;
%     end
%     if phase == 2 && dist2 <= 0.01
%         phase = 3;
%         disp('x')
%     end
% 
%     % Select controller based on phase
%     switch phase
%         case 1
%             wn = wn1;
%             qControl = qCtrl(1,:);
%         case 2
%             wn = wn2;
%             qControl = qCtrl(2,:);
%         case 3
%             wn =  wn3;
%             qControl = qDes(end,:);
%     end
% 
%     % Prefilter dynamics
%     A = [zeros(3), eye(3); -diag(wn).^2, -2 * diag(zeta) * diag(wn)];
%     B = [zeros(3); diag(wn).^2];
% 
%     % PD control gains
%     Kp = diag([70 70 70]);  
%     Kd = diag([120 120 120]);  
% 
%     % Control law
%     controller = Kp * (x(1:3) - q) + Kd * (x(4:6) - qd);
% 
%     % Dynamics
%     [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
%     tau = M * controller + C * qd;
%     qdd = M \ (tau - C * qd);
% 
%     % Final state derivative
%     dxdt = [A * x(1:6) + B * qControl'; qd; qdd];
% end
% 
save(sprintf('data%d.mat', 3), ...
    'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal');



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

