clear; clc; close all;

% === Define Target Points ===
xTarget = [
    -0.02, -0.03, 0.0;
     0.00, -0.03, 0.0;
     0.02, -0.03, 0.0
];

numCtrl = size(xTarget, 1);  % Number of control points

% === Inverse Kinematics for Control Points ===
xCtrlList = xTarget;
qCtrlList = zeros(numCtrl, 3);
for i = 1:numCtrl
    qCtrlList(i, :) = IK(xCtrlList(i,1), xCtrlList(i,2), xCtrlList(i,3));
end

qDes = [qCtrlList; 0 0 0];  % Final configuration

% === Frequency Setup ===
wnList = repmat([2 2 2], numCtrl+1, 1);  % (control phases + final)

% === Simulation Parameters ===
tspan = 5;
initPrms = [tspan; wnList(:); xCtrlList(:)];

% === Initial Simulation ===
t_uniform = 0:0.001:tspan;
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wnList, xCtrlList), ...
                      t_uniform, zeros(12, 1));

[Px, Py, Pz] = FK(qDes(end,1), qDes(end,2), qDes(end,3));
xFinal = [Px, Py, Pz];

% === Bounds ===
tolRad = 0.02;
lb = [1; 0.5*ones(3*(numCtrl+1),1); (xCtrlList(:) - tolRad)];
ub = [10; 15*ones(3*(numCtrl+1),1); (xCtrlList(:) + tolRad)];

% === Optimization ===
wt = [1000, 10, 0.0001];  % weights: path, end, time
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal, numCtrl);
nonlconFunc = @(params) trajConstraint(params, qDes, xTarget, xFinal, numCtrl);

options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp', ...
    'TolCon', 1e-6, 'OptimalityTolerance', 1e-7, 'StepTolerance', 1e-3, 'MaxIterations', 50);

problem = createOptimProblem('fmincon', ...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', nonlconFunc);

ms = MultiStart('UseParallel', true, 'Display', 'iter');
[Opt, fval] = run(ms, problem, 5);

% === Simulate Optimized Trajectory ===
tOpt = 0:0.001:Opt(1);
wnListOpt = reshape(Opt(2:1+3*(numCtrl+1)), numCtrl+1, 3);
xCtrlListOpt = reshape(Opt(2+3*(numCtrl+1):end), numCtrl, 3);
[tOpt, yOpt] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1), wnListOpt, xCtrlListOpt), ...
                     tOpt, zeros(12, 1));

[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9));

% === Plotting ===
figure; hold on; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Generalized Trajectory with Optimized Control Points');

plot3(CxOpt, CyOpt, CzOpt, '.-');                % Trajectory
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*');  % Targets
plot3(xFinal(1), xFinal(2), xFinal(3), 'o');     % Final

% Control Points
for i = 1:numCtrl
    plot3(xCtrlListOpt(i,1), xCtrlListOpt(i,2), xCtrlListOpt(i,3), 'd');
end
legend('Trajectory','Target Points','Final','Control Points');

save(sprintf('data_general.mat'), ...
    'Opt','tOpt','yOpt','tInit','yInit','xTarget','xFinal');


function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal, numCtrl)
    tspan = prms(1);
    wnList = reshape(prms(2:1+3*(numCtrl+1)), numCtrl+1, 3);
    xCtrlList = reshape(prms(2+3*(numCtrl+1):end), numCtrl, 3);

    tUni = 0:0.001:tspan;
    [~, y] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, tspan, wnList, xCtrlList), ...
                   tUni, zeros(12, 1));
    [x0, y0, z0] = FK(y(:,7), y(:,8), y(:,9));
    xOut = [x0, y0, z0];

    distMidF = 0;
    for i = 1:size(xTarget,1)
        distMidF = distMidF + min(sum((xOut - xTarget(i,:)).^2, 2));
    end
    distEndErr = sum((xOut(end,:) - xFinal).^2);
    timePenalty = tspan;

    error = wt(1)*distMidF + wt(2)*distEndErr + wt(3)*timePenalty;
end


function [c, ceq] = trajConstraint(prms, qDes, xTarget, xFinal, numCtrl)
    ceq = [];
    tspan = prms(1);
    wnList = reshape(prms(2:1+3*(numCtrl+1)), numCtrl+1, 3);
    xCtrlList = reshape(prms(2+3*(numCtrl+1):end), numCtrl, 3);

    tUni = 0:0.001:tspan;
    [~, y] = ode45(@(t,x) myTwolinkwithprefilter(t, x, qDes, tspan, wnList, xCtrlList), ...
                   tUni, zeros(12, 1));
    [x0, y0, z0] = FK(y(:,7), y(:,8), y(:,9));
    xOut = [x0, y0, z0];

    c = [];
    for i = 1:size(xTarget,1)
        c = [c; 0.001 - min(sum((xOut - xTarget(i,:)).^2, 2))];
    end
end





function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wnList, xCtrlList)
    persistent phase

    if t == 0 || isempty(phase)
        phase = 1;
    end

    zeta = [1 1 1];
    q = x(7:9);
    qd = x(10:12);
    [x_now, y_now, z_now] = FK(q(1), q(2), q(3));
    x_curr = [x_now, y_now, z_now];

    numCtrl = size(xCtrlList, 1);
    qCtrl = zeros(numCtrl, 3);
    for i = 1:numCtrl
        qCtrl(i,:) = IK(xCtrlList(i,1), xCtrlList(i,2), xCtrlList(i,3));
    end

    if phase <= numCtrl
        if norm(x_curr - xCtrlList(phase,:)) <= 0.01
            phase = phase + 1;
        end
    end

    if phase <= numCtrl
        wn = wnList(phase, :);
        qControl = qCtrl(phase, :);
    else
        wn = wnList(end, :);
        qControl = qDes(end, :);
    end

    A = [zeros(3), eye(3); -diag(wn).^2, -2*diag(zeta).*diag(wn)];
    B = [zeros(3); diag(wn).^2];
    Kp = diag([70 70 70]);
    Kd = diag([120 120 120]);
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
    tau = M*controller + C*qd;
    qdd = M \ (tau - C*qd);

    dxdt = [A*x(1:6) + B*qControl'; qd; qdd];
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

