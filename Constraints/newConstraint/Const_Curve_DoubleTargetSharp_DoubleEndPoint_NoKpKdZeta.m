clear; clc;
close all;

% Define desired trajectory and Middle Points
qDes1 = [0.1914, -0.0445,  0.3336];
qDes2 = [0,      -0.0368,  0.3031];
qDes = [qDes1;qDes2];

[xDes1, yDes1, zDes1] = FK(qDes1(1), qDes1(2), qDes1(3));
xDes1 = [xDes1, yDes1, zDes1];

[xDes2, yDes2, zDes2] = FK(qDes2(1), qDes2(2), qDes2(3));
xDes2 = [xDes2, yDes2, zDes2];
xDes = [xDes1;xDes2];

xMid = zeros(4,3);
qMid = zeros(4,3);

xMid(1,:) = [0.02, 0, 0.01];
xMid(2,:) = [0.03, 0, 0.04];
xMid(3,:) = [0.03, 0, 0.07];
xMid(4,:) = [0.015, 0, 0.03];

qMid(1,:) = IK(xMid(1,1), xMid(1,2), xMid(1,3));
qMid(2,:) = IK(xMid(2,1), xMid(2,2), xMid(2,3));
qMid(3,:) = IK(xMid(3,1), xMid(3,2), xMid(3,3));
qMid(4,:) = IK(xMid(4,1), xMid(4,2), xMid(4,3));

% Parameters
tspan = [2 4 6 8 10 12];
wn = [1 1 1 ... 
      1 1 1 ...
      1 1 1 ...
      1 1 1 ...
      1 1 1 ...
      1 1 1];

% Weights
wt = [50, 1, 0.02]; % [Target, End, Time]

initPrms = [tspan, wn];

% Initial Condition
[ti, yi] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn), [0 tspan(end)], zeros(12, 1));

[xOut,yOut,zOut] = FK(yi(:,7),yi(:,8),yi(:,9));
figure; hold on; grid on;
plot(xOut,zOut)
plot(xMid(1,1),xMid(1,3),'*')
plot(xMid(2,1),xMid(2,3),'*')
plot(xMid(3,1),xMid(3,3),'*')
plot(xMid(4,1),xMid(4,3),'*')

plot(xDes1(1,1),xDes1(1,3),'o')
plot(xDes2(1,1),xDes2(1,3),'o')

% Lower and Upper Limits
lb = [0 0 0 0 0 0  ... % time
      0.5 0.5 0.5  0.5 0.5 0.5  0.5 0.5 0.5  0.5 0.5 0.5]; % Wn
ub = [10 10 10 10 10 10 ... % time
      5 5 5   5 5 10   5 5 5 5 5 5 5 5 5]; % Wn

% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xMid, xDes);

% Run optimization
options = optimoptions('fmincon','PlotFcns', 'optimplot', 'Display', 'off', ... 
                        'TolCon', 1e-10); % Added constraint tolerance

% Create optimization problem
problem = createOptimProblem('fmincon',...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xMid));

% MultiStart setup
ms = MultiStart('UseParallel', true, 'Display', 'iter');
numStarts = 10; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);

% Simulate with optimal parameters
[tt, yy] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1:6),  Opt(7:end)), ...
                  [0 Opt(6)], zeros(12, 1));

%%% Plotting
[xi, yi_plot, zi] = FK(yi(:,7), yi(:,8), yi(:,9)); % Initial Trajectory
[x_opt, y_opt, z_opt] = FK(yy(:,7), yy(:,8), yy(:,9)); % Optimized Trajectory

figure; hold on; grid on;
plot(xi, zi,'--')
plot(x_opt,z_opt,'.-')
plot(xMid(1,1),xMid(1,3),'*')
plot(xMid(2,1),xMid(2,3),'*')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')

plot(xDes(1),xDes(3),'o')
legend('Initial Trajectory','Optimized Trajectory','Midpoint','Endpoint')


% Objective Function
function error = objectiveFunction(prms, qDes, wt, xMid, xDes)
    x0 = zeros(12, 1);
    x0(1:3) = qDes(1,:);

    % Simulate the system
    [~, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:6),prms(7:end)), ...
                    [0 prms(6)], x0);

    [xOut,yOut,zOut] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [xOut,yOut,zOut];
    % Calculate minimum distance to middle point
    dx1 = sqrt(sum((xOut - xMid(1,:)).^2,2));
    dx2 = sqrt(sum((xOut - xMid(2,:)).^2,2));
    distMid1 = sum(dx1,1);
    distMid2 = sum(dx2,1);
    distMid = distMid1 + distMid2; 

    % End point error
    dxEnd = sqrt(sum((xOut(end,:) - xDes).^2,2));
    distEndErr = sum(dxEnd,1);
    
    % Time penalty
    timePenalty = prms(1);

    % Composite error (normalized)
    error = wt(1)*distMid + wt(2)*distEndErr + wt(3)*timePenalty;
end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xMid)
    ceq = []; % No equality constraints

    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:6),prms(7:end)), ...
                    [0 prms(6)], zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    x = [x,y,z];
    % Calculate distances to midpoint in 3D space
    distance1 = sqrt(sum((x - xMid(1,:)).^2,2));
    distance2 = sqrt(sum((x - xMid(2,:)).^2,2));
    distance3 = sqrt(sum((x - xMid(3,:)).^2,2));
    distance4 = sqrt(sum((x - xMid(4,:)).^2,2));

    % End point error
    distEndErr1 = sqrt(sum((x(end,:) - [0.05, 0.0,0.05]).^2,2));
    distEndErr2 = sqrt(sum((x(end,:) - [0.05, 0.0,0.05]).^2,2));

    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distance1) - 0.000001;
         min(distance2) - 0.000001;
         min(distance3) - 0.000001;
         min(distance4) - 0.000001;
         % prms(1) - prms(2);
         % prms(2) - prms(3);
         distEndErr1    - 0.000001;
         distEndErr2    - 0.000001]; 
end

% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,wn)
    zeta = [1 1 1];
    A11=[zeros(3), eye(3); -diag(wn(1:3)).^2,-2*diag(zeta)*diag(wn(1:3))];
    B11=[zeros(3); diag(wn(1:3)).^2];
    
    A12=[zeros(3), eye(3); -diag(wn(4:6)).^2,-2*diag(zeta)*diag(wn(4:6))];
    B12=[zeros(3); diag(wn(4:6)).^2];
    
    A13=[zeros(3), eye(3); -diag(wn(7:9)).^2,-2*diag(zeta)*diag(wn(7:9))];
    B13=[zeros(3); diag(wn(7:9)).^2];

    A21=[zeros(3), eye(3); -diag(wn(10:12)).^2,-2*diag(zeta)*diag(wn(10:12))];
    B21=[zeros(3); diag(wn(10:12)).^2];
    
    A22=[zeros(3), eye(3); -diag(wn(13:15)).^2,-2*diag(zeta)*diag(wn(13:15))];
    B22=[zeros(3); diag(wn(13:15)).^2];
    
    A23=[zeros(3), eye(3); -diag(wn(16:18)).^2,-2*diag(zeta)*diag(wn(16:18))];
    B23=[zeros(3); diag(wn(16:18)).^2];

    q=x(7:9);
    qd=x(10:12);

    Kp=diag([70 70 70]);  
    Kd=diag([20 20 20]);  

    controller=Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);
    if t <= t_st(1)
        dxdt=[A11*x(1:6)+B11*qDes(1,:)'; qd; qdd];
    elseif t > t_st(1) && t <= t_st(2)
        dxdt=[A12*x(1:6)+B12*qDes(1,:)'; qd; qdd];
    elseif t > t_st(2) && t <= t_st(3)
        dxdt=[A13*x(1:6)+B13*qDes(1,:)'; qd; qdd];
    elseif t > t_st(3) && t <= t_st(4)
        dxdt=[A21*x(1:6)+B21*qDes(2,:)'; qd; qdd];
    elseif t > t_st(4) && t <= t_st(5)
        dxdt=[A22*x(1:6)+B22*qDes(2,:)'; qd; qdd];
    else
        dxdt=[A23*x(1:6)+B23*qDes(2,:)'; qd; qdd];
    end
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


