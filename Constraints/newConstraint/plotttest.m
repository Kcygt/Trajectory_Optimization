%%%%%
% 
% Optimal Parameter:
% Time: 3.3134
% zeta: 0.86265     0.55127     0.99884
% Wn:   16.1593      3.08098      3.56322
%%%%%

clear; clc;
close all;

% Define desired trajectory and Middle Points
qDes = [0.1914, -0.0445, 0.3336];
[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xDes, yDes, zDes];

xMid = [0.015, 0, 0.04];
xMid = [0.04, 0, 0.005];
% xMid = [0.03, 0, 0.03];

qMid = IK(xMid(1), xMid(2), xMid(3));

% Parameters
tspan = 10;
zeta = [1 1 1];
wn1 = [3 1 1.5];
wn2 = [1 5 1];
wn3 = [2 3 5];


% Initial Condition
[ti1, yi1] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  zeta,wn1), [0 tspan], zeros(12, 1));
[ti2, yi2] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  zeta,wn2), [0 tspan], zeros(12, 1));
[ti3, yi3] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  zeta,wn3), [0 tspan], zeros(12, 1));

[xi1, yi_plot, zi1] = FK(yi1(:,7), yi1(:,8), yi1(:,9)); % Initial Trajectory
[xi2, yi_plot, zi2] = FK(yi2(:,7), yi2(:,8), yi2(:,9)); % Initial Trajectory
[xi3, yi_plot, zi3] = FK(yi3(:,7), yi3(:,8), yi3(:,9)); % Initial Trajectory

colors = ["#77AC30", "#7E2F8E", "#0072BD"];

figure
subplot 131
hold on; grid on
plot(ti2,yi2(:,7),"Color",colors(2),'LineWidth',1.5)
plot(ti1,yi1(:,7),"Color",colors(1),'LineWidth',1.5)
plot(ti3,yi3(:,7),"Color",colors(3),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Joint Position (rad)')
title(' Joint 1 for different Natural Frequency')
text(0.4, 0.18, '\omega_{n2q_1} = 3', 'Color', colors(1), 'FontSize', 10);
text(2.5, 0.12, '\omega_{n1q_1} = 1', 'Color', colors(2), 'FontSize', 10);
text(1.7, 0.16, '\omega_{n3q_1} = 1.5', 'Color', colors(3), 'FontSize', 10);


subplot 132
hold on; grid on
plot(ti2,yi2(:,9),"Color",colors(2),'LineWidth',1.5)
plot(ti1,yi1(:,9),"Color",colors(1),'LineWidth',1.5)
plot(ti3,yi3(:,9),"Color",colors(3),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Joint Position (rad)')
title(' Joint 3 for different Natural Frequency')
text(1, 0.32, '\omega_{n2q_3} = 2', 'Color', colors(1), 'FontSize', 10);
text(2.5, 0.2, '\omega_{n1q_3} = 1', 'Color', colors(2), 'FontSize', 10);
text(1.2, 0.3, '\omega_{n3q_3} = 5', 'Color', colors(3), 'FontSize', 10);

subplot 133
hold on; grid on
plot(xi2,zi2,"Color",colors(2),'LineWidth',2.5)
plot(xi1,zi1,"Color",colors(1),'LineWidth',2.5)
plot(xi3,zi3,"Color",colors(3),'LineWidth',2.5)
xlabel('X Axis (m)')
ylabel('Z Axis (m)')
title('Cartesian Space Trajectory')
legend('Case 1(\omega_{n1q_1} = 1 \omega_{n1q_3} = 1)','Case 2(\omega_{n2q_1} = 3 \omega_{n2q_3} = 2)','Case 3(\omega_{n2q_1} = 1.5 \omega_{n2q_3} = 5)')


% 
% plot(xi,zi)
% plot(0,0,'*')
% plot(xDes(1),xDes(3),'*')
% plot(xMid(1),xMid(3),'o')
% xlabel('z Axis (m)')
% ylabel('x Axis (m)')
% title('Cartesian Space Trajectory')
% legend('Optimized Trajectory','Start Point','End Point','Target Point')


% Lower and Upper Limits
lb = [2 ... % time
      0.5 0.5 0.5 ... % Wn
      0.1 0.1 0.1]; % Wn
ub = [10 ... % time
      1 1 1 ... % Zeta
      20 20 20]; % Wn

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
numStarts = 5; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);

% Simulate with optimal parameters
[tt, yy] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1),  Opt(2:4), Opt(5:7)), ...
                  [0 Opt(1)], zeros(12, 1));

%%% Plotting
[xi1, yi_plot, zi] = FK(yi1(:,7), yi1(:,8), yi1(:,9)); % Initial Trajectory
[x_opt, y_opt, z_opt] = FK(yy(:,7), yy(:,8), yy(:,9)); % Optimized Trajectory

figure; hold on; grid on;
plot(xi1, zi,'--')
plot(x_opt,z_opt,'.-')
plot(xMid(1),xMid(3),'*')
plot(xDes(1),xDes(3),'o')
legend('Initial Trajectory','Optimized Trajectory','Midpoint','Endpoint')

disp('Optimal Parameter:')
disp(['Time: ', num2str(Opt(1))])
disp(['zeta: ', num2str(Opt(2:4))])
disp(['Wn:   ', num2str(Opt(5:7))])

% Objective Function
function error = objectiveFunction(prms, qDes, wt, xMid, xDes)
    x0 = zeros(12, 1);
    x0(1:3) = qDes;

    % Simulate the system
    [~, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),prms(2:4),prms(5:7)), ...
                    [0 prms(1)], x0);

    [xOut,yOut,zOut] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [xOut,yOut,zOut];
    % Calculate minimum distance to middle point
    dx = sqrt(sum((xOut - xMid).^2,2));
    distMid = sum(dx,1);
    % distMid = min(dx);
    

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
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),prms(2:4),prms(5:7)), ...
                    [0 prms(1)], zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    x = [x,y,z];
    % Calculate distances to midpoint in 3D space
    distance = sqrt(sum((x - xMid).^2,2));
    
    % End point error
    distEndErr = sqrt(sum((x(end,:) - [0.05, 0.0,0.05]).^2,2));

    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distance) - 0.000001;
         distEndErr    - 0.000001]; 
end

% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,zeta,wn)
    A1=[zeros(3), eye(3); -diag(wn).^2,-2*diag(zeta)*diag(wn)];
    B1=[zeros(3); diag(wn).^2];

    q=x(7:9);
    qd=x(10:12);

    Kp=diag([70 70 70]);  
    Kd=diag([20 20 20]);  

    controller=Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);

    dxdt=[A1*x(1:6)+B1*qDes(:); qd; qdd];
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


