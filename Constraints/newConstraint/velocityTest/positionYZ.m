clear; clc;
close all;
% % Parameters
% tspan = [ 0.84558      2.2716 ];
% wn1 =  [ 1.1914     0.54921      1.0401 ];
% wn2 =  [ 5.064      6.8163      8.3261 ];
% CtrlPnt = [   0.046569    0.010029    0.012623 ];
% Define desired trajectory and Middle Points


% 
% Optimal Parameter:
% tspan = [ 0.26696     0.83201 ];
% wn1 =  [ 3.21964       30.335      10.7319 ];
% wn2 =  [ 33.9988      27.2333      18.2567 ];
% CtrlPnt = [   0.01        0.03        0.05 ];
% 
% Optimal Parameter:
% tspan = [ 0.15414     0.70348 ];
% wn1 =  [ 39.9999      10.4065      11.1872 ];
% wn2 =  [ 7.93251      29.2536      8.92199 ];
% CtrlPnt = [   0.024682        0.01        0.05 ];

qDes = [ 0   0.198678167676855   0.327814256075948 ];

[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xDes, yDes, zDes];

xMid = zeros(3,3);

xMid(1,:) = [0, 0.015, 0.01];
xMid(2,:) = [0, 0.025, 0.03];
xMid(3,:) = [0, 0.035, 0.045];


% Parameters
tspan = [10 20];
wn1 = [1 1 1];
wn2 = [1 1 1];
CtrlPnt = xMid(2,:);
qCtrl = IK(CtrlPnt(1), CtrlPnt(2), CtrlPnt(3));


qDes =[qCtrl;qDes];

% Weights
wt = [100, 1, 0.08, 0.0001];   % [Target, End, Time]

initPrms = [tspan, wn1, wn2, CtrlPnt];

t_uniform = 0:0.01:tspan(2);

% Initial Condition
[ti, yi] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  wn1,wn2,CtrlPnt), t_uniform, zeros(12, 1));

[xi, yi_plot, zi] = FK(yi(:,7), yi(:,8), yi(:,9)); % Initial Trajectory
% Plot
figure(1); hold on; grid on;
plot(0,0,'o',xDes(2),xDes(3),'o')
plot(xMid(1,2),xMid(1,3),'*')
plot(xMid(2,2),xMid(2,3),'*')
plot(xMid(3,2),xMid(3,3),'*')
plot(CtrlPnt(2),CtrlPnt(3),'d')
plot(yi_plot,zi)
xlabel('Y axes')
ylabel('Z Axes')
title(' Trajectories')

% Lower and Upper Limits
lb = [0 0      0.5 0.5 0.5     0.5 0.5 0.5    0.01 0.01 0.01];     % Wn
ub = [2 2      40 40 40        40 40 40       0.05 0.03 0.05];      % wn

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
t_opt = 0:0.01:Opt(2);

% Simulate with optimal parameters
[tt, yy] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1:2),  Opt(3:5), Opt(6:8),Opt(9:11)), ...
                  t_opt, zeros(12, 1));

%%% Plotting
[xi, yi_plot, zi] = FK(yi(:,7), yi(:,8), yi(:,9));     % Initial Trajectory
[x_opt, y_opt, z_opt] = FK(yy(:,7), yy(:,8), yy(:,9)); % Optimized Trajectory

figure; hold on; grid on;
plot(yi_plot, zi,'--')
plot(y_opt,z_opt,'.-')
plot(xMid(1,2),xMid(1,3),'*')
plot(xMid(2,2),xMid(2,3),'*')
plot(xMid(3,2),xMid(3,3),'*')

plot(xDes(2),xDes(3),'o')
plot(Opt(10),Opt(11),'d')

legend('Initial Trajectory','Optimized Trajectory','Target Point','End Point','Control Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
disp('Optimal Parameter:')
disp(['tspan = [ ', num2str(Opt(1:2)), ' ];'])
disp(['wn1 =  [ ', num2str(Opt(3:5)), ' ];'])
disp(['wn2 =  [ ', num2str(Opt(6:8)), ' ];'])
disp(['CtrlPnt = [   ', num2str(Opt(9:11)), ' ];'])



% 3D Plot
figure; hold on; grid on; view(3); axis equal;

% Plot trajectories
plot3(xi, yi_plot, zi, '--', 'LineWidth', 1.5);      % Initial Trajectory
plot3(x_opt, y_opt, z_opt, '.-', 'LineWidth', 2);    % Optimized Trajectory

% Plot important points
plot3(xMid(1,1), xMid(1,2), xMid(1,3), '*', 'MarkerSize', 8); % Target point 1
plot3(xMid(2,1), xMid(2,2), xMid(2,3), '*', 'MarkerSize', 8); % Target point 2
plot3(xMid(3,1), xMid(3,2), xMid(3,3), '*', 'MarkerSize', 8); % Target point 3

plot3(xDes(1), xDes(2), xDes(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);  % Target point
plot3(Opt(9), Opt(10), Opt(11), 'd', 'MarkerSize', 8);                  % Control point

% Labels and legend
legend('Initial Trajectory', 'Optimized Trajectory', ...
       'Mid Point 1', 'Mid Point 2', 'Mid Point 3', ...
       'Target Point', 'Control Point');

xlabel('X axis (m)');
ylabel('Y axis (m)');
zlabel('Z axis (m)');
title('3D Cartesian Space Trajectory Results');


% Objective Function
function error = objectiveFunction(prms, qDes, wt, xMid, xDes)
    
    x0 = zeros(12, 1);
    % x0(1:3) = qDes(1,:);  & look at it to understand why you are using

    % time interpolation
    T_total = prms(2);
    t_uniform = 0:0.01:T_total;

    % Simulate the system
    [t, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:2), prms(3:5), prms(6:8), prms(9:11)), ...
                    t_uniform, x0);
    % y_uniform = interp1(t,y,t_uniform);
    [xOut,yOut,zOut] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [xOut,yOut,zOut];
    
    % Calculate minimum distance to middle point
    distMidF = sum(min(sum((xOut - permute(xMid, [3 2 1])).^2, 2), [], 1));

    % End point error
    distEndErr = sum((xOut(end,:) - xDes).^2,2);
    
    % Time penalty
    timePenalty = prms(2);

    % Composite error (normalized)
    error = wt(1) * distMidF    + ...
            wt(2) * distEndErr + ...
            wt(3) * timePenalty;
end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xMid)
    ceq = []; % No equality constraints
    T_total = prms(2);
    t_uniform = 0:0.01:T_total;
    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:2),prms(3:5),prms(6:8),prms(9:11)), ...
                    t_uniform, zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    x = [x,y,z];
    % Calculate distances to midpoint in 3D space
    distanceMid1  = sum((x - xMid(1,:)).^2,2);
    distanceMid2  = sum((x - xMid(2,:)).^2,2);
    distanceMid3  = sum((x - xMid(3,:)).^2,2);

    
    % End point error
    distEndErr = sum((x(end,:) - [0.0, 0.05,0.05]).^2,2);

 
    
    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distanceMid1) - 0.0000001;
         min(distanceMid2) - 0.0000001;
         min(distanceMid3) - 0.0000001;
         distEndErr    - 0.0000001;
         prms(1) - prms(2)];

end

% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,wn1,wn2,ctrlPnt)
    zeta = [1 1 1];

    A1=[zeros(3), eye(3); -diag(wn1).^2,-2*diag(zeta)*diag(wn1)];
    B1=[zeros(3); diag(wn1).^2];
    
    A2=[zeros(3), eye(3); -diag(wn2).^2,-2*diag(zeta)*diag(wn2)];
    B2=[zeros(3); diag(wn2).^2];
    
    qCtrl = IK(ctrlPnt(1), ctrlPnt(2), ctrlPnt(3));

    q=x(7:9);
    qd=x(10:12);
    
    Kp = diag([70 70 70]);  
    Kd = diag([20 20 20]);  

    controller = Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);

    if t <= t_st(1)
        dxdt = [A1*x(1:6) + B1*qCtrl'; qd; qdd];
    else
        dxdt = [A2*x(1:6) + B2*qDes(2,:)'; qd; qdd];
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


