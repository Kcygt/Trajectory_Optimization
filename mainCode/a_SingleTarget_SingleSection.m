clear; clc;
close all;

% Define desired trajectory and Middle Points
qDes = [ 0.077364659427973   0.197461387421127   0.332371171870866 ];
[Px, Py, Pz] = FK(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

xTarget1 = [0.01, 0.04, 0.02];
xTarget2 = [0, 0.015, 0.04];
xTarget3 = [0,0.03 , 0.03];

% Select one of the xTargets
xTarget = xTarget1;  % Change this to xTarget2 or xTarget3 as needed

% Determine which xTarget was selected
if isequal(xTarget, xTarget1)
    dataNum = 1;
elseif isequal(xTarget, xTarget2)
    dataNum = 2;
elseif isequal(xTarget, xTarget3)
    dataNum = 3;
else
    error('xTarget does not match any predefined targets.');
end

% Parameters
tspan = 10;
wn = [1 1 1];

% Weights
wt = [1000, 10, 0.01]; % [Target, End, Time]

initPrms = [tspan, wn];

% Initial Condition
[tInit, yInit] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  wn), [0 tspan], zeros(12, 1));

t_uniform = 0:0.01:tspan;

% Lower and Upper Limits
lb = [0 ... % time
      0.5 0.5 0.5 ]; % Wn
ub = [2 ... % time
      25 25 25]; % Wn

% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal);

% Run optimization
options = optimoptions('fmincon','PlotFcns', [], 'Display', 'off', ... 
                        'TolCon', 1e-10,    'MaxFunctionEvaluations', 200); % Limit ; % Added constraint tolerance

% Create optimization problem
problem = createOptimProblem('fmincon',...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xTarget,xFinal));

% MultiStart setup
ms = MultiStart('UseParallel', true, 'Display', 'iter');
numStarts = 5; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);

% Simulate with optimal parameters
[tOpt, yOpt] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1),  Opt(2:4)), ...
                  [0 Opt(1)], zeros(12, 1));

%%% Plotting
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9)); % Initial Trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9)); % Optimized Trajectory
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3)); % Optimized Trajectory

% 3D Cartesian Trajectory
figure; hold on; grid on;
plot3(CxInit,CyInit,CzInit,'--')
plot3(CxDes,CyDes,CzDes,'o')
plot3(CxOpt,CyOpt,CzOpt,'.')
plot3(xTarget(1),xTarget(2),xTarget(3),'*')
plot3(xFinal(1),xFinal(2),xFinal(3),'o')
legend('Init','Desired','Optimized','Target','End')
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
title('3D Trajectory')
view(45, 30)  % <-- This ensures 3D view
disp(['Opt Param: ', num2str(Opt)])


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
    'Opt','tOpt','yOpt','tInit','yInit','xTarget');




% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xDes)
    x0 = zeros(12, 1);
    x0(1:3) = qDes;

    % Simulate the system
    [~, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),prms(2:4)), ...
                    [0 prms(1)], x0);

    [xO,yO,zO] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [xO,yO,zO];
   
    % Calculate minimum distance to middle point
    distMid = min(sqrt(sum((xOut - xTarget).^2,2)));
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - xDes).^2,2)));
    
    % Time penalty
    timePenalty = prms(1);

    % Composite error (normalized)
    error = wt(1)*distMid + wt(2)*distEndErr + wt(3)*timePenalty;
end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xTarget,xFinal)
    ceq = []; % No equality constraints

    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1),prms(2:4)), ...
                    [0 prms(1)], zeros(12, 1));
    [xO,yO,zO] = FK(yy(:,7),yy(:,8),yy(:,9));
    xOut = [xO,yO,zO];
   
    % Calculate minimum distance to middle point
    distMid = min(sqrt(sum((xOut - xTarget).^2,2)));
    
    % End point error
    distEndErr = min(sqrt(sum((xOut - xFinal).^2,2)));

    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distMid) - 1e-10;
         distEndErr    - 1e-10]; 
end

% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,wn)
    zeta = [1 1 1];
    A1=[zeros(3), eye(3); -diag(wn).^2,-2*diag(zeta)*diag(wn)];
    B1=[zeros(3); diag(wn).^2];

    q=x(7:9);
    qd=x(10:12);

    Kp=diag([70 70 70]);  
    Kd=diag([120 120 120]);  

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


