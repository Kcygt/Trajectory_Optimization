clear; clc;
close all;

% Define desired trajectory and Middle Points
qDes = [    0.191425481525343   0.190854007095390   0.356154760943820 ];
[Px, Py, Pz] = FK(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];


xTarget = zeros(2,3);
xTarget(1,:) = [0.015, 0.02, 0.01];
xTarget(2,:) = [0.035, 0.03, 0.04];
% 
% xTarget(1,:) = [0, 0.01, 0.02];
% xTarget(2,:) = [0, 0.04, 0.03];
% 
% xTarget(1,:) = [0, 0.02, 0.02];
% xTarget(2,:) = [0, 0.04, 0.04];
% % 
% xTarget(1,:) = [0, 0.02, 0.01];
% xTarget(2,:) = [0, 0.04, 0.03];

% 

tspan = [.4 .6 3];
wn = [3 5 7   3 10 1    1 1 1];
% wn = [2.6269    0.9641    0.9804  1.4005    2.9646    5.9315  1.0418    0.7773    2.9987];
% tspan = [0.4579    1.4884    7.4918];
% Weights
wt = [1000, 5, 0.001]; % [Target, End, Time]

initPrms = [tspan, wn];

tUni = 0:0.001: tspan(end);
% Initial Condition
[tInit, yInit] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn),tUni, zeros(12, 1));

% %%% Plotting
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9)); % Initial Trajectory
% 
% % Cartesian Space Trajectory 
figure; hold on; grid on;
plot3(CxInit,CyInit,CzInit,'--')
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*','LineW',1.2,'MarkerSize',8,'Color',[0,0.4,0.8])
plot3(xFinal(1),xFinal(2),xFinal(3),'p','LineW',1.5,'MarkerSize',14,'Color',[1,0.5,0.05])
legend('Init','Desired','Opt','Target','End')
xlabel('X'); ylabel('Y'); zlabel('Z')
title('3D Trajectory')
view(45,30)



% Lower and Upper Limits
lb = [0 0 0  ... % time
      0.5 0.5 0.5  .5 0.5 0.5  .5 0.5 0.5 ]; % Wn
ub = [5 5 5 ... % time
      10  10  10   10  10  10   10 10  10]; % Wn

% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xTarget, xFinal);

% Run optimization
% options = optimoptions('fmincon','PlotFcns', 'optimplot', 'Display', 'off', ... 
%                         'TolCon', 1e-10,'MaxIterations',50); % Added constraint tolerance
options = optimoptions('fmincon', ...
    'Display', 'none', ...
    'Algorithm', 'sqp', ...
    'TolCon', 1e-10, ...
    'OptimalityTolerance', 1e-10, ...
    'StepTolerance', 1e-10, ...
    'MaxIterations', 150);
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
numStarts =10; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);
tUni = 0:0.001: Opt(3);

% Simulate with optimal parameters
[tOpt, yOpt] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1:3),  Opt(4:12)), ...
                 tUni, zeros(12, 1));

%%% Plotting
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9)); % Initial Trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9)); % Optimized Trajectory
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3)); % Optimized Trajectory


dataNum = 17;  % Change this to 2, 3, etc. for other runs

% Cartesian Space Trajectory 
% 3D Cartesian Trajectory (Short)
figure; hold on; grid on;
plot3(CxInit,CyInit,CzInit,'--')
plot3(CxDes,CyDes,CzDes,'o','LineW',1,'Color',[0.17,0.63,0.17],'MarkerSize',7)
plot3(CxOpt,CyOpt,CzOpt,'-.','LineW',1.2,'Color',[0.8,0,0.8])
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*','LineW',1.2,'MarkerSize',8,'Color',[0,0.4,0.8])
plot3(xFinal(1),xFinal(2),xFinal(3),'p','LineW',1.5,'MarkerSize',14,'Color',[1,0.5,0.05])
legend('Init','Desired','Opt','Target','End')
xlabel('X'); ylabel('Y'); zlabel('Z')
title('3D Trajectory')
view(45,30)
disp(['Opt Param: ', num2str(Opt)])

% saveas(gcf, sprintf('data%dCartesianPosition.fig', dataNum))  % Dynamic name


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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Objective Function
function error = objectiveFunction(prms, qDes, wt, xTarget, xFinal)
    x0 = zeros(12, 1);
    x0(1:3) = qDes;
    tUni = 0:0.001:prms(3);
    % Simulate the system
    [~, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:3),prms(4:12)), ...
                    tUni, x0);

    [x,y,z] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [x,y,z];
    % Calculate minimum distance to middle point
    distMid1 = min(sqrt(sum((xOut - xTarget(1,:)).^2,2)));
    distMid2 = min(sqrt(sum((xOut - xTarget(2,:)).^2,2)));

    distMid = distMid1 + distMid2; 

    % End point error
    distEndErr = min(sqrt(sum((xOut - xFinal).^2,2)));


    % Time penalty
    timePenalty = prms(3);

    % Composite error (normalized)
    % error = wt(1)*distMid + wt(2)*distEndErr + wt(3)*timePenalty + 0.01*d;
    error = wt(1)*distMid + wt(2)*distEndErr + wt(3)*timePenalty ;

end

% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xTarget,xFinal)
    ceq = []; % No equality constraints
    tUni = 0:0.001: prms(3);

    % Simulate trajectory
    [~, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:3),prms(4:12)), ...
                    tUni, zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    xOut = [x,y,z];
    % Calculate distances to midpoint in 3D space
    distMid1 = min(sqrt(sum((xOut - xTarget(1,:)).^2,2)));
    distMid2 = min(sqrt(sum((xOut - xTarget(2,:)).^2,2)));


    % End point error
    distEndErr = min(sqrt(sum((xOut - xFinal).^2,2)));

    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distMid1) - 1e-10;
        min(distMid2) - 1e-10;
        distEndErr    - 1e-10;
        prms(1) - prms(2);
        prms(2) - prms(3)]; 
end

% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,wn)
    zeta = [1 1 1];
    A1=[zeros(3), eye(3); -diag(wn(1:3)).^2,-2*diag(zeta)*diag(wn(1:3))];
    B1=[zeros(3); diag(wn(1:3)).^2];
    
    A2=[zeros(3), eye(3); -diag(wn(4:6)).^2,-2*diag(zeta)*diag(wn(4:6))];
    B2=[zeros(3); diag(wn(4:6)).^2];
    
    A3=[zeros(3), eye(3); -diag(wn(7:9)).^2,-2*diag(zeta)*diag(wn(7:9))];
    B3=[zeros(3); diag(wn(7:9)).^2];

    q=x(7:9);
    qd=x(10:12);

    Kp=diag([70 70 70]);  
    Kd=diag([120 120 120]);  
    
    
    controller=Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);
    if t <= t_st(1)
        dxdt=[A1*x(1:6)+B1*qDes(:); qd; qdd];
    elseif t > t_st(1) && t <= t_st(2)
        dxdt=[A2*x(1:6)+B2*qDes(:); qd; qdd];
    else
        dxdt=[A3*x(1:6)+B3*qDes(:); qd; qdd];
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


