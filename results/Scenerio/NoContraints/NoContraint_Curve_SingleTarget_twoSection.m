%%% Change xMid point
clear; clc;

% Define desired trajectory and Middle Points
qDes = [0.1914, -0.0445, 0.3336];

% xMid = [0.035, 0, 0.015];
xMid = [0.015, 0, 0.035];

qMid = IK(xMid(1),xMid(2),xMid(3));

% Parameters
time = [10 20];  % Time
zeta1 = [1 1 1];
zeta2 = [1 1 1];
wn1 = [1.3 1.1 1.1];      % Prefilter Omega     
wn2 = [1.3 1.1 1.1];
kj1 = [20 20 20];         % Spring constants
kj2 = [20 20 20];         % Spring constants
bj1 = [5 5 5];            % Damping constants
bj2 = [5 5 5];            % Damping constants
wt = [10, 1e-4, 400];     % Weights [qDes, Time, qMid]
% wt = [10, 1e-4, 500];   % Weights [qDes, Time, qMid]

% Optimization setup
initParams = [time zeta1 zeta2  wn1 wn2 kj1 kj2 bj1 bj2]; % Initial guess

[init_T, init_Y] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, time,  zeta1,zeta2, wn1,wn2, kj1, kj2, bj1, bj2), [0 time(2)], zeros(12, 1));
[xInit, yInit, zInit] = FK(init_Y(:,7), init_Y(:,8), init_Y(:,9));
[xD, yD, zD] = FK(init_Y(:,1), init_Y(:,2), init_Y(:,3));

% plot(xInit,zInit,'+',xD,zD,'.')
% Lower and upper boundaries 
lb = [2   5.1   0.5 0.8 0.6  0.5 0.8 0.6  1  1  1  1  1  1    70 70 70    70  70  70   10 10 10   10 10 10  ];   
ub = [5   10  1   1   1    1   1   1    10 10 10 10 10 10   150 150 150 150 150 150  40 40 40   40 40 40  ];  

% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xMid);

% Run optimization
options = optimset('Display', 'iter', 'TolFun', 1e-6, 'MaxIter', 400);
optimalParams = fmincon(objectiveFunc, initParams, [], [], [], [], lb, ub, [], options);

% Simulate with optimal parameters
[t, y] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, optimalParams(1:2), optimalParams(3:5),   optimalParams(6:8),   ...
                                                                               optimalParams(9:11),  optimalParams(12:14), ...
                                                                               optimalParams(15:17), optimalParams(18:20), ...
                                                                               optimalParams(21:23), optimalParams(24:26)), ...
                                                                               [0 optimalParams(2)], zeros(12, 1));

% Output
[xAct, yAct, zAct] = FK(y(:,7), y(:,8), y(:,9));
[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
% torque = bj .* (y(10:12) - y(4:6)) + Kj .* (y(7:9) - y(1:3));
% Plotting
figure; hold on; grid on;
plot(xInit, zInit, '-.');
plot(xAct, zAct, '.');
plot(xDes, zDes, 'o');
plot(xMid(1),xMid(3),'*')
xlabel('X axis'); ylabel('Y axis');
legend('Initial', 'Optimized', 'Desired')
title('Cartesian Trajectory Tracking');




disp(['Optimized Parameters: ', num2str(optimalParams)]);

% Objective function
function error = objectiveFunction(params, qDes, wt, xMid)
    x0 = zeros(12, 1);
    x0(1:3) = qDes; 

    % Simulate the system
    [t, y] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, params(1:2),  params(3:5),   params(6:8),   ...
                                                                             params(9:11),  params(12:14), ...
                                                                             params(15:17), params(18:20), ...
                                                                             params(21:23), params(24:26)), [0 params(2)], x0);
    [xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
    xDes = [xDes, yDes, zDes];
    [xAct, yAct, zAct] = FK(y(:,7), y(:,8), y(:,9));
    xAct = [xAct, yAct, zAct];
    
    
    % Calculate error metric
    distEnd = min(sum((xAct - xDes ).^2, 2) + sum((params(2) - t).^2, 2)); 

    % distMid = sum(arrayfun(@(i) min(sum((xAct - xMid(i, :)).^2, 2)), 1:size(xMid,1)));
    % distMid = min(sum(  (xAct - xMid(1, :)).^2  , 2) + ((t - params(1)).^2)  );
    distMid = min(sum(  (xAct - xMid(1, :)).^2  , 2)  );
    
    error = wt(1) * distEnd   +  ...
            wt(2) * params(2) + ... 
            wt(3) * distMid;
end

% myTwolinkwithprefilter function
function dxdt= myTwolinkwithprefilter(t, x, qDes,time, zeta1,zeta2,wn1, wn2, kj1,kj2,bj1,bj2)
    
    A1 = [zeros(3,3) eye(3);
        -eye(3)*diag(wn1).^2  -eye(3)*2*diag(zeta1)*diag(wn1)];
    B1 = [zeros(3,3); diag(wn1).^2];
    
    A2 = [zeros(3,3) eye(3);
        -eye(3)*diag(wn2).^2  -eye(3)*2*diag(zeta2)*diag(wn2)];
    B2 = [zeros(3,3); diag(wn2).^2];
    
    q   = x(7:9);
    qd  = x(10:12);
   
    Kp1 = diag(kj1);  
    Kd1 = diag(bj1);  
    
    Kp2 = diag(kj2);  
    Kd2 = diag(bj2);  
    
    controller1 = Kp1 * (x(1:3) - q) + Kd1 * (x(4:6) - qd);
    controller2 = Kp2 * (x(1:3) - q) + Kd2 * (x(4:6) - qd);

    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
    % torque = Kd * (x(4:6) - qd) + Kp * (x(1:3) - q);
    tau1 = M * (controller1) + C * qd ;
    tau2 = M * (controller2) + C * qd ;

    qdd1 = M \ (tau1 - C * qd );
    qdd2 = M \ (tau2 - C * qd );

    % qdd = M \ ( torque - C * qd);
    if t < time(1)
        dxdt = [A1*x(1:6) + B1*qDes(:); qd; qdd1];
    else
        dxdt = [A2*x(1:6) + B2*qDes(:); qd; qdd2];
    end
end

function [x, y, z] = FK(q1, q2, q3)
    l1 = 0.208; 
    l2 = 0.168;  
    x = sin(q1) .* (l1 * cos(q2) + l2 * sin(q3));
    y = l2 - l2 * cos(q3) + l1 * sin(q2);
    z = -l1 + cos(q1) .* (l1 * cos(q2) + l2 * sin(q3));
end

function [q1, q2, q3] = IK(x, y, z)
    l1 = 0.208; 
    l2 = 0.168;  
    q1 = atan2(x, z + l1);

    R = sqrt(x^2 + (z + l1)^2);
    r = sqrt(x^2 + (y - l2)^2 + (z + l1)^2);
    Beta  = atan2(y - l2, R);
    Gamma = acos((l1^2 + r^2 - l2^2) / (2 * l1 * r));
    q2 = Gamma + Beta;

    Alpha = acos((l1^2 + l2^2 - r^2) / (2 * l1 * l2));
    q3 = q2 + Alpha - pi/2;
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





