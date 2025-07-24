clear; clc;
close all;

qDes = [ 0   0.198678167676855   0.327814256075948 ];
[Px, Py, Pz] = FK(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

xCtrl = zeros(2,3);
qCtrl = zeros(2,3);

% Parameters
tspan =  1.7575;
wn1 = [ 5.4914      4.6608];
wn2 = [ 9.7524      5.0223 ];
wn3 = [ 4.8855      5.7357 ];

Gain = 5;
tspan1 =  tspan/ Gain;
wn11 = wn1*Gain;
wn21 = wn2*Gain;
wn31 = wn3*Gain;


xCtrl(1,:) =  [0 0.014382   0.0045277];
xCtrl(2,:) = [0 0.035209    0.028134];



qCtrl(1,:) = IK(xCtrl(1,1), xCtrl(1,2), xCtrl(1,3));
qCtrl(2,:) = IK(xCtrl(2,1), xCtrl(2,2), xCtrl(2,3));

qDes =[qCtrl; qDes];


t_uniform = 0:0.001:tspan;
t_uniform1 = 0:0.001:tspan1;


% Initial Condition
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  wn1,wn2,wn3,xCtrl(1,2:3),xCtrl(2,2:3)), t_uniform, zeros(12, 1));
[tOut1, yOut1] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan1,  wn11,wn21,wn31,xCtrl(1,2:3),xCtrl(2,2:3)), t_uniform1, zeros(12, 1));

% Forward Kinematics
[CxInit, CyInit, CzInit] = FK(yInit(:,7), yInit(:,8), yInit(:,9)); % Initial Trajectory
[CxOpt, CyOpt, CzOpt] = FK(yOut1(:,7), yOut1(:,8), yOut1(:,9)); % Optimized Trajectory



dataNum = 18;  % Change this to 2, 3, etc. for other runs

r = 0.01; % Radius
theta = linspace(0, 2*pi, 100); % Angle for circle

x_circle1 = xCtrl(1,2) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle1 = xCtrl(1,3) + r*sin(theta); % y = center_y + r*sin(θ)
x_circle2 = xCtrl(2,3) + r*cos(theta); % x = center_x + r*cos(θ)
y_circle2 = xCtrl(2,3) + r*sin(theta); % y = center_y + r*sin(θ)
% Recompute distances to control points
ctrl1 = xCtrl(1,:);
ctrl2 = xCtrl(2,:);
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
plot(CyInit, CzInit, '.-')
plot(CyOpt, CzOpt, '.-')
plot(xFinal(2), xFinal(3), 'o')
plot(xCtrl(1,2), xCtrl(1,3), 'd')
plot(xCtrl(2,2), xCtrl(2,2), 'd')
plot(x_circle1, y_circle1, 'b--', 'LineWidth', 1.5);
plot(x_circle2, y_circle2, 'b--', 'LineWidth', 1.5);

legend('Optimized Trajectory','Target Point 1','Target Point 2','End Point','Control Point 1','Control Point 2')
xlabel('Y axis (m)')
ylabel('Z axis (m)')
title('Cartesian Space Trajectory Results with Phase Changes')


save(sprintf('Gdata%d.mat', dataNum), ...
   'tOut1', 'yOut1');

 

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

