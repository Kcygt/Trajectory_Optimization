clear; clc;
close all;

%%%%% Optimal Parameters %%%%%
% Result 1:
% tspan = [ 0.61 ];
% wn1 =  [ 11.5504      12.1778       11.316 ];
% wn2 =  [ 8.53909      11.6646      10.9705 ];
% CtrlPnt = [   0    0.045368     0.01003 ];

qDes = [ 0   0.198678167676855   0.327814256075948 ];
[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xDes, yDes, zDes];

xTarget = zeros(3,3);
xTarget(1,:) = [0, 0.015, 0.005];
xTarget(2,:) = [0, 0.025, 0.015];
xTarget(3,:) = [0, 0.035, 0.025];


% Parameters
tspan =  0.61;
wn1 =  [ 11.5504      12.1778       11.316 ];
wn2 =  [ 8.53909      11.6646      10.9705 ];
CtrlPnt = [   0    0.045368     0.01003 ];
qCtrl = IK(CtrlPnt(1), CtrlPnt(2), CtrlPnt(3));
qFinal =[qCtrl;qDes];
t_uniform = 0:0.01:tspan;


% GAIN 
gain = 5;
tspanG =  gain * 0.61;
wn1G =  gain * [ 11.5504      12.1778       11.316 ];
wn2G =  gain * [ 8.53909      11.6646      10.9705 ];
t_uniformG = 0:0.01:tspanG;

% Initial Condition
[tt, yy] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qFinal, tspan,  wn1,wn2,CtrlPnt), t_uniform, zeros(12, 1));
[ttG, yyG] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qFinal, tspanG,  wn1G,wn2G,CtrlPnt), t_uniformG, zeros(12, 1));


% Forward Kinematics
[xInit, yInit, zInit] = FK(yy(:,7), yy(:,8), yy(:,9));     % Initial Trajectory
[xG, yG, zG] = FK(yyG(:,7), yyG(:,8), yyG(:,9));     % Initial Trajectory

t_Vmax = 1./wn1;
t_VmaxG = 1./wn1G;


%%% Plotting
figure; hold on; grid on;
plot(yInit, zInit,'--')
plot(yG, zG,'--')

plot(xTarget(1,2),xTarget(1,3),'*')
plot(xTarget(2,2),xTarget(2,3),'*')
plot(xTarget(3,2),xTarget(3,3),'*')
plot(xDes(2),xDes(3),'o')
plot(CtrlPnt(2),CtrlPnt(3),'d')
legend('Initial Trajectory','Gain','Target Point 1','Target Point 2','Target Point 3','End Point','Control Point')
xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')


% Velocity Plot
figure; hold on; grid on;
plot(tt, yy(:,10:12))
plot(ttG, yyG(:,10:12))

xlabel('Time (s)')
ylabel('Velocity (rad/s)')
title('Velocity')
% Add vertical dashed lines at t_Vmax times (excluding index 1)
for k = 2:length(t_Vmax)
    xline(t_Vmax(k), '--k');
end

% Create legend with numeric time values
legend( ...
    'Actual Joint 1', ...
    'Actual Joint 2', ...
    'Actual Joint 3', ...
    'Gain Actual Joint 1', ...
    'Gain Actual Joint 2', ...
    'Gain Actual Joint 3', ...
    sprintf('Time switching joint 2 = %.4f s', t_Vmax(2)), ...
    sprintf('Time switching joint 3 = %.4f s', t_Vmax(3)) ...
);
%%%%%%%%%%%% FUNCTION %%%%%%%%%%%%%

% 
function dxdt = myTwolinkwithprefilter(t, x,qDes,tspan , wn1, wn2, ctrlPnt)
    zeta = [1 1 1];  % Damping ratio

    % Compute per-joint switching times
    t_Vmax = 1 ./ wn1;

    % Get initial control point joint angles via IK
    qCtrl = IK(ctrlPnt(1), ctrlPnt(2), ctrlPnt(3));

    % Initialize natural frequency and control target for each joint
    wn = zeros(1, 3);
    qControl = zeros(1, 3);

    % Per-joint switching logic based on t_Vmax
    for i = 1:3
        if t <= t_Vmax(i)
            wn(i) = wn1(i);
            qControl(i) = qCtrl(i);
        else
            wn(i) = wn2(i);
            qControl(i) = qDes(2, i);  % Desired joint angle after switch
        end
    end

    % Construct system matrices for prefilter dynamics
    A = [zeros(3), eye(3); -diag(wn).^2, -2 * diag(zeta) * diag(wn)];
    B = [zeros(3); diag(wn).^2];

    % Extract actual joint state
    q  = x(7:9);     % Joint angles
    qd = x(10:12);   % Joint velocities

    % PD control gains
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  

    % Control law using prefiltered desired joint values
    controller = Kp * (x(1:3) - q) + Kd * (x(4:6) - qd);

    % Dynamics matrices
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));

    % Compute torque and acceleration
    tau = M * controller + C * qd;
    qdd = M \ (tau - C * qd);

    % Return state derivative
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


