clear; clc;
close all;
%%%
% Time   0.4579    1.4884    7.4918 
% Wn 1 = 2.6269    0.9641    0.9804    
% Wn 2 = 1.4005    2.9646    5.9315
% Wn 3 = 1.0418    0.7773    2.9987
%%%
% Define desired trajectory and Middle Points
qDes = [0.1914, -0.0445, 0.3336];
[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xDes, yDes, zDes];

xMid = zeros(2,3);
qMid = zeros(2,3);
xMid(1,:) = [0.02, 0, 0.01];
xMid(2,:) = [0.03, 0, 0.04];

qMid(1,:) = IK(xMid(1,1), xMid(1,2), xMid(1,3));
qMid(2,:) = IK(xMid(2,1), xMid(2,2), xMid(2,3));


tspanI = [1 2 10];
wnI = [1 1 1  1 1 1  1 1 1];

tspanOpt = [0.4579    1.4884    7.4918 ];
wnOpt = [2.6269    0.9641    0.9804  1.4005    2.9646    5.9315 1.0418    0.7773    2.9987];

K1 = 1.5;
tspanGain1 = tspanOpt / K1;
wnOptGain1 = wnOpt * K1;

K2 = 2;
tspanGain2 = tspanOpt / K2;
wnOptGain2 = wnOpt * K2;

K3 = 4;
tspanGain3 = tspanOpt * K3;
wnOptGain3 = wnOpt * K3;

K4 = 6;
tspanGain4 = tspanOpt / K4;
wnOptGain4 = wnOpt * K4;


% Initial Condition
[tI, OutI] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanI, wnI), [0 tspanI(end)], zeros(12, 1));
[tOpt, OutOpt] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanOpt, wnOpt), [0 tspanOpt(end)], zeros(12, 1));
[tGain1, OutGain1] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanGain1, wnOptGain1), [0 tspanGain1(end)], zeros(12, 1));
[tGain2, OutGain2] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanGain2, wnOptGain2), [0 tspanGain2(end)], zeros(12, 1));
[tGain3, OutGain3] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanGain3, wnOptGain3), [0 tspanGain3(end)], zeros(12, 1));
[tGain4, OutGain4] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspanGain4, wnOptGain4), [0 tspanGain4(end)], zeros(12, 1));


[xI,yI,zI] = FK(OutI(:,7),OutI(:,8),OutI(:,9));
[xOpt,yOpt,zOpt] = FK(OutOpt(:,7),OutOpt(:,8),OutOpt(:,9));
[xGain1,yGain1,zGain1] = FK(OutGain1(:,7),OutGain1(:,8),OutGain1(:,9));
[xGain2,yGain2,zGain2] = FK(OutGain2(:,7),OutGain2(:,8),OutGain2(:,9));
[xGain3,yGain3,zGain3] = FK(OutGain3(:,7),OutGain3(:,8),OutGain3(:,9));
[xGain4,yGain4,zGain4] = FK(OutGain4(:,7),OutGain4(:,8),OutGain4(:,9));



figure; hold on; grid on;
plot(xI,zI)
plot(xOpt,zOpt)
plot(xGain1,zGain1)
plot(xGain2,zGain2)
plot(xGain3,zGain3)
plot(xGain4,zGain4)

plot(xMid(1,1),xMid(1,3),'*')
plot(xMid(2,1),xMid(2,3),'*')
plot(xDes(1),xDes(3),'o')

xlabel('X axis (m)')
ylabel('Y axis (m)')
title('Cartesian Space Trajectory Results')
legend('Initial Trajectory','Optimized Trajectory', ...
        'Gain1 = 1.5',  'Gain1 = 2', 'Gain1 = 4', 'Gain1 = 6',...
          'Target point1','Target Point 2','Endpoint')



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
    Kd=diag([20 20 20]);  

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


