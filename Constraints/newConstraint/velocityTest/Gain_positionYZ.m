% ========== Setup ==========
% Desired Joint Angles
qFinal = [0, 0.198678167676855, 0.327814256075948];
[xFinal, yFinal, zFinal] = FK(qFinal(1), qFinal(2), qFinal(3));
xDes = [xFinal, yFinal, zFinal];

% Mid Targets for Visualization
xMid = [0, 0.015, 0.01;
        0, 0.025, 0.03;
        0, 0.035, 0.045];

% Control Point
CtrlPnt = [0.010883, 0.029955, 0.010116];

% Gains to simulate
gains = [0.2, 0.7, 1, 1.5, 2, 5];
colors = lines(length(gains)); % Color for each trajectory

figure(1); hold on; grid on;
plot(0,0,'ko','DisplayName','Start Point');
plot(xDes(2),xDes(3),'ro','DisplayName','End Point');

for i = 1:3
    plot(xMid(i,2), xMid(i,3), 'g*', 'DisplayName', ['Target Point ' num2str(i)]);
end
plot(CtrlPnt(2), CtrlPnt(3), 'md', 'DisplayName', 'Control Point');

% Simulate for each Gain
for i = 1:length(gains)
    [yi_plot, zi] = simulateTrajectory(gains(i), qFinal, CtrlPnt);
    plot(yi_plot, zi, 'Color', colors(i,:), 'DisplayName', ['Trajectory (Gain = ' num2str(gains(i)) ')']);
end

xlabel('Y Axis');
ylabel('Z Axis');
title('Trajectories under Varying Gains');
legend show;



% Velocity Plots
figure(2); clf; hold on; grid on;
colors = lines(6); % To distinguish each gain

legend_entries = {};

for i = 1:length(gains)
    Gain = gains(i);
    tspan = [0.57963 1.1805] / Gain;
    wn1 = [0.51304 1.7148 1.0131] * Gain;
    wn2 = [8.7237 6.7598 9.4406] * Gain;
    CtrlPnt = [0.010883 0.029955 0.010116];
    qCtrl = IK(CtrlPnt(1), CtrlPnt(2), CtrlPnt(3));
    qDes = [qCtrl; 0 0.198678167676855 0.327814256075948];
    t_uniform = 0:0.01:tspan(2);
    [~, yi] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn1, wn2, CtrlPnt), t_uniform, zeros(12, 1));
    
    % Plot joint velocities
    for j = 1:3
        subplot(3, 1, j); hold on; grid on;
        plot(t_uniform, yi(:, 9 + j), 'Color', colors(i,:), 'LineWidth', 1.5);
        ylabel(['q' num2str(j) ' velocity (rad/s)']);
    end
    legend_entries{end+1} = ['Gain = ' num2str(Gain)];
end

subplot(3, 1, 3);
xlabel('Time (s)');
legend(legend_entries, 'Location', 'bestoutside');
sgtitle('Joint Velocities for Different Gains');



% ========== Functions ==========

function [yPlot, zPlot] = simulateTrajectory(Gain, qFinal, CtrlPnt)
    tspan = [0.57963, 1.1805] / Gain;
    wn1 = [0.51304, 1.7148, 1.0131] * Gain;
    wn2 = [8.7237, 6.7598, 9.4406] * Gain;

    qCtrl = IK(CtrlPnt(1), CtrlPnt(2), CtrlPnt(3));
    qDes = [qCtrl; qFinal];
    t_uniform = 0:0.01:tspan(2);

    [~, yi] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn1, wn2, CtrlPnt), t_uniform, zeros(12, 1));
    [~, yPlot, zPlot] = FK(yi(:,7), yi(:,8), yi(:,9));
end

function dxdt = myTwolinkwithprefilter(t, x, qDes, t_st, wn1, wn2, ctrlPnt)
    zeta = [1 1 1];
    A1 = [zeros(3), eye(3); -diag(wn1).^2, -2*diag(zeta).*diag(wn1)];
    B1 = [zeros(3); diag(wn1).^2];
    A2 = [zeros(3), eye(3); -diag(wn2).^2, -2*diag(zeta).*diag(wn2)];
    B2 = [zeros(3); diag(wn2).^2];

    qCtrl = IK(ctrlPnt(1), ctrlPnt(2), ctrlPnt(3));
    q = x(7:9);
    qd = x(10:12);

    Kp = diag([70 70 70]);
    Kd = diag([20 20 20]);
    u = Kp*(x(1:3) - q) + Kd*(x(4:6) - qd);
    [M, C, ~] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));
    qdd = M \ (M*u + C*qd - C*qd); % simplify torque computation

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


