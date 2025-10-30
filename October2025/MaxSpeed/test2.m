clear; clc; close all;
dataNumber = 8;

%%%%% Configuration - Choose control point indices %%%%%
indexControlPoint = [1, 3];   % choose directly from xTarget rows

%%%%% Optimal Parameters %%%%%
qDes = [0, 0, 0];
[xStart, y, z] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xStart, y, z];

xTarget = [ -0.025, -0.02, 0;
            0.0,    -0.04,  0;
            0.025,  -0.02, 0 ];

Gain = 2;
numControlPoints = numel(indexControlPoint);
tOpt = 8.73914910024956/Gain;

% Initialize wn parameters
numWnSets = numControlPoints + 1;
wn = cell(numWnSets, 1);
for i = 1:numWnSets
    wn{i} = [1 1 1];mat
end



% Optimized parameters
Opt = [tOpt, ...
       0.54265465869543 0.74500245479823 0.4041549846575, ...
       1.291549580544353   0.588600790350165   0.899720271351260, ...
       0.530407636217632  1.119714064852254   2.085093134460414, ...
       -0.100792879495735, -0.041846054140405, 0.0024602513732, ...
       0.08369453639168,   -0.1199777273287158, -0.001215467863];




% Control points (from optimization result)
ctrlPoints = [ Opt(11),Opt(12),Opt(13);Opt(14),Opt(15),Opt(16)];

% Convert to joint space
qCtrl = zeros(numControlPoints, 3);
for i = 1:numControlPoints
    qCtrl(i,:) = IK(ctrlPoints(i,1), ctrlPoints(i,2), ctrlPoints(i,3));
end

% Combine joint configurations
qDes = [qCtrl; qDes];

% Time vector
t_uniform = 0:0.01:tOpt;



% Extract optimized parameters
wnOpt = cell(numWnSets, 1);
ctrlPnt = zeros(numControlPoints, 3);
paramIdx = 2;
for i = 1:numWnSets
    wnOpt{i} = Gain*Opt(paramIdx:paramIdx+2);
    paramIdx = paramIdx + 3;
end
for i = 1:numControlPoints
    ctrlPnt(i,:) = Opt(paramIdx:paramIdx+2);
    paramIdx = paramIdx + 3;
end

% Simulate optimized
[tOpt, yOpt] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tOpt, wnOpt, ctrlPnt), ...
                      t_uniform, zeros(12, 1));

% Switching times
t_Vmax = cell(numWnSets-1, 1);
for i = 1:numWnSets-1
    t_Vmax{i} = 1./wnOpt{i};
end

% Forward Kinematics
[CxOpt, CyOpt, CzOpt] = FK(yOpt(:,7), yOpt(:,8), yOpt(:,9));
[CxDes, CyDes, CzDes] = FK(yOpt(:,1), yOpt(:,2), yOpt(:,3));

%% ------------ 3D Cartesian plotting ------------
col_init = [0.70 0.70 0.70];
col_opt  = [0.20 0.60 1.00];
col_tgt  = [1.00 0.55 0.10];
col_ctrl = [0.10 0.70 0.10];
col_end  = [0.85 0.10 0.10];
col_start= [0.10 0.70 0.70];

xStart= [0,0,0];
xFinal   = xDes;

figure('Name','Cartesian Space Trajectory (3D)');
hold on; grid on; axis equal
view(45,25);


% Optimized trajectory (3D)
plot3(CxOpt, CyOpt, CzOpt, '-',  'LineWidth', 2.0, 'Color', col_opt,  'DisplayName','Optimized Trajectory');

% Targets
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', col_tgt, 'MarkerEdgeColor','k', 'DisplayName','Target Points');

% Optimized control points
for i = 1:numControlPoints
    plot3(ctrlPnt(i,1), ctrlPnt(i,2), ctrlPnt(i,3), ...
        'd', 'MarkerSize', 9, 'MarkerFaceColor', col_ctrl, 'MarkerEdgeColor','k', ...
        'DisplayName', sprintf('Control Point %d', i));
end

% Start & End
plot3(xStart(1), xStart(2), xStart(3), 'o', 'MarkerSize', 9, ...
      'MarkerFaceColor', col_start, 'MarkerEdgeColor','k', 'DisplayName','Start');
plot3(xFinal(1), xFinal(2), xFinal(3), 's', 'MarkerSize', 10, ...
      'MarkerFaceColor', col_end, 'MarkerEdgeColor','k', 'DisplayName','End');

% Connect control points visually
if numControlPoints >= 2
    plot3(ctrlPnt(:,1), ctrlPnt(:,2), ctrlPnt(:,3), ...
          ':', 'LineWidth', 1.0, 'Color', [0.3 0.8 0.3], 'HandleVisibility','off');
end

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('Cartesian Space Trajectory Results (data %d)', dataNumber));
legend('Location','bestoutside');

% ----------- Fixed axis limits -----------
xlim([-0.3 0.2]);
ylim([-0.3 0.2]);
zlim([-0.3 0.2]);
% axis vis3d;  % keeps equal scaling

disp(['Optimal Parameter: ', mat2str(Opt,4)]);

save('Sdata8.mat', 'tOpt', 'wnOpt', 'xTarget', 'yOpt', 'ctrlPoints','xFinal','xStart');

%% ------------ Joint Position Plot ------------
t = tOpt;
q_des  = yOpt(:,1:3);
q_act  = yOpt(:,7:9);
colors = lines(3);

figure('Name','Joint Positions','Color','w');
hold on; grid on;
for j = 1:3
    plot(t, q_des(:,j), '--', 'LineWidth', 1.5, 'Color', colors(j,:), 'DisplayName', sprintf('q%d Desired', j));
    plot(t, q_act(:,j),  '-',  'LineWidth', 2.0, 'Color', colors(j,:), 'DisplayName', sprintf('q%d Actual', j));
    for stg = 1:numel(t_Vmax)
        xline(t_Vmax{stg}(j), '--k', 'HandleVisibility','off');
    end
end
xlabel('Time (s)');
ylabel('Joint Position (rad)');
title('Joint Positions: Desired vs Actual');
legend('Location','best');

%% ------------ Joint Velocity Plot ------------
qd_des = yOpt(:,4:6);
dq_act = yOpt(:,10:12);

figure('Name','Joint Velocities','Color','w');
hold on; grid on;
for j = 1:3
    plot(t, qd_des(:,j), '--', 'LineWidth', 1.5, 'Color', colors(j,:), 'DisplayName', sprintf('dq%d Desired', j));
    plot(t, dq_act(:,j),  '-',  'LineWidth', 2.0, 'Color', colors(j,:), 'DisplayName', sprintf('dq%d Actual', j));
    for stg = 1:numel(t_Vmax)
        xline(t_Vmax{stg}(j), '--k', 'HandleVisibility','off');
    end
end
xlabel('Time (s)');
ylabel('Joint Velocity (rad/s)');
title('Joint Velocities: Desired vs Actual');
legend('Location','best');



%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%

% Helper function to extract parameters
function [wn, ctrlPoints] = extractParameters(prms, numControlPoints)
    numWnSets = numControlPoints + 1;
    
    wn = cell(numWnSets, 1);
    ctrlPoints = zeros(numControlPoints, 3);
    
    paramIdx = 2;
    for i = 1:numWnSets
        wn{i} = prms(paramIdx:paramIdx+2);
        paramIdx = paramIdx + 3;
    end
    for i = 1:numControlPoints
        ctrlPoints(i,:) = prms(paramIdx:paramIdx+2);
        paramIdx = paramIdx + 3;
    end
end

function dxdt = myTwolinkwithprefilter(t, x, qDes, tspan, wn, ctrlPoints)
    zeta = [1 1 1];  % Damping ratio
    numControlPoints = size(ctrlPoints, 1);
    numWnSets = numControlPoints + 1;

    % Compute per-joint switching times
    t_Vmax = cell(numWnSets-1, 1);
    t_Vmax{1} = 1 ./ wn{1};
    for i = 2:numWnSets-1
        t_Vmax{i} = t_Vmax{i-1} + 1 ./ wn{i};
    end

    % Get control point joint angles via IK
    qCtrl = zeros(numControlPoints, 3);
    for i = 1:numControlPoints
        qCtrl(i,:) = IK(ctrlPoints(i,1), ctrlPoints(i,2), ctrlPoints(i,3));
    end

    % Initialize natural frequency and control target for each joint
    wn_current = zeros(1, 3);
    qControl = zeros(1, 3);

    % Per-joint switching logic based on t_Vmax
    for i = 1:3
        if t <= t_Vmax{1}(i)
            wn_current(i) = wn{1}(i);
            qControl(i) = qCtrl(1, i);
        else
            % Find which stage we're in
            stage = 1;
            for j = 1:numWnSets-1
                if t > t_Vmax{j}(i)
                    stage = j + 1;
                end
            end
            
            wn_current(i) = wn{stage}(i);
            if stage <= numControlPoints
                qControl(i) = qCtrl(stage, i);
            else
                qControl(i) = qDes(end, i);  % Final desired joint angle
            end
        end
    end

    % Construct system matrices for prefilter dynamics
    A = [zeros(3), eye(3); -diag(wn_current).^2, -2 * diag(zeta) * diag(wn_current)];
    B = [zeros(3); diag(wn_current).^2];

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


