saveData = 1;
dataNumber = 2;
clear 
close all
%% ===== CONFIGURATION SECTION =====
numPhases = 3;  % Change this to 2, 3, 4, 5, etc.

%% ===== AUTOMATIC SETUP =====
qDes = [0 0.1987 0.3278];
xTarget(1,:) = [0.0, 0.02, 0.02];
numTargets = size(xTarget, 1);

baseTspan = 5;
baseWn =  [10  2 10 ];

if numPhases <= length(baseTspan)
    tspan = baseTspan(1:numPhases);
    wn = baseWn(1:(3*numPhases));
else
    tspan = [baseTspan, baseTspan(end) + (1:(numPhases-length(baseTspan)))];
    additionalWn = repmat([2.0, 2.0, 2.0], 1, numPhases - length(baseWn)/3);
    wn = [baseWn, additionalWn];
end

[Px, Py, Pz] = FKnew(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

tUni = 0:0.01:tspan(end);

%% ===== Controller Gains to Compare =====
Kp_list = {diag([3 3 3]), diag([1 1 1]), diag([50 50 50])};
Kd_list = {diag([1  1 1]), diag([10 10 10]), diag([60 60 60])};

results = {};

%% ===== Run Simulations =====
for i = 1:length(Kp_list)
    Kp = Kp_list{i};
    Kd = Kd_list{i};
    
    [tSim, ySim] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn, Kp, Kd), ...
                         tUni, zeros(12, 1));
    [Cx, Cy, Cz] = FKnew(ySim(:,7), ySim(:,8), ySim(:,9));
    
    % Store results
    results{i}.t = tSim;
    results{i}.Cy = Cy;
    results{i}.Cz = Cz;
    results{i}.q_actual  = ySim(:,7:9);
    results{i}.qd_actual = ySim(:,10:12);
    results{i}.q_des  = ySim(:,1:3);
    results{i}.qd_des = ySim(:,4:6);
    results{i}.Kp = diag(Kp)';
    results{i}.Kd = diag(Kd)';
    results{i}.error = norm([Cy(end), Cz(end)] - [xFinal(2), xFinal(3)]);
    
    % Compute RMSE for q and qd
    results{i}.rmse_q  = sqrt(mean((results{i}.q_actual - results{i}.q_des).^2));
    results{i}.rmse_qd = sqrt(mean((results{i}.qd_actual - results{i}.qd_des).^2));
end

%% ===== Cartesian YZ Plane Plot =====
figure(1); hold on; grid on;
xlabel('Y (m)'); ylabel('Z (m)');
title(sprintf('Cartesian Trajectory (YZ plane) with %d Targets, %d Phases', numTargets, numPhases));

colors = lines(length(results));   % MATLAB palette
legendEntries = {};

for i = 1:length(results)
    plot(results{i}.Cy, results{i}.Cz, 'Color', colors(i,:), 'LineWidth', 2);
    legendEntries{end+1} = sprintf('Kp=[%d,%d], Kd=[%d,%d], Err=%.4f', ...
        results{i}.Kp(2:3), results{i}.Kd(2:3), results{i}.error);
end

plot(xFinal(2), xFinal(3), 'o', 'MarkerSize', 10, 'Color', [1,0.5,0.05]);
legendEntries{end+1} = 'Final Point';
legend(legendEntries, 'Location', 'northeastoutside');
%% ===== Joint Position Tracking (One Figure per Gain Set) =====
for i = 1:length(results)
    figure(1+i);
    sgtitle(sprintf('Joint Position Tracking - Kp=[%d,%d], Kd=[%d,%d]', ...
        results{i}.Kp(2:3), results{i}.Kd(2:3)));
    for j = 2:3
        subplot(2,1,j-1); hold on; grid on;
        plot(results{i}.t, results{i}.q_actual(:,j), 'Color', colors(i,:), 'LineWidth', 2);
        plot(results{i}.t, results{i}.q_des(:,j), 'k--', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel(sprintf('q%d (rad)', j));
        legend('Actual','Desired','Location','best');
        
        % === Add RMSE as text on the plot ===
        rmse_val = results{i}.rmse_q(j);
        xPos = results{i}.t(end) * 0.7;   % place text ~70% of the time axis
        yPos = mean([min(results{i}.q_actual(:,j)) max(results{i}.q_actual(:,j))]); % mid y
        text(xPos, yPos, sprintf('RMSE = %.4f ', rmse_val), ...
            'FontSize', 10, 'Color', 'r', 'FontWeight','bold');
    end
end

%% ===== Joint Velocity Tracking (One Figure per Gain Set) =====
for i = 1:length(results)
    figure(1+length(results)+i);
    sgtitle(sprintf('Joint Velocity Tracking - Kp=[%d,%d], Kd=[%d,%d]', ...
        results{i}.Kp(2:3), results{i}.Kd(2:3)));
    for j = 2:3
        subplot(2,1,j-1); hold on; grid on;
        plot(results{i}.t, results{i}.qd_actual(:,j), 'Color', colors(i,:), 'LineWidth', 2);
        plot(results{i}.t, results{i}.qd_des(:,j), 'k--', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel(sprintf('dq%d (rad/s)', j));
        legend('Actual','Desired','Location','best');
        
        % === Add RMSE as text on the plot ===
        rmse_val = results{i}.rmse_qd(j);
        xPos = results{i}.t(end) * 0.7;   
        yPos = mean([min(results{i}.qd_actual(:,j)) max(results{i}.qd_actual(:,j))]);
        text(xPos, yPos, sprintf('RMSE = %.4f ', rmse_val), ...
            'FontSize', 10, 'Color', 'r', 'FontWeight','bold');
    end
end


%% ===== Dynamics Function with Prefilter =====
function dxdt = myTwolinkwithprefilter(t, x, qDes, t_st, wn, Kp, Kd)
    zeta = [1 1 1];  % keep damping fixed
    
    % Determine current phase
    phase = 1;
    for i = 1:length(t_st)
        if t > t_st(i)
            phase = i + 1;
        end
    end
    
    % Select wn parameters
    wnPhase = wn((phase-1)*3+1:phase*3);
    
    A = [zeros(3), eye(3); -diag(wnPhase).^2, -2*diag(zeta)*diag(wnPhase)];
    B = [zeros(3); diag(wnPhase).^2];
    
    q = x(7:9);
    qd = x(10:12);
    
    % PD control
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    
    [M,C,G] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau = M*(controller) + C*qd;
    qdd = M\(tau-C*qd);
    
    dxdt = [A*x(1:6) + B*qDes(:); qd; qdd];
end


%% ===== Forward Kinematics (FK) =====
function [x, y, z, valid] = FKnew(q1, q2, q3)
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07;   q2_max = 1.97;
    q3_min = -0.4;    q3_max = 2.92;
    
    l1 = 0.208; 
    l2 = 0.168;
    
    q1_orig = q1; q2_orig = q2; q3_orig = q3;
    
    q1 = max(q1_min, min(q1_max, q1));
    q2 = max(q2_min, min(q2_max, q2));
    q3 = max(q3_min, min(q3_max, q3));
    
    q3_min_rel = max(q3_min, q2 - 0.93);
    q3_max_rel = min(q3_max, q2 + 0.95);
    q3 = max(q3_min_rel, min(q3_max_rel, q3));
    
    valid = true(size(q1));
    valid = valid & (q1_orig >= q1_min & q1_orig <= q1_max);
    valid = valid & (q2_orig >= q2_min & q2_orig <= q2_max);
    valid = valid & (q3_orig >= q3_min & q3_orig <= q3_max);
    valid = valid & (q3_orig >= q3_min_rel & q3_orig <= q3_max_rel);
    
    x = sin(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
    y = l2 - l2 .* cos(q3) + l1 .* sin(q2);
    z = -l1 + cos(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
end


%% ===== Inverse Kinematics (IK) =====
function [Q, valid] = IKnew(x,y,z)
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07; q2_max = 1.97;
    q3_min = -0.4; q3_max = 2.92;
    
    l1=0.208; 
    l2=0.168;
    
    q1=atan2(x,z+l1);
    if q1 < q1_min || q1 > q1_max
        valid = false;
        Q = [q1, 0, 0];
        return;
    end

    R=sqrt(x^2+(z+l1)^2);
    r=sqrt(x^2+(y-l2)^2+(z+l1)^2);
    
    Beta=atan2(y-l2,R);
    Gamma=acos((l1^2+r^2-l2^2)/(2*l1*r));
    q2=Gamma+Beta;

    Alpha=acos((l1^2+l2^2-r^2)/(2*l1*l2));
    q3=q2+Alpha-pi/2;

    Q=[q1,q2,q3];
    
    valid = true;
    if q2 < q2_min || q2 > q2_max, valid = false; end
    if q3 < q3_min || q3 > q3_max, valid = false; end
end



%% ===== COMPUTE M, C, G =====
function [M, C, G] = compute_M_C_G(theta1, theta2,theta3, dtheta1, dtheta2,dtheta3)
    % link lengths
    l1 = 0.208; l2 = 0.168; l3 = 0.0325;
    l5 = -0.0368; l6 = 0.0527;
    g = -9.80665;

    % Masses and inertias (same as before)
    m_a = 0.0202; m_c = 0.0249; m_be = 0.2359; m_df = 0.1906;
    Ia_xx = 0.4864e-4; Ia_yy = 0.001843e-4; Ia_zz = 0.4864e-4;
    Ic_xx = 0.959e-4; Ic_yy = 0.959e-4; Ic_zz = 0.0051e-4;
    Ibe_xx = 11.09e-4; Ibe_yy = 10.06e-4; Ibe_zz = 0.591e-4;
    Idf_xx = 7.11e-4; Idf_yy = 0.629e-4; Idf_zz = 6.246e-4;
    Ibaseyy = 11.87e-4;
    
    % Mass matrix
    M11 = (1/8*(4*Ia_yy + 4*Ia_zz + 8*Ibaseyy + 4*Ibe_yy + 4*Ibe_zz + 4*Ic_yy + 4*Ic_zz + 4*Idf_zz + 4*l1^2*m_a + l2^2*m_a + l1^2*m_c + 4*l3^2*m_c) + ...
           1/8*(4*Ibe_yy - 4*Ibe_zz + 4*Ic_zz + l1^2*(4*m_a+m_c)) * cos(2*theta2) + ...
           1/8*(4*Ia_yy - 4*Ia_zz + 4*Idf_yy - 4*Idf_zz - l2^2*m_a - 4*l3^2*m_c)*cos(2*theta3) + l1*(l2*m_a+l3*m_c)*cos(theta2)*sin(theta3));
    M22 = 1/4*(4*(Ibe_xx + Ic_xx + l1^2*m_a) + l1^2*m_c);
    M23 = -1/2*l1*(l2*m_a+l3*m_c)*sin(theta2-theta3);
    M32 = M23;
    M33 = 1/4*(4*Ia_xx + 4*Idf_xx + l2^2*m_a + 4*l3^2*m_c);
    M = [M11 0 0; 0 M22 M23; 0 M32 M33];
    
    % Coriolis (simplified)
    C = zeros(3);
    
    % Gravity
    N2 = 1/2*g*(2*l1*m_a + 2*l5*m_be + l1*m_c)*cos(theta2);
    N3 = 1/2*g*(l2*m_a + 2*l3*m_c - 2*l6*m_df)*sin(theta3);
    G = [0; N2; N3];
end
