close all
clear

saveData = 1;   % Set to 1 if you want to save figures
dataNumber = 2; % Used in filenames

%% ===== CONFIGURATION SECTION =====
numPhases = 3;  % Change this to 2, 3, 4, 5, etc.

% Fixed natural frequencies
wn = [1 2 4];

% List of damping ratio (zeta) cases
zetaCases = {
    [0.1 0.3 0.2];
    [0.3 0.2 0.5];
    [0.4 1 1 ]
    [0.5 3 5];
};

%% ===== AUTOMATIC SETUP =====
qDes = [0, 0.1987, 0.3278];
xTarget(1,:) = [0.0,0.0,0.02];
numTargets = size(xTarget, 1);

%% ===== PLOTTING (YZ plane with all cases) =====
figure(1); hold on; grid on;
xlabel('Y (m)'); ylabel('Z (m)');
title('Trajectories for Different Damping Ratios');

colors = lines(length(zetaCases)); % distinct colors
legendEntries = {};

baseTspan = 5;
tspan = repmat(baseTspan,1,numPhases);

for c = 1:length(zetaCases)
    zeta = zetaCases{c};  % Current damping ratio

    % Compute final position for plotting
    [Px, Py, Pz] = FKnew(qDes(1), qDes(2), qDes(3));
    xFinal = [Px, Py, Pz];

    % Run simulation
    tUni = 0:0.01:tspan(end);
    [~, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn, zeta), ...
                       tUni, zeros(12, 1));

    % Extract trajectory
    [~, CyInit, CzInit] = FKnew(yInit(:,7), yInit(:,8), yInit(:,9));
    figure(1); hold on; grid on;

    % Plot trajectory (YZ)
    plot(CyInit, CzInit, 'LineWidth', 1.5, 'Color', colors(c,:));
    legendEntries{end+1} = ['\zeta_{' num2str(c) '} = [' num2str(zeta(2)) ', ' num2str(zeta(3)) ']'];

    %% ===== JOINT POSITION PLOTTING =====
    q1 = yInit(:,7);
    q2 = yInit(:,8);
    q3 = yInit(:,9);
    
    figure(2); hold on; grid on;
    subplot(2,1,1); hold on; grid on;
    plot(tUni, q2, 'Color', colors(c,:), 'LineWidth', 1.2);
    ylabel('q2 (rad)');
    
    subplot(2,1,2); hold on; grid on;
    plot(tUni, q3, 'Color', colors(c,:), 'LineWidth', 1.2);
    ylabel('q3 (rad)'); xlabel('Time (s)');
end

%% ===== FINAL POINTS AND LEGENDS =====
palette = lines(3);  
cTarget = palette(1,:); 
cFinal  = palette(2,:); 
cStart  = palette(3,:); 

% Plot starting point
figure(1); hold on; grid on;
plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', cStart, 'MarkerEdgeColor', 'k');

% Plot final point
plot(xFinal(2), xFinal(3), 's', 'MarkerSize', 10, 'MarkerFaceColor', cFinal, 'MarkerEdgeColor', 'k');

legendEntries{end+1} = 'Starting Point';
legendEntries{end+1} = 'Final Point';
legendEntries{end+1} = 'Desired Joint Position';

legend(legendEntries, 'Location', 'northeastoutside');

t = linspace(0,2,500);   % Example: from 0 to 2 sec

% Add reference lines to joint plots
figure(2); hold on; grid on;
subplot(2,1,1); yline(qDes(2), '--k', 'LineWidth', 2);
stairs(t, [0; qDes(2)*ones(length(t)-1,1)], '--k', 'LineWidth', 2);
xlim([0 5])
title('Joint 2 Position')

subplot(2,1,2); yline(qDes(3), '--k', 'LineWidth', 2);
stairs(t, [0 ; qDes(3)*ones(length(t)-1,1)], '--k', 'LineWidth', 2);
xlim([0 5])
title('Joint 3 Position')
sgtitle('Joint Position for Different Damping Ratios', 'FontWeight', 'bold', 'FontSize', 12);
legend(legendEntries([1:4,6]), 'Location', 'northeastoutside');

%% ===== DYNAMICS FUNCTION WITH PREFILTER =====
function dxdt = myTwolinkwithprefilter(t, x, qDes, t_st, wn, zeta)
    % Determine current phase based on time
    phase = 1;
    for i = 1:length(t_st)
        if t > t_st(i)
            phase = i + 1;
        end
    end
    
    % Select wn for current phase
    wnPhase = wn((phase-1)*3+1:phase*3);
    
    A = [zeros(3), eye(3); -diag(wnPhase).^2, -2*diag(zeta).*diag(wnPhase)];
    B = [zeros(3); diag(wnPhase).^2];
    
    q = x(7:9);
    qd = x(10:12);
    
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  
    
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    
    [M,C,~] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau = M*(controller) + C*qd;
    qdd = M\(tau-C*qd);
    
    dxdt = [A*x(1:6) + B*qDes(:); qd; qdd];
end

%% ===== FORWARD KINEMATICS =====
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
