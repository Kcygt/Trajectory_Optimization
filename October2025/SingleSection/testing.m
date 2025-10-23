% clear; clc;
% close all;

saveData = 1; % 0 dont save data
dataNumber = 2;
Gain=4;
%% ===== CONFIGURATION SECTION =====

% Define number of phases (this determines tspan and wn)
numPhases = 1;  % Change this to 2, 3, 4, 5, etc.

%% ===== AUTOMATIC SETUP (Don't modify below this line) =====
qDes = [   0.3139    0.2332    0.8081]; % [0.1, 0.1, 0.1]

% Case 1
% qTarget = [ 0.3120  0.1945    0.3430];

% 
% % Case 2
qTarget = [0.0745, 0.2254, 0.4038];


% Case 3
% qTarget = [         0.1815    0.1412    0.0936;
%                       0.2751    0.1992    0.3260];


% Preallocate result
xTarget = zeros(size(qTarget,1),3 );  % 3 rows (x,y,z), N columns

for i = 1:size(qTarget, 1)
    q1 = qTarget(i, 1);
    q2 = qTarget(i, 2);
    q3 = qTarget(i, 3);
    [x, y, z] = FK(q1, q2, q3);
    xTarget(i,:) = [x; y; z];
end


% Get number of targets
numTargets = size(xTarget, 1);


% Base parameters (will be automatically adjusted)
baseTspan = 1.9204 / Gain;
baseWn = Gain * [3.3104   15.0000    5.9583];

% Automatically generate tspan and wn based on numPhases
if numPhases <= length(baseTspan)
    % Use existing base parameters if we have enough
    tspan = baseTspan(1:numPhases);
    wn = baseWn(1:(3*numPhases));
else
    % Extend parameters if we need more phases
    tspan = [baseTspan, baseTspan(end) + (1:(numPhases-length(baseTspan)))];
    additionalWn = repmat([2.0, 2.0, 2.0], 1, numPhases - length(baseWn)/3);
    wn = [baseWn, additionalWn];
end

fprintf('=== Automatic Configuration ===\n');
fprintf('Number of targets: %d\n', numTargets);
fprintf('Number of phases: %d\n', numPhases);
fprintf('Number of switching times: %d\n', length(tspan));
fprintf('Number of wn parameters: %d\n', length(wn));
fprintf('Total optimization parameters: %d\n\n', length(tspan) + length(wn));

% Compute final position
[Px, Py, Pz] = FKnew(qDes(1), qDes(2), qDes(3));
xFinal = [Px, Py, Pz];

% Build initial parameter vector
initPrms = [tspan, wn];

% Initial simulation
tUni = linspace(0, tspan(end), 1000)';
[tInit, yInit] = ode45(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan, wn), tUni, zeros(12, 1));

% Extract initial trajectory
[CxInit, CyInit, CzInit] = FKnew(yInit(:,7), yInit(:,8), yInit(:,9));
plot3(CxInit, CyInit, CzInit, 'b', 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Initial Configuration / Forward Kinematics');
axis equal;


s = num2str(Gain);
firstDigit = str2double(s(1));   % → 3

% %% ===== PLOTTING =====
save(sprintf('SG%ddata%d.mat', firstDigit,dataNumber), ...
        'xTarget', 'xFinal','yInit','tUni');


% %%
% % Save results
% if saveData==1
%     save(sprintf('Sdata%d.mat', dataNumber), ...
%         'Opt', 'tOpt', 'yOpt', 'xTarget', 'xFinal','optimalTimes', 'optimalWn');
%     print('Data not saved')
% 
% else
%     print('Data not saved')
% end

%% ===== HELPER FUNCTIONS =====


% Dynamics Function with Prefilter
function dxdt = myTwolinkwithprefilter(t, x, qDes, t_st, wn)
    zeta = [1 1 1];
    
    % Determine current phase based on time
    phase = 1;
    for i = 1:length(t_st)
        if t > t_st(i)
            phase = i + 1;
        end
    end
    
    % Select wn parameters for current phase
    wnPhase = wn((phase-1)*3+1:phase*3);
    
    A = [zeros(3), eye(3); -diag(wnPhase).^2, -2*diag(zeta)*diag(wnPhase)];
    B = [zeros(3); diag(wnPhase).^2];
    
    q = x(7:9);
    qd = x(10:12);
    
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  
    
    controller = Kp*(x(1:3)-q) + Kd*(x(4:6)-qd);
    
    [M,C,G] = compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau = M*(controller) + C*qd;
    qdd = M\(tau-C*qd);
    
    dxdt = [A*x(1:6) + B*qDes(:); qd; qdd];
end

% Forward Kinematics (FK)
function [x, y, z, valid] = FKnew(q1, q2, q3)
    % Joint limits
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07;   q2_max = 1.97;
    q3_min = -0.4;    q3_max = 2.92;
    
    % Link lengths
    l1 = 0.208; 
    l2 = 0.168;
    
    % Store original values for validation
    q1_orig = q1;
    q2_orig = q2;
    q3_orig = q3;
    
    % Clamp to absolute limits
    q1 = max(q1_min, min(q1_max, q1));
    q2 = max(q2_min, min(q2_max, q2));
    q3 = max(q3_min, min(q3_max, q3));
    
    % Relative limits for q3 based on q2
    q3_min_rel = max(q3_min, q2 - 0.93);
    q3_max_rel = min(q3_max, q2 + 0.95);
    
    % Apply relative limits
    q3 = max(q3_min_rel, min(q3_max_rel, q3));
    
    % Validity check (element-wise)
    valid = true(size(q1));
    valid = valid & (q1_orig >= q1_min & q1_orig <= q1_max);
    valid = valid & (q2_orig >= q2_min & q2_orig <= q2_max);
    valid = valid & (q3_orig >= q3_min & q3_orig <= q3_max);
    valid = valid & (q3_orig >= q3_min_rel & q3_orig <= q3_max_rel);
    
    % Forward kinematics (element-wise operations)
    x = sin(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
    y = l2 - l2 .* cos(q3) + l1 .* sin(q2);
    z = -l1 + cos(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
end


% Inverse Kinematics (IK)
function [Q, valid] = IKnew(x,y,z)
    % Joint limits
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07; q2_max = 1.97;
    q3_min = -0.4; q3_max = 2.92;
    
    % Link lengths
    l1=0.208; 
    l2=0.168;
    
    % Calculate inverse kinematics
    q1=atan2(x,z+l1);
    
    % Check if q1 is within limits
    if q1 < q1_min || q1 > q1_max
        valid = false;
        Q = [q1, 0, 0];
        warning('Joint 1 out of limits: [%.4f, %.4f]', q1_min, q1_max);
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
    
    % Check basic joint limits
    valid = true;
    if q2 < q2_min || q2 > q2_max
        valid = false;
        warning('Joint 2 out of limits: [%.4f, %.4f]', q2_min, q2_max);
    end
    
    if q3 < q3_min || q3 > q3_max
        valid = false;
        warning('Joint 3 out of limits: [%.4f, %.4f]', q3_min, q3_max);
    end
    
    % Check relationship between joint 2 and joint 3
    if valid
        % When Joint 2 is at position x, Joint 3 range is [x-0.86, x+0.95]
        q3_min_rel = q2 - 0.86;
        q3_max_rel = q2 + 0.95;
        
        % When Joint 3 is at position -x, Joint 2 range is [x-0.95, x+0.93]
        % This is equivalent to: when Joint 2 is at position x, Joint 3 range is [x-0.93, x+0.95]
        % We'll use the more restrictive constraint
        q3_min_rel2 = q2 - 0.93;
        q3_max_rel2 = q2 + 0.95;
        
        % Take the more restrictive limits
        q3_min_rel = max(q3_min_rel, q3_min_rel2);
        q3_max_rel = min(q3_max_rel, q3_max_rel2);
        
        % Apply absolute limits
        q3_min_rel = max(q3_min_rel, q3_min);
        q3_max_rel = min(q3_max_rel, q3_max);
        
        if q3 < q3_min_rel || q3 > q3_max_rel
            valid = false;
            warning('Joint 3 out of relationship limits: [%.4f, %.4f] for q2=%.4f', q3_min_rel, q3_max_rel, q2);
        end
    end
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