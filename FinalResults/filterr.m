% Define damping ratio (critical damping)
zeta = 1.0;

% Define different natural frequencies
wn_list = [.5, 1, 3];   % Low, medium, high natural frequencies

% Time vector
t = 0:0.001:10;

% Preallocate responses
responses = zeros(length(wn_list), length(t));

% Define the custom colors
colors = ["#77AC30", "#7E2F8E", "#0072BD"];

% Simulate each system
for i = 1:length(wn_list)
    wn = wn_list(i);
    A = [0 1; -wn^2 -2*zeta*wn];
    B = [0; wn^2];  % Must match wn
    C = [1 0];
    D = 0;
    
    sys = ss(A, B, C, D);
    
    [y, t_out] = step(sys, t);
    responses(i, :) = y;
end

% Step input
u = ones(size(t));

% Plotting
figure;
plot(t_out, u, 'k--', 'LineWidth', 1.5); hold on; % Step input
plot(t_out, responses(1,:), 'Color', colors(1), 'LineWidth', 2);
plot(t_out, responses(2,:), 'Color', colors(2), 'LineWidth', 2);
plot(t_out, responses(3,:), 'Color', colors(3), 'LineWidth', 2);
grid on;
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Step Response with Different Natural Frequencies (\zeta=1)');
ylim([0 1.2]); % Adjust the limits if necessary
legend('Step Input')
% Add text annotations directly on the plot
% text(1.5, 1.05, 'Step Input', 'Color', 'k', 'FontSize', 10);
text(5.5, 0.7, '\omega_n = 0.5', 'Color', colors(1), 'FontSize', 10);
text(3.5, 0.8, '\omega_n = 1', 'Color', colors(2), 'FontSize', 10);
text(1.5, 0.9, '\omega_n = 3', 'Color', colors(3), 'FontSize', 10);



%%%%%%%%%%%%%%%%%%%%%
% % Define natural frequency
% wn1 = 10;  % Example value
% 
% % Define different damping ratios
% zeta_overdamped = 5;  
% zeta_critical   = 1.0;  
% zeta_underdamped = 0.2; 
% 
% % Time vector
% t = 0:0.001:2;
% 
% % Define systems
% A_over = [0 1; -wn1^2 -2*zeta_overdamped*wn1];
% B = [0; wn1^2];
% C = [1 0];
% D = 0;
% sys_over = ss(A_over, B, C, D);
% 
% A_critical = [0 1; -wn1^2 -2*zeta_critical*wn1];
% sys_critical = ss(A_critical, B, C, D);
% 
% A_under = [0 1; -wn1^2 -2*zeta_underdamped*wn1];
% sys_under = ss(A_under, B, C, D);
% 
% % Simulate step responses
% [y_over, t_out] = step(sys_over, t);
% [y_critical, ~] = step(sys_critical, t);
% [y_under, ~] = step(sys_under, t);
% 
% % Step input
% u = ones(size(t));
% 
% % Plotting
% figure;
% plot(t_out, u, 'k--', 'LineWidth', 1.5); hold on; % Step input
% plot(t_out, y_over, 'Color',"#77AC30", 'LineWidth', 2);
% plot(t_out, y_critical, 'Color',"#7E2F8E",'LineWidth', 2);
% plot(t_out, y_under, 'Color', "#0072BD", 'LineWidth', 2);
% grid on;
% xlabel('Time (seconds)');
% ylabel('Amplitude');
% title('Damping Ratio Impact for Second-Order Filter System');
% legend('Step Input')
% % Add text annotations directly on the plot
% % text(1.5, 1.05, 'Step Input', 'Color', 'k', 'FontSize', 10);
% text(0.8, 0.45, 'Overdamped (\zeta=1.5)', 'Color', "#77AC30", 'FontSize', 10);
% text(0.22, 0.6, 'Critically Damped (\zeta=1.0)', 'Color', "#7E2F8E", 'FontSize', 10);
% text(.5, 1.4, 'Underdamped (\zeta=0.3)', 'Color', "#0072BD", 'FontSize', 10);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % System parameters
% wn1 = 2;            % Natural frequency
% zeta1 = 1;        % Damping ratio
% 
% 
% wn2 = 0.5;            % Natural frequency
% zeta2 = 1;        % Damping ratio
% 
% 
% wn3 = 5;            % Natural frequency
% zeta3 = 1;        % Damping ratio
% 
% % State-space matrices
% A1 = [0 1;
%      -wn1^2 -2*zeta1*wn1];
% 
% B1 = [0;
%       wn1^2];
% 
% C1 = [1 0];   % Output is position
% D1 = 0;
% 
% 
% A2 = [0 1;
%      -wn2^2 -2*zeta2*wn2];
% 
% B2 = [0;
%       wn2^2];
% 
% C2 = [1 0];   % Output is position
% D2 = 0;
% 
% 
% A3 = [0 1;
%      -wn3^2 -2*zeta3*wn3];
% 
% B3 = [0;
%       wn3^2];
% 
% C3 = [1 0];   % Output is position
% D3 = 0;
% 
% % Create state-space system
% sys1 = ss(A1, B1, C1, D1);
% sys2 = ss(A2, B2, C2, D2);
% sys3 = ss(A3, B3, C3, D3);
% 
% % Time vector for simulation
% t = 0:0.1:10;
% 
% % Step response
% Out1 = step(sys1, t);
% Out2 = step(sys2, t);
% Out3 = step(sys3, t);
% 
% 
% %%
% 
% % Number of points
% N = 100;
% 
% % Parameter (angle for half ellipse)
% theta = linspace(0, pi, N);
% 
% % Base half-ellipse (horizontal: a, vertical: b)
% a = 1;  % x-radius
% b = 0.5;  % y-radius
% x = a * cos(theta)+0.1;
% y = b * sin(theta)+0.5;
% 
% % Normalize so it starts at (0, 0) and ends at (0.5, 0.5)
% % Rotate 45 degrees (to align from (0,0) to (0.5,0.5))
% angle = -pi/7;
% R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
% coords = R * [x; y];
% 
% % Scale to match desired final point
% end_point = [0.05; 0.05];
% scale_factor = norm(end_point) / norm(coords(:, end));
% coords = coords * scale_factor/ 5;
% 
% % Plot
% plot(coords(1, :), coords(2, :), 'b', 'LineWidth', 2);
% hold on;
% plot(0, 0, 'ro');         % Start point
% plot(0.05, 0.05, 'go');     % End point
% axis equal;
% grid on;
% xlabel('X');
% ylabel('Y');
% title('Half Ellipse from (0,0) to (0.5,0.5)');
% legend('Half-Ellipse', 'Start (0,0)', 'End (0.5,0.5)');
% 
% 
% 
% %%
% 
% 
% subplot 311
% plot(Out1)
% subplot 312
% plot(Out2)
% subplot 313
% plot(Out3)
