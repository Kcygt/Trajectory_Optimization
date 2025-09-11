%% Single-Step Trajectory for 3-Link Robot with Symbolic LaTeX Text and Figure Titles
clear; clc; close all;

% Time settings
dt = 0.01;
T  = 5;          % total simulation time
t  = 0:dt:T;

% Step time
t0 = 2;

% Start and goal positions for each joint
q_start = [0; 0; 0];   % joint 1, 2, 3
q_goal  = [1; -0.5; 0.8];

% Build step trajectories
u = zeros(3, length(t));
for j = 1:3
    u(j,:) = q_start(j) * ones(size(t));
    u(j, t >= t0) = q_goal(j);
end

% Plot trajectories
figureHandle = figure;  % store figure handle

% Big overall title
sgtitle('Single Section Method','FontSize',12,'FontWeight','bold');

for j = 1:3
    subplot(3,1,j);
    plot(t, u(j,:), 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel(['q_' num2str(j) ' (rad)']);  % units
    
    title(['Joint ' num2str(j)],'FontSize',12,'FontWeight','normal');
    
    % Adaptive text position
    y_mean = mean([q_start(j), q_goal(j)]);
    if y_mean >= 0
        y_text = 0.4;
    else
        y_text = 0.55;
    end
    x_text = 0.55;
    
    % Symbolic text
    txt = sprintf('$\\omega_{n%d}, \\zeta_%d, K_{p%d}, K_{d%d}$', j, j, j, j);
    text(x_text, y_text, txt, 'Units', 'normalized', 'FontSize', 12, ...
         'BackgroundColor','w', 'EdgeColor','k', 'Interpreter','latex');
    
    % Adjust y-axis limits
    y_min = min(q_start(j), q_goal(j)) - 0.2*abs(q_goal(j)-q_start(j));
    y_max = max(q_start(j), q_goal(j)) + 0.2*abs(q_goal(j)-q_start(j));
    ylim([y_min, y_max]);
end

% Save the figure as PNG and FIG
saveas(figureHandle, 'jointRepresentationSS.png');  % PNG format
saveas(figureHandle, 'jointRepresentationSS.fig');  % MATLAB figure format
