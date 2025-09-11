%% Four-Step Trajectory for 3-Link Robot with Bold Symbolic Parameters (Fixed LaTeX)
clear; clc; close all;

% Time settings
dt = 0.01;
T  = 4;
t  = 0:dt:T;

% Step times
t0 = 0; t1 = 1; t2 = 2; t3 = 3;

% Start and goal positions
q_start = [0; 0; 0];
q_goal  = [1; -0.5; 0.8];

% Intermediate targets
q_target1 = q_start + (q_goal - q_start)*0.25;
q_target2 = q_start + (q_goal - q_start)*0.5;
q_target3 = q_start + (q_goal - q_start)*0.75;

% Section colors
colors = [0.121, 0.466, 0.705;   
          1.000, 0.498, 0.054;   
          0.839, 0.152, 0.156;   
          0.172, 0.627, 0.172];  

% Build step trajectory
u = zeros(3, length(t));
for j = 1:3
    u(j,:) = q_start(j);
    u(j, t >= t0 & t < t1) = q_target1(j);
    u(j, t >= t1 & t < t2) = q_target2(j);
    u(j, t >= t2 & t < t3) = q_target3(j);
    u(j, t >= t3) = q_goal(j);
end

% Plot each joint
for j = 1:3
    figure(j); grid on; hold on;

    % Section start times (X)
    x_starts = [t0, t1, t2, t3];
    % Y: middle of step
    y_mids = [(q_start(j)+q_target1(j))/2, ...
              (q_target1(j)+q_target2(j))/2, ...
              (q_target2(j)+q_target3(j))/2, ...
              (q_target3(j)+q_goal(j))/2];

    % Plot steps
    plot([t(1), t0], [q_start(j), q_start(j)], 'LineWidth',2,'Color',colors(1,:));
    plot([t0,t0],[q_start(j),q_target1(j)],'LineWidth',2,'Color',colors(1,:));
    plot([t0,t1],[q_target1(j),q_target1(j)],'LineWidth',2,'Color',colors(1,:));

    plot([t1, t1],[q_target1(j),q_target2(j)],'LineWidth',2,'Color',colors(2,:));
    plot([t1,t2],[q_target2(j),q_target2(j)],'LineWidth',2,'Color',colors(2,:));

    plot([t2, t2],[q_target2(j),q_target3(j)],'LineWidth',2,'Color',colors(3,:));
    plot([t2,t3],[q_target3(j),q_target3(j)],'LineWidth',2,'Color',colors(3,:));

    plot([t3, t3],[q_target3(j),q_goal(j)],'LineWidth',2,'Color',colors(4,:));
    plot([t3,t(end)],[q_goal(j),q_goal(j)],'LineWidth',2,'Color',colors(4,:));

    % Place text at start of section and middle Y
    for s = 1:4
        % Fixed LaTeX formatting - use square brackets and simplified names
        str = sprintf('$[\\omega_{n%d%d}, \\zeta_{%d%d}, K_{p%d%d}, K_{d%d%d}]$', ...
                      s,j,s,j,s,j,s,j);
        text(x_starts(s) + 0.7, y_mids(s), str, ...
             'Color', colors(s,:), 'Interpreter','latex', ...
             'FontSize', 12, 'FontWeight','bold', 'HorizontalAlignment','center');
    end
    % Invisible lines for legend
    h1 = plot(NaN,NaN,'LineWidth',2,'Color',colors(1,:));
    h2 = plot(NaN,NaN,'LineWidth',2,'Color',colors(2,:));
    h3 = plot(NaN,NaN,'LineWidth',2,'Color',colors(3,:));
    h4 = plot(NaN,NaN,'LineWidth',2,'Color',colors(4,:));

    legend([h1,h2,h3,h4],{'Section 1','Section 2','Section 3','Section 4'}, 'Location','best');

    xlabel('Time [s]');
    ylabel(['q_' num2str(j) ' (rad)']);
    title(['Joint ' num2str(j)]);
    ylim([min(u(j,:))-0.2, max(u(j,:))+0.2]);

    % Save figure
    saveas(gcf, ['joint' num2str(j) '_stepTrajectoryMS.png']);
    saveas(gcf, ['joint' num2str(j) '_stepTrajectoryMS.fig']);
end
