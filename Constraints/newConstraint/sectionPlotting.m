t = 0:0.01:10;

p1 = 0.3*ones(350, 1);
p2 = 0.7*ones(350, 1);
p3 = 1*ones(298, 1);
colors = ["#77AC30", "#7E2F8E", "#0072BD"];

% Create the plot
hold on; grid on
stairs(t(1:351), [0; p1], 'Color',colors(1), 'LineWidth', 2);  % Section 1: red dashed
stairs(t(352:702), [0.3; p2], 'Color',colors(2), 'LineWidth', 2); % Section 2: green dash-dot
stairs(t(703:end), [0.7; p3], 'Color',colors(3), 'LineWidth', 2);  % Section 3: blue dotted

% Labels and title
xlabel('Time (s)');
ylabel('Joint Position (rad)');
title('Step Function with Different Colors and Line Styles');
legend('Section 1', 'Section 2', 'Section 3');


text(0.2, 0.35, '\omega_{n1}   t_1 ', 'Color', colors(1), 'FontSize', 10);
text(4.2, 0.75, '\omega_{n2}   t_2', 'Color', colors(2), 'FontSize', 10);
text(7.5, 1.05, '\omega_{n3}   t_3', 'Color', colors(3), 'FontSize', 10);
text(9.9, 1.01, 'o  End Point', 'Color', colors(3), 'FontSize', 10);

hold off;
