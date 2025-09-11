% Define start, targets, and final points
x_start = 0;       
y_start = 0;       

x_target1 = 0.2;      
y_target1 = 0.05;    

x_target2 = 0.3;      
y_target2 = 0.2;    

x_target3 = 0.4;      
y_target3 = 0.45;    

x_final = 0.5;       
y_final = 0.5;       

% Define colors (RGB)
color_start = [0.121, 0.466, 0.705];   % Start: blue
color_target = [1.000, 0.498, 0.054];  % Targets: orange (same for all)
color_final = [0.172, 0.627, 0.172];   % Final: green

% Plot points
figure;
hold on;

h1 = scatter(x_start, y_start, 100, color_start, 'o', 'filled');  % Start: circle
h2 = scatter([x_target1, x_target2, x_target3], [y_target1, y_target2, y_target3], ...
             100, color_target, 's', 'filled');                    % Targets: square, same color
h3 = scatter(x_final, y_final, 100, color_final, 'p', 'filled');   % Final: pentagram

% Add labels
text(x_start, y_start, '  Start', 'FontSize', 12);
text(x_target1, y_target1, '  Target1', 'FontSize', 12);
text(x_target2, y_target2, '  Target2', 'FontSize', 12);
text(x_target3, y_target3, '  Target3', 'FontSize', 12);
text(x_final, y_final, '  Final', 'FontSize', 12);

% Set labels, title, grid, and aspect ratio
xlabel('x axis (m)');
ylabel('y axis (m)');
title('Cartesian Space Position');
grid on;
axis equal;

% Add legend
legend([h1, h2, h3], {'Start Point', 'Target Points', 'Final Point'}, 'Location', 'best');
