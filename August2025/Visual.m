% Set up a 3D workspace figure with proper 3D view
figure('Position', [100, 100, 800, 600]);
hold on;
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;
view(45, 30); % Set a good 3D viewing angle

% Set the axis limits for XY and Z ranges
xlim([-0.1 0.2]);
ylim([-0.1 0.2]);
zlim([-0.1 0.2]);

title('3D Robot Workspace - Click a point in XY plane, then input Z value');

% Plot a simple sphere representing the workspace (3D object)
[spX, spY, spZ] = sphere(20);
surf(spX * 0.25, spY * 0.25, spZ * 0.25, 'FaceAlpha', 0.2, 'FaceColor', 'cyan');

% Add coordinate axes for better 3D reference
quiver3(0, 0, 0, 0.1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.02); % X-axis
quiver3(0, 0, 0, 0, 0.1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.02); % Y-axis  
quiver3(0, 0, 0, 0, 0, 0.1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.02); % Z-axis
text(0.12, 0, 0, 'X', 'Color', 'red', 'FontSize', 12);
text(0, 0.12, 0, 'Y', 'Color', 'green', 'FontSize', 12);
text(0, 0, 0.12, 'Z', 'Color', 'blue', 'FontSize', 12);

% Get the 2D point from the user (interactive XY-plane selection)
disp('Click a point in the XY plane to select the target (click 1 point)');
[x, y] = ginput(1);  % Get the clicked point in 2D

% Show the selected point in the XY plane in 3D space
scatter3(x, y, 0, 150, 'r', 'filled', 'MarkerEdgeColor', 'red');  % This shows the point in 3D space at z = 0
text(x, y, 0, sprintf('  Target (%.3f, %.3f, 0)', x, y), 'Color', 'red', 'FontSize', 10, 'FontWeight', 'bold');

% Ask for the Z-coordinate input from the user
prompt = {'Enter Z coordinate (e.g., 0.1):'};
dlgtitle = 'Z Coordinate Input';
dims = [1 35];
definput = {'0'};
answer = inputdlg(prompt, dlgtitle, dims, definput);

% Extract the Z value and display it
z = str2double(answer{1});
disp(['Target point: X = ', num2str(x), ', Y = ', num2str(y), ', Z = ', num2str(z)]);

% Visualize the 3D target point
scatter3(x, y, z, 150, 'g', 'filled', 'MarkerEdgeColor', 'green');  % This is the 3D target point
text(x, y, z, sprintf('  3D Target (%.3f, %.3f, %.3f)', x, y, z), 'Color', 'green', 'FontSize', 10, 'FontWeight', 'bold');

% Draw a vertical line from XY plane to 3D target for better visualization
plot3([x, x], [y, y], [0, z], 'k--', 'LineWidth', 1);

% Compute the joint angles using IK
Q = IK(x, y, z);  % Get the joint angles
disp('Computed joint angles (q1, q2, q3):');
disp(rad2deg(Q));  % Display joint angles in degrees

% Visualize the arm based on the computed joint angles using FK
[x_fk, y_fk, z_fk] = FK(Q(1), Q(2), Q(3));  % Forward kinematics

% Initialize the arm's link positions
l1 = 0.208;  % Link 1 length
l2 = 0.168;  % Link 2 length

link1_end = [0, 0, 0];  % Base
link2_end = [l1 * cos(Q(1)), l1 * sin(Q(1)), 0];  % End of link1
link3_end = [x_fk, y_fk, z_fk];  % End of link2 (end effector)

% Plot the robot arm in 3D with thicker lines
plot3([link1_end(1), link2_end(1)], [link1_end(2), link2_end(2)], [link1_end(3), link2_end(3)], 'b-', 'LineWidth', 4);
plot3([link2_end(1), link3_end(1)], [link2_end(2), link3_end(2)], [link2_end(3), link3_end(3)], 'g-', 'LineWidth', 4);

% Add joint markers for visualization
scatter3(link1_end(1), link1_end(2), link1_end(3), 120, 'k', 'filled'); % Joint 1 (base)
scatter3(link2_end(1), link2_end(2), link2_end(3), 120, 'k', 'filled'); % Joint 2
scatter3(link3_end(1), link3_end(2), link3_end(3), 120, 'r', 'filled'); % Joint 3 (end effector)

% Add labels for joints
text(link1_end(1), link1_end(2), link1_end(3), '  Base', 'Color', 'black', 'FontSize', 10);
text(link2_end(1), link2_end(2), link2_end(3), '  Joint 2', 'Color', 'black', 'FontSize', 10);
text(link3_end(1), link3_end(2), link3_end(3), '  End Effector', 'Color', 'red', 'FontSize', 10);

% Show a line from the base to the target for visual reference
plot3([0, x], [0, y], [0, z], 'r--', 'LineWidth', 2);

% Add legend
legend('Workspace', 'XY Target', '3D Target', 'Link 1', 'Link 2', 'Base to Target', 'Location', 'best');

% Enable 3D rotation for better interaction
rotate3d on;
disp('You can now rotate the 3D view by clicking and dragging');

% FK Function (to calculate the arm's position in 3D)
function [x, y, z] = FK(q1, q2, q3)
    l1 = 0.208; % Length of link 1
    l2 = 0.168; % Length of link 2

    % Forward Kinematics equations
    x = sin(q1) * (l1 * cos(q2) + l2 * sin(q3));
    y = l2 - l2 * cos(q3) + l1 * sin(q2);
    z = -l1 + cos(q1) * (l1 * cos(q2) + l2 * sin(q3));
end

% IK Function (Inverse Kinematics based on selected target)
function Q = IK(x, y, z)
    l1 = 0.208;  % Length of link 1
    l2 = 0.168;  % Length of link 2

    % Joint 1 (rotation around Y-axis)
    q1 = atan2(x, -(z + l1));  % Adjusting based on FK assumptions

    % Projected distances for link lengths
    R = sqrt(x^2 + (z + l1)^2);
    r = sqrt(x^2 + (y - l2)^2 + (z + l1)^2);

    % Solving for q2 and q3 using the law of cosines
    Beta = atan2(y - l2, R);
    Gamma = acos((l1^2 + r^2 - l2^2) / (2 * l1 * r));
    q2 = Gamma + Beta;

    Alpha = acos((l1^2 + l2^2 - r^2) / (2 * l1 * l2));
    q3 = q2 + Alpha - pi/2;

    Q = [q1, q2, q3];
end
