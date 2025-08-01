% Convert tspan times to indices
t1 = tspan(1); t2 = tspan(2); t3 = tspan(3);
[~, i1] = min(abs(tUni - t1));
[~, i2] = min(abs(tUni - t2));
[~, i3] = min(abs(tUni - t3));

% Plot segmented trajectory
figure; hold on; grid on;

% Segment 1: [0, t1]
plot3(CxInit(1:i1), CyInit(1:i1), CzInit(1:i1), 'r-', 'LineWidth', 1.5)

% Segment 2: (t1, t2]
plot3(CxInit(i1+1:i2), CyInit(i1+1:i2), CzInit(i1+1:i2), 'g-', 'LineWidth', 1.5)

% Segment 3: (t2, t3]
plot3(CxInit(i2+1:i3), CyInit(i2+1:i3), CzInit(i2+1:i3), 'b-', 'LineWidth', 1.5)

% Final segment: (t3, end]
plot3(CxInit(i3+1:end), CyInit(i3+1:end), CzInit(i3+1:end), 'k--', 'LineWidth', 1.2)

% Plot targets and final position
plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3), '*', 'LineWidth', 1.2, 'MarkerSize', 8, 'Color', [0,0.4,0.8])
plot3(xFinal(1), xFinal(2), xFinal(3), 'p', 'LineWidth', 1.5, 'MarkerSize', 14, 'Color', [1,0.5,0.05])

% Labels and styling
legend('Segment 1 (0–t1)', 'Segment 2 (t1–t2)', 'Segment 3 (t2–t3)', 'Post t3', 'Target', 'End')
xlabel('X'); ylabel('Y'); zlabel('Z')
title('Segmented Initial 3D Trajectory')
view(90,0)
