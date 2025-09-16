clc; clear; close all;

% Define start and end points
startP = [0 0 0];
endP   = [10 10 5];

% Define 3 target points (example)
targets = [2 1 1;
           4 5 2;
           7 8 3];

% Define 2 control points (example)
controlPts = [3 2 1;
              6 6 2.5];

% Combine all points: start -> targets -> control -> end
points = [startP; targets; controlPts; endP];

% Interpolate trajectory using spline for smoothness
t = 1:size(points,1);
tt = linspace(1, size(points,1), 200); % dense parameterization
traj = spline(t, points', tt)';

% Plot trajectory
figure;
plot3(traj(:,1), traj(:,2), traj(:,3), 'b-', 'LineWidth', 2); hold on;
scatter3(points(:,1), points(:,2), points(:,3), 100, 'r', 'filled'); % key points
text(points(:,1), points(:,2), points(:,3), ...
    {'Start','T1','T2','T3','C1','C2','End'}, 'VerticalAlignment','bottom');

% Plot spheres around control points
[XS, YS, ZS] = sphere(30); % unit sphere mesh
r = 0.01; % 1 cm = 0.01 m (assuming units are in meters)
for i = 1:size(controlPts,1)
    surf(r*XS + controlPts(i,1), ...
         r*YS + controlPts(i,2), ...
         r*ZS + controlPts(i,3), ...
         'FaceAlpha',0.3,'EdgeColor','none','FaceColor','g');
end

% Beautify plot
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trajectory with Targets, Control Points and Spheres');
view(3);
