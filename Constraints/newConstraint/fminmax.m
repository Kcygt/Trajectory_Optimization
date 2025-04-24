% Time vector
t = linspace(0, 10, 1000);

% Create a function with 4 peaks
y = sin(2*pi*0.5*t) + 0.5*sin(2*pi*1.5*t + pi/4);
s = linspace(0, 5, 1000);
y = y + s;
% Plot it
plot(t, y, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time');
ylabel('Amplitude');
title('Waveform with 4 Peaks');


% Find local minima
local_min_idx = islocalmin(y);
hold on;
plot(t(local_min_idx), y(local_min_idx), 'ro', 'MarkerSize', 8, 'DisplayName', 'Local Minima');

% Global minimum
[y_min, idx_min] = min(y);
plot(t(idx_min), y_min, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','k', 'DisplayName', 'Global Minimum');

legend show;