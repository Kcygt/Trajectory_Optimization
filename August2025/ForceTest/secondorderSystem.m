clc;
clear;

% Fixed natural frequency
omega_n = 5;

% Different damping ratios
zeta_values = [0.2, 1.0, 1.5];  % Underdamped, critically damped, overdamped

% Corresponding legend labels
legend_labels = {'Underdamped (\zeta = 0.2)', 'Critically damped (\zeta = 1.0)', 'Overdamped (\zeta = 1.5)'};

% Time vector
t = linspace(0, 5, 1000);

% Unit step input (starts at 0)
u = ones(size(t));
u(t == 0) = 0;

% Palette colors
colors = lines(length(zeta_values));

% Create figure
figure;
hold on;

% Plot step responses
for i = 1:length(zeta_values)
    zeta = zeta_values(i);
    num = [omega_n^2];
    den = [1, 2*zeta*omega_n, omega_n^2];
    sys = tf(num, den);
    
    y = lsim(sys, u, t);
    
    plot(t, y, 'Color', colors(i, :), 'LineWidth', 1.8, 'DisplayName', legend_labels{i});
end

% Plot step input last (on top)
plot(t, u, '--k', 'LineWidth', 2, 'DisplayName', 'Step Input');

% Finalize plot
title(['Step Response for Different Damping Ratios (\omega_n = ', num2str(omega_n), ')']);
xlabel('Time (seconds)');
ylabel('Amplitude');
legend('Location', 'southeast');
grid on;
hold off;


clc;
clear;

% Damping ratio (adjust if needed)
zeta = 1;

% Natural frequencies to compare
omega_n_values = [2, 5, 10];

% Time vector
t = linspace(0, 5, 1000);  % From 0 to 5 seconds

% Unit step input (starts at 0, jumps to 1)
u = ones(size(t));  
u(t == 0) = 0;

% Use palette colors
colors = lines(length(omega_n_values));

% Create figure
figure;
hold on;

% Plot each system's response first
for i = 1:length(omega_n_values)
    omega_n = omega_n_values(i);

    % Define second-order system
    num = [omega_n^2];
    den = [1, 2*zeta*omega_n, omega_n^2];
    sys = tf(num, den);

    % Simulate system response to step input
    y = lsim(sys, u, t);

    % Plot response
    plot(t, y, 'Color', colors(i, :), 'LineWidth', 1.8, ...
         'DisplayName', ['\omega_n = ', num2str(omega_n)]);
end

% Plot the step input last so it appears on top
plot(t, u, '--k', 'LineWidth', 2, 'DisplayName', 'Step Input');

% Final plot formatting
title(['Step Response for Different \omega_n Values (Zeta = ', num2str(zeta), ')']);
xlabel('Time (seconds)');
ylabel('Amplitude');
legend('Location', 'southeast');
grid on;
hold off;
