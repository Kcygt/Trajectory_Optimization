% Parameters
wn = 5; % Natural frequency (rad/s)
zeta = 1; % Damping ratio

% Time vector
t = 0:0.01:7;

% Transfer function
num = wn^2;
den = [1 2*zeta*wn wn^2];
sys = tf(num, den);

% Step response
[y, t_out] = step(sys, t);

% Numerical derivative (velocity)
velocity = diff(y) ./ diff(t');

% Find max velocity and corresponding time
[max_vel, idx_max] = max(velocity);
t_max_vel = t_out(idx_max);

% Compute step response characteristics
info = stepinfo(y, t_out);  % Corrected line
rise_time = info.RiseTime;
settling_time = info.SettlingTime;

fprintf('Max velocity: %.4f\n', max_vel);
fprintf('Time of max velocity: %.4f seconds\n', t_max_vel);
fprintf('Rise Time: %.4f seconds\n', rise_time);
fprintf('Settling Time: %.4f seconds\n', settling_time);

% Plot response and velocity
figure(1); grid on; hold on;
plot(t_out, y, 'DisplayName', sprintf('\\zeta = %.1f', zeta));
plot(t_out(1:end-1), velocity, '--', 'DisplayName', 'Velocity');
plot(t_max_vel, max_vel, 'ro', 'DisplayName', 'Max Velocity');
legend show;
xlabel('Time (s)');
ylabel('Response / Velocity');
title('Step Response and Velocity of Second-Order System');
