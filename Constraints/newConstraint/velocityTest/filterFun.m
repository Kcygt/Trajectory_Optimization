% Parameters
wn = 5; % Natural frequency (rad/s)
zeta = 1; % Damping ratio

% Time vector
t = 0:0.01:5;

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

fprintf('Max velocity: %.4f\n', max_vel);
fprintf('Time of max velocity: %.4f seconds\n', t_max_vel);


% Plot response and velocity
figure; grid on; hold on;
plot(t_out, y, 'DisplayName', sprintf('\\zeta = %.1f', zeta));
plot(t_out(1:end-1), velocity);
% plot(t_max_vel, max_vel, 'ro', 'DisplayName', 'Max Velocity');
% legend show;
xlabel('Time (s)');
ylabel('Response / Velocity');
title('Step Response and Velocity of Second-Order System');

% Display max velocity info
% fprintf('Maximum velocity: %.4f at time t = %.4f seconds\n', max_v_

% % Parameters
% wn = 3; % Natural frequency (rad/s)
% zeta = 1; % Different damping values
% 
% % Time vector
% t = 0:0.01:5;
% 
% % Plot step response 
% num = wn^2;
% den = [1 2*zeta*wn wn^2];
% sys = tf(num, den);
% [y, t_out] = step(sys, t);
% 
% figure;grid on;hold on
% plot(t_out, y, 'DisplayName', sprintf('\\zeta = %.1f', zeta));
% plot(t_out(1:end-1),diff(y),'DisplayName', 'Velocity')
% legend show;
% xlabel('Time (s)');
% ylabel('Response');
% title('Step Response of Second-Order System for Different Damping Ratios');
