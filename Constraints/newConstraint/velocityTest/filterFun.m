% Parameters
wn = 3; % Natural frequency (rad/s)
zeta = 1; % Different damping values

% Time vector
t = 0:0.01:5;

% Plot step response 
num = wn^2;
den = [1 2*zeta*wn wn^2];
sys = tf(num, den);
[y, t_out] = step(sys, t);

figure;grid on;
plot(t_out, y, 'DisplayName', sprintf('\\zeta = %.1f', zeta));
legend show;
xlabel('Time (s)');
ylabel('Response');
title('Step Response of Second-Order System for Different Damping Ratios');
