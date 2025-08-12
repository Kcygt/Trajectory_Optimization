% Second-order system demo
% Transfer function: H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

clear; clc; close all;

% Define parameters
wn = 5;      % Natural frequency (rad/s)
zeta = 1.5;  % Damping ratio

% Create transfer function
num = [wn^2];
den = [1 2*zeta*wn wn^2];
sys = tf(num, den);

% Step response
figure;
step(sys, 5); % 5 seconds simulation
grid on;
title(sprintf('Step Response (\\omega_n = %.2f rad/s, \\zeta = %.2f)', wn, zeta));

% Frequency response
figure;
bode(sys);
grid on;
title(sprintf('Bode Plot (\\omega_n = %.2f rad/s, \\zeta = %.2f)', wn, zeta));

% Optional: Pole-zero map
figure;
pzmap(sys);
grid on;
title('Pole-Zero Map');
