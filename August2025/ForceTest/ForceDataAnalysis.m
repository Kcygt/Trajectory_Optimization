% close all
% clear
% clc

load('Fdata.mat')

time = 0:0.001:9.999;  % 1x5001 vector

% Extract relevant data
x_data = Fdata(1800:8314,1);
y_data = Fdata(1800:8314,2);
z_data = Fdata(1800:8314,3);
Fz = Fdata(1800:8314,6);

% Forward kinematics
[x, y, z] = FK(x_data, y_data, z_data);

% Adjust y to start from 0
y_relative = y - y(1);

% --- Linear regression between Fz and y_relative ---
p = polyfit(y_relative, Fz, 1);   % Fit: Fz = p(1)*y + p(2)
Fz_fit = polyval(p, y_relative); % Evaluate fit

% Display regression equation
fprintf('Linear regression equation: Fz = %.4f * y + %.4f\n', p(1), p(2));

% --- Plot Fz vs. y with regression line ---
figure; hold on; grid on;
plot(y_relative, Fz, 'b.', 'DisplayName', 'Original Data');
plot(y_relative, Fz_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Linear Fit');
xlabel('y - y(1)');
ylabel('Fz');
title('Linear Regression: Fz vs. y');
legend;
