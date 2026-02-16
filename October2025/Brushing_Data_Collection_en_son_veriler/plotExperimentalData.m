function plotExperimentalData(csvFileName, matFileName)

% Load experimental data from data logger
csvData = csvread(csvFileName, 1, 0);

% Load experimental data from Matlab workspace
matData = load(matFileName);

% Plot the position trajectory in xz-plane
figure(1)
hold on
grid on

plot(matData.x_d_star(:,1), matData.x_d_star(:,3))
plot(csvData(:, 3), csvData(:,5), 'r--')

legend('Desired trajectory', 'Actual trajectory', 'Location', 'best')

xlabel('x-axis [m]')
ylabel('z-axis [m]')


% Plot position trajectory with respect to time
figure(2)

subplot 311
hold on, grid on
plot(matData.t_vec, matData.x_d_star(:,1))
plot(csvData(:, 2), csvData(:, 3), 'r--')
xlabel('Time [s]')
ylabel('X position [m]')

subplot 312
hold on, grid on
plot(matData.t_vec, matData.x_d_star(:,2))
plot(csvData(:, 2), csvData(:, 4), 'r--')
xlabel('Time [s]')
ylabel('Y position [m]')

subplot 313
hold on, grid on
plot(matData.t_vec, matData.x_d_star(:,3))
plot(csvData(:, 2), csvData(:, 5), 'r--')
xlabel('Time [s]')
ylabel('Z position [m]')

% Plot velocity trajectory with respect to time
figure(3)

subplot 311
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,1))
plot(csvData(:, 2), csvData(:, 6), 'r--')
xlabel('Time [s]')
ylabel('X velocity [m/s]')

subplot 312
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,2))
plot(csvData(:, 2), csvData(:, 7), 'r--')
xlabel('Time [s]')
ylabel('Y velocity [m/s]')

subplot 313
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,3))
plot(csvData(:, 2), csvData(:, 8), 'r--')
xlabel('Time [s]')
ylabel('Z velocity [m/s]')

% Plot acceleration trajectory with respect to time
figure(4)

subplot 311
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,1))
plot(csvData(:, 2), csvData(:, 9), 'r--')
xlabel('Time [s]')
ylabel('X acceleration [m/s^2]')

subplot 312
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,2))
plot(csvData(:, 2), csvData(:, 10), 'r--')
xlabel('Time [s]')
ylabel('Y acceleration [m/s^2]')

subplot 313
hold on, grid on
% plot(matData.t_vec, matData.x_d_star(:,3))
plot(csvData(:, 2), csvData(:, 11), 'r--')
xlabel('Time [s]')
ylabel('Z acceleration [m/s^2]')

% Plot forces with respect to time
figure(5)

subplot 311
hold on, grid on
plot(matData.t_vec, matData.f_d_star(:,1))
plot(csvData(:, 2), csvData(:, 12), 'r--')
xlabel('Time [s]')
ylabel('R force [N]')

subplot 312
hold on, grid on
plot(matData.t_vec, matData.f_d_star(:,2))
plot(csvData(:, 2), csvData(:, 13), 'r--')
xlabel('Time [s]')
ylabel('\phi force [N]')

subplot 313
hold on, grid on
plot(matData.t_vec, matData.f_d_star(:,3))
plot(csvData(:, 2), csvData(:, 14), 'r--')
xlabel('Time [s]')
ylabel('Z force [N]')

end