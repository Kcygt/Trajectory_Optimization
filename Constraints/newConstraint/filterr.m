% System parameters
wn1 = 2;            % Natural frequency
zeta1 = 1;        % Damping ratio


wn2 = 0.5;            % Natural frequency
zeta2 = 1;        % Damping ratio


wn3 = 5;            % Natural frequency
zeta3 = 1;        % Damping ratio

% State-space matrices
A1 = [0 1;
     -wn1^2 -2*zeta1*wn1];

B1 = [0;
      wn1^2];

C1 = [1 0];   % Output is position
D1 = 0;


A2 = [0 1;
     -wn2^2 -2*zeta2*wn2];

B2 = [0;
      wn2^2];

C2 = [1 0];   % Output is position
D2 = 0;


A3 = [0 1;
     -wn3^2 -2*zeta3*wn3];

B3 = [0;
      wn3^2];

C3 = [1 0];   % Output is position
D3 = 0;

% Create state-space system
sys1 = ss(A1, B1, C1, D1);
sys2 = ss(A2, B2, C2, D2);
sys3 = ss(A3, B3, C3, D3);

% Time vector for simulation
t = 0:0.1:10;

% Step response
Out1 = step(sys1, t);
Out2 = step(sys2, t);
Out3 = step(sys3, t);


%%

% Number of points
N = 100;

% Parameter (angle for half ellipse)
theta = linspace(0, pi, N);

% Base half-ellipse (horizontal: a, vertical: b)
a = 1;  % x-radius
b = 0.5;  % y-radius
x = a * cos(theta)+0.1;
y = b * sin(theta)+0.5;

% Normalize so it starts at (0, 0) and ends at (0.5, 0.5)
% Rotate 45 degrees (to align from (0,0) to (0.5,0.5))
angle = -pi/7;
R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
coords = R * [x; y];

% Scale to match desired final point
end_point = [0.05; 0.05];
scale_factor = norm(end_point) / norm(coords(:, end));
coords = coords * scale_factor/ 5;

% Plot
plot(coords(1, :), coords(2, :), 'b', 'LineWidth', 2);
hold on;
plot(0, 0, 'ro');         % Start point
plot(0.05, 0.05, 'go');     % End point
axis equal;
grid on;
xlabel('X');
ylabel('Y');
title('Half Ellipse from (0,0) to (0.5,0.5)');
legend('Half-Ellipse', 'Start (0,0)', 'End (0.5,0.5)');



%%


subplot 311
plot(Out1)
subplot 312
plot(Out2)
subplot 313
plot(Out3)
