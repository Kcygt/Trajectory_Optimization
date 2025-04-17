% Scalar values
zeta = 1;
wn = 2;

% State-space matrices for 1D second-order system
A = [0 1;
    -wn^2  -2*zeta*wn];

B = [0;
     wn^2];

C = eye(2);   % To view both position and velocity states
D = [0; 0];

% Create state-space system
sys = ss(A, B, C, D);

% Time vector
t = 0:0.01:10;

[y, t] = step(sys, t);
plot(t, y(:,1), 'b', t, y(:,2), 'r--');
legend('Position (out1)', 'Velocity (out2)');
title('Step Response of Position and Velocity');
xlabel('Time (s)');
ylabel('Response');