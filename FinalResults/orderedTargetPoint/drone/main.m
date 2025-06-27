% Parameters
params.m = 1.0; 
params.J = diag([0.02, 0.02, 0.04]); 
params.g = 9.81;

% Initial state: [pos; vel; quat; omega]
X0 = [0;0;0; 0;0;0; 1;0;0;0; 0;0;0];

% Time span
tspan = [0 20];

% Integrate
[t, X] = ode45(@(t,X) quadrotor_dynamics(t,X,params,@simple_controller), tspan, X0);

% Plot trajectory
figure;
plot3(X(:,1),X(:,2),X(:,3)); xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
