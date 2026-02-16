% Matlab program for Real-time data logger
% ---------------------------------------------------------------------
% File    : Haptic_Master_Data_Collection_with_Data_Logger.m
%
% Purpose : Matlab program to demonstrate the use of the Real-time
%           data logger
%
% History:
%  2022-07-06  EH    created
% ---------------------------------------------------------------------

% clear all
close all;
clc;

%----------------------------------------------------------------------
% Global Variables
%----------------------------------------------------------------------
hapticMasterDeviceHandle = -1;            % Handle to connected device
hapticMasterIPAddress    = '192.168.0.25'; % Device IP Address
retVal                   = 0;             % Return value of the haSendCommand
inertia                  = 2.5;           % Mass of the EndEffector
springConstant           = 50;           % spring value
dampFactor               = 0.1;           % damping factor for the spring
hmCalibrated             = 0;             % Isy HM calibrated?
ch                       = ' ';           % Key pressed in figure view

outputFileName = 'demo_';

homePositionVector = [-0.070949, 0.0,-0.197637 ]';



%----------------------------------------------------------------------
% Parameters for the optimal trajectory deformation algorithm
%----------------------------------------------------------------------
% Sampling time of the robot control loop
T = 1/2048;

% Sampling time for the trajectory deformation loop
%     delta = 5e-2;
delta = 1/2048 * 10;

% Duration of the interaction
tau = 1;

% Admittance of the optimal variation
mu = 0.01;

% Number of degrees-of-freedom for the robot
num_dof = 3;

% Stopping time for the robot control
t_stop = 16;

% Simulation time vector
t_vec = (0:T:t_stop)';

% Generate the original desired trajectory, x_d_star
x_d_star = zeros(length(t_vec), 3);
f_d_star = zeros(length(t_vec), 3);
for i = 1:length(t_vec)
    [xTmp, fTmp] = hippodrome(t_vec(i), homePositionVector);
    x_d_star(i, :) = xTmp';
    f_d_star(i, :) = fTmp';
end


%----------------------------------------------------------------------
% Pre-computation for the optimal trajectory deformation algorithm
%----------------------------------------------------------------------
% Ratio of sampling times
%     r = delta / T;

% Number of waypoints
N = double(int64(tau / delta) + 1);

% Constraint matrix
B = zeros(4, N);
B(1, 1) = 1;
B(2, 2) = 1;
B(3, N-1) = 1;
B(4, N) = 1;

% Finite differencing matrix
A = finiteDifferencingMatrix(N);

R = A' * A;

G = (eye(N) - inv(R) * B' * inv(B * inv(R) * B') * B) * inv(R) * ones(N,1);

% Shape of the optimal variation
H = sqrt(N) / norm(G) * G;

X_curr = [];

gamma_d_x = lineSegment(x_d_star(:, 1), 1, N, delta, T);
gamma_d_y = lineSegment(x_d_star(:, 2), 1, N, delta, T);
gamma_d_z = lineSegment(x_d_star(:, 3), 1, N, delta, T);
gamma_d = [gamma_d_x, gamma_d_y, gamma_d_z];



tic
t = toc;
tPrev = t;


loopCounter = 1;
X_generate = [];

Gamma_X = [gamma_d_x];
Gamma_Z = [gamma_d_z];
ind = []; 
while t <= t_stop
    f_e = [0.0; 0.0; 0.0];
    
    t=toc;
    tau_i = t;
    tau_f = tau_i + tau;
    x_d_curr = zeros(3, 1);
    
    
%     [springPositionVector, fd] = hippodrome(t, homePositionVector);

    if loopCounter < size(x_d_star, 1)
        springPositionVector = x_d_star(loopCounter, :)';
        fd = f_d_star(loopCounter, :)';
    else
        springPositionVector = x_d_star(end, :)';
        fd = f_d_star(end, :)';
    end
    
    f_err = f_e - fd;
    %         [x_d_curr_z, ~, gamma_d(:, 3)] = optimalDeformation(delta, mu, gamma_d(:, 3), f_err(3), H, tau_f);
    %         [x_d_curr_z, ~, gamma_d(:, 3)] = optimalDeformation(delta, mu, gamma_d(:, 3), 0.0, H, tau_f);
    [x_d_curr_x, ~, gamma_d_x] = optimalDeformation2(x_d_star(:, 1), loopCounter, N, delta, T, gamma_d_x, 0, H, mu);
    [x_d_curr_z, ~, gamma_d_z] = optimalDeformation2(x_d_star(:, 3), loopCounter, N, delta, T, gamma_d_z, 0, H, mu);
    
    Gamma_X = [Gamma_X, gamma_d_x];
    Gamma_Z = [Gamma_Z, gamma_d_z];
    
    springPositionVector(3) = x_d_curr_z;
    
    X_generate = [X_generate, springPositionVector];
    
    t = toc;
    while t - tPrev < delta
        t = toc;
    end
    tPrev = t;
    
    loopCounter = loopCounter + delta/T;
end



figure(1)
hold on
grid on

plot(x_d_star(:, 1), x_d_star(:, 3))

for i=1:300:3260
    plot(Gamma_X(:, i), Gamma_Z(:, i), 'r.')
end

plot(X_generate(1, :), X_generate(3, :), 'gs')