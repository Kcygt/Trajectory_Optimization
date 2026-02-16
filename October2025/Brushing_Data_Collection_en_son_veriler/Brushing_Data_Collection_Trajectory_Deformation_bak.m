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

% close all, clear all
% close all;
% clear all;

%----------------------------------------------------------------------
% Global Variables
%----------------------------------------------------------------------
hapticMasterDeviceHandle = -1;            % Handle to connected device
hapticMasterIPAddress    = '192.168.0.25'; % Device IP Address
retVal                   = 0;             % Return value of the haSendCommand
inertia                  = 2.5;           % Mass of the EndEffector
springConstant           = 150;           % spring value
dampFactor               = 0.1;           % damping factor for the spring
hmCalibrated             = 0;             % Isy HM calibrated?
ch                       = ' ';           % Key pressed in figure view

outputFileName = 'demo_';

homePositionVector = [-0.070949, 0.0,-0.197637 ];



%-------------------------------------------------------------------
% Initialization
%-------------------------------------------------------------------
% load HapticAPI2 library

if ( ~libisloaded('HapticAPI2') )
    loadlibrary( 'HapticAPI2' , 'HapticAPI2.h' );
end

disp('Opening device....');
[hapticMasterDeviceHandle, hapticMasterIPAddress] = calllib( 'HapticAPI2', 'haDeviceOpen', hapticMasterIPAddress );

if (hapticMasterDeviceHandle ~= -1)
    disp ( ['Connected to device ' hapticMasterIPAddress ])
    
    %-------------------------------------------------------------------
    % Check if HapticMASTER is position calibrated
    %-------------------------------------------------------------------
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'get position_calibrated' );
    if (retVal ~= 0)
        disp (['--- ERROR: ' response]);
    end
    
    %-------------------------------------------------------------------
    % If not, initialize HapticMASTER first (search end stops)
    %-------------------------------------------------------------------
    if ( strcmp(response,'"false"') )
        
        [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state init' );
        if (retVal ~= 0)
            disp (['--- ERROR: ' response]);
        end
        
        while (~hmCalibrated)
            [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'get state' );
            if (retVal ~= 0)
                disp (['--- ERROR: ' response]);
            end
            
            if ( strcmp(response,'"stop"') )
                hmCalibrated = 1;
            end
        end
    end
    
  
    %-------------------------------------------------------------------
    % STEP 1: Data playing
    %-------------------------------------------------------------------
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, ['set inertia ' num2str(inertia)] );
    [~, response] = haSendCommand( hapticMasterDeviceHandle, 'set state position ' );
    
    disp('Homing the device')
    if homePosition(hapticMasterDeviceHandle, homePositionVector) ~= 0
        break
    end
    disp('Homing finished')
    
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state stop ' );
    
    prompt = 'Do you want proceed to playing? y/n [y]: ';
    txt = input(prompt, 's');
    if isempty(txt)
        txt = 'y';
    end
    
    if txt ~= 'y'
        %-------------------------------------------------------------------
        % Terminate program execution
        %-------------------------------------------------------------------
        [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'remove all' );
        if (retVal ~= 0)
            disp (['--- ERROR: ' response]);
        else
            disp (response);
        end
        
        [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state stop' );
        if (retVal ~= 0)
            disp (['--- ERROR: ' response]);
        else
            disp (response);
        end
        
        retVal = calllib( 'HapticAPI2', 'haDeviceClose', hapticMasterDeviceHandle );
        break
    end
    
    % Create inertia
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, ['set inertia ' num2str(inertia)] );
    
    % Create a damper
    %     [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create damper myDamper');
    %     [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set myDamper dampcoef [0.0,0.0,10.0]');
    %     [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set myDamper enable');
    
    % Create a spring X
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringX');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX pos [', num2str(homePositionVector(1)), ',', num2str(homePositionVector(2)), ',', num2str(homePositionVector(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX stiffness ' num2str(springConstant)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX direction [1.0,0.0,0.0]');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX dampfactor ' num2str(dampFactor)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX maxforce 25.0');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX enable');
    
    % Create a spring Y
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringY');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY pos [', num2str(homePositionVector(1)), ',', num2str(homePositionVector(2)), ',', num2str(homePositionVector(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY stiffness ' num2str(springConstant)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY direction [0.0,1.0,0.0]');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY dampfactor ' num2str(dampFactor)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY maxforce 25.0');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY enable');
    
    % Create a spring Z
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringZ');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ pos [', num2str(homePositionVector(1)), ',', num2str(homePositionVector(2)), ',', num2str(homePositionVector(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ stiffness ' num2str(springConstant)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ direction [0.0,0.0,1.0]');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ dampfactor ' num2str(dampFactor)]);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ maxforce 25.0');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ enable');
    
    % Declare a real-time data logger
    [dataLoggerDeviceHandle, hapticMasterIPAddress] = calllib( 'HapticAPI2', 'haDataLoggerOpen', hapticMasterIPAddress );
    
    matrixColumnCount = 0;
    
    % Add scoped variables to the data logger
    
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, 'samplenr', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, 'sampletime', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.pos X', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.pos Y', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.pos Z', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.vel X', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.vel Y', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.vel Z', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.acc X', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.acc Y', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Model data.Cartesian.acc Z', matrixColumnCount );
    [~, ~, matrixColumnCount] = calllib( 'HapticAPI2', 'haDataLoggerAddParameter', dataLoggerDeviceHandle, '#HapticMASTER.Measured force.Force Sensor Z.output(scaled)', matrixColumnCount );
    
    % Start the data logger
    [retVal] = calllib( 'HapticAPI2', 'haDataLoggerStart', dataLoggerDeviceHandle );
    
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state position ' );
    
    %-----------------------------------
    % Trajectory Definition
    %-----------------------------------
    R=0.03;
    D=0.04;
    samplePoints=10000; 
    theta = linspace( pi/2, -pi/2, samplePoints);
    Line1 = [linspace(homePositionVector(1), D-homePositionVector(1), samplePoints); zeros(1,samplePoints); linspace(homePositionVector(3),homePositionVector(3),samplePoints)];
    CircleRight = [ R * cos(theta)+D-homePositionVector(1) ; zeros(1,samplePoints); (-R * sin(theta))+R+homePositionVector(3)];
    Line2 = [linspace(D-homePositionVector(1),homePositionVector(1),samplePoints); zeros(1,samplePoints); linspace((homePositionVector(3)+2*R),(homePositionVector(3)+2*R),samplePoints)];
    CircleLeft = [ -R * cos(theta)+homePositionVector(1); zeros(1,samplePoints); R * sin(theta)+R+homePositionVector(3)];
    trajectory = [[Line1,CircleRight],[Line2,CircleLeft]];
    
    %----------------------------------------------------------------------
    % Parameters for the optimal trajectory deformation algorithm
    %----------------------------------------------------------------------
    % Sampling time of the robot control loop
    T = 3e-3;
    
    % Sampling time for the trajectory deformation loop
    delta = 1e-2;
    
    % Duration of the interaction
    tau = 1;
    
    % Admittance of the optimal variation
    mu = 0.05;
    
    % Number of degrees-of-freedom for the robot
    num_dof = 3;
    
    % Stopping time for the robot control
    % t_stop = 20;
    
    % Simulation time vector
    % t_vec = (0:T:t_stop)';
    
    % Generate the original desired trajectory, x_d_star
    x_d_star = trajectory';
    
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
    
    completeMatrix_play = zeros(1,12,'double');
    springPositionVector = homePositionVector;
    outputVector = zeros(32000,1,'double');
    completeMatrix_record = trajectory;
    i = 1;
    F=[ones(length(completeMatrix_record)/4,1);zeros(length(completeMatrix_record)/4*3,1)];
    
    disp('Starting playing the recorded trajectory')
    
    tic
    tPrev = toc;
    while i <= size(completeMatrix_record,2)
        t=toc;
        
        [~, response] = haSendCommand( hapticMasterDeviceHandle, 'get measforce' );
        f_e = eval(response);
%         f_e = [0.0; 0.0; 0.0];
        
        tau_i = t;
        tau_f = tau_i + tau;
        x_d_curr = zeros(3, 1);
        
%         for j = 1:3
%             [x_d_curr(i), ~, gamma_d(:, j)] = optimalDeformation(delta, mu, gamma_d(:, i), f_e(i), H, tau_f, i);
%         end
        
        [x_d_curr_z, ~, gamma_d(:, 3)] = optimalDeformation(delta, mu, gamma_d(:, 3), f_e(3), H, tau_f);
        springPositionVector = [completeMatrix_record(1:2, i); x_d_curr_z];
        
%         springPositionVector = completeMatrix_record(1:3,i);
        
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        
        [retVal, outputVector] = calllib( 'HapticAPI2', 'haDataLoggerFlushVector', dataLoggerDeviceHandle, outputVector );
        outputMatrix = reshape ( outputVector(1:retVal*matrixColumnCount), retVal, matrixColumnCount );
        completeMatrix_play = vertcat ( completeMatrix_play, outputMatrix );
        i=i+200;
        
        t = toc;
        while t - tPrev < 1e-1
            t = toc;
        end
        tPrev = t;
    end
    
    disp('Finished playing the recorded trajectory')
 
    %-------------------------------------------------------------------
    % Terminate program execution
    %-------------------------------------------------------------------
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'remove all' );
    if (retVal ~= 0)
        disp (['--- ERROR: ' response]);
    else
        disp (response);
    end
    
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state stop' );
    if (retVal ~= 0)
        disp (['--- ERROR: ' response]);
    else
        disp (response);
    end
    
    retVal = calllib( 'HapticAPI2', 'haDeviceClose', hapticMasterDeviceHandle );
else
    disp ('Error, unable to connect to device')
end
%-----------------------------------
% Plotting
%-----------------------------------
subplot(2,1,1),hold on, grid on, xlabel('Time [s]'), ylabel('Position [m]')
plot(completeMatrix_record(1,:), completeMatrix_record(3,:))
plot(completeMatrix_play(2:end,3), completeMatrix_play(2:end,5), 'r--')
subplot(2,1,2),hold on, grid on, xlabel('Time [s]'), ylabel('Force [N]')
plot(completeMatrix_play(:,2),completeMatrix_play(:,12))
save demo2_multiple_loop.mat completeMatrix_record completeMatrix_play