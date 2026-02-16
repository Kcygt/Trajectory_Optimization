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
springConstant           = 100;           % spring value
dampCoefficient          = [90.0,90.0, 90.0]; % Damp Coefficient
dampFactor               = 0.0;           % damping factor for the spring
hmCalibrated             = 0;             % Isy HM calibrated?
ch                       = ' ';           % Key pressed in figure view


outputFileName = 'demo_';

homePositionVector = [0.05, 0.0, -0.107 ]';

% Admittance of the best variation
mu =1.5;


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
    % STEP 1: Homing the device
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
    % Create a damper
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create damper myDamper');
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set myDamper dampcoef [', num2str(dampCoefficient(1)), ',', num2str(dampCoefficient(2)), ',', num2str(dampCoefficient(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set myDamper enable');
    
    
    % Create inertia
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, ['set inertia ' num2str(inertia)] );
    
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
    
    
    %----------------------------------------------------------------------
    % Parameters for the optimal trajectory deformation algorithm
    %----------------------------------------------------------------------
    % Sampling time of the robot control loop
    %     T = 1e-3;
    T = 1/2048;
    
    
    % Sampling time for the trajectory deformation loop
    %     delta = 5e-2;
    delta = T*40;
    
    
    % Duration of the interaction
    tau =delta * 5;
    
    
    
    % Number of degrees-of-freedom for the robot
    num_dof = 3;
    
    % Stopping time for the robot control
    t_stop = 32;
    %     t_stop = 32;
    % Simulation time vector
    t_vec = (0:T:t_stop)';
    
    % t_vec = linspace(0,16,32769)';
  
    completeMatrix_play1 = zeros(1,12,'double');
    trajectory_1 = zeros(1,12,'double');
    trajectory_2 = zeros(1,12,'double');
    trajectory_3 = zeros(1,12,'double');


    jj=1; % The recorded trajectory index in z-direction
    
    loopCounter=0; % the loop of the robot/trajectory
    
%     %%%% Experimental 3D surface
%         R = [  0.8944666,  0.0000000, -0.4471347;
%                0.0000000,  1.0000000,  0.0000000;
%                0.4471347,  0.0000000, 0.8944666 ];
%     
    
    %%%%%% no rotation
    R = [  1.0,  0.0,  0.0;
        0.0,  1.0,  0.0;
        0.0,  0.0,  1.0 ];
    % Generate the original desired trajectory, x_d_star
    x_d_star = zeros(length(t_vec), 3);
    x_d_star2 = zeros(length(t_vec), 3);
    
    f_d_star = zeros(length(t_vec), 3);
    for i = 1:length(t_vec)
%         [xTmp, fTmp] = hippodrome(t_vec(i), homePositionVector,loopCounter,x_d_star,completeMatrix_play1,jj);
                [xTmp, fTmp] = hippodrome2(t_vec(i), homePositionVector,loopCounter,x_d_star,completeMatrix_play1,jj);
        
        x_d_star(i, :) = xTmp';
        %         if i<8100
        x_d_star2(i, :) = xTmp'*R';
        %         else
        %         x_d_star2(i, :) = xTmp'*R3D';
        %         end
        f_d_star(i, :) = fTmp';
    end
    x_offset = -x_d_star2(1,:)+x_d_star(1,:);
    
    for i = 1:size(x_d_star,1)
        x_d_star2(i, :) = x_d_star2(i, :) + x_offset;
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
    H = (sqrt(N) / norm(G)) * G;
    
    X_curr = [];
    
    gamma_d_x = lineSegment(x_d_star(:, 1), 1, N, delta, T);
    gamma_d_y = lineSegment(x_d_star(:, 2), 1, N, delta, T);
    gamma_d_z = lineSegment(x_d_star(:, 3), 1, N, delta, T);
    gamma_d = [gamma_d_x, gamma_d_y, gamma_d_z];
    
    completeMatrix_play = zeros(1,12,'double');
    springPositionVector = homePositionVector;
    outputVector = zeros(65000,1,'double');
    
    %     disp('Starting playing the recorded trajectory')
    disp('hippodrome trajectory is running')
    X = [];
    tic
    t = toc;
    tPrev = t;
    while t <= 32
        [~, response] = haSendCommand( hapticMasterDeviceHandle, 'get measforce' );
%         f_e = eval(response)';
        f_e = [0.0; 0.0; 0.0];
        
        
        if loopCounter == 0  % reset the loopcounter
            t=toc;         
        end
        
        
        tau_i = t;
        tau_f = tau_i + tau;
        x_d_curr = zeros(3, 1);
        
        
%         [springPositionVector, fd] = hippodrome(t, homePositionVector, loopCounter, x_d_star, completeMatrix_play1,jj);
        [springPositionVector, fd] = hippodrome2(t, homePositionVector, loopCounter, x_d_star, completeMatrix_play1,jj);
        
        jj= jj+42;  % z direction index for recorded trajectory
        f_err = f_e - fd;
        
        [x_d_curr_z, ~, gamma_d(:, 3)] = optimalDeformation(delta, mu, gamma_d(:, 3), f_err(3), H, tau_f, loopCounter,x_d_star,completeMatrix_play1,jj);
        springPositionVector(3) = x_d_curr_z;
        
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [~, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        
        [retVal, outputVector] = calllib( 'HapticAPI2', 'haDataLoggerFlushVector', dataLoggerDeviceHandle, outputVector );
        outputMatrix = reshape ( outputVector(1:retVal*matrixColumnCount), retVal, matrixColumnCount );
        completeMatrix_play = vertcat ( completeMatrix_play, outputMatrix );
        
        
        t = toc;
        while t - tPrev < delta
            t = toc;
        end
        tPrev = t;
        
        % save the following trajectory and control the time and loop and also plotting
        
        if t > 32 
            %-----------------------------------
            % Plotting
            %-----------------------------------
            subplot(2,1,1),hold on, grid off, xlabel('x-Direction [m]'), ylabel('z-Direction [m]')
            plot(completeMatrix_play(2:end,3), completeMatrix_play(2:end,5),'b--','LineWidth',2.5);
%             legend('The first trajectory');hold on
            subplot(2,1,2),hold on, grid on, xlabel('Time [s]'), ylabel('Force [N]')
            plot(t_vec, f_d_star(:, 3));
            plot(completeMatrix_play(2:end,2),completeMatrix_play(2:end,12), 'r--');
%             legend([p5],{'Th eprevious Mesaured Force'});hold on
            disp(sprintf('loop number %d',loopCounter+1))
            
            
            if loopCounter == 0
                trajectory_first = completeMatrix_play;
                
            elseif loopCounter == 1
                trajectory_second= completeMatrix_play;
                
            else 
                trajectory_third = completeMatrix_play;
            end 
            
            loopCounter = loopCounter + 1;
            completeMatrix_play1 = completeMatrix_play;
            completeMatrix_play = zeros(1,12,'double');
            
            t = 0;
            jj=1;
            tic
            tPrev = toc;
        end
        
        
        if  loopCounter == 1 % how many loop the robot run
            break
        end
    end
    pause(1)
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

subplot(2,1,1),hold on, grid off, xlabel('x-Direction [m]'), ylabel('z-Direction [m]')
plot(completeMatrix_play1(2:end,3), completeMatrix_play1(2:end,5),'r','LineWidth',2.5)
plot(x_d_star(:,1), x_d_star(:,3),'g','LineWidth',2.5)
plot(x_d_star2(1:16000,1), x_d_star2(1:16000,3),'k','LineWidth',2)
legend('The Previous Trajectories Followed','','','The Last Trajectory Followed','The First Given Trajectories ','The Surface')
subplot(2,1,2),hold on, grid on, xlabel('Time [s]'), ylabel('Force [N]')
p1=plot(t_vec, f_d_star(:, 3),'LineWidth',2);
plot(t_vec+32, f_d_star(:, 3),'LineWidth',2)
plot(t_vec+64, f_d_star(:, 3),'LineWidth',2)
p2=plot(completeMatrix_play1(2:end,2),completeMatrix_play1(2:end,12), 'r--');
legend([p1 p2],{'The Desired Force','The Measured Force'})
save The_last_declined_ramp_surface_failed.mat