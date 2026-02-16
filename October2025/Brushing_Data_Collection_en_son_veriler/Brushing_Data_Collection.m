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
springConstant           = 120;           % spring value
dampFactor               = 0.1;           % damping factor for the spring
hmCalibrated             = 0;             % Isy HM calibrated?
ch                       = ' ';           % Key pressed in figure view

outputFileName = 'demo_';

homePositionVector = [-0.0445844, 0.0263959 , -0.204109 ];
%-------------------------------------------------------------------
% Initialization
%-------------------------------------------------------------------
% Use getkey.m as a callback function from the current figure
% set( gcf, 'keypressfcn', 'LastKeyPressed = getkey;');
% LastKeyPressed = 0;

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
    
    % Home the robot for the demonstration
    
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, ['set inertia ' num2str(inertia)] );
    [~, response] = haSendCommand( hapticMasterDeviceHandle, 'set state force ' );
    
    disp('Homing the device')
    if homePosition(hapticMasterDeviceHandle, homePositionVector) ~= 0
        break
    end
    disp('Homing finished')
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'set state stop ' );
    
    prompt = 'Do you want proceed to demonstration? y/n [y]: ';
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
    
    %-------------------------------------------------------------------
    % STEP 1: Data recording
    %-------------------------------------------------------------------
    %-------------------------------------------------------------------
    % Create the haptic world (inertia, block, state force)
    %-------------------------------------------------------------------
    [retVal, response] = haSendCommand( hapticMasterDeviceHandle, ['set inertia ' num2str(inertia)] );
    
    [~, response] = haSendCommand( hapticMasterDeviceHandle, 'set state force ' );
    
    
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
    
    outputVector = zeros(6000,1,'double');
    completeMatrix_record = zeros(1,12,'double');
    %-------------------------------------------------------------------
    % Update EndEffector's position until ESC key is pressed
    %-------------------------------------------------------------------
    bShouldStop = 0;
    
    disp('Start the demonstration')
    while ( bShouldStop ~= 1 )
        [retVal, outputVector] = calllib( 'HapticAPI2', 'haDataLoggerFlushVector', dataLoggerDeviceHandle, outputVector );
        outputMatrix = reshape ( outputVector(1:retVal*matrixColumnCount), retVal, matrixColumnCount );
        completeMatrix_record = vertcat ( completeMatrix_record, outputMatrix );
        
        if ( outputMatrix (1,1) >= 6000 )
            bShouldStop = 1;
        end
    end
    
    % Stop the data logger
    [retVal] = calllib( 'HapticAPI2', 'haDataLoggerStop', dataLoggerDeviceHandle );
    
    % Save data
    csvwrite( [outputFileName, 'record', '.csv'], completeMatrix_record(2:end, :))
    
    %-------------------------------------------------------------------
    % STEP 2: Data playing
    %-------------------------------------------------------------------
    
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
    
    %     bShouldStop = 0;
    completeMatrix_play = zeros(1,12,'double');
    springPositionVector = homePositionVector;
    i = 2;
    
    disp('Starting playing the recorded trajectory')
    while i <= size(completeMatrix_record, 1)
        springPositionVector = completeMatrix_record(i, 3:5);
        
        [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ pos [', num2str(springPositionVector(1)), ',', num2str(springPositionVector(2)), ',', num2str(springPositionVector(3)), ']']);
        
        [retVal, outputVector] = calllib( 'HapticAPI2', 'haDataLoggerFlushVector', dataLoggerDeviceHandle, outputVector );
        outputMatrix = reshape ( outputVector(1:retVal*matrixColumnCount), retVal, matrixColumnCount );
        completeMatrix_play = vertcat ( completeMatrix_play, outputMatrix );
        
        i = i + 80;
    end
    
    disp('Finished playing the recorded trajectory')
    % Stop the data logger
    %     [retVal] = calllib( 'HapticAPI2', 'haDataLoggerStop', dataLoggerDeviceHandle );
    
    % Save data
    %     csvwrite( [outputFileName, 'play', '.csv'], completeMatrix_play(2:end, :))
    
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

figure(1), hold on, grid on, xlabel('Time [s]'), ylabel('Position [m]')
plot(completeMatrix_record(2:end, 2), completeMatrix_record(2:end, 5))
plot(completeMatrix_play(2:end, 2), completeMatrix_play(2:end, 5), 'r--')
save demo2_multiple_loop.mat completeMatrix_record completeMatrix_play