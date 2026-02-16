function  retVal = homePosition(hapticMasterDeviceHandle, x)

springConstant = 100;
dampFactor = 0.6;

% Create a damper
% [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create damper myDamper');
% [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set myDamper dampcoef [0.0,0.0,10.0]');
% [retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set myDamper enable');

% Create a spring X
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringX');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX pos [0.0,0.0,0.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX stiffness ' num2str(springConstant)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX direction [1.0,0.0,0.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX dampfactor ' num2str(dampFactor)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX maxforce 25.0');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringX enable');

% Create a spring Y
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringY');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY pos [0.0,0.0,0.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY stiffness ' num2str(springConstant)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY direction [0.0,1.0,0.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY dampfactor ' num2str(dampFactor)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY maxforce 25.0');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringY enable');

% Create a spring Z
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'create spring mySpringZ');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ pos [0.0,0.0,0.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ stiffness ' num2str(springConstant)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ direction [0.0,0.0,1.0]');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ dampfactor ' num2str(dampFactor)]);
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ maxforce 25.0');
[retVal, response] = haSendCommand(hapticMasterDeviceHandle, 'set mySpringZ enable');

t = 0;
tic
while ( t < 5 )
    t=toc;
    
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringX pos [', num2str(x(1)), ',', num2str(x(2)), ',', num2str(x(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringY pos [', num2str(x(1)), ',', num2str(x(2)), ',', num2str(x(3)), ']']);
    [retVal, response] = haSendCommand(hapticMasterDeviceHandle, ['set mySpringZ pos [', num2str(x(1)), ',', num2str(x(2)), ',', num2str(x(3)), ']']);
end

[retVal, response] = haSendCommand( hapticMasterDeviceHandle, 'remove all' );
if (retVal ~= 0)
    disp (['--- ERROR: ' response]);
else
    disp (response);
end

end
