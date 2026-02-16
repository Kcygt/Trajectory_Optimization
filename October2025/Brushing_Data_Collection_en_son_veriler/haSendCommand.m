% haSendCommand - Send a string command to the HapticMASTER
% ---------------------------------------------------------------------
% File    : haSendCommand.m
%
% Purpose : Send a string command to the HapticMASTER
%
% Parameters:
%       deviceHandle    - A handle to the HapticMASTER device
%       command         - The string command to be sent to the device
%
% Returns:
%       retVal          - Return value of the function execution
%       response        - The string returned from the function
%
% History:
%  2010-11-08  EH    created
% ---------------------------------------------------------------------
function [retVal, response]=haSendCommand(deviceHandle, command)

retVal = 0;

% Initialize the 2 output strings to be long enough
% If the strings are initialized shorter, problems occure when the string
% is too short to accept the returned string
dummyStr = '                                                                                                ';
response = '                                                                                                ';

% Calling the haSendString function in the HapticAPI2 library
[retVal, dummyStr, response] = calllib ('HapticAPI2', 'haDeviceSendString', deviceHandle, command, response);
