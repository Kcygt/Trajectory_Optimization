function sendTrigger(msg)
% Interactively send a message to ANSWER using memmapfile class.

filename = fullfile('.', 'trigger.dat');

% Create the communications file if it is not already there.
if ~exist(filename, 'file')
    [f, msg] = fopen(filename, 'wb');
    if f ~= -1
        fwrite(f, zeros(1,256), 'uint8');
        fclose(f);
    else
        error('MATLAB:demo:send:cannotOpenFile', ...
            'Cannot open file "%s": %s.', filename, msg);
    end
end

% Memory map the file.
m = memmapfile(filename, 'Writable', true, 'Format', 'uint8');

if (strcmp(msg, 'start'))
    disp('Sending trigger to start the data logger')
    m.Data(1) = '1';
else
    disp('Sending trigger to stop the data logger')
    m.Data(1) = '0';
end

end