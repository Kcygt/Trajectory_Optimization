function gamma_d = lineSegment(x_d_star, startingIndex, N, delta, T)
% Slice the trajectory vector for a window (line segment) of size N
%
% x_d_star      : Original desired trajectory
% startingIndex : Value of the index for slicing the trajectory vector
% N             : Size of the window
% delta         : Sampling time of the parameterized line segment
% T             : Sampling time of the simulation


% Index increment
indexIncrement = delta / T;

% Stopping index
stoppingIndex = min(startingIndex+indexIncrement*(N-1), length(x_d_star));

gamma_d = x_d_star(startingIndex:indexIncrement:stoppingIndex);

% Check the size of the line segment (moving window) for zero padding
s = size(gamma_d);

if s(1) < N
    gamma_d = [gamma_d; gamma_d(end)*ones(N-s(1), 1)];
%     gamma_d = [gamma_d; zeros(N-s(1), 1)];
end

end
