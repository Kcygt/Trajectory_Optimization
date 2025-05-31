function  v = velocityFun(time, V_max)
% triangleVelocityProfile generates a triangular velocity profile.
% Inputs:
%   T     - total duration (seconds)
%   V_max - maximum velocity (peak of the triangle)
%   dt    - time step (e.g., 0.01)
% Outputs:
%   t - time vector
%   v - velocity profile (triangular shape)

    
    t_half = time(end)/2;
    
    v = zeros(size(time));
    
    for i = 1:length(time)
        if time(i) <= t_half
            % Increasing phase
            v(i) = (V_max / t_half) * time(i);
        else
            % Decreasing phase
            v(i) = V_max - (V_max / t_half) * (time(i) - t_half);
        end
    end
end
