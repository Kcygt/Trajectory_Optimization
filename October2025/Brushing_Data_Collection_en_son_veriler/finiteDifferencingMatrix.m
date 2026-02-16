function A = finiteDifferencingMatrix(N)
% The finite differencing matrix
%
% N : Number of waypoints in the parameterization of the trajectory

% Initialize the matrix
A = zeros(N+3, N);

% Row index
i = 1;

for j=1:N
    A(i:i+3, j) = [1; -3; 3; -1];
    i = i + 1;
end

end


