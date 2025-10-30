load('Pdata1.mat')
i = 1:5000;
force = Pdata1(i,6);   % Extract force values
position = y(i);       % Extract displacement values

T = table(i', position, force, 'VariableNames', {'Index', 'Displacement', 'Force'});
disp(T);


x = position - position(1);    % Displacement relative to first position
F = force;

% Avoid division by zero
valid = x ~= 0;
k = F(valid) ./ x(valid);      % Calculate k where displacement is non-zero

% Optionally take the average spring constant (assuming linearity)
k_mean = mean(k);

% Display result
fprintf('Estimated average spring constant k = %.4f N/m\n', k_mean);
