% Create a sample mesh grid
[X, Y] = meshgrid(-5:0.1:5, -5:0.1:5);  % Create a grid of X and Y values

% Define Z as a function of X and Y (for example, a Gaussian function)
Z = exp(-(X.^2 + Y.^2));  % Gaussian function for illustration

% Create the contour plot
figure;
contour(X, Y, Z, 20);  % 20 contour levels
colorbar;  % Show colorbar to indicate the values

% Labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Isoheight (Contour) Plot');

