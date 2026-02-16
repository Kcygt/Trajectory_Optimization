function [x_d_curr, xp_d_curr, gamma_d_new] = optimalDeformation(delta, mu, gamma_d, f_h, H, tau_f, loopCounter,x_d_star,completeMatrix_play1,jj, index)
% Calculate the optimal trajectory deformation under applied human force
%
% delta    : Sampling time of the waypoint parameterization
% mu       : Admittance of the optimal variation
% gamma_d  : Segment of the original desired trajectory
% f_h      : Human force applied to the trajectory at a time instant
% H        : Positive definite matrix (gradient for the optimization)
% tau_f    : Timestamp for the ending point of the moving window
%
% x_d_curr : Position on the deformed trajectory
% xp_d_curr: Velocity on the deformed trajectory


% The deformed trajectory under human force
gamma_d_tilde = gamma_d + mu * delta * H * f_h;

% x_d_curr = gamma_d_tilde(1);

x_d_curr = mean(gamma_d_tilde);
% 
xp_d_curr = (gamma_d_tilde(2) - gamma_d_tilde(1)) / delta;

xOffset = [0.05; 0.0; -0.107 ];
% x_new = hippodrome(tau_f + delta, xOffset, loopCounter, x_d_star, completeMatrix_play1,jj);
x_new = hippodrome2(tau_f + delta, xOffset, loopCounter, x_d_star, completeMatrix_play1,jj);


x_new = x_new(3);

if nargin == 11
    gamma_d_new = [gamma_d_tilde(2:end); x_new(index)];
else
    gamma_d_new = [gamma_d_tilde(2:end); x_new];
end

end
