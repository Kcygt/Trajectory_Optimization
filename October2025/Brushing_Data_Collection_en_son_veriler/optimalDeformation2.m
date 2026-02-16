function [x_d_curr, xp_d_curr, gamma_d_new] = optimalDeformation2(x_d_star, index, N, delta, T, gamma_d, f_h, H, mu)
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

x_d_curr = gamma_d_tilde(1);
xp_d_curr = (gamma_d_tilde(2) - gamma_d_tilde(1)) / delta;

tmpLineSegment = lineSegment(x_d_star, index + delta/T, N, delta, T);
x_new = tmpLineSegment(end);

gamma_d_new = [gamma_d_tilde(2:end); x_new];


end
