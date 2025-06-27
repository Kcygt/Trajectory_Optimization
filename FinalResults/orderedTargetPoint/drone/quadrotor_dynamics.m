function dX = quadrotor_dynamics(t, X, params, controller)
    % Unpack state
    p = X(1:3);          % position
    v = X(4:6);          % velocity
    q = X(7:10);         % quaternion
    omega = X(11:13);    % body rates

    % Normalize quaternion
    q = q / norm(q);

    % Convert quaternion to rotation matrix
    R = quat2rotm(q');

    % Desired control from your SE3 controller
    [f, tau] = controller(t, p, v, q, omega);

    % Translational dynamics
    dp = v;
    dv = [0; 0; -params.g] + (R * [0; 0; -f]) / params.m;

    % Rotational dynamics
    omega_cross = skew(omega);
    domega = params.J \ (tau - omega_cross * params.J * omega);

    % Quaternion kinematics
    Omega = [0, -omega'; omega, -skew(omega)];
    dq = 0.5 * Omega * q;

    % Concatenate derivatives
    dX = [dp; dv; dq; domega];
end

