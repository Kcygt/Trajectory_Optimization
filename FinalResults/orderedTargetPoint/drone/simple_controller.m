function [f, tau] = simple_controller(t, p, v, q, omega)
    % Desired hover at [0;0;1]
    pos_des = [0;0;1];
    vel_des = [0;0;0];
    yaw_des = 0; % not used here

    % Gains
    kp_pos = 4;
    kd_pos = 2;
    kp_att = 4;
    kd_att = 0.1;

    % Compute desired force in world frame
    e_p = pos_des - p;
    e_v = vel_des - v;
    F_des = kp_pos*e_p + kd_pos*e_v + [0;0;9.81];

    % Desired body z axis
    z_b_des = F_des / norm(F_des);

    % Compute desired rotation matrix: we can choose any x-y frame perpendicular to z_b_des
    x_c = [cos(yaw_des); sin(yaw_des); 0];
    y_b_des = cross(z_b_des, x_c); y_b_des = y_b_des/norm(y_b_des);
    x_b_des = cross(y_b_des, z_b_des);
    R_des = [x_b_des y_b_des z_b_des];

    % Current rotation matrix
    R = quat2rotm(q');

    % Rotation error
    R_err = 0.5 * (R_des' * R - R' * R_des);
    e_R = [R_err(3,2); R_err(1,3); R_err(2,1)];

    % Angular velocity error
    e_omega = -omega; % desired omega=0

    % Control
    f = norm(F_des);
    tau = kp_att * e_R + kd_att * e_omega;
end
