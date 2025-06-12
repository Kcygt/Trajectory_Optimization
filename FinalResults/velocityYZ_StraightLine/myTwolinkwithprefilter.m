function dxdt = myTwolinkwithprefilter(t, x,qDes,tspan , wn1, wn2, ctrlPnt)
    zeta = [1 1 1];  % Damping ratio

    % Compute per-joint switching times
    t_Vmax = 0.7 ./ wn1;

    % Get initial control point joint angles via IK
    qCtrl = IK(ctrlPnt(1), ctrlPnt(2), ctrlPnt(3));

    % Initialize natural frequency and control target for each joint
    wn = zeros(1, 3);
    qControl = zeros(1, 3);

    % Per-joint switching logic based on t_Vmax
    for i = 1:3
        if t <= t_Vmax(i)
            wn(i) = wn1(i);
            qControl(i) = qCtrl(i);
        else
            wn(i) = wn2(i);
            qControl(i) = qDes(2, i);  % Desired joint angle after switch
        end
    end

    % Construct system matrices for prefilter dynamics
    A = [zeros(3), eye(3); -diag(wn).^2, -2 * diag(zeta) * diag(wn)];
    B = [zeros(3); diag(wn).^2];

    % Extract actual joint state
    q  = x(7:9);     % Joint angles
    qd = x(10:12);   % Joint velocities

    % PD control gains
    Kp = diag([70 70 70]);  
    Kd = diag([120 120 120]);  

    % Control law using prefiltered desired joint values
    controller = Kp * (x(1:3) - q) + Kd * (x(4:6) - qd);

    % Dynamics matrices
    [M, C, G] = compute_M_C_G(q(1), q(2), q(3), qd(1), qd(2), qd(3));

    % Compute torque and acceleration
    tau = M * controller + C * qd;
    qdd = M \ (tau - C * qd);

    % Return state derivative
    dxdt = [A * x(1:6) + B * qControl'; qd; qdd];
end
