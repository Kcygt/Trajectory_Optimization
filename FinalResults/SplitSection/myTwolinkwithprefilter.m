function dxdt = myTwolinkwithprefilter(t, x,qDes,tspan , wn1, xCtrl1 )
    zeta = [1 1 1];  % Damping ratio

     % Initialize natural frequency and control target for each joint
    % qCtrl = zeros(1,3);
    % Per-joint switching logic based on t_Vmax
    qCtrl = IK(xCtrl1(1), xCtrl1(2), xCtrl1(3));

    wn = wn1;
    qControl = qCtrl(1,:);
     
    % if t <= tspan(1)
    %     wn = wn1;
    %     qControl = qCtrl(1,:);
    % elseif t > tspan(1) && t <= tspan(2)
    %     wn = wn2;
    %     qControl = qCtrl(2,:);
    % else
    %     wn = wn3;
    %     qControl = qDes(end, :);  % Desired joint angle after switch
    % end


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

