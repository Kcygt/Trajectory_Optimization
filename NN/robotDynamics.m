function dxdt = robotDynamics(t,x,params)

persistent phase

if t == 0
    phase = 1;
end

zeta = [1 1 1];

q = x(7:9);
qd = x(10:12);

[xNow,yNow,zNow] = FK(q(1),q(2),q(3));

xCurr = [xNow yNow zNow];

numControlPoints = size(params.qCtrl,1);

if phase <= numControlPoints

    dist = norm(xCurr - params.qCtrl(phase,:));

    if dist < 0.01
        phase = phase + 1;
    end

end

if phase <= numControlPoints

    qControl = params.qDes(phase,:);
    wn = params.wn{phase};

else

    qControl = params.qDes(end,:);
    wn = params.wn{end};

end

A = [zeros(3), eye(3);
    -diag(wn).^2, -2*diag(zeta)*diag(wn)];

B = [zeros(3);
    diag(wn).^2];

Kp = diag([70 70 70]);
Kd = diag([120 120 120]);

controller = ...
    Kp*(x(1:3)-q) + ...
    Kd*(x(4:6)-qd);

[M,C,G] = compute_M_C_G(...
    q(1),q(2),q(3),...
    qd(1),qd(2),qd(3));

	au = M*controller + C*qd;

qdd = M\(tau - C*qd);

prefilter = A*x(1:6) + B*qControl';

dxdt = [prefilter; qd; qdd];

end