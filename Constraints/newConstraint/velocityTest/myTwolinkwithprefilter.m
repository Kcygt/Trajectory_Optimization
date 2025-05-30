% Dynamics Function with Prefilter
function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,wn1,wn2,ctrlPnt)
    zeta = [1 1 1];

    A1=[zeros(3), eye(3); -diag(wn1).^2,-2*diag(zeta)*diag(wn1)];
    B1=[zeros(3); diag(wn1).^2];
    
    A2=[zeros(3), eye(3); -diag(wn2).^2,-2*diag(zeta)*diag(wn2)];
    B2=[zeros(3); diag(wn2).^2];
    
    qCtrl = IK(ctrlPnt(1), ctrlPnt(2), ctrlPnt(3));

    q=x(7:9);
    qd=x(10:12);
    
    Kp = diag([70 70 70]);  
    Kd = diag([20 20 20]);  

    controller = Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);

    if t <= t_st(1)
        dxdt = [A1*x(1:6) + B1*qCtrl'; qd; qdd];
    else
        dxdt = [A2*x(1:6) + B2*qDes(2,:)'; qd; qdd];
    end
end