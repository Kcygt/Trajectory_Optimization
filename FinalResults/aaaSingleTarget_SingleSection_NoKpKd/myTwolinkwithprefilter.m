function dxdt= myTwolinkwithprefilter(t,x,qDes,t_st,zeta,wn,Kp,Kd)
    A1=[zeros(3), eye(3); -diag(wn).^2,-2*diag(zeta)*diag(wn)];
    B1=[zeros(3); diag(wn).^2];

    q=x(7:9);
    qd=x(10:12);

    Kp=diag(Kp);  
    Kd=diag(Kd);  

    controller=Kp*(x(1:3)-q)+Kd*(x(4:6)-qd);

    [M,C,G]=compute_M_C_G(q(1),q(2),q(3),qd(1),qd(2),qd(3));
    
    tau=M*(controller)+C*qd;
    
    qdd=M\(tau-C*qd);

    dxdt=[A1*x(1:6)+B1*qDes(:); qd; qdd];
end
