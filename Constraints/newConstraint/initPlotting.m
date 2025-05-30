[ti, yi] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, tspan,  wn1,wn2,CtrlPnt), [0 tspan(2)], zeros(12, 1));

[x,y,z] = FK(yi(:,7),yi(:,8),yi(:,9));
figure; hold on; grid on;
plot(y,z,'.')
plot(xMid(1,2),xMid(1,3),'o')
plot(xMid(2,2),xMid(2,3),'o')
plot(xMid(3,2),xMid(3,3),'o')
plot(0,0,'.',0.05,0.05,'.')
plot(CtrlPnt(2),CtrlPnt(3),'d')