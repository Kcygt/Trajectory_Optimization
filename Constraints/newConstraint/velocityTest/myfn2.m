% should be able to call with 
%[tt3,yy3]=ode45(@myfn2,[0 4],[0;-1]);

function y=myfn2(t,x)
if t<.5
    wn=3;zeta=.6;
    A=[0 1;-wn^2 -2*zeta*wn];
else
    wn=1;zeta=1.6;
    A=[0 1;-wn^2 -2*zeta*wn];
end
y=A*x+[0;wn^2]*1; % step to 1
end