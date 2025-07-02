%% generalised path
% Note 'genpath.m' is a matlab function
startpoint=[.01  0.02];
cp=[.01  0.02; .05 .06; .01 .05; 0.01 0.02]; % control points
cpall=[startpoint;cp];
ts=[0 1 2 8  ]; % switch times
hsr=[]; %hyper sphere radius
k=20;b=.50;m=1; % b values of 3 5 7 19
maxtimespan=20;
%%
vsoptions = odeset('Events',@(t,y)hypersphereboundaryeventfn(t,y,ts(1)),'OutputSel',1,'Refine',4);

[t,x,te,ye,ie] = ode45(@(t,y)testfn(t,y,[cp(1,:) k/m b/m]) ,[0 maxtimespan],[startpoint 0 0],vsoptions); % 
% tt=t;xx=x; % record path so far
for jj=2:length(cp)
    vsoptions = odeset('Events',@(t,y)hypersphereboundaryeventfn(t,y,ts(jj)),'OutputSel',1,'Refine',4);
    tnewstart=te+eps; % go slightly over the line
    ynewstart=ye+eps;
    [t1,x1,te,ye,ie] = ode45(@(t,y)testfn(t,y,[cp(jj,:) k/m b/m]) ,[tnewstart maxtimespan],ynewstart,vsoptions); % 
    t=[t; t1];x=[x;x1];
    if isempty(te);warning('time fail. Perhaps you ran out of time?');break;end
end

figure(1);plot(t,x);hold on
figure(2);plot(cpall(:,1),cpall(:,2),'d',x(:,1),x(:,2))

function dotx=testfn(t,x,params)
% Generalised trajectory generation
% impliments $\ddot{x}=fc/m-k/mx - b/m \dot{x}$
% states are [x v]^T (must be a column vector)
% params are [cp k/m b/m] (must be a row vector)

% m \dot{v}= -b v -k (x-cp) +f
% f=0
  bom=params(4);kom=params(3);cp=params(1:2);
  dotx=[x(3:4);-bom.*x(3:4)-kom.*(x(1:2)-cp')];
end

function [pos,isterminal,direction] = hypersphereboundaryeventfn(t,y,tend)
%pos = y(2); % The value that we want to be zero (i.e. velocity)
  pos=t-tend;
  isterminal = 1;  % Halt integration 
  direction = 0;   % The zero can be approached from either direction
end
