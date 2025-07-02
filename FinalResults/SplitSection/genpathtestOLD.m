%% generalised path
startpoint=[0 0];
cp=[.05 .05; .01 .05; 0 0]; % control points
ts=[.5 .9 1.1]; % switch times
hsr=[]; %hyper sphere radius
k=20;b=.1;m=1;
maxtimespan=10;
%%
vsoptions = odeset('Events',@(t,y)hypersphereboundaryeventfn(t,y,ts(1)),'OutputSel',1,'Refine',4);

[t,x,te,ye,ie] = ode45(@(t,y)testfn(t,y,[cp(1,:) -k/m -b/m]) ,[0 maxtimespan],[0 0 startpoint],vsoptions); % upper
% tcross=te;
for jj=2:length(cp)
%    if ye(1)<-fc/k; %upper (positive velocity)
%    end
end


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
