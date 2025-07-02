% [t,y,tcross]=MYMBKF(m,k,b,fc,x0,max_ode_cycles,maxtimespan)
% Solve the 1 dimensional mass-spring-damping-friction model
% 
% This should be identical to the version  in massspringfriction.m
% note that the position sign change event is not called (at the moment)
function [t,x,tcross]=mymbkf(m,k,b,fc,x0,max_ode_cycles,maxtimespan)
vsoptions = odeset('Events',@velsigneventfn,'OutputSel',1,'Refine',4);
%psoptions = odeset('Events',@possigneventfn,'OutputSel',1,'Refine',4);
fc0=sign(x0(2))*fc; % initial sign of friction based on starting velocity

[t,x,te,ye,ie] = ode45(@(t,y)massspringfrictionfn(t,y,[-fc0 -k -b]/m) ,[0 maxtimespan],x0,vsoptions); % upper
tcross=te;
for jj=2:max_ode_cycles
    if ye(1)<-fc/k; %upper (positive velocity)
      ye(2)=ye(2)+eps;
      [tu,xu,te,ye,ie] = ode45(@(t,y)massspringfrictionfn(t,y,[ -fc -k -b]/m) ,[te maxtimespan],ye,vsoptions); %upper
      t=[t; tu];x=[x;xu];
      if isempty(te);warning('upper fail. Perhaps you ran out of time?');break;end
      tcross(jj)=te;
    elseif ye(1)>fc/k %lower
      ye(2)=ye(2)-eps; 
      [tl,xl,te,ye,ie] = ode45(@(t,y)massspringfrictionfn(t,y,[ fc -k -b]/m) ,[te maxtimespan],ye,vsoptions);
      t=[t; tl];x=[x;xl];
      if isempty(te);warning('lower fail. Perhaps you ran out of time?');break;end
      tcross(jj)=te;
    else
        break
    end
end
% Minor bug. If the loop runs without the break, it reports one fewer cycle
disp(sprintf('cycles= %d/%d',jj-1,max_ode_cycles))
end



function dotx=massspringfrictionfn(t,x,params)
% Energy removal via friction
% Friction is controlled by the coulomb force (part of the params?)
% impliments $\ddot{x}=fc/m-k/mx - b/m \dot{x}$
% states are [x v]^T (must be a column vector)
% params are [fc k b]/m (must be a row vector)
dotx=[x(2);params*[1;x]];
end

function [position,isterminal,direction] = velsigneventfn(t,y)
position = y(2); % The value that we want to be zero (i.e. velocity)
isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction

end
function [position,isterminal,direction] = possigneventfn(t,y)
position = y(1); % The value that we want to be zero (i.e. velocity)
isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction
end
