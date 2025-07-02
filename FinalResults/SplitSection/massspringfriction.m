%% mass-spring-friction system
% Assumes spring is connected to x=0 so f=-kx
% 
% Unfortunately the way ode solvers works is that we can't just say
% fc=-K*sign(v), so need to change fc on each part of the cycle
%
% Theory is mass+spring+friction produces an elipse in the phase plane
%           mass+spring+damping gives a hard to characterise spiral

x0=[-.6 1.5];% [0;2.4];x0=x0(:); % start state m and m/s. Should be a column vector
fc=.7; % coulomb friction N
b=.1;
m=1; % mass
k=20;% stiffness N/m
max_ode_cycles=20;
maxtimespan=20; % assumes time starts at 0

%% Examples of events in ode
% https://uk.mathworks.com/help/matlab/math/ode-event-location.html
% Documents izhievent.m  play/Walking/myEvent.m
% C:\Users\wharwin\OneDrive - University of Reading\playwith\phaseportraits
% C:\Users\wharwin\OneDrive - University of Reading\playwith\MatthewPeterKelly-Bouncing_Ball_Matlab
% C:\Users\wharwin\OneDrive - University of Reading\Docstoshare\play\phaseportraits

[t,x,tcross]=mymbkf(m,k,0,fc,x0,max_ode_cycles,maxtimespan);
[t1,x1,tcross1]=mymbkf(m,k,b,0,x0,max_ode_cycles,maxtimespan);
figure(1);plot(t,x,t1,x1);grid on;title('Energy loss via friction or damping');xlabel('time (s)');legend({'x nl','v nl','x lin','v lin'})
figure(2);plot(x(:,1),x(:,2),fc/k*[1 -1],[0 0],'x',x1(:,1),x1(:,2));grid on;title('Energy loss via friction or damping');xlabel('x');ylabel('v')
snapnow
%% crossing times
diff(tcross);
%% Energy
figure(3)
%plot(t,0.5*k*x(:,1).^2+0.5*m*x(:,2).^2,t1,0.5*k*x1(:,1).^2+0.5*m*x1(:,2).^2);title('System stored Energy')
%% Energy in no loss system
[t2,x2,tcross2]=mymbkf(m,k,0,0,x0,max_ode_cycles,maxtimespan);
plot(t,0.5*k*x(:,1).^2+0.5*m*x(:,2).^2,t1,0.5*k*x1(:,1).^2+0.5*m*x1(:,2).^2,t2,0.5*k*x2(:,1).^2+0.5*m*x2(:,2).^2);title('System stored Energy')

%%
% publish('massspringfriction')
% 
% Note as of Sept 2024 the 'pdf' version in windows R2024a fails. The solution is create the
% html version and print from edge/google-chrome.
% publishWithRptgen('massspringfriction') fails to include the pictures
disp(pubinfo(mfilename))
