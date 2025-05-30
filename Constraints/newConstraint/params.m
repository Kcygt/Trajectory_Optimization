% Define desired trajectory and Middle Points
qDes = [0 0.1987 0.3278];
[xDes, yDes, zDes] = FK(qDes(1), qDes(2), qDes(3));
xDes = [xDes, yDes, zDes];

xMid = zeros(3,3);
xMid(1,:) = [ 0, 0.015, 0.005];
xMid(2,:) = [ 0, 0.03,  0.02];
xMid(3,:) = [ 0, 0.045, 0.035];

% Initial Parameters
tspan = [10 20];
wn1 = [1 1 1];
wn2 = [1 1 1];
CtrlPnt = [0,0.03,0.03];

qDes =[CtrlPnt;qDes];

% Weights
wt = [150,  5, 0.08];   % [Target, End, Time]

initPrms = [tspan, wn1, wn2, CtrlPnt];


% Lower and Upper Limits
lb = [0 0  ...         % time switching and end
      0.5 0.5 0.5 ...  % wn1
      0.5 0.5 0.5 ...  % wn2 
      0   0   0];      % Control point
ub = [5 5 ...          % time switching and end
      20 20 20 ...     % wn1
      20 20 20 ...     % wn2 
      0.06 0.06 0.06]; % Control point
