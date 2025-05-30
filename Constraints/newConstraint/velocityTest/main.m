clear; clc;
close all;
% Get parameters
params

% see the initial results
% initPlotting

% Start the optimization
% Objective Function
objectiveFunc = @(params) objectiveFunction(params, qDes, wt, xMid, xDes);

% Run optimization
options = optimoptions('fmincon','PlotFcns', 'optimplot', 'Display', 'off', ... 
                        'TolCon', 1e-10); % Added constraint tolerance

% Create optimization problem
problem = createOptimProblem('fmincon',...
    'objective', objectiveFunc, ...
    'x0', initPrms, ...
    'lb', lb, ...
    'ub', ub, ...
    'options', options, ...
    'nonlcon', @(prms) trajConstraint(prms, qDes, xMid));

% MultiStart setup
ms = MultiStart('UseParallel', true, 'Display', 'iter');
numStarts = 5; % Number of random starting points

% Run MultiStart optimization
[Opt, fval] = run(ms, problem, numStarts);

% Simulate with optimal parameters
[tt, yy] = ode23s(@(t, x) myTwolinkwithprefilter(t, x, qDes, Opt(1:2),  Opt(3:5), Opt(6:8),Opt(9:11)), ...
                  [0 Opt(2)], zeros(12, 1));

% Final plotting
plotting



% Objective Function
function error = objectiveFunction(prms, qDes, wt, xMid, xDes)
    
    x0 = zeros(12, 1);
    % x0(1:3) = qDes(1,:);  & look at it to understand why you are using

    % Simulate the system
    [ttime, y] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:2), prms(3:5), prms(6:8), prms(9:11)), ...
                    [0 prms(2)], x0);

    [xOut,yOut,zOut] = FK(y(:,7),y(:,8),y(:,9));
    xOut = [xOut,yOut,zOut];
    
    % Calculate minimum distance to middle point
    dx1 = sum((xOut - xMid(1,:)).^2,2);
    [distMid1, idx1 ] = min(dx1);
    % distMid1 = dx1(idx1-5:idx1+5);
    
    dx2 = sum((xOut - xMid(2,:)).^2,2);
    [distMid2 , idx2 ] = min(dx2);
    % distMid2 = dx2(idx2-5:idx2+5);


    dx3 = sum((xOut - xMid(3,:)).^2,2);
    [distMid3, idx3 ] = min(dx3);
    % distMid3 = dx1(idx3-3:end);
    
    distMidF = sum(distMid1,1) + sum(distMid2,1)+ sum(distMid3,1);

    % actvel=y(1:3); actspeed=actvel'*actvel;
    %velPenaly = (actspeed - des)^2;

     % End point error
    dxEnd = sum((xOut(end,:) - xDes).^2,2);
    distEndErr = min(dxEnd);
    
    % Time penalty
    timePenalty = prms(2);

    % Composite error (normalized)
    error = wt(1) * distMidF    + ...
            wt(2) * distEndErr  + ...
            wt(3) * timePenalty ;
end
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


% Constraint Function for Midpoint Proximity
function [c, ceq] = trajConstraint(prms,qDes,xMid)
    ceq = []; % No equality constraints

    % Simulate trajectory
    [ttime, yy] = ode23s(@(t,x) myTwolinkwithprefilter(t,x,qDes,prms(1:2),prms(3:5),prms(6:8),prms(9:11)), ...
                    [0 prms(2)], zeros(12, 1));
    [x,y,z] = FK(yy(:,7),yy(:,8),yy(:,9));     % Optimized Trajectory
    x = [x,y,z];
    % Calculate distances to midpoint in 3D space
    distanceMid1  = sum((x - xMid(1,:)).^2,2);
    distanceMid2  = sum((x - xMid(2,:)).^2,2);
    distanceMid3  = sum((x - xMid(3,:)).^2,2);
    S = floor(length(yy)/5);
    % End point error
    distEndErr = sqrt(sum((x(end,:) - [0.05, 0.0,0.05]).^2,2));
    vel1 = min(abs(yy(S:end-S,10)));
    vel2 = min(abs(yy(S:end-S,11)));
    vel3 = min(abs(yy(S:end-S,12)));

    % Nonlinear inequality constraint: min distance <= 10cm (0.1m)
    c = [min(distanceMid1) - 0.00001;
         min(distanceMid2) - 0.00001;
         min(distanceMid3) - 0.00001;
         prms(1) - prms(2);
         distEndErr    - 0.0001]; 
end

% t = 0:0.1:5;
% v = sin(t);  % example velocity profile
% 
% a = diff(v)./diff(t);
% neg_indices = find(a < 0);
% 
% t_neg = t(neg_indices + 1);