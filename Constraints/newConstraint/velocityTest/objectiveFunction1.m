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