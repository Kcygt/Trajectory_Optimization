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
