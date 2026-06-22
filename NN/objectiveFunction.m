function error = objectiveFunction(prms,qDes,wt,xTarget,numControlPoints)

qCtrl = extractQctrl(prms,numControlPoints);
wnParams = extractWn(prms,numControlPoints);

qCtrlIK = zeros(numControlPoints,3);

for i = 1:numControlPoints
    qCtrlIK(i,:) = IK(...
        qCtrl(i,1),...
        qCtrl(i,2),...
        qCtrl(i,3));
end

params.qDes = [qCtrlIK; qDes(end,:)];
params.qCtrl = qCtrlIK;
params.wn = wnParams;

[t,y] = ode15s(...
    @(t,x)robotDynamics(t,x,params),...
    0:0.005:prms(1),...
    zeros(12,1));

[x0,y0,z0] = FK(y(:,7),y(:,8),y(:,9));

xOut = [x0 y0 z0];

trackingError = 0;

for i = 1:size(xTarget,1)

    trackingError = trackingError + ...
        min(sum((xOut - xTarget(i,:)).^2,2));

end

finalError = sum((xOut(end,:) - [0 0 0]).^2);

timePenalty = prms(1);

error = ...
    wt(1)*trackingError + ...
    wt(2)*finalError + ...
    wt(3)*timePenalty;

end