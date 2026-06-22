function qCtrl = extractQctrl(Opt,numControlPoints)

qCtrl = zeros(numControlPoints,3);

startIdx = 2 + (numControlPoints+1)*3;

for i = 1:numControlPoints

    idx = startIdx + (i-1)*3;

    qCtrl(i,:) = Opt(idx:idx+2);

end

end