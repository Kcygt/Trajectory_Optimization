function wnParams = extractWn(Opt,numControlPoints)

wnParams = cell(numControlPoints+1,1);

startIdx = 2;

for i = 1:(numControlPoints+1)

    idx = startIdx + (i-1)*3;

    wnParams{i} = Opt(idx:idx+2);

end

end