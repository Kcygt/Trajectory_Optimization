function xTarget = generateRandomTargets(numTargets)

xTarget = zeros(numTargets,3);

x0 = 0.01;

for i = 1:numTargets

    xTarget(i,1) = x0 + 0.03*(i-1);
    xTarget(i,2) = -0.05 + 0.04*rand;
    xTarget(i,3) = 0;

end

end