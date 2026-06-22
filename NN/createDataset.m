clear;
clc;
close all;

%% SETTINGS
numSamples = 2000;
numTargets = 5;
numControlPoints = 3;

X = [];
Y = [];
for k = 1:numSamples

    fprintf('Generating Sample %d / %d\n',k,numSamples);

    %% RANDOM TARGETS
    xTarget = generateRandomTargets(numTargets);

    %% CONTROL POINTS
    controlPointIndices = [1 3 5];
    xCtrl = xTarget(controlPointIndices,:);

    %% FINAL POSITION
    qFinal = [0 0 0];

    %% CONTROL POINT IK
    qCtrl = zeros(numControlPoints,3);

    for i = 1:numControlPoints
        initPrms = [initPrms xCtrl(i,:)];
    end

    %% BOUNDS
    lb = 0;
    ub = 5;

    for i = 1:(numControlPoints+1)
        lb = [lb 0.01*ones(1,3)];
        ub = [ub 10*ones(1,3)];
    end

    tolRad = 0.025;

    for i = 1:numControlPoints
        lb = [lb xCtrl(i,:) - tolRad];
        ub = [ub xCtrl(i,:) + tolRad];
    end

    %% WEIGHTS
    wt = [1000 10 0.0001];

    %% OPTIMIZATION
    objectiveFunc = @(params)objectiveFunction(...
        params,qDes,wt,xTarget,numControlPoints);

    options = optimoptions('fmincon',...
        'Display','none',...
        'Algorithm','sqp',...
        'MaxIterations',40);

    problem = createOptimProblem('fmincon',...
        'objective',objectiveFunc,...
        'x0',initPrms,...
        'lb',lb,...
        'ub',ub,...
        'options',options);

    ms = MultiStart('UseParallel',true,'Display','off');

    try

        [Opt,~] = run(ms,problem,3);

        %% DATASET INPUT
        xInput = xTarget(:)';

        %% DATASET OUTPUT
        yOutput = Opt;

        X = [X; xInput];
        Y = [Y; yOutput];

    catch

        fprintf('Sample failed\n');

    end

end

save('trajectoryDataset.mat','X','Y');

fprintf('Dataset generation completed.\n');