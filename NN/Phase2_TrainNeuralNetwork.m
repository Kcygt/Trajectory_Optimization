%% Phase2_TrainNeuralNetwork.m
% Trains a neural network policy (using RL Toolbox actor architecture)
% via supervised learning on the fmincon dataset generated in Phase 1.
%
% The network maps:
%   INPUT  -> flattened target positions  [1 x inputDim]
%   OUTPUT -> optimal parameters          [1 x outputDim]
%             (tspan, wn values, control point positions)
%
% OUTPUT: TrainedPolicy.mat  containing the trained network and normalisation stats

clear; clc; close all;

%% ===== LOAD DATA =====
load('TrainingData.mat');   % inputData, outputData, inputDim, outputDim
fprintf('Loaded %d samples  (inputDim=%d, outputDim=%d)\n', ...
        size(inputData,1), inputDim, outputDim);

%% ===== NORMALISE =====
% Normalise inputs and outputs to zero mean / unit variance.
% The inverse transform is applied at inference time.
inputMean  = mean(inputData, 1);
inputStd   = std(inputData,  0, 1) + 1e-8;   % avoid divide-by-zero
outputMean = mean(outputData, 1);
outputStd  = std(outputData,  0, 1) + 1e-8;

inputNorm  = (inputData  - inputMean)  ./ inputStd;
outputNorm = (outputData - outputMean) ./ outputStd;

%% ===== TRAIN / VAL SPLIT =====
rng(42);
N        = size(inputNorm,1);
splitIdx = floor(0.85 * N);
perm     = randperm(N);

trainIdx = perm(1:splitIdx);
valIdx   = perm(splitIdx+1:end);

XTrain = inputNorm(trainIdx,:)';    % [inputDim  x nTrain]
YTrain = outputNorm(trainIdx,:)';   % [outputDim x nTrain]
XVal   = inputNorm(valIdx,:)';
YVal   = outputNorm(valIdx,:)';

fprintf('Train: %d samples   Val: %d samples\n', numel(trainIdx), numel(valIdx));

%% ===== NETWORK ARCHITECTURE (RL Toolbox actor style) =====
% We build a dlnetwork — the same internal representation used by
% rlContinuousDeterministicActor — so the trained weights can be loaded
% directly into an actor if you later want to extend to online RL.

hiddenSize = [256, 256, 128];   % Three hidden layers

layers = [
    featureInputLayer(inputDim, 'Name','input', 'Normalization','none')

    fullyConnectedLayer(hiddenSize(1), 'Name','fc1')
    layerNormalizationLayer('Name','ln1')
    reluLayer('Name','relu1')

    fullyConnectedLayer(hiddenSize(2), 'Name','fc2')
    layerNormalizationLayer('Name','ln2')
    reluLayer('Name','relu2')

    fullyConnectedLayer(hiddenSize(3), 'Name','fc3')
    layerNormalizationLayer('Name','ln3')
    reluLayer('Name','relu3')

    fullyConnectedLayer(outputDim, 'Name','output')
    % No activation on the output — we predict normalised continuous values
    % and denormalise at inference time.
];

net = dlnetwork(layerGraph(layers));
fprintf('Network created:  %d parameters\n', sum(cellfun(@numel, {net.Learnables.Value{:}})));

%% ===== CUSTOM TRAINING LOOP =====
% We use a manual loop (rather than trainnet) for full control over
% learning rate scheduling, gradient clipping, and val-loss monitoring.

numEpochs      = 200;
miniBatchSize  = 32;
initLR         = 3e-4;
lrDropFactor   = 0.5;
lrDropEpochs   = 50;   % halve LR every N epochs
gradClipVal    = 1.0;
patience       = 30;   % early-stopping patience (epochs)

% Initialise Adam state
avgGrad   = [];
avgSqGrad = [];
iteration = 0;

bestValLoss  = inf;
waitCount    = 0;
bestWeights  = net.Learnables;

trainLossHist = zeros(numEpochs, 1);
valLossHist   = zeros(numEpochs, 1);

fprintf('\nStarting training...\n');
fprintf('%-8s %-14s %-14s %-10s\n','Epoch','Train MSE','Val MSE','LR');
fprintf('%s\n', repmat('-',1,50));

for epoch = 1:numEpochs

    % --- Learning rate schedule ---
    currentLR = initLR * lrDropFactor^floor((epoch-1)/lrDropEpochs);

    % --- Shuffle training data ---
    shuffIdx = randperm(size(XTrain,2));
    XTrain   = XTrain(:, shuffIdx);
    YTrain   = YTrain(:, shuffIdx);

    % --- Mini-batch loop ---
    nBatches   = floor(size(XTrain,2) / miniBatchSize);
    epochLoss  = 0;

    for b = 1:nBatches
        batchIdx = (b-1)*miniBatchSize + (1:miniBatchSize);
        XBatch   = dlarray(XTrain(:, batchIdx), 'CB');   % Channels x Batch
        YBatch   = YTrain(:, batchIdx);

        % Compute gradients via automatic differentiation
        [loss, grads] = dlfeval(@modelLoss, net, XBatch, YBatch);
        epochLoss     = epochLoss + extractdata(loss);

        % Gradient clipping
        grads = dlupdate(@(g) g .* min(1, gradClipVal/max(1e-8, norm(extractdata(g),'fro'))), grads);

        iteration = iteration + 1;
        [net, avgGrad, avgSqGrad] = adamupdate(net, grads, avgGrad, avgSqGrad, iteration, currentLR);
    end

    trainLoss = epochLoss / nBatches;

    % --- Validation loss ---
    XValDL  = dlarray(XVal, 'CB');
    YPredVal = extractdata(forward(net, XValDL));
    valLoss  = mean((YPredVal - YVal).^2, 'all');

    trainLossHist(epoch) = trainLoss;
    valLossHist(epoch)   = valLoss;

    % --- Early stopping ---
    if valLoss < bestValLoss
        bestValLoss = valLoss;
        bestWeights = net.Learnables;
        waitCount   = 0;
    else
        waitCount = waitCount + 1;
        if waitCount >= patience
            fprintf('\nEarly stopping at epoch %d (best val MSE=%.6f)\n', epoch, bestValLoss);
            break;
        end
    end

    % --- Print every 10 epochs ---
    if mod(epoch,10) == 0 || epoch == 1
        fprintf('%-8d %-14.6f %-14.6f %-10.2e\n', epoch, trainLoss, valLoss, currentLR);
    end
end

% Restore best weights
net.Learnables = bestWeights;
fprintf('\nTraining complete. Best val MSE: %.6f\n', bestValLoss);

%% ===== EVALUATION =====
XValDL    = dlarray(XVal, 'CB');
YPredNorm = extractdata(forward(net, XValDL));

% Denormalise predictions and ground truth
YPred = (YPredNorm' .* outputStd) + outputMean;
YTrue = (YVal'      .* outputStd) + outputMean;

% Per-output RMSE
rmsePerOutput = sqrt(mean((YPred - YTrue).^2, 1));
paramNames    = buildParamNames(numControlPoints);
fprintf('\nPer-parameter RMSE on validation set:\n');
for i = 1:outputDim
    fprintf('  %-30s  RMSE = %.5f\n', paramNames{i}, rmsePerOutput(i));
end

%% ===== PLOT TRAINING CURVES =====
figure;
semilogy(trainLossHist(trainLossHist>0), 'b-', 'LineWidth',1.5); hold on;
semilogy(valLossHist(valLossHist>0),     'r-', 'LineWidth',1.5);
xlabel('Epoch'); ylabel('MSE (normalised)'); grid on;
legend('Train','Validation'); title('Training curves');

%% ===== WRAP AS RL TOOLBOX ACTOR (optional, for future online RL) =====
% This lets you load the weights directly into an rlContinuousDeterministicActor
% if you later want to fine-tune with DDPG or TD3.
try
    obsInfo = rlNumericSpec([inputDim 1], 'LowerLimit',-inf, 'UpperLimit',inf);
    actInfo = rlNumericSpec([outputDim 1],'LowerLimit',-inf, 'UpperLimit',inf);

    actorNet  = net;   % already a dlnetwork
    actor     = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo, ...
                    'ObservationInputNames','input');
    fprintf('Wrapped as rlContinuousDeterministicActor — ready for future RL fine-tuning.\n');
catch ME
    fprintf('Actor wrap skipped (%s) — net saved directly.\n', ME.message);
    actor = [];
end

%% ===== SAVE =====
save('TrainedPolicy.mat', 'net', 'actor', ...
     'inputMean','inputStd','outputMean','outputStd', ...
     'inputDim','outputDim','numControlPoints', ...
     'trainLossHist','valLossHist','bestValLoss');
fprintf('Saved TrainedPolicy.mat\n');

%% ===== LOCAL FUNCTIONS =====

function [loss, grads] = modelLoss(net, X, Y)
    YPred = forward(net, X);
    loss  = mean((YPred - dlarray(Y)).^2, 'all');
    grads = dlgradient(loss, net.Learnables);
end

function names = buildParamNames(numCP)
    names = {'tspan'};
    for ph = 1:(numCP+1)
        for j = 1:3
            names{end+1} = sprintf('wn_phase%d_joint%d', ph, j);
        end
    end
    for cp = 1:numCP
        for ax = {'x','y','z'}
            names{end+1} = sprintf('ctrlPt%d_%s', cp, ax{1});
        end
    end
end
