function [isFound, bestModelParams, inliers] = msac(...
    allPoints, params, funcs, varargin)
% MSAC M-estimator SAmple Consensus (MSAC) algorithm that is used for point
% cloud model fitting. allPoints must be an M-by-N matrix, where each point
% is a row vector.
%
% allPoints - M-by-2 or M-by-2-by-2 array of [x y] coordinates
%
% params    - struct containing the following fields:
%               sampleSize
%               maxDistance
%               confidence
%               maxNumTrials
%
% funcs     - struct containing the following function handles
%               fitFunc
%               evalFunc
%               checkFunc

% Copyright 2015 The MathWorks, Inc.
%
% References:
% ----------
%   P. H. S. Torr and A. Zisserman, "MLESAC: A New Robust Estimator with
%   Application to Estimating Image Geometry," Computer Vision and Image
%   Understanding, 2000.

%#codegen

confidence = params.confidence;
sampleSize = params.sampleSize;
maxDistance = params.maxDistance;

threshold = cast(maxDistance, 'like', allPoints);
numPts    = size(allPoints,1);
idxTrial  = 1;
numTrials = int32(params.maxNumTrials);
maxDis    = cast(threshold * numPts, 'like', allPoints);
bestDis   = maxDis;

if isfield(params, 'defaultModel')
    bestModelParams = params.defaultModel;
else
    bestModelParams = zeros(0, 'like', allPoints);
end
bestInliers = false(numPts, 1);

% Create a random stream. It uses a fixed seed for the testing mode and a
% random seed for other mode.
coder.extrinsic('vision.internal.testEstimateGeometricTransform');
if isempty(coder.target) && vision.internal.testEstimateGeometricTransform
    rng('default');
end

maxSkipTrials = params.maxNumTrials * 10;
skipTrials = 0;

prevMsgLength = 0;

while idxTrial <= numTrials && skipTrials < maxSkipTrials
    % Random selection without replacement
    indices = randperm(numPts, sampleSize);
    
    % Compute a model from samples
    samplePoints = allPoints(indices, :, :);
    modelParams = funcs.fitFunc(samplePoints, varargin{:});
    
    % Validate the model 
    isValidModel = funcs.checkFunc(modelParams, varargin{:});
    
    if isValidModel
        % Evaluate model with truncated loss
        dis = funcs.evalFunc(modelParams, allPoints, varargin{:});
        dis(dis > threshold) = threshold;
        accDis = sum(dis);

        % Update the best model found so far
        if iscell(modelParams)
            [minDis, minIdx] = min(accDis);
            if minDis < bestDis
                bestDis = minDis;
                bestInliers = dis(:, minIdx) < threshold;
                bestModelParams = modelParams{minIdx};
                inlierNum = cast(sum(dis(:, minIdx) < threshold), 'like', allPoints);
                num = computeLoopNumber(sampleSize, ...
                    confidence, numPts, inlierNum);
                numTrials = min(numTrials, num);
            end
        elseif ~isempty(accDis) && all(accDis < bestDis)
            bestDis = accDis;
            bestInliers = dis < threshold;
            bestModelParams = modelParams;
            inlierNum = cast(sum(dis < threshold), 'like', allPoints);
            num = vision.internal.ransac.computeLoopNumber(sampleSize, ...
                    confidence, numPts, inlierNum);
            numTrials = min(numTrials, num);
        end
        
        idxTrial = idxTrial + 1;
    else
        skipTrials = skipTrials + 1;
    end
end

isFound = funcs.checkFunc(bestModelParams, varargin{:}) && ...
    ~isempty(bestInliers) && sum(bestInliers(:)) >= sampleSize;
if isFound
    if isfield(params, 'recomputeModelFromInliers') && ...
            params.recomputeModelFromInliers
        modelParams = funcs.fitFunc(allPoints(bestInliers, :, :), varargin{:});
        
        dis = funcs.evalFunc(modelParams, allPoints, varargin{:});
        if iscell(modelParams)
            dis(dis >= threshold) = threshold;
            [~, minIdx] = min(sum(dis));
            if isempty(minIdx) || isempty(dis(:, minIdx))
                isFound = false;
                inliers = false(size(allPoints, 1), 1);
                return;
            end
            dis = dis(:, minIdx);
            bestModelParams = modelParams{minIdx};
        else
            bestModelParams = modelParams;
        end
        inliers = (dis < threshold);
        if ~any(inliers)
            isFound = false;
            return;
        end
    else
        inliers = bestInliers;
    end
    
    if isempty(coder.target) && numTrials >= int32(params.maxNumTrials)
        warning(message('vision:ransac:maxTrialsReached'));
    end
else
    inliers = false(size(allPoints, 1), 1);
end

