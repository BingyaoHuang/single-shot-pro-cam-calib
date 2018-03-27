%% Simulate the four calibration methods using synthetic data, then generates statistical comparison results.
% This script runs syntheticCalib with different gaussian white noise added
% to different places, and each experiment is run 100 times to produce
% statistical comparison results.

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%%
clear; close all; clc;

%% Options
dataRoot = 'data';
dataName = 'simulation';
calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName));
calibInfo.dataName = dataName;

%% Set simulation parameters

% set noise type
noiseType = 4;

% noise max sigma
maxSigma = 1;

% noise step size
noiseStep = 0.02;

% noise vector
vecNoise = 0:noiseStep:maxSigma;

% number of noise sigmas
numSigma = length(vecNoise);

% number of repeats at each noise level
numTrials = 100;

% add gaussian white noise on projector image and multiple trial each noise level
disp('[Simulation] Gaussian white noise added to camera & projector image and checkerboard...');
disp(['Noise standard deviation range is 0 ~ ', num2str(maxSigma)]);

tic

% a cell array to store current sigma's results
curResults = cell(1, numTrials);

% a cell array to store all synthetic calibration results
finalResults = cell(numSigma, numTrials);

% main loop
for i = 1:numSigma
    parfor trial = 1:numTrials
        disp(['Noise sigma = ', num2str(vecNoise(i)), '    ', 'trial ', num2str(trial)]);
        curResults{1, trial} = Calibration.syntheticCalib(noiseType, vecNoise(i), calibInfo);
    end
    
    finalResults(i, :) = curResults;
end

totalTime = toc

%% Save results to mat
curDateTime = char(datetime('now', 'Format', 'MM-dd-y-HH-mm'));

fileName = [dataName, '_Type_', num2str(noiseType), ...
    '_Sigma_',num2str(vecNoise(1)), '-',num2str(noiseStep),'-', num2str(vecNoise(end)), ...
    '_Trials_', num2str(numTrials), '_Date_', curDateTime];

% save as mat
save([fileName, '.mat']);

%% Plot errors
close all

figIds = [];
figIds(1) = Calibration.plotErrors(finalResults, vecNoise, [1:4], [1, 4], 'General Errors');
figIds(2) = Calibration.plotErrors(finalResults, vecNoise, [1:4], [5, 12], 'Projector Intrinsics Errors');
figIds(3) = Calibration.plotErrors(finalResults, vecNoise, [1:4], [13, 20], 'Camera Intrinsics Errors');

%% Export figures
% save figures
SAVE_FIG = 0;

if (SAVE_FIG)
    savefig(figIds, [fileName, '.fig']);
    
    % export as png format
    exportPath = [];
    
    for i = 1:length(figIds)
        curFig = figure(figIds(i));
        figName = strrep(curFig.Name, ' ', '_');
        export_fig(curFig, [exportPath, figName, '.pdf'], '-transparent');
    end
end
