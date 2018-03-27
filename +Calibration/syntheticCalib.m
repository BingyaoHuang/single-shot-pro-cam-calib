function ret = syntheticCalib(noiseType, noiseSigma, calibInfo)
%% Compare four calibration methods using synthetic data.
% Notations
% gt:
%     ground truth
% wb:
%     white board, also called calibration board where checkerboard is
%     attached to the center of it.
% cb: checkerboard
% sl:
%     structured light that is projected by the projector,
%     e.g, slPtsCamImg: structured light points in camera image space
% PR: proposed method
% DG: degraded proposed method (w/o BA)
% GH: generalized global homography method
% MT: Moreno & Taubin method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
disp('Calibrating...');

% for debug
verbose = 0;

format short

%% Noise description (noiseType)
% 0. No noise
% 1. Camera image noise
% 2. Projector image noise (before projection).
% 3. Calibration board noise.
% 4. All noises above

%% 1. Load ground truth parameters
% Load calibration data as synthetic params ground truth
gt = Calibration.loadGroundTruth('simulation_ground_truth.yml', calibInfo);

%% 2. Generate synthetic data using gt params
gt = Calibration.generateSyntheticData(gt, noiseType, noiseSigma, verbose);
synthData = gt.synthData;
PR = synthData.PR;
MT = synthData.MT;

%% 3. Calibrate camera using checkerboard points

% camParams is global homography and Moreno & Taubin method's camera
% paramter.
camParams = Calibration.calibrateInitGuess(synthData.cbPtsModel, synthData.cbPtsCamImg, calibInfo);

%% 4. Use calibrated cam params to undistort camera checkerboard and SL nodes

% # of calibration board poses
numSets = length(synthData.cbPtsCamImg);

% checkerboard points in projector image space
cbPtsPrjImgGH = cell(1, numSets);

% undistorted checkerboard and sl grid nodes in camera image space
cbPtsCamImgUndistort = cell(1, numSets);
slPtsCamImgUndistortPR = cell(1, numSets);

for i = 1:numSets
    % undistort checkerboard points in camera image
    cbPtsCamImgUndistort{i} = ImgProc.cvUndistortPoints(synthData.cbPtsCamImg{i}, camParams.camK, camParams.camKc);
    
    % undistort grid points in camera image
    slPtsCamImgUndistortPR{i} = ImgProc.cvUndistortPoints(PR.slPtsCamImg{i}, camParams.camK, camParams.camKc);
    
    % find a global homography between camera image space and projector
    % image space
    [Hcp, ~] = cv.findHomography(slPtsCamImgUndistortPR{i}, PR.slPtsPrjImg{i});
    
    % transform camera checkerboard points to projector image space using
    % global Hcp
    % NOTE: the cb corners have to be undistorted
    cbPtsPrjImgGH{i} = ImgProc.applyHomography(cbPtsCamImgUndistort{i}, Hcp);
end

%% 5. Compute SL nodes in world space and model space for PR and DG

% estimate grid points in world space using camera calibration tvecs and
% rvecs
slPtsModelPR = cell(1, numSets); % estimated grid points in model space
slPtsWorldPR = cell(1, numSets); % estimated grid points in world space

for i = 1:numSets
    % according to Zhang's method the homoraphy from cb model space is:
    % Hmc = lambda*K*[r1, r2, t], where r1, r2 are 1st and 2nd column of R
    R = cv.Rodrigues(camParams.rVecs{i});
    Hmc = camParams.camK * [R(:, 1:2), camParams.tVecs{i}];
    lambda = 1 / norm(inv(camParams.camK) * Hmc(:, 1));
    Hmc = lambda * Hmc;
    
    % transform camera image grid points to white board model space using
    % Hmc, then use calibrated tvecs and rvecs to transform grid points from
    % model space to world space.
    % NOTE: the cb corners have to be undistorted
    curSlPtsModelPR = ImgProc.applyHomography(slPtsCamImgUndistortPR{i}, inv(Hmc));
    slPtsModelPR{i} = [curSlPtsModelPR, zeros(length(curSlPtsModelPR), 1)];

    % transform grid points from model space to world space.
    slPtsWorldPR{i} = Calibration.rotateTranslatePoints(slPtsModelPR{i}, camParams.tVecs{i}', camParams.rVecs{i}, true);
end

%% 6. [Global Homography] Calibrate using checkerboard points and global homography

prjParamsGH = Calibration.calibrateInitGuess(synthData.cbPtsModel, cbPtsPrjImgGH, calibInfo);

% 2. Stereo calibration using OpenCV
stereoParamsGH = Calibration.calibrateStereoInitGuess(synthData.cbPtsModel, synthData.cbPtsCamImg, cbPtsPrjImgGH, camParams, prjParamsGH, calibInfo);
stereoParamsGH.sets = calibInfo.sets;
stereoParamsGH.dataName = calibInfo.dataName;

% 3. Calculate reprojection errors
stereoParamsGH = Calibration.calcReprojectionError(stereoParamsGH, synthData.cbPtsModel, synthData.cbPtsCamImg, cbPtsPrjImgGH);

if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsGH.worldPts'), stereoParamsGH.R, stereoParamsGH.T, 'Global homography calibration: Xm ');
end

%% 7. Subsample SL nodes to save computation time and RAM

for i = 1:length(slPtsModelPR)
    idx = round(linspace(1, length(slPtsModelPR{i}), 500));
    slPtsModelPR{i} = slPtsModelPR{i}(idx, :);
    PR.slPtsCamImg{i} = PR.slPtsCamImg{i}(idx, :);
    PR.slPtsPrjImg{i} = PR.slPtsPrjImg{i}(idx, :);
    
    % also do the same to slPtsWorldPR
    slPtsWorldPR{i} = slPtsWorldPR{i}(idx, :);
end

%% 8. [Degraded proposed] w/o BA, use DG for short
% 1. Use 'withoutBA' option to specify no bundle adjustment on Xm
stereoParamsDG = Calibration.stereoCalibrate(slPtsModelPR, PR.slPtsCamImg, PR.slPtsPrjImg, calibInfo.camImgSize, calibInfo.prjImgSize, 'withoutBA');
stereoParamsDG.sets = calibInfo.sets;
stereoParamsDG.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParamsDG = Calibration.calcReprojectionError(stereoParamsDG, stereoParamsDG.modelPts, stereoParamsDG.camImgPts, stereoParamsDG.prjImgPts);

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsDG.worldPts'), stereoParamsDG.R, stereoParamsDG.T, 'Propsoed w/o BA calibration: Xm ');
end

%% 9. [proposed] with BA, use PR for short
% 1. Use 'BA' option to specify bundle adjustment on Xm
stereoParamsPR = Calibration.stereoCalibrate(slPtsModelPR, PR.slPtsCamImg, PR.slPtsPrjImg, calibInfo.camImgSize, calibInfo.prjImgSize, 'BA');
stereoParamsPR.sets = calibInfo.sets;
stereoParamsPR.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParamsPR = Calibration.calcReprojectionError(stereoParamsPR, stereoParamsPR.modelPts, stereoParamsPR.camImgPts, stereoParamsPR.prjImgPts);

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsDG.worldPts'), stereoParamsDG.R, stereoParamsDG.T, 'Propsoed calibration: Xm ');
end

%% 10. Compute local homographies and warp corners for Moreno & Taubin method

% same cb corners as GH method
cbPtsModelMT = synthData.cbPtsModel;
cbPtsCamImgMT = synthData.cbPtsCamImg;

% warped checkerboard corners in projector image space (use local homographies)
cbPtsPrjImgMT = [];

% local homography windows size (default parameter)
winSize = 40/2;

% number of cb corners
numCorners = calibInfo.numCorners;

% for each sets (calibration board pose)
for i = 1:numSets
    cbPtsPrjImgMT{i} = zeros(numCorners, 2);
    
    % undistort grid points in camera image
    curSlPtsCamImgUndistortMT = ImgProc.cvUndistortPoints(MT.slPtsCamImg{i}, camParams.camK, camParams.camKc);
    curSlPtsPrjImgMT = MT.slPtsPrjImg{i};
    
    % for each cb corner
    for j = 1:numCorners
        
        % The current checkerboard point
        curCbPtsCamImgRoundMT = round(cbPtsCamImgUndistort{i}(j, :));
        
        % only use the SL points that are within winSize range of cb point
        % to estimate local homographies
        i1 = curSlPtsCamImgUndistortMT(:, 1) < curCbPtsCamImgRoundMT(1) + winSize;
        i2 = curSlPtsCamImgUndistortMT(:, 1) > curCbPtsCamImgRoundMT(1) - winSize;
        i3 = curSlPtsCamImgUndistortMT(:, 2) < curCbPtsCamImgRoundMT(2) + winSize;
        i4 = curSlPtsCamImgUndistortMT(:, 2) > curCbPtsCamImgRoundMT(2) - winSize;
        inlierIdx = i1 & i2 & i3 & i4;
        
        if (nnz(inlierIdx) < 4)
            warning('Not enough points to fit a homography, the camera calibration is not very accurate');
            cbPtsPrjImgMT{i}(j, :) = nan;
        else
            % 2d grid points in camera and projector image that are within the window
            curSlPtsCamImgWin = curSlPtsCamImgUndistortMT(inlierIdx, :);
            curSlPtsPrjImgWin = curSlPtsPrjImgMT(inlierIdx, :);
            
            % calculate local homography
            Hcp = cv.findHomography(curSlPtsCamImgWin, curSlPtsPrjImgWin);
            
            % transform camera checkerboard corners to projector image
            % space using Hcp
            cbPtsPrjImgMT{i}(j, :) = ImgProc.applyHomography(cbPtsCamImgUndistort{i}(j, :), Hcp);
        end
    end
end

% get rid of nan if fail to estimate a local homogrphy
for i = 1:numSets
    nanIdx = sum(isnan(cbPtsPrjImgMT{i}), 2) > 0;
    
    cbPtsModelMT{i}(nanIdx, :) = [];
    cbPtsCamImgMT{i}(nanIdx, :) = [];
    cbPtsPrjImgMT{i}(nanIdx, :) = [];
end

%% 11. [Moreno & Taubin method] use MT for short

% 1. Calibrate projector using local homography warped checkerboard corners
prjParamsMT = Calibration.calibrateInitGuess(cbPtsModelMT, cbPtsPrjImgMT, calibInfo);

% 2. Stereo calibration using OpenCV
stereoParamsMT = Calibration.calibrateStereoInitGuess(cbPtsModelMT, cbPtsCamImgMT, cbPtsPrjImgMT, camParams, prjParamsMT, calibInfo);
stereoParamsMT.sets = calibInfo.sets;
stereoParamsMT.dataName = calibInfo.dataName;

% 3. Calculate reprojection errors
stereoParamsMT = Calibration.calcReprojectionError(stereoParamsMT, cbPtsModelMT, cbPtsCamImgMT, cbPtsPrjImgMT);

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsMT.worldPts'), stereoParamsMT.R, stereoParamsMT.T, 'Moreno & Taubin calibration: Xm ');
end

%% 12. [debug] Compare GH and MT warped checkerboard points in projector image space
if (verbose)
    % MT is + and GH is o
    figure;
    title('warped checkerboard corners in projector image, MT is + and GH is o');
    hold on;
    planeColors = hsv(numSets);
    
    for i = 1:numSets
        plot(cbPtsPrjImgGH{i}(:, 1), cbPtsPrjImgGH{i}(:, 2), 'o', 'Color', planeColors(i, :));
        plot(cbPtsPrjImgMT{i}(:, 1), cbPtsPrjImgMT{i}(:, 2), '+', 'Color', planeColors(i, :));
    end
end

%% 9. Visualize all error metrics
[errMT, stereoParamsMT] = Calibration.showErrors(stereoParamsMT, gt, 'Moreno & Taubin');
[errGH, stereoParamsGH] = Calibration.showErrors(stereoParamsGH, gt, 'Global homography');
[errDG, stereoParamsDG] = Calibration.showErrors(stereoParamsDG, gt, 'Proposed w/o BA');
[errPR, stereoParamsPR] = Calibration.showErrors(stereoParamsPR, gt, 'Proposed');

%% Convert err to table
ret = [errMT; errGH; errDG; errPR];
resultsTable = struct2table(ret);
disp(resultsTable);

end
