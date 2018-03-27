function [gt] = generateSyntheticData(gt, noiseType, noiseSigma, verbose)
%% Generate synthetic calibration data using ground truth parameters.
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
% perturbation added to control points
lambda = 20; 

%% Create calibration boards in world space
cbSize = gt.calibInfo.boardSize; % number of bw squares in X and Y

% Total number of checkerboard points
numCorners = gt.calibInfo.numCorners;

% Calibration board size in mm
inch2mm = 20.54; % inches to mm

% white board (calibration board) width and height in mm.
wbW = 30*inch2mm;
wbH = 20*inch2mm;

% Create centered calibration board at wb model space origin (4 points),
% defined as bottom right corner as the 1st point and counterclockwise
% order from c1 to c4
% Coord system is the same as OpenCV +X is to the right , +Y is to the down

% X and Y is flipped here to be consistent with Calibration-CV-3.3-091417
% data
wbCornersCentered = [wbH/2  wbW/2 0;
    -wbH/2  wbW/2 0;
    -wbH/2 -wbW/2 0;
    wbH/2 -wbW/2 0];

% Checkerboard corners in real model space
cbPtsModel = generateCheckerboardPoints(cbSize, gt.calibInfo.sqSize);
cbPtsModel = [cbPtsModel(:,2) cbPtsModel(:,1)];

% convert to 3d by setting z to 0
cbPtsModel3d = [cbPtsModel, zeros(length(cbPtsModel), 1)];

% find checkerboard center coord in model space, in cb model space, the top
% left is cb model space origin (0,0)
cbCenter = max(cbPtsModel3d) / 2;

% then we need to calculate white board's 4 corners in cb model space, note
% wb center is the same as cb center, so we have:
wbCornersModel = wbCornersCentered + cbCenter;

% Loads the board R, T from existing calib to save simulation time
camRvecs = cell2mat(gt.camRvecs)';
camTvecs = cell2mat(gt.camTvecs)';

prjRvecs = cell2mat(gt.prjRvecs)';
prjTvecs = cell2mat(gt.prjTvecs)';

% number of calibration board poses
numSets = gt.calibInfo.numSets;

%% Generate synthetic checkerboard & calibration board points.
% Allocate variables

wbCornersCamView = zeros(4, 3, numSets); % white board's 4 corners in camera view space
cbPtsCamView = zeros(numCorners, 3, numSets); % checkerboard points in camera view space
cbPtsCamImg = zeros(numCorners, 2, numSets); % checkerboard points in camera image space

% noisy checkerboard board (cb) corners
cbPtsModel3dNoisy = cbPtsModel3d;

if(noiseType == 3 || noiseType == 4)
    % add Gaussian white noise to checkerboard boarad 3d points
    cbPtsModel3dNoisy = cbPtsModel3d + noiseSigma*randn(size(cbPtsModel3d));
end

for i = 1:numSets
    % Generate rotated and translated plane points, since we use
    % camParams's rotation and translation matrix, they are to translate
    % points in cb model space to camera view space, so both wb corners and
    % cb points should be defined in cb model space
    wbCornersCamView(:,:,i) = Calibration.rotateTranslatePoints(wbCornersModel, camTvecs(i,:), camRvecs(i,:), true);
    
    % Also generate rotated and translated checkeboard points
    cbPtsCamView(:,:,i) = Calibration.rotateTranslatePoints(cbPtsModel3dNoisy, camTvecs(i,:), camRvecs(i,:), true);
    
    % Project camera view space checkerboard points to camera image space.
    % TODO: skew is ignored by opencv projectPoints function, so we need to
    % write our own project points function
    cbPtsCamImg(:,:,i) = cv.projectPoints(cbPtsCamView(:,:,i), [0 0 0], [0 0 0], gt.camK, 'DistCoeffs', gt.camKc);
end

% check if all checkerboards are within camera image range
for i = 1:numSets
    if(nnz(cbPtsCamImg(:,1,i) > gt.calibInfo.camW) ||  nnz(cbPtsCamImg(:,2,i) > gt.calibInfo.camH))
        error(['Set ', i, ' checkerboard is out of camera image']);
    end
end

% add noise to camera checkerboard points
if(noiseType == 1 || noiseType == 4)
    cbPtsCamImg = cbPtsCamImg + noiseSigma*randn(size(cbPtsCamImg));
end

% Generate planes from white boards' corners
planes = Calibration.planeStructFromPoints(wbCornersCamView);

%% Generate synthetic projector nodes for proposed method
[~, ~, horiPos, vertPos] = ImgProc.createDeBruijnSeq(gt.calibInfo.prjW, gt.calibInfo.prjH);

% Number of node points
numNodes = length(horiPos) * length(vertPos);

% SL color grid node coordinates in 2d image
slPtsPrjImgRawPR = zeros(numNodes, 2);

% create raw (w/o noise) grid nodes' projector image coordinates
k = 0;
for i = 1:length(horiPos)
    for j = 1:length(vertPos)
        k = k+1;
        slPtsPrjImgRawPR(k,:) = [vertPos(j), horiPos(i)];
    end
end

% MT projected image (cam)
[w1, w2] = meshgrid(1:gt.calibInfo.prjW, 1:gt.calibInfo.prjH);
slPtsPrjImgRawMT  = [w1(:) w2(:)];

%% Add noise
slPtsPrjImgNoisyPR = slPtsPrjImgRawPR;
slPtsPrjImgNoisyMT = slPtsPrjImgRawMT;

% add noise to projector image space
if(noiseType == 2 || noiseType == 4)  
    slPtsPrjImgNoisyPR = slPtsPrjImgNoisyPR + noiseSigma*randn(size(slPtsPrjImgNoisyPR));
    slPtsPrjImgNoisyMT = slPtsPrjImgNoisyMT + noiseSigma*randn(size(slPtsPrjImgNoisyMT));
end

%% Backproject the 2D points as rays
rayDirsPR = Calibration.backProjectPts2d(slPtsPrjImgNoisyPR, gt.prjOrg, gt.prjK, gt.prjKc, gt.R, gt.T, verbose);
rayDirsMT = Calibration.backProjectPts2d(slPtsPrjImgNoisyMT, gt.prjOrg, gt.prjK, gt.prjKc, gt.R, gt.T, false);

if(verbose)
    % plot planes on the given figure name
    Calibration.plotPlanes(planes, 'Projected rays');
end

%% intersect rays with planes
if(noiseType == 3 || noiseType == 4)
    % add noise to sl points on calibration board
    [slPtsWorldPR, inlierIdxPR, intersectedWbIdxPR] = Calibration.findRayPlaneIntersections(planes, rayDirsPR, gt.prjOrg, noiseSigma, verbose);
    [slPtsWorldMT, inlierIdxMT, intersectedWbIdxMT] = Calibration.findRayPlaneIntersections(planes, rayDirsMT, gt.prjOrg,noiseSigma,false);
else
    [slPtsWorldPR, inlierIdxPR, intersectedWbIdxPR] = Calibration.findRayPlaneIntersections(planes, rayDirsPR, gt.prjOrg, 0, verbose);
    [slPtsWorldMT, inlierIdxMT, intersectedWbIdxMT] = Calibration.findRayPlaneIntersections(planes, rayDirsMT, gt.prjOrg, 0,false);
end

% keep the checkerboard corners on the planes that are intersected by rays
cbPtsCamImg = cbPtsCamImg(:,:,logical(intersectedWbIdxPR));

% only keep those points that intersects planes
for i =1:numSets
    slPtsPrjImgPR{i} = slPtsPrjImgNoisyPR(inlierIdxPR{i},:);
    slPtsPrjImgMT{i} = slPtsPrjImgNoisyMT(inlierIdxMT{i},:);
end

%% Visualize ground truth and 3d grid points
if(verbose)
    cameraParams = cameraParameters('IntrinsicMatrix', gt.camK', ...
        'RadialDistortion', gt.camKc(1:2), ...
        'TangentialDistortion', gt.camKc(3:4), ...
        'RotationVectors', camRvecs(intersectedWbIdxPR,:), ...
        'TranslationVectors', camTvecs(intersectedWbIdxPR,:), ...
        'WorldPoints', cbPtsModel, ...
        'WorldUnits', 'mm');
       
    projectorParams = cameraParameters('IntrinsicMatrix', gt.prjK', ...
        'RadialDistortion', gt.prjKc(1:2), ...
        'TangentialDistortion', gt.prjKc(3:4), ...
        'RotationVectors', prjRvecs(intersectedWbIdxPR,:), ...
        'TranslationVectors', prjTvecs(intersectedWbIdxPR,:), ...
        'WorldPoints', cbPtsModel, ...
        'WorldUnits', 'mm');
    
    stereoParam = stereoParameters(cameraParams, projectorParams, gt.R', gt.T);
    
    % Show camera calibration (matlab setup) - verbose
    figure;
    showExtrinsics(stereoParam);
    daspect([1 1 1]);
    view(3);
    axis vis3d tight;
    hold on;

    % font size    
    grid on
    xlabel('X(mm)', 'FontWeight','bold');
    ylabel('Z(mm)', 'FontWeight','bold'); %showExtrinsics YZ are swapped
    zlabel('Y(mm)', 'FontWeight','bold');
    set(gca, 'FontSize', 30);

    % Plot 3d grid point in world space
    for i = 1:numSets
        % sl grid points
        curSlPtsWorldPR = slPtsWorldPR{i};
        scatter3(curSlPtsWorldPR(:,1), curSlPtsWorldPR(:,3), curSlPtsWorldPR(:,2), 5, 'go', 'filled',...
            'MarkerEdgeColor', 'k');
        
        % checkerboard points
        curCbPtsCamView = cbPtsCamView(:,:,i);
        hold on;scatter3(curCbPtsCamView(:,1), curCbPtsCamView(:,3), curCbPtsCamView(:,2), 'bo', 'filled',...
            'MarkerEdgeColor', 'k');
    end
end

%% Project 3d ray-plane intersection points to camera image space
numUsedSets = nnz(intersectedWbIdxPR);

% grid points projected to camera image
slPtsCamImgPR = [];
slPtsCamImgMT = [];

% ground truth points used to calculate 3D alignment error
gtPtsCamImg = [];
gtPtsPrjImg = [];
gtPtsWorld = [];

% for each intersected plane, we project the 3d grid points to camera image
% space using ground truth
for i = 1:numUsedSets
    
    curSlPtsCamImgPR = cv.projectPoints(slPtsWorldPR{i}, [0 0 0], [0 0 0], gt.camK, 'DistCoeffs', gt.camKc);
    
    % check if the points are within camera image range
    camInlierIdxPR = curSlPtsCamImgPR(:,1) < gt.calibInfo.camW & curSlPtsCamImgPR(:,2) < gt.calibInfo.camH;
    slPtsCamImgPR{i} = curSlPtsCamImgPR(camInlierIdxPR,:);
    
    % also only keep the inlier grid points in projector image
    curSlPtsPrjImgPR = slPtsPrjImgPR{i};
    slPtsPrjImgPR{i} = curSlPtsPrjImgPR(camInlierIdxPR, :);
    
    % also only keep the inlier grid points in world space
    curSlPtsWorldPR = slPtsWorldPR{i};
    slPtsWorldPR{i} = curSlPtsWorldPR(camInlierIdxPR, :);
    
    %% Do the same for Moreno & Taubin method's data
    curSlPtsCamImgMT = cv.projectPoints(slPtsWorldMT{i}, [0 0 0], [0 0 0], gt.camK, 'DistCoeffs', gt.camKc);
    
    % check if the points are within camera image range
    camInlierIdxMT = curSlPtsCamImgMT(:,1) < gt.calibInfo.camW & curSlPtsCamImgMT(:,2) < gt.calibInfo.camH;
    slPtsCamImgMT{i} = curSlPtsCamImgMT(camInlierIdxMT,:);
    
    % also only keep the inlier grid points in projector image
    curSlPtsPrjImgMT = slPtsPrjImgMT{i};
    slPtsPrjImgMT{i} = curSlPtsPrjImgMT(camInlierIdxMT, :);
    
    % also only keep the inlier grid points in world space
    curSlPtsWorldMT = slPtsWorldMT{i};
    slPtsWorldMT{i} = curSlPtsWorldMT(camInlierIdxMT, :);
    
    %% add noise to camera image space
    if(noiseType == 1 || noiseType == 4)
        slPtsCamImgPR{i} = slPtsCamImgPR{i} + noiseSigma*randn(size(slPtsCamImgPR{i}));
        slPtsCamImgMT{i} = slPtsCamImgMT{i} + noiseSigma*randn(size(slPtsCamImgMT{i}));
    end

end

%% ground truth points for calibration error computation

% add perturbation to make them form a nonplanar pattern to avoid "overfitting"
cbPtsCamViewPerturb = cbPtsCamView + lambda*randn(size(cbPtsCamView)); 

for i = 1:numUsedSets
    gtPtsWorld{i} = cbPtsCamViewPerturb(:,:,i);
    gtPtsCamImg{i} = cv.projectPoints(gtPtsWorld{i}, [0 0 0], [0 0 0], gt.camK, 'DistCoeffs', gt.camKc);
    gtPtsPrjImg{i} = cv.projectPoints(gtPtsWorld{i}, cv.Rodrigues(gt.R), gt.T', gt.prjK, 'DistCoeffs', gt.prjKc);
end

%% Store synthetic ground truth data in gt

% 3d planes
synthData.planes = planes;

% convert checkerboard poitns (model space) to mex OpenCV format
cbPtsModelCv = repmat(cbPtsModel3d, 1, 1,numel(gt.calibInfo.sets));
cbPtsModelCv = num2cell(cbPtsModelCv, [1,2]);

% checkerboard points in checkerboard model space
synthData.cbPtsModel = squeeze(cbPtsModelCv)';

% here we assume all the checkerboard points are within camera image range,
% so no inlier detection, TODO: check inliers
cbPtsCamImgCv = num2cell(cbPtsCamImg, [1,2]);
cbPtsCamImgCv = squeeze(cbPtsCamImgCv)';
synthData.cbPtsCamImg = cbPtsCamImgCv;    % checkerboard points in camera image space

% Proposed method data
PR.slPtsWorld = slPtsWorldPR;      % SL nodes in world space
PR.slPtsCamImg = slPtsCamImgPR;    % SL nodesin camera image space
PR.slPtsPrjImg = slPtsPrjImgPR;    % SL nodes in proj image space

% Moreno & Taubin method data
MT.slPtsWorld = slPtsWorldMT;      % SL nodes in world space
MT.slPtsCamImg = slPtsCamImgMT;    % SL nodes in camera image space
MT.slPtsPrjImg = slPtsPrjImgMT;    % SL nodes in proj image space

% store both proposed and Moreno & Taubin synthetic data
synthData.PR = PR;
synthData.MT = MT;

% store control points
synthData.gtPtsCamImg = gtPtsCamImg;
synthData.gtPtsPrjImg = gtPtsPrjImg;
synthData.gtPtsWorld = gtPtsWorld;

% store synthetic data in ground truth for output
gt.synthData = synthData;
