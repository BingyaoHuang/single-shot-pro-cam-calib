%% Compare Moreno & Taubin, global homography, proposed w/o BA and proposed.
% Compares four calibration methods mentioned in the paper.

%% Suffix:
% PR: proposed method
% DG: degraded proposed method (w/o BA)
% GH: generalized global homography method
% MT: Moreno & Taubin method
% e.g., stereoParamsPR: stereo parameters obtained using proposed method.

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
% clc; 
clear; close all

%% Options
dataRoot = 'data';
dataName = '6-3-19';

calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName));
calibInfo.dataName = dataName;

% folder where extracted checkerboard corners are stored
cornerDir = fullfile(calibInfo.path, 'matlabCorners');

% debug option, enable for visuals/figures
verbose = 0;

% checkerboard corners will be extracted and saved by the script.
% If no entry in calib-info.yml file has been modified since the last
% calibration, you can set the two flags to true to speed up recalibration.
% But you must set them to false if you change sets, camera, projector or
% checkerobard settings in calib-info.yml.
useExistingCamCorners = 0;
useExistingPrjCorners = 0;

format short


%% Step 1: Get checkerboard corners from camera image
if (~useExistingCamCorners)
    disp('Extracting checkerboard corners from camera image...');
    
    % 1. Get the checkerboard points for the selected calibInfo.sets from camera image
    [camCorners, usedImIdx] = Calibration.getCameraCorners(calibInfo);
    
    % 2. Save the camera corners, and load them to eliminate rounding errors.
    Calibration.saveCorners(camCorners, 'cam_', cornerDir, calibInfo.sets, usedImIdx);
    camCorners = camCorners(:, :, usedImIdx);
    
    % 3. Eliminate any potentially unused calibInfo.sets during checkerboard detection
    calibInfo.sets = calibInfo.sets(usedImIdx);
    calibInfo.numSets = numel(calibInfo.sets);
else
    % Read camCorners
    disp('Reading existing camera corners...');
    camCorners = Calibration.readCorners('cam_', cornerDir, calibInfo);    
end

% find opencv checkerboard corners and matlab checkerboard corners matches (for MT method)
% imLightNames = ImgProc.getImageNames(calibInfo.path, 'light');
% imLightNames = imLightNames(calibInfo.sets);
% [cornersCV, ret] = cv.findChessboardCorners(rgb2gray(imread(imLightNames{1})), calibInfo.boardSize-1);
% cornersCV = cell2mat(cornersCV');
% [idx, err] = knnsearch(cornersCV, camCorners(:,:,1));


%% Step 2: Calibrate camera, (skip 1 if using existing camera corners)
disp(' ');
disp('Calibrating camera using checkerboard corners...');

% 1. Generate world corners
modelCornersCell = Calibration.generateModelCorners(calibInfo);

% 2. Convert camCorners to (mex)OpenCV format
camCornersCell = squeeze(mat2cell(camCorners, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';

% 3. Calibrate camera, only used to warp nodes to model space (not init guess)
camParams = Calibration.calibrateInitGuess(modelCornersCell, camCornersCell, calibInfo);

%% Step 3: Calculate projector points for global homography and nodes for proposed method
disp(' ');
disp('Extracting and saving grid nodes and warped projector corners...');

if (~useExistingPrjCorners)
    % 1. Get projector corners using matching & homography
    [nodesCell, prjCornersGH] = Calibration.getNodesAndPrjCorners(calibInfo, camParams, camCorners, verbose);
    
    % 2. Save projector points and node pairs
    Calibration.saveCorners(prjCornersGH, 'proj_', cornerDir, calibInfo.sets, ones(calibInfo.numSets,1));
    cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml'), nodesCell);
else
    disp('Reading existing warped projector corners and nodes...');
    prjCornersGH = Calibration.readCorners('proj_', cornerDir, calibInfo);
    nodesCell = cv.FileStorage(fullfile(calibInfo.path, 'nodePairs.yml')).nodePairs;
end

%% Step 4: Read Moreno & Taubin warped checkerboard corners and SL node coordinates
disp(' ');
disp('Reading existing Moreno & Taubin warped projector corners and nodes...');

% 1. read Moreno & Taubin checkerboard corners warped by local
% homographies. (MT stands for their method related data)
cornerDirMT = fullfile(calibInfo.path, 'MT');
prjCornersMT = Calibration.readCorners('proj_', cornerDirMT, calibInfo);

% change order to match matlab checkerboard corner orders
% prjCornersMT = prjCornersMT(idx,:,:);

if (verbose)
    for i = 1:calibInfo.numSets
        figure;
        hold on;
        plot(prjCornersGH(:, 1, i), prjCornersGH(:, 2, i), 'ro'); hold on
        plot(prjCornersMT(:, 1, i), prjCornersMT(:, 2, i), 'b+');
        title(['Set ', num2str(calibInfo.sets(i)),...
            ': warped checkerboard corners in projector image, red is global homography, blue is Moreno & Taubin' ]);
        hold off;
    end
end

%% debug
% camCornersMT = Calibration.readCorners('cam_', cornerDirMT, calibInfo);
% i=1;
% figure;
% for j = 1:size(camCorners,1)
%     plot(camCorners(j, 1, i), camCorners(j, 2, i), 'ro'); text(camCorners(j, 1, i), camCorners(j, 2, i),num2str(j)); hold on
%     plot(camCornersMT(j, 1, i), camCornersMT(j, 2, i), 'b+');text(camCornersMT(j, 1, i), camCornersMT(j, 2, i),num2str(j));
% end
% 
% figure;
% tst =  camCorners(idx,:,:);
% for j = 1:size(camCorners,1)
%     plot(tst(j, 1, i), tst(j, 2, i), 'ro'); text(tst(j, 1, i), tst(j, 2, i),num2str(j)); hold on
%     plot(camCornersMT(j, 1, i), camCornersMT(j, 2, i), 'b+');text(camCornersMT(j, 1, i), camCornersMT(j, 2, i),num2str(j));
% end


%% Step 5. Warp node points (Xc) in camera image to model space (Xm)
disp(' ');
disp('Warpping nodes to model space (Xc to Xm) for proposed method...');
% refer to paper to understand Xm, Xc, Xp
Xm = []; % node points in model space
Xc = []; % node points in camera image space
Xp = []; % node points in projector image space

% 1. Undistorted node points in camera image space
xcUndistort = cell(1, calibInfo.numSets);

% 2. Warp Xc to model space to get Xm
for i = 1:calibInfo.numSets
    % according to Zhang's method the homography Hmc is:
    % H = lambda*K*[r1, r2, t], where r1, r2 are 1st and 2nd column of R
    R = cv.Rodrigues(camParams.rVecs{i});
    Hmc = camParams.camK * [R(:, 1:2), camParams.tVecs{i}];
    lambda = 1 / norm(inv(camParams.camK) * Hmc(:, 1));
    Hmc = lambda * Hmc;
    
    % undistort grid points in camera image space
    Xc{i} = nodesCell{i}(:, 1:2);
    Xp{i} = nodesCell{i}(:, 3:4);
    
    % undistort grid points in camera image
    xcUndistort{i} = ImgProc.cvUndistortPoints(Xc{i}, camParams.camK, camParams.camKc);
    
    % transform camera image grid points to white board model space using
    % H, then use calibrated tvecs and rvecs to transform grid points from
    % model space to world space.
    % NOTE: the cb corners have to be undistorted
    curXm = ImgProc.applyHomography(xcUndistort{i}, inv(Hmc));
    curXm = [curXm, zeros(length(curXm), 1)];
    Xm{i} = curXm;
    
    % random sample points to reduce computation and ram
%     k = 500;
%     if(length(Xm{i}) > k)
%         idx = randsample(length(Xm{i}), k);
%         Xm{i} = Xm{i}(idx,:);
%         Xc{i} = Xc{i}(idx,:);
%         Xp{i} = Xp{i}(idx,:);
%     end
    
end


%% Step 6. Calibrate camera and projector using the proposed method (PR)
disp(' ');
disp('[Proposed] Performing camera-projector calibration using BA...');

% 1. Use 'BA' option to specify bundle adjustment on Xm
stereoParamsPR = Calibration.stereoCalibrate(Xm, Xc, Xp, calibInfo.camImgSize, calibInfo.prjImgSize, 'BA');
stereoParamsPR.sets = calibInfo.sets;
stereoParamsPR.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParamsPR = Calibration.calcReprojectionError(stereoParamsPR, stereoParamsPR.modelPts, stereoParamsPR.camImgPts, stereoParamsPR.prjImgPts);

% 3. Display calibration results
stereoParamsPR

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsPR.worldPts'), stereoParamsPR.R, stereoParamsPR.T, 'Proposed calibration: Xm ');
end

%% Calibrate degraded proposed (DG for short)
disp(' ');
disp('[Proposed w/o BA] Performing camera-projector calibration using degreded propsoed method (w/o BA)...');

% 1. Use 'withoutBA' option to specify no bundle adjustment on Xm
stereoParamsDG = Calibration.stereoCalibrate(Xm, Xc, Xp, calibInfo.camImgSize, calibInfo.prjImgSize, 'withoutBA');
stereoParamsDG.sets = calibInfo.sets;
stereoParamsDG.dataName = calibInfo.dataName;

% 2. Calculate reprojection errors
stereoParamsDG = Calibration.calcReprojectionError(stereoParamsDG, stereoParamsDG.modelPts, stereoParamsDG.camImgPts, stereoParamsDG.prjImgPts);

% 3. display calibration results
stereoParamsDG
% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsDG.worldPts'), stereoParamsDG.R, stereoParamsDG.T, 'Propsoed w/o BA calibration: Xm ');
end

%% Calibrate Moreno & Taubin (MT for short)
disp(' ');
disp('[Moreno & Taubin] Performing camera-projector calibration using Moreno & Taubin method...');

% 1. Calibrate projector using local homography warped checkerboard corners
prjCornersCellMT = squeeze(mat2cell(prjCornersMT, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';
prjParamsMT = Calibration.calibrateInitGuess(modelCornersCell, prjCornersCellMT, calibInfo);

% 2. Stereo calibration using OpenCV
stereoParamsMT = Calibration.calibrateStereoInitGuess(modelCornersCell, camCornersCell, prjCornersCellMT, camParams, prjParamsMT, calibInfo);
stereoParamsMT.sets = calibInfo.sets;
stereoParamsMT.dataName = calibInfo.dataName;
    
% 3. Calculate reprojection errors
stereoParamsMT = Calibration.calcReprojectionError(stereoParamsMT, modelCornersCell, camCornersCell, prjCornersCellMT);

% 4. display calibration results
stereoParamsMT

% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsMT.worldPts'), stereoParamsMT.R, stereoParamsMT.T, 'Moreno & Taubin calibration: Xm ');
end

%% Calibrate global homography (GH for short)
disp(' ');
disp('[Global homography] Performing camera-projector calibration using generalized global homography method...');

% 1. Calibrate projector using local homography warped checkerboard corners
prjCornersCellGH = squeeze(mat2cell(prjCornersGH, calibInfo.numCorners, 2, ones(1, calibInfo.numSets)))';
prjParamsGH = Calibration.calibrateInitGuess(modelCornersCell, prjCornersCellGH, calibInfo);

% 2. Stereo calibration using OpenCV
stereoParamsGH = Calibration.calibrateStereoInitGuess(modelCornersCell, camCornersCell, prjCornersCellGH, camParams, prjParamsGH, calibInfo);
stereoParamsGH.sets = calibInfo.sets;
stereoParamsGH.dataName = calibInfo.dataName;

% 3. Calculate reprojection errors
stereoParamsGH = Calibration.calcReprojectionError(stereoParamsGH, modelCornersCell, camCornersCell, prjCornersCellGH);

% 4. display calibration results
stereoParamsGH
% debug
if (verbose)
    Reconstruct.visualizePts3d(cell2mat(stereoParamsGH.worldPts'), stereoParamsGH.R, stereoParamsGH.T, 'Global homography calibration: Xm ');
end

%% Step 7. Save all calibration data
calTypes = {'moreno_taubin', 'global_homography', 'degraded_proposed', 'proposed'};

cv.FileStorage(fullfile(calibInfo.resultDir, [calTypes{1}, '.yml']), stereoParamsMT);
cv.FileStorage(fullfile(calibInfo.resultDir, [calTypes{2}, '.yml']), stereoParamsGH);
cv.FileStorage(fullfile(calibInfo.resultDir, [calTypes{3}, '.yml']), stereoParamsDG);
cv.FileStorage(fullfile(calibInfo.resultDir, [calTypes{4}, '.yml']), stereoParamsPR);

disp(' ');
disp(['Calibration data saved to ', calibInfo.resultDir]);

%% Step 8. Compare calibration reprojection errors
algNames = {'Moreno & Taubin', 'Global homography',  'Proposed w/o BA', 'Proposed'};
reprojErrs = zeros(length(calTypes), 3);
for i=1:length(calTypes)
    curCalType = calTypes{i};
    param = cv.FileStorage(fullfile(calibInfo.resultDir, [curCalType, '.yml']));
    
    % calibration reprojection error
    reprojErrs(i,1) = param.camReprojErr;
    reprojErrs(i,2) = param.prjReprojErr;
    reprojErrs(i,3) = param.stereoReprojErr;
end

% reprojection error
calibTable = array2table(reprojErrs,...
    'VariableNames',{'camera','projector','stereo'},...
    'RowNames', algNames);

disp(' ');
disp('Calibration reprojection errors:'); 
disp(calibTable);

%% reprojection error scattered plot
% colorList = lines(5);
colorList = [
    0    0.4470    0.7410;
%     0.8020    0.2804    0.1039;
    0.9290    0.6940    0.1250
    0.4660    0.6740    0.1880;
    0.6350    0.0780    0.1840;
    ];

sz = 60;
figure;
% cam
sp1 = subplot(1,2,1); 
a = scatter(stereoParamsMT.camRes(:,1), stereoParamsMT.camRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(1,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
b = scatter(stereoParamsGH.camRes(:,1), stereoParamsGH.camRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(2,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
c = scatter(stereoParamsDG.camRes(:,1), stereoParamsDG.camRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(3,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
d = scatter(stereoParamsPR.camRes(:,1), stereoParamsPR.camRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(4,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
axis(sp1, [-2, 2,-1.5, 1.5]); box on;
uistack(a, 'top');
legend([a,b,c,d], {'Moreno & Taubin', 'Global homography','Proposed w/o BA', 'Proposed'}, 'Location', 'northwest'); 
xlabel('\fontsize{13}{0}\bf\selectfont x (pixel)', 'interpreter', 'latex');
ylabel('\fontsize{13}{0}\bf\selectfont y (pixel)', 'interpreter', 'latex');
set(gca, 'FontSize', 16); 
title('\fontsize{15}{0}\bf\selectfont Camera reprojection error', 'interpreter', 'latex');

% prj
sp2 = subplot(1,2,2); 
e = scatter(stereoParamsMT.prjRes(:,1), stereoParamsMT.prjRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(1,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
f = scatter(stereoParamsGH.prjRes(:,1), stereoParamsGH.prjRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(2,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
g = scatter(stereoParamsDG.prjRes(:,1), stereoParamsDG.prjRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(3,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
h = scatter(stereoParamsPR.prjRes(:,1), stereoParamsPR.prjRes(:,2), sz, 'LineWidth', .1, 'markerfacecolor', colorList(4,:), 'MarkerEdgeColor', 'black'); daspect([1,1,1]); hold on;
% axis(sp2, [-15, 15, -11.25, 11.25]); box on;
axis(sp2, [-14, 14, -10.5, 10.5]); box on;
uistack(e, 'top');
uistack(h, 'top');
% legend([e,f,g,h], {'Moreno & Taubin [29]', 'Global homography','Proposed w/o BA', 'Proposed'}, 'Location', 'northwest'); 
xlabel('\fontsize{13}{0}\bf\selectfont x (pixel)', 'interpreter', 'latex');
ylabel('\fontsize{13}{0}\bf\selectfont y (pixel)', 'interpreter', 'latex');
set(gca, 'FontSize', 16); 
title('\fontsize{15}{0}\bf\selectfont Projector reprojection error', 'interpreter', 'latex');
hold off;

%% Step 9. Compare 3D reconstruction
% for calibration-6-3-19, journal paper results are reported using
% reonSet 11 (frog), 18 (girl), 21 (jar)
reconSet = '11'; 
linkedFigs = Calibration.compareRealData(reconSet, calTypes, algNames, calibInfo);