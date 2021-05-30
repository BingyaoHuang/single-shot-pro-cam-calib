function [nodePairs, prjCorners] = getNodesAndPrjCorners(calibInfo, camParams, camCorners, verbose, useParallel)
%% Extract stuctured light grid node pairs and warped checkerboard corners in projector image (for global homography-based method).

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
if(nargin < 5)
    useParallel = verbose;
end

% parse image names
imLightNames = ImgProc.getImageNames(calibInfo.path, 'lightGrid', calibInfo.sets);
imColorNames = ImgProc.getImageNames(calibInfo.path, 'colorGrid', calibInfo.sets);
imPatternName = fullfile(calibInfo.path, 'pattern.png');

% generate color grid pattern if it does not exist
if(~exist(imPatternName, 'file'))
    imwrite(ImgProc.genStructuredLight(calibInfo.prjW, calibInfo.prjH), imPatternName);
end

prjCorners = zeros(size(camCorners));
nodePairs = cell(1, size(camCorners,3));

% for each set
if(~useParallel) % debug
    for i = 1:size(prjCorners,3)
        disp(['Extracting projector corners for image ', num2str(i)]);
        [prjCorners(:,:,i), nodePairs{i}] = findMatchAndWarp(imLightNames{i}, ...
            imColorNames{i}, imPatternName,camCorners(:,:,i), camParams, calibInfo, verbose);
        disp(['Done image ' num2str(i)]);
    end
else % non-debug
    parfor i = 1:size(prjCorners,3)
        disp(['Extracting projector corners for image ', num2str(i)]);
        [prjCorners(:,:,i), nodePairs{i}] = findMatchAndWarp(imLightNames{i}, ...
            imColorNames{i}, imPatternName,camCorners(:,:,i), camParams, calibInfo, verbose);
        disp(['Done image ' num2str(i)]);
    end
end

%% local function
function [prjCorners, nodePairs] = findMatchAndWarp(imLightName, imColorName, ...
    imPatternName, camCorners, camParams, calibInfo, verbose)

%% Find matched nodes
prjW = calibInfo.prjW;
prjH = calibInfo.prjH;

camK = camParams.camK;
camKc = camParams.camKc;

% extract matched color grid Nodes' coords in camera and projector images
[camNodes, prjNodes] = ImgProc.getMatchedNodes(imLightName, imColorName, camCorners, prjW, prjH, verbose);

%% Remove SL node outliers
% most cameras do not need undistort at this stage (except fisheye). If the
% init cam calibration is not accurate, the undistortion will mess up the
% correctly matched SL nodes, thus most of them are removed as outliers.

% camNodesUndistort = ImgProc.cvUndistortPoints(camNodes, camK, camKc);
camNodesUndistort = camNodes;

% Get rid of outliers using homography
[Hcp, inliers] = cv.findHomography(camNodesUndistort, prjNodes, ...
    'Method', 'Ransac', ...
    'RansacReprojThreshold', 1, ...
    'MaxIters', 1000);

inliers = logical(inliers);

if(nnz(inliers) <= 0)
    disp(['Aborting set ' string(imColorName(end-5:end-4)) ', no inliers!']);
    return
end

% node points are used by degraded proposed (w/o BA) and proposed (with BA)
nodePairs = [camNodes(inliers, :), prjNodes(inliers, :)];

% verbose. visualize inlier matches
if(verbose)
    figure('Name', 'getNodesAndPrjCorners');
    imPattern = imread(imPatternName);
    % most cameras do not need undistort at this stage (except fisheye)
    %     imColorGrid = cv.undistort(imread(imColorName), camK, camKc);
    imColorGrid = imread(imColorName);
    
    subplot(2,1,1);
    showMatchedFeatures(imPattern, imColorGrid, prjNodes, camNodes, 'montage');
    title(['All matches of set ', string(imColorName(end-5:end-4))]);
    drawnow
    
    % SL nodes (inliers)
    camPts = nodePairs(:,1:2);
    prjPts = nodePairs(:,3:4);
    
    subplot(2,1,2);
    showMatchedFeatures(imPattern, imColorGrid, prjPts, camPts, 'montage');
    title(['Inlier matches of set ', string(imColorName(end-5:end-4))]);
    drawnow
    
    % warp camera image and points to projector image space to see if
    % the grid and points overlap
    imColorGridPrj = cv.warpPerspective(imColorGrid, Hcp, 'DSize',[prjW,prjH]);
    camPtsWarp = ImgProc.applyHomography(camPts,Hcp);
    
    figure('Name', 'getNodesAndPrjCorners');
    showMatchedFeatures(imPattern, imColorGridPrj, prjPts, camPtsWarp, 'falsecolor');
    title(['Inlier matches of projector image and warped camera image of' string(imColorName(end-5:end-4))]);
end

%% [Global Homography] Warp camera corners to projector image. prjCorners are
% used by Global Homography method only, the proposed method uses SL points
% instead for calibration.
prjCorners = ImgProc.applyHomography(camCorners, Hcp);
