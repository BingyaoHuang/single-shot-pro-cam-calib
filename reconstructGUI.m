function reconstructGUI(app)
%% Reconstruction function for the app.
% Reconstruct 3d points from the structured light patterns and images.

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

%% Options
dataRoot = app.dataRoot;
dataName = app.dataName;

calibInfo = Calibration.loadCalibInfo(fullfile(dataRoot, dataName));
dataPath = fullfile(dataRoot, dataName);
imgIdx = app.reconOption.sets;

stereoParams = app.stereoParams;

% debug option, enable for visuals/figures
verbose = app.reconOption.verbose;

% close previous waitbar
delete(findall(0,'type','figure','tag','TMWWaitbar'));

% start waitbar
msg = 'Extracting structured light nodes from camera image...';
waitBarHandle = waitbar(0.3, msg, 'Name', 'Reconstructing...');
set(findall(waitBarHandle, '-not', {'Type', 'AnnotationPane'}), 'Units', 'normalized');
waitBarHandle.Position(3) = 0.3;
disp(msg)

%% Step 1: Extract SL points from selected image

imLightName = fullfile(dataPath, sprintf('lightGrid%02d.png', imgIdx));
imColorName = fullfile(dataPath, sprintf('colorGrid%02d.png', imgIdx));
imPatternName = fullfile(dataPath, 'pattern.png');

% generate color grid pattern if it does not exist
if(~exist(imPatternName, 'file'))
    imwrite(ImgProc.genStructuredLight(calibInfo.prjW, calibInfo.prjH), imPatternName);
end

%% Find matched nodes
camW = calibInfo.camW;
camH = calibInfo.camH;
prjW = calibInfo.prjW;
prjH = calibInfo.prjH;

imgName = app.ImageListBox.Value;
imMaskName = fullfile(dataPath, ['objectROI', imgName{1}(end-1:end), '.png']);

if(app.reconOption.useExistingMask)
    if(exist(imMaskName, 'file'))
        imMask = imread(imMaskName);
    else
        uialert(app.ProCamCalibUIFigure, [imMaskName, ' does not exist. Check off Use Existing Mask!'], 'File not found');
        close(waitBarHandle);
        return;
    end
else
    % manually select roi to be reconstructed
    msg = 'Waiting for user to draw the target object ROI';
    waitbar(0.4, waitBarHandle, msg);
    disp(msg);
    app.ManualSegButton.ButtonPushedFcn(app, [])
    imMask = imread(imMaskName);
end

% extract matched color grid Nodes' coords in camera and projector images
msg = 'Matching structured light pattern...';
waitbar(0.45, waitBarHandle, msg);
disp(msg);

[camNodes, prjNodes, Nodes, Edges] = ImgProc.getMatchedNodes(imLightName, imColorName, [], prjW, prjH, verbose);

%% triangulate nodes
msg = 'Triangulating node coordinates...';
waitbar(0.5, waitBarHandle, msg);
disp(msg);

% intrinsics
camK = stereoParams.camK;
camKc = stereoParams.camKc;

prjK = stereoParams.prjK;
prjKc = stereoParams.prjKc;

% extrinsics
R = stereoParams.R;
T = stereoParams.T;
F = stereoParams.F;

% VERY IMPORTANT!!!!, undistort camera image points first
camNodesUndistort = ImgProc.cvUndistortPoints(camNodes, camK, camKc);
prjNodesUndistort = ImgProc.cvUndistortPoints(prjNodes, prjK, prjKc);

% remove epipolar outliers
[inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camNodesUndistort, prjNodesUndistort, app.reconOption.epiThresh, verbose);
camNodesUndistort = camNodesUndistort(inlierIdx,:);
prjNodesUndistort = prjNodesUndistort(inlierIdx,:);

% % draw camera and projector points epipolar lines
if(verbose)
    imCamUndistort = cv.undistort(imread(imColorName), camK, camKc);
    imPrjUndistort = cv.undistort(imread(imPatternName), prjK, prjKc);
    
    % draw epipolar lines
    ImgProc.drawEpipolarLine(F, camNodesUndistort, imCamUndistort, imPrjUndistort);
    ImgProc.drawEpipolarLine(F', prjNodesUndistort, imPrjUndistort, imCamUndistort);
end

% triangulate Node's 3d coordinates
nodePts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camNodesUndistort, prjNodesUndistort);

% visualize reconstructed 3d Nodes
Reconstruct.visualizePts3d(nodePts3d, R, T, 'Node points only');

%% triangulate Edges
msg = 'Triangulating edge coordinates...';
waitbar(0.7, waitBarHandle, msg);
disp(msg);

% reconstruct edge points under projector distortion assumption and it works well
if(app.reconOption.useEdge)
    camEdges = [];
    prjEdges = [];
    
    for i=1:numel(Edges)
        curEdge = Edges(i);
        if(numel(curEdge.nodes) ~= 2)
            continue;
        end
        
        node1 = Nodes(curEdge.nodes(1));
        node2 = Nodes(curEdge.nodes(2));
        
        p1 = [node1.activeCol, node1.activeRow];
        p2 = [node2.activeCol, node2.activeRow];
        
        if(any( [p1, p2] < 0))
            continue;
        end
        
        % two nodes's cam coords
        c1 = ImgProc.cvUndistortPoints(node1.Centroid, camK, camKc);
        c2 = ImgProc.cvUndistortPoints(node2.Centroid, camK, camKc);
        
        % two nodes's prj coords, (should be undistorted)
        p1 = ImgProc.cvUndistortPoints(p1, prjK, prjKc);
        p2 = ImgProc.cvUndistortPoints(p2, prjK, prjKc);
        
        % edge points' row and col
        [rows, cols] = ind2sub([camH, camW], curEdge.PixelIdxList);
        
        % linear interpolate edge pixels' projector coord
        for k = 1:length(rows)
            curCamEdgePt2d = [cols(k), rows(k)];
            %             d1 = abs(curCamEdgePt2d - c1);
            %             d2 = abs(curCamEdgePt2d - c2);
            %             s = d1./(d1+d2);
            %             curPrjEdgePt2d = (1-s).*p1 + s.*p2;
            d1 = norm(curCamEdgePt2d - c1);
            d2 = norm(curCamEdgePt2d - c2);
            s = d1/(d1+d2);
            curPrjEdgePt2d = (1-s)*p1 + s*p2;
            curPrjEdgePt2d = curPrjEdgePt2d(1:2); % convert back to non-homogenious
            prjEdges = [prjEdges; curPrjEdgePt2d];
            camEdges = [camEdges; curCamEdgePt2d];
        end
    end
    
    camEdgesUndistort = camEdges;
    prjEdgesUndistort = prjEdges;
    
    % remove epipolar outliers
    [inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camEdgesUndistort, prjEdgesUndistort, app.reconOption.epiThresh, verbose);
    camEdgesUndistort = camEdgesUndistort(inlierIdx,:);
    prjEdgesUndistort = prjEdgesUndistort(inlierIdx,:);
    
    % triangulate
    edgePts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camEdgesUndistort, prjEdgesUndistort);
    
    % visualize
    Reconstruct.visualizePts3d([nodePts3d; edgePts3d], R, T, 'Both Node and Edge points');
end

%% interpolate point cloud
msg = 'Interpolating point cloud...';
waitbar(0.9, waitBarHandle, msg);
disp(msg);

imROI = ImgProc.maskImage(imread(imLightName), imMask);

ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud(nodePts3d), imROI, imMask, verbose);
% ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud([nodePts3d; edgePts3d]), imROI, imMask, verbose);

figure;
h = pcshow(ptCloudInterp, 'MarkerSize', 150);
title('Interpolated point cloud')

% save interpolated point cloud
pcFileName = fullfile(dataPath, ['pointCloud', imgName{1}(end-1:end), '.ply']);
pcwrite(ptCloudInterp, pcFileName);

%% Finished
waitbar(1.0, waitBarHandle, 'Reconstruction done!');
close(waitBarHandle);
uiconfirm(app.ProCamCalibUIFigure,['Point Cloud saved to ', pcFileName], 'Reconstruction complete!', 'Options', {'OK'},'icon','success');
end
