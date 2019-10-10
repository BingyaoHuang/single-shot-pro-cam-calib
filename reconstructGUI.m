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
verbose = app.calibOption.verbose;

% start waitbar
msg = 'Extracting structured light nodes from camera image...';
waitBarHandle = waitbar(0.3, msg, 'Name', 'Reconstructing...');
set(findall(waitBarHandle),'Units', 'normalized');
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

if(~app.calibOption.useExistingMask)
    % manually select roi to be reconstructed
    msg = 'Waiting user to draw the region to be reconstructed...';
    waitbar(0.4, waitBarHandle, msg);
    disp(msg);
    app.ManualSegButton.ButtonPushedFcn(app, [])
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
Reconstruct.visualizePts3d(nodePts3d, R, T, 'Node points');

%% triangulate Edges
msg = 'Triangulating edge coordinates...';
waitbar(0.7, waitBarHandle, msg);
disp(msg);

%% reconstruct edge points using ray-plane intersection (assume no projector distortion, and it does not work well)
if(0)
    validNodes = Nodes([Nodes.activeCol] > 0 & [Nodes.activeRow] > 0);
    validNodes = validNodes(inlierIdx); % mis-decoded nodes have wrong edge lables and they should not pass epipolar test...
    
    prjOrg = (-R'*T)';
    edgePts3d = [];
    
    usedEdgeIdx = [];
    for i=1:length(validNodes)
        curNode = validNodes(i);
        
        for j = 1:2
            if(j == 1)
                % horizontal edge in projector image space
                newEdgeIdx = ~ismember(curNode.hEdges, usedEdgeIdx);
                if(all(~newEdgeIdx))
                    continue
                end
                
                usedEdgeIdx = [usedEdgeIdx, curNode.hEdges(newEdgeIdx)];
                
                edgePixelIdx = vertcat(Edges(curNode.hEdges(newEdgeIdx)).PixelIdxList);
                c1 = [curNode.activeCol+1, curNode.activeRow];
                c2 = [curNode.activeCol-1, curNode.activeRow];
            else
                % vertical edge in projector image space
                newEdgeIdx = ~ismember(curNode.vEdges, usedEdgeIdx);
                if(all(~newEdgeIdx))
                    continue
                end
                
                usedEdgeIdx = [usedEdgeIdx, curNode.vEdges(newEdgeIdx)];
                edgePixelIdx = vertcat(Edges(curNode.vEdges(newEdgeIdx)).PixelIdxList);
                c1 = [curNode.activeCol, curNode.activeRow+1];
                c2 = [curNode.activeCol, curNode.activeRow-1];
            end
            
            % undistort and convert to homogenious coord
            c1 = [ImgProc.cvUndistortPoints(c1, prjK, prjKc), 1];
            c2 = [ImgProc.cvUndistortPoints(c2, prjK, prjKc), 1];
            
            % unproject to 3d
            pts3d = (R' * (prjK \ [c1;c2]' - T))';
            
            % plane equation:Ax+By+Cz+1=0; solving for ABC need 3 points.
            % Ax = b, solve for x, where x = [A,B,C]'
            planeParams = [prjOrg;pts3d]\(-ones(3,1));
            
            [rows, cols] = ind2sub([camH,camW], edgePixelIdx);
            camEdgesUndistort = ImgProc.cvUndistortPoints([cols,rows], camK, camKc);
            %     camEdgesUndistort = [cols,rows];
            
            Z = -1./(planeParams(1)*(camEdgesUndistort(:,1) -camK(1,3)) /camK(1,1)+planeParams(2)*(camEdgesUndistort(:,2)-camK(2,3))/camK(2,2)+planeParams(3));
            X = Z.*(camEdgesUndistort(:,1) -camK(1,3))/camK(1,1);
            Y = Z.*(camEdgesUndistort(:,2) -camK(2,3))/camK(2,2);
            
            edgePts3d = [edgePts3d;[X,Y,Z]];
        end
    end
    
    % visualize
    Reconstruct.visualizePts3d(edgePts3d, R, T, 'Edge points');
    Reconstruct.visualizePts3d([nodePts3d; edgePts3d], R, T, 'Node and Edge points');
end

%% reconstruct edge points under projector distortion assumption and it works well
camEdges = [];
prjEdges = [];
validNodesIdx = find([Nodes.activeCol] > 0 & [Nodes.activeRow] > 0);
validNodesIdx = validNodesIdx(inlierIdx);

for i=1:length(validNodesIdx)
    curNode = Nodes(validNodesIdx(i));
    
    for j = 1:length(curNode.edges)
        curEdge = Edges(curNode.edges(j));
        otherNodeIdx = setdiff(curEdge.nodes, validNodesIdx(i));
        if(~ismember(otherNodeIdx, validNodesIdx))
            continue;
        end
        
        otherNode = Nodes(setdiff(curEdge.nodes, validNodesIdx(i)));
        
        % linear interpolate edge pixels' projector coord
        [rows, cols] = ind2sub([camH,camW], curEdge.PixelIdxList);
        
        % undistort cam edge points
        edgePts2d = ImgProc.cvUndistortPoints([cols,rows], camK, camKc);
        rows = edgePts2d(:, 2);
        cols = edgePts2d(:, 1);
              
        if (isempty(otherNode))
           continue;
        end
        
        % two cam nodes coords
        c1 = ImgProc.cvUndistortPoints(otherNode.Centroid, camK, camKc);
        c2 = ImgProc.cvUndistortPoints(curNode.Centroid, camK, camKc);
        
        for k = 1:length(rows)
            d1 = norm([cols(k), rows(k)] - c1);
            d2 = norm([cols(k), rows(k)] - c2);
            s = d1/(d1+d2);
            
            % projector points also should be undistorted!
            p1 = [ImgProc.cvUndistortPoints([otherNode.activeCol, otherNode.activeRow], prjK, prjKc), 1];
            p2 = [ImgProc.cvUndistortPoints([curNode.activeCol, curNode.activeRow], prjK, prjKc), 1];
            curEdgePts2d = (1-s)*p1 + s*p2;
            curEdgePts2d = curEdgePts2d(1:2); % convert back to non-homogenious
            prjEdges = [prjEdges; curEdgePts2d];
            camEdges = [camEdges; [cols(k), rows(k)]];
        end
    end
end

% camEdgesUndistort = ImgProc.cvUndistortPoints(camEdges, camK, camKc);
% prjEdgesUndistort = ImgProc.cvUndistortPoints(prjEdges, prjK, prjKc);
camEdgesUndistort = camEdges;
prjEdgesUndistort = prjEdges;

% remove epipolar outliers
[inlierIdx, d] = Reconstruct.findEpipolarInliers(F, camEdgesUndistort, prjEdgesUndistort, app.reconOption.epiThresh, verbose);
camEdgesUndistort = camEdgesUndistort(inlierIdx,:);
prjEdgesUndistort = prjEdgesUndistort(inlierIdx,:);

% triangulate
edgePts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camEdgesUndistort, prjEdgesUndistort);

% visualize
if(verbose)
    Reconstruct.visualizePts3d([nodePts3d; edgePts3d], R, T, 'Reconstructed Nodes and Edges points');
end

%% interpolate point cloud
msg = 'Interpolating point cloud...';
waitbar(0.9, waitBarHandle, msg);
disp(msg);

imgName = app.ImageListBox.Value;
imMask = imread(fullfile(dataPath, ['objectROI', imgName{1}(end-1:end), '.png']));
imROI = ImgProc.maskImage(imread(imLightName), imMask);

% ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud(nodePts3d), imROI, imMask, verbose);
ptCloudInterp = Reconstruct.interpolatePtCloud(camK, camKc, pointCloud([nodePts3d; edgePts3d]), imROI, imMask, verbose);

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