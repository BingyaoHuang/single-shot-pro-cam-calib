function ptCloudOut = interpolatePtCloud(camK, camKc, ptCloudIn, imROI, imMask, verbose)
%% Interpolate the input sparse point cloud

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

%% interpolate point cloud
x = ptCloudIn.Location(:,1);
y = ptCloudIn.Location(:,2);
z = ptCloudIn.Location(:,3);

% # of interpolated points in X Y meshgrid
numPts = 100;

% xRange contains lower and higher bounds
xRange = ptCloudIn.XLimits;
xDiff = xRange(1) - xRange(2);
xStep = -xDiff / numPts;

yRange = ptCloudIn.YLimits;
yDiff = yRange(1) - yRange(2);
yStep = -yDiff / numPts;

%% use griddata to interpolate a mesh from given point cloud
[xq, yq] = meshgrid(xRange(1):xStep:xRange(2), yRange(1):yStep:yRange(2));
zq = griddata(x, y, z, xq, yq, 'v4');

% interpolated denser point cloud
pts3d = [xq(:), yq(:), zq(:)];

%% only keep the interpolated points in mask ROI
% project interpolated mesh points to camera image
pts2d = cv.projectPoints(pts3d, zeros(3,1), zeros(3,1), camK, 'DistCoeffs', camKc);

% convert to linear indices
pts2d = round(pts2d);
pts2dColor = pts2d;

% filter point cloud
inlierIdx = (pts2d(:,2) > 0) & (pts2d(:,1) > 0) & (pts2d(:,2) < size(imMask,1)) & (pts2d(:,1) < size(imMask,2));
pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);
pts2dIdx = sub2ind(size(imMask), pts2d(:,2), pts2d(:,1));

% keep 2d, 3d point in ROI
pixVal = imMask(pts2dIdx);
pts2dInliers = pts2d(pixVal>0,:);
pts3dInliers = pts3d(pixVal>0,:);

if(1)
    % plot surf
    pts2dColor(pts2dColor <= 0) = 1;
    pts2dColor(pts2dColor(:,1) > size(imMask,2), 1) = size(imMask,2);
    pts2dColor(pts2dColor(:,2) > size(imMask,1), 2) = size(imMask,1);
    
    r = reshape(imROI(sub2ind(size(imROI), pts2dColor(:,2), pts2dColor(:, 1), 1*ones(size(pts2dColor,1), 1))), [numPts+1, numPts+1]);
    g = reshape(imROI(sub2ind(size(imROI), pts2dColor(:,2), pts2dColor(:, 1), 2*ones(size(pts2dColor,1), 1))), [numPts+1, numPts+1]);
    b = reshape(imROI(sub2ind(size(imROI), pts2dColor(:,2), pts2dColor(:, 1), 3*ones(size(pts2dColor,1), 1))), [numPts+1, numPts+1]);
    
    figure;
    h = surf(xq, yq, zq, cat(3, r, g, b), 'FaceColor', 'texturemap', 'EdgeColor', 'texturemap', 'FaceLighting', 'gouraud');
    title('Reconstructed mesh')
    h.XData(pixVal==0) = nan;
    h.YData(pixVal==0) = nan;
    h.ZData(pixVal==0) = nan;
    h.CData(pixVal==0) = nan;
    %     h.EdgeColor = [0.3,0.3,0.3];
    daspect([1 1 1]);
    % view(3);
    axis vis3d tight;
    rotate3d on
end

% convert ROI points 2d to linear indices
pts2dInlierIdx = sub2ind(size(imMask), pts2dInliers(:,2), pts2dInliers(:,1));

% assign color to pt3d
pts3dInliersColor = [];
for i=1:3
    imC = imROI(:,:,i);
    
    % concatenate RGB channel to pt3dColor
    pts3dInliersColor = [pts3dInliersColor, imC(pts2dInlierIdx)];
end

% create ptCloud with colors
ptCloudOut = pointCloud(pts3dInliers, 'Color', pts3dInliersColor);

%% Debug info
if(verbose)
    figure;
    pcshow(ptCloudOut, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 60);
    title('Interpolated point cloud with color')
    daspect([1 1 1]);
    % view(3);
    axis vis3d tight;
    
    figure;
    scatter3(x,y,z, 'bo');
    hold on
    % scatter3(ROIpt3d(:,1),ROIpt3d(:,2),ROIpt3d(:,3),'go');hold off
    mesh(xq, yq, zq);hold off
    title('Interpolated mesh with Nodes in blue circle');
    
    % plot3(x,y,z,'o');
    daspect([1 1 1]);
    % view(3);
    axis vis3d tight;
end
end
