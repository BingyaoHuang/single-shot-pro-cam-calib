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
pts2dColor = pts2d;

% remove points that are out of camera's fov
inlierIdx = (pts2d(:,2) > 0) & (pts2d(:,1) > 0) & (pts2d(:,2) < size(imMask,1)) & (pts2d(:,1) < size(imMask,2));
pts2dColor(~inlierIdx,:) = 1;
pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);

% only keep masked 2d, 3d points
pixVal = interp2(im2single(imMask), pts2d(:, 1), pts2d(:, 2));
pts2dInliers = pts2d(pixVal>0,:);
pts3dInliers = pts3d(pixVal>0,:);

% convert to single for color interpolation
imROI = im2single(imROI);

% assign color to pt3d
r = interp2(imROI(:,:,1), pts2dInliers(:, 1), pts2dInliers(:, 2));
g = interp2(imROI(:,:,2), pts2dInliers(:, 1), pts2dInliers(:, 2));
b = interp2(imROI(:,:,3), pts2dInliers(:, 1), pts2dInliers(:, 2));

% create ptCloud with colors
ptCloudOut = pointCloud(pts3dInliers, 'Color', [r,g,b]);

%% Debug info
if(verbose)
    %% plot point cloud
    figure;
    pcshow(ptCloudOut, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 60);
    title('Interpolated point cloud with color')
    daspect([1 1 1]);
    % view(3);
    axis vis3d tight;
    
    %% plot mesh (edges only)
    figure;
    scatter3(x,y,z, 'bo'); hold on
    mesh(xq, yq, zq);hold off
    title('Interpolated mesh, Nodes are shown as blue circles');
    daspect([1 1 1]);
    % view(3);
    axis vis3d tight;
    
    %% plot surface
    % interpolate color
    r = reshape(interp2(imROI(:,:,1), pts2dColor(:, 1), pts2dColor(:, 2)), [numPts+1, numPts+1]);
    g = reshape(interp2(imROI(:,:,2), pts2dColor(:, 1), pts2dColor(:, 2)), [numPts+1, numPts+1]);
    b = reshape(interp2(imROI(:,:,3), pts2dColor(:, 1), pts2dColor(:, 2)), [numPts+1, numPts+1]);

    figure;
    h = surf(xq, yq, zq, cat(3, r, g, b), 'FaceColor', 'texturemap', 'EdgeColor', 'texturemap', 'FaceLighting', 'gouraud', 'LineStyle', 'none');
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
end
