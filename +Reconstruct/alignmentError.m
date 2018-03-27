function err = alignmentError(pcCalib, pcGroundTruth, param, camW, camH, verbose, figName)
%% Calculate alignment error between calibration and Intel RealSense F200 captured point cloud.
% Calculate and compare the 3D alignment errors between reconstructed 
% point cloud and Intel RealSense F200 captured point cloud. Four 
% calibration methods are compared.

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
if(nargin < 7)
    figName = '';
end

pts3d = pcCalib.Location;

% convert point cloud to depth map
imD = (reshape(pcGroundTruth.Location(:,3), camW, camH))';
imD = double(imD);

%% interpolate point cloud
imMask = zeros(size(imD));
imMask(imD > 0) = 1;
rp = regionprops(imMask, 'boundingbox');
npX = rp.BoundingBox(3) / 2;
npY = rp.BoundingBox(4) / 2;

% xRange contains lower and higher bounds
xRange = double(pcCalib.XLimits);
xDiff = xRange(1) - xRange(2);
xStep = -xDiff / npX;

yRange = double(pcCalib.YLimits);
yDiff = yRange(1) - yRange(2);
yStep = -yDiff/npY;

% use griddata to interpolate a mesh from given point cloud
[xq,yq] = meshgrid(xRange(1):xStep:xRange(2), yRange(1):yStep:yRange(2));
zq = griddata(pts3d(:,1),pts3d(:,2),pts3d(:,3), xq, yq,'cubic');

% pts3dInterp is the interpolated denser point cloud
pts3dInterp = [xq(:), yq(:), zq(:)];

%% crop extra interpolated mesh using ground truth point cloud
[~, pts3dInterpAligned, ~] = pcregrigid(pointCloud(pts3dInterp), pcGroundTruth,'Extrapolate', true);
[~, ~, ~, pcInlierIdx] = Reconstruct.filterPointCloud(pts3dInterpAligned, imD, param, camW, camH);
pcOutlierIdx = setdiff(1:size(xq,1)*size(xq,2), pcInlierIdx);

if(verbose)
    figure('Name', figName);
    h = surf(xq, yq, zq, 'FaceColor', 'interp', 'FaceLighting', 'gouraud');
    
    % remove interpolated meth that are out of ground truth range
    h.XData(pcOutlierIdx) = nan;
    h.YData(pcOutlierIdx) = nan;
    h.ZData(pcOutlierIdx) = nan;
    h.CData(pcOutlierIdx) = nan;
    
%     h.EdgeColor = 'none';
    h.EdgeColor = [0.3,0.3,0.3];
    h.LineStyle = ':';
    colormap jet
    
    % title
    title(figName)
    caxis([-50 50])
    colorbar
    
    % remove ticks
    set(gca,'xtick',[])
    set(gca,'xticklabel',[])
    set(gca,'ytick',[])
    set(gca,'yticklabel',[])
    set(gca,'ztick',[])
    set(gca,'zticklabel',[])
    
    % font
    set(gca, 'FontSize', 20);
    daspect([1 1 1]);
    axis vis3d tight
    
    rotate3d on
end

% light('Position',[0 1 -1]) 

%% calculte 3D alignment error using knn search
[~, err] = knnsearch(pcGroundTruth.Location, pts3dInterp);

% visualize errors as pseudocolor
if(verbose)
    errOutlierIdx = isnan(err);
    errInlierIdx = ~errOutlierIdx;
    h.ZData(errOutlierIdx) = nan;
    h.CData(errInlierIdx) = err(errInlierIdx); % use err as color      
end

% remove nan values
err = err(~isnan(err));
end
