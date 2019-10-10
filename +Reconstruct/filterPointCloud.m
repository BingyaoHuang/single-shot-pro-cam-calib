function [pts3d, pts2d, vecDrs, idxOut] = filterPointCloud(pcIn, imMask, param, camW, camH)
%% Filter point cloud using input Intel RealSense depth image as a mask.
% We only keep the points within the depth image range.

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

% convert to double
pts3d = double(pcIn.Location);

% project to 2d image
pts2d = cv.projectPoints(pts3d, [0,0,0], [0,0,0], param.camK, 'DistCoeffs', param.camKc);
pts2d = round(pts2d);

% in case of out of image range due to rounding
inlierIdx = pts2d(:,1)<=camW & pts2d(:,2)<=camH & pts2d(:,1)>0 & pts2d(:,2)>0;

pts2d = pts2d(inlierIdx,:);
pts3d = pts3d(inlierIdx,:);

% get those points' depth values in realsense depth mask image
idxA = sub2ind(size(imMask), pts2d(:,2),  pts2d(:,1));
vecDrs = imMask(idxA);

% only keep points in realsense depth mask image range
idx = vecDrs > 0;
% idx = vecDrs > -inf; % dont filter using depth map, if the point cloud is cleaned
vecDrs = vecDrs(idx);
pts2d = pts2d(idx,:);
pts3d = pts3d(idx,:);

if(nargout == 4)
    tmp = find(inlierIdx);
    idxOut = tmp(idx);
end
end