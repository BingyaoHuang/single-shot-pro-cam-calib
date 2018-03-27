function [ptCloud, reprjErr] = pointCloudFromPts2d(camPtsImg, prjPtsImg, camK, prjK, R, T)
%% Create point cloud given 2d point pairs and calibration parameters.
% This function create point cloud given 3d points and its 2d projections.
% The camera and projector intrinsics and extrinsics should be provided.

% NOTE: the input camPtsImg and prjPtsImg should be undistorted points
% See also: Reconstruct.triangulatePoints

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

if(nargout > 1)
    % Compute the 3-D points
    [pts3d, reprjErr] = Reconstruct.triangulatePoints(camK, prjK, R, T, camPtsImg, prjPtsImg);
else
    pts3d = Reconstruct.triangulatePoints(camK, prjK, R, T, camPtsImg, prjPtsImg);
end

% Create the point cloud
ptCloud = pointCloud(pts3d);
end

