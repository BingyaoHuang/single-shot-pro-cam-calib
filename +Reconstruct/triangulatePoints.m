function [pts3d, reprjErr] = triangulatePoints(camK, prjK, R, T, camPtsImg, prjPtsImg)
%% Triangulate 3d points given camera and projector image points and params.
% Triangulate 3d world points given their projections in camera and
% projector image space, as well as camera and projector intrinsics and
% extrinsics.
% See also: triangulate

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

% projection and view matrix in OpenCV format
camPV = camK*[eye(3),zeros(3,1)];

prjViewMat = [R, T];
prjPV = prjK*prjViewMat;

% Compute the 3-D points
if(nargout > 1)
    [pts3d, reprjErr] = triangulate(camPtsImg, prjPtsImg, camPV', prjPV');
else
    pts3d = triangulate(camPtsImg, prjPtsImg, camPV', prjPV');
end

end