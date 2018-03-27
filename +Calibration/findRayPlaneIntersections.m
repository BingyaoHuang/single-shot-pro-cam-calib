function [pts3d, inlierIdx, intersectBoardsIdx] = findRayPlaneIntersections(planes, rayDirs, rayStartPoint, noiseSigma, verbose)
%% Find 3d intersections between projector rays and 3d planes
% Calculates the intersections between projector rays and 3d white board
% planes, the intersections are then checked if they are within the area
% of the calibration board. The 3d intersections and their corresponding 2d
% inliers are returned.

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
% number of rays
nRays = length(rayDirs);

% prepare ray direction and start point for intersection calculation
v = rayDirs; % ray directions
p0 = repmat(rayStartPoint, nRays, 1); % ray start point

% intersection points and inlier indices
pts3d = [];
inlierIdx = [];
intersectBoardsIdx = []; % stores intersected plane (board)'s indices

if(verbose)
    % plane colors for visualization
%     planeColors = hsv(length(planes));
    
    % Draw current plane        
    Calibration.plotPlanes(planes);
end

% for each plane we intersect all rays to it
for i = 1:length(planes)
    d = planes(i).d;
    N = repmat(planes(i).normal, nRays, 1);
    
    % the distance beween the intersection and p0
    t = -(dot(p0, N, 2) + d) ./ dot(v, N, 2);
    
    % add noise 
    t = t + noiseSigma*randn(size(t));
    
    curInters = p0 + t .* v;
    
    % only keep intersections within the calibration board range
    [curInters, curInlierIdx] = keepInliers(curInters, planes(i));
    
    % store the current plane's intersection inliers and their indices
    pts3d{i} = curInters;
    inlierIdx{i} = curInlierIdx;
    intersectBoardsIdx = [intersectBoardsIdx, i];
    
    % visualization
    if(verbose)
               
        % Plot intersections
        hold on
        scatter3(curInters(:,1), curInters(:,2), curInters(:,3), 'g.');        
        
        % Plot camera checkerboard corners
        %         scatter3(cbPtsCamView(:,1,j), cbPtsCamView(:,2,j), cbPtsCamView(:,3,j), 'bo');
    end
end

end


%% Keep only intersections within plane rectangle range

function [curInters, inlierIdx] = keepInliers(curInters, plane)

nInters = length(curInters);

% 3 corner points of the current plane
c1 = plane.corners(1,:);
c2 = plane.corners(2,:);
c4 = plane.corners(4,:);

% two coordinate bases formed from the 3 points above
v1 = c2 - c1;
v2 = c4 - c1;

% vp is a vector from c1 to an intersection point p, i.e., v = p - c1
vp = curInters - c1;

% we project v to v1 and v2, since v1 and v2 are perpendicular, the
% projections should be smaller than v1 and v2 length and sign should
% be positive
vdotv1 = dot(vp, repmat(v1, nInters, 1), 2);
vdotv2 = dot(vp, repmat(v2, nInters, 1), 2);

% An intersection point p that is within rectangle plane only when:
% 1. dot products dot(vp, v1) > 0 and dot(vp, v2) > 0
% 2. vp's projection onto v1 and v2 are smaller than v1 and v2, i.e.,
%    |vp|*|v1|*cos(theta) < |v1|^2, |vp|*|v2|*cos(phi) < |v2|^2
cond1 = vdotv1 > 0 & vdotv2 > 0;
cond2 = vdotv1 < sum(v1.^2) & vdotv2 < sum(v2.^2);
inlierIdx = cond1 & cond2;

% keep only inlier intersections
curInters = curInters(inlierIdx,:);


%% In case of parallelogram shape instead of rectangle
% an intersection point p should be a linear combiantion of the 2 bases
% above, i.e., p = t1*v1 + t2*v2, and 0<t1<1, 0<t2<1 if the point is
% within the square, thus:
%  px = t1*v1x + t2*v2x;
%  py = t1*v1y + t2*v2y;
%  pz = t1*v1z + t2*v2z;
% we solve the equation above to get t1 and t2 by forming a linear
% equation: Ax = b, where A = [v1x, v2x; v1y, v2y], x = [t1; t2], b =
% [px; py], thus we have
% A = [v1x, v2x; v1y, v2y];
% b = [px, py];
% x = A\b;

%% In case of general polygon
% https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not


end