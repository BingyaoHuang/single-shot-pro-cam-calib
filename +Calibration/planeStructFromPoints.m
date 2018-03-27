function planes = planeStructFromPoints(planePoints)
%% Create plane structures from 3d points using 3-point algorithm.
% See also: Calibration.plotPlanes

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
nPtsPerPlane = size(planePoints, 1);

if (nPtsPerPlane < 3)
    error('Need at least 3 points to create a plane');
end

nPlanes = size(planePoints, 3);

% Generate planes
planes = [];

for i = 1:nPlanes
    p1 = planePoints(1, :, i);
    p2 = planePoints(2, :, i);
    p3 = planePoints(3, :, i);
    
    % create plane from 3 points
    plane = planeFrom3Pts(p1, p2, p3);
    
    % store this plane's 4 corners
    plane{1, 1}.corners = planePoints(:, :, i);
    planes = [planes, plane{1, 1}];
end

end

%% Local functions
% This function calculate planes from 3 points,
% of planes is lenght of p1/2/3

function planeOut = planeFrom3Pts(p1, p2, p3)

if (numel(p1) ~= numel(p2) || numel(p1) ~= numel(p3))
    error('Input vector lenghts must be equal!');
end

planeOut{size(p1, 1)} = [];

for i = 1:size(p1, 1)
    
    v1 = p1(i, :) - p2(i, :);
    v2 = p3(i, :) - p2(i, :);
    
    normal = cross(v1, v2);
    
    if (normal <= 0)
        error('Normal length is negative or zero!');
    end
    
    % plane normal
    plane.normal = normal / norm(normal);
    
    % a point on plane
    plane.point = p2(i, :);
    
    % distance between plane and origin
    plane.d =- (dot(plane.normal, plane.point));
    planeOut{i} = plane;
    
end

end
