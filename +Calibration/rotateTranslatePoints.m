function [pointsOut] = rotateTranslatePoints(points, tVec, rVec, aboutOrigin)
%% Rotates a set of points about points center or coordinate system origin.

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

R = cv.Rodrigues(rVec);

if (aboutOrigin)
    pointsOut = bsxfun(@plus, R * points', tVec')';
else
    R = makehgtform('xrotate', rVec(1), 'yrotate', rVec(2), 'zrotate', rVec(3), 'translate', tVec);
    center = repmat([tVec(1); tVec(2); tVec(3); 0], 1, size(points, 1));
    points(:, 4) = 1;
    pointsTemp = (R * (points' - center) + center)';
    pointsOut = pointsTemp(:, 1:3);
end
end
