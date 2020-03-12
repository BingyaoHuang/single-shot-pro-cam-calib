function visualizePts3d(pts3d, R, T, figTitle)
%% Visualize 3d reconstruction results with camera & projector extrinsics
% See also: showExtrinsics

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

% projector origin in world space
% camOrg = [0,0,0];
Rt = R';
prjOrg = (Rt*(-T))';

% draw
figure('Name', [figTitle ' 3d point cloud'] );
hold on;
title([figTitle ' 3d point cloud'] );

% plot camera and projector models
cameraSize = 20 ;
plotCamera('Size', cameraSize, 'Color', 'b', 'Label', 'Camera', 'Opacity', 0);
grid on

% matlab buildtin function takes rotated OpenCV's R
plotCamera('Location', prjOrg, 'Orientation', Rt', 'Size', cameraSize, ...
    'Color', 'r', 'Label', 'Projector', 'Opacity', 0);

% Label the axes
xlabel('X', 'fontsize', 30);
ylabel('Y', 'fontsize', 30);
zlabel('Z', 'fontsize', 30);

% plot the 3d points in green
if(~isempty(pts3d))
    h = scatter3(pts3d(:,1),pts3d(:,2),pts3d(:,3), 10, 'go', 'filled',...
        'MarkerEdgeColor', 'k');
end

hold off;
daspect([1 1 1]);
view(3);
% axis vis3d tight;
h.Clipping = 'off'; % disable clipping

rotate3d on
end