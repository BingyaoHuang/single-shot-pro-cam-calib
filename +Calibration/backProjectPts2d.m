function rayDirs = backProjectPts2d(pts2d, prjOrg, prjK, prjKc, R, T, verbose)
%% Back project projector grid points to 3d space as rays using projector intrinsics and R, T. 
% To simulate the real physics on projector, we need to distort the 
% projector grid points before projection.

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
% [very important] should undistort projector raw points rather than distort
pts2dUndistort = ImgProc.cvUndistortPoints(pts2d, prjK, prjKc);

% Scale of the ray length for plotting the ray
s = 1;

% number of points
nPts = length(pts2dUndistort);

% Convert prjPointsImg to homogenious coord
pts2dHomo = [pts2dUndistort, ones(nPts, 1)];

% Ray transform only invovles rotation, no translation
% a point at infinity on the projector ray in world coord system
pts3dInf = (R' * (prjK \ pts2dHomo'*s - T))';
% prjPtWorld = (R' * (prjK_inv * prjPointsImgHomo'*s - T))'; % slower

% Calculate ray directions
rayDirs = pts3dInf - prjOrg;
% rayDirs = rayDirs./norm(rayDirs);


% visualize projector, camera and rays in world space
if(verbose)
    figure('Name', 'Projected rays');
    title('Back projected rays from 2d image to 3d space');
    hold on;
    
    % Plot camera
    cameraSize = 10;
    plotCamera('Size', cameraSize, 'Color', 'b', 'Label', 'Camera', 'Opacity', 0);
    
    % Plot projector
    plotCamera('Location', prjOrg, 'Orientation', R, 'Size', cameraSize, ...
        'Color', 'r', 'Label', 'Projector', 'Opacity', 0);
    
    % Plot projector rays
    for i = 1:size(pts3dInf, 1)
        plot3([prjOrg(1), pts3dInf(i,1)], [prjOrg(2), pts3dInf(i,2)], [prjOrg(3), pts3dInf(i,3)], 'r');
    end
    
    % Label the axes
    xlabel(gca, 'X (mm)');
    
    % note that the Y and the Z axes are switched
    ylabel(gca, 'Y (mm)');
    zlabel(gca, 'Z (mm)');
    
    % adjust display params
    grid on
    daspect([1 1 1]);
    view(3);
    axis vis3d tight;
    set(gca,'XAxisLocation','top','YAxisLocation', 'left');
    drawnow;
end
end