%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute 3d alignment error given params
% Author: Bingyao Huang <hby001@gmail.com>
% Date: 09/06/2017

%% Descriptions
% This function computes 3d alignment error given params. Alignment error
% is the mean distance between reconstructed 3d points and the ground truth
% 3d points.

%% Coordinate system (very important)
% OpenCV:  camera right is +X, camera up is -Y and camera forward is +Z.
% World origin is camera optical center, thus camera view space = world 
% space.

% R and T are rotation matrix and translation vector that brings a point in
% camera view space (world space) to projector view space.

% So a point in projector space Pprjview can be expressed in camera view
% (world) space as: Pworld = Pcamview = R'*Pprjview + (-R'*T)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rmsAlignErr] = alignmentErrorSynthetic(estPts3d, gtpts3d, verbose, figName)

% alignment residual
alignRes = estPts3d - gtpts3d;

% squared alignment error
squaredAlignErr = alignRes.^2;

% sum of squared alignment error
sse = sum(squaredAlignErr, 2);

% visualize
if(verbose)
    figure('Name', [figName, ' Alignment Error']);
    hold on;
    title([figName, ' Alignment Error']);
    
    % boxplot alignment error in X, Y, Z direction
    subplot(1,2,1);
    boxplot(abs(alignRes), 'Notch','on');
    title('X,Y,Z direction absolute alignment error (mm)');
   
    % boxplot total alignment error
    subplot(1,2,2);
    boxplot(sqrt(sse), 'Notch','on')
    title('Total absolute alignment error (mm)');
end

% root mean squared total alignment error (rmse)
rmsAlignErr = sqrt(mean(sse));

end