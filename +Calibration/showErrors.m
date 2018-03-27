function [err, paramsOut] = showErrors(params, gt, figName, verbose)
%% Compute calibration errors given stereo parameters from synthetic calibration.
% Error types:
% 1. General errors
%   a. Reprojection error
%   b. 3D alignment error: RMS of Euclidean distance between reconstructed and
%   ground truth point cloud.
%   c. Rotation error
%   d. Translation error
% 2. Projector intrinsics errors
% 3. Camera intrinsics errors
% See also: Calibration.poseErr, Calibration.alignmentErrorSynthetic

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

if(nargin < 4)
    verbose = false;
end

% concatenate every plane's gt points for 3D alignment error calculation
gtPtsCamImg = cell2mat(gt.synthData.gtPtsCamImg');
gtPtsPrjImg = cell2mat(gt.synthData.gtPtsPrjImg');
gtPtsWorld =  cell2mat(gt.synthData.gtPtsWorld');

%% unpack parameters
% intrinsics
camK = params.camK;
camKc = params.camKc;
prjK = params.prjK;
prjKc = params.prjKc;

% extrinsics
R = params.R;
T = params.T;

rvec = cv.Rodrigues(R)';
%% Reconstruct gt points using calibrated parameters 
% VERY IMPORTANT!!!!, undistort camera image points first
gtCamPtsUndistort = ImgProc.cvUndistortPoints(gtPtsCamImg, camK, camKc);
gtCrjPtsUndistort = ImgProc.cvUndistortPoints(gtPtsPrjImg, prjK, prjKc);

% triangulate
reconPtsWorld = Reconstruct.triangulatePoints(camK, prjK, R, T, gtCamPtsUndistort, gtCrjPtsUndistort);
paramsOut.reconPtsWorld = reconPtsWorld;

% visualize
if(verbose)
    Reconstruct.visualizePts3d(reconPtsWorld, R, T, figName);
end

%% Reprojection error (sum of cam and prj)
camPts2d = cv.projectPoints(reconPtsWorld, [0 0 0], [0 0 0], camK, 'DistCoeffs', camKc);
prjPts2d = cv.projectPoints(reconPtsWorld, rvec, T', prjK, 'DistCoeffs', prjKc);

% compute residual between reprojected pts2d and captured pts2d
camReprojRes = camPts2d - gtPtsCamImg;
prjReprojRes = prjPts2d - gtPtsPrjImg;

stereoReprojRes = [camReprojRes;prjReprojRes];

% root mean square reprojection error
rmsReprojErr = sqrt( mean(sum(stereoReprojRes.^2, 2)) );

%% 3D alignment error
rmsAlignErr = Calibration.alignmentErrorSynthetic(reconPtsWorld, gtPtsWorld, verbose, figName);

%% Parameteres estimation error
% intrinsics error
camKErr = params.camK - gt.camK;
camKcErr = params.camKc - gt.camKc;
prjKErr = params.prjK - gt.prjK;
prjKcErr = params.prjKc - gt.prjKc;

% pose error (R, T)
[rotErr, transErr] = Calibration.poseErr([params.R, params.T], [gt.R, gt.T] );

%% General errors
err.name = figName;

% reconstruction, reprojection and 3d alignment error
% err.rmsReconErr = rmsReconErr;
err.rmsReprojErr = rmsReprojErr;
err.rmsAlignErr = rmsAlignErr;
% err.mEpipolarErr = mEpipolarErr;

% extrinsic err
err.rotErr = rotErr;
err.transErr = transErr;

%% Projector errors
% projection matrix err (absolute percentage value)
err.fxErrPrj = abs(prjKErr(1,1) / gt.prjK(1,1)) * 100;
err.fyErrPrj = abs(prjKErr(2,2) / gt.prjK(2,2)) * 100;

% projection matrix err (absolute value)
err.cxErrPrj = abs(prjKErr(1,3));
err.cyErrPrj = abs(prjKErr(2,3));

% distortion factors err (absolute percentage value)
err.k1ErrPrj = abs(prjKcErr(1) / gt.prjKc(1)) * 100;
err.k2ErrPrj = abs(prjKcErr(2) / gt.prjKc(2)) * 100;
err.p1ErrPrj = abs(prjKcErr(3) / gt.prjKc(3)) * 100;
err.p2ErrPrj = abs(prjKcErr(4) / gt.prjKc(4)) * 100;

%% Camera error

err.fxErrCam = abs(camKErr(1,1) / gt.camK(1,1)) * 100;
err.fyErrCam = abs(camKErr(2,2) / gt.camK(2,2)) * 100;

% projection matrix err (absolute value)
err.cxErrCam = abs(camKErr(1,3));
err.cyErrCam = abs(camKErr(2,3));

% distortion factors err (absolute percentage value)
err.k1ErrCam = abs(camKcErr(1) / gt.camKc(1)) * 100;
err.k2ErrCam = abs(camKcErr(2) / gt.camKc(2)) * 100;
err.p1ErrCam = abs(camKcErr(3) / gt.camKc(3)) * 100;
err.p2ErrCam = abs(camKcErr(4) / gt.camKc(4)) * 100;

end