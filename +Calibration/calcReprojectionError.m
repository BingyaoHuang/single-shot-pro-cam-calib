function paramOut = calcReprojectionError(paramIn, modelPts, camImgPts, prjImgPts)
%% Calculates reprojection errors using calibrated cam and prj parameters.
% NOTE: the reprojection errors are calculated differently from OpenCV 
% stereoCalibrate()'s stereo reprojection error. We consider R, T errors 
% are part of the projector's calibration error.

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
paramOut = paramIn;

% intrinsics
camK = paramIn.camK;
camKc = paramIn.camKc;

prjK = paramIn.prjK;
prjKc = paramIn.prjKc;

% extrinsics
R = paramIn.R;
T = paramIn.T;

camRvecs = paramIn.camRvecs;
camTvecs = paramIn.camTvecs;

% residuals
camRes = [];
prjRes = [];

for i=1:length(modelPts)
    %% Camera residual
    camCurViewRes = camImgPts{i} - cv.projectPoints(modelPts{i}, camRvecs{i}, camTvecs{i}, camK, 'DistCoeffs', camKc);
    camRes = [camRes; camCurViewRes];
    paramOut.camPerViewReprojErr{i} = sqrt(mean(sum(camCurViewRes.^2, 2)));
    
    %% Projector residual (different from OpenCV's single camera reprojection error, we incorporate extrinsics error here)
    % the rotation and translation are calculated using checkerbord to camera rvec,tvec and
    % estimated camera to projector R,T from stereo calibration.
    camR = cv.Rodrigues(camRvecs{i});
    prjRvecs{i} = cv.Rodrigues(R*camR);
    prjTvecs{i} = R*camTvecs{i} + T;
    
    prjCurViewRes = prjImgPts{i} - cv.projectPoints(modelPts{i}, prjRvecs{i}, prjTvecs{i}, prjK, 'DistCoeffs', prjKc);
    prjRes = [prjRes; prjCurViewRes];
    paramOut.prjPerViewReprojErr{i} = sqrt(mean(sum(prjCurViewRes.^2, 2)));
    
    paramOut.stereoPerViewReprojErr{i} = sqrt(mean(sum([camCurViewRes; prjCurViewRes].^2, 2)));
end

% camera and projector RMS reprojection error
paramOut.camReprojErr = sqrt(mean(sum(camRes.^2, 2)));
paramOut.prjReprojErr = sqrt(mean(sum(prjRes.^2, 2)));

% stereo RMS reprojection error
stereoRes = [camRes; prjRes];
paramOut.stereoReprojErr = sqrt( mean(sum(stereoRes.^2, 2)) );
end