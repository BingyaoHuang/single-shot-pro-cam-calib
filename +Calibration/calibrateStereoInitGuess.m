function stereoParams = calibrateStereoInitGuess(modelPtsCell, camPtsCell, prjPtsCell, camParams, prjParams, calibInfo)
%% Stereo calibration (used by global homography and Moreno & Taubin)

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
% OpenCV calibrateCamera and stereoCalibrate criteria struct
calibCriteria = struct('type', 'Count+EPS', 'maxCount', 1000, 'epsilon', 1e-6);

% 1. OpenCV stereo calibration
stereoParams = cv.stereoCalibrate(modelPtsCell, camPtsCell, prjPtsCell, calibInfo.camImgSize, ...
    'Criteria',  calibCriteria, 'FixIntrinsic', true, ...
    'CameraMatrix1', camParams.camK, 'CameraMatrix2', prjParams.camK, ...
    'DistCoeffs1', camParams.camKc, 'DistCoeffs2', prjParams.camKc);


stereoParams.camK = camParams.camK;
stereoParams.camKc = camParams.camKc(1:4);

% projector intrinsics
stereoParams.prjK = prjParams.camK;
stereoParams.prjKc = prjParams.camKc(1:4);

% camera extrinsics
stereoParams.camRvecs = camParams.rVecs;
stereoParams.camTvecs = camParams.tVecs;

% projector extrinsics
stereoParams.prjRvecs = prjParams.rVecs;
stereoParams.prjTvecs = prjParams.tVecs;

% for point cloud display
stereoParams.modelPts = modelPtsCell;
for i = 1:calibInfo.numSets  
    % use ri,ti to convert to world space
    stereoParams.worldPts{i} = Calibration.rotateTranslatePoints(...
        stereoParams.modelPts{i}, stereoParams.camTvecs{i}', stereoParams.camRvecs{i}, true);
end

stereoParams.camImgPts = camPtsCell;
stereoParams.prjImgPts = prjPtsCell;

% remove OpenCV fields
stereoParams = rmfield(stereoParams, {'cameraMatrix1', 'cameraMatrix2', 'distCoeffs1', 'distCoeffs2','reprojErr'});

end