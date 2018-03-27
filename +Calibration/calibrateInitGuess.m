function camParams = calibrateInitGuess(modelCornersCell, camCornersCell, calibInfo)
%Calibrate a single camera or projector using OpenCV's calibrateCamera
% See also: cv.calibrateCamera

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

% 2. OpenCV camera calibration
[camK, camKc, reprojErr, rVecs, tVecs] = cv.calibrateCamera(modelCornersCell, camCornersCell, calibInfo.camImgSize, ...
    'Criteria', calibCriteria, 'FixK3', true);

camParams.camK = camK;
camParams.camKc = camKc(1:4);
camParams.reprojErr = reprojErr;
camParams.rVecs = rVecs;
camParams.tVecs = tVecs;
end