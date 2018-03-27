function gt = loadGroundTruth(fileName, calibInfo)
%% Load camera and projector parameters ground truth for synthetic calibraiton.

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
params = cv.FileStorage(fullfile(calibInfo.resultDir, fileName));

% load calibration data
gt = params;

% remove skew if it has
gt.camK(1,2) = 0;
gt.prjK(1,2) = 0;

% remove k3 if it has
gt.camKc = gt.camKc(1:4);
gt.prjKc = gt.prjKc(1:4);

%% Calculate camera and projector origin
gt.camOrg = [0, 0, 0];
gt.prjOrg = (-gt.R'*gt.T)';

%% Load calibration data info
gt.calibInfo = calibInfo;

end