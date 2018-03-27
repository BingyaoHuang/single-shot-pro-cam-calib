function calibInfo = loadCalibInfo(calibPath)
%% Load calibration information, such as checkerboard/cam/prj image size.
% See also: calibrate, compareCalibrations

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

% load calibration info
calibInfo = cv.FileStorage(fullfile(calibPath, 'calib-info.yml'));
calibInfo.path = calibPath;
calibInfo.resultDir = fullfile(calibPath, 'results');

% create results dir if does not exist
if ~exist(calibInfo.resultDir, 'dir')
    mkdir(calibInfo.resultDir);
end

% convert cell to mat
if(isa(calibInfo.boardSize,'cell'))
    calibInfo.boardSize = cell2mat(calibInfo.boardSize);
end

if(isa(calibInfo.sets,'cell'))
    calibInfo.sets = cell2mat(calibInfo.sets);
end

% camera and projector image size
calibInfo.camImgSize = [calibInfo.camW, calibInfo.camH];
calibInfo.prjImgSize = [calibInfo.prjW, calibInfo.prjH];

% number of checkerboard corners and used position sets
calibInfo.numCorners = (calibInfo.boardSize(1) - 1) * (calibInfo.boardSize(2) - 1);
calibInfo.numSets = numel(calibInfo.sets);

end