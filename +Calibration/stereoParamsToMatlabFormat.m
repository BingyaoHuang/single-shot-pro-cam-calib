function stereoParamsOut = stereoParamsToMatlabFormat(stereoParamsIn, cbSize)
%% Convert OpenCV format stereo parameters to Matlab native format
% Note, to convert between OpenCV and Matlab, the matrices need to be rotated

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
camIntrinsics = stereoParamsIn.camK';
prjIntrinsics = stereoParamsIn.prjK';

camDistortion = stereoParamsIn.camKc;
prjDistortion = stereoParamsIn.prjKc;

R = stereoParamsIn.R';
T = stereoParamsIn.T';

camRvecs = reshape(cell2mat(stereoParamsIn.camRvecs)', [],3);
camTvecs = reshape(cell2mat(stereoParamsIn.camTvecs)', [],3);

prjRvecs = reshape(cell2mat(stereoParamsIn.prjRvecs)', [],3);
prjTvecs = reshape(cell2mat(stereoParamsIn.prjTvecs)', [],3);

%% convert to matlab format
camParams = constructCamParams(camIntrinsics, camDistortion, camRvecs, camTvecs, cbSize);
prjParams = constructCamParams(prjIntrinsics, prjDistortion, prjRvecs, prjTvecs, cbSize);

stereoParamsOut = stereoParameters(camParams, prjParams, R, T);
end

function camParams = constructCamParams(K, Kc, rVecs, tVecs, cbSize)

% fake WorldPoints since cameraParameters requires this
worldPoints = generateCheckerboardPoints(cbSize, 5);

% construct a struct
camParams = struct('IntrinsicMatrix', K,  ...
    'NumRadialDistortionCoefficients', 2, ...
    'RadialDistortion', Kc(1:2), ...
    'TangentialDistortion', Kc(3:4), ...
    'WorldPoints', worldPoints, ...
    'RotationVectors', rVecs,...
    'TranslationVectors', tVecs);

% convert struct to cameraParameter class
camParams = cameraParameters(camParams);
end