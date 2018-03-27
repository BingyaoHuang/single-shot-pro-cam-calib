function [camCorners, usedImIdx] = getCameraCorners(calibInfo)
%% Extract checkerboard corners given calibInfo.

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
% Get a list of images that has 'light' in file name.
imLightNames = ImgProc.getImageNames(calibInfo.path, 'light');
imLightNames = imLightNames(calibInfo.sets);

% extract checkerboard corners
[camCornersTemp, boardSize, usedImIdx] = detectCheckerboardPoints(imLightNames);

if(nnz(boardSize == calibInfo.boardSize) < 2)
    error('Extracted checkerboard size does not match calib-info.yml, please check calib-info.yml and checkerboard image');
end

% initialize corner coordiantes with [-1, -1] if fail to extract corners
% from some images, this is to avoid MT GUI file not found error
numImages = numel(usedImIdx);
numCorners = (boardSize(1)-1)*(boardSize(2)-1);

camCorners = -ones(numCorners,2, numImages);
camCorners(:,:,usedImIdx) = camCornersTemp;

if(nnz(usedImIdx) < length(usedImIdx))
    warning(['Unable to extract checkerboard corners in set(s): ','[', num2str(find(usedImIdx == 0))',']'])
end
end


