function [ corners ] = readCorners(prefix, cornerDir, calibInfo)
%% Read 2-column points from a text file named with a prefix under cornerDir.
% See also: Calibration.saveCorners

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

numCorners = calibInfo.numCorners;
numSets = calibInfo.numSets;

corners = -ones(numCorners,2,numSets);
usedSetIdx = false(numSets, 1);

for i = 1:numSets
    tPath = fullfile(cornerDir, [prefix, sprintf('%02d',calibInfo.sets(i)-1), '.txt']);
    fileInfo = dir(tPath);
    
    if(fileInfo.bytes == 0)
        curCorners = -ones(numCorners,2);
    else
        try
            curCorners = dlmread(tPath);
            if(curCorners(1,1) ~= -1)
                usedSetIdx(i) = 1;
            else
                usedSetIdx(i) = 0;
            end
        catch
            curCorners = -ones(numCorners,2);
            usedSetIdx(i) = 0;
        end
    end
    
    corners(:,:,i) = curCorners;
end

corners = corners(:,:,usedSetIdx);

if(nnz(usedSetIdx) < length(usedSetIdx))
    warning(['Checkerboard corners in set(s) ', '[',num2str(find(usedSetIdx == 0)'),...
        ']', ' are set to -1 since matlab cannot extract checkerboard corners'])
end
end

