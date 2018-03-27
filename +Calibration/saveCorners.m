function saveCorners(corners, prefix, cornerDir, sets, usedImIdx)
%% Save extracted camera/projector checkerboard corners to a text file named with a prefix under cornerDir.
% See also: Calibration.readCorners

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
if ~exist(cornerDir, 'dir')
    mkdir(cornerDir);
end

% save corners of all selected sets to txt files with a file name prefix

for i = 1:size(corners, 3)
    % file index starts from 0
    fId = fopen(fullfile(cornerDir, [prefix, sprintf('%02d', sets(i)-1), '.txt']), 'w');
    fprintf(fId, '%3.6f %3.6f\r\n', corners(:,:,i)');
    fclose(fId);
end

% if it's camera corners, also write usedImIdx to file

if (strcmp(prefix, 'cam_'))
    fId = fopen(fullfile(cornerDir, 'usedImIdx.txt'), 'w');
    fprintf(fId, '%2.0d %2.0d \r\n', [sets', usedImIdx]');
    fclose(fId);
end

end
