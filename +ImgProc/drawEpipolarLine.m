function drawEpipolarLine(F, p1, im1, im2, verbose)
%% ImgProc.drawEpipolarLine draws the epipolar lines of p1 using F (transforms p1 to p2, i.e., 
% transforms points p1 in im1 to lines to im2)
% See also: epipolarLine, Reconstruct.findEpipolarInliers, reconstructGUI

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
epiLines = epipolarLine(F, p1);

if(nargin < 5)
    verbose = true;
end

% Compute the intersection points of the lines and the image border.
boarderPoints = lineToBorderPoints(epiLines, size(im2));
offCols = max(size(im1,2),size(im2,2));

if(verbose)
    % Show epipolar lines in the images.
    figure;imshowpair(im1, im2, 'montage');
    title('Epipolar lines');
    hold on
    plot(p1(:,1), p1(:,2), 'go', 'markerfacecolor','g'); 
    hold on 
    line(boarderPoints(:, [1,3])'+ offCols, boarderPoints(:, [2,4])', 'LineWidth', 2); 
    hold off
end

end