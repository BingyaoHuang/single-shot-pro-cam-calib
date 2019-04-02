function [inlierIdx, d] = findEpipolarInliers(F, camPoints, prjPoints, thresh, verbose)
%% findEpipolarInliers outputs epipolar inliers indices and the distances.

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

%% calculate point to epipolar sampson distance
d = Reconstruct.distToEpipolarLine(F, camPoints, prjPoints);

%% use percentile as epipolar distance threshold
if(nargin <= 4)
    thresh = min(prctile(d,90),thresh);
    verbose = false;
end

if(nargin < 5)
    thresh = min(prctile(d,90),thresh);
    verbose = true;
end

inlierIdx = d < thresh;

if(verbose)
    figure; 
    boxplot(d, 'Notch','on');
    title('Epipolar distances');
    h = findobj(gcf,'tag','Upper Whisker');
    upperLim = h.YData;
    ylim([0,upperLim(2)])
end

end
