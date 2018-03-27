function im = genStructuredLight(prjW, prjH)
%% Generate color-coded structured light pattern for calibration.
% prjW and prjH are the projector screen resolution's width and height in pixel.

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

[horiList, vertList, horiPos, vertPos] = ImgProc.createDeBruijnSeq(prjW, prjH);

r = [1, 0, 0];
y = [1, 0.75, 0];
l = [0.5, 1, 0];
g = [0, 1, 0.5];
c = [0, 1, 1];
b = [0, 0.25, 1];
p = [0.75, 0, 1];
m = [1, 0, 0.75];

colorList = [r; y; l; g; c; b; p; m];

% rgb image
im = zeros(prjH, prjW, 3);

for j = 1:length(vertPos)
    curPos = vertPos(j);
    curPixel = reshape(colorList(vertList(j),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [prjH, 3, 1]);
    im(:,curPos-1:curPos+1,:) = curStripe;
end

for i = 1:length(horiPos)
    curPos = horiPos(i);
    curPixel = reshape(colorList(horiList(i),:), [1, 1, 3]);
    curStripe = repmat(curPixel, [3, prjW, 1]);
    im(curPos-1:curPos+1,:,:) = curStripe;
end

% fs(im)
end

