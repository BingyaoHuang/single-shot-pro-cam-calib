function [imBWGrid, imColorGridMasked, imBWBoardMask] = segColorGrid(whiteLight, colorGrid, camCorners, verbose)
%% Extracts the color grid on white board.
% See also: ImgProc.getMatchedNotes

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

imLight = imread(whiteLight);
imColorGrid = imread(colorGrid);

if(verbose)
    figure('Name', 'segColorGrid', 'units','normalized','outerposition',[0 0 1 1]);
end

%% Flood fill using the 1st checkerboard corner
% Convert RGB image into HSV color space.
% imHsv = rgb2hsv(imLight);
% imV = imHsv(:,:,3); % intensity channelim
% imLightEnhance = ImgProc.imadjust3(imLight);
% imGray = mat2gray(rgb2gray(imLightEnhance)); % normalized gray

% find color grid pixels
im1 = rgb2hsv(imColorGrid);
im2 = rgb2hsv(imLight);
s = mean(im1(:,:,3)) / mean(im2(:,:,3));
s = min(s, 1);
imGrid = imColorGrid-s*imLight;
imGrayGrid = rgb2gray(imColorGrid-s*imLight);
imGridROI = bwconvhull(bwareafilt(imGrayGrid > mean(imGrayGrid(:)), 1));
imLightMasked = ImgProc.maskImage(imLight, imGridROI);
imGrayMasked = rgb2gray(imLightMasked); % normalized gray

imBWBoard = false(size(imGrayMasked));

% get bounding box corners of the checkerboard
maxXY = max(camCorners);
minXY = min(camCorners);

permsX = perms([minXY(:,1), maxXY(:,1)]);
permsY = perms([minXY(:,2), maxXY(:,2)]);

% check the best flood fill results when using the 4 corners
winSize = 5;

for i = 1:length(permsX)
    for j = 1:length(permsY)
        % init seed set to corner
        x = round(permsX(i));
        y = round(permsY(j));
        
        % find the brightest pixel index in a 5x5 local roi
        imRoi = imgaussfilt(imGrayMasked(y-winSize:y+winSize,x-winSize:x+winSize));
        [maxVal, maxIdx] = max(imRoi(:));
        
        % convert back to global row and col
        [row, col] = ind2sub(size(imRoi), maxIdx);
        col = col + x - winSize - 1;
        row = row + y - winSize - 1;
        
        % Flood fill using geodesic distance
        imCurSeg = ImgProc.segFloodFill(imLightMasked, row, col);
        imBWBoard = imBWBoard | imCurSeg;
        
        if(verbose)
            %             fs(imCurSeg);hold on; plot(col,row,'ro');
        end
    end
end

% imBWBoard = bwconvhull(imBWBoard);

% imWeight = graydiffweight(imLabNorm, col, row, 'GrayDifferenceCutoff', tol);
% imBWBoard = imsegfmm(imWeight, col, row, 0.01);

if(verbose)
    subplot(2,2,1);
    imshow(imBWBoard);
    title('Flood fill white board');
    drawnow
end

%% Crop checkerboard area

% find checkerboard area
imBWCb = ~imBWBoard;

seeds = [];
for i = 1:length(camCorners)
    x = round(camCorners(i,1));
    y = round(camCorners(i,2));
    imBWCb(y-winSize:y+winSize,x-winSize:x+winSize) = 1;
    
    % seeds for bwselect
    seeds = [seeds; [x,y]];
end

% select the checkerboard area using all corners as seeds
imBWCb = bwselect(imBWCb, seeds(:,1), seeds(:,2), 8);

imBWCb = bwconvhull(imBWCb);
% imBWCb( round(minXY(:,2)):round(maxXY(:,2)), round(minXY(:,1)):round(maxXY(:,1))) = 1;
imBWBoardMask = imfill(imBWBoard, 'holes');
imBWBoardMask(imBWCb(:)) = 0;

if(verbose)
    subplot(2,2,2);
    imshow(imBWBoardMask);
    title('White board without checkerboard area');
    drawnow
end

%% Segment color grid
% im1 = rgb2hsv(imColorGrid);
% im2 = rgb2hsv(imLight);
% s = mean(im1(:,:,3)) / mean(im2(:,:,3));
% s = min(s, 1);
imColorGridMasked = ImgProc.maskImage(imGrid, imBWBoardMask);

% enhance color
imColorGridMasked = ImgProc.imadjust3(imColorGridMasked);

% to hsv
imHsv = rgb2hsv(imColorGridMasked);

% adaptive thresholding using Gaussian filter
sigma = 3;
imBWGrid = ImgProc.adaptiveThresh(imHsv(:,:,3), sigma);

% only keep the largest area
imBWGrid = bwareafilt(imBWGrid, 1);
% imBWGrid = bwmorph(imBWGrid,'clean', inf);

% close and open
se1 = strel('line',2,0);
se2 = strel('line',2,90);
imBWGrid = imclose(imBWGrid, se1);
imBWGrid = imclose(imBWGrid, se2);

% clean edges
imBWCb = imdilate(imBWCb, ones(8, 8));
imBWGrid(imBWCb(:)) = 0;

if(verbose)
    subplot(2,2,3);
    imshow(imColorGridMasked);
    title('Color grid masked');
    drawnow
    
    subplot(2,2,4);
    imshow(imBWGrid);
    title('Color grid binary mask');
    drawnow
end

% imColorGridMasked = ImgProc.maskImage(imColorGridMasked, imBWGrid);
end