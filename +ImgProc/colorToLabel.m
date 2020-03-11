function [imAllLabel,imHoriLabel, imVertLabel] = colorToLabel(imColorGrid, imNode, imHoriEdge, imVertEdge, verbose)
%% Extract color labels 1,2,3,...,8 from color grid image.

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

if(nargin < 5)
    verbose = false;
end

if(verbose)
    figure('Name', 'colorToLabel', 'units','normalized','outerposition',[0 0  1  1]);
    disp('converting color grid image to labeled images')
end

%% Create wide imNode, imHoriEdge and imVertEdge for color masking
imHoriRGB = maskColorGrid(imColorGrid, imNode, imHoriEdge);
imVertRGB = maskColorGrid(imColorGrid, imNode, imVertEdge);

%% Use kmeans for horizontal and vertical color detection
[imHoriLabel, imHoriRecv] = clusterColors(imHoriRGB, 1);
[imVertLabel, imVertRecv] = clusterColors(imVertRGB, 0);

if(verbose)
    figure;
    subplot(2,1,1);
    imshowpair(imHoriRGB, imHoriRecv, 'Montage');
    title('[horizontal] Original color grid and detected labels in pseudocolor');
    drawnow
    
    subplot(2,1,2);
    imshowpair(imVertRGB, imVertRecv, 'Montage');
    title('[vertical] Original color grid and detected labels in pseudocolor');
    drawnow
end

%% combine horizontal and vertical labels
imAllLabel = imHoriLabel + imVertLabel;

end

%% Local functions
function imMaskedRGB = maskColorGrid(imEnhance, imNode, imEdgeMask)

% dilated imNode
imNodeWide = imdilate(imNode, strel('disk',1));
%     figure;imshow(imNodeWide);
%     title('Dilated Nodes');

% subtract imNodeWide from imHoriWide to get rid of color interference at
% Node intersection
imEdgeMaskWide = imdilate(imEdgeMask, strel('disk',1));
imMaskedRGB = ImgProc.maskImage(imEnhance, imEdgeMaskWide - imNodeWide);
%     figure;
%     imshowpair(imEdgeMaskWide, imMaskedRGB, 'Montage');
%     title('Masked colors');

end

%% Local functions
function [imLabelOut, imRecoverRGB] = clusterColors(imRGB, isHorizontal)
k = 4;
% for horizontal stripes, there are 5 ranges due to red is (0~0.125)
% and (0.95, 1)
if(isHorizontal)
    k = 5;
end

imHSV = rgb2hsv(imRGB);
imHue = imHSV(:,:,1);
imSat = imHSV(:,:,2);
imVal = imHSV(:,:,3);
colorPixelIdx = find(imVal>0);

if(length(colorPixelIdx) < 10)
    error('Not enough colored pixel to cluster, probably no edges are detected');
end

vecHue = imHue(colorPixelIdx);
vecSat = imSat(colorPixelIdx);
vecVal = imVal(colorPixelIdx);

vecHSV = [vecHue, vecSat, vecVal];

% Liu, Dongju, and Jian Yu. "Otsu method and K-means." In 2009 Ninth International Conference on Hybrid Intelligent Systems, vol. 1, pp. 344-349. IEEE, 2009.
% Otsu is the global optimal k-means, although it is a little slower.
% figure; histogram(vecHue, 360);
use_kmean = 0;
if use_kmean
    % incase local minima, replicate 5 times
    [vecLabels, centroids] = kmeans(vecHue, k, 'Replicates', 5);
        [sortedCentroids, idx] = sort(centroids(:,1), 1, 'ascend');
    vecSortedLabels = vecLabels;
    
    for i=1:numel(idx)
        vecSortedLabels(vecLabels == idx(i)) = i;
    end

    vertValsMins = zeros(numel(idx),1);
    minVals = vertValsMins;
    if(any(diff(sortedCentroids) < 45/360))
        if(~isHorizontal)
            vertVals = [45 150 225 315]/360;
        else
            vertVals = [0 90 180 285 360]/360;
        end
        
        for i = 1:numel(idx)
            val = sortedCentroids(i);
            [minVal, iidx] = min(abs(vertVals - val));
            vertValsMins(i) = iidx;
            minVals(i) = minVal;
        end
        
        vecSortedLabels2 = zeros(size(vecLabels));
        
        for i = 1:numel(vertValsMins)
            vecSortedLabels2(vecLabels == idx(i)) = vertValsMins(i);
        end
        
        vecSortedLabels = vecSortedLabels2;
    end
else
    % Otsu
    thresh = multithresh(vecHue, k-1);
    vecLabels = imquantize(vecHue, thresh);
    centroids = mean([0, thresh; thresh, 1])';
    vecSortedLabels = vecLabels;
end

if(isHorizontal)    %correct for red being on either side
    vecSortedLabels(vecSortedLabels==5) = 1;
end

imSize = size(imHue);
imLabel = zeros(imSize,'uint8');
imLabel(colorPixelIdx) = vecSortedLabels;

%     figure;imagesc(imLabel);
%     figure;imagesc(imRGB);

%     figure;
%     imshowpair(imRGB, imLabel, 'Montage');
%     title('RGB and labels');

if(isHorizontal)
    % horizontal colors
    imRecoverHue = double(imLabel-1)*0.25;
    imLabelOut = 2*imLabel-1;
else
    % vertical colors
    imRecoverHue = double(imLabel-1)*0.25+0.125;
    imLabelOut = 2*imLabel;
end

imRecoverHue(imLabel == 0) = 0;

imRecoverHSV = imHSV;
imRecoverHSV(:,:,1) = imRecoverHue;
imRecoverHSV(:,:,2) = ones(imSize,'double');  % saturation
imV = zeros(imSize,'double');
imV(colorPixelIdx) = 1;
imRecoverHSV(:,:,3) = imV;
imRecoverRGB = hsv2rgb(imRecoverHSV);  % value

end
