function imSkele = bw2skele(imBW)
%% Skeletonize a binary image of grid.

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

imSkele = zeros(size(imBW));
skele5 = bwmorph(imBW,'skel',inf);

while(nnz(imSkele - skele5))
    imSkele = skele5;
    
    % remove spur pixels
    skele2 = bwmorph(imSkele,'spur', inf);
    %figure;imshowpair(imSkele, skele2, 'Montage');
    
    % remove isolated pixels
    skele3 = bwmorph(skele2,'clean', inf);
    %figure;imshowpair(skele2, skele3, 'Montage');
    
    % skele4 = skele3;
    skele4 = bwmorph(skele3,'fill', inf);
    %figure;imshowpair(skele4, skele3, 'Montage');
    
    skele5 = bwmorph(skele4,'thin', inf);
    % figure;imshowpair(skele5, skele4, 'Montage');
    % imshow(skele5-skele);
end

% smooth small spurs
%     se = strel('arbitrary',[0,1,0;1,1,1;0,1,0]);
se1 = strel('line',2,0);
se2 = strel('line',2,90);
imSkele = imclose(imSkele, se1);
imSkele = imclose(imSkele, se2);
imSkele = bwmorph(imSkele, 'thin',inf);

% zero out the boarders
imSkele(1,:) = 0;
imSkele(end,:) = 0;
imSkele(:,1) = 0;
imSkele(:,end) = 0;

end