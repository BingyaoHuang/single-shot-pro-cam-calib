function imOut = maskImage(im, imMask)
%% Mask im using a mask imMask, returns the masked image

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
if(ndims(im) < 3)
    if(isa(im, 'logical'))
        imOut = logical(im.*imMask);
    elseif(isa(im, 'uint8'))
        imOut = im.*uint8(imMask);
    elseif(isa(im, 'double'))
        imOut = im.*double(imMask);
    end
elseif(isa(im, 'uint8'))
    imOut = im.*repmat(uint8(imMask),[1,1,3]);
elseif(isa(im, 'double'))
    imOut = im.*repmat(double(imMask),[1,1,3]);
end
end