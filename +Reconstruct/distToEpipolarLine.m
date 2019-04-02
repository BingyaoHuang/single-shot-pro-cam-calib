function d = distToEpipolarLine(F, p1, p2)
%% Given a fundamental matrix F, compute the matched point pair to their 
% corresponding epipolarlines' distances, then sum the two distances.
% Extracted from Matlab built-in function estimateFundamentalMatrix.m

% Very important: F must map a point p1 to an epipolar line in p2 image space, not the other way around.
% More details: http://stackoverflow.com/questions/26582960/sampson-error-for-five-point-essential-matrix-estimation

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

% convert to homogenious 
p1 = [p1, ones(size(p1,1),1)]';
p2 = [p2, ones(size(p2,1),1)]';

% euclidean distance 
d = sum(p2.*(F*p1), 1) .^ 2;

% sampson distance 
epl1 = F * p1;
epl2 = F' * p2;
d = d ./ (epl1(1,:).^2 + epl1(2,:).^2 + epl2(1,:).^2 + epl2(2,:).^2);
end
