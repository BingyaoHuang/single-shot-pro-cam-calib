function [rotErrDeg, transErr] = poseErr(m1, m2)
%% Compute pose error between input pose matrix m1 and pose matrix m2
% This function computes the pose error between input pose 1 and pose 2,
% where a pose matrix is a 3x4 matrix [R|T]. Where 
% rotErrDeg:  rotation vector angle error
% transErr:  translation vector error
% See also: Calibration.showErrors

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

% convert to 4x4 matrix for multiplication
m1 = [m1; [0,0,0,1]];
m2 = [m2; [0,0,0,1]];

% pose diff matrix
% m = inv(m1)*m2;
m = m1\m2; % same as the equation above

% rotation error
rotDiff = m(1:3, 1:3);
rotErrRad = acos( (trace(rotDiff) - 1) / 2 );
rotErrDeg = rotErrRad*180/pi;

% translation error
transDiff = m(1:3, 4);
transErr = norm(transDiff);
end