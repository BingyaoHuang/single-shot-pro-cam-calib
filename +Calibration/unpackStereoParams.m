function [x0] = unpackStereoParams(camK, camKc, prjK, prjKc, R, T)
% Unpack parameters for camera and projector calibration using lsqnonlin
% This function unpacks vectorized camera and projector parameters from
% matrices.
% See also: lsqnonlin, Calibration.packStereoParams

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

%% camera

% intrinsics
fx0_c = camK(1,1);
fy0_c = camK(2,2);
cx0_c = camK(1,3);
cy0_c = camK(2,3);

%% projector
% intrinsics
fx0_p = prjK(1,1);
fy0_p = prjK(2,2);
cx0_p = prjK(1,3);
cy0_p = prjK(2,3);
% sk0_p = prjK(1,2);

if(nargin == 2)
    x0 = [fx0_c, fy0_c, cx0_c, cy0_c, fx0_p, fy0_p, cx0_p, cy0_p]; % ignore skew
else
    if(nargin > 4)
        % rotation matrix to rotation vector
        % rvec = rotationMatrixToVector(R);
        
        rvec = cv.Rodrigues(R)';
        x0 = [fx0_c, fy0_c, cx0_c, cy0_c, camKc, fx0_p, fy0_p, cx0_p, cy0_p, prjKc, rvec, T']; % ignore skew
    else
        x0 = [fx0_c, fy0_c, cx0_c, cy0_c, camKc, fx0_p, fy0_p, cx0_p, cy0_p, prjKc]; % ignore skew
    end
end
    

end