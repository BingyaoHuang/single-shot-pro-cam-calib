function [camK, camKc, prjK, prjKc, R, T] = packStereoParams(x)
%% Packs vectorized stereo parameters to matrices.

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
% projector intrinsics
fx_c = x(1);
fy_c = x(2);
cx_c = x(3);
cy_c = x(4);
sk_c = 0;

camK = [fx_c, sk_c, cx_c;
    0   , fy_c, cy_c;
    0   , 0   , 1  ];

% camera distortion factors (2 radio, 2 tangential)
k1_c = x(5);
k2_c = x(6);
p1_c = x(7);
p2_c = x(8);

camKc = [k1_c, k2_c, p1_c, p2_c];

%% projector
% projector intrinsics
fx_p = x(9);
fy_p = x(10);
cx_p = x(11);
cy_p = x(12);
sk_p = 0;

prjK = [fx_p, sk_p, cx_p;
    0   , fy_p, cy_p;
    0   , 0   , 1  ];

% projector distortion factors (2 radio, 2 tangential)
k1_p = x(13);
k2_p = x(14);
p1_p = x(15);
p2_p = x(16);

prjKc = [k1_p, k2_p, p1_p, p2_p];


% if also need R,T
if(nargout > 4)
    % R = rotationVectorToMatrix(x(10:12));
    R = cv.Rodrigues(x(17:19));
    
    % translation vector
    T = [x(20); x(21); x(22)];
end

end