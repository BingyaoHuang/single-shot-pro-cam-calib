% Projector-Camera Stereo calibration parameters:

% Intrinsic parameters of camera:
fc_left = [ 622.595861 623.914383 ]; % Focal Length
cc_left = [ 320.065680 252.110220 ]; % Principal point
alpha_c_left = [ 0.000000 ]; % Skew
kc_left = [ 0.102750 -0.503813 0.003233 -0.000139 0.000000 ]; % Distortion

% Intrinsic parameters of projector:
fc_right = [ 1336.690469 1333.550479 ]; % Focal Length
cc_right = [ 362.521491 573.484100 ]; % Principal point
alpha_c_right = [ 0.000000 ]; % Skew
kc_right = [ -0.088902 0.296069 0.000862 -0.002839 0.000000 ]; % Distortion

% Extrinsic parameters (position of projector wrt camera):
om = [ -0.088537 0.368356 -0.027127 ]; % Rotation vector
T = [ -366.653873 -414.078641 1227.794209 ]; % Translation vector
