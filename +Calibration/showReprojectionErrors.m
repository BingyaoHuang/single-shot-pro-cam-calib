function h = showReprojectionErrors(stereoParams, calibInfo)
%% Similar to MATLAB's showReprojectionErrors

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

%% cam per view errors
setNames = arrayfun(@(x) ['Set', num2str(x,'%02.f')], calibInfo.sets, 'UniformOutput', false);
h = figure(name='Reprojection errors', Units='normalized', Position=[0.3 0.3 0.5 0.3]);
% h = tiledlayout(1,3, TileSpacing='compact');

% % cam per view errors
% nexttile
% bar(categorical(setNames), cell2mat(stereoParams.camPerViewReprojErr));
% title('Camera per-view errors');
% 
% % prj per view errors
% nexttile
% bar(categorical(setNames), cell2mat(stereoParams.prjPerViewReprojErr));
% title('Projector per-view errors');
% 
% % stereo per view errors
% nexttile
% bar(categorical(setNames), cell2mat(stereoParams.stereoPerViewReprojErr));
% title('Stereo per-view errors');

bar(categorical(setNames), cell2mat([stereoParams.camPerViewReprojErr; stereoParams.prjPerViewReprojErr; stereoParams.stereoPerViewReprojErr]));
title('Per-view reprojection errors');
legend({'Camera', 'Projector', 'Stereo'});

end