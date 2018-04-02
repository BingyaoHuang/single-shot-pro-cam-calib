function [rmsAlignErr] = alignmentErrorSynthetic(estPts3d, gtpts3d, verbose, figName)
%% Compute 3d alignment error for synthetic data.
% 3d alignment error is the mean distance between reconstructed 3d points 
% and the ground truth 3d points.

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

% alignment residual
alignRes = estPts3d - gtpts3d;

% squared alignment error
squaredAlignErr = alignRes.^2;

% sum of squared alignment error
sse = sum(squaredAlignErr, 2);

% visualize
if(verbose)
    figure('Name', [figName, ' Alignment Error']);
    hold on;
    title([figName, ' Alignment Error']);
    
    % boxplot alignment error in X, Y, Z direction
    subplot(1,2,1);
    boxplot(abs(alignRes), 'Notch','on');
    title('X,Y,Z direction absolute alignment error (mm)');
   
    % boxplot total alignment error
    subplot(1,2,2);
    boxplot(sqrt(sse), 'Notch','on')
    title('Total absolute alignment error (mm)');
end

% root mean squared total alignment error (rmse)
rmsAlignErr = sqrt(mean(sse));

end