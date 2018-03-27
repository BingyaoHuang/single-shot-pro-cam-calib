function plotPlanes(planes, figureName)
%% Plot 3d planes given plane structure
% See also: Calibration.planeStructFromPoints

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

nPlanes = length(planes);

% each plane has a different color
planeColors = hsv(nPlanes);

if(nargin > 1)
    if(isempty(figureName))
        error('Figure name is empty');
    end
    
    if(~isa(figureName, 'char'))
        error('Figure name should be a string');
    end
    
    % find figure by name
    figHandle = findobj( 'Type', 'Figure', 'Name', figureName );
    
    if(length(figHandle) > 1)
        warning(['More than 1 figures named', figureName, 'are found, use the 1st figure']);
    end
    
    if(length(figHandle) < 1)
        warning(['No figure named', figureName, 'is found, create one instead']);
        figHandle = figure('name', figureName);
    end
    
    figure(figHandle(1));
end

% plot
for i = 1:nPlanes
    corners = planes(i).corners;
    p = patch(corners(:,1), corners(:,2), corners(:,3), planeColors(i,:));
    p.FaceAlpha = 0.3;
end

end