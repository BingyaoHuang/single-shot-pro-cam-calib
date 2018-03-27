function h = plotErrors(results, vecNoise, algIdx, metricRange, figName)
%% Plot synthetic calibration results, such as camera/projector intrinsics, extrinsics and reprojection errors.

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

% generate figures for latex
LATEX = false;

results = results(1:length(vecNoise), :);

% extract # of sigmas and trials
[numSigma, numTrials] = size(results);

%% Concatenate all trials to cell arrays
% extract metric names
metricNames = fieldnames(results{1, 1});
metricNames = metricNames(2:end);

% averaged results from multiple trials
concatTestResults = cell(1, numSigma);

% calculate means from all trials
for i = 1:numSigma
    allTirals = struct2cell(results{i, 1})';
    
    % keep only the seleted algortihms
    allTirals = allTirals(algIdx,:);
    
    % convert struct to cell array
    for trial = 2:numTrials % start from the 2nd trial
        curTrial = struct2cell(results{i, trial})';
        
        % keep only the seleted algortihms
        curTrial = curTrial(algIdx,:);
        
        % concatenate each metric from all trials
        for alg = 1:size(allTirals, 1)
            for metric = 2:length(metricNames)+1
                allTirals{alg, metric}  = [allTirals{alg, metric}, curTrial{alg, metric}];
            end
        end
    end
    
    concatTestResults{i} = allTirals;
end

%%
metricNames{1} = ['\fontsize{13}{0}\bf\selectfont Reprojection error', ' (pixel)'];
metricNames{2} = ['\fontsize{13}{0}\bf\selectfont 3D alignment error', ' (mm)'];

metricNames{3} = ['\fontsize{13}{0}\bf\selectfont Rotation error', ' (degree)'];
metricNames{4} = ['\fontsize{13}{0}\bf\selectfont Translation error', ' (mm)'];

% projector intrinsics error
metricNames{5} = '\fontsize{13}{0}\boldmath$f^{\prime}_{x}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{6} = '\fontsize{13}{0}\boldmath$f^{\prime}_{y}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{7} = '\fontsize{13}{0}\boldmath$c^{\prime}_{x}$ \fontsize{13}{0}\bf\selectfont error (pixel)';
metricNames{8} = '\fontsize{13}{0}\boldmath$c^{\prime}_{y}$ \fontsize{13}{0}\bf\selectfont error (pixel)';

% projector distortion factor error
metricNames{9} =  '\fontsize{13}{0}\boldmath$k^{\prime}_{1}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{10} = '\fontsize{13}{0}\boldmath$k^{\prime}_{2}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{11} = '\fontsize{13}{0}\boldmath$p^{\prime}_{1}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{12} = '\fontsize{13}{0}\boldmath$p^{\prime}_{2}$ \fontsize{13}{0}\bf\selectfont error (\%)';

% camera intrinsics error
metricNames{13} = '\fontsize{13}{0}\boldmath$f_{x}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{14} = '\fontsize{13}{0}\boldmath$f_{y}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{15} = '\fontsize{13}{0}\boldmath$c_{x}$ \fontsize{13}{0}\bf\selectfont error (pixel)';
metricNames{16} = '\fontsize{13}{0}\boldmath$c_{y}$ \fontsize{13}{0}\bf\selectfont error (pixel)';

% camera distortion factor error
metricNames{17} = '\fontsize{13}{0}\boldmath$k_{1}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{18} = '\fontsize{13}{0}\boldmath$k_{2}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{19} = '\fontsize{13}{0}\boldmath$p_{1}$ \fontsize{13}{0}\bf\selectfont error (\%)';
metricNames{20} = '\fontsize{13}{0}\boldmath$p_{2}$ \fontsize{13}{0}\bf\selectfont error (\%)';

%% plot
if(LATEX)
    if(metricRange(2) - metricRange(1) == 3)
        h = figure('Name', figName, 'units','normalized','outerposition',[0 0 2 0.7]);
    else
        h = figure('Name', figName, 'units','normalized','outerposition',[0 -0.5 2 1.4]);
    end
else
    h = figure('Name', figName, 'units','normalized','outerposition',[0 0 1 1]);
end

title(figName);

% extract algorithm names
algNames = concatTestResults{1}(:,1);

% set marker style for different algorithms
markerStyles = {'s', '*', 'o', '+', 'x','d'};
lineStyles = {'-.', ':', '-.', '-',':'};


colors = lines(4);
if(LATEX)
    % better color in grayscale
    colors = [
        0    0.4470    0.7410;
        0.8020    0.2804    0.1039;
        0.4660    0.6740    0.1880;
        0.6350    0.0780    0.1840;
        ];
    
end

% decide subplot layout
nPlots = metricRange(2) - metricRange(1) + 1;

if(nPlots <= 4)
    subplotDims = [1, nPlots];
elseif(nPlots > 4 && nPlots <=8)
    subplotDims = [2, ceil(nPlots/2)];
else
    subplotDims = [4, ceil(nPlots/4)];
end

% subplotDims = [ceil(nPlots/2), 2];

subPlotsHandle = {};

if(LATEX)
    width = 5;
else
    width = 3;
end

% plot each error metric in seperate subplots
for metric = metricRange(1):metricRange(2)
    subPlotsHandle{metric} = subplot(subplotDims(1), subplotDims(2), metric - metricRange(1) + 1);
    
    % for each algorrithm
    for alg = 1:length(algNames)
        curMedian = [];
        
        % for each noise level
        for i = 1:numSigma
            curMedian = [curMedian, median(concatTestResults{i}{alg, metric+1})];
        end
        plot(vecNoise, curMedian, 'LineWidth', width, 'LineStyle', lineStyles{alg},  'Color', colors(alg,:));
        
        hold on
    end
    hold off;
    
    if(metric == metricRange(1))
        if(LATEX)
            legend({'Moreno & Taubin [17]', 'Global homography', 'Proposed w/o BA', 'Proposed'}, 'Location', 'northwest', 'FontSize', 29);
        else
            legend({'Moreno & Taubin [17]', 'Global homography', 'Proposed w/o BA', 'Proposed'}, 'Location', 'northwest');
        end
    end
    
    % figure properties
    if(LATEX)
        set(gca, 'FontSize', 30);
    end
    grid on
    xlabel('Noise Level (\sigma)', 'FontWeight','bold');
    title(metricNames{metric}, 'interpreter', 'latex');
end

%% Shrink vertical and horizontal spaces between subplots
if(LATEX)
    if(nPlots <= 4)
        drawnow
        % shrink vertical spacing between subplots
        for metric = metricRange(1):metricRange(2)
            sh = subPlotsHandle{metric};
            sh.Position(1) = sh.Position(1)-0.02*(metric-metricRange(1));
        end
    else % when more than 1 row
        
        firstRowEnd = floor(( metricRange(1) + metricRange(2) ) / 2);
        for metric = metricRange(1):firstRowEnd
            sh = subPlotsHandle{metric};
            sh.Position(1) = sh.Position(1)-0.02*(metric-metricRange(1));
            
            % also increase 1st row's spacing to the top
            sh.Position(2) = sh.Position(2)+0.01;
        end
        
        
        for metric = firstRowEnd+1:metricRange(2)
            drawnow
            sh = subPlotsHandle{metric};
            sh.Position(1) = sh.Position(1)-0.02*(metric-firstRowEnd-1);
            
            % also increase 2nd row's spacing to the prev row
            sh.Position(2) = sh.Position(2)-0.025;
        end
    end
end
end