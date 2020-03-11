function NodesOut = correctColors(Nodes, verbose)
%% Correct each Node's color by majority voting on the same stripe.

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
if(nargin < 2)
    verbose = false;
end

if(verbose)
    disp('Correcting color labels in each horizontal and vertical stripe')
end

NodesOut = Nodes;
numNodes = numel(Nodes);
[Nodes.hCorrected] = deal(0);
[Nodes.vCorrected] = deal(0);
for i = 1:numNodes
    %% find all nodes on the same horizontal line
    if(~Nodes(i).hCorrected)
        eastNeighbors = ImgProc.findNbInDir(Nodes, i, 'E',0);
        westNeighbors = ImgProc.findNbInDir(Nodes, i, 'W',0);
        horiNodeList = [i, eastNeighbors, westNeighbors];
        
        horiColors = [Nodes(horiNodeList).horiColor];
        majorityColor = mode(horiColors);
        [NodesOut(horiNodeList).horiColor] = deal(majorityColor);
        [Nodes(horiNodeList).hCorrected] = deal(1);
    end
    
    %% find all nodes on the same vertical line
    if(~Nodes(i).vCorrected)
        northNeighbors = ImgProc.findNbInDir(Nodes, i, 'N',0);
        southNeighbors = ImgProc.findNbInDir(Nodes, i, 'S',0);
        vertNodeList = [i, northNeighbors, southNeighbors];
        
        vertColors = [Nodes(vertNodeList).vertColor];
        majorityColor = mode(vertColors);
        [NodesOut(vertNodeList).vertColor] = deal(majorityColor);
        [Nodes(vertNodeList).vCorrected] = deal(1);
    end
end

if(verbose)
    numHoriChange = nnz([NodesOut.horiColor] - [Nodes.horiColor]);
    numVertChange = nnz([NodesOut.vertColor] - [Nodes.vertColor]);

    disp(['horizontal corrections:  = ', num2str(numHoriChange)])
    disp(['vertical corrections:  = ', num2str(numVertChange)])
end
end
