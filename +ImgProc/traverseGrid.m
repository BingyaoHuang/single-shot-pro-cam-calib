function Nodes = traverseGrid(Nodes, Edges, verbose)
%% Traverse the color grid graph to find the neighbors of each Node

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

if(nargin < 3)
    verbose = 0;
end

%% create the ajacency matrix and graph from Edges and Nodes
numNodes = numel(Nodes);
numEdges = numel(Edges); 

A = zeros(numNodes, numNodes); % for both horizontal and vertical
% hA = zeros(numNodes, numNodes); % for horizontal only

for i = 1:numEdges
    curEdge = Edges(i);
    
    if (numel(curEdge.nodes) == 2)% ignore end point and end edge
        edgeLength = numel(curEdge.PixelIdxList);
        A(curEdge.nodes(1), curEdge.nodes(2)) = edgeLength;
        A(curEdge.nodes(2), curEdge.nodes(1)) = edgeLength;
%         if(curEdge.isH)
%             hA(curEdge.nodes(1), curEdge.nodes(2)) = edgeLength;
%             hA(curEdge.nodes(2), curEdge.nodes(1)) = edgeLength;
%         end
    end
end

% vA = A - hA; % for vertical only

% create a graph object of ajacency matrix
% then we can use all the functions in http://www.mathworks.com/help/matlab/ref/graph-object.html
G = graph(A);

%% Traverse
[Nodes.neighbor] = deal([]);
[Nodes.N] = deal([-1]);
[Nodes.S] = deal([-1]);
[Nodes.W] = deal([-1]);
[Nodes.E] = deal([-1]);
[Nodes.error] = deal([]);

for i =1:numNodes
    myRow = Nodes(i).Centroid(2);
    myCol = Nodes(i).Centroid(1);
    neighborNodes = neighbors(G, i);
    Nodes(i).neighbor = neighborNodes;
    
    numNeighbors = numel(neighborNodes);
    count = 0;
    for j = 1:numNeighbors
        curNode = neighborNodes(j);
        row = Nodes(curNode).Centroid(2);
        col = Nodes(curNode).Centroid(1);
        
        rowDiff = abs(row - myRow);
        colDiff = abs(col - myCol);
        flag = rowDiff >= colDiff;
        
        if(myRow >= row && flag)
            if(Nodes(i).N < 0)
                Nodes(i).N = curNode;
                count= count+1;
            end
        elseif(myRow <= row && flag)
            if(Nodes(i).S < 0)
                Nodes(i).S = curNode;
                count= count+1;
            end
        elseif(myCol <= col)
            if(Nodes(i).E < 0)
                Nodes(i).E = curNode;
                count= count+1;
            end
        elseif(myCol >= col)
            if(Nodes(i).W < 0)
                Nodes(i).W = curNode;
                count= count+1;
            end
        end
    end
    
    if(count ~= numNeighbors)
        Nodes(i).error = 1;
    end
end


%% double check if we have unmatched neighbors
for i =1:numNodes
    N = Nodes(i).N;
    S = Nodes(i).S;
    W = Nodes(i).W;
    E = Nodes(i).E;
    
    if(N > 0)
        if(Nodes(N).S ~= i)
            Nodes(N).error = i;
            Nodes(i).error = N;
        end
    end
    
    if(S > 0)
        if(Nodes(S).N ~= i)
            Nodes(S).error = i;
            Nodes(i).error = S;
        end
    end
    
    if(W > 0)
        if(Nodes(W).E ~= i)
            Nodes(W).error = i;
            Nodes(i).error = W;
        end
    end
    
    if(E > 0)
        if(Nodes(E).W ~= i)
            Nodes(E).error = i;
            Nodes(i).error = E;
        end
    end
end


errorNodes = [Nodes.error];
if(verbose)
    disp(['Get ', num2str(nnz(errorNodes)), ' error nodes during traverse'])
end
end
