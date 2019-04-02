function [Edges, Nodes] = skeleToStruct(imNode, imEdge)
%% Extract Edges and Nodes structures from Node and Edge images
% See also: ImgProc.getMatchedNodes

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
%     imNode = logical(imSkele - imEdge);
imEdgeLabel = zeros(size(imNode),'uint16');

% create edges
Edges = regionprops(imEdge,'PixelIdxList', 'area', 'Orientation', 'solidity');

if(mean([Edges.Area]) < 5)
    warning('Color grid too dense, try moving the board away from the projector or moving the camera closer to the board')
end

% get rid of tortuous and long edges
Edges = Edges([Edges.Solidity] > 0.5);
Edges = Edges([Edges.Area] < mean([Edges.Area]) + 3*std([Edges.Area]));
[Edges(:).nodes] = deal([]);
[Edges(:).end] = deal([]);
[Edges(:).isH] = deal([]);  % a flag indicates it is horizontal

numEdges = numel(Edges);
for i = 1:numEdges
    imEdgeLabel(Edges(i).PixelIdxList) = i;
end

% create nodes
Nodes = regionprops(imNode,'PixelIdxList', 'area', 'centroid');
[Nodes(:).edges] = deal([]);
[Nodes(:).hEdges] = deal(-1);
[Nodes(:).vEdges] = deal(-1);

% assign edges for node i and also assign nodes for the corresponding
% edges
imSize = size(imEdge);
numNodes = numel(Nodes);
for i = 1:numNodes
    
    % get all pixel indices in this node
    nodePixelIdx = Nodes(i).PixelIdxList;
    
    % for each pixel index in this node
    for k=1:numel(nodePixelIdx)
        % convert pixel indices to subscripts
        [row, col] = ind2sub(imSize, nodePixelIdx(k));
        
        % search 3x3x3 neighborhood and try to find an edge
        [r, c] = meshgrid(row-1:row+1, col-1:col+1);
        neighbors = sub2ind(imSize, r(:), c(:));
        
        % remove the node pixels from the test for speed up
        neighbors = setdiff(neighbors,Nodes(i).PixelIdxList);
        
        % use the neighbors indices to get Edge ids from imEdgeLabel
        matchEdges = imEdgeLabel(neighbors);
        matchEdges = unique(matchEdges(matchEdges~=0));
        
        % for each edge we found in the neighborhood of this node
        for j = 1:numel(matchEdges)
            edgeId = matchEdges(j);
            
            % we pass the current node's index to Edge's nodes list
            Edges(edgeId).nodes = [Edges(edgeId).nodes, i];  % may have self loops
            Edges(edgeId).end = [Edges(edgeId).end, nodePixelIdx(k)];
            if(numel(Edges(edgeId).nodes) > 2)
                dbstop in ImgProc.skeleToStruct
            end
            
            % we also pass the current edge's index to Node's edges list
            Nodes(i).edges = unique([Nodes(i).edges, edgeId]);
            
            % assign each node vertical and horizontal Edge id
            orientation = Edges(edgeId).Orientation;
            if (orientation > -45 && orientation < 45)
                Edges(edgeId).isH = 1;
                hEdges = Nodes(i).hEdges;
                hEdges = unique([hEdges, edgeId]);
                Nodes(i).hEdges = hEdges(hEdges >0);
            else
                Edges(edgeId).isH = 0;
                vEdges = Nodes(i).vEdges;
                vEdges = unique([vEdges, edgeId]);
                Nodes(i).vEdges = vEdges(vEdges >0);
            end
        end
    end
end

end