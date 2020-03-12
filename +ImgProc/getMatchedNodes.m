function [camPoints, prjPoints, Nodes, Edges] = getMatchedNodes(whiteLight, colorGrid, camCorners, prjW, prjH, verbose)
%% Get matched Nodes on both camera and projector image.
% Tthe outputs are matched camera and projector 2D point pairs.

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


%% Step 1. Extract grid mask on the white board
[imBWGrid, imColorGridMasked] = ImgProc.segColorGrid(whiteLight, colorGrid, camCorners, verbose);
imSize = size(imBWGrid);

%% Step 2. Skeletonize
imSkele = ImgProc.bw2skele(imBWGrid);

if (verbose)
    figure;
    imshowpair(imBWGrid, imSkele, 'Montage');
    title('Grid mask and skeletonized grid mask');
end

%% Step 3. Extract edge, node endpoint and # neighbors image
[imEdge, imNode, ~, ~] = extractStructs(imSkele);

%% Step 4. Pre-extract Nodes and Edges structures for cleaning
[Edges, Nodes] = ImgProc.skeleToStruct(imNode, imEdge);

% get rid unnormally long edges
    hEdge = Edges(logical([Edges.isH]));
    hEdge = hEdge([hEdge.Area] < mean([hEdge.Area]) + 2*std([hEdge.Area]));

    vEdge = Edges(~logical([Edges.isH]));
    vEdge = vEdge([vEdge.Area] < mean([vEdge.Area]) + 2*std([vEdge.Area]));

    Edges = [hEdge; vEdge];

% find isolated edges (area == 1) and remove those edges from imEdge
% and add to imNodes
labels = [Edges.Area] == 1;
isoEdges = Edges(labels);
vecNumNodes = cellfun(@numel, {isoEdges.nodes});
leafEdgeIdx = vecNumNodes == 1;
nodeEdgeIdx = vecNumNodes == 2;

imEdge([isoEdges(nodeEdgeIdx).PixelIdxList]) = 0;
imNode([isoEdges(nodeEdgeIdx).PixelIdxList]) = 1;


% Extract Nodes and Edges from cleaned imEdge and imNode
[Edges, Nodes] = ImgProc.skeleToStruct(imNode, imEdge);
numNodes = numel(Nodes);
numEdges = numel(Edges);

% imMix = imEdge + imNode*3;
% figure;
% title('imMix - Edges and Nodes');
% imagesc(imMix);

%% step 5. create labeled edge and node images for debugging

if (verbose)   
%     imEdgeLabel = zeros(size(imNode), 'uint16');
%     imNodeLabel = zeros(size(imNode), 'uint16');
%     
%     for i = 1:numEdges
%         imEdgeLabel(Edges(i).PixelIdxList) = i;
%     end
%     
%     for i = 1:numNodes
%         imNodeLabel(Nodes(i).PixelIdxList) = i;
%     end
%     
%     subplot(2,2,3);drawnow
%     title('imEdgeLabel - Labeled Edges');
%     imagesc(imEdgeLabel);
%     caxis([0 0.1]);
%     myMap = [0, 0, 0; 1, 1, 1];
%     colormap(myMap);
%     
%     subplot(2,2,4);drawnow
%     title('imNodeLabel - Labeled Nodes');
%     imagesc(imNodeLabel);
%     caxis([0 0.1]);
%     myMap = [0, 0, 0; 1, 1, 1];
%     colormap(myMap);   
end

%% step 6. create the ajacency matrix and graph from Edges and Nodes
A = zeros(numNodes, numNodes);
    
    for i = 1:numEdges
    curEdge = Edges(i);
    
    if (numel(curEdge.nodes) == 2)% ignore end point and end edge
        edgeLength = numel(curEdge.PixelIdxList);
        A(curEdge.nodes(1), curEdge.nodes(2)) = edgeLength;
        A(curEdge.nodes(2), curEdge.nodes(1)) = edgeLength;
    end
    
end

%% step 6. create the ajacency matrix and graph from Edges and Nodes

%% step 7. traverse the grid to assign four neighbors to Nodes
Nodes = ImgProc.traverseGrid(Nodes, Edges);

%% step 8. get horizontal and vertical edges
horiIdx = [Edges.isH] == 1;
vertIdx = ~horiIdx;

horiEdges = Edges(horiIdx);
vertEdges = Edges(vertIdx);

imHoriEdge = false(imSize);
imHoriEdge(vertcat(horiEdges.PixelIdxList)) = 1;
% figure;
% imshow(imHoriEdge);
% title('horizontal edges');

imVertEdge = false(imSize);
imVertEdge(vertcat(vertEdges.PixelIdxList)) = 1;
% figure;
% imshow(imVertEdge);
% title('vertical edges');

%% step 9. read color grid and convert RGB to labels
[imAllLabel, imHoriLabel, imVertLabel] = ImgProc.colorToLabel(imColorGridMasked, imNode, imHoriEdge, imVertEdge, verbose);
% figure;
% imshowpair(imColorGrid, imAllLabel,'Montage');
% title('color grid and all labels');

%% step 10. assign each node horizontal and vertical color labels
[Nodes(:).horiColor] = deal(-1);
[Nodes(:).vertColor] = deal(-1);

% imNodeWide = imdilate(imNode, strel('disk', 1));
imNodeWide = imdilate(imNode, strel('square', 3));

imHoriWide = imdilate(imHoriEdge, strel('disk', 1));
imHoriWide = logical(imHoriWide .* (~imNodeWide));

imVertWide = imdilate(imVertEdge, strel('disk', 1));
imVertWide = logical(imVertWide .* (~imNodeWide));

% labeled images (faster than imreconstruct)
imHoriWideL = bwlabel(imHoriWide);
imVertWideL = bwlabel(imVertWide);

for i = 1:numel(Nodes)
    hEdges = [Nodes(i).hEdges];
    hEdges = hEdges(hEdges > 0);
    curHoriEdgePixIdx = vertcat(Edges(hEdges).PixelIdxList); 
    
%     imMarker = false(imSize);
%     imMarker(curHoriEdgePixIdx) = 1;
%     imWideRecon = imreconstruct(imMarker, imHoriWide, 4);
%     curHoriEdgeLabels = imHoriLabel(imWideRecon > 0);
    
    hLabels = imHoriWideL(curHoriEdgePixIdx);
    idx = ismember(imHoriWideL, hLabels(hLabels>0));
    curHoriEdgeLabels = imHoriLabel(idx); 
    
    % vertical
    vEdges = [Nodes(i).vEdges];
    vEdges = vEdges(vEdges > 0);
    curVertEdgePixIdx = vertcat(Edges(vEdges).PixelIdxList);
    
%     imMarker = false(imSize);
%     imMarker(curVertEdgePixIdx) = 1;
%     imWideRecon = imreconstruct(imMarker, imVertWide, 4);
%     curVertEdgeLabels = imVertLabel(imWideRecon > 0);
    
    vLabels = imVertWideL(curVertEdgePixIdx);
    idx = ismember(imVertWideL, vLabels(vLabels>0));
    curVertEdgeLabels = imVertLabel(idx); 
    
    % vote majority
    Nodes(i).horiColor = mode(curHoriEdgeLabels);
    Nodes(i).vertColor = mode(curVertEdgeLabels);
end

%% step 11. correct each Node's label by majority voting on each stripe

% NodesBackup = Nodes; % keep a copy of old Nodes for debug
Nodes = ImgProc.correctColors(Nodes);

%% step 12. decode the De Bruijn sequence

Nodes = ImgProc.decodeDebruijn(Nodes, prjW, prjH);
[camPoints, prjPoints] = extractNodesPosition(Nodes);

% fill missing coordinates by majority vote on the same stripe
for i = 1:numel(Nodes)   
    % find all nodes on the same horizontal line
    Nnb = ImgProc.findNbInDir(Nodes, i, 'N', 0);
    Snb = ImgProc.findNbInDir(Nodes, i, 'S', 0);
    Enb = ImgProc.findNbInDir(Nodes, i, 'E', 0);
    Wnb = ImgProc.findNbInDir(Nodes, i, 'W', 0);

    % activeRow
    hNodeIdx = [flip(Wnb), i, Enb];
    activeRow = [Nodes(hNodeIdx).activeRow];
    if(nnz(activeRow > 0))
        [Nodes(hNodeIdx).activeRow] = deal(mode(activeRow(activeRow > 0)));
    end

    % activeCol
    vNodeIdx = [flip(Nnb), i, Snb];
    activeCol = [Nodes(vNodeIdx).activeCol];
    if(nnz(activeCol > 0))
        [Nodes(vNodeIdx).activeCol] = deal(mode(activeCol(activeCol > 0)));
    end
end

% fill missing node prj coordinates using linear interpolation
for i = 1:numel(Nodes)
    % find all nodes on the same horizontal line
    Nnb = ImgProc.findNbInDir(Nodes, i, 'N',0);
    Snb = ImgProc.findNbInDir(Nodes, i, 'S',0);
    Enb = ImgProc.findNbInDir(Nodes, i, 'E',0);
    Wnb = ImgProc.findNbInDir(Nodes, i, 'W',0);

    hNodeIdx = [flip(Wnb), i, Enb];
%     hStripes = [Nodes(hNodeIdx).hEdges];

    vNodeIdx = [flip(Nnb), i, Snb];
%     vStripes = [Nodes(vNodeIdx).vEdges];

    % fill -1 with linear interpolation
    
    % horizontal stripes
    vecCol = [Nodes(hNodeIdx).activeCol];
    vecCol = num2cell(fillmissing(vecCol, 'linear', 'MissingLocations', vecCol<0));
    [Nodes(hNodeIdx).activeCol] = vecCol{:};
    
    vecRow = [Nodes(hNodeIdx).activeRow];
    vecRow = num2cell(fillmissing(vecRow, 'linear', 'MissingLocations', vecRow<0));
    [Nodes(hNodeIdx).activeRow] = vecRow{:};
    
    % vertical stripes
    vecRow = [Nodes(vNodeIdx).activeRow];
    vecRow = num2cell(fillmissing(vecRow, 'linear', 'MissingLocations', vecRow<0));
    [Nodes(vNodeIdx).activeRow] = vecRow{:};   
    
    vecCol = [Nodes(vNodeIdx).activeCol];
    vecCol = num2cell(fillmissing(vecCol, 'linear', 'MissingLocations', vecCol<0));
    [Nodes(vNodeIdx).activeCol] = vecCol{:};
end

%% Local functions
function [camPoints, prjPoints, validNodeIdx] = extractNodesPosition(Nodes)

validNodes = [Nodes.activeCol] > 0 & [Nodes.activeRow] > 0;
validNodeIdx = find(validNodes);

camPoints = vertcat(Nodes(validNodes).Centroid);
prjPoints = [[Nodes(validNodes).activeCol]', [Nodes(validNodes).activeRow]'];
end

% convert skeleton image to node, edge endpoints images
function [imEdges, imNodes, imEndPoints, imNeighbor] = extractStructs(imSkele)
imNeighbor = imfilter(double(imSkele), [1, 1, 1; 1, 0, 1; 1, 1, 1]);
imNeighbor = imNeighbor .* imSkele;

imEdges = imNeighbor == 2;
imEndPoints = imNeighbor == 1;
imNodes = logical(imSkele - imEdges - imEndPoints);
end

% also find edge's 3d coord
