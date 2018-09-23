function Nodes = decodeDebruijn(Nodes, prjW, prjH)
%% Decode the 2D coordinates of the node given its De Bruijn codeword
% See also: ImgProc.createDeBruijnSeq

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
imW = 1920;
imH = 1080;

if(prjW > imW)
    imW = prjW;
end

if(prjH > imH)
    imH = prjH;
end

[horiList, vertList, horiPos, vertPos] = ImgProc.createDeBruijnSeq(imW, imH);

[Nodes.horiCode] = deal(-1);
[Nodes.vertCode] = deal(-1);

[Nodes.activeRow] = deal(-1);
[Nodes.activeCol] = deal(-1);

for i = 1:numel(Nodes)
    node = Nodes(i);
    N = node.N;
    S = node.S;
    W = node.W;
    E = node.E;
    if(N < 0 || S < 0 || W < 0 || E < 0)
        continue;
    end
    
    Nodes(i).horiCode = [Nodes(W).vertColor, node.vertColor, Nodes(E).vertColor];
    Nodes(i).vertCode = [Nodes(N).horiColor, node.horiColor, Nodes(S).horiColor];
    
    Nodes(i).activeCol = findMatch(vertList, vertPos, Nodes(i).horiCode);
    Nodes(i).activeRow = findMatch(horiList, horiPos, Nodes(i).vertCode);
end
end

%% Local functions
function [pos] = findMatch(list, activePos, code)
pos = -1;
for i = 1:numel(list)-2
    if(code(1) == list(i) && code(2) == list(i+1) && code(3) == list(i+2))
        pos = activePos(i + 1);
        break;
    end
end
end