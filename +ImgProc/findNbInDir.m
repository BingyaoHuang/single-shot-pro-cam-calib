function nextId = findNbInDir(Nodes, id, direction, iIn)
%% Find all nodes along a given direction in the graph
% this function returns a list of Nodes id on the same stripe,
% the search direction is specified by user input

%% License
% ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
% Copyright (c) 2018 Bingyao Huang
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% If you publish results obtained using this software, please cite our paper.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

% Enable for debugging..
%     persistent i;
%
%     if nargin == 4
%       i = iIn;
%     else
%       i = i+1;
%     end
%
%     display([direction ' ' num2str(i)]);
node = Nodes(id);
switch direction
    case 'N'
        nextId = node.N;
    case 'S'
        nextId = node.S;
    case 'E'
        nextId = node.E;
    case 'W'
        nextId = node.W;
    otherwise
        error('wrong direction')
end

if(id == nextId)
    nextId = [];
    disp(['node.' direction ' of ' num2str(id) ' is pointing to itself..returning']);
    return;
end

if(nextId < 0)
    nextId = [];
    return;
end

nextId = [nextId, ImgProc.findNbInDir(Nodes, nextId, direction)];
end