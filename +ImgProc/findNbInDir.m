%% Find all nodes along a given direction in the graph
% this function returns a list of Nodes id on the same stripe,
% the search direction is specified by user input
function nextId = findNbInDir(Nodes, id, direction, iIn)
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